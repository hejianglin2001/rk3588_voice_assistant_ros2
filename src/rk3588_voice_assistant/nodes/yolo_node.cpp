// yolo_node — YOLO NPU 目标检测 Action 服务端 (MIPI 摄像头实时输入)
#include "rclcpp/rclcpp.hpp"
#include "rk3588_voice_assistant/yolo_node.hpp"
#include "yolo/yolo_engine.hpp"
#include "yolo/camera_capture.h"
#include "yolo/rga_convert.h"
// stb 单头文件 — 直接写 JPG，零外部依赖（替代手写 BMP，本地双击即看）
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "yolo/stb_image_write.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <thread>
#include <unordered_map>
#include <vector>
#include <sys/stat.h>

// 中文→COCO 类名映射（常用目标，匹配时归一化）
// ponytail: 只覆盖演示常用词，缺的走英文原名直传
static const char* zhToEn(const std::string& s) {
    static const std::unordered_map<std::string, std::string> m = {
        {"苹果","apple"},{"香蕉","banana"},{"橘子","orange"},{"橙子","orange"},
        {"人","person"},{"猫","cat"},{"狗","dog"},{"汽车","car"},{"车","car"},
        {"杯子","cup"},{"瓶子","bottle"},{"手机","cell phone"},{"笔记本电脑","laptop"},
        {"电脑","laptop"},{"书","book"},{"椅子","chair"},{"桌子","dining table"},
        {"电视","tv"},{"蛋糕","cake"},{"披萨","pizza"},{"三明治","sandwich"},
        {"西兰花","broccoli"},{"胡萝卜","carrot"},{"热狗","hot dog"},
    };
    auto it = m.find(s);
    return it != m.end() ? it->second.c_str() : s.c_str();
}

// ---- 板端无屏：检测图落盘（画框 + 置信度 + 24-bit BMP，零依赖）----
// ponytail: 只画数字置信度（0-9 .），类别靠文件名/日志，避免塞整套 ASCII 字体
static const uint8_t kFont[12][7] = {
    {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},  // 0
    {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},  // 1
    {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F},  // 2
    {0x1F,0x02,0x04,0x02,0x01,0x11,0x0E},  // 3
    {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},  // 4
    {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},  // 5
    {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E},  // 6
    {0x1F,0x01,0x02,0x04,0x08,0x08,0x08},  // 7
    {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},  // 8
    {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C},  // 9
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C},  // .
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00},  // space
};
static const uint8_t* glyph(char c) {
    if (c >= '0' && c <= '9') return kFont[c - '0'];
    if (c == '.') return kFont[10];
    return kFont[11];
}
static inline void setPx(uint8_t* rgb, int W, int H, int x, int y,
                         uint8_t r, uint8_t g, uint8_t b) {
    if (x < 0 || y < 0 || x >= W || y >= H) return;
    uint8_t* p = rgb + (y * W + x) * 3;
    p[0] = r; p[1] = g; p[2] = b;
}
static void drawBox(uint8_t* rgb, int W, int H, int x, int y, int w, int h) {
    int x0 = std::max(0, x), y0 = std::max(0, y);
    int x1 = std::min(W - 1, x + w - 1), y1 = std::min(H - 1, y + h - 1);
    for (int t = 0; t < 3; ++t) {  // 3px 绿色边框
        int lx = x0 + t, ly = y0 + t, rx = x1 - t, ry = y1 - t;
        if (lx > x1 || ly > y1) break;
        for (int px = lx; px <= rx; ++px) { setPx(rgb, W, H, px, ly, 0, 255, 0); setPx(rgb, W, H, px, ry, 0, 255, 0); }
        for (int py = ly; py <= ry; ++py) { setPx(rgb, W, H, lx, py, 0, 255, 0); setPx(rgb, W, H, rx, py, 0, 255, 0); }
    }
}
static void drawText(uint8_t* rgb, int W, int H, int x0, int y0,
                     const std::string& s, int scale) {
    int cx = x0;
    for (char c : s) {
        const uint8_t* g = glyph(c);
        for (int r = 0; r < 7; ++r)
            for (int col = 0; col < 5; ++col)
                if (g[r] & (0x10 >> col))
                    for (int dy = 0; dy < scale; ++dy)
                        for (int dx = 0; dx < scale; ++dx)
                            setPx(rgb, W, H, cx + col * scale + dx, y0 + r * scale + dy, 255, 255, 255);
        cx += 6 * scale;
    }
}
YoloNode::YoloNode(const rclcpp::NodeOptions& options)
    : LifecycleNode("yolo_node", options) {
    declare_parameter("model_path", "");
    declare_parameter("conf_threshold", 0.25);
    declare_parameter("camera_device", "/dev/video11");
    declare_parameter("camera_width", 1920);
    declare_parameter("camera_height", 1080);
    declare_parameter("save_dir", "/home/topeet/code/rk3588_voice_assistant_ros2/yolo_out");
}

YoloNode::CallbackReturn YoloNode::on_configure(const rclcpp_lifecycle::State&) {
    auto path = get_parameter("model_path").as_string();
    if (path.empty()) {
        RCLCPP_WARN(get_logger(), "model_path 未设置"); return CallbackReturn::FAILURE;
    }

    YoloEngine::Config cfg;
    cfg.model_path = path;
    cfg.conf_threshold = static_cast<float>(get_parameter("conf_threshold").as_double());
    cfg.iou_threshold = 0.45f;

    yolo_ = std::make_unique<YoloEngine>();
    if (!yolo_->Init(cfg)) {
        RCLCPP_ERROR(get_logger(), "YOLO 模型加载失败: %s", path.c_str()); return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "YOLO 模型就绪: %s", path.c_str());

    save_dir_ = get_parameter("save_dir").as_string();
    mkdir(save_dir_.c_str(), 0755);  // 已存在会 EEXIST，忽略
    RCLCPP_INFO(get_logger(), "检测图保存目录: %s", save_dir_.c_str());

    // 打开摄像头
    cam_w_ = static_cast<int>(get_parameter("camera_width").as_int());
    cam_h_ = static_cast<int>(get_parameter("camera_height").as_int());
    auto cam_dev = get_parameter("camera_device").as_string();

    camera_ = std::make_unique<CameraCapture>();
    if (!camera_->Open(cam_dev, cam_w_, cam_h_) || !camera_->Start()) {
        RCLCPP_WARN(get_logger(), "摄像头 %s 不可用，使用测试图", cam_dev.c_str());
        camera_.reset();
    } else {
        RCLCPP_INFO(get_logger(), "摄像头就绪: %s %dx%d UYVY", cam_dev.c_str(), cam_w_, cam_h_);
    }

    action_server_ = rclcpp_action::create_server<YoloDetect>(
        this->get_node_base_interface(), this->get_node_clock_interface(),
        this->get_node_logging_interface(), this->get_node_waitables_interface(),
        "/yolo_detect",
        std::bind(&YoloNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&YoloNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&YoloNode::execute, this, std::placeholders::_1));

    // 视觉上下文广播：检测期间持续发布画面物体，llm_node 保存供视觉问答
    rclcpp::QoS ctx_qos(10);
    ctx_qos.reliable().durability_volatile();
    ctx_pub_ = create_publisher<rk3588_voice_assistant_interfaces::msg::VisionContext>(
        "/vision_context", ctx_qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/yolo_markers", ctx_qos);

    return CallbackReturn::SUCCESS;
}

YoloNode::CallbackReturn YoloNode::on_activate(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(get_logger(), "yolo_node activated"); return CallbackReturn::SUCCESS;
}
YoloNode::CallbackReturn YoloNode::on_deactivate(const rclcpp_lifecycle::State&) {
    return CallbackReturn::SUCCESS;
}
YoloNode::CallbackReturn YoloNode::on_cleanup(const rclcpp_lifecycle::State&) {
    action_server_.reset();
    if (camera_) { camera_->Close(); camera_.reset(); }
    yolo_.reset();
    return CallbackReturn::SUCCESS;
}
YoloNode::CallbackReturn YoloNode::on_shutdown(const rclcpp_lifecycle::State&) {
    return on_cleanup(rclcpp_lifecycle::State());
}

rclcpp_action::GoalResponse YoloNode::handle_goal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const YoloDetect::Goal> goal) {
    if (goal->target_class.empty()) return rclcpp_action::GoalResponse::REJECT;
    RCLCPP_INFO(get_logger(), "检测请求: target=%s duration=%ds",
                goal->target_class.c_str(), goal->duration_sec);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse YoloNode::handle_cancel(const std::shared_ptr<GoalHandle>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

void YoloNode::execute(const std::shared_ptr<GoalHandle> gh) {
    auto goal = gh->get_goal();
    auto result = std::make_shared<YoloDetect::Result>();
    result->detected = false; result->best_confidence = 0.0f;

    auto deadline = now() + rclcpp::Duration::from_seconds(goal->duration_sec);
    float conf_th = static_cast<float>(get_parameter("conf_threshold").as_double());
    bool has_camera = (camera_ && camera_->IsOpen());
    const char* en_target = zhToEn(goal->target_class);  // 中文→英文归一化
    bool saved = false;  // 每个 goal 只落盘最高置信度那一帧，避免刷满磁盘

    while (rclcpp::ok() && now() < deadline) {
        if (gh->is_canceling()) { gh->canceled(result); return; }

        uint8_t* rgb = nullptr;  // 画框落盘需可写
        uint8_t* rga_buf = nullptr;

        if (has_camera) {
            auto frame = camera_->Capture();
            if (frame.data) {
                rga_buf = rga_uyvy_to_rgb(frame.data, cam_w_, cam_h_, 640, 640);
                if (rga_buf) rgb = rga_buf;
            }
        }

        // 摄像头不可用时 fallback 到灰图
        if (!rgb) {
            static std::vector<uint8_t> dummy(640 * 640 * 3, 128);
            rgb = dummy.data();
        }

        auto dets = yolo_->Detect(rgb, 640, 640);

        // 板端无屏：命中目标 → 画框，最高置信度帧落盘 BMP 供拉回查看
        float best_hit = 0.0f; bool hit = false;
        for (auto& d : dets) {
            if (d.class_name == en_target && d.confidence >= conf_th) {
                hit = true;
                if (d.confidence > best_hit) best_hit = d.confidence;
                drawBox(rgb, 640, 640, d.x, d.y, d.w, d.h);
            }
        }
        if (hit && !saved) {
            char name[256];
            snprintf(name, sizeof(name), "%s/detect_%s_%d.jpg",
                     save_dir_.c_str(), en_target, static_cast<int>(best_hit * 100));
            drawText(rgb, 640, 640, 8, 8,
                     std::to_string(static_cast<int>(best_hit * 100)), 2);
            if (stbi_write_jpg(name, 640, 640, 3, rgb, 90))
                RCLCPP_INFO(get_logger(), "已保存检测图: %s", name);
            saved = true;
        }
        free(rga_buf);

        // 发布画面物体上下文（置信度百分比，喂给 LLM 做视觉问答）
        std::string objects;
        for (auto& d : dets) {
            if (d.confidence < conf_th) continue;
            if (!objects.empty()) objects += ",";
            objects += d.class_name + "("
                     + std::to_string(static_cast<int>(d.confidence * 100)) + ")";
        }
        if (!objects.empty()) {
            auto ctx = rk3588_voice_assistant_interfaces::msg::VisionContext();
            ctx.objects = objects;
            ctx_pub_->publish(ctx);
        }

        // 发布 bbox Marker 供 PC rviz2 可视化（图像坐标 y 向下，翻转为 rviz 的 y 向上）
        visualization_msgs::msg::MarkerArray markers;
        int mid = 0;
        for (auto& d : dets) {
            if (d.confidence < conf_th) continue;
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "camera";
            m.header.stamp = now();
            m.ns = "yolo";
            m.id = mid++;
            m.type = visualization_msgs::msg::Marker::CUBE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = d.x + d.w / 2.0f;
            m.pose.position.y = 640.0f - (d.y + d.h / 2.0f);
            m.pose.position.z = 0.0f;
            m.pose.orientation.w = 1.0f;
            m.scale.x = static_cast<double>(d.w);
            m.scale.y = static_cast<double>(d.h);
            m.scale.z = 0.01;
            m.color.a = 0.6f;
            m.color.r = 0.0f;
            m.color.g = 1.0f;
            m.color.b = 0.0f;
            m.lifetime = rclcpp::Duration::from_seconds(1.0);
            markers.markers.push_back(m);
        }
        marker_pub_->publish(markers);

        auto fb = std::make_shared<YoloDetect::Feedback>();
        fb->elapsed_sec = 0;
        for (auto& d : dets) {
            if (d.confidence >= conf_th) fb->all_objects.push_back(d.class_name);
            if (d.class_name == en_target && d.confidence > result->best_confidence) {
                result->detected = true; result->best_class = goal->target_class;  // 中文回传，LLM 回复更自然
                result->best_confidence = d.confidence;
                result->bbox_x = d.x; result->bbox_y = d.y;
                result->bbox_w = d.w; result->bbox_h = d.h;
            }
        }
        gh->publish_feedback(fb);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (result->detected)
        RCLCPP_INFO(get_logger(), "检测到 %s (%.2f)", result->best_class.c_str(), result->best_confidence);
    else
        RCLCPP_INFO(get_logger(), "未检测到 %s", goal->target_class.c_str());
    gh->succeed(result);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    auto node = std::make_shared<YoloNode>();
    exe.add_node(node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
    return 0;
}
