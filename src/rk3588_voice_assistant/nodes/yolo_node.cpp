// yolo_node — YOLO NPU 目标检测 Action 服务端 (MIPI 摄像头实时输入)
#include "rclcpp/rclcpp.hpp"
#include "rk3588_voice_assistant/yolo_node.hpp"
#include "yolo/yolo_engine.hpp"
#include "yolo/camera_capture.h"
#include "yolo/rga_convert.h"
#include <chrono>
#include <thread>
#include <unordered_map>

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

YoloNode::YoloNode(const rclcpp::NodeOptions& options)
    : LifecycleNode("yolo_node", options) {
    declare_parameter("model_path", "");
    declare_parameter("conf_threshold", 0.25);
    declare_parameter("camera_device", "/dev/video11");
    declare_parameter("camera_width", 1920);
    declare_parameter("camera_height", 1080);
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

    while (rclcpp::ok() && now() < deadline) {
        if (gh->is_canceling()) { gh->canceled(result); return; }

        const uint8_t* rgb = nullptr;
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
        free(rga_buf);

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
