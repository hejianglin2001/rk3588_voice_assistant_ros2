// llm_node — RKLLM NPU 推理 + YOLO 目标检测 (LifecycleNode)
#include "rclcpp/rclcpp.hpp"
#include "rk3588_voice_assistant/llm_node.hpp"
#include "llm/rkllm_engine.hpp"
#include <algorithm>
#include <cctype>
#include <cstring>

LlmNode::LlmNode(const rclcpp::NodeOptions& options)
    : LifecycleNode("llm_node", options) {
    declare_parameter("model_path", "");
    declare_parameter("max_tokens", 128);
    declare_parameter("context_len", 512);
}

LlmNode::CallbackReturn LlmNode::on_configure(const rclcpp_lifecycle::State&) {
    auto path = get_parameter("model_path").as_string();
    if (path.empty()) {
        RCLCPP_FATAL(get_logger(), "model_path 未设置"); return CallbackReturn::FAILURE;
    }

    RKLLMEngine::Config cfg;
    cfg.model_path = path;
    cfg.max_new_tokens = get_parameter("max_tokens").as_int();
    cfg.max_context_len = get_parameter("context_len").as_int();

    llm_ = std::make_unique<RKLLMEngine>();
    if (!llm_->Init(cfg)) {
        RCLCPP_FATAL(get_logger(), "LLM 加载失败"); return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "LLM 模型就绪: %s", path.c_str());

    rclcpp::QoS text_qos(10);
    text_qos.reliable().durability_volatile();
    response_pub_ = create_publisher<std_msgs::msg::String>("/llm_response", text_qos);

    reset_srv_ = create_service<std_srvs::srv::Empty>(
        "/reset_context", std::bind(&LlmNode::onReset, this,
                                    std::placeholders::_1, std::placeholders::_2));

    // YOLO Action client
    yolo_client_ = rclcpp_action::create_client<YoloDetect>(this, "/yolo_detect");

    return CallbackReturn::SUCCESS;
}

LlmNode::CallbackReturn LlmNode::on_activate(const rclcpp_lifecycle::State&) {
    rclcpp::QoS text_qos(10);
    text_qos.reliable().durability_volatile();
    asr_sub_ = create_subscription<std_msgs::msg::String>(
        "/recognized_text", text_qos,
        std::bind(&LlmNode::onAsrResult, this, std::placeholders::_1));
    text_sub_ = create_subscription<std_msgs::msg::String>(
        "/text_input", text_qos,
        std::bind(&LlmNode::onText, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "llm_node activated");
    return CallbackReturn::SUCCESS;
}

LlmNode::CallbackReturn LlmNode::on_deactivate(const rclcpp_lifecycle::State&) {
    asr_sub_.reset(); text_sub_.reset();
    return CallbackReturn::SUCCESS;
}

LlmNode::CallbackReturn LlmNode::on_cleanup(const rclcpp_lifecycle::State&) {
    yolo_client_.reset();
    llm_.reset();
    response_pub_.reset(); reset_srv_.reset();
    return CallbackReturn::SUCCESS;
}

LlmNode::CallbackReturn LlmNode::on_shutdown(const rclcpp_lifecycle::State&) {
    return on_cleanup(rclcpp_lifecycle::State());
}

void LlmNode::onReset(const std_srvs::srv::Empty::Request::SharedPtr,
                       std_srvs::srv::Empty::Response::SharedPtr) {
    std::lock_guard<std::mutex> lock(llm_mutex_);
    if (llm_ && llm_->IsReady()) { llm_->ClearHistory(); RCLCPP_INFO(get_logger(), "上下文已清除"); }
}

// ---- 意图识别 ----
// 只判断是不是"找/检测某物"的意图。目标词交给 LLM 抽——std::regex 字节级不懂中文，
// "检测一下一个人你能不能检测到一个人" 这种长句正则抓不干净，LLM 能抽对。
bool LlmNode::isDetectIntent(const std::string& text) {
    static const std::regex re(R"((?:找|寻找|检测|find))", std::regex::icase);
    return std::regex_search(text, re);
}

// LLM 抽词结果轻量清理：去首尾空白 + 尾部标点（字节级安全，不拆碎 UTF-8）
static std::string cleanTarget(std::string t) {
    auto is_space = [](unsigned char c) { return std::isspace(c); };
    t.erase(t.begin(), std::find_if(t.begin(), t.end(),
        [&](unsigned char c) { return !is_space(c); }));
    t.erase(std::find_if(t.rbegin(), t.rend(),
        [&](unsigned char c) { return !is_space(c); }).base(), t.end());
    static const char* suffix[] = {
        "。", "！", "？", "、", "，", ".", "!", "?", ",", ";", ":", "～", "~", "\"", "'"};
    for (;;) {
        bool hit = false;
        for (auto s : suffix) {
            size_t n = std::strlen(s);
            if (t.size() >= n && t.compare(t.size() - n, n, s) == 0) {
                t.erase(t.size() - n);
                hit = true;
                break;
            }
        }
        if (!hit) break;
    }
    return t;
}

// ---- 调用 YOLO ----
void LlmNode::callYoloDetect(const std::string& target) {
    if (!yolo_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_WARN(get_logger(), "YOLO action server 未就绪");
        auto out = std_msgs::msg::String();
        out.data = "[robot]: YOLO 检测服务未就绪，请稍后再试";
        response_pub_->publish(out);
        return;
    }

    auto goal = YoloDetect::Goal();
    goal.target_class = target;
    goal.duration_sec = 5;

    RCLCPP_INFO(get_logger(), "→ 调 YOLO 检测: %s", target.c_str());

    auto opts = rclcpp_action::Client<YoloDetect>::SendGoalOptions();

    opts.goal_response_callback =
        [this](const GoalHandleYolo::SharedPtr&) {
            RCLCPP_INFO(get_logger(), "YOLO goal accepted");
        };

    opts.result_callback =
        [this, target](const GoalHandleYolo::WrappedResult& result) {
            std::string detection_info;
            if (result.result && result.result->detected) {
                auto& r = *result.result;
                detection_info = "检测到" + r.best_class + "，"
                    "置信度" + std::to_string(static_cast<int>(r.best_confidence * 100)) + "%，"
                    "位置(" + std::to_string(r.bbox_x) + "," + std::to_string(r.bbox_y) + ")，"
                    "大小" + std::to_string(r.bbox_w) + "x" + std::to_string(r.bbox_h);
            } else {
                detection_info = "未检测到" + target;
            }
            RCLCPP_INFO(get_logger(), "YOLO 结果: %s", detection_info.c_str());

            // 直接模板回复（确定性，不再调 LLM 总结——那次输出只打 stdout 是白跑的）
            auto out = std_msgs::msg::String();
            out.data = "[robot]: " + detection_info;
            response_pub_->publish(out);
        };

    yolo_client_->async_send_goal(goal, opts);
}

// ---- 消息处理 ----
void LlmNode::onText(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data.empty()) return;
    RCLCPP_INFO(get_logger(), "[text] %s", msg->data.c_str());

    // 检测意图 → LLM 抽词 → YOLO
    if (isDetectIntent(msg->data)) {
        std::string target;
        {
            std::lock_guard<std::mutex> lock(llm_mutex_);
            if (!llm_ || !llm_->IsReady()) return;
            target = llm_->RunSync(
                "从下面这句话提取用户要找/检测的物体，只输出物体名（中文或英文，如「苹果」或「person」）。"
                "没有明确物体就输出「无」。\n用户的话：" + msg->data);
        }
        target = cleanTarget(target);
        if (target.empty() || target == "无") {
            auto out = std_msgs::msg::String();
            out.data = "[robot]: 没听清要找什么，能再说一遍吗？";
            response_pub_->publish(out);
            return;
        }
        RCLCPP_INFO(get_logger(), "LLM 抽词: %s", target.c_str());
        callYoloDetect(target);
        return;
    }

    // 正常 LLM 对话
    std::lock_guard<std::mutex> lock(llm_mutex_);
    if (!llm_ || !llm_->IsReady()) return;

    llm_->Run(msg->data);
    auto out = std_msgs::msg::String();
    out.data = "[robot]: (see stdout)";
    response_pub_->publish(out);
}

void LlmNode::onAsrResult(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "[asr→llm] %s", msg->data.c_str());
    onText(msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    auto node = std::make_shared<LlmNode>();
    exe.add_node(node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
    return 0;
}
