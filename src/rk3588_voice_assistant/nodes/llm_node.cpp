// llm_node — RKLLM NPU 推理 + YOLO 目标检测 (LifecycleNode)
#include "rclcpp/rclcpp.hpp"
#include "rk3588_voice_assistant/llm_node.hpp"
#include "llm/rkllm_engine.hpp"

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
std::string LlmNode::extractDetectTarget(const std::string& text) {
    // 匹配: "找X" "寻找X" "找一下X" "find X" "检测X"
    static const std::regex re(
        R"(找(?:一下|一个)?(\S+?)[。！？\.\!\?]?$|寻找(\S+)|^find\s+(\S+)|检测(\S+))",
        std::regex::icase);
    std::smatch m;
    if (std::regex_search(text, m, re)) {
        for (int i = 1; i <= 4; i++) {
            if (m[i].matched && !m[i].str().empty()) return m[i].str();
        }
    }
    return "";
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

            std::lock_guard<std::mutex> lock(llm_mutex_);
            auto out = std_msgs::msg::String();
            if (llm_ && llm_->IsReady()) {
                llm_->Run("用户让你找「" + target + "」。检测结果：" + detection_info
                    + "。请用一句话告诉用户结果。");
                out.data = "[robot]: " + detection_info;
            } else {
                out.data = "[robot]: " + detection_info;
            }
            response_pub_->publish(out);
        };

    yolo_client_->async_send_goal(goal, opts);
}

// ---- 消息处理 ----
void LlmNode::onText(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data.empty()) return;
    RCLCPP_INFO(get_logger(), "[text] %s", msg->data.c_str());

    // 优先检测"找X"命令
    auto target = extractDetectTarget(msg->data);
    if (!target.empty()) {
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
