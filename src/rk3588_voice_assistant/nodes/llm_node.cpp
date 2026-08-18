// llm_node — 认知层：RKLLM NPU 推理，输出结构化 TaskCommand（LifecycleNode）
// 不直接调 YOLO：意图判断(正则) + 目标抽词/回答生成(LLM)，结果下发 /task_command 给决策层
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
    task_pub_ = create_publisher<rk3588_voice_assistant_interfaces::msg::TaskCommand>(
        "/task_command", text_qos);

    reset_srv_ = create_service<std_srvs::srv::Empty>(
        "/reset_context", std::bind(&LlmNode::onReset, this,
                                    std::placeholders::_1, std::placeholders::_2));

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
    ctx_sub_ = create_subscription<rk3588_voice_assistant_interfaces::msg::VisionContext>(
        "/vision_context", text_qos,
        std::bind(&LlmNode::onVisionContext, this, std::placeholders::_1));
    query_sub_ = create_subscription<std_msgs::msg::String>(
        "/llm_query", text_qos,
        std::bind(&LlmNode::onLlmQuery, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "llm_node activated");
    return CallbackReturn::SUCCESS;
}

LlmNode::CallbackReturn LlmNode::on_deactivate(const rclcpp_lifecycle::State&) {
    asr_sub_.reset(); text_sub_.reset(); ctx_sub_.reset(); query_sub_.reset();
    return CallbackReturn::SUCCESS;
}

LlmNode::CallbackReturn LlmNode::on_cleanup(const rclcpp_lifecycle::State&) {
    llm_.reset();
    response_pub_.reset(); task_pub_.reset(); reset_srv_.reset();
    return CallbackReturn::SUCCESS;
}

LlmNode::CallbackReturn LlmNode::on_shutdown(const rclcpp_lifecycle::State&) {
    return on_cleanup(rclcpp_lifecycle::State());
}

void LlmNode::onReset(const std_srvs::srv::Empty::Request::SharedPtr,
                       std_srvs::srv::Empty::Response::SharedPtr) {
    std::lock_guard<std::mutex> lock(llm_mutex_);
    if (llm_ && llm_->IsReady()) { llm_->ClearHistory(); RCLCPP_INFO(get_logger(), "上下文已清除"); }
    last_ctx_.clear();  // 视觉上下文一并清空
}

// 保存最近一帧画面物体，供"桌上有什么"这类视觉问答使用
void LlmNode::onVisionContext(const rk3588_voice_assistant_interfaces::msg::VisionContext::SharedPtr msg) {
    if (msg->objects.empty()) return;
    last_ctx_ = msg->objects;
}

// decision_node 拼接好的最终回答请求 → LLM 生成自然语言回复
void LlmNode::onLlmQuery(const std_msgs::msg::String::SharedPtr msg) {
    std::string reply;
    {
        std::lock_guard<std::mutex> lock(llm_mutex_);
        if (!llm_ || !llm_->IsReady()) return;
        reply = llm_->RunSync(msg->data);
    }
    auto out = std_msgs::msg::String();
    out.data = "[robot]: " + reply;
    response_pub_->publish(out);
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

// ---- 消息处理 ----
void LlmNode::onText(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data.empty()) return;
    RCLCPP_INFO(get_logger(), "[text] %s", msg->data.c_str());

    // 检测意图 → LLM 抽词 → 下发 task_command(detect)
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
        auto cmd = rk3588_voice_assistant_interfaces::msg::TaskCommand();
        cmd.action = "detect";
        cmd.target = target;
        cmd.raw = msg->data;
        task_pub_->publish(cmd);
        return;
    }

    // 对话意图 → LLM 生成回答（注入视觉上下文）→ 下发 task_command(chat)
    std::string prompt = msg->data;
    if (!last_ctx_.empty())
        prompt = "当前画面检测到的物体：" + last_ctx_
               + "\n用户问：" + msg->data
               + "\n请结合检测结果回答，不要编造画面里没有的物体。";

    std::string reply;
    {
        std::lock_guard<std::mutex> lock(llm_mutex_);
        if (!llm_ || !llm_->IsReady()) return;
        reply = llm_->RunSync(prompt);
    }
    auto cmd = rk3588_voice_assistant_interfaces::msg::TaskCommand();
    cmd.action = "chat";
    cmd.target = reply;
    cmd.raw = msg->data;
    task_pub_->publish(cmd);
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
