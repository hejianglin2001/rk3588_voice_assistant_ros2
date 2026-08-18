// ============================================================================
// llm_node — 认知层：NPU 大模型推理（LifecycleNode）
//
// 职责：把用户文本转成结构化任务命令，下发给决策层；并生成最终自然语言回答。
// 不直接调用 YOLO——解耦认知层与执行层（通过 decision_node 路由）。
//
// Sub: /recognized_text (String)         — ASR 结果
// Sub: /text_input      (String)         — 文字输入
// Sub: /vision_context  (VisionContext)  — 画面物体上下文（视觉问答用）
// Sub: /llm_query       (String)         — decision_node 拼接的最终回答请求
// Pub: /task_command    (TaskCommand)    — 结构化任务命令
// Pub: /llm_response    (String)         — 最终回复
// Srv: /reset_context   (Empty)          — 清空对话历史 + 视觉上下文
// ============================================================================
#pragma once

#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rk3588_voice_assistant_interfaces/msg/task_command.hpp"
#include "rk3588_voice_assistant_interfaces/msg/vision_context.hpp"

class RKLLMEngine;

class LlmNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit LlmNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LlmNode() override = default;

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

private:
    void onText(const std_msgs::msg::String::SharedPtr msg);
    void onAsrResult(const std_msgs::msg::String::SharedPtr msg);
    void onReset(const std_srvs::srv::Empty::Request::SharedPtr,
                 std_srvs::srv::Empty::Response::SharedPtr);
    void onVisionContext(const rk3588_voice_assistant_interfaces::msg::VisionContext::SharedPtr msg);
    void onLlmQuery(const std_msgs::msg::String::SharedPtr msg);

    // 意图识别：只判断是否是"找/检测某物"的意图（目标词交给 LLM 抽，正则搞不定中文分词）
    bool isDetectIntent(const std::string& text);

    std::unique_ptr<RKLLMEngine> llm_;
    std::mutex llm_mutex_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr asr_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_sub_;
    rclcpp::Subscription<rk3588_voice_assistant_interfaces::msg::VisionContext>::SharedPtr ctx_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr query_sub_;
    rclcpp::Publisher<rk3588_voice_assistant_interfaces::msg::TaskCommand>::SharedPtr task_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;

    std::string last_ctx_;   // 最近一帧画面物体上下文（视觉问答用）
};
