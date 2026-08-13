// ============================================================================
// llm_node — NPU 大模型推理 + YOLO 目标检测调用 (LifecycleNode)
//
// Sub: /recognized_text (String) — ASR 结果
// Sub: /text_input      (String) — 文字输入
// Pub: /llm_response    (String) — LLM 回复
// Srv: /reset_context   (Empty)  — 清空对话历史
// Act: /yolo_detect     (Client) — 调用 YOLO 检测
// ============================================================================
#pragma once

#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rk3588_voice_assistant_interfaces/action/yolo_detect.hpp"

class RKLLMEngine;

class LlmNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit LlmNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LlmNode() override = default;

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using YoloDetect = rk3588_voice_assistant_interfaces::action::YoloDetect;
    using GoalHandleYolo = rclcpp_action::ClientGoalHandle<YoloDetect>;

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

    // 意图识别：只判断是否是"找/检测某物"的意图（目标词交给 LLM 抽，正则搞不定中文分词）
    bool isDetectIntent(const std::string& text);
    // 发送 YOLO Action，异步回调中喂 LLM
    void callYoloDetect(const std::string& target);

    std::unique_ptr<RKLLMEngine> llm_;
    std::mutex llm_mutex_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr asr_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
    rclcpp_action::Client<YoloDetect>::SharedPtr yolo_client_;
};
