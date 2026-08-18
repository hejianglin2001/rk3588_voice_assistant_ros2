// ============================================================================
// decision_node — 决策层：任务命令路由（普通 Node，纯逻辑无硬件资源）
//
// 职责：接收认知层下发的 /task_command，按 action 路由到执行层：
//   detect → 调用 /yolo_detect Action（视觉检测）
//   chat   → 直接透传回复到 /llm_response
// 检测结束后把结果拼接成 /llm_query，交回 llm_node 生成自然语言回答。
//
// Sub: /task_command  (TaskCommand) — 结构化任务命令
// Pub: /llm_response  (String)      — chat 直接回复
// Pub: /llm_query     (String)      — detect 结果拼接的回答请求
// Act: /yolo_detect   (Client)      — 调用视觉检测
// ============================================================================
#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "rk3588_voice_assistant_interfaces/msg/task_command.hpp"
#include "rk3588_voice_assistant_interfaces/action/yolo_detect.hpp"

class DecisionNode : public rclcpp::Node {
public:
    using YoloDetect = rk3588_voice_assistant_interfaces::action::YoloDetect;
    using GoalHandleYolo = rclcpp_action::ClientGoalHandle<YoloDetect>;

    explicit DecisionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~DecisionNode() override = default;

private:
    void onTaskCommand(const rk3588_voice_assistant_interfaces::msg::TaskCommand::SharedPtr msg);
    void callYoloDetect(const std::string& target, const std::string& raw);

    rclcpp::Subscription<rk3588_voice_assistant_interfaces::msg::TaskCommand>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr query_pub_;
    rclcpp_action::Client<YoloDetect>::SharedPtr yolo_client_;
};
