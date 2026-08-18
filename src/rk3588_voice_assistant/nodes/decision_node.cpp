// decision_node — 决策层：把 /task_command 路由到执行层
#include "rclcpp/rclcpp.hpp"
#include "rk3588_voice_assistant/decision_node.hpp"
#include <chrono>

DecisionNode::DecisionNode(const rclcpp::NodeOptions& options)
    : Node("decision_node", options) {
    rclcpp::QoS qos(10);
    qos.reliable().durability_volatile();

    cmd_sub_ = create_subscription<rk3588_voice_assistant_interfaces::msg::TaskCommand>(
        "/task_command", qos,
        std::bind(&DecisionNode::onTaskCommand, this, std::placeholders::_1));
    response_pub_ = create_publisher<std_msgs::msg::String>("/llm_response", qos);
    query_pub_ = create_publisher<std_msgs::msg::String>("/llm_query", qos);
    yolo_client_ = rclcpp_action::create_client<YoloDetect>(this, "/yolo_detect");

    RCLCPP_INFO(get_logger(), "decision_node 就绪，等待 /task_command");
}

void DecisionNode::onTaskCommand(
    const rk3588_voice_assistant_interfaces::msg::TaskCommand::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "[task] action=%s target=%s", msg->action.c_str(), msg->target.c_str());

    if (msg->action == "detect") {
        callYoloDetect(msg->target, msg->raw);
    } else if (msg->action == "chat") {
        // chat：llm_node 已生成完整回复，直接透传
        auto out = std_msgs::msg::String();
        out.data = "[robot]: " + msg->target;
        response_pub_->publish(out);
    } else {
        RCLCPP_WARN(get_logger(), "未知 action: %s", msg->action.c_str());
    }
}

void DecisionNode::callYoloDetect(const std::string& target, const std::string& raw) {
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
    opts.result_callback =
        [this, target, raw](const GoalHandleYolo::WrappedResult& result) {
            std::string detection_info;
            if (result.result && result.result->detected) {
                auto& r = *result.result;
                detection_info = "检测到 " + r.best_class + "，置信度 "
                    + std::to_string(static_cast<int>(r.best_confidence * 100)) + "%";
            } else {
                detection_info = "未检测到 " + target;
            }
            RCLCPP_INFO(get_logger(), "YOLO 结果: %s", detection_info.c_str());

            // 结果交回认知层，让 LLM 生成自然语言回答
            auto q = std_msgs::msg::String();
            q.data = "检测结果：" + detection_info
                   + "\n用户原话：" + raw
                   + "\n请用一句自然的中文回答用户。";
            query_pub_->publish(q);
        };

    yolo_client_->async_send_goal(goal, opts);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DecisionNode>());
    rclcpp::shutdown();
    return 0;
}
