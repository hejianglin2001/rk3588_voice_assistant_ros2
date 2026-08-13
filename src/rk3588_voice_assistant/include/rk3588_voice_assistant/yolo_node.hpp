// ============================================================================
// yolo_node — YOLO NPU 目标检测 Action 服务端 (LifecycleNode)
//
// Action: /yolo_detect (YoloDetect)
// 输入:   MIPI 摄像头 /dev/video11 (UYVY) → RGA → RGB → NPU
// ============================================================================
#pragma once

#include <memory>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rk3588_voice_assistant_interfaces/action/yolo_detect.hpp"

class YoloEngine;
class CameraCapture;

class YoloNode : public rclcpp_lifecycle::LifecycleNode {
public:
    using YoloDetect = rk3588_voice_assistant_interfaces::action::YoloDetect;
    using GoalHandle = rclcpp_action::ServerGoalHandle<YoloDetect>;

    explicit YoloNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~YoloNode() override = default;

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

private:
    rclcpp_action::GoalResponse
    handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const YoloDetect::Goal>);
    rclcpp_action::CancelResponse
    handle_cancel(const std::shared_ptr<GoalHandle>);
    void execute(const std::shared_ptr<GoalHandle>);

    std::unique_ptr<YoloEngine> yolo_;
    std::unique_ptr<CameraCapture> camera_;
    rclcpp_action::Server<YoloDetect>::SharedPtr action_server_;

    int cam_w_ = 1920, cam_h_ = 1080;  // camera capture resolution
    std::string save_dir_;             // 检测图落盘目录（板端无屏，供拉回查看）
};
