// ============================================================================
// audio_vad_node — 麦克风采集 + VAD 断句 (LifecycleNode)
//
// Sub: /voice_trigger   (Empty)      — 触发一次录音
// Pub: /utterance_audio  (AudioChunk) — VAD 断句后的 16kHz 单声道 PCM
// Srv: /vad_state       (Trigger)    — 查询当前 VAD 状态
// ============================================================================
#pragma once

#include <memory>
#include <atomic>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rk3588_voice_assistant_interfaces/msg/audio_chunk.hpp"

namespace audio { class AudioCapture; }
class VadEngine;

class AudioVadNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit AudioVadNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~AudioVadNode() override;

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

private:
    void onTrigger(const std_msgs::msg::Empty::SharedPtr msg);
    void onVadState(const std_srvs::srv::Trigger::Request::SharedPtr req,
                    std_srvs::srv::Trigger::Response::SharedPtr res);

    // ---- 硬件 ----
    std::unique_ptr<audio::AudioCapture> mic_;
    std::unique_ptr<VadEngine> vad_;

    // ---- ROS2 ----
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
    rclcpp::Publisher<rk3588_voice_assistant_interfaces::msg::AudioChunk>::SharedPtr audio_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr vad_state_srv_;

    std::atomic<bool> listening_{false};
};
