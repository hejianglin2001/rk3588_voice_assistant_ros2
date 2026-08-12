// ============================================================================
// asr_node — 语音识别 (LifecycleNode)
//
// Sub: /utterance_audio (AudioChunk) — 16kHz 单声道 PCM
// Pub: /recognized_text  (String)    — ASR 识别结果
// ============================================================================
#pragma once

#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "rk3588_voice_assistant_interfaces/msg/audio_chunk.hpp"

class SherpaASR;

class AsrNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit AsrNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~AsrNode() override = default;

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

private:
    void onAudio(const rk3588_voice_assistant_interfaces::msg::AudioChunk::SharedPtr msg);

    std::unique_ptr<SherpaASR> asr_;
    rclcpp::Subscription<rk3588_voice_assistant_interfaces::msg::AudioChunk>::SharedPtr audio_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_pub_;
};
