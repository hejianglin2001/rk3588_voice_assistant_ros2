// asr_node — sherpa-onnx 语音识别 (LifecycleNode)
#include "rclcpp/rclcpp.hpp"
#include "rk3588_voice_assistant/asr_node.hpp"
#ifdef HAS_ASR
#include "asr/sherpa_asr.hpp"
#endif

AsrNode::AsrNode(const rclcpp::NodeOptions& options)
    : LifecycleNode("asr_node", options) {
    declare_parameter("asr_model_dir", "/home/topeet/code/rkllm_weight");
}

AsrNode::CallbackReturn AsrNode::on_configure(const rclcpp_lifecycle::State&) {
#ifdef HAS_ASR
    asr_ = std::make_unique<SherpaASR>();
    SherpaASR::Config cfg;
    cfg.model_dir = get_parameter("asr_model_dir").as_string();
    cfg.sample_rate = 16000; cfg.num_threads = 4;
    if (!asr_->Init(cfg)) {
        RCLCPP_ERROR(get_logger(), "ASR 模型加载失败"); return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "ASR 模型就绪: %s", cfg.model_dir.c_str());
#endif

    rclcpp::QoS text_qos(10);
    text_qos.reliable().durability_volatile();
    text_pub_ = create_publisher<std_msgs::msg::String>("/recognized_text", text_qos);

    return CallbackReturn::SUCCESS;
}

AsrNode::CallbackReturn AsrNode::on_activate(const rclcpp_lifecycle::State&) {
    rclcpp::QoS audio_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    audio_sub_ = create_subscription<rk3588_voice_assistant_interfaces::msg::AudioChunk>(
        "/utterance_audio", audio_qos,
        std::bind(&AsrNode::onAudio, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "asr_node activated");
    return CallbackReturn::SUCCESS;
}

AsrNode::CallbackReturn AsrNode::on_deactivate(const rclcpp_lifecycle::State&) {
    audio_sub_.reset();
    return CallbackReturn::SUCCESS;
}

AsrNode::CallbackReturn AsrNode::on_cleanup(const rclcpp_lifecycle::State&) {
    asr_.reset();
    text_pub_.reset();
    return CallbackReturn::SUCCESS;
}

AsrNode::CallbackReturn AsrNode::on_shutdown(const rclcpp_lifecycle::State&) {
    return on_cleanup(rclcpp_lifecycle::State());
}

void AsrNode::onAudio(const rk3588_voice_assistant_interfaces::msg::AudioChunk::SharedPtr msg) {
#ifdef HAS_ASR
    if (!asr_ || !asr_->IsReady()) return;
    auto text = asr_->Recognize(msg->samples.data(), msg->samples.size());
    if (text.empty()) return;
    RCLCPP_INFO(get_logger(), "ASR: %s", text.c_str());

    auto out = std_msgs::msg::String();
    out.data = text;
    text_pub_->publish(out);
#endif
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    auto node = std::make_shared<AsrNode>();
    exe.add_node(node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
    return 0;
}
