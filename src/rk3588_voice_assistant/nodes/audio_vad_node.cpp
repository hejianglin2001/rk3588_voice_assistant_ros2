// audio_vad_node — 麦克风采集 + VAD 断句 (LifecycleNode)
#include "rclcpp/rclcpp.hpp"
#include "rk3588_voice_assistant/audio_vad_node.hpp"
#include "audio/audio_capture.hpp"
#ifdef HAS_VAD
#include "vad/vad_engine.hpp"
#endif
#include <unistd.h>

static std::vector<int16_t> extractLeft(const std::vector<int16_t>& stereo) {
    std::vector<int16_t> mono(stereo.size() / 2);
    for (size_t i = 0; i < mono.size(); ++i) mono[i] = stereo[i * 2];
    return mono;
}
static std::vector<int16_t> resampleTo16k(const std::vector<int16_t>& src, uint32_t sr) {
    if (sr == 16000) return src;
    double r = 16000.0 / sr;
    std::vector<int16_t> dst(static_cast<size_t>(src.size() * r));
    for (size_t i = 0; i < dst.size(); ++i) {
        double s = i / r;
        size_t i0 = static_cast<size_t>(s), i1 = std::min(i0 + 1, src.size() - 1);
        dst[i] = static_cast<int16_t>(src[i0] + (s - i0) * (src[i1] - src[i0]));
    }
    return dst;
}

AudioVadNode::AudioVadNode(const rclcpp::NodeOptions& options)
    : LifecycleNode("audio_vad_node", options) {
    declare_parameter("mic_device", "plughw:2,0");
    declare_parameter("sample_rate", 44100);
    declare_parameter("channels", 2);
}

AudioVadNode::~AudioVadNode() = default;

AudioVadNode::CallbackReturn AudioVadNode::on_configure(const rclcpp_lifecycle::State&) {
    auto dev = get_parameter("mic_device").as_string();
    auto sr  = get_parameter("sample_rate").as_int();
    auto ch  = get_parameter("channels").as_int();

    audio::AudioCaptureConfig cfg;
    cfg.device_id = dev; cfg.sample_rate = static_cast<uint32_t>(sr);
    cfg.channels = static_cast<uint32_t>(ch); cfg.period_frames = 480;
    mic_ = std::make_unique<audio::AudioCapture>(cfg);
    if (!mic_->Start()) {
        RCLCPP_ERROR(get_logger(), "麦克风启动失败"); return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "麦克风就绪: %s %ldHz %ldch", dev.c_str(), sr, ch);

#ifdef HAS_VAD
    vad_ = std::make_unique<VadEngine>();
    vad_->Init({16000, 2});
    RCLCPP_INFO(get_logger(), "VAD 就绪");
#endif

    rclcpp::QoS audio_qos(5);
    audio_qos.best_effort().durability_volatile();
    audio_pub_ = create_publisher<rk3588_voice_assistant_interfaces::msg::AudioChunk>("/utterance_audio", audio_qos);

    vad_state_srv_ = create_service<std_srvs::srv::Trigger>(
        "/vad_state", std::bind(&AudioVadNode::onVadState, this,
                                std::placeholders::_1, std::placeholders::_2));

    return CallbackReturn::SUCCESS;
}

AudioVadNode::CallbackReturn AudioVadNode::on_activate(const rclcpp_lifecycle::State&) {
    rclcpp::QoS trigger_qos(1);
    trigger_qos.reliable().transient_local();
    trigger_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/voice_trigger", trigger_qos,
        std::bind(&AudioVadNode::onTrigger, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "audio_vad_node activated");
    return CallbackReturn::SUCCESS;
}

AudioVadNode::CallbackReturn AudioVadNode::on_deactivate(const rclcpp_lifecycle::State&) {
    trigger_sub_.reset();
    return CallbackReturn::SUCCESS;
}

AudioVadNode::CallbackReturn AudioVadNode::on_cleanup(const rclcpp_lifecycle::State&) {
    mic_.reset(); vad_.reset();
    audio_pub_.reset(); vad_state_srv_.reset();
    return CallbackReturn::SUCCESS;
}

AudioVadNode::CallbackReturn AudioVadNode::on_shutdown(const rclcpp_lifecycle::State&) {
    return on_cleanup(rclcpp_lifecycle::State());
}

void AudioVadNode::onVadState(const std_srvs::srv::Trigger::Request::SharedPtr,
                               std_srvs::srv::Trigger::Response::SharedPtr res) {
    res->success = true;
    if (listening_.load()) res->message = "listening";
    else if (!vad_ || !vad_->IsReady()) res->message = "error: VAD not ready";
    else res->message = "idle";
}

void AudioVadNode::onTrigger(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
    if (listening_.exchange(true)) { RCLCPP_WARN(get_logger(), "已在录音"); return; }

    RCLCPP_INFO(get_logger(), "开始录音...");
    const size_t maxSamples = mic_->ActualSampleRate() * 2 * 10;
    std::vector<int16_t> buffer; buffer.reserve(maxSamples);

#ifdef HAS_VAD
    std::vector<int16_t> vadBuf; vadBuf.reserve(16000);
    int silenceFrames = 0, speechFrames = 0;
    constexpr int kFrame = 320, kMinSpeech = 3, kSilence = 100;
    enum { kPending, kSpeaking } state = kPending;
    const uint32_t srcRate = mic_->ActualSampleRate();
#endif

    mic_->Flush();
    size_t total = 0;
    while (total < maxSamples && rclcpp::ok()) {
        size_t chunk = mic_->ActualSampleRate() / 15;
        size_t old = buffer.size(); buffer.resize(old + chunk);
        size_t n = mic_->ReadSamples(buffer.data() + old, chunk);
        buffer.resize(old + n); total += n;

#ifdef HAS_VAD
        if (vad_ && vad_->IsReady() && n > 0) {
            auto mono16 = resampleTo16k(extractLeft({buffer.data() + old, buffer.data() + old + n}), srcRate);
            vadBuf.insert(vadBuf.end(), mono16.begin(), mono16.end());
            while (vadBuf.size() >= static_cast<size_t>(kFrame)) {
                bool v = vad_->IsVoice(vadBuf.data(), kFrame);
                vadBuf.erase(vadBuf.begin(), vadBuf.begin() + kFrame);
                if (state == kPending) {
                    if (v) { if (++speechFrames >= kMinSpeech) state = kSpeaking; }
                    else speechFrames = 0;
                } else {
                    if (v) silenceFrames = 0; else if (++silenceFrames >= kSilence) goto done;
                }
            }
        }
#endif
        usleep(5'000);
    }
#ifdef HAS_VAD
done:
#endif
    if (buffer.size() < mic_->ActualSampleRate()) {
        RCLCPP_INFO(get_logger(), "太短，忽略");
        listening_.store(false); return;
    }

    auto mono16 = resampleTo16k(extractLeft(buffer), mic_->ActualSampleRate());
    rk3588_voice_assistant_interfaces::msg::AudioChunk msg;
    msg.header.stamp = now();
    msg.header.frame_id = "audio";
    msg.samples.assign(mono16.begin(), mono16.end());
    msg.sample_rate = 16000;

    audio_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "语音已发布 (%zu采样, ~%.1fs)", mono16.size(),
                static_cast<double>(mono16.size()) / 16000.0);
    listening_.store(false);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    auto node = std::make_shared<AudioVadNode>();
    exe.add_node(node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
    return 0;
}
