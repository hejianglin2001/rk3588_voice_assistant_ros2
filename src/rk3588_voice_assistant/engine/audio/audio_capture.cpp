// Copyright (c) 2025 Edge Voice AI Assistant
//
// AudioCapture 实现 — Pimpl 隔离 miniaudio 头文件

#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"

#include "audio_capture.hpp"
#include "ring_buffer.hpp"

#include <cstring>

namespace audio {

// ============================================================================
// Pimpl — miniaudio 库内部类型不暴露到头文件
// ============================================================================
struct AudioCapture::Impl {
    ma_context context{};
    ma_device device{};
    bool context_initialized = false;
    bool device_initialized = false;
};

// ============================================================================
// RAII 生命周期
// ============================================================================

AudioCapture::AudioCapture(const AudioCaptureConfig& config)
    : impl_(std::make_unique<Impl>())
    , ring_buffer_(std::make_unique<RingBuffer<int16_t>>(config.sample_rate * 2))  // 2 秒缓冲
    , config_(config)
{
}

AudioCapture::~AudioCapture()
{
    Stop();
}

// ============================================================================
// Start/Stop
// ============================================================================

Result<void> AudioCapture::Start()
{
    if (impl_->device_initialized) {
        return Result<void>::Fail("AudioCapture: already started");
    }

    // --- 1. 初始化 miniaudio context（强制 ALSA，跳过 PulseAudio）---
    ma_backend backends[] = { ma_backend_alsa };
    ma_result result = ma_context_init(backends, 1, nullptr, &impl_->context);
    if (result != MA_SUCCESS) {
        return Result<void>::Fail(std::string("ma_context_init failed: ") + ma_result_description(result));
    }
    impl_->context_initialized = true;

    // --- 2. 指定 ALSA 采集设备 ---
    ma_device_id device_id{};
    ma_device_id* p_device_id = nullptr;
    if (!config_.device_id.empty()) {
        strncpy(device_id.alsa, config_.device_id.c_str(), sizeof(device_id.alsa) - 1);
        p_device_id = &device_id;
    }

    // --- 3. 配置采集设备 ---
    ma_device_config dev_cfg = ma_device_config_init(ma_device_type_capture);
    dev_cfg.capture.pDeviceID   = p_device_id;
    dev_cfg.capture.format      = ma_format_s16;
    dev_cfg.capture.channels    = config_.channels;
    dev_cfg.sampleRate          = config_.sample_rate;
    dev_cfg.periodSizeInFrames  = config_.period_frames;
    dev_cfg.dataCallback        = &AudioCapture::DataCallback;
    dev_cfg.pUserData           = this;

    result = ma_device_init(&impl_->context, &dev_cfg, &impl_->device);
    if (result != MA_SUCCESS) {
        ma_context_uninit(&impl_->context);
        impl_->context_initialized = false;
        return Result<void>::Fail(std::string("ma_device_init failed: ") + ma_result_description(result));
    }
    impl_->device_initialized = true;
    actual_sample_rate_ = impl_->device.sampleRate;

    // --- 3. 启动音频流 ---
    result = ma_device_start(&impl_->device);
    if (result != MA_SUCCESS) {
        ma_device_uninit(&impl_->device);
        impl_->device_initialized = false;
        ma_context_uninit(&impl_->context);
        impl_->context_initialized = false;
        return Result<void>::Fail(std::string("ma_device_start failed: ") + ma_result_description(result));
    }

    running_.store(true, std::memory_order_release);
    return Result<void>::Success();
}

void AudioCapture::Stop()
{
    running_.store(false, std::memory_order_release);

    if (impl_->device_initialized) {
        ma_device_stop(&impl_->device);
        ma_device_uninit(&impl_->device);
        impl_->device_initialized = false;
    }

    if (impl_->context_initialized) {
        ma_context_uninit(&impl_->context);
        impl_->context_initialized = false;
    }
}

// ============================================================================
// 数据回调 — 实时音频线程
// ============================================================================

void AudioCapture::DataCallback(ma_device* device, void* /*output*/,
                                 const void* input, uint32_t frame_count)
{
    if (input == nullptr) return;

    // 💡 【面试考点】这里只能做 O(1) 的无锁操作。
    //   不能：malloc、fopen、printf、lock。
    //   可以：memcpy、atomic store、ring buffer push。
    //   违规后果：音频 underrun → ALSA xrun → 录音丢帧/爆音。
    auto* self = static_cast<AudioCapture*>(device->pUserData);
    if (self == nullptr) return;

    const auto* samples = static_cast<const int16_t*>(input);
    uint32_t total_samples = frame_count * self->config_.channels;

    self->ring_buffer_->Write(samples, total_samples);
}

// ============================================================================
// 数据读取 — 业务线程
// ============================================================================

size_t AudioCapture::ReadSamples(int16_t* dst, size_t count) noexcept
{
    return ring_buffer_->Read(dst, count);
}

size_t AudioCapture::AvailableSamples() const noexcept
{
    return ring_buffer_->AvailableRead();
}

void AudioCapture::Flush() noexcept
{
    ring_buffer_->Reset();
}

}  // namespace audio
