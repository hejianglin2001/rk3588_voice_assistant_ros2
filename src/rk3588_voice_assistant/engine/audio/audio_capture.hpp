// Copyright (c) 2025 Edge Voice AI Assistant
//
// AudioCapture — miniaudio RAII 封装，工业级音频采集
//
// 设计原则：
// 1. miniaudio callback 运行在 ALSA 高优先级实时线程 — 只做 memcpy 进 RingBuffer
// 2. 所有设备操作封装在 RAII 生命周期内
// 3. 错误处理走 Result<T>，不抛异常

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include "audio/ring_buffer.hpp"

// 前向声明 — 不在此头文件暴露 miniaudio 实现细节
struct ma_device;
struct ma_context;

namespace audio {

// ============================================================================
// Result<T> — 工业级错误处理，避免 C++ 异常在跨模块边界的副作用
// ============================================================================
template <typename T>
struct Result {
    T value{};
    std::string error_msg;
    bool ok = false;

    static Result Success(T val) { return {std::move(val), {}, true}; }
    static Result Fail(std::string_view msg) { return {{}, std::string(msg), false}; }

    explicit operator bool() const noexcept { return ok; }
};

template <>
struct Result<void> {
    std::string error_msg;
    bool ok = false;

    static Result Success() { return {{}, true}; }
    static Result Fail(std::string_view msg) { return {std::string(msg), false}; }

    explicit operator bool() const noexcept { return ok; }
};

// ============================================================================
// AudioCapture 配置
// ============================================================================
struct AudioCaptureConfig {
    /// ALSA 设备标识符，例如 "plughw:2,0"
    std::string device_id;
    /// 采样率，Hz，常用 16000 (ASR) 或 44100
    uint32_t sample_rate = 16000;
    /// 声道数：1（单声道，ASR 场景主流）或 2
    uint32_t channels = 1;
    /// miniaudio 每次回调的帧数，影响延迟和 CPU 负载
    uint32_t period_frames = 480;  // ~30ms @ 16kHz
};

// ============================================================================
// AudioCapture — miniaudio RAII 封装
// ============================================================================
class AudioCapture {
public:
    using SampleType = int16_t;  // S16_LE

    explicit AudioCapture(const AudioCaptureConfig& config);
    ~AudioCapture();

    AudioCapture(const AudioCapture&) = delete;
    AudioCapture& operator=(const AudioCapture&) = delete;
    AudioCapture(AudioCapture&&) = delete;
    AudioCapture& operator=(AudioCapture&&) = delete;

    /// 打开设备 + 启动音频流
    Result<void> Start();

    /// 停止音频流 + 关闭设备
    void Stop();

    /// 从内部 RingBuffer 读取已捕获的 PCM 样本（业务线程调用）
    /// @param dst   目标缓冲区指针
    /// @param count 期望读取的样本数
    /// @return 实际读取的样本数
    size_t ReadSamples(int16_t* dst, size_t count) noexcept;

    /// RingBuffer 中可读取的样本数
    size_t AvailableSamples() const noexcept;

    /// 丢弃 RingBuffer 中的所有缓存数据
    void Flush() noexcept;

    /// 设备是否正在采集
    bool IsRunning() const noexcept { return running_.load(std::memory_order_acquire); }

    /// 获取实际采样率（可能与配置不同，取决于硬件能力）
    uint32_t ActualSampleRate() const noexcept { return actual_sample_rate_; }

private:
    /// miniaudio 数据回调 — 运行在 ALSA 实时音频线程
    /// 💡 【面试考点】这里为什么只做 memcpy？
    ///   答：ma_device 的 data_callback 在中断上下文/高优先级线程中执行，
    ///   做任何阻塞操作（malloc、lock、文件 I/O）都可能阻塞音频硬件 DMA，
    ///   导致 overrun（录音丢帧）或 underrun（播放爆音）。
    ///   正确做法：无锁写入 RingBuffer，业务线程异步消费。
    static void DataCallback(struct ma_device* device, void* output,
                             const void* input, uint32_t frame_count);

    // Pimpl — 隔离 miniaudio 头文件，避免污染编译单元
    struct Impl;
    std::unique_ptr<Impl> impl_;

    std::unique_ptr<RingBuffer<int16_t>> ring_buffer_;
    AudioCaptureConfig config_;
    uint32_t actual_sample_rate_ = 0;
    std::atomic<bool> running_{false};
};

}  // namespace audio
