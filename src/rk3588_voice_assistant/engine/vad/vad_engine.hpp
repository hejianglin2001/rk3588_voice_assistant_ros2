// ============================================================================
// 文件: src/vad/vad_engine.hpp
// 作用: VAD（语音活动检测）引擎的 C++ RAII 封装
// 底层: libfvad（WebRTC VAD 的 C 独立库）
// ============================================================================
//
// 💡 【面试考点】为什么用 WebRTC VAD 而不是简单的能量阈值？
//   答: 能量阈值（RMS > threshold）在真实环境中几乎不可用——风扇、空调、
//   键盘敲击都会误触发。WebRTC VAD 用高斯混合模型（GMM）对每帧频谱做
//   语音/噪声分类，6 个特征维度（子带能量），即使在 60dBA 背景噪声下
//   也能把语音和噪声分开。这是 Google 在 WebRTC 项目里喂了大量数据训练
//   出来的，工业界事实标准。
//
//   面试追问："为什么选 20ms 帧？"
//   答: 语音的准稳态假设（短时平稳性）在 20-30ms 成立——声带振动和声道
//   形状在这个窗口内近似不变。10ms 太短（频谱分辨率不够），30ms 太长
//   （瞬态辅音会被模糊）。WebRTC 只支持这三种，20ms 是延迟和精度的最优折中。

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

/// @brief WebRTC VAD 的轻量 RAII 封装
///
/// libfvad 只接受特定采样率 (8k/16k/32k/48k) 和特定帧长 (10/20/30ms)
/// 的 mono PCM，返回 0（静音）或 1（语音）。
class VadEngine {
public:
    struct Config {
        /// 采样率，必须为 8000、16000、32000 或 48000
        int sample_rate = 16000;
        /// 激进程度: 0=最保守(quality), 1=低码率, 2=激进, 3=最激进
        /// 越高越倾向于判为"非语音"，室内安静环境用 2 即可
        int mode = 2;
    };

    VadEngine() = default;

    // 禁止拷贝
    VadEngine(const VadEngine&) = delete;
    VadEngine& operator=(const VadEngine&) = delete;

    ~VadEngine();

    /// 初始化 VAD 引擎
    /// @return true 成功，false 失败（参数不合法或内存不足）
    bool Init(const Config& config);

    /// 检测一帧音频是否为语音
    /// @param frame  PCM 采样（int16_t, mono）
    /// @param len    帧长（样本数），必须等于 10/20/30ms 对应的值
    ///               16000Hz: 160(10ms), 320(20ms), 480(30ms)
    /// @return true=语音, false=静音/噪声
    bool IsVoice(const int16_t* frame, size_t len) noexcept;

    /// 重置 VAD 内部状态（新一轮对话时调用）
    void Reset() noexcept;

    /// 是否已初始化
    bool IsReady() const noexcept { return fvad_ != nullptr; }

private:
    // libfvad 的 opaque handle，避免在头文件暴露 C 类型
    void* fvad_ = nullptr;
};
