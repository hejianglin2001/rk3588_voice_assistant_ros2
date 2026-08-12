// ============================================================================
// 文件: src/asr/sherpa_asr.hpp
// 作用: 语音识别引擎（ASR）的 C++ RAII 封装
// 底层: sherpa-onnx 离线识别器（C API）
// 为什么需要:
//   1. 把 C 风格的 Create/Destroy 变成构造/析构自动管理
//   2. 对外只暴露一个 Recognize() 方法 — 输入音频，输出文字
//   3. 自动处理 int16 → float 转换（sherpa-onnx 要求 float 输入）

#pragma once

#include <memory>
#include <string>
#include <vector>

// ============================================================================
// SherpaASR — 离线语音识别引擎
// ============================================================================
class SherpaASR {
public:
    /// 模型文件路径配置
    struct Config {
        /// 模型目录，包含 encoder.onnx, decoder.onnx, joiner.onnx, tokens.txt
        /// 或者 paraformer 模型目录，包含 model.onnx, tokens.txt
        std::string model_dir;

        /// 采样率（必须和模型训练时一致，通常 16000）
        int32_t sample_rate = 16000;

        /// CPU 推理线程数（RK3588 有 8 核，建议用 4 核留给其他任务）
        int32_t num_threads = 4;

        /// 是否开启调试输出
        bool debug = false;
    };

    SherpaASR() = default;

    // 禁止拷贝
    SherpaASR(const SherpaASR&) = delete;
    SherpaASR& operator=(const SherpaASR&) = delete;

    ~SherpaASR();

    /// 加载模型
    /// @return true 表示成功
    bool Init(const Config& config);

    /// 识别一段 PCM 音频，返回识别文字
    /// @param samples S16_LE 格式的 PCM 采样（int16_t）
    /// @param num_samples 采样个数（= 帧数 × 声道数）
    /// @return 识别出的中文文本（UTF-8），失败返回空字符串
    std::string Recognize(const int16_t* samples, size_t num_samples);

    /// 是否已初始化
    bool IsReady() const { return recognizer_ != nullptr; }

private:
    void* recognizer_ = nullptr;  // SherpaOnnxOfflineRecognizer*
    int32_t sample_rate_ = 16000;
    int32_t num_threads_ = 4;
};
