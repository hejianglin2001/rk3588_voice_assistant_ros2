// ============================================================================
// 文件: src/asr/sherpa_asr.cpp
// 作用: SherpaASR 的具体实现
// 依赖: libsherpa-onnx.so（需要下载到 third_party/sherpa-onnx/lib/）
// ============================================================================

#include "asr/sherpa_asr.hpp"

#include <cstring>
#include <glob.h>    // POSIX 文件名匹配
#include <iostream>

// sherpa-onnx C API
#include "sherpa-onnx/c-api/c-api.h"

// ---- 小工具: 在目录下按模式找文件 ----
// find_file("models/", "*encoder*.onnx") → "models/encoder-epoch-12-avg-4.onnx"
// 优先用 .int8.onnx 量化版（更小更快），否则用 FP32
static std::string find_model_file(const std::string& dir,
                                    const std::string& glob_pattern) {
    std::string full = dir + "/" + glob_pattern;
    glob_t g{};
    int ret = glob(full.c_str(), 0, nullptr, &g);
    std::string best;
    for (size_t i = 0; i < g.gl_pathc; ++i) {
        std::string path(g.gl_pathv[i]);
        // 优先 INT8 量化版
        if (path.find(".int8.") != std::string::npos) {
            best = path;
            break;
        }
        if (best.empty()) best = path;  // 保留第一个作为备选
    }
    globfree(&g);
    return best;
}

// sherpa-onnx 接受 float 输入，我们需要把 S16_LE → float
static std::vector<float> s16_to_float(const int16_t* samples, size_t n)
{
    std::vector<float> out(n);
    for (size_t i = 0; i < n; ++i) {
        out[i] = static_cast<float>(samples[i]) / 32768.0f;
    }
    return out;
}

// ============================================================================
// RAII 析构
// ============================================================================
SherpaASR::~SherpaASR()
{
    if (recognizer_ != nullptr) {
        SherpaOnnxDestroyOfflineRecognizer(
            static_cast<SherpaOnnxOfflineRecognizer*>(recognizer_));
        recognizer_ = nullptr;
    }
}

// ============================================================================
// Init — 加载 ASR 模型
// ============================================================================
bool SherpaASR::Init(const Config& config)
{
    if (recognizer_ != nullptr) {
        std::cerr << "[asr] 已经初始化过了" << std::endl;
        return false;
    }

    sample_rate_ = config.sample_rate;
    num_threads_ = config.num_threads;

    // ---- 构造 sherpa-onnx 配置 ----
    // 用大缓冲区零初始化，避免结构体大小与 .so 不匹配导致段错误
    // 所有未设置的字段默认为 0/NULL → sherpa-onnx 使用默认值
    alignas(64) static char cfg_buf[4096] = {};
    auto* sherpa_cfg = reinterpret_cast<SherpaOnnxOfflineRecognizerConfig*>(cfg_buf);

    // 特征配置
    sherpa_cfg->feat_config.sample_rate = sample_rate_;
    sherpa_cfg->feat_config.feature_dim  = 80;

    // 模型配置
    auto& model = sherpa_cfg->model_config;

    // 自动匹配模型文件名（兼容 encoder.onnx 或 encoder-epoch-12-avg-4.onnx 等）
    std::string enc = find_model_file(config.model_dir, "*encoder*.onnx");
    std::string dec = find_model_file(config.model_dir, "*decoder*.onnx");
    std::string joi = find_model_file(config.model_dir, "*joiner*.onnx");
    std::string tok = find_model_file(config.model_dir, "tokens.txt");

    if (enc.empty() || dec.empty() || joi.empty() || tok.empty()) {
        std::cerr << "[asr] 模型文件不完整！在 " << config.model_dir << " 下需要找到:"
                  << "\n  encoder*.onnx: " << (enc.empty() ? "❌" : "✅")
                  << "\n  decoder*.onnx: " << (dec.empty() ? "❌" : "✅")
                  << "\n  joiner*.onnx:  " << (joi.empty() ? "❌" : "✅")
                  << "\n  tokens.txt:   " << (tok.empty() ? "❌" : "✅")
                  << std::endl;
        return false;
    }

    std::cout << "[asr] encoder: " << enc << std::endl;
    std::cout << "[asr] decoder: " << dec << std::endl;
    std::cout << "[asr] joiner:  " << joi << std::endl;

    model.transducer.encoder = enc.c_str();
    model.transducer.decoder = dec.c_str();
    model.transducer.joiner  = joi.c_str();
    model.tokens             = tok.c_str();
    model.num_threads        = num_threads_;
    model.debug              = config.debug ? 1 : 0;
    // provider 保持 nullptr = 默认 CPU 推理

    // ---- 创建识别器 ----
    std::cout << "[asr] 正在加载 ASR 模型..." << std::endl;
    auto* rec = SherpaOnnxCreateOfflineRecognizer(sherpa_cfg);
    if (rec == nullptr) {
        std::cerr << "[asr] 模型加载失败！请检查 model_dir 路径: "
                  << config.model_dir << std::endl;
        return false;
    }

    recognizer_ = const_cast<void*>(static_cast<const void*>(rec));
    std::cout << "[asr] 模型加载完成" << std::endl;
    return true;
}

// ============================================================================
// Recognize — 音频 → 文字
// ============================================================================
std::string SherpaASR::Recognize(const int16_t* samples, size_t num_samples)
{
    if (recognizer_ == nullptr) {
        std::cerr << "[asr] 引擎未初始化！" << std::endl;
        return {};
    }

    auto* rec = static_cast<SherpaOnnxOfflineRecognizer*>(recognizer_);

    // ---- 1. 创建识别流 ----
    auto* stream = SherpaOnnxCreateOfflineStream(rec);
    if (stream == nullptr) {
        std::cerr << "[asr] 创建识别流失败" << std::endl;
        return {};
    }

    // ---- 2. int16 → float，喂入模型 ----
    auto float_samples = s16_to_float(samples, num_samples);
    SherpaOnnxAcceptWaveformOffline(stream, sample_rate_,
                                     float_samples.data(),
                                     static_cast<int32_t>(float_samples.size()));

    // ---- 3. 解码 ----
    SherpaOnnxDecodeOfflineStream(rec, stream);

    // ---- 4. 取结果 ----
    const auto* result = SherpaOnnxGetOfflineStreamResult(stream);
    std::string text;
    if (result != nullptr && result->text != nullptr) {
        text = result->text;
        std::cout << "[asr] 识别结果: " << text << std::endl;
    }

    // ---- 5. 清理 ----
    SherpaOnnxDestroyOfflineStream(stream);
    return text;
}
