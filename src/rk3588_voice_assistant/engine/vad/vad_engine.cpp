// ============================================================================
// 文件: src/vad/vad_engine.cpp
// 作用: VadEngine 的实现 — 薄封裝 libfvad
// ============================================================================

#include "vad/vad_engine.hpp"

#include <cstring>
#include <iostream>

// libfvad C API — 头文件在 third_party/libfvad/include/
#include "fvad.h"

// ============================================================================
// ~VadEngine
// ============================================================================
VadEngine::~VadEngine()
{
    if (fvad_ != nullptr) {
        fvad_free(static_cast<Fvad*>(fvad_));
        fvad_ = nullptr;
    }
}

// ============================================================================
// Init
// ============================================================================
bool VadEngine::Init(const Config& config)
{
    if (fvad_ != nullptr) {
        std::cerr << "[vad] 已经初始化过了" << std::endl;
        return false;
    }

    auto* inst = fvad_new();
    if (inst == nullptr) {
        std::cerr << "[vad] fvad_new() 失败" << std::endl;
        return false;
    }

    if (fvad_set_sample_rate(inst, config.sample_rate) < 0) {
        std::cerr << "[vad] 不支持的采样率: " << config.sample_rate << std::endl;
        fvad_free(inst);
        return false;
    }

    if (fvad_set_mode(inst, config.mode) < 0) {
        std::cerr << "[vad] 不支持的模式: " << config.mode << std::endl;
        fvad_free(inst);
        return false;
    }

    fvad_ = inst;
    std::cout << "[vad] 初始化完成, rate=" << config.sample_rate
              << "Hz, mode=" << config.mode << std::endl;
    return true;
}

// ============================================================================
// IsVoice — 调 libfvad 做 GMM 分类
// ============================================================================
bool VadEngine::IsVoice(const int16_t* frame, size_t len) noexcept
{
    if (fvad_ == nullptr) return false;

    int result = fvad_process(static_cast<Fvad*>(fvad_), frame, len);
    // result: 1=voice, 0=silence, -1=error
    return result == 1;
}

// ============================================================================
// Reset
// ============================================================================
void VadEngine::Reset() noexcept
{
    if (fvad_ != nullptr) {
        fvad_reset(static_cast<Fvad*>(fvad_));
    }
}
