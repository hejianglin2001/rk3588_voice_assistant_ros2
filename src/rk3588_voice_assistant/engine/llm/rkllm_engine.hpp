// ============================================================================
// 文件: src/llm/rkllm_engine.hpp
// 作用: 把 RKLLM（瑞芯微 NPU 大模型推理）的 C 接口封装成 C++ RAII 类
// 为什么需要:
//   1. 自动管理生命周期 — 构造时初始化，析构时自动释放，不会忘记 destroy
//   2. 对外只暴露 3 个方法 — 使用简单，调用者不需要关心 NPU 参数细节
//   3. 后续可以替换为其他 LLM 后端（如 llama.cpp）而不改 main()

#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "rkllm.h"

// ============================================================================
// RKLLMEngine — 大模型推理引擎（RAII 封装）
// ============================================================================
class RKLLMEngine {
public:
    /// 初始化参数（只填必要的，其他用默认值）
    struct Config {
        std::string model_path;
        int max_new_tokens = 128;         // 单次最多生成多少 token
        int max_context_len = 512;        // 上下文窗口大小
    };

    RKLLMEngine() = default;

    // 禁止拷贝
    RKLLMEngine(const RKLLMEngine&) = delete;
    RKLLMEngine& operator=(const RKLLMEngine&) = delete;

    ~RKLLMEngine();

    /// 加载模型并初始化 NPU
    /// 返回 true 表示成功，false 表示失败
    bool Init(const Config& config);

    /// 发送一句提示词，等待模型生成回复（阻塞调用）
    /// 结果通过内部 callback 实时打印到 stdout
    void Run(const std::string& prompt);

    /// 清空对话历史，开始新一轮对话
    void ClearHistory();

    /// 是否已初始化
    bool IsReady() const { return handle_ != nullptr; }

private:
    /// C 库的回调函数 — 模型每生成一个 token 会调用一次
    /// 这里把 token 文本直接打印到屏幕上
    /// 返回值: 0 表示继续生成，非 0 表示中断推理
    static int TokenCallback(RKLLMResult* result, void* userdata, LLMCallState state);

    LLMHandle handle_ = nullptr;  // RKLLM C 接口的句柄
};
