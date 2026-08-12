// ============================================================================
// 文件: src/llm/rkllm_engine.cpp
// 作用: RKLLMEngine 的具体实现
// 依赖: 瑞芯微 librkllmrt.so（NPU 推理运行时库）
// ============================================================================

#include "llm/rkllm_engine.hpp"

#include <cstdio>
#include <cstring>
#include <iostream>

// ============================================================================
// TokenCallback — 推理结果回调
// 💡 面试考点: 为什么 LLM 用"流式回调"而不是"全部生成完再返回"？
//   答: 用户体验 — 用户看到逐字输出感觉更快（首 token 延迟 vs 总延迟）
//   技术原因 — 不缓存全部结果节省内存，支持提前中断生成
// ============================================================================
int RKLLMEngine::TokenCallback(RKLLMResult* result, void* /*userdata*/, LLMCallState state)
{
    if (state == RKLLM_RUN_FINISH) {
        printf("\n");
    } else if (state == RKLLM_RUN_ERROR) {
        printf("\n[llm] 推理出错\n");
    } else if (state == RKLLM_RUN_NORMAL) {
        printf("%s", result->text);  // 逐 token 打印
    }
    return 0;  // 返回 0 让推理继续
}

// ============================================================================
// ~RKLLMEngine — 析构时自动释放 NPU 资源
// 💡 C 接口封装最重要的原则: 谁 init 谁 destroy，绝不交给调用者手动管理
// ============================================================================
RKLLMEngine::~RKLLMEngine()
{
    if (handle_ != nullptr) {
        rkllm_destroy(handle_);
        handle_ = nullptr;
    }
}

// ============================================================================
// Init — 加载模型到 NPU
// ============================================================================
bool RKLLMEngine::Init(const Config& config)
{
    if (handle_ != nullptr) {
        std::cerr << "[llm] 已经初始化过了" << std::endl;
        return false;
    }

    // ---- 1. 设置模型参数 ----
    RKLLMParam param = rkllm_createDefaultParam();
    param.model_path          = config.model_path.c_str();
    param.max_new_tokens      = config.max_new_tokens;
    param.max_context_len     = config.max_context_len;
    param.skip_special_token  = true;

    // 采样参数（控制生成文本的随机性/创造性）
    param.top_k               = 1;      // Top-K 采样
    param.top_p               = 0.95f;  // Top-P 采样
    param.temperature         = 0.8f;   // 温度：越高越随机
    param.repeat_penalty      = 1.1f;   // 重复惩罚
    param.frequency_penalty   = 0.0f;
    param.presence_penalty    = 0.0f;

    // NPU 扩展参数
    param.extend_param.base_domain_id = 0;
    param.extend_param.embed_flash    = 1;

    // ---- 2. 注册回调 ----
    RKLLMCallback cb = {};
    cb.result_callback = TokenCallback;

    // ---- 3. 初始化（加载模型到 NPU 内存）----
    std::cout << "[llm] 正在加载模型到 NPU..." << std::endl;
    int ret = rkllm_init(&handle_, &param, &cb);
    if (ret != 0) {
        std::cerr << "[llm] 初始化失败, 错误码=" << ret << std::endl;
        return false;
    }

    std::cout << "[llm] 模型加载完成" << std::endl;
    return true;
}

// ============================================================================
// Run — 发送提示词，等待推理完成
// ============================================================================
void RKLLMEngine::Run(const std::string& prompt)
{
    if (handle_ == nullptr) {
        std::cerr << "[llm] 引擎未初始化！" << std::endl;
        return;
    }

    // 准备输入结构体
    RKLLMInput input;
    std::memset(&input, 0, sizeof(input));
    input.input_type   = RKLLM_INPUT_PROMPT;
    input.prompt_input = const_cast<char*>(prompt.c_str());

    // 推理参数：生成模式，不保留历史
    RKLLMInferParam infer_param;
    std::memset(&infer_param, 0, sizeof(infer_param));
    infer_param.mode         = RKLLM_INFER_GENERATE;
    infer_param.keep_history = 0;

    printf("robot: ");
    fflush(stdout);
    rkllm_run(handle_, &input, &infer_param, nullptr);
}

// ============================================================================
// ClearHistory — 清空 KV Cache，对模型来说等于"开始新对话"
// ============================================================================
void RKLLMEngine::ClearHistory()
{
    if (handle_ != nullptr) {
        rkllm_clear_kv_cache(handle_, 1, nullptr, nullptr);
        std::cout << "[llm] 对话历史已清空" << std::endl;
    }
}
