# RK3588 端侧离线语音助手 + 视觉检测

> RK3588 ARM64 板端的**全离线**四节点 ROS2 Pipeline：语音输入 → 断句 → 识别 → 对话 / 视觉检测。
> 全程不上云：ASR（sherpa-onnx）、LLM（RKLLM NPU）、YOLO（RKNN NPU）都在板端跑。

```
麦克风 → VAD 断句 → 语音识别(ASR) → LLM 对话 ──(找 X 时)──→ YOLO 视觉检测
```

**一句话亮点**：在单块 RK3588 上，用三套硬件（NPU / RGA / CPU）协同跑通了「语音 + 大模型 + 视觉」的离线闭环，工程上对标机器人公司的量产标准（LifecycleNode 状态管理、QoS 分级、Action 长任务建模、规则 + LLM 混合意图识别）。

---

## 目录

1. [硬件平台：RK3588 三硬件协同](#1-硬件平台rk3588-三硬件协同)
2. [系统架构](#2-系统架构)
3. [技术栈逐项剖析](#3-技术栈逐项剖析)
4. [设计决策与面试话术](#4-设计决策与面试话术)
5. [端到端数据流走查](#5-端到端数据流走查)
6. [编译 / 运行 / 测试](#6-编译--运行--测试)
7. [已知坑](#7-已知坑)
8. [面试高频 Q&A](#8-面试高频-qa)
9. [未来规划](#9-未来规划)

---

## 1. 硬件平台：RK3588 三硬件协同

RK3588 是瑞芯微旗舰 SoC：8 核 CPU（4×A76 + 4×A55）、**3 核 NPU（6 TOPS）**、Mali GPU、**RGA 2D 硬件引擎**。

本项目的一条检测链路，三种硬件各干各的、互不浪费：

| 步骤 | 硬件 | 代码 | 为什么是它 |
|------|------|------|-----------|
| UYVY→RGB + 缩放到 640×640 | **RGA** | `rga_uyvy_to_rgb()` | 像素搬运/格式转换是纯数据搬运，RGA 硬件 1ms 干完，CPU 要几十 ms |
| YOLO 模型前向（卷积） | **NPU** | `rknn_run()` | 卷积矩阵乘是 NPU 的看家本事，6 TOPS 秒出结果 |
| NMS 后处理（遍历 80×8400） | **CPU** | `postprocess()` | 非规则逻辑（排序、IoU、分支），NPU 不擅长，CPU 灵活 |

> 💡 **面试考点**：为什么后处理不也放 NPU？NMS 是数据相关的串行逻辑（排序 + 条件抑制），NPU 适合规则的张量运算；硬塞 NPU 反而要反复搬运中间结果，更慢。**异构计算的本质是「把每类计算交给最擅长的硬件」**，不是什么都往 NPU 堆。

---

## 2. 系统架构

```
        Empty               AudioChunk              String               String
 /voice_trigger      /utterance_audio       /recognized_text       /llm_response
      │  RELIABLE          BEST_EFFORT            RELIABLE              RELIABLE
      ▼                    ▼                      ▼                     ▼
┌─────────────┐   ┌──────────────┐   ┌───────────────┐   ┌──────────────────┐
│ audio_vad   │──→│   asr_node   │──→│    llm_node   │   │    yolo_node     │
│   _node     │   │              │   │               │   │                  │
│ [Lifecycle] │   │ [Lifecycle]  │   │  [Lifecycle]  │   │   [Lifecycle]    │
│ mic + VAD   │   │ sherpa-onnx  │   │  RKLLM NPU    │   │  RKNN NPU + RGA  │
└─────────────┘   └──────────────┘   └──────┬────────┘   └────────▲─────────┘
                                            │ Action  /yolo_detect │
                                            │ client  ─────────────┘
                                            │           YoloDetect
                                            ▼
                                      相机 /dev/video11 (MIPI UYVY)
```

| 节点 | 订阅 | 发布 | Service | Action | 硬件/模型 |
|------|------|------|---------|--------|----------|
| `audio_vad_node` | `/voice_trigger` | `/utterance_audio` | `/vad_state` | — | miniaudio + libfvad（ES8388 声卡） |
| `asr_node` | `/utterance_audio` | `/recognized_text` | — | — | sherpa-onnx（zh-small int8） |
| `llm_node` | `/recognized_text`, `/text_input` | `/llm_response` | `/reset_context` | `/yolo_detect` (client) | RKLLM NPU（Qwen3.5-0.8B W8A8） |
| `yolo_node` | — | — | — | `/yolo_detect` (server) | RKNN NPU（yolo26n）+ MIPI 相机 + RGA |

---

## 3. 技术栈逐项剖析

### 3.1 ROS2 LifecycleNode（状态管理）

四节点全部继承 `rclcpp_lifecycle::LifecycleNode`，用**显式状态机**管理硬件资源，而不是在构造函数里一把梭。

状态机：`Unconfigured → Inactive → Active`（还有 `Finalized`）。

| 回调 | audio_vad_node | asr_node | llm_node | yolo_node |
|------|---------------|----------|----------|-----------|
| `on_configure` | 打开麦克风 | 加载 ASR 模型 | 加载 LLM 模型（**最慢 ~10s**） | 加载 YOLO 模型 + 开相机 |
| `on_activate` | 订阅 /voice_trigger | 订阅 /utterance_audio | 订阅 /recognized_text | 激活 action server |
| `on_deactivate` | 取消订阅 | 取消订阅 | 取消订阅 | — |
| `on_cleanup` | 释放 mic | 释放模型内存 | 释放 NPU 句柄 | 关相机 + 释放模型 |

launch 文件用 `TimerAction` 按依赖错峰启动：LLM（最慢）先 configure → 12s 后 activate → ASR + YOLO 13s configure / 28s activate → audio_vad 最后 29s 起。

> 💡 **面试话术**：「用 LifecycleNode 管理 NPU/麦克风等硬件资源，保证模型加载完成前不会收到推理请求——否则 plain Node 启动时 LLM 还在加载，ASR 消息先到，要么空指针要么超时。加载耗时 LLM ~10s、ASR ~15s、YOLO ~1s，所以 launch 里按最慢的 LLM 先起、错峰 transition。」

### 3.2 ROS2 QoS（服务质量分级）

不同数据流对可靠性和延迟的要求不同，不能一刀切：

| Topic | Reliability | Durability | Depth | 理由 |
|-------|------------|------------|-------|------|
| `/voice_trigger` | Reliable | **Transient Local** | 1 | 触发信号不能丢；transient_local 让晚订阅者也能拿到最近一条 |
| `/utterance_audio` | **Best Effort** | Volatile | 5 | 音频帧丢了不影响 ASR 语义，省掉 DDS ACK 换低延迟 |
| `/recognized_text` | Reliable | Volatile | 10 | 文本语义不能丢 |
| `/llm_response` | Reliable | Volatile | 10 | 同上 |

> 💡 **面试考点**：`Best Effort` 和 `Reliable` 是**不兼容**的两组 QoS，发布方和订阅方必须对齐，否则 DDS 会静默丢消息（本项目中 audio 若用默认 `rmw_qos_profile_sensor_data`=reliable 去订阅 best_effort 发布，音频一条都收不到，日志只刷 `RELIABILITY_QOS_POLICY` 警告）。这是踩过的真坑。

### 3.3 ROS2 Action（长任务建模）

视觉检测是「持续 N 秒 + 周期反馈 + 可取消」的长任务，**Topic 建模不了 cancel，Service 是同步一次性**，只有 Action 三者兼备。

`/yolo_detect`（`YoloDetect.action`）三部分：

| 部分 | 字段 | 说明 |
|------|------|------|
| Goal | `target_class`, `duration_sec` | 找什么、测多久 |
| Result | `detected`, `best_class`, `best_confidence`, `bbox_x/y/w/h` | 最终结果 |
| Feedback | `elapsed_sec`, `all_objects`, `progress` | 周期上报画面里所有物体 |

client 端用 `async_send_goal` 带三个回调：`goal_response_callback`（server 是否接单）、`result_callback`（最终结果）、可选 `feedback_callback`（实时进度）。server 端 `execute()` 里循环抓帧 → 推理 → `publish_feedback`，被 cancel 时 `gh->canceled(result)`。

> 💡 **面试话术**：「Action 是 ROS2 对『有生命周期、可反馈、可取消的异步任务』的一等公民建模。Topic 是单向数据流，Service 是同步一问一答，检测这种『开始—跑 N 秒—随时报进度—可取消』的语义，只有 Action 能完整表达。」

### 3.4 ROS2 Service vs Topic（接口选型）

- **Service**（命令式、一次一问一答）：`/reset_context`（清空 LLM 上下文）、`/vad_state`（查麦克风状态）
- **Topic**（数据流、持续发布）：音频、文本、识别结果
- **Action**（长任务）：视觉检测

> 💡 原则：「Service 用于命令，Topic 用于数据流，Action 用于长任务——不混用。」

### 3.5 miniaudio 音频采集（实时线程 + 无锁 RingBuffer）

采集 ES8388 声卡（`plughw:2,0`，44.1kHz 双声道）。核心设计：**DataCallback 只做 memcpy**。

```cpp
// 面试考点：为什么 callback 里只 memcpy？
// ma_device 的 data_callback 跑在 ALSA 高优先级实时线程（近似中断上下文），
// 做 malloc / 加锁 / 文件 IO 都会阻塞音频 DMA → overrun（丢帧）/ underrun（爆音）。
// 正确做法：无锁写 RingBuffer，业务线程异步消费。
```

音频数据用 `RingBuffer<int16_t>`（SPSC 无锁队列）缓冲，业务线程（ROS 回调）在 `ReadSamples` 里消费。

> 💡 **面试考点**：生产者-消费者解耦 + 无锁。音频是硬实时流，回调里不能做任何可能阻塞的事；用无锁环形缓冲把「中断上下文」和「业务逻辑」隔开。

### 3.6 WebRTC VAD 断句（状态机）

用 libfvad（WebRTC VAD 的 C 封装），**mode 2**，帧长 **320 样本 = 20ms @ 16kHz**。

断句状态机（`audio_vad_node.cpp` 的核心逻辑）：

```
kPending ──连续 3 帧有声(kMinSpeech=3, 60ms)──→ kSpeaking
kSpeaking ──连续 100 帧静音(kSilence=100, 2s)──→ 断句完成(done)
```

- **kMinSpeech=3**：3 帧连续有声才确认「开始说话」，过滤瞬时噪声误触发。
- **kSilence=100**：连续 2 秒静音判「说完」，这是中文语音的典型句间停顿。
- VAD 模式 0（最保守，容易判有声）→ 3（最激进，容易判静音），当前 mode 2 是折中。

> 💡 **面试考点**：VAD 是**能量/频谱特征 + 状态机**，不是简单的「响度阈值」。状态机加滞回（3 帧进入、100 帧退出）是为了抗抖动，避免一句话中间被切碎。
>
> ⚠️ **遗留问题**：板端风扇/环境底噪被 VAD 判成语音，导致 2s 静音触发不了，实际会录满 10s 才发布。修法两条：mode 2→3（更激进判静音），或调低 `Capture Digital Volume`（numid 25，当前 192）。

### 3.7 sherpa-onnx ASR（离线识别）

zh-small int8 量化模型（encoder / decoder / joiner 三件套），16kHz 单声道，4 线程 CPU 推理（ASR 不吃 NPU，CPU 够）。

- 优先找 `.int8.onnx` 量化版（更小更快），否则回退 FP32。
- 封装成 `SherpaASR::Recognize(samples, n) -> string`，输入 PCM 输出汉字。

> 💡 **面试考点**：ASR 是「编码器-解码器 + 连接时序分类（transducer/CTC）」结构，int8 量化在不明显掉点的情况下把模型体积和推理耗时砍半，是端侧部署的标配手段。

### 3.8 RKLLM 大模型推理（NPU）

Qwen3.5-0.8B，**W8A8 量化**（权重 8bit + 激活 8bit），跑在 NPU。采样参数：

```cpp
param.top_k = 1;            // Top-K 采样（greedy 候选）
param.top_p = 0.95f;        // 核采样
param.temperature = 0.8f;   // 越高越随机
param.repeat_penalty = 1.1f;// 重复惩罚
infer_param.keep_history = 0;  // 不保留 KV cache（每轮独立）
```

**流式 token 回调**：`rkllm_run` 不一次吐完，而是每生成一个 token 回调一次 `TokenCallback`，逐字打印。

> 💡 **面试考点 1**：为什么用流式回调而不是全部生成完再返回？**首 token 延迟**——用户看到第一个字更快，体验上「更快」；且不用缓存全部结果，支持提前中断。
> 💡 **面试考点 2**：`keep_history=0` 意味着每轮无上下文，所以 prompt 必须**自包含**（把任务要求写全）。代价是省 KV cache 内存、避免长对话显存膨胀；代价是记不住多轮对话。

### 3.9 RKNN YOLO 检测（NPU）

yolo26n，**split 双输出**：`bbox[4,8400]` + `score[80,8400]`（8400 = 3 个检测头网格总数，80 = COCO 类数）。输入 640×640 RGB。

后处理全在 CPU：对 8400 个 anchor 求每个的 argmax 类别置信度 → 过滤 `conf_threshold=0.25` → **NMS（IoU 阈值 0.45）** → 输出 Detection 列表。

> 💡 **面试考点**：`split` 是把 YOLO 的检测头拆成 bbox 和 score 两个输出张量，方便 RKNN 定点量化；NMS 是「非极大值抑制」，把同一目标上重叠的多个框合并成一个，核心指标是 IoU（交并比）。

### 3.10 RGA 硬件转换（不依赖 OpenCV）

`rga_uyvy_to_rgb()`：UYVY(1920×1080) → RGB(640×640)，用 RGA 的 `imresize` 一步完成**格式转换 + 缩放**，1ms 级。

> 💡 **面试考点**：为什么不用 OpenCV？① OpenCV 在 ARM 上吃 CPU 软解；② RGA 是硬件 DMA 搬运 + 硬件缩放，快一个数量级；③ 交叉编译 + 板端部署省一个重依赖。**嵌入式视觉里「预处理走硬件引擎」是标配**。

### 3.11 意图识别：规则（正则）+ LLM 抽词（混合）

这是本项目最有「设计味道」的地方。用户的自然语言要路由到「对话」还是「检测」，并抽出目标词：

```
用户: "帮我检测一下一个人你能不能检测到一个人"
   │
   ├─ 第 1 层：正则只判意图  isDetectIntent("找|寻找|检测|find") → 命中「检测」
   │
   ├─ 第 2 层：LLM 抽词（槽位填充）RunSync("从这句话提取物体名，只输出物体名") → "person"
   │
   └─ zhToEn 归一化 → 发送 YoloDetect goal(target_class=person)
```

**为什么正则不能抽词**：`std::regex` 是字节级的、不懂中文分词。「检测一下一个人你能不能检测到一个人」里哪个是目标（人）、哪个是虚词（一下/一个）、哪个是尾巴（你能不能…），正则抓 `\S+` 会把整句吞掉 → YOLO 拿垃圾串去匹配 class，必然未检测到。**这是踩过的真坑**，改成了「正则判意图 + LLM 抽词」两层。

> 💡 **面试话术**：「意图识别我用规则 + LLM 混合：**确定性意图（有没有"找/检测"）用正则，快速零成本；开放性的槽位填充（具体找什么）交给 LLM**。纯规则搞不定中文分词的长尾，纯 LLM 又慢、输出格式不可控。分层是机器人意图识别的标准打法——类似 Function Calling 的简化版：LLM 输出结构化参数，我拿去调工具（YOLO）。」

配套 `RKLLMEngine::RunSync(prompt) -> string`：跟 `Run`（只流式打印）的区别是 token 同时收进 buffer 返回，供抽词这种「要拿结果」的场景用。

### 3.12 检测结果落盘（stb_image_write）

板端无显示器，检测到目标时把当帧画上**绿色 bbox + 左上角置信度数字**，用 `stbi_write_jpg` 存 JPG 到 `yolo_out/`（`save_dir` 参数可配）。

- 选 stb 单头文件（public domain），自带 JPG 编码器，**板端零外部依赖**，不用装 libjpeg/OpenCV。
- 只在「第一次命中目标的那帧」保存一次，避免 5 秒刷 50 张图撑爆磁盘。
- 文件名 `detect_<类别>_<置信度>.jpg`，如 `detect_person_37.jpg`。

---

## 4. 设计决策与面试话术

| 决策 | 为什么 | 面试话术 |
|------|--------|---------|
| LifecycleNode 而非 plain Node | 模型加载慢，要状态机保证就绪 | 「硬件资源生命周期显式管理」 |
| 音频 Best Effort / 文本 Reliable | 音频可丢帧、文本不能丢 | 「QoS 分级，按数据语义选可靠性」 |
| Action 做检测 | 长任务 + 反馈 + 可取消 | 「Topic 管不了 cancel，Service 是同步的」 |
| RGA 预处理 | 硬件转换比 CPU 快一个数量级 | 「预处理走硬件引擎」 |
| 意图：正则 + LLM 分层 | 规则快且确定，LLM 会抽词 | 「确定性意图规则、开放性槽位 LLM」 |
| keep_history=0 | 省 KV cache，单轮足够 | 「无状态推理，prompt 自包含」 |
| 流式 token 回调 | 首 token 延迟低 | 「体验优先，逐字输出」 |
| NPU 串行（LLM/YOLO 不同时跑） | 单发单收业务天然错开 | 「当前串行，未来并发用 core mask 隔离」 |
| 接口独立 `*_interfaces` 包 | 接口被多包共享，纯 rosidl 无 runtime | 「接口与实现解耦」 |
| engine/ 与 nodes/ 分离 | 硬件封装与 ROS 胶水层解耦 | 「引擎层不依赖 ROS，可独立测试/换后端」 |

---

## 5. 端到端数据流走查

**场景 A：语音对话**

```
发 /voice_trigger (Empty)
  → audio_vad_node 录音 + VAD 断句（或 10s 满）
  → 重采样 44.1k 双声道 → 16k 单声道 int16[]  → AudioChunk 发布
  → asr_node sherpa-onnx 识别 → String 发布
  → llm_node 正则没命中「找/检测」→ Run() 流式对话 → 回复
```

**场景 B：语音触发视觉检测**

```
...同上直到 llm_node
  → 正则命中「检测」→ RunSync() 抽词 "person"
  → zhToEn(person)=person → async_send_goal(YoloDetect{target=person, duration=5})
  → yolo_node 相机抓帧 UYVY → RGA 转 RGB 640×640 → NPU 推理
  → NMS → 匹配 person → 画框 + 存 JPG → 返回 result(detected=true, conf=0.74, bbox)
  → llm_node result_callback 模板拼回复 → 发布 /llm_response
```

**场景 C：直接调 YOLO（绕过 LLM）**

```
ros2 action send_goal /yolo_detect ... "{target_class: person, duration_sec: 4}"
```

---

## 6. 编译 / 运行 / 测试

### 板端编译

```bash
cd ~/code/rk3588_voice_assistant_ros2
source /opt/ros/humble/setup.bash
./build.sh          # = rm -rf build install log && colcon build
```

标准 `src/` 布局，colcon 默认发现 `src/` 下两个包（`*_interfaces` + 主包）。

### 运行

```bash
source /opt/ros/humble/setup.bash
source ~/code/rk3588_voice_assistant_ros2/install/setup.bash
./run.sh            # 初始化 ES8388 mic 路由 + 注入 LD_LIBRARY_PATH + ros2 launch
```

`run.sh` 里三句 amixer 初始化麦克风（跟老项目 llm_demo 一致）：

```bash
amixer -c 2 sset 'Differential Mux' 'Line 2'   # PGA 路由指向板载差分主麦
amixer -c 2 cset numid=42 on                    # Main Mic Switch = on
amixer -c 2 cset numid=43 off                   # Headset Mic Switch = off
```

启动后等 ~30s（LLM 最慢），`ros2 lifecycle list` 确认四节点 `active`。

### 测试

```bash
# 生命周期
ros2 lifecycle list /llm_node /asr_node /yolo_node /audio_vad_node

# 文字 → 对话
ros2 topic pub --once /text_input std_msgs/msg/String "{data: '你好'}"

# 文字 → 检测（长句也能抽对词）
ros2 topic pub --once /text_input std_msgs/msg/String "{data: '帮我检测一下一个人'}"

# 直接调 YOLO（绕过 LLM）
ros2 action send_goal /yolo_detect rk3588_voice_assistant_interfaces/action/YoloDetect \
  "{target_class: 'person', duration_sec: 4}"

# service
ros2 service call /reset_context std_srvs/srv/Empty
ros2 service call /vad_state std_srvs/srv/Trigger

# 语音全链路（触发 VAD 录音，然后说话）
ros2 topic pub --once /voice_trigger std_msgs/msg/Empty "{}"
```

---

## 7. 已知坑

### librknnrt.so 版本
`Invalid RKNN model version 6` → 第三方目录里的 `librknnrt.so` 太旧（v1.4.0）。用板端系统 v2.3.2 覆盖：
```bash
cp /usr/lib/librknnrt.so third_party/rknn/aarch64/librknnrt.so
```

### LLM 与 YOLO 是两套 NPU runtime
`librkllmrt.so`（RKLLM）跑大模型，`librknnrt.so`（RKNN）跑常规算子，不能混用。yolo26n 是 RKNN 模型 v6，必须 `librknnrt.so` v2.3.2。

### ARM64 std::regex crash
params YAML 不能用 `/**` 通配节点名（ROS2 Humble 内部转 regex，ARM libstdc++ 有 bug）。用显式节点名 `/llm_node:`。

### QoS 不匹配 → 静默丢消息
音频发布 best_effort、订阅若用 reliable，DDS 直接丢消息。务必两端对齐（本项目已统一 best_effort + volatile）。

### 麦克风 busy
`plughw:2,0` 被占用报 `Device or resource busy`。`fuser /dev/snd/*` 找进程或 reboot。

### 相机权限
`/dev/video11` 是 `root:video`，运行用户需在 `video` 组。adbd push 的文件是 root 属主，编译前 `chown -R topeet:topeet`。

### VAD 断句录满 10s
底噪被判语音 → 2s 静音触发不了 → 录满 10s。见 3.6 的修法。

---

## 8. 面试高频 Q&A

**Q1：为什么音频用 Best Effort，不怕丢吗？**
A：ASR 对「丢几帧」不敏感（几十 ms 的音频缺失不影响整句语义），但 Best Effort 省掉 DDS 的 ACK 往返，降延迟。文本语义完整才必须 Reliable。这是「按数据语义选 QoS」。

**Q2：LLM 和 YOLO 都在 NPU 上，会不会抢？**
A：当前业务是单发单收，LLM 推理和 YOLO 推理天然串行（说「找苹果」只跑 YOLO，检测完才让 LLM 生成总结），不同时占 NPU。真要并发（边检测边播报），给 RKLLM 设 NPU core mask，留一个 core 给 YOLO。

**Q3：为什么不直接用 OpenCV 做图像预处理？**
A：OpenCV 是 CPU 软解，RGA 是硬件 DMA + 硬件缩放，快一个数量级，还省一个重依赖。嵌入式视觉预处理走硬件引擎是标配。

**Q4：意图识别为什么正则 + LLM 混合，不纯用 LLM？**
A：纯 LLM 每次都要跑一次推理（慢 + 输出格式不可控），纯正则搞不定中文分词（长句抓不干净）。分层：确定性意图用正则（零成本），开放性槽位用 LLM（它擅长）。

**Q5：keep_history=0 有什么取舍？**
A：省 KV cache 内存、避免长对话显存膨胀，但每轮无上下文，prompt 必须自包含。当前单轮问答够用，多轮对话需求起来再开。

**Q6：miniaudio 回调为什么只做 memcpy？**
A：回调跑在 ALSA 高优先级实时线程，做 malloc/加锁/IO 会阻塞音频 DMA，导致丢帧或爆音。无锁写 RingBuffer，业务线程异步消费。

**Q7：Action 和 Service 怎么选？**
A：一次性同步 → Service；单向数据流 → Topic；有生命周期、可反馈、可取消的长任务 → Action。检测是第三类。

---

## 9. 未来规划

| 项目 | 为什么暂不做 |
|------|-------------|
| 目标跟踪（ByteTrack/DeepSORT） | 当前是逐帧检测取最高置信度，tracking 需时序关联，独立 feature |
| TTS（piper-tts） | 需额外模型 + C API 封装，LLM 文本输出已满足 demo |
| diagnostic_updater | 单节点健康 service 已够，批量监控再加 |
| ComposableNode | AudioChunk 零拷贝收益小，相机独立进程更稳 |
| lifecycle_manager 节点 | 当前 TimerAction + ExecuteProcess 已可靠 |
| 中文类名全覆盖 | 当前映射 23 个常用词 + LLM 抽词兜底，全量 80 类再补 |
| NPU core mask 隔离 | 等「边检测边对话」并发需求出现再加 |
