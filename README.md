# RK3588 端侧离线语音助手 + 视觉检测

> RK3588 ARM64 板端的**全离线**五节点 ROS2 Pipeline：语音输入 → 断句 → 识别 → 意图理解 → 决策路由 → 对话 / 视觉检测。
> 全程不上云：ASR（sherpa-onnx）、LLM（RKLLM NPU）、YOLO（RKNN NPU）都在板端跑。

```
麦克风 → VAD 断句 → 语音识别(ASR) → LLM 对话 ──(找 X 时)──→ YOLO 视觉检测
```

**一句话亮点**：在单块 RK3588 上，用三套硬件（NPU / RGA / CPU）协同跑通了「语音 + 大模型 + 视觉」的离线闭环，工程上对标机器人公司的量产标准（LifecycleNode 状态管理、QoS 分级、Action 长任务建模、规则 + LLM 混合意图识别），并打通了**跨机 DDS 单播 + PC 端 RViz2 实时画框**。

---

## 目录

0. [零基础导读（先看这个）](#0-零基础导读先看这个)
1. [硬件平台：RK3588 三硬件协同](#1-硬件平台rk3588-三硬件协同)
2. [系统架构](#2-系统架构)
3. [技术栈逐项剖析](#3-技术栈逐项剖析)

5. [端到端数据流走查](#5-端到端数据流走查)
6. [编译 / 运行 / 测试](#6-编译--运行--测试)
7. [已知坑](#7-已知坑)


---

## 0. 零基础导读（先看这个）

> 如果你第一次接触 ROS2 / 嵌入式 / 大模型，先读这一节。后面 1–9 节都是这节的展开和深化。

### 0.1 这个项目到底做了什么

一句话：**在单块 RK3588 板子上，做了一个能「听懂你说话 → 跟你聊天 → 还能帮你在摄像头画面里找东西」的语音助手，全程不联网。**

类比：它像 Siri/小爱同学，但有三个区别——① 不依赖云端，所有模型在板子上离线跑；② 多了「眼睛」（摄像头 + YOLO 目标检测）；③ 用 ROS2 这种机器人行业标准框架搭，方便后续接机械臂、导航等。

### 0.2 先搞懂 15 个核心词（大白话版）

#### 机器人框架类

| 词 | 大白话 |
|----|--------|
| **ROS2** | 机器人软件的操作系统：把程序拆成很多小进程（节点），统一管理它们怎么通信、怎么启动、怎么互相发现 |
| **节点 Node** | 一个独立的程序进程，只干一件事（录音的只管录音、识别的只管识别） |
| **Topic（话题）** | 广播频道：发布者往频道丢消息，订阅者从频道收。单向、持续、一对多，像微信群公告 |
| **Service（服务）** | 一问一答：客户端问、服务端答，同步的，像打电话 |
| **Action（动作）** | 长任务：有开始、有进度反馈、可中途取消，像点外卖（下单→接单→骑手位置→送达/取消） |
| **DDS** | ROS2 底层的通信协议，负责把消息从 A 机器送到 B 机器 |
| **组播 / 单播** | 组播=大喇叭广播（所有人都听，但有些网络会屏蔽）；单播=打电话（点对点，绕开屏蔽） |
| **LifecycleNode** | 带「状态机」的节点：Unconfigured→Inactive→Active，像开机自检→待机→运行，保证硬件/模型准备好才接活 |
| **QoS** | 消息的「投递保证」：Reliable=保证送到（丢可重发），Best Effort=尽力发（丢了拉倒），各取所需 |

#### 硬件类

| 词 | 大白话 |
|----|--------|
| **CPU** | 通用芯片，啥都能算，但神经网络这种海量矩阵运算它算得慢 |
| **NPU** | 专门算神经网络的芯片，跑卷积/矩阵飞快（RK3588 有 6 TOPS），但只会算特定运算 |
| **RGA** | 专门搬像素、缩放/旋转图像的硬件，干「1920×1080 转 640×640」比 CPU 快一个数量级 |
| **量化（W8A8 / int8）** | 把模型参数从 32bit 浮点压成 8bit 整数，体积和算力需求砍半，精度几乎不掉 |

#### 模型类（本项目四个 AI 模型）

| 词 | 全称 | 干什么 |
|----|------|--------|
| **VAD** | Voice Activity Detection | 判断「有没有人在说话」，给录音断句 |
| **ASR** | Automatic Speech Recognition | 语音→文字（sherpa-onnx，CPU 跑） |
| **LLM** | Large Language Model | 理解文字意思、生成回答（Qwen3.5-0.8B，NPU 跑） |
| **YOLO** | You Only Look Once | 目标检测，在图像里框出物体（yolo26n，NPU 跑） |

### 0.3 一次请求的完整旅程（最核心，建议读三遍）

用户对麦克风说「**帮我找找有没有人**」，看数据怎么流水线流转：

```
第 1 步  录音 + 断句（audio_vad_node）
        麦克风持续录音，VAD 一直判断「有没有人说话」。
        检测到开始说话 → 等连续 2 秒静音 → 判定「说完了」，得到一段完整音频。

第 2 步  语音转文字（asr_node）
        那段音频发给 asr_node，sherpa-onnx 识别成文字「帮我找找有没有人」。

第 3 步  意图理解（llm_node）
        文字到了 llm_node，分两件事：
        ① 正则检查：有没有「找/检测」这类词 → 命中，判定是「找东西」意图；
        ② LLM 抽词：把这句话丢给大模型，让它只回答「要找的物体名」→ 得到 "person"。

第 4 步  下发命令（llm_node → decision_node）
        llm_node 打包成一条结构化命令 TaskCommand(action=detect, target=person)，
        发到 /task_command。

第 5 步  决策路由（decision_node）
        收到命令，看到 action=detect → 调 /yolo_detect 这个 Action，
        让 yolo_node 开始检测「person」。

第 6 步  视觉检测（yolo_node）
        循环抓相机帧：V4L2 抓 UYVY → RGA 转成 640×640 RGB → NPU 跑 YOLO
        → 后处理 NMS → 得到一堆框，匹配「person」。

第 7 步  三条支路同时走（yolo_node）
        ① 发 /vision_context（画面里有哪些物体）→ llm_node 存下来备用；
        ② 发 /yolo_markers（检测框）→ PC 端 rviz2 实时画绿框；
        ③ 检测结束把最终结果（检测到 person 82%）返回给 decision_node。

第 8 步  生成回答（decision_node → llm_node）
        decision_node 把「检测结果 + 用户原话」拼成问题发回 llm_node，
        llm_node 用大模型生成一句自然语言回答，发到 /llm_response。
```

> 一句话串起来：**录音 → 断句 → 识别 → 理解 → 决策 → 检测 → 回答**。5 个节点接力，数据像流水线一站一站往下传。如果用户只是闲聊（比如「你好」），第 3 步正则没命中「找/检测」，就跳过检测直接生成回答，链路更短。

### 0.4 代码目录地图（每个文件干什么）

```
rk3588_voice_assistant_ros2/
├── run.sh            # 启动脚本：初始化麦克风 + 注入环境变量 + ros2 launch
├── build.sh          # 编译脚本：colcon build
├── cyclonedds.xml    # 跨机 DDS 单播配置（板端这份）
├── src/
│   ├── rk3588_voice_assistant_interfaces/   # 自定义消息接口（独立包）
│   │   ├── msg/
│   │   │   ├── AudioChunk.msg      # 一段音频（采样率 + PCM 数据）
│   │   │   ├── TaskCommand.msg     # 任务命令（action + target + raw）
│   │   │   └── VisionContext.msg   # 画面物体摘要
│   │   └── action/
│   │       └── YoloDetect.action   # 检测任务（目标+时长 / 结果+bbox / 进度反馈）
│   └── rk3588_voice_assistant/              # 主包
│       ├── nodes/                   # 5 个节点（每个 .cpp 是一段可执行程序）
│       │   ├── audio_vad_node.cpp   # 录音 + VAD 断句
│       │   ├── asr_node.cpp         # 语音识别
│       │   ├── llm_node.cpp         # 意图理解 + 抽词 + 对话
│       │   ├── decision_node.cpp    # 决策路由（纯逻辑，不碰硬件）
│       │   └── yolo_node.cpp        # 视觉检测 + 发 marker
│       ├── engine/                  # 引擎层（硬件/模型封装，不依赖 ROS，可独立测试）
│       │   ├── audio/               # 麦克风采集（miniaudio）+ 无锁环形缓冲
│       │   ├── asr/sherpa_asr.*     # sherpa-onnx ASR 封装
│       │   ├── llm/rkllm_engine.*   # RKLLM 大模型封装（流式 / 同步两种）
│       │   ├── vad/vad_engine.*     # WebRTC VAD 封装
│       │   └── yolo/                # RKNN YOLO + 相机抓帧 + RGA 转换 + JPG 落盘
│       ├── launch/                  # 启动文件（编排 5 节点 + 生命周期转换顺序）
│       ├── config/params.yaml       # 所有参数（模型路径、阈值、相机等）
│       └── third_party/             # 预编译 .so 库 + 模型 + 头文件
└── ...
```

> **engine/ 和 nodes/ 为什么分开？** nodes/ 是「ROS 胶水层」——只负责收消息、发消息、编排流程；engine/ 是「干活的」——不 import 任何 ROS 头文件，纯 C++ 封装硬件/模型。好处：① 换后端（比如换 ASR 引擎）只改 engine/，不动 ROS 逻辑；② engine/ 能脱离 ROS 单独写单元测试。这是「硬件/算法与框架解耦」的工程习惯。

### 0.5 五个节点职责速查表（面试前背这个）

| 节点 | 一句话职责 | 输入 | 输出 | 用啥硬件/模型 |
|------|-----------|------|------|--------------|
| audio_vad_node | 录音 + 判断说没说完 | /voice_trigger 触发 | /utterance_audio | 麦克风 + libfvad(VAD) |
| asr_node | 声音转文字 | /utterance_audio | /recognized_text | sherpa-onnx(CPU) |
| llm_node | 理解意思、抽词、生成回答 | 文字 | /task_command 命令 | RKLLM(NPU) |
| decision_node | 路由：聊天 / 找东西 | /task_command | 调 /yolo_detect | 无（纯逻辑） |
| yolo_node | 框出画面里的物体 | /yolo_detect 命令 | /vision_context + /yolo_markers | 相机 + RGA + RKNN(NPU) |

---

## 1. 硬件平台：RK3588 三硬件协同

RK3588 是瑞芯微旗舰 SoC：8 核 CPU（4×A76 + 4×A55）、**3 核 NPU（6 TOPS）**、Mali GPU、**RGA 2D 硬件引擎**。

本项目的一条检测链路，三种硬件各干各的、互不浪费：

| 步骤 | 硬件 | 代码 | 为什么是它 |
|------|------|------|-----------|
| UYVY→RGB + 缩放到 640×640 | **RGA** | `rga_uyvy_to_rgb()` | 像素搬运/格式转换是纯数据搬运，RGA 硬件 1ms 干完，CPU 要几十 ms |
| YOLO 模型前向（卷积） | **NPU** | `rknn_run()` | 卷积矩阵乘是 NPU 的看家本事，6 TOPS 秒出结果 |
| NMS 后处理（遍历 80×8400） | **CPU** | `postprocess()` | 非规则逻辑（排序、IoU、分支），NPU 不擅长，CPU 灵活 |



---

## 2. 系统架构

四层架构（**感知 → 认知 → 决策 → 执行**），对应机器人系统的标准分层：

```text
语音 ──→ [感知] audio_vad_node(录音 + VAD 断句) → asr_node(语音识别)
      ──→ [认知] llm_node(RKLLM NPU) 理解意图 → 生成结构化 /task_command
      ──→ [决策] decision_node 路由：chat 直接回复 / detect 调用视觉检测
      ──→ [执行] yolo_node(RKNN NPU + RGA) 检测 → 发布 /vision_context 画面上下文 + /yolo_markers(PC rviz 画框)
      ──→ [认知] llm_node 结合视觉上下文 → 生成最终回答 → /llm_response
```

| 节点 | 层 | 订阅 | 发布 | Service | Action | 硬件/模型 |
|------|----|------|------|---------|--------|----------|
| `audio_vad_node` | 感知 | `/voice_trigger` | `/utterance_audio` | `/vad_state` | — | miniaudio + libfvad（ES8388 声卡） |
| `asr_node` | 感知 | `/utterance_audio` | `/recognized_text` | — | — | sherpa-onnx（zh-small int8） |
| `llm_node` | 认知 | `/recognized_text`, `/text_input`, `/vision_context`, `/llm_query` | `/task_command`, `/llm_response` | `/reset_context` | — | RKLLM NPU（Qwen3.5-0.8B W8A8） |
| `decision_node` | 决策 | `/task_command` | `/llm_response`, `/llm_query` | — | `/yolo_detect` (client) | 纯逻辑路由，无硬件 |
| `yolo_node` | 执行 | — | `/vision_context`, `/yolo_markers` | — | `/yolo_detect` (server) | RKNN NPU（yolo26n）+ MIPI 相机 + RGA |

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



### 3.2 ROS2 QoS（服务质量分级）

不同数据流对可靠性和延迟的要求不同，不能一刀切：

| Topic | Reliability | Durability | Depth | 理由 |
|-------|------------|------------|-------|------|
| `/voice_trigger` | Reliable | **Transient Local** | 1 | 触发信号不能丢；transient_local 让晚订阅者也能拿到最近一条 |
| `/utterance_audio` | **Best Effort** | Volatile | 5 | 音频帧丢了不影响 ASR 语义，省掉 DDS ACK 换低延迟 |
| `/recognized_text` | Reliable | Volatile | 10 | 文本语义不能丢 |
| `/llm_response` | Reliable | Volatile | 10 | 同上 |



### 3.3 ROS2 Action（长任务建模）

视觉检测是「持续 N 秒 + 周期反馈 + 可取消」的长任务，**Topic 建模不了 cancel，Service 是同步一次性**，只有 Action 三者兼备。

`/yolo_detect`（`YoloDetect.action`）三部分：

| 部分 | 字段 | 说明 |
|------|------|------|
| Goal | `target_class`, `duration_sec` | 找什么、测多久 |
| Result | `detected`, `best_class`, `best_confidence`, `bbox_x/y/w/h` | 最终结果 |
| Feedback | `elapsed_sec`, `all_objects`, `progress` | 周期上报画面里所有物体 |

client 端用 `async_send_goal` 带三个回调：`goal_response_callback`（server 是否接单）、`result_callback`（最终结果）、可选 `feedback_callback`（实时进度）。server 端 `execute()` 里循环抓帧 → 推理 → `publish_feedback`，被 cancel 时 `gh->canceled(result)`。



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

> ⚠️ **遗留问题**：板端风扇/环境底噪被 VAD 判成语音，导致 2s 静音触发不了，实际会录满 10s 才发布。修法两条：mode 2→3（更激进判静音），或调低 `Capture Digital Volume`（numid 25，当前 192）。

### 3.7 sherpa-onnx ASR（离线识别）

zh-small int8 量化模型（encoder / decoder / joiner 三件套），16kHz 单声道，4 线程 CPU 推理（ASR 不吃 NPU，CPU 够）。

- 优先找 `.int8.onnx` 量化版（更小更快），否则回退 FP32。
- 封装成 `SherpaASR::Recognize(samples, n) -> string`，输入 PCM 输出汉字。



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


### 3.9 RKNN YOLO 检测（NPU）

yolo26n，**split 双输出**：`bbox[4,8400]` + `score[80,8400]`（8400 = 3 个检测头网格总数，80 = COCO 类数）。输入 640×640 RGB。

后处理全在 CPU：对 8400 个 anchor 求每个的 argmax 类别置信度 → 过滤 `conf_threshold=0.25` → **NMS（IoU 阈值 0.45）** → 输出 Detection 列表。


### 3.10 RGA 硬件转换（不依赖 OpenCV）

`rga_uyvy_to_rgb()`：UYVY(1920×1080) → RGB(640×640)，用 RGA 的 `imresize` 一步完成**格式转换 + 缩放**，1ms 级。



### 3.11 意图识别：规则（正则）+ LLM 抽词（混合）

这是本项目最有「设计味道」的地方。用户的自然语言要路由到「对话」还是「检测」，并抽出目标词：

```
用户: "帮我检测一下一个人你能不能检测到一个人"
   │
   ├─ 第 1 层：正则只判意图  isDetectIntent("找|寻找|检测|find") → 命中「检测」
   │
   ├─ 第 2 层：LLM 抽词（槽位填充）RunSync("从这句话提取物体名，只输出物体名") → "person"
   │
   └─ 下发 /task_command(action=detect, target=person) 给决策层
```

**为什么正则不能抽词**：`std::regex` 是字节级的、不懂中文分词。「检测一下一个人你能不能检测到一个人」里哪个是目标（人）、哪个是虚词（一下/一个）、哪个是尾巴（你能不能…），正则抓 `\S+` 会把整句吞掉 → YOLO 拿垃圾串去匹配 class，必然未检测到。**这是踩过的真坑**，改成了「正则判意图 + LLM 抽词」两层。



配套 `RKLLMEngine::RunSync(prompt) -> string`：跟 `Run`（只流式打印）的区别是 token 同时收进 buffer 返回，供抽词这种「要拿结果」的场景用。

### 3.12 检测结果落盘（stb_image_write）

板端无显示器，检测到目标时把当帧画上**绿色 bbox + 左上角置信度数字**，用 `stbi_write_jpg` 存 JPG 到 `yolo_out/`（`save_dir` 参数可配）。

- 选 stb 单头文件（public domain），自带 JPG 编码器，**板端零外部依赖**，不用装 libjpeg/OpenCV。
- 只在「第一次命中目标的那帧」保存一次，避免 5 秒刷 50 张图撑爆磁盘。
- 文件名 `detect_<类别>_<置信度>.jpg`，如 `detect_person_37.jpg`。

### 3.13 决策层解耦：TaskCommand + DecisionNode + VisionContext

这是 V2 升级的核心，把「LLM 直接调 YOLO」重构成机器人系统的标准分层。

**为什么 LLM 不该直接调 YOLO**：认知层（理解意图）和执行层（视觉检测）耦合后，每加一个执行能力（机械臂、导航）都要改认知层代码。解耦后 LLM 只输出结构化命令，由决策层统一路由。

| 组件 | 方向 | 职责 |
|------|------|------|
| `TaskCommand.msg` | 认知 → 决策 | 结构化任务命令：`action`(detect/chat) + `target` + `raw`(用户原话) |
| `decision_node` | 决策 | 订阅 `/task_command`，`detect`→调 YOLO Action，`chat`→透传回复到 `/llm_response` |
| `VisionContext.msg` | 执行 → 认知 | `yolo_node` 检测期间发布画面物体列表（如 `person(87),cup(92)`），`llm_node` 保存为 `last_ctx_` |
| `/llm_query` | 决策 → 认知 | 检测结束后 decision_node 把结果拼成 prompt，交回 llm_node 生成自然语言回答 |

**视觉上下文注入**（多模态的关键）：`yolo_node` 每帧把检测到的物体发 `/vision_context`，`llm_node` 保存为 `last_ctx_`。用户再问「桌上有什么」时，llm_node 把 `last_ctx_` 拼进 prompt，让 LLM 基于真实画面回答，而不是编造。

**关键取舍**：`action` 字段（detect/chat）由**正则**判断而非 LLM 输出——0.8B 小模型的结构化输出不稳定（偶尔漏 key、混中文），正则判意图免费且确定，LLM 只负责抽词/生成（自由文本），规避了结构化输出的单点故障。



### 3.14 RViz2 Marker 可视化（跨机 bbox 显示）

板端无屏，但检测结果要「看得见」。yolo_node 在检测期间额外发一路 `visualization_msgs/MarkerArray` 到 `/yolo_markers`，PC 端 rviz2 订阅后实时画框（跨机 DDS 见第 6 节）。

| 字段 | 值 | 为什么 |
|------|-----|--------|
| `type` | `CUBE` | 2D 检测框用薄立方体表达，rviz 零插件即可渲染 |
| `frame_id` | `camera` | 相机坐标系，rviz 的 Fixed Frame 设成 `camera` 即可直接显示（单相机无位姿，不搭 TF 树） |
| `position.y` | `640 - (d.y + d.h/2)` | **Y 轴翻转**：图像坐标 y 向下、rviz y 向上，不翻转框会上下颠倒 |
| `scale.x/y/z` | `d.w` / `d.h` / `0.01` | 框宽高即检测框宽高，z 压成 0.01 的薄片 |
| `lifetime` | 1s | 检测期每 100ms 刷一次，停止后 1s 自动消失，不留残影 |







## 5. 端到端数据流走查

**场景 A：语音对话**

```
发 /voice_trigger (Empty)
  → audio_vad_node 录音 + VAD 断句（或 10s 满）
  → 重采样 44.1k 双声道 → 16k 单声道 int16[]  → AudioChunk 发布
  → asr_node sherpa-onnx 识别 → String 发布
  → llm_node 正则没命中「找/检测」→ Run() 流式对话 → 回复
```

**场景 B：语音触发视觉检测（经过决策层）**

```
...同上直到 llm_node
  → 正则命中「检测」→ RunSync() 抽词 "person"
  → 下发 /task_command(action=detect, target=person, raw=原话)
  → decision_node 收到 → 调 /yolo_detect Action
  → yolo_node 相机抓帧 UYVY → RGA 转 RGB 640×640 → NPU 推理
  → NMS → 匹配 person → 持续发布 /vision_context（llm_node 保存 last_ctx_）
  → 检测结束 → result(detected=true, conf=0.74) 回 decision_node
  → decision_node 拼 /llm_query（结果 + 原话）→ llm_node 生成自然回答 → /llm_response
```

**场景 C：直接调 YOLO（绕过 LLM 和决策层）**

```
ros2 action send_goal /yolo_detect ... "{target_class: person, duration_sec: 4}"
```

**场景 D：视觉问答（基于上次检测结果，不重新检测）**

```
检测完成后（llm_node 已保存 last_ctx_）
  → 用户问「桌上有什么」（正则没命中「找/检测」→ chat）
  → llm_node 把 last_ctx_ 拼进 prompt → LLM 回答「画面里有 person 和 cup」
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

启动后等 ~30s（LLM 最慢），`ros2 lifecycle list` 确认四个 lifecycle 节点 `active`（decision_node 是普通 Node，启动即就绪）。

### 测试

```bash
# 生命周期（四个 lifecycle 节点）
ros2 lifecycle list /llm_node /asr_node /yolo_node /audio_vad_node

# 决策层与视觉上下文
ros2 topic echo /task_command      # 结构化命令（detect/chat + target）
ros2 topic echo /vision_context    # 画面物体列表
ros2 topic echo /llm_query         # 检测结果回答请求

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

### 跨机 DDS 单播（板端 ↔ PC）

板端和 PC 是两台机器，默认 DDS 靠**组播**做发现。但板端 WiFi 的 AP 开了 **AP 隔离（client isolation）**，上行组播被拦，两端互相发现不了。解法：改用 **Cyclone DDS 单播**，把对端 IP 写进 `Peers` 直连，绕开组播。

两端各放一份 `cyclonedds.xml`（板端在 `~/code/rk3588_voice_assistant_ros2/`，PC 在 `~/`），内容相同、只差 `NetworkInterfaceAddress` 一个 IP：

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <NetworkInterfaceAddress>192.168.0.157</NetworkInterfaceAddress>  <!-- 板端 157 / PC 170 -->
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>30</MaxAutoParticipantIndex>
      <Peers>
        <Peer Address="192.168.0.157"/>
        <Peer Address="192.168.0.170"/>
        <Peer Address="127.0.0.1"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

注入环境变量（板端 `run.sh` 已注入，PC 手动或走 `pc_rviz.sh`）：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$SCRIPT_DIR/cyclonedds.xml   # 注意：不是 CYCLONE_DDS_URI
```



### RViz2 可视化（PC 端）

```bash
# 1. PC 端验证跨机发现（5 个板端节点都该出现）
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/alin/cyclonedds.xml
ros2 node list --no-daemon --spin-time 10

# 2. 起 rviz2（预配 MarkerArray 显示 + Fixed Frame camera）
bash deploy/pc_tools/pc_rviz.sh

# 3. 触发检测（板端跑，走完整 action 链路，检测 15s）
adb shell "bash /tmp/trigger_yolo.sh"    # 或直接对麦克风说「找找有没有人」
```

PC 端工具都在 `deploy/pc_tools/`：

| 文件 | 作用 |
|------|------|
| `pc_rviz.sh` | 注入 DDS env + 用预配 config 起 rviz2 |
| `yolo_rviz.rviz` | rviz 配置：`MarkerArray` 订阅 `/yolo_markers`、`Fixed Frame: camera`、Orbit 距离 1000 |
| `trigger_yolo.sh` | **板端**触发脚本（`adb push` 到板端 `/tmp/` 后跑），直接给 `/yolo_detect` 发 15s goal |

> ⚠️ **为什么触发要在板端跑**：`/yolo_detect` 用的是自定义 action 类型 `rk3588_voice_assistant_interfaces/action/YoloDetect`，PC 没装这个接口包，`ros2 action send_goal` 在 PC 上解析不了类型。板端有完整 install，所以触发放板端。

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

### 跨机 DDS 发现不了（PC 列不出板端节点）
按顺序排查四个坑（都是踩过的真坑）：

1. **环境变量名写错**：是 `CYCLONEDDS_URI`，不是 `CYCLONE_DDS_URI`。写错 Cyclone **静默忽略**、回退默认组播，不报任何错——查 bug 时 `strings libddsc.so | grep CYCLONE` 才找到真名。
2. **`ParticipantIndex=none`**：默认值跟单播**不兼容**（拿随机端口），必须显式 `auto`。
3. **`MaxAutoParticipantIndex` 不够**：默认 10，5 节点 + daemon + CLI 超过上限，报 `Failed to find a free participant index for domain 0`，调成 30。
4. **CLI 等待太短**：`ros2 node list --no-daemon` 默认 spin 时间短，等不到板端周期 SPDP，加 `--spin-time 10`。

> 定位手段：PC 端 `tcpdump -i ens33 'udp and host 192.168.0.157'` 看板端有没有往 170:7410+ 发 SPDP；`CYCLONEDDS_URI` 配 `<Tracing><Verbosity>CONFIG</Verbosity><OutputFile>/tmp/dds.log</OutputFile></Tracing>` 能看到 Cyclone 内部「trying to find a free participant index」这类关键日志。

### NetworkInterfaceAddress 弃用警告
Cyclone 0.10.5 里 `NetworkInterfaceAddress` 已**弃用**（打 warning 但功能正常）。不加它，板端在 AP 隔离下选不出正确的出网接口、不往 PC 发 SPDP；加了它才能跨机。正规写法是 `<General><Interfaces><NetworkInterface name="wlan0"/></Interfaces></General>`（按接口名而非 IP），当前版本弃用元素仍可用所以先留着。

---


---

