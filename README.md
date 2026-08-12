# rk3588_voice_assistant

RK3588 端侧离线语音助手 + 视觉检测 — 四节点 ROS2 Pipeline（LifecycleNode + QoS + Services + Action）

```
麦克风 → VAD 断句 → 语音识别 → LLM 对话 →（找 X 时）→ YOLO 视觉检测
```

全链路离线：语音识别（sherpa-onnx）、对话生成（RKLLM NPU）、目标检测（RKNN NPU）都在 RK3588 板端跑，不上云。

## 架构

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
| `audio_vad_node` | `/voice_trigger` (Empty) | `/utterance_audio` (AudioChunk) | `/vad_state` (Trigger) | — | miniaudio + libfvad |
| `asr_node` | `/utterance_audio` | `/recognized_text` (String) | — | — | sherpa-onnx (zh-small int8) |
| `llm_node` | `/recognized_text`, `/text_input` | `/llm_response` (String) | `/reset_context` (Empty) | `/yolo_detect` (client) | RKLLM NPU (Qwen3.5-0.8B W8A8) |
| `yolo_node` | — | — | — | `/yolo_detect` (server) | RKNN NPU (yolo26n) + MIPI 相机 + RGA |

## 设计决策

### LifecycleNode

四节点全部使用 `rclcpp_lifecycle::LifecycleNode`，管理硬件资源生命周期：

| 回调 | audio_vad_node | asr_node | llm_node | yolo_node |
|------|---------------|----------|----------|-----------|
| `on_configure` | 打开麦克风 | 加载 ASR 模型 | 加载 LLM 模型（NPU，最慢） | 加载 YOLO 模型 + 打开相机 |
| `on_activate` | 订阅 /voice_trigger | 订阅 /utterance_audio | 订阅 /recognized_text | 激活 action server |
| `on_deactivate` | 取消订阅 | 取消订阅 | 取消订阅 | — |
| `on_cleanup` | 释放 mic | 释放模型 | 释放 NPU 句柄 | 关相机 + 释放模型 |

launch 文件按序 transition：LLM（最慢）→ ASR + YOLO → audio_vad。

**为什么**: 确保模型加载完成前不会收到推理请求，避免空指针或超时。加载耗时：LLM ~10s、ASR ~15s、YOLO ~1s，launch 里用 `TimerAction` 错峰。

### QoS 配置

| Topic | Reliability | Durability | Depth | 理由 |
|-------|------------|------------|-------|------|
| `/voice_trigger` | Reliable | Transient Local | 1 | 触发信号不能丢 |
| `/utterance_audio` | Best Effort | Volatile | 5 | 音频帧丢了不影响 ASR |
| `/recognized_text` | Reliable | Volatile | 10 | 文本不能丢 |
| `/llm_response` | Reliable | Volatile | 10 | 文本不能丢 |
| `/text_input` | Reliable | Volatile | 10 | 调试文字输入 |

**为什么**: 音频用 Best Effort 降低延迟（无 DDS ACK），文本用 Reliable 保证语义完整。

### Service 接口

- `/reset_context` (std_srvs/Empty) — 清空 LLM 对话历史，切换任务用
- `/vad_state` (std_srvs/Trigger) — 查询 VAD 状态（idle/listening）

**为什么**: Service 用于命令式操作，Topic 用于数据流 — ROS2 设计原则。

### Action 接口（视觉检测）

`/yolo_detect` (YoloDetect.action)，LLM 作为 client，YOLO 作为 server：

| 部分 | 字段 | 说明 |
|------|------|------|
| Goal | `target_class` | 要找的物体（中英文皆可，如 "苹果"/"apple"） |
| Goal | `duration_sec` | 持续检测时长（0 = 单帧） |
| Result | `detected` / `best_class` / `best_confidence` | 是否找到 + 最优结果 |
| Result | `bbox_x/y/w/h` | 边界框（640×640 归一化坐标） |
| Feedback | `all_objects` | 画面中实时检测到的所有物体 |

**为什么用 Action**: 视觉检测是"持续一段时间 + 周期性反馈 + 可取消"的长任务，Topic 建模不了 cancel/feedback。

### LLM → YOLO 函数调用（Function Calling）

用户说"找苹果"时，LLM 不生成回复，而是触发视觉检测：

```
用户: 找苹果
  └─ llm_node 正则匹配意图 → 提取目标 "苹果"
       ├─ 中文→英文归一化: 苹果 → apple
       ├─ 发送 YoloDetect goal (target_class=apple, duration=5s)
       ├─ yolo_node 相机抓帧 → RGA 转 RGB → NPU 推理 → 匹配目标
       └─ 结果回调 → 拼检测结果 → 喂给 LLM → 发布 /llm_response
```

意图正则：`找X` / `寻找X` / `找一下X` / `find X` / `检测X`（`llm_node.cpp` 的 `extractDetectTarget`）。

中文→英文类名映射：模型输出 COCO 英文类名（"apple"），语音输入是中文（"苹果"），在 `yolo_node.cpp` 的 `zhToEn` 里归一化（23 个常用词，缺的走英文直传）。

### 相机输入管线

```
/dev/video11 (rkisp_mainpath, MIPI CSI)
   → V4L2 mmap 采集 (UYVY 4:2:2, 1920×1080)
   → RGA 硬件缩放 + 色彩转换 (UYVY→RGB, 640×640)
   → YOLO NPU 推理
```

- 用 RGA（Rockchip 硬件 2D 引擎）做格式转换，**不依赖 OpenCV**，1ms 内完成
- 相机打不开时自动 fallback 到测试灰图，不影响节点启动

## 目录

```
rk3588_voice_assistant_ros2/                 ← 仓库根 = ROS2 workspace
├── build.sh                                ← 一键编译（colcon build，默认 src/）
├── .gitignore                              ← build/ install/ log/
├── README.md
└── src/                                    ← colcon 标准 src 目录
    ├── rk3588_voice_assistant_interfaces/  ← 接口包（纯 rosidl，无 runtime）
    │   ├── CMakeLists.txt  package.xml
    │   ├── msg/
    │   │   └── AudioChunk.msg              # header + int16[] samples + sample_rate
    │   └── action/
    │       └── YoloDetect.action           # 视觉检测 goal/result/feedback
    └── rk3588_voice_assistant/             ← 主包（4 节点实现）
        ├── CMakeLists.txt  package.xml
        ├── config/
        │   └── params.yaml                 # 各节点参数（模型路径/相机/阈值）
        ├── launch/
        │   └── voice_assistant.launch.py   # 4 节点 + 按序 lifecycle transition
        ├── nodes/                          # 4 个 ROS2 节点 .cpp
        │   ├── audio_vad_node.cpp
        │   ├── asr_node.cpp
        │   ├── llm_node.cpp                # 含 YOLO action client + 意图识别
        │   └── yolo_node.cpp               # 含中文类名映射
        ├── include/rk3588_voice_assistant/ # 4 个节点 .hpp
        ├── engine/                         # 引擎层（纯 C++，不依赖 ROS）
        │   ├── audio/                      # miniaudio 采集 + ring_buffer + wav_writer
        │   ├── vad/                        # libfvad 封装
        │   ├── asr/                        # sherpa-onnx 封装
        │   ├── llm/                        # RKLLM 封装
        │   └── yolo/                       # yolo_engine + camera_capture + rga_convert
        └── third_party/                    # 板端 .so + 头文件（自包含）
            ├── rkllm/                      # librkllmrt.so
            ├── rknn/                       # librknnrt.so (v2.3.2)
            ├── sherpa-onnx/                # libsherpa-onnx-c-api.so + libonnxruntime.so
            ├── libfvad/                    # 源码编译
            ├── miniaudio/                  # header-only
            └── rga/                        # RGA 头文件（librga.so 用板端系统库）
```

**为什么这样分**: 接口（msg/action）独立成 `*_interfaces` 包，是 ROS2 评审第一眼的强约定——接口会被多个包共享，且纯 rosidl 无 runtime 依赖。主包内 engine/ 与 nodes/ 分离，讲清"硬件封装和 ROS 胶水层解耦"。第三方 .so 随包走，GitHub clone 下来板端能直接编译。

## 依赖

| 库 | 用途 | 来源 |
|----|------|------|
| `librkllmrt.so` | LLM NPU 推理 | third_party/rkllm |
| `librknnrt.so` (v2.3.2) | YOLO NPU 推理 | third_party/rknn |
| `librga.so` | 硬件色彩转换/缩放 | 板端系统库 /usr/lib |
| `libsherpa-onnx-c-api.so` + `libonnxruntime.so` | ASR 离线识别 | third_party/sherpa-onnx |
| `libfvad` | WebRTC VAD | third_party/libfvad（源码编译） |
| `miniaudio` | 音频采集（header-only） | third_party/miniaudio |
| `rclcpp` + `rclcpp_lifecycle` + `rclcpp_action` + `std_msgs` + `std_srvs` | ROS2 Humble | 系统 |

> 注意：LLM 和 YOLO 用的是**两套不同的 NPU runtime**。`librkllmrt.so`（RKLLM）跑大模型，`librknnrt.so`（RKNN）跑常规算子。yolo26n 是 RKNN 模型 v6，**必须用 librknnrt.so v2.3.2**（v1.4.0 会报 `Invalid RKNN model version 6`）。

## 编译 & 运行

### 板端编译

```bash
# 前置：项目已推到板端，.so 已就位
cd ~/code/rk3588_voice_assistant_ros2
source /opt/ros/humble/setup.bash
./build.sh
# 等价于: rm -rf build install log && colcon build
```

标准 `src/` 布局，colcon 默认发现 `src/` 下的两个包，无需 `--base-paths`。

### 运行

```bash
source /opt/ros/humble/setup.bash
source ~/code/rk3588_voice_assistant_ros2/install/setup.bash
ros2 launch rk3588_voice_assistant voice_assistant.launch.py
# launch 自动注入 LD_LIBRARY_PATH，无需手动设置
```

启动后等 ~30s（LLM 加载最慢），用 `ros2 lifecycle list` 确认四节点都 `active`。

### 测试

```bash
# 查看生命周期状态
ros2 lifecycle list /llm_node /asr_node /yolo_node /audio_vad_node

# 文字 → LLM 对话
ros2 topic pub --once /text_input std_msgs/msg/String "{data: '你好'}"

# 文字 → 触发视觉检测（找苹果）
ros2 topic pub --once /text_input std_msgs/msg/String "{data: '找苹果'}"

# 直接调 YOLO action（绕过 LLM）
ros2 action send_goal /yolo_detect rk3588_voice_assistant_interfaces/action/YoloDetect \
  "{target_class: 'person', duration_sec: 4}"

# 测试 service
ros2 service call /reset_context std_srvs/srv/Empty
ros2 service call /vad_state std_srvs/srv/Trigger

# 语音全链路（触发 VAD 录音）
ros2 topic pub --once /voice_trigger std_msgs/msg/Empty "{}"
```

## 已知坑

### librknnrt.so 版本

`Invalid RKNN model version 6` → 第三方目录里的 `librknnrt.so` 太旧（v1.4.0）。用板端系统的 v2.3.2 覆盖：

```bash
cp /usr/lib/librknnrt.so third_party/rknn/aarch64/librknnrt.so
```

### ARM64 `std::regex` crash

params YAML 不能用 `/**` 通配符节点名（ROS2 Humble 内部转成 regex，ARM libstdc++ 有 bug）。已修复：用显式节点名 `/llm_node:` 而非 `/**`。

### 麦克风 busy

ALSA 设备 `plughw:2,0` 被占用时报 `Device or resource busy`。`fuser /dev/snd/*` 找进程或 `reboot`。

### 相机权限

`/dev/video11` 是 `root:video` 权限，运行用户需在 `video` 组（`id topeet` 确认）。adbd push 上去的文件是 root 属主，编译/写日志前先 `chown -R topeet:topeet`。

### 找不到 .so

`error while loading shared libraries: librkllmrt.so` → 未 source install/setup.bash，或手动 `ros2 run` 时没设 LD_LIBRARY_PATH（launch 会自动注入）。

## 未来规划

| 项目 | 为什么暂不做 |
|------|-------------|
| **目标跟踪** | 当前是逐帧检测 + 取最高置信度，真正的 tracking（ByteTrack/DeepSORT）需要时序关联，独立 feature |
| **TTS 节点 (piper-tts)** | 需额外模型和 C API 封装，LLM 文本输出已满足 demo |
| **diagnostic_updater** | 单节点健康用 service 已够，批量监控再加 aggregator |
| **ComposableNode** | AudioChunk ~100KB 零拷贝收益小；相机帧是独立进程更稳 |
| **ILLMEngine / IYoloEngine 抽象** | 各只有一个实现，加虚接口是过度设计 |
| **lifecycle_manager** | 当前 TimerAction + ExecuteProcess 已可靠，manager 节点待加入 |
| **中文类名全覆盖** | 当前映射 23 个常用词，缺的走英文直传；全量 80 类映射 + LLM 翻译再补 |
