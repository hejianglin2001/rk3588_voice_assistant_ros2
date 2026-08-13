#!/bin/bash
# ============================================================================
# run.sh — 一键启动语音助手
# 作用:
#   1. 初始化 ES8388 麦克风路由（跟老项目 llm_demo 的 run.sh 一致）
#   2. 注入 LD_LIBRARY_PATH + source install/setup.bash
#   3. ros2 launch 四节点 pipeline
#
# 用法:  ./run.sh
# 前置:  已执行 ./build.sh 编译完成
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# ---- ES8388 麦克风初始化 ----
# PGA 路由指向板载差分主麦 (Differential)，必须把主麦开关打开，
# 否则录音采到的是底噪 (peak ~400)，VAD 判静音 → 录满 10s，ASR 无输出。
amixer -c 2 sset 'Differential Mux' 'Line 2' 2>/dev/null || true
amixer -c 2 cset numid=42 on  2>/dev/null || true   # Main Mic Switch = on
amixer -c 2 cset numid=43 off 2>/dev/null || true   # Headset Mic Switch = off

# ---- 环境 ----
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# ---- 启动 ----
echo "============================================"
echo "  RK3588 语音助手 (audio_vad → asr → llm → yolo)"
echo "============================================"
echo "  等 ~30s 模型加载完，发 /voice_trigger 开始说话"
echo "============================================"

exec ros2 launch rk3588_voice_assistant voice_assistant.launch.py
