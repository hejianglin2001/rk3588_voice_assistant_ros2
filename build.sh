#!/bin/bash
# 编译本项目
# 用法: ./build.sh
# 前置: source /opt/ros/humble/setup.bash

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# 清理旧产物
rm -rf build install log

# 标准 src/ 布局，colcon 默认发现 src/ 下的两个包
colcon build

echo ""
echo "编译完成，运行:"
echo "  source install/setup.bash"
echo "  ros2 launch rk3588_voice_assistant voice_assistant.launch.py"
