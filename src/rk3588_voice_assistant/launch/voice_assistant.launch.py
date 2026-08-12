"""rk3588_voice_assistant — 语音助手启动文件"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('rk3588_voice_assistant')
    params_path = os.path.join(pkg_dir, 'config', 'params.yaml')

    # 第三方 .so 安装在 lib/rk3588_voice_assistant/
    install_prefix = os.path.normpath(os.path.join(pkg_dir, '..', '..'))
    pkg_lib = os.path.join(install_prefix, 'lib', 'rk3588_voice_assistant')

    lib_path = os.pathsep.join([pkg_lib])
    ld_path = lib_path + os.pathsep + os.environ.get('LD_LIBRARY_PATH', '')
    env = {'LD_LIBRARY_PATH': ld_path}

    # ---- 节点 ----
    llm_node = Node(
        package='rk3588_voice_assistant', executable='llm_node',
        name='llm_node', output='screen',
        arguments=['--ros-args', '--params-file', params_path],
        additional_env=env,
    )
    asr_node = Node(
        package='rk3588_voice_assistant', executable='asr_node',
        name='asr_node', output='screen',
        arguments=['--ros-args', '--params-file', params_path],
        additional_env=env,
    )
    audio_vad_node = Node(
        package='rk3588_voice_assistant', executable='audio_vad_node',
        name='audio_vad_node', output='screen',
        arguments=['--ros-args', '--params-file', params_path],
        additional_env=env,
    )
    yolo_node = Node(
        package='rk3588_voice_assistant', executable='yolo_node',
        name='yolo_node', output='screen',
        arguments=['--ros-args', '--params-file', params_path],
        additional_env=env,
    )

    # ---- 按序 lifecycle 转换 ----
    def lifecycle_cmd(node, transition):
        return ['bash', '-c',
            f'source /opt/ros/humble/setup.bash && ros2 lifecycle set {node} {transition}']

    return LaunchDescription([
        DeclareLaunchArgument('model_path',
            default_value='/home/topeet/code/rkllm_weight/Qwen3.5-0.8B_w8a8_rk3588.rkllm'),
        DeclareLaunchArgument('asr_model_dir',
            default_value='/home/topeet/code/rkllm_weight'),
        DeclareLaunchArgument('yolo_model',
            default_value='/home/topeet/code/rkllm_weight/yolo26n_split.rknn'),

        llm_node, asr_node, audio_vad_node, yolo_node,

        TimerAction(period=1.0, actions=[
            ExecuteProcess(cmd=lifecycle_cmd('/llm_node', 'configure')),
        ]),
        TimerAction(period=12.0, actions=[
            ExecuteProcess(cmd=lifecycle_cmd('/llm_node', 'activate')),
        ]),
        TimerAction(period=13.0, actions=[
            ExecuteProcess(cmd=lifecycle_cmd('/asr_node', 'configure')),
            ExecuteProcess(cmd=lifecycle_cmd('/yolo_node', 'configure')),
        ]),
        TimerAction(period=28.0, actions=[
            ExecuteProcess(cmd=lifecycle_cmd('/asr_node', 'activate')),
            ExecuteProcess(cmd=lifecycle_cmd('/yolo_node', 'activate')),
        ]),
        TimerAction(period=29.0, actions=[
            ExecuteProcess(cmd=lifecycle_cmd('/audio_vad_node', 'configure')),
        ]),
        TimerAction(period=30.0, actions=[
            ExecuteProcess(cmd=lifecycle_cmd('/audio_vad_node', 'activate')),
        ]),
    ])
