"""Chat-with-tools voice stack: ear (mic/ASR) + chat (Ollama LLM with
validated tools) + mouth (TTS) + behavior_ctrl (the motion seam).

The LLM converses freely and proposes tool calls; chat_node validates and
clamps each one (tools.py -> intent_schema) before dispatching to
behavior_ctrl, which clamps again. The ear's stop-word path calls
behavior/estop directly, outside the LLM. Requires the Ollama daemon on the
Jetson host (voice_control_plan.md Phase 2 setup) with a tool-capable model
pulled; with the daemon down the node degrades to RuleBrain basic commands.

Normally included from bringup:  use_voice:=true on bringup_ros2_control.
Standalone (sim / dev box):      ros2 launch ugv_voice chat.launch.py
Start a conversation with:
    ros2 topic pub --once /voice/listen_once std_msgs/msg/Empty '{}'
then just talk; say "goodbye" to end the session.

use_behavior_ctrl:=false skips the behavior_ctrl node for graphs that
already run one — never leave it out otherwise; without it motion tools
are refused with "not responding" and the stop word has nothing to stop.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('ugv_voice'),
                          'config', 'voice_params.yaml')

    use_behavior_ctrl_arg = DeclareLaunchArgument(
        'use_behavior_ctrl', default_value='true',
        description='Start the behavior_ctrl action server (the motion seam). '
                    'Only set false if another launch already runs it.')

    behavior_ctrl = Node(
        package='ugv_tools', executable='behavior_ctrl',
        name='behavior_ctrl', output='screen',
        condition=IfCondition(LaunchConfiguration('use_behavior_ctrl')))

    return LaunchDescription([
        use_behavior_ctrl_arg,
        behavior_ctrl,
        Node(package='ugv_voice', executable='ear_node',
             name='voice_ear', parameters=[params], output='screen'),
        Node(package='ugv_voice', executable='chat_node',
             name='voice_chat', parameters=[params], output='screen'),
        Node(package='ugv_voice', executable='mouth_node',
             name='voice_mouth', parameters=[params], output='screen'),
    ])
