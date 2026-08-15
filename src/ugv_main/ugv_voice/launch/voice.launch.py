"""Kid-facing voice control: ear (mic/ASR) + brain (intents) + mouth (TTS),
plus the hardened behavior_ctrl action server they command motion through.

This is the scripted-speech pipeline — every spoken line comes from
responses.py, and every motion passes the intent_schema validator and
behavior_ctrl's own clamps. (The talk-only LLM mode is chat.launch.py.)

Normally included from bringup:  use_voice:=true on bringup_ros2_control.
Standalone (sim / dev box):      ros2 launch ugv_voice voice.launch.py
Push-to-talk (default mode):
    ros2 topic pub --once /voice/listen_once std_msgs/msg/Empty '{}'

use_behavior_ctrl:=false skips the behavior_ctrl node for graphs that
already run one — never leave it out otherwise; without it voice commands
go nowhere (the brain refuses with "not responding" rather than moving).
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
        Node(package='ugv_voice', executable='brain_node',
             name='voice_brain', parameters=[params], output='screen'),
        Node(package='ugv_voice', executable='mouth_node',
             name='voice_mouth', parameters=[params], output='screen'),
    ])
