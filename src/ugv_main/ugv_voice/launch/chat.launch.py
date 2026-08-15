"""Talk-only LLM chat: ear (mic/ASR) + chat (Ollama) + mouth (TTS).

Deliberately does NOT launch brain_node or behavior_ctrl — nothing in this
graph can command motion. Requires the Ollama daemon on the Jetson host
(see voice_control_plan.md Phase 3 setup) with the configured model pulled.

Start a conversation with:
    ros2 topic pub --once /voice/listen_once std_msgs/msg/Empty '{}'
then just talk; say "goodbye" to end the session.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('ugv_voice'),
                          'config', 'voice_params.yaml')
    return LaunchDescription([
        Node(package='ugv_voice', executable='ear_node',
             name='voice_ear', parameters=[params], output='screen'),
        Node(package='ugv_voice', executable='chat_node',
             name='voice_chat', parameters=[params], output='screen'),
        Node(package='ugv_voice', executable='mouth_node',
             name='voice_mouth', parameters=[params], output='screen'),
    ])
