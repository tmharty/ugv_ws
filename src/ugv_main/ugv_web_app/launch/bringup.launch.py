# On-demand web UI (vizanti + rosbridge). Deliberately NOT included in any
# standard bringup launch: rosbridge serves an unauthenticated WebSocket with
# full ROS-graph access (including /cmd_vel) to anyone who can reach the port,
# and the containers run with --network host. Launch this only while you are
# actively using the web UI:
#
#   ros2 launch ugv_web_app bringup.launch.py
#
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    host_arg = DeclareLaunchArgument(
        'host', default_value='0.0.0.0',
        description='Interface the vizanti flask server binds to '
                    '(rosbridge itself always binds all interfaces)'
    )

    ugv_web_app_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('vizanti_server'), 'launch'),
         '/vizanti_server.launch.py']),
        launch_arguments={
            'host': LaunchConfiguration('host'),
        }.items()
    )

    return LaunchDescription([
        host_arg,
        ugv_web_app_launch
    ])
