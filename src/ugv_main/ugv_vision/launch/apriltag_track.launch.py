from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():

    # Include the camera launch file
    camera_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ugv_vision'), 'launch'),
         '/camera.launch.py'])
    )
    
    # Include the apriltag tracking launch file (UGV-specific composable-node setup,
    # maintained first-party; the apriltag_ros nodes/cfg come from upstream via
    # ugv_else.repos).
    apriltag_track_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ugv_vision'), 'launch'),
         '/apriltag_bringup.launch.py'])
    )
                
    # Return the launch description
    return LaunchDescription([
        camera_launch,
        apriltag_track_launch
    ])
