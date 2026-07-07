from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Declare launch argument for whether to launch RViz2
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false',
                                     description='Whether to launch RViz2')

    # Declare launch argument for simulation clock
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false',
                                     description='Use simulation (Gazebo) clock if true')

    # slam_toolbox mapping parameters
    slam_params = os.path.join(
        get_package_share_directory('ugv_slam'), 'config', 'slam_toolbox_mapping.yaml')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('ugv_slam'), 'rviz', 'view_slam_2d.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # Include launch description for robot pose publisher
    robot_pose_publisher_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('robot_pose_publisher'), 'launch'),
         '/robot_pose_publisher_launch.py'])
    )

    # slam_toolbox online async mapping node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # Return launch description
    return LaunchDescription([
        use_rviz_arg,
        use_sim_time_arg,
        rviz_node,
        robot_pose_publisher_launch,
        slam_toolbox_node
    ])
