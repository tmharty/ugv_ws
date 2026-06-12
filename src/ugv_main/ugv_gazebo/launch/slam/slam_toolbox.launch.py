from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Declare the launch argument for use_sim_time
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true',
                                     description='Use simulation (Gazebo) clock if true')

    # slam_toolbox mapping parameters (single source of truth in ugv_slam)
    slam_params = os.path.join(
        get_package_share_directory('ugv_slam'), 'config', 'slam_toolbox_mapping.yaml')

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

    # Get the ugv_gazebo package share directory
    ugv_gazebo_dir = get_package_share_directory('ugv_gazebo')
    # Get the rviz config file
    rviz_slam_2d_config = os.path.join(ugv_gazebo_dir, 'rviz', 'view_slam_2d.rviz')

    # Launch rviz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_slam_2d_config],
    )

    # Launch robot_pose_publisher node
    robot_pose_publisher_node = Node(package="robot_pose_publisher", executable="robot_pose_publisher",
            name="robot_pose_publisher",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"use_sim_time": True},
                {"is_stamped": True},
                {"map_frame": "map"},
                {"base_frame": "base_footprint"}
            ]
    )

    # Return the launch description
    return LaunchDescription([
        use_sim_time_arg,
        slam_toolbox_node,
        rviz2_node,
        robot_pose_publisher_node
    ])
