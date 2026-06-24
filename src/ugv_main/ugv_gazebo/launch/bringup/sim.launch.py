#!/usr/bin/env python3
"""new-Gazebo (Harmonic) bringup for the UGV beast.

Sim twin of ugv_bringup/launch/bringup_ros2_control.launch.py. Same control stack
as the real robot — only the source of wheel/IMU data differs:

    cmd_vel (Twist) --> twist_stamper --> diff_cont/cmd_vel (TwistStamped) --> diff_cont
        --> gz_ros2_control --> gz physics (track friction)
    gz IMU --> gz_ros2_control --> imu_sensor_broadcaster --> /imu/data_raw
        --> complementary_filter --> /imu/data
    diff_cont wheel odom (/odom_raw) + /imu/data --> EKF --> /odom + odom->base_footprint

gz sensors (lidar, RGBD, pan-tilt cam) reach ROS through ros_gz_bridge. The
controller_manager runs INSIDE gz (loaded by the gz_ros2_control system plugin in
ugv_beast.gazebo.xacro), so there is no ros2_control_node here.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ugv_description = get_package_share_directory('ugv_description')
    ugv_bringup = get_package_share_directory('ugv_bringup')
    ugv_gazebo = get_package_share_directory('ugv_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = {'use_sim_time': True}

    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world', default_value=os.path.join(ugv_gazebo, 'worlds', 'ugv_world.sdf'),
        description='Absolute path to the gz world file')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    x_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_arg = DeclareLaunchArgument('y_pose', default_value='0.0')

    # --- let gz resolve model://world and package:// mesh URIs ---
    # models/ holds the maze (model://world); the share parents let gz find
    # package://ugv_description meshes.
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.pathsep.join([
            os.path.join(ugv_gazebo, 'models'),
            os.path.dirname(ugv_description),   # <prefix>/share, contains ugv_description
            os.path.dirname(ugv_gazebo),
        ]))

    # --- robot_description (sim variant: gz_ros2_control + gz sensors + friction) ---
    xacro_file = os.path.join(ugv_description, 'urdf', 'ugv_beast.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=true']), value_type=str)

    controllers_yaml = os.path.join(ugv_bringup, 'config', 'ugv_beast_controllers.yaml')
    bridge_config = os.path.join(ugv_gazebo, 'launch', 'bringup', 'ros_gz_bridge.yaml')

    # --- start gz (server + gui) ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v3 ', world],
                          'on_exit_shutdown': 'true'}.items())

    # --- robot_state_publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}, use_sim_time],
        output='screen')

    # --- spawn the robot from /robot_description into gz ---
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'ugv_beast',
                   '-x', x_pose, '-y', y_pose, '-z', '0.1'],
        output='screen')

    # --- ros<->gz bridge (clock + sensors; IMU/wheels go through ros2_control) ---
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}, use_sim_time],
        output='screen')

    # --- controller spawners (controller_manager lives inside gz_ros2_control) ---
    # The odom/imu remaps mirror the real launch's controller_manager remaps so the
    # EKF (/odom_raw) and complementary filter (/imu/data_raw) wiring is identical.
    def spawner(name, controller_ros_args=None):
        args = [name, '--controller-manager', '/controller_manager']
        if controller_ros_args:
            args += ['--controller-ros-args', controller_ros_args]
        return Node(package='controller_manager', executable='spawner',
                    arguments=args, parameters=[use_sim_time], output='screen')

    jsb_spawner = spawner('joint_state_broadcaster')
    imu_spawner = spawner('imu_sensor_broadcaster',
                          controller_ros_args='-r /imu_sensor_broadcaster/imu:=/imu/data_raw')
    diff_spawner = spawner('diff_cont',
                           controller_ros_args='-r /diff_cont/odom:=/odom_raw')

    # Start spawners only after the robot (and its in-sim controller_manager) exists.
    after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[jsb_spawner]))
    after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[imu_spawner, diff_spawner]))

    # --- cmd_vel Twist -> TwistStamped (diff_drive_controller is stamped-only) ---
    twist_stamper = Node(
        package='ugv_tools', executable='twist_stamper',
        parameters=[{'frame_id': 'base_footprint'}, use_sim_time],
        remappings=[('cmd_vel_in', '/cmd_vel'), ('cmd_vel_out', '/diff_cont/cmd_vel')],
        output='screen')

    # --- IMU orientation (raw gyro/accel -> imu/data) ---
    imu_complementary_filter = Node(
        package='imu_complementary_filter', executable='complementary_filter_node',
        name='complementary_filter_gain_node', output='screen',
        parameters=[{'do_bias_estimation': True}, {'do_adaptive_gain': True},
                    {'use_mag': False}, {'gain_acc': 0.01}, {'gain_mag': 0.01},
                    use_sim_time])

    # --- sensor-fused odometry + odom->base_footprint TF ---
    ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(ugv_bringup, 'param', 'ekf.yaml'), use_sim_time],
        remappings=[('/odometry/filtered', '/odom')])

    return LaunchDescription([
        world_arg, x_arg, y_arg,
        gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn,
        bridge,
        after_spawn,
        after_jsb,
        twist_stamper,
        imu_complementary_filter,
        ekf_node,
    ])
