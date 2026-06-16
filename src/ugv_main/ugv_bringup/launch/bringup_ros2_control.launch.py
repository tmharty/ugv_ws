#!/usr/bin/env python3
"""Real-robot bringup using ros2_control.

Replaces the legacy ugv_driver + ugv_bringup + base_node_ekf path. The
ugv_hardware SystemInterface owns the ESP32 UART; diff_drive_controller produces
wheel odometry on /odom_raw (TF off) which robot_localization fuses with /imu/data
to publish /odom and the odom->base_footprint transform (same wiring as before).

    cmd_vel (Twist) --> diff_cont --> ugv_hardware --[UART]--> ESP32
    ESP32 --[UART]--> ugv_hardware --> joint_states / imu/data_raw / (mag,voltage)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ugv_description = get_package_share_directory('ugv_description')
    ugv_bringup = get_package_share_directory('ugv_bringup')

    serial_device = LaunchConfiguration('serial_device')
    serial_device_arg = DeclareLaunchArgument(
        'serial_device', default_value='/dev/ttyAMA0',
        description='ESP32 UART device (/dev/ttyTHS1 on Jetson, /dev/ttyAMA0 on Raspberry Pi)')

    xacro_file = os.path.join(ugv_description, 'urdf', 'ugv_beast.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=false serial_device:=', serial_device]),
        value_type=str)

    controllers_yaml = os.path.join(ugv_bringup, 'config', 'ugv_beast_controllers.yaml')

    # --- robot_state_publisher (uses the ros2_control-enabled description) ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    # --- controller_manager ---
    # Topic remaps keep the existing Twist /cmd_vel contract and feed the EKF on /odom_raw.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers_yaml],
        remappings=[
            ('/diff_cont/cmd_vel_unstamped', '/cmd_vel'),
            ('/diff_cont/odom', '/odom_raw'),
            ('/imu_sensor_broadcaster/imu', '/imu/data_raw'),
        ],
        output='screen',
    )

    def spawner(name):
        return Node(
            package='controller_manager', executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager'],
            output='screen')

    jsb_spawner = spawner('joint_state_broadcaster')
    imu_spawner = spawner('imu_sensor_broadcaster')
    diff_spawner = spawner('diff_cont')

    # Chain the spawners so they activate in a deterministic order.
    after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[imu_spawner, diff_spawner]))

    # --- IMU orientation (raw gyro/accel -> imu/data) ---
    imu_complementary_filter = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
        ],
    )

    # --- sensor-fused odometry + odom->base_footprint TF ---
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(ugv_bringup, 'param', 'ekf.yaml')],
        remappings=[('/odometry/filtered', '/odom')],
    )

    # --- lidar ---
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_bringup'), 'launch', 'ldlidar', 'ldlidar.launch.py')))

    # --- low-battery alarm (moved out of ugv_driver) ---
    battery_alarm = Node(package='ugv_bringup', executable='battery_alarm', output='screen')

    return LaunchDescription([
        serial_device_arg,
        robot_state_publisher,
        controller_manager,
        jsb_spawner,
        after_jsb,
        imu_complementary_filter,
        ekf_node,
        lidar_launch,
        battery_alarm,
    ])
