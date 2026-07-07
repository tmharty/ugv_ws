#!/usr/bin/env python3
"""Convenience one-command launch: robot bringup (ros2_control) + slam_toolbox.

Equivalent to running, in two terminals:

    ros2 launch ugv_bringup bringup_ros2_control.launch.py
    ros2 launch ugv_slam slam_toolbox.launch.py

Prefer the two-terminal form while tuning: SLAM can then be restarted freely
without restarting the robot layer. Never launch this while bringup is already
running — that would start a second driver on the same serial port.

Arguments given on the command line (use_rviz:=..., serial_device:=...)
propagate into both includes.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ugv_bringup'), 'launch', 'bringup_ros2_control.launch.py')))

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ugv_slam'), 'launch', 'slam_toolbox.launch.py')))

    return LaunchDescription([
        bringup_launch,
        slam_launch,
    ])
