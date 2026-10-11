# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Run the stationary mobile manipulator with MoveIt and mock hardware."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    moveit_share = Path(get_package_share_directory('arm_moveit_config'))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(moveit_share / 'launch/demo.launch.py')),
            launch_arguments={'use_rviz': LaunchConfiguration('rviz')}.items()),
    ])
