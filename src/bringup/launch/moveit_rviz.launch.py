# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Run the stationary mobile manipulator with MoveIt and mock hardware."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    moveit_share = Path(get_package_share_directory('arm_moveit_config'))
    bringup_share = Path(get_package_share_directory('bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('arm_commands', default_value='true',
                             description='Enable pose and gripper test commands through MoveIt'),
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(moveit_share / 'launch/demo.launch.py')),
            launch_arguments={'use_rviz': LaunchConfiguration('rviz')}.items())]),
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(bringup_share / 'launch/arm_command_bridge.launch.py')),
            condition=IfCondition(LaunchConfiguration('arm_commands')),
            launch_arguments={'use_sim_time': 'false'}.items())]),
    ])
