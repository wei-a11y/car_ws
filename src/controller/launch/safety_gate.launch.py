# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('controller'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/safety_gate.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable', default_value='true',
                             description='false inhibits automatic commands (zero output)'),
        Node(
            package='controller', executable='safety_gate_node', name='safety_gate',
            output='screen', parameters=[LaunchConfiguration('params_file'), {
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'enable': ParameterValue(LaunchConfiguration('enable'), value_type=bool),
            }]),
    ])
