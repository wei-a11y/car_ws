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
        DeclareLaunchArgument('planning_frame', description='Verified Path/map frame'),
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/lqr_tracker.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='controller', executable='lqr_tracker_node', name='lqr_controller',
            output='screen', parameters=[LaunchConfiguration('params_file'), {
                'planning_frame': ParameterValue(
                    LaunchConfiguration('planning_frame'), value_type=str),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }]),
    ])
