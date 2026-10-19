# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('controller'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=str(share / 'config/path_tracking.yaml')),
        Node(package='controller', executable='record_path_tracking.py', name='path_recorder',
             output='screen', parameters=[LaunchConfiguration('params_file')]),
    ])
