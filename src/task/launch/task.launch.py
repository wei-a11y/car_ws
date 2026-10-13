# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('task'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/task.yaml')),
        DeclareLaunchArgument('points_file', default_value=str(share / 'config/task_points.yaml')),
        Node(package='task', executable='task_node', name='task_manager', output='screen',
             parameters=[LaunchConfiguration('params_file'),
                         {'points_file': LaunchConfiguration('points_file')}]),
    ])
