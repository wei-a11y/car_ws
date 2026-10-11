# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Run only the optional arm command bridge; reuse the active MoveIt stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _launch_bridge(context):
    """Apply mode overrides to the same parameter mapping as the node's YAML."""
    path = Path(LaunchConfiguration('params_file').perform(context))
    parameters = yaml.safe_load(path.read_text())['arm_command_bridge']['ros__parameters']
    parameters['use_sim_time'] = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    parameters['position_only'] = LaunchConfiguration('position_only').perform(context) == 'true'
    return [Node(
        package='bringup', executable='arm_command_bridge.py', name='arm_command_bridge',
        output='screen', parameters=[parameters])]


def generate_launch_description():
    """Load test-interface parameters without spawning another robot or controller."""
    share = Path(get_package_share_directory('bringup'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=str(share / 'config/arm_command_bridge.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'position_only', default_value='true', choices=['true', 'false'],
            description='Position target by default; false adds an orientation constraint'),
        OpaqueFunction(function=_launch_bridge),
    ])
