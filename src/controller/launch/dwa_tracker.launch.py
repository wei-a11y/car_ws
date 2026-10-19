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
    share = Path(
        get_package_share_directory('controller')
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'planning_frame',
            default_value='map',
            description='Planning frame'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=str(
                share / 'config/dwa_tracker.yaml'
            )
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        Node(
            package='controller',
            executable='dwa_tracker_node',
            name='dwa_controller',
            output='screen',

            parameters=[
                LaunchConfiguration('params_file'),

                {
                    'planning_frame': ParameterValue(
                        LaunchConfiguration('planning_frame'),
                        value_type=str
                    ),

                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'),
                        value_type=bool
                    ),
                }
            ]
        ),
    ])