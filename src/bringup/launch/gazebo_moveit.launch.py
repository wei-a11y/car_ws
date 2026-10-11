# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Run the Gazebo base, arm and gripper with one controller manager and MoveIt."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from bringup.description import gazebo_moveit_config
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from moveit_configs_utils.launches import generate_move_group_launch


def _launch_combined(context):
    bringup_share = Path(get_package_share_directory('bringup'))
    arm_params_file = LaunchConfiguration('arm_params_file').perform(context)
    config = gazebo_moveit_config(arm_params_file)
    position_only = LaunchConfiguration('position_only').perform(context) == 'true'
    config.robot_description_kinematics['robot_description_kinematics']['arm'][
        'position_only_ik'] = position_only
    move_group = generate_move_group_launch(config)
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', str(config.package_path / 'config/moveit.rviz')],
        parameters=[config.robot_description, config.robot_description_semantic,
                    config.robot_description_kinematics, config.planning_pipelines,
                    config.joint_limits, {'use_sim_time': True}],
    )
    commands = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_share / 'launch/arm_command_bridge.launch.py')),
        condition=IfCondition(LaunchConfiguration('arm_commands')),
        launch_arguments={
            'use_sim_time': 'true',
            'position_only': LaunchConfiguration('position_only'),
        }.items(),
    )
    return [
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup_share / 'launch/sim.launch.py')),
            launch_arguments={
                'with_arm': 'true', 'rviz': 'false',
                'arm_params_file': arm_params_file,
                'gui': LaunchConfiguration('gui'),
                'world': LaunchConfiguration('world'),
                'map_yaml': LaunchConfiguration('map_yaml'),
                'gazebo_params_file': LaunchConfiguration('gazebo_params_file'),
                'physics_params_file': LaunchConfiguration('physics_params_file'),
            }.items())]),
        GroupAction([
            SetParameter(name='use_sim_time', value=True),
            *move_group.entities,
            rviz,
            GroupAction([commands]),
        ]),
    ]


def generate_launch_description():
    bringup_share = Path(get_package_share_directory('bringup'))
    description_share = Path(get_package_share_directory('car_description'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('arm_commands', default_value='true',
                             description='Enable pose and gripper test commands through MoveIt'),
        DeclareLaunchArgument('position_only', default_value='true', choices=['true', 'false'],
                             description='Use position-only IK/goals; false requires full poses'),
        DeclareLaunchArgument(
            'world', default_value=str(description_share / 'world_model/model.world')),
        DeclareLaunchArgument(
            'map_yaml', default_value=str(description_share / 'map/edited/edited.yaml')),
        DeclareLaunchArgument(
            'gazebo_params_file',
            default_value=str(bringup_share / 'config/gazebo_moveit.yaml')),
        DeclareLaunchArgument(
            'arm_params_file', default_value=str(bringup_share / 'config/gazebo_arm.yaml'),
            description='Simulation-only joint limits, PID gains and initial positions'),
        DeclareLaunchArgument(
            'physics_params_file',
            default_value=str(bringup_share / 'config/gazebo_arm_physics.yaml'),
            description='Simulation-only ODE solver and constraint settings'),
        OpaqueFunction(function=_launch_combined),
    ])
