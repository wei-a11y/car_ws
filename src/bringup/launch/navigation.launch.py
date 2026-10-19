# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('car_description'))
    bringup = Path(get_package_share_directory('bringup'))
    plan = Path(get_package_share_directory('plan'))
    controller = Path(get_package_share_directory('controller'))
    config = LaunchConfiguration('navigation_params_file')
    return LaunchDescription([
        DeclareLaunchArgument('start_sim', default_value='true',
                             description='false reuses an already running sim/map server'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true',
                             description='Open the map-frame navigation RViz'),
        DeclareLaunchArgument('world', default_value=str(share / 'world_model/model.world')),
        DeclareLaunchArgument('map_yaml', default_value=str(share / 'map/edited/edited.yaml')),
        DeclareLaunchArgument('amcl_params_file', default_value=str(share / 'config/amcl.yaml')),
        DeclareLaunchArgument('navigation_params_file',
                             default_value=str(share / 'config/navigation_baseline.yaml')),
        DeclareLaunchArgument('enable', default_value='true',
                             description='false inhibits commands; never bypasses Safety Gate'),
        DeclareLaunchArgument('controller_type',default_value='dwa',
                              description='Controller selection: dwa or lqr'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel',
                             description='Override with a debug topic for isolated testing'),
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch/sim.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_sim')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'map_yaml': LaunchConfiguration('map_yaml'),
                'gui': LaunchConfiguration('gui'),
                'rviz': 'false',
                'gazebo_params_file': config,
            }.items())]),
        Node(package='rviz2', executable='rviz2', name='navigation_rviz', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(share / 'config/navigation.rviz')],
             parameters=[{'use_sim_time': True}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[LaunchConfiguration('amcl_params_file')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen', parameters=[{
                 'use_sim_time': True, 'autostart': True, 'node_names': ['amcl'],
             }]),
        Node(package='plan', executable='planning_grid_node', name='planning_grid',
             output='screen', parameters=[str(plan / 'config/planning_grid.yaml'), config]),
        Node(package='plan', executable='astar_node', name='planner', output='screen',
             parameters=[str(plan / 'config/astar.yaml'), config]),

     Node(
     package='controller',
     executable='dwa_tracker_node',
     name='dwa_controller',
     output='screen',
     condition=IfCondition(
          PythonExpression([
               "'", LaunchConfiguration('controller_type'),
               "' == 'dwa'"
          ])
     ),
     parameters=[
          str(controller / 'config/dwa_tracker.yaml'),
          config
     ]
     ),

     Node(
     package='controller',
     executable='lqr_tracker_node',
     name='lqr_controller',
     output='screen',
     condition=IfCondition(
          PythonExpression([
               "'", LaunchConfiguration('controller_type'),
               "' == 'lqr'"
          ])
     ),
     parameters=[
          str(controller / 'config/lqr_tracker.yaml'),
          config,
     ]
     ),
        Node(package='controller', executable='safety_gate_node', name='safety_gate',
             output='screen', parameters=[str(controller / 'config/safety_gate.yaml'), {
                 'use_sim_time': True,
                 'enable': ParameterValue(LaunchConfiguration('enable'), value_type=bool),
                 'cmd_vel_topic': ParameterValue(
                     LaunchConfiguration('cmd_vel_topic'), value_type=str),
             }]),
    ])
