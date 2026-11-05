# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Real navigation using the existing car URDF and the installed map1 map."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_nodes(context):
    # Do not route this argument to sim.launch.py: that would share real command,
    # scan, odometry and TF topics with Gazebo in the same ROS domain.
    if LaunchConfiguration('start_sim').perform(context).lower() != 'false':
        raise RuntimeError(
            'navigation_real.launch.py requires start_sim:=false. '
            'Run simulation separately in a different ROS_DOMAIN_ID.')

    use_safety_gate = LaunchConfiguration('use_safety_gate').perform(context).lower() == 'true'
    if not use_safety_gate and LaunchConfiguration('enable').perform(context).lower() != 'true':
        raise RuntimeError(
            'use_safety_gate:=false requires enable:=true. '
            'For isolated checks, also set cmd_vel_topic to a debug topic.')

    map_name = LaunchConfiguration('map_yaml').perform(context).strip()
    if not map_name:
        raise RuntimeError(
            'Set map_yaml to an existing map YAML; the default is map1/room.yaml '
            'installed with car_description.')
    map_yaml = Path(map_name).expanduser().resolve()
    if not map_yaml.is_file():
        raise RuntimeError(f'Map YAML does not exist: {map_yaml}')

    share = Path(get_package_share_directory('car_description'))
    plan = Path(get_package_share_directory('plan'))
    controller = Path(get_package_share_directory('controller'))
    config = LaunchConfiguration('navigation_params_file')
    limits = LaunchConfiguration('motion_limits_file')
    # Reuse the original model unchanged. Reading its Gazebo tags does not start
    # Gazebo or load any hardware controller into robot_state_publisher.
    robot_description = (share / 'urdf/car_urdf.urdf').read_text(encoding='utf-8')
    robot_description = robot_description.replace('$(find car_description)', str(share))
    # Explicit last overrides prevent inherited simulation YAML from enabling
    # /clock even when the deployment parameter files are customized.
    real_time = {'use_sim_time': False}
    tracker_overrides = dict(real_time)
    # Keep the normal raw-topic YAML contract unless direct output is requested.
    tracker_overrides['allow_direct_cmd_vel'] = not use_safety_gate
    if not use_safety_gate:
        tracker_overrides['raw_cmd_topic'] = ParameterValue(
            LaunchConfiguration('cmd_vel_topic'), value_type=str)

    return [
        GroupAction(
            condition=IfCondition(LaunchConfiguration('publish_robot_model')),
            actions=[
                Node(
                    package='robot_state_publisher', executable='robot_state_publisher',
                    name='robot_state_publisher', output='screen', parameters=[{
                        'use_sim_time': False,
                        'robot_description': ParameterValue(robot_description, value_type=str),
                    }]),
                Node(
                    package='joint_state_publisher', executable='joint_state_publisher',
                    name='joint_state_publisher', output='screen', parameters=[real_time],
                    condition=IfCondition(LaunchConfiguration('publish_joint_states'))),
            ]),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen', parameters=[config, {
                'use_sim_time': False, 'yaml_filename': str(map_yaml),
            }]),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl', output='screen',
            parameters=[LaunchConfiguration('amcl_params_file'), config, real_time]),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_real_localization', output='screen', parameters=[{
                'use_sim_time': False, 'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }]),
        Node(
            package='plan', executable='planning_grid_node', name='planning_grid',
            output='screen', parameters=[
                str(plan / 'config/planning_grid.yaml'), config, real_time]),
        Node(
            package='plan', executable='astar_node', name='planner', output='screen',
            parameters=[LaunchConfiguration('planner_params_file'), config, limits, real_time]),
        Node(
            package='controller', executable='lqr_tracker_node', name='lqr_controller',
            output='screen', parameters=[
                LaunchConfiguration('lqr_params_file'), config, limits, tracker_overrides]),
        Node(
            package='controller', executable='safety_gate_node', name='safety_gate',
            condition=IfCondition(LaunchConfiguration('use_safety_gate')),
            output='screen', parameters=[
                str(controller / 'config/safety_gate.yaml'), config, limits, {
                    'use_sim_time': False,
                    'enable': ParameterValue(LaunchConfiguration('enable'), value_type=bool),
                    'cmd_vel_topic': ParameterValue(
                        LaunchConfiguration('cmd_vel_topic'), value_type=str),
                }]),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', str(share / 'config/navigation_real.rviz')],
            parameters=[real_time]),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('car_description'))
    plan = Path(get_package_share_directory('plan'))
    controller = Path(get_package_share_directory('controller'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_sim', default_value='false', choices=['false'],
            description='Real hardware only; Gazebo is never started by this entry.'),
        DeclareLaunchArgument(
            'map_yaml', default_value=str(share / 'map1/room.yaml'),
            description='Map YAML; defaults to the installed copy of car_ws/map1/room.yaml.'),
        DeclareLaunchArgument(
            'publish_robot_model', default_value='true', choices=['true', 'false'],
            description='Publish the original car_urdf.urdf; false reuses an external publisher.'),
        DeclareLaunchArgument(
            'publish_joint_states', default_value='true', choices=['true', 'false'],
            description='Default joint positions for display; false uses real /joint_states.'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'use_safety_gate', default_value='true', choices=['true', 'false'],
            description='false bypasses Gate and sends LQR directly to cmd_vel_topic; '
                        'requires enable:=true.'),
        DeclareLaunchArgument(
            'enable', default_value='false',
            description='With Gate, false publishes zero commands. Direct output requires true.'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument(
            'navigation_params_file', default_value=str(share / 'config/navigation_real.yaml')),
        DeclareLaunchArgument(
            'amcl_params_file', default_value=str(share / 'config/amcl_real.yaml')),
        DeclareLaunchArgument(
            'motion_limits_file', default_value=str(share / 'config/motion_limits_real.yaml')),
        DeclareLaunchArgument(
            'planner_params_file', default_value=str(plan / 'config/astar.yaml')),
        DeclareLaunchArgument(
            'lqr_params_file', default_value=str(controller / 'config/lqr_tracker.yaml')),
        OpaqueFunction(function=_launch_nodes),
    ])
