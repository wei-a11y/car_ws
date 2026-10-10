# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Complete simulated recovery deployment; do not also start navigation.launch."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from task_runtime.recovery_launch import confined_path, prepare_world, source_task_directory


def _setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    share = Path(get_package_share_directory('car_description'))
    plan = Path(get_package_share_directory('plan'))
    controller = Path(get_package_share_directory('controller'))
    task_root = source_task_directory()
    config = yaml.safe_load(Path(value('recovery_file')).read_text(encoding='utf-8'))
    if not isinstance(config, dict):
        raise ValueError('recovery_file must contain a configuration mapping')
    planning = config.get('planning', {})
    if planning.get('unknown_is_obstacle', True) is not True:
        raise ValueError('Recovery requires unknown_is_obstacle=true')
    routing = config.get('navigation', {})
    grid_overrides = {key: planning[key] for key in (
        'occupied_threshold', 'unknown_is_obstacle', 'robot_radius', 'safety_margin',
        'inflate_map_boundary', 'max_cells') if key in planning}
    grid_overrides['map_topic'] = routing.get('working_map_topic', '/task/working_map')
    planner_overrides = {'path_topic': '/task/candidate_path'}
    if 'max_cells' in planning:
        planner_overrides['max_cells'] = planning['max_cells']
    planner_overrides.update({f'post_processing.{key}': planning[key] for key in (
        'enable_shortcut', 'resample_spacing', 'shortcut_max_lookahead', 'max_output_points')
        if key in planning})
    navigation = value('navigation_params_file')
    runtime = confined_path(task_root, task_root / 'logs/runtime')
    runtime.mkdir(parents=True, exist_ok=True)
    gui_ini = runtime / 'gui.ini'
    if not gui_ini.exists():
        gui_ini.write_text('[geometry]\n', encoding='utf-8')
    actions = [
        SetEnvironmentVariable('GAZEBO_LOG_PATH', str(task_root / 'logs/gazebo')),
        SetEnvironmentVariable('GAZEBO_GUI_INI_FILE', str(gui_ini)),
        SetEnvironmentVariable('XDG_CACHE_HOME', str(task_root / 'logs/cache')),
    ]
    if value('start_sim').lower() in ('true', '1', 'yes'):
        world = prepare_world(value('world'), task_root)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/sim.launch.py')),
            launch_arguments={
                'world': str(world), 'map_yaml': value('map_yaml'),
                'gui': value('gui'), 'rviz': value('rviz'),
                'rviz_config': str(share / 'config/navigation.rviz'),
                'gazebo_params_file': navigation,
            }.items()))
    actions.extend([
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[value('amcl_params_file')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen', parameters=[{
                 'use_sim_time': True, 'autostart': True, 'node_names': ['amcl'],
             }]),
        Node(package='plan', executable='planning_grid_node', name='planning_grid',
             output='screen', parameters=[str(plan / 'config/planning_grid.yaml'),
                                          navigation, grid_overrides]),
        Node(package='plan', executable='astar_node', name='planner', output='screen',
             parameters=[str(plan / 'config/astar.yaml'), navigation,
                         planner_overrides]),
        Node(package='controller', executable='lqr_tracker_node', name='lqr_controller',
             output='screen', parameters=[
                 str(controller / 'config/lqr_tracker.yaml'), navigation,
                 {'path_topic': routing.get('execution_path_topic', '/plan')}]),
        Node(package='controller', executable='safety_gate_node', name='safety_gate',
             output='screen', parameters=[str(controller / 'config/safety_gate.yaml'),
                                          {'use_sim_time': True}]),
        Node(package='task', executable='recovery_task_node', name='task_manager',
             output='screen', parameters=[value('params_file'), {
                 'points_file': value('points_file'), 'recovery_file': value('recovery_file'),
                 'scenarios_file': value('scenarios_file'),
                 'path_topic': '/task/candidate_path',
                 'log_directory': str(confined_path(task_root, task_root / 'logs')),
             }]),
    ])
    return actions


def generate_launch_description():
    task = Path(get_package_share_directory('task'))
    car = Path(get_package_share_directory('car_description'))
    defaults = {
        'start_sim': ('true', 'false requires compatible Gazebo and map services already running'),
        'gui': ('true', 'Show Gazebo client'),
        'rviz': ('true', 'Show navigation RViz'),
        'world': (str(car / 'world_model/model.world'), 'Read-only source world'),
        'map_yaml': (str(car / 'map/edited/edited.yaml'), 'Map server YAML'),
        'amcl_params_file': (str(car / 'config/amcl.yaml'), 'AMCL parameters'),
        'navigation_params_file': (str(car / 'config/navigation_baseline.yaml'),
                                   'Existing navigation deployment overrides'),
        'params_file': (str(task / 'config/task.yaml'), 'Existing task parameters'),
        'points_file': (str(task / 'config/task_points.yaml'),
                        'Task points; read again each start'),
        'recovery_file': (str(task / 'config/recovery.yaml'),
                          'Recovery configuration (plain YAML)'),
        'scenarios_file': (str(task / 'config/recovery_scenarios.yaml'),
                           'Registered simulated objects'),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, (default, description) in defaults.items()],
        OpaqueFunction(function=_setup),
    ])
