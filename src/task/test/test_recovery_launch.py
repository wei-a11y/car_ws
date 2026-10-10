# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
import xml.etree.ElementTree as ET

import pytest

from task_runtime.recovery_launch import confined_path, prepare_world, source_task_directory


def test_world_copy_preserves_source_and_resolves_relative_assets(tmp_path):
    original = tmp_path / 'original.world'
    content = ('<sdf version="1.7"><world name="test"><include>'
               '<uri>../assets/model.sdf</uri></include><include>'
               '<uri>model://ground_plane</uri></include></world></sdf>')
    original.write_text(content)
    root = tmp_path / 'task'
    root.mkdir()
    target = prepare_world(original, root)
    assert original.read_text() == content
    assert target.is_relative_to(root)
    world = ET.parse(target).getroot().find('world')
    assert [u.text for u in world.iter('uri')] == [
        (original.parent / '../assets/model.sdf').resolve().as_uri(),
        'model://ground_plane']
    plugins = world.findall('plugin')
    assert len(plugins) == 1
    assert plugins[0].get('filename') == 'libgazebo_ros_state.so'
    assert plugins[0].findtext('ros/namespace') == '/task/gazebo'
    assert plugins[0].findtext('update_rate') == '30.0'


def test_world_writes_cannot_escape_or_overwrite_source(tmp_path):
    original = tmp_path / 'original.world'
    original.write_text('<sdf><world name="a"/></sdf>')
    root = tmp_path / 'task'
    root.mkdir()
    with pytest.raises(ValueError, match='inside'):
        prepare_world(original, root, tmp_path / 'escape.world')
    (root / 'escape').symlink_to(tmp_path, target_is_directory=True)
    with pytest.raises(ValueError, match='inside'):
        prepare_world(original, root, root / 'escape/elsewhere.world')
    local = root / 'original.world'
    local.write_text(original.read_text())
    with pytest.raises(ValueError, match='differ'):
        prepare_world(local, root, local)


def test_rejects_conflicting_plugin_and_multiple_worlds(tmp_path):
    original = tmp_path / 'original.world'
    root = tmp_path / 'task'
    root.mkdir()
    original.write_text('<sdf><world><plugin name="other" '
                        'filename="libgazebo_ros_state.so"/></world></sdf>')
    with pytest.raises(ValueError, match='namespace'):
        prepare_world(original, root)
    original.write_text('<sdf><world/><world/></sdf>')
    with pytest.raises(ValueError, match='exactly one'):
        prepare_world(original, root)


def test_source_location_and_confined_log_root():
    root = source_task_directory()
    assert root.name == 'task' and root.parent.name == 'src'
    assert confined_path(root, root / 'logs') == root / 'logs'


def test_launch_preserves_parameter_layers_and_routes_through_task(monkeypatch):
    import importlib.util
    from launch import LaunchContext
    launch_file = source_task_directory() / 'launch/obstacle_recovery.launch.py'
    spec = importlib.util.spec_from_file_location('recovery_launch_under_test', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update({
        'start_sim': 'false',
        'recovery_file': str(source_task_directory() / 'config/recovery.yaml'),
        'navigation_params_file': '/original/navigation.yaml',
        'amcl_params_file': '/original/amcl.yaml',
        'params_file': '/original/task.yaml',
        'points_file': '/user/task_points.yaml',
        'scenarios_file': '/user/scenarios.yaml',
    })
    nodes = {node['name']: node for node in module._setup(context) if isinstance(node, dict)}
    grid = nodes['planning_grid']['parameters']
    assert grid[0].endswith('/config/planning_grid.yaml')
    assert grid[1] == '/original/navigation.yaml'
    assert grid[2]['map_topic'] == '/task/working_map'
    assert grid[2]['safety_margin'] == 0.10
    assert nodes['planner']['parameters'][-1]['path_topic'] == '/task/candidate_path'
    assert nodes['lqr_controller']['parameters'][-1]['path_topic'] == '/plan'
    assert nodes['task_manager']['parameters'][-1]['path_topic'] == '/task/candidate_path'
    safety = nodes['safety_gate']['parameters']
    assert len(safety) == 2 and safety[-1] == {'use_sim_time': True}
    assert safety[0].endswith('/config/safety_gate.yaml')
