# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Real-entry routing smoke tests, isolated from hardware and real command topics."""
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import rclpy
from rcl_interfaces.srv import GetParameters


ROOT = Path(__file__).resolve().parents[3]
OUTPUT = '/real_routing_test/cmd_vel'


def wait(node, predicate, timeout=15.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not predicate():
        rclpy.spin_once(node, timeout_sec=0.05)
    assert predicate(), 'Timed out waiting for real-navigation routing'


def parameters(node, name, keys):
    client = node.create_client(GetParameters, f'/{name}/get_parameters')
    assert client.wait_for_service(timeout_sec=8.0)
    future = client.call_async(GetParameters.Request(names=keys))
    wait(node, future.done)
    result = dict(zip(keys, future.result().values))
    node.destroy_client(client)
    return result


@pytest.fixture(autouse=True)
def isolated_environment(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(180 + os.getpid() % 10))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(ROOT / 'log/real_navigation_routing'))


def arguments(*extra):
    return ['ros2', 'launch', 'car_description', 'navigation_real.launch.py',
            'rviz:=false', f'map_yaml:={ROOT / "map1/room.yaml"}',
            f'cmd_vel_topic:={OUTPUT}', *extra]


@pytest.mark.parametrize('guarded', [True, False])
def test_real_entry_routes_only_one_final_publisher(guarded):
    rclpy.init()
    node = rclpy.create_node('real_routing_test')
    log_path = ROOT / 'log/real_navigation_routing' / f'guarded_{guarded}.log'
    log_path.parent.mkdir(parents=True, exist_ok=True)
    extra = [] if guarded else ['use_safety_gate:=false', 'enable:=true']
    with log_path.open('w') as log:
        process = subprocess.Popen(arguments(*extra), stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            wait(node, lambda: 'lqr_controller' in node.get_node_names())
            values = parameters(node, 'lqr_controller',
                                ['raw_cmd_topic', 'allow_direct_cmd_vel', 'use_sim_time'])
            assert values['raw_cmd_topic'].string_value == (
                '/cmd_vel_raw' if guarded else OUTPUT)
            assert values['allow_direct_cmd_vel'].bool_value is not guarded
            assert not values['use_sim_time'].bool_value
            publisher = 'safety_gate' if guarded else 'lqr_controller'
            wait(node, lambda: [p.node_name for p in node.get_publishers_info_by_topic(OUTPUT)]
                 == [publisher])
            if guarded:
                gate = parameters(node, 'safety_gate', ['enable', 'cmd_vel_topic'])
                assert not gate['enable'].bool_value
                assert gate['cmd_vel_topic'].string_value == OUTPUT
            else:
                assert 'safety_gate' not in node.get_node_names()
                assert not node.get_publishers_info_by_topic('/cmd_vel_raw')
            assert not node.get_publishers_info_by_topic('/cmd_vel')
            assert process.poll() is None
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.0)
            node.destroy_node()
            rclpy.shutdown()
        assert process.returncode == 0, log_path.read_text()


def test_bypass_requires_explicit_enable_before_nodes_start():
    result = subprocess.run(arguments('use_safety_gate:=false'), capture_output=True,
                            timeout=15.0)
    assert result.returncode != 0
    assert 'requires enable:=true' in (result.stdout + result.stderr).decode()
