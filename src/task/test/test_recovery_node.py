# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Recovery transaction invariants; ROS integration tests follow below."""
import json
import time
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from nav_msgs.msg import Path

from task_runtime.recovery_core import Obstacle, Pose3
from task_runtime.recovery_node import (
    RecoveryTaskManager, RestorationLedger, pose_message,
)


def test_original_pose_survives_retry_and_process_restart(tmp_path):
    original = Obstacle('bag', 'bag', Pose3(1, 2, 0.2, 0.3), (0.2, 0.2, 0.4), 1., 1)
    displaced = Obstacle('bag', 'bag', Pose3(5, 6, 1.2, 1.), (0.2, 0.2, 0.4), 1., 2)
    ledger = RestorationLedger(tmp_path)
    ledger.add(original, 'round1')
    ledger.add(displaced, 'round1')
    reloaded = RestorationLedger(tmp_path)
    assert reloaded.items['bag']['original'] == dict(x=1, y=2, z=0.2, yaw=0.3)
    assert json.loads(ledger.filename.read_text()) == reloaded.items
    reloaded.remove('bag')
    assert RestorationLedger(tmp_path).items == {}


def test_old_and_duplicate_segment_arrival_cannot_advance_mission():
    end = pose_message(Pose3(1., 2., 0., 0.3), 'map', Time(sec=1))
    callbacks = []
    node = SimpleNamespace(mode='ACTIVE', nav_state='DRIVE_SEGMENT', execution_stamp=10**9,
                           active_path=Path(header=end.header, poses=[end]), frame='map',
                           pending_chunk=[Pose3(0., 2.), Pose3(1., 2.)],
                           at_endpoint=lambda: True, stop_then=callbacks.append,
                           chunk_completed=lambda: None)
    old = pose_message(Pose3(1., 2., 0., 0.3), 'map', Time(sec=0, nanosec=99))
    RecoveryTaskManager.on_reached(node, old)
    assert not callbacks
    RecoveryTaskManager.on_reached(node, end)
    RecoveryTaskManager.on_reached(node, end)
    assert len(callbacks) == 1
    assert node.execution_stamp is None


def test_release_barrier_prevents_old_path_publication():
    node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda:
                           SimpleNamespace(nanoseconds=100)), release_ros=100,
                           last_request_stamp=90)
    # Any observation/publisher access would fail this minimal peer: barrier must return first.
    RecoveryTaskManager.publish_chunk(node)


def test_unresolved_restoration_rejects_start_before_base_release(tmp_path):
    ledger = RestorationLedger(tmp_path)
    ledger.add(Obstacle('bag', 'bag', Pose3(1, 2, 0.2), (0.2, 0.2, 0.4), 1., 1), 'old')
    node = SimpleNamespace(ledger=ledger)
    response = SimpleNamespace(success=False, message='')
    result = RecoveryTaskManager.on_start(node, None, response)
    assert not result.success
    assert 'unresolved restoration' in result.message


def test_ledger_requires_distinct_consecutive_restoration_observations(tmp_path):
    ledger = RestorationLedger(tmp_path)
    original = Obstacle('bag', 'bag', Pose3(1, 2, 0.2), (0.2, 0.2, 0.4), 1., 1)
    ledger.add(original, 'old')
    current = [original]
    node = SimpleNamespace(ledger=ledger, backend_config={'verification_samples': 3},
                           backend=SimpleNamespace(held_id=None,
                               get_observation=lambda _: current[0]),
                           restoration_counts={}, restoration_stamps={})
    for _ in range(4):
        RecoveryTaskManager.verify_ledger(node)
    assert ledger.items
    for stamp in (2, 3):
        current[0] = Obstacle('bag', 'bag', original.pose, original.size, 1., stamp)
        RecoveryTaskManager.verify_ledger(node)
    assert not ledger.items


def test_temporary_placement_waits_for_new_post_lift_scan():
    placements = []
    node = SimpleNamespace(
        nav_state='WAIT_MANIPULATION_SCAN', state_started=time.monotonic(),
        minimum_scan_stamp=10**9, scan=SimpleNamespace(header=SimpleNamespace(stamp=Time(sec=1))),
        chosen=SimpleNamespace(temporary_pose=Pose3(1, 1)),
        n={'inspection_timeout_sec': 1.0}, begin_place=placements.append)
    RecoveryTaskManager.navigation_tick(node)
    assert not placements
    node.scan.header.stamp = Time(sec=1, nanosec=1)
    RecoveryTaskManager.navigation_tick(node)
    assert placements == [Pose3(1, 1)]


def test_second_obstacle_while_holding_cannot_start_another_grasp():
    failures = []
    node = SimpleNamespace(
        refresh_observation=lambda: None, pending_failure=None,
        backend=SimpleNamespace(held_id='first'), fail=lambda *args: failures.append(args))
    # No planner/grasp interfaces are supplied: encountering the second obstacle
    # must hand control to restoration recovery before any new manipulation.
    RecoveryTaskManager.handle_blockage(node, None, Pose3(0, 0), Pose3(1, 0))
    assert failures[0][0] == 'SECOND_OBSTACLE'


def test_exhausted_grasp_waits_for_original_pose_verification_before_replanning():
    states = []
    node = SimpleNamespace(
        base_pose=lambda: Pose3(0, 0), stopped=lambda: True, operation_pose=Pose3(0, 0),
        p={'position_tolerance': 0.05, 'heading_tolerance': 0.1},
        backend=SimpleNamespace(result={'success': False, 'code': 'GRASP_NOT_FOLLOWING'},
                                held_id=None),
        event=lambda *args, **kwargs: True, nav_state='GRASP', grasp_attempts=3,
        n={'max_grasp_attempts': 3}, excluded=set(),
        chosen=SimpleNamespace(object_id='bag', kind='carry'), transition=states.append)
    RecoveryTaskManager.operation_tick(node)
    assert states == ['VERIFY_UNMOVED']
    assert ('bag', 'carry') in node.excluded
