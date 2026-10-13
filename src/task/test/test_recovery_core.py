# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Native planning, observation limits and whole-episode feasibility."""
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from task_runtime.recovery_core import GridModel, Obstacle, Pose3, RecoveryPlanner, ScanView, split_path


def config():
    return yaml.safe_load((Path(__file__).parents[1] / 'config/recovery.yaml').read_text())


def grid():
    return GridModel(dict(width=100, height=100, resolution=0.1,
                          origin_x=-5.0, origin_y=-5.0, origin_yaw=0.0),
                     [0]*10000, config()['planning'])


def obstacle(label='bag', x=0.05, y=0.05):
    return Obstacle('target', label, Pose3(x, y, 0.22), (0.24, 0.24, 0.44), 0.99, 100)


def scan(value=5.0):
    return SimpleNamespace(ranges=[value]*721, angle_min=-math.pi,
                           angle_increment=math.pi/360, range_min=0.1, range_max=8.0)


def test_split_bends_reversal_duplicates_final_yaw():
    segments = split_path([Pose3(0, 0), Pose3(1, 0), Pose3(1, 0),
                           Pose3(2, 0), Pose3(2, 1), Pose3(2, 0, yaw=1.2)])
    assert len(segments) == 3
    assert segments[0][-1].x == 2
    assert segments[-1][-1].yaw == 1.2
    assert split_path([Pose3(0, 0, yaw=0.4)])[0][0].yaw == 0.4


def test_ray_occlusion_invalid_and_bounded_no_return():
    view = ScanView(scan(2.0), Pose3(0, 0))
    assert view.classify(Pose3(1, 0)) == 'clear'
    assert view.classify(Pose3(2, 0)) == 'blocked'
    assert view.classify(Pose3(3, 0)) == 'unknown'
    for value in (math.nan, -math.inf, 9.0):
        assert ScanView(scan(value), Pose3(0, 0), True).classify(Pose3(1, 0)) == 'unknown'
    assert ScanView(scan(math.inf), Pose3(0, 0)).classify(Pose3(1, 0)) == 'unknown'
    permissive = ScanView(scan(math.inf), Pose3(0, 0), True)
    assert permissive.classify(Pose3(7, 0)) == 'clear'
    assert permissive.classify(Pose3(9, 0)) == 'unknown'


def test_corridor_detects_off_center_obstacle_and_preserves_stopping_margin():
    msg = scan()
    msg.ranges[360] = 2.0
    result = ScanView(msg, Pose3(0, 0)).inspect_segment(
        Pose3(0, 0), Pose3(4, 0), 0.4, self_clear_radius=0.4)
    assert result.state == 'blocked'
    assert result.clear_distance < 1.6
    assert result.blocking_points


def test_native_route_overlay_preserves_static_and_unknown():
    model = grid()
    base, goal, obj = Pose3(-2.05, 0.05), Pose3(2.05, 0.05), obstacle('stool')
    route = model.route(base, goal, [obj])
    assert route and any(abs(p.y-0.05) > 0.5 for p in route)
    changed = list(model.raw)
    changed[5050], changed[5051] = 100, -1
    occupied = GridModel(model.geometry, changed, model.config)
    restored = occupied.overlay([obj], exclude_ids=[obj.object_id])
    assert restored[5050:5052] == [100, -1]
    assert not occupied.placement_free(obj.pose, obj.size)


def test_stool_only_detours_and_unknown_class_never_grasps():
    for label in ('stool', 'unregistered'):
        selected, logs = RecoveryPlanner(config(), grid()).choose(
            Pose3(-2.05, 0.05), Pose3(2.05, 0.05), obstacle(label))
        assert selected and selected.kind == 'detour'
        assert any(entry.get('reason') == 'class_not_movable' for entry in logs)


def test_cost_can_select_carry_or_detour_without_forcing_policy():
    c = config()
    c['cost'].update(grasp_seconds=0, place_seconds=0, angular_speed=1000,
                     time_weight=1, work_weight=0)
    selected, logs = RecoveryPlanner(c, grid()).choose(
        Pose3(-2.05, 0.05), Pose3(2.05, 0.05), obstacle())
    assert selected and selected.kind == 'carry', logs
    assert selected.rollback_route and selected.routes['departure']
    c['cost']['grasp_seconds'] = 1000
    selected, _ = RecoveryPlanner(c, grid()).choose(
        Pose3(-2.05, 0.05), Pose3(2.05, 0.05), obstacle())
    assert selected.kind == 'detour'


def test_temporary_requires_both_side_reach_and_restoration():
    c = config()
    selected, logs = RecoveryPlanner(c, grid()).choose(
        Pose3(-2.05, 0.05), Pose3(2.05, 0.05), obstacle('box'), excluded={'detour'})
    assert selected and selected.kind == 'temporary', logs
    assert selected.temporary_pose and selected.original_pose == obstacle('box').pose
    assert selected.rollback_route
    c['geometry']['arm_reach'] = 0.3
    rejected, _ = RecoveryPlanner(c, grid()).choose(
        Pose3(-2.05, 0.05), Pose3(2.05, 0.05), obstacle('box'), excluded={'detour'})
    assert rejected is None


def test_observation_freshness_exclusions_and_bad_cost():
    model, c = grid(), config()
    selected, logs = RecoveryPlanner(c, model).choose(
        Pose3(-2.05, 0.05), Pose3(2.05, 0.05), obstacle(),
        excluded={'detour'}, now_ns=2000000000)
    assert selected is None
    assert any(entry.get('reason') == 'observation_stale_or_future' for entry in logs)
    rejected, _ = RecoveryPlanner(c, model).choose(
        Pose3(-2.05, 0.05), Pose3(2.05, 0.05), obstacle(),
        excluded={'detour', ('target', 'carry')})
    assert rejected is None
    c['cost']['linear_speed'] = 0
    with pytest.raises(ValueError):
        RecoveryPlanner(c, model)


def test_no_route_and_static_restoration_collision_reject_all_schemes():
    model = grid()
    raw = list(model.raw)
    # A full-height wall separates start and goal; the target is embedded in it.
    for row in range(100):
        raw[row * 100 + 50] = 100
    blocked = GridModel(model.geometry, raw, model.config)
    for label in ('bag', 'stool'):
        selected, logs = RecoveryPlanner(config(), blocked).choose(
            Pose3(-2.05, 0.05), Pose3(2.05, 0.05), obstacle(label))
        assert selected is None
        assert any(entry.get('reason') == 'no_route' for entry in logs)
    assert any(entry.get('reason') == 'class_not_movable' for entry in logs)


def test_unknown_map_policy_cannot_diverge_from_native_evaluator():
    model = grid()
    with pytest.raises(ValueError, match='unknown_is_obstacle'):
        GridModel(model.geometry, model.raw, {'unknown_is_obstacle': False})


def test_evaluation_preserves_anonymous_returns_without_pinching_removed_target():
    model, obj = grid(), obstacle()
    changed = list(model.raw)
    changed[5050] = -1  # Original unknown cell inside the object stays unknown.
    model = GridModel(model.geometry, changed, model.config)
    evaluated = model.with_unclassified_hits([(0.05, 0.05), (0.15, 0.05), (2.05, 0.05)], [obj])
    assert evaluated.raw[5050] == -1
    assert evaluated.raw[5051] == 0  # Known removable body is not baked into static data.
    assert evaluated.raw[5070] == 100  # Unclassified scan return remains an obstacle.
