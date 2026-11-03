# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Acceptance must reject incomplete or locally regressing experiments."""
from copy import deepcopy
import csv
import json
from pathlib import Path
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'tools'))
from analyze_joint_bezier import geometry_gate, same_geometry  # noqa: E402
from run_bezier_gazebo import dual_switches_match  # noqa: E402
from analyze_joint_speed import bounded_gate, curve_groups, interval_time  # noqa: E402
from joint_speed_profiles import reverse_profile  # noqa: E402
from run_joint_gazebo import validate_capture  # noqa: E402
from analyze_joint_gazebo import analyze, tracking_nonregression, tracking_statistics  # noqa: E402


def sample(k=2., q=10., rms=4.):
    return {'regions': [{'K_bound': k, 'Q_bound': q, 'nodal_rate_rms': rms}],
            'joins_valid': True, 'stops': 0, 'fallback': '', 'time': 40.}


SETTINGS = {'repeats': 5, 'acceptance': {'curvature_reduction': .1,
            'rate_rms_reduction': .15, 'regression_time_loss': .02}}
REGIONS = [{'range': [1., 4.], 'target': True}]


def test_rejects_missing_repeat_and_keeps_worst_run():
    old = [sample() for _ in range(5)]
    new = [sample(1.7, 9., 3.) for _ in range(5)]
    assert geometry_gate(old, new, REGIONS, SETTINGS, True)['pass']
    assert not geometry_gate(old, new[:-1], REGIONS, SETTINGS, True)['pass']
    new[-1]['regions'][0]['Q_bound'] = 11.
    assert not geometry_gate(old, new, REGIONS, SETTINGS, True)['pass']


def test_rejects_rate_regression_even_when_curvature_improves():
    old = [sample() for _ in range(5)]
    new = [sample(1.5, 8., 3.8) for _ in range(5)]
    assert not geometry_gate(old, new, REGIONS, SETTINGS, True)['pass']


def test_speed_pair_includes_continuous_certificates():
    row = {k: 0. for k in ('x', 'y', 'yaw', 'curvature', 'stop_required', 'segment_type',
                           'curvature_bound', 'curvature_rate_bound')}
    row['speed_limit'] = .3
    changed = deepcopy(row)
    changed['speed_limit'] = .2
    assert same_geometry([row], [changed])
    changed['curvature_rate_bound'] = 1.
    assert not same_geometry([row], [changed])


def test_dual_baseline_requires_actual_planner_and_executor_switches():
    planner = {'post_processing.enable_window_bezier': True,
               'post_processing.enable_turn_speed_coordination': True,
               'enable_dynamic_angular_budget': False}
    experiment = {'parameters': {
        '/planner': {'values': planner},
        '/lqr_controller': {'values': {'enable_dynamic_angular_budget': False}}}}
    assert dual_switches_match(experiment)
    experiment['parameters']['/lqr_controller']['values']['enable_dynamic_angular_budget'] = True
    assert not dual_switches_match(experiment)
    assert not dual_switches_match({'parameters': {}})


def test_speed_gate_rejects_local_slowdown_hidden_by_long_route():
    distance = np.array([0., 1., 2., 100.])
    before = np.full(4, .5)
    after = np.array([.5, .45, .5, .5])
    assert interval_time(distance, after, 0, 100) < interval_time(distance, before, 0, 100)*1.02
    assert not bounded_gate((distance, before), (distance, after), [(0., 2.)], .02)


def test_speed_gate_uses_squared_speed_and_independent_geometry_groups():
    distance = np.array([0., 1.])
    speed = np.array([0., 1.])
    assert abs(interval_time(distance, speed, 0., .25)-1.) < 1e-12
    distance = np.array([0., 1., 2., 3., 100.])
    before = np.full(5, .5)
    after = np.array([.5, .45, .5, .5, .5])
    rows = [{'x': float(x), 'y': 0., 'curvature_bound': .11 if i < 2 else 0.,
             'curvature_rate_bound': 0., 'stop_required': False} for i, x in enumerate(distance)]
    motion = {'v_nominal': .5, 'v_max': .6, 'angular_acceleration': 2.5,
              'angular_acceleration_fraction': .75, 'w_max': .6,
              'feedforward_angular_fraction': .75, 'lateral_acceleration': .1}
    spans = curve_groups(rows, motion)
    assert spans == [(0., 2.)]
    assert not bounded_gate((distance, before), (distance, after), spans, .02)


def test_reverse_profile_reverses_outgoing_certificates_and_signed_curvature():
    rows = [{'index': i, 'x': float(i), 'y': 0., 'yaw': .2, 'curvature': .1*i,
             'stop_required': int(i == 2), 'segment_type': 1, 'speed_limit': .3,
             'curvature_bound': i+1., 'curvature_rate_bound': i+2.} for i in range(3)]
    report = {'report': {'geometry_reference_s': [0., 1., 2.], 'windows': [{'id': 0}]}}
    reversed_rows, reverse_report = reverse_profile(rows, report)
    assert reversed_rows[0]['curvature_bound'] == rows[1]['curvature_bound']
    assert reversed_rows[1]['curvature_bound'] == rows[0]['curvature_bound']
    assert reversed_rows[0]['curvature'] == -rows[-1]['curvature']
    assert not reversed_rows[0]['stop_required'] and reversed_rows[-1]['stop_required']
    assert reverse_report['report']['geometry_reference_s'] == [0., 1., 2.]
    assert reverse_report['report']['windows'] == []
    assert report['report']['windows'] == [{'id': 0}]


def test_closed_loop_capture_rejects_missing_map_without_losing_other_checks(tmp_path):
    epoch = tmp_path/'record/run_test/path_0001'
    epoch.mkdir(parents=True)
    (epoch/'experiment.json').write_text(json.dumps({'map': None, 'parameters': {}}))
    row = {k: 0. for k in ('x', 'y', 'yaw', 'curvature', 'stop_required', 'segment_type',
                           'curvature_bound', 'curvature_rate_bound', 'speed_limit')}
    with (epoch/'planned_path.csv').open('w') as file:
        writer = csv.DictWriter(file, fieldnames=list(row))
        writer.writeheader()
        writer.writerow(row)
    (epoch/'actual_path.csv').write_text('position_error,lateral_error\n0.01,0.005\n')
    limits = {'max_position_error': .1, 'max_lateral_error': .1}
    result = validate_capture(tmp_path, [row], 'A', 'expected', limits)
    assert not result['map_matches'] and 'map metadata' in result['capture_error']
    assert result['profile_matches'] and result['tracking_bounds_match']
    (epoch/'actual_path.csv').write_text('position_error,lateral_error\nnan,0.005\n')
    result = validate_capture(tmp_path, [row], 'A', 'expected', limits)
    assert not result['tracking_bounds_match'] and 'nonfinite' in result['capture_error']


def test_closed_loop_capture_rejects_missing_epoch(tmp_path):
    result = validate_capture(tmp_path, [], 'A', 'expected', {})
    assert not any(result[k] for k in (
        'configuration_matches', 'map_matches', 'profile_matches', 'tracking_bounds_match'))
    assert 'found 0' in result['capture_error']


def test_closed_loop_tracking_metrics_use_numeric_state_and_exclude_alignment():
    data = pd.DataFrame({'state': [1, 2, 2, 2], 'geometry_valid': [1, 1, 0, 1],
                         'curvature': [.2, .2, .2, 0.], 'lateral_error': [9., .01, 9., .02],
                         'heading_error': [9., .02, 9., .04], 'v_actual': [9., .3, 9., .4],
                         'v_ref': [0., .29, 0., .39]})
    metrics = tracking_statistics(data, .1)
    assert metrics['tracking_samples'] == 2 and metrics['curved_tracking_samples'] == 1
    assert metrics['lateral_p95'] == .01
    assert abs(metrics['heading_p95']-.039) < 1e-12
    assert abs(metrics['speed_p95']-.01) < 1e-12


def test_closed_loop_rejects_single_tracking_regression_hidden_by_median():
    baseline = [{'repeat': i, 'lateral_p95': .01, 'heading_p95': .02, 'speed_p95': .03}
                for i in (1, 2, 3)]
    groups = {g: deepcopy(baseline) for g in 'ABCD'}
    limits = {f'{s}_p95_absolute_increase': v for s, v in (
        ('lateral', .002), ('heading', .01), ('speed', .005))}
    limits.update({f'{s}_p95_relative_increase': .1 for s in ('lateral', 'heading', 'speed')})
    assert all(tracking_nonregression(groups, limits).values())
    groups['D'][-1]['lateral_p95'] = .013
    assert not tracking_nonregression(groups, limits)['D_lateral_nonregression']


def test_closed_loop_incomplete_capture_is_reported_as_failure(tmp_path):
    experiment = {'configuration': {'repeats_per_direction': 3},
                  'tracking_limits': {}, 'probe_limit': None}
    (tmp_path/'experiment.json').write_text(json.dumps(experiment))
    trial = tmp_path/'A_outward_1'
    trial.mkdir()
    recorded = {'variant': 'A', 'direction': 'outward', 'repeat': 1, 'goal_reached': False}
    (trial/'result.json').write_text(json.dumps(recorded))
    result = analyze(tmp_path)
    assert not result['passed'] and not result['routes']['outward']['complete']
    assert not result['trials'][0]['valid']
    assert 'found 0' in result['trials'][0]['analysis_error']
