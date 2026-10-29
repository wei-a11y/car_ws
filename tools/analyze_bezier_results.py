#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Freeze offline platform windows before A/B, then evaluate every recorded trial."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pandas as pd
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/controller/scripts'))
from tracking_analysis import load_config, summarize  # noqa: E402


def progress(path):
    return np.r_[0, np.cumsum(np.hypot(np.diff(path['x']), np.diff(path['y'])))]


def project(point, path):
    xy = path[['x', 'y']].to_numpy()
    delta = np.diff(xy, axis=0)
    fraction = np.clip(np.sum((point - xy[:-1]) * delta, axis=1)
                       / np.sum(delta**2, axis=1), 0, 1)
    distances = np.linalg.norm(xy[:-1] + fraction[:, None] * delta - point, axis=1)
    index = np.argmin(distances)
    return float(progress(path)[index] + fraction[index] * np.linalg.norm(delta[index]))


def freeze(offline, destination, config):
    cases = [json.loads(line) for line in (offline / 'offline.jsonl').read_text().splitlines()]
    frozen = {'offline_directory': str(offline.resolve()),
              'offline_manifest_sha256': hashlib.sha256(
                  (offline / 'manifest.json').read_bytes()).hexdigest(), 'scenarios': {}}
    for case in cases:
        if case['flags'] != 7:
            continue
        name = case['scenario']
        base = pd.read_csv(offline / f'{name}_0.csv')
        candidate = pd.read_csv(offline / f'{name}_7.csv')
        speed = pd.read_csv(offline / f'{name}_7_speed.csv')
        windows = []
        for number, window in enumerate(case['report']['windows']):
            if 'begin_index' not in window:
                continue
            a, b = window['begin_index'], window['end_index']
            curved = [i for i in range(a, b) if base.iloc[i]['segment_type'] != 0]
            if not curved:
                continue
            start, end = (base.iloc[i][['x', 'y']].to_numpy(dtype=float)
                          for i in (curved[0], curved[-1] + 1))
            low, high = project(start, candidate), project(end, candidate)
            samples = speed[(speed['distance'] >= low) & (speed['distance'] <= high)]['speed']
            endpoints = np.sqrt(np.interp([low, high], speed['distance'], speed['speed']**2))
            minimum = float(np.min(np.r_[samples, endpoints]))
            windows.append({'id': number+1, 'start_xy': start.tolist(), 'end_xy': end.tolist(),
                            'offline_minimum_speed': minimum,
                            'eligible': minimum >= config['target_turn_speed_min'],
                            'offline_result': window['result']})
        frozen['scenarios'][name] = {
            'map_fingerprint': case['report']['grid_fingerprint_fnv1a64'], 'windows': windows}
    destination.write_text(json.dumps(frozen, indent=2)+'\n')


def analyze(root, config):
    frozen = json.loads((root / 'frozen_windows.json').read_text())
    results = []
    for result_file in sorted(root.glob('*/result.json')):
        trial = json.loads(result_file.read_text())
        epochs = sorted(result_file.parent.glob('record/run_*/path_*/actual_path.csv'))
        if len(epochs) != 1:
            trial['analysis_error'] = 'Expected exactly one recorded path epoch'
            results.append(trial)
            continue
        epoch = epochs[0].parent
        planned = pd.read_csv(epoch / 'planned_path.csv')
        actual = pd.read_csv(epoch / 'actual_path.csv')
        _, summary = summarize(planned, actual, config)
        (epoch / 'tracking_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        metadata = json.loads((epoch / 'experiment.json').read_text())
        report = json.loads((epoch / 'planning_report.json').read_text())
        scenario = frozen['scenarios'][trial['scenario']['name']]
        parameters = metadata['parameters']
        planner = parameters['/planner'].get('values', {})
        controller = parameters['/lqr_controller'].get('values', {})
        expected = trial['variant'] == 'combined'
        trial['mode_matches'] = (
            planner.get('post_processing.enable_window_bezier') == expected and
            planner.get('post_processing.enable_turn_speed_coordination') == expected and
            planner.get('enable_dynamic_angular_budget') == expected and
            controller.get('enable_dynamic_angular_budget') == expected)
        trial['source_tree_sha256'] = metadata['source'].get('source_tree_sha256')
        keys = ('v_nominal', 'v_max', 'w_max', 'linear_acceleration', 'linear_deceleration',
                'angular_acceleration', 'lateral_acceleration',
                'feedforward_angular_fraction', 'angular_acceleration_fraction', 'min_turn_speed',
                'q_lateral', 'q_heading', 'r_angular', 'speed_pid.kp',
                'speed_pid.ki', 'speed_pid.kd')
        physical = {name: {k: node.get('values', {})[k] for k in keys
                           if k in node.get('values', {})}
                    for name, node in parameters.items()}
        physical['grid'] = {k: parameters['/planning_grid'].get('values', {}).get(k)
                            for k in ('robot_radius', 'safety_margin', 'occupied_threshold')}
        trial['physical_signature'] = hashlib.sha256(
            json.dumps(physical, sort_keys=True).encode()).hexdigest()
        trial['maximum_tracking_error'] = float(actual['position_error'].max())
        trial['tracking_bound_pass'] = (
            trial['maximum_tracking_error'] <= controller['max_tracking_error'])
        trial.update(recorded_tracking_duration_sec=summary['duration_sec'],
                     duration_sec=trial['duration_ros'],
                     events=summary['smoothness']['v_actual']['significant_acceleration_events'],
                     lateral_p95=summary['curve_lateral_m']['p95_abs'],
                     unexpected_stop_count=summary['unexpected_stop_count'],
                     jerk_p95=summary['smoothness']['v_actual']['jerk_p95_abs'],
                     snapshot_matches_plan=(metadata['map']['fingerprint_fnv1a64'] ==
                                            report['grid_fingerprint_fnv1a64'] ==
                                            scenario['map_fingerprint']),
                     windows=[], epoch=str(epoch))
        for window in scenario['windows']:
            low = project(np.array(window['start_xy']), planned)
            high = project(np.array(window['end_xy']), planned)
            selected = actual[(actual['progress'] >= low) & (actual['progress'] <= high)
                              & (actual['state'] == 2) & (actual['geometry_valid'] > 0)]
            assessment = dict(window, samples=len(selected))
            if len(selected):
                target = float(selected['v_ref'].median())
                assessment.update(reference_min=float(selected['v_ref'].min()),
                                  reference_range=float(
                                      selected['v_ref'].max()-selected['v_ref'].min()),
                                  platform_target=target,
                                  platform_error_p95=float(np.percentile(
                                      np.abs(selected['v_actual']-target), 95)))
                assessment['platform_pass'] = (
                    assessment['reference_min'] >= config['target_turn_speed_min'] and
                    assessment['reference_range'] <= config['platform_speed_error_tolerance'] and
                    assessment['platform_error_p95'] <= config['platform_speed_error_tolerance'])
            else:
                assessment['platform_pass'] = False
            trial['windows'].append(assessment)
        if 'linear_angular_demand' in actual:
            tracking = actual[(actual['state'] == 2) & (actual['geometry_valid'] > 0)]
            demand = tracking['linear_angular_demand'] + tracking['curvature_angular_demand']
            trial['maximum_feedforward_demand'] = float(demand.max())
            trial['feedforward_bound_pass'] = trial['maximum_feedforward_demand'] <= (
                controller['angular_acceleration'] * controller['angular_acceleration_fraction']
                + 1e-8)
        results.append(trial)
    bench = yaml.safe_load(Path('src/plan/config/bezier_benchmark.yaml').read_text())['benchmark']
    expected = 2 * len(bench['scenarios']) * bench['repeats_per_direction']
    acceptance = {'complete': len(results) == expected,
                  'all_modes_match': bool(results) and all(t.get('mode_matches') for t in results),
                  'same_source_tree': len({t.get('source_tree_sha256') for t in results}) == 1,
                  'same_physical_parameters': len({t.get('physical_signature')
                                                   for t in results}) == 1,
                  'execution_bounds_pass': bool(results) and all(
                      t.get('tracking_bound_pass') and t.get('feedforward_bound_pass')
                      for t in results),
                  'all_goals_reached': bool(results) and all(t['goal_reached'] for t in results),
                  'all_map_snapshots_match': bool(results) and all(
                      t.get('snapshot_matches_plan', False) for t in results), 'routes': {}}
    all_platforms = []
    for scenario in bench['scenarios']:
        name = scenario['name']
        groups = {v: [t for t in results if t['variant'] == v and t['scenario']['name'] == name
                      and 'duration_sec' in t] for v in ('baseline', 'combined')}
        if any(len(group) != bench['repeats_per_direction'] for group in groups.values()):
            acceptance['routes'][name] = {'complete': False}
            continue
        base, new = groups['baseline'], groups['combined']
        old_time = float(np.median([t['duration_sec'] for t in base]))
        new_time = float(np.median([t['duration_sec'] for t in new]))
        old_events, new_events = (sum(t['events'] for t in group) for group in (base, new))
        old_error = float(np.median([t['lateral_p95'] for t in base]))
        new_error = float(np.median([t['lateral_p95'] for t in new]))
        platforms = [w['platform_pass'] for t in new for w in t['windows'] if w['eligible']]
        all_platforms.extend(platforms)
        acceptance['routes'][name] = {
            'complete': True, 'baseline_time_median': old_time, 'combined_time_median': new_time,
            'time_ratio': new_time/old_time, 'baseline_events': old_events,
            'combined_events': new_events,
            'event_reduction': 1-new_events/old_events if old_events else None,
            'baseline_lateral_p95_median': old_error, 'combined_lateral_p95_median': new_error,
            'time_pass': new_time <= old_time*config['acceptance_time_ratio'],
            'events_pass': new_events <= old_events*(1-config['acceptance_event_reduction'])
            if old_events else new_events == 0,
            'lateral_pass': new_error-old_error <= max(
                config['acceptance_lateral_absolute'],
                old_error*config['acceptance_lateral_relative']),
            'no_new_stops': all(t['unexpected_stop_count'] == 0 for t in new),
            'eligible_window_observations': len(platforms), 'platforms_pass': bool(platforms)
            and all(platforms)}
    acceptance['platforms_pass'] = bool(all_platforms) and all(all_platforms)
    acceptance['passed'] = (acceptance['complete'] and acceptance['all_goals_reached']
                            and acceptance['all_map_snapshots_match']
                            and acceptance['all_modes_match']
                            and acceptance['same_source_tree']
                            and acceptance['same_physical_parameters']
                            and acceptance['execution_bounds_pass']
                            and acceptance['platforms_pass']
                            and all(r.get('complete') and all(r.get(k) for k in (
                                'time_pass', 'events_pass', 'lateral_pass', 'no_new_stops'))
                                    for r in acceptance['routes'].values()))
    document = {'acceptance': acceptance, 'trials': results, 'frozen_windows': frozen}
    (root / 'comparison.json').write_text(json.dumps(document, indent=2, allow_nan=False)+'\n')
    print(json.dumps(acceptance, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gazebo-dir', required=True, type=Path)
    parser.add_argument('--offline-dir', type=Path)
    parser.add_argument('--freeze-only', action='store_true')
    parser.add_argument('--config', default='src/controller/config/path_tracking.yaml')
    args = parser.parse_args()
    config = load_config(args.config)['analysis']
    destination = args.gazebo_dir / 'frozen_windows.json'
    if not destination.exists():
        if args.offline_dir is None:
            parser.error('--offline-dir is required to freeze windows before A/B')
        freeze(args.offline_dir, destination, config)
    if not args.freeze_only:
        analyze(args.gazebo_dir, config)


if __name__ == '__main__':
    main()
