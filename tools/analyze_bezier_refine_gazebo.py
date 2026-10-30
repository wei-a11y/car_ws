#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Compare current/optimized Bezier tracking with both speed features disabled."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/controller/scripts'))
from tracking_analysis import load_config, summarize  # noqa: E402


def analyze(root):
    experiment = json.loads((root/'geometry_experiment.json').read_text())
    settings = load_config()['analysis']
    config = experiment['configuration']
    trials = []
    for file in sorted(root.glob('*/result.json')):
        trial = json.loads(file.read_text())
        epochs = sorted(file.parent.glob('record/run_*/path_*/actual_path.csv'))
        trial['valid'] = False
        if len(epochs) == 1:
            directory = epochs[0].parent
            planned = pd.read_csv(directory/'planned_path.csv')
            actual = pd.read_csv(epochs[0])
            _, summary = summarize(planned, actual, settings)
            (directory/'tracking_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
            metadata = json.loads((directory/'experiment.json').read_text())
            report = json.loads((directory/'planning_report.json').read_text())
            params = metadata['parameters']
            planner = params['/planner'].get('values', {})
            controller = params['/lqr_controller'].get('values', {})
            keys = ('v_nominal', 'v_max', 'w_max', 'linear_acceleration', 'linear_deceleration',
                    'angular_acceleration', 'lateral_acceleration', 'feedforward_angular_fraction',
                    'angular_acceleration_fraction', 'min_turn_speed', 'q_lateral', 'q_heading',
                    'r_angular', 'speed_pid.kp', 'speed_pid.ki', 'speed_pid.kd')
            physical = {name: {k: values.get('values', {}).get(k) for k in keys}
                        for name, values in params.items()}
            mode = (planner.get('post_processing.enable_window_bezier') is True and
                    planner.get('post_processing.enable_turn_speed_coordination') is False and
                    planner.get('enable_dynamic_angular_budget') is False and
                    controller.get('enable_dynamic_angular_budget') is False)
            expected = config['refine']['compatibility_defaults'].copy()
            if trial['variant'] == 'optimized_bezier':
                expected.update(experiment['overrides'])
            mode = mode and all(planner.get('post_processing.'+k) == v
                                for k, v in expected.items())
            trial.update(mode_matches=mode, lateral_p95=summary['curve_lateral_m']['p95_abs'],
                         lateral_max=summary['lateral_m']['max_abs'],
                         unexpected_stops=summary['unexpected_stop_count'],
                         max_position_error=float(actual['position_error'].max()),
                         duration_sec=trial['duration_ros'],
                         physical_signature=hashlib.sha256(json.dumps(
                             physical, sort_keys=True).encode()).hexdigest(),
                         source_tree=metadata['source'].get('source_tree_sha256'),
                         map_fingerprint=report['grid_fingerprint_fnv1a64'],
                         snapshot_matches=report['grid_fingerprint_fnv1a64'] ==
                         metadata['map']['fingerprint_fnv1a64'])
            trial['valid'] = (mode and trial['snapshot_matches'] and bool(trial['source_tree'])
                              and trial['max_position_error'] <= controller['max_tracking_error'])
        trials.append(trial)
    routes = {}
    for scenario in config['scenarios']:
        groups = {v: [r for r in trials if r['variant'] == v and
                      r['scenario']['name'] == scenario['name']] for v in (
                          'current_bezier', 'optimized_bezier')}
        complete = all(len(g) == config['repeats_per_direction'] for g in groups.values())
        result = {'complete': complete}
        if complete and all(t['valid'] for g in groups.values() for t in g):
            old, new = groups['current_bezier'], groups['optimized_bezier']
            old_error, new_error = (float(np.median([r['lateral_p95'] for r in g]))
                                    for g in (old, new))
            old_time, new_time = (float(np.median([r['duration_sec'] for r in g]))
                                  for g in (old, new))
            result.update(lateral_p95_before=old_error, lateral_p95_after=new_error,
                          lateral_max_before=max(r['lateral_max'] for r in old),
                          lateral_max_after=max(r['lateral_max'] for r in new),
                          time_before=old_time, time_after=new_time,
                          lateral_pass=new_error-old_error <= max(
                              settings['acceptance_lateral_absolute'],
                              old_error*settings['acceptance_lateral_relative']),
                          time_pass=new_time <= old_time*settings['acceptance_time_ratio'],
                          goals_pass=all(r['goal_reached']
                                         for g in groups.values() for r in g),
                          stops_pass=all(r['unexpected_stops'] == 0 for r in new))
        result['passed'] = complete and all(result.get(k, False) for k in (
            'lateral_pass', 'time_pass', 'goals_pass', 'stops_pass'))
        routes[scenario['name']] = result
    consistent = bool(trials) and all(t['valid'] for t in trials) and all(
        len({t.get(key) for t in trials}) == 1 for key in (
            'source_tree', 'physical_signature', 'map_fingerprint'))
    document = {'passed': consistent and all(r['passed'] for r in routes.values()),
                'consistent_sources_parameters_map': consistent,
                'routes': routes, 'trials': trials}
    (root/'geometry_tracking_comparison.json').write_text(json.dumps(document, indent=2)+'\n')
    print(json.dumps({k: v for k, v in document.items() if k != 'trials'}, indent=2))
    return document


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    analyze(parser.parse_args().root)


if __name__ == '__main__':
    main()
