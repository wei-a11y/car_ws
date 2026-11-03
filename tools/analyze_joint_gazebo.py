#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Check all frozen-profile trials, with TRACKING-only heading/speed statistics."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pandas as pd

from run_bezier_refine import write_json

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/controller/scripts'))
from tracking_analysis import STATES, load_config, summarize  # noqa: E402


def tracking_statistics(data, curvature_threshold):
    """The recorder stores the TrackingStatus numeric enum (TRACKING = 2)."""
    tracking = data['state'].eq(2) & data['geometry_valid'].gt(0)
    curved = tracking & data['curvature'].abs().ge(curvature_threshold)

    def p95(values):
        values = np.asarray(values, dtype=float)
        values = values[np.isfinite(values)]
        return float(np.percentile(np.abs(values), 95)) if len(values) else None

    return dict(tracking_samples=int(tracking.sum()), curved_tracking_samples=int(curved.sum()),
                lateral_p95=p95(data.loc[curved, 'lateral_error']),
                heading_p95=p95(data.loc[tracking, 'heading_error']),
                speed_p95=p95(data.loc[tracking, 'v_actual']-data.loc[tracking, 'v_ref']))


def tracking_nonregression(grouped, limits):
    """Check each paired run; aggregate medians must not hide a bad repetition."""
    baseline = {r['repeat']: r for r in grouped['A']}
    checks = {}
    for variant in ('B', 'C', 'D'):
        for key, stem in (('lateral_p95', 'lateral'), ('heading_p95', 'heading'), ('speed_p95', 'speed')):
            checks[f'{variant}_{stem}_nonregression'] = all(
                r['repeat'] in baseline and r[key]-baseline[r['repeat']][key] <= max(
                    limits[f'{stem}_p95_absolute_increase'],
                    limits[f'{stem}_p95_relative_increase']*baseline[r['repeat']][key])
                for r in grouped[variant])
    return checks


def analyze(root):
    experiment = json.loads((root/'experiment.json').read_text())
    config = experiment['configuration']
    limits = experiment['tracking_limits']
    analysis = load_config()['analysis']
    trials = []
    for file in sorted(root.glob('*/result.json')):
        result = json.loads(file.read_text())
        result['valid'] = False
        epochs = list(file.parent.glob('record/run_*/path_0001/actual_path.csv'))
        if len(epochs) == 1:
            try:
                directory = epochs[0].parent
                data = pd.read_csv(epochs[0])
                planned = pd.read_csv(directory/'planned_path.csv')
                _, summary = summarize(planned, data, analysis)
                write_json(directory/'tracking_summary.json', summary)
                metadata = json.loads((directory/'experiment.json').read_text())
                parameters = metadata['parameters']
                result.update(tracking_statistics(data, analysis['curve_curvature_threshold']))
                result.update(lateral_max=float(data['lateral_error'].abs().max()),
                              position_max=float(data['position_error'].abs().max()),
                              unexpected_stops=summary['unexpected_stop_count'],
                              source_tree=metadata['source'].get('source_tree_sha256'),
                              full_summary=summary)
                physical_keys = ('v_nominal', 'v_max', 'w_max', 'linear_acceleration', 'linear_deceleration',
                                 'angular_acceleration', 'lateral_acceleration', 'feedforward_angular_fraction',
                                 'angular_acceleration_fraction', 'min_turn_speed', 'q_lateral', 'q_heading',
                                 'r_angular', 'speed_pid.kp', 'speed_pid.ki', 'speed_pid.kd',
                                 'goal_position_tolerance', 'max_tracking_error', 'feedforward_preview_distance',
                                 'wheel_radius', 'wheel_separation')
                physical = {name: {k: entry.get('values', {}).get(k) for k in physical_keys}
                            for name, entry in parameters.items()}
                result['physical_signature'] = hashlib.sha256(json.dumps(physical, sort_keys=True).encode()).hexdigest()
                result['valid'] = all(result.get(k, False) for k in (
                    'goal_reached', 'configuration_matches', 'map_matches', 'profile_matches', 'source_tree')) and (
                    all(entry.get('status') == 'ok' for entry in parameters.values())) and (
                    all(result[k] is not None for k in ('lateral_p95', 'heading_p95', 'speed_p95'))) and (
                    result['position_max'] <= limits['max_position_error'] and
                    result['lateral_max'] <= limits['max_lateral_error'] and result['unexpected_stops'] == 0)
                if len(data):
                    last = data.iloc[-1]
                    terminal = {k: float(last[k]) for k in (
                        'remaining', 'progress', 'v_ref', 'v_cmd', 'v_actual',
                        'lateral_error', 'heading_error', 'position_error')}
                    terminal.update(state=STATES.get(int(last['state']), 'UNKNOWN'),
                        endpoint_distance=float(np.hypot(last['x']-planned.iloc[-1]['x'],
                                                         last['y']-planned.iloc[-1]['y'])),
                        goal_position_tolerance=parameters.get('/lqr_controller', {}).get(
                            'values', {}).get('goal_position_tolerance'))
                    write_json(file.parent/'terminal_diagnosis.json', terminal)
            except (OSError, KeyError, TypeError, ValueError) as error:
                result.update(valid=False, analysis_error=str(error))
        else:
            result['analysis_error'] = f'Expected one recorded path epoch, found {len(epochs)}'
        trials.append(result)
    routes = {}
    for direction in ('outward', 'inward'):
        grouped = {g: [r for r in trials if r['variant'] == g and r['direction'] == direction]
                   for g in ('A', 'B', 'C', 'D')}
        complete = all(len(r) == config['repeats_per_direction'] for r in grouped.values())
        route = {'complete': complete, 'passed': False}
        if complete and all(r['valid'] for rows in grouped.values() for r in rows):
            medians = {g: {k: float(np.median([r[k] for r in rows])) for k in (
                'duration_ros', 'lateral_p95', 'heading_p95', 'speed_p95')} for g, rows in grouped.items()}
            baseline, candidate = medians['A'], medians['D']
            checks = {'time_improves': candidate['duration_ros'] < baseline['duration_ros']}
            checks.update(tracking_nonregression(grouped, limits))
            route.update(medians=medians, checks=checks, passed=all(checks.values()))
        routes[direction] = route
    consistent = bool(trials) and all(r['valid'] for r in trials) and all(
        len({r.get(k) for r in trials}) == 1 for k in ('source_tree', 'physical_signature'))
    result = {'passed': consistent and experiment['probe_limit'] is None and all(
        r['passed'] for r in routes.values()), 'consistent': consistent, 'limits': limits,
        'routes': routes, 'trials': trials}
    write_json(root/'tracking_comparison.json', result)
    print(json.dumps({k: v for k, v in result.items() if k != 'trials'}, indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    analyze(parser.parse_args().root)
