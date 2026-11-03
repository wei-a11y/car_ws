#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Independent fixed-geometry, acceleration, local-time and six-way joint acceptance."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

from analyze_bezier_p1 import distances, load_path
from analyze_joint_bezier import load_result, same_geometry
from run_bezier_refine import sha, write_json


def interval_time(distance, speed, begin, end):
    """Integrate v² linear in s; interpolate only the clipped interval endpoints."""
    left = np.maximum(distance[:-1], begin)
    right = np.minimum(distance[1:], end)
    valid = right > left
    ds = np.diff(distance)[valid]
    v2 = speed[:-1][valid]**2
    slope = np.diff(speed**2)[valid]/ds
    a = np.sqrt(np.maximum(0, v2+slope*(left[valid]-distance[:-1][valid])))
    b = np.sqrt(np.maximum(0, v2+slope*(right[valid]-distance[:-1][valid])))
    if np.any(a+b <= 0):
        return math.inf
    return float(np.sum(2*(right[valid]-left[valid])/(a+b)))


def curve_groups(rows, motion, threshold=.10, max_gap=1.):
    progress = distances(rows)
    groups, stopped = [], False
    budget = motion['angular_acceleration']*motion['angular_acceleration_fraction']/2
    for i, row in enumerate(rows[:-1]):
        stopped = stopped or bool(row['stop_required'])
        k, q = row['curvature_bound'], row['curvature_rate_bound']
        cap = motion['v_max']
        if k:
            cap = min(cap, motion['feedforward_angular_fraction']*motion['w_max']/k,
                      math.sqrt(motion['lateral_acceleration']/k))
        if q:
            cap = min(cap, math.sqrt(budget/q))
        if k >= threshold or cap < motion['v_nominal']-1e-10:
            if groups and not stopped and progress[i]-groups[-1][1] <= max_gap:
                groups[-1] = (groups[-1][0], float(progress[i+1]))
            else:
                groups.append((float(progress[i]), float(progress[i+1])))
            stopped = False
        stopped = stopped or bool(rows[i+1]['stop_required'])
    return groups


def feasible(rows, distance, speed, motion):
    if len(speed) != 2*len(rows)-1 or not np.all(np.isfinite(speed)) or np.any(speed < 0):
        return False
    if speed[0] != 0 or speed[-1] != 0 or np.any(np.diff(distance) <= 0):
        return False
    budget = motion['angular_acceleration']*motion['angular_acceleration_fraction']/2
    for i, row in enumerate(rows[:-1]):
        k, q = row['curvature_bound'], row['curvature_rate_bound']
        cap = min(motion['v_nominal'], motion['v_max'], row['speed_limit'])
        acceleration, braking = motion['linear_acceleration'], motion['linear_deceleration']
        if k:
            cap = min(cap, motion['feedforward_angular_fraction']*motion['w_max']/k,
                      math.sqrt(motion['lateral_acceleration']/k))
            acceleration, braking = min(acceleration, budget/k), min(braking, budget/k)
        if q:
            cap = min(cap, math.sqrt(budget/q))
        for j in (2*i, 2*i+1):
            a = (speed[j+1]**2-speed[j]**2)/(2*(distance[j+1]-distance[j]))
            if max(speed[j], speed[j+1]) > cap+1e-8 or a > acceleration+1e-8 or -a > braking+1e-8:
                return False
        if rows[i+1]['stop_required'] and speed[2*i+2] != 0:
            return False
    return True


def bounded_gate(base, candidate, spans, limit):
    distance, before = base
    other_distance, after = candidate
    if not np.array_equal(distance, other_distance):
        return False
    all_spans = [(float(distance[0]), float(distance[-1])), *spans]
    return all(interval_time(distance, after, a, b) <=
               interval_time(distance, before, a, b)*(1+limit)+1e-8 for a, b in all_spans)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    root = args.root
    settings = json.loads((root/'joint_configuration.json').read_text())
    cases = json.loads((root/'inputs.json').read_text())
    geometry = json.loads((root/'geometry_inputs.json').read_text())
    regions = json.loads((root/'regions.json').read_text())
    results, all_pass = {}, True
    complete = json.loads((root/'completed.json').read_text())
    if complete['selected_cases'] is not None:
        raise ValueError('Development probes cannot pass full acceptance')
    for case in cases:
        name = case['name']
        runs = []
        for repeat in range(settings['repeats']):
            key = f'{name}_{repeat+1}'
            groups = {}
            for group in ('A0', 'A', 'B0', 'B', 'C', 'D'):
                directory, result, rows = load_result(root, group, key)
                source = geometry[key]['B0' if group in ('B0', 'B', 'D') else 'A0']
                if sha(source['path']) != source['sha256']:
                    raise ValueError('Frozen geometry changed since the replay')
                paired = same_geometry(rows, load_path(Path(source['path'])))
                speed = np.genfromtxt(directory/f'case_{group}_speed.csv', delimiter=',', names=True)
                motion = yaml.safe_load((directory/'motion.yaml').read_text())['/**']['ros__parameters']
                valid = feasible(rows, speed['distance'], speed['speed'], motion)
                valid = valid and paired and not motion['enable_dynamic_angular_budget']
                t = interval_time(speed['distance'], speed['speed'], 0, speed['distance'][-1])
                path_s = distances(rows)
                alignment = sum(abs(math.remainder(rows[i]['yaw']-math.atan2(
                    rows[i]['y']-rows[i-1]['y'], rows[i]['x']-rows[i-1]['x']), 2*math.pi))/
                    motion['w_max'] for i in range(1, len(rows)-1) if rows[i]['stop_required'])
                valid = valid and abs(t+alignment-result['executable_time']) < 1e-7
                groups[group] = {'result': result, 'rows': rows, 'envelope':
                                 (speed['distance'], speed['speed']), 'time': t+alignment,
                                 'events': result['report']['speed_events_after'],
                                 'variation': float(np.sum(np.abs(np.diff(speed['speed'])))),
                                 'valid': valid}
            for group, base in (('C', 'A0'), ('D', 'B0')):
                trace = groups[group]['result']['report']['coordination']
                if not trace or trace.get('policy') != 'bounded' or trace.get('fallback'):
                    groups[group]['valid'] = False
                    continue
                reported = [(g['begin_s'], g['end_s']) for g in trace['groups']]
                parameters = yaml.safe_load((root/group/key/'planner.yaml').read_text())[
                    'planner']['ros__parameters']['post_processing']
                motion = yaml.safe_load((root/group/key/'motion.yaml').read_text())[
                    '/**']['ros__parameters']
                spans = curve_groups(groups[base]['rows'], motion,
                    parameters['coordination_curvature_threshold'], parameters['coordination_group_max_gap'])
                groups[group]['valid'] &= len(spans) == len(reported) and np.allclose(
                    spans, reported, rtol=0, atol=1e-8)
                groups[group]['valid'] &= bounded_gate(groups[base]['envelope'],
                    groups[group]['envelope'], spans, settings['acceptance']['coordination_time_loss'])
                stop_s = distances(groups[group]['rows'])[
                    [bool(r['stop_required']) for r in groups[group]['rows']]]
                groups[group]['valid'] &= not any(a+1e-8 < s < b-1e-8 for a, b in spans for s in stop_s)
                groups[group]['valid'] &= groups[group]['events'] <= groups[base]['events']
                groups[group]['valid'] &= groups[group]['time'] <= geometry[key][base]['time_limit']+1e-8
            record = {g: {k: d[k] for k in ('time', 'events', 'variation', 'valid')}
                      for g, d in groups.items()}
            if name in ('latest', 'latest_reverse'):
                target = next(r['range'] for r in regions[name] if r['target'])
                turn_speeds = {}
                for group, source_group in (('A', 'A0'), ('D', 'B0')):
                    source_file = Path(geometry[key][source_group]['path']).with_name(
                        f'{source_group}_report.json')
                    source_report = json.loads(source_file.read_text())['report']
                    path_s = distances(groups[group]['rows'])
                    a, b = np.interp(target, source_report['geometry_reference_s'], path_s)
                    distance, speed = groups[group]['envelope']
                    turn_speeds[group] = float((b-a)/interval_time(distance, speed, a, b))
                record['continuous_turn_average_speed'] = turn_speeds
                joint = groups['D']['time'] < groups['A']['time'] and (
                    groups['D']['events'] < groups['A']['events']) and turn_speeds['D'] > turn_speeds['A']
            else:
                joint = groups['D']['time'] <= groups['A']['time']*(
                    1+settings['acceptance']['regression_time_loss'])+1e-8
            record['pass'] = joint and all(g['valid'] for g in groups.values())
            all_pass &= record['pass']
            runs.append(record)
        results[name] = runs
    write_json(root/'analysis.json', results)
    write_json(root/'acceptance.json', {'pass': bool(all_pass), 'complete': True,
                                     'cases': {n: all(r['pass'] for r in v) for n, v in results.items()}})
    print(json.dumps({'pass': bool(all_pass), 'cases': {n: all(r['pass'] for r in v)
                                                      for n, v in results.items()}}, indent=2))


if __name__ == '__main__':
    main()
