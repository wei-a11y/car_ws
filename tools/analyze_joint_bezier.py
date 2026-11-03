#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Strict paired replay checks and fixed-safe-path geometry acceptance; no tolerance relaxation."""
import argparse
import json
from pathlib import Path

import numpy as np

from analyze_bezier_p1 import distances, load_path
from analyze_bezier_refine import inspect_curves, region_metrics
from run_bezier_refine import check_protected, write_json


GEOMETRY_FIELDS = ('x', 'y', 'yaw', 'curvature', 'stop_required', 'segment_type',
                   'curvature_bound', 'curvature_rate_bound')


def same_geometry(first, second):
    return len(first) == len(second) and all(
        a[k] == b[k] for a, b in zip(first, second) for k in GEOMETRY_FIELDS)


def load_result(root, group, name):
    directory = root/group/name
    result = json.loads((directory/'offline.jsonl').read_text())
    rows = load_path(directory/f'case_{group}.csv')
    if result['flags'] != (5 if group in ('A', 'B', 'C', 'D') else 1):
        raise ValueError('Incorrect effective functional switches')
    if result['report']['dynamic_angular_budget']:
        raise ValueError('Dynamic budget must remain disabled')
    return directory, result, rows


def fixed_regions(rows, report):
    mapping = report['geometry_reference_s']
    roots = [w for w in report['windows'] if w['parent'] == -1]
    regions = []
    for window in sorted(roots, key=lambda w: w['baseline_begin_s']):
        a, b = window['baseline_begin_s'], window['baseline_end_s']
        signs = [np.sign(r['curvature']) for r, s in zip(rows, mapping)
                 if a <= s <= b and abs(r['curvature']) >= .1]
        reverse = any(x != y for x, y in zip(signs, signs[1:]))
        regions.append({'range': [a, b], 'target': bool(reverse),
                        'basis': 'baseline_root_window_with_signed_curvature_reversal'
                        if reverse else 'protected_baseline_root_window'})
    if not regions:
        regions = [{'range': [0., float(mapping[-1])], 'target': False,
                    'basis': 'entire_frozen_safe_path'}]
    reverse = [r for r in regions if r['target']]
    if reverse:
        # Freeze the principal continuous-turn group before evaluating candidates;
        # the other groups are non-regression regions, including the front section.
        principal = max(reverse, key=lambda r: r['range'][1]-r['range'][0])
        for region in regions:
            region['target'] = region is principal
            region['basis'] = ('principal_baseline_reversal_window' if region['target']
                               else 'protected_baseline_root_window')
    return regions


def summarize_run(directory, result, rows, regions):
    report = result['report']
    mapping = report['geometry_reference_s']
    if len(mapping) != len(rows) or any(b <= a for a, b in zip(mapping, mapping[1:])):
        raise ValueError('Invalid safe-path reference mapping')
    dense, joins = inspect_curves(report, directory)
    metrics = [region_metrics(rows, mapping, region['range'], dense) for region in regions]
    return {'regions': metrics, 'time': result['executable_time'],
            'length': float(distances(rows)[-1]), 'processing_seconds': result['seconds'],
            'joins_valid': all(j['position_gap'] < 1e-8 and j['heading_gap'] < 1e-8 and
                               j['curvature_gap'] < 1e-8 for j in joins),
            'stops': sum(r['stop_required'] for r in rows[:-1]),
            'fallback': report['geometry_fallback_reason'],
            'events': report['speed_events_after']}


def geometry_gate(baselines, candidates, regions, settings, main):
    if len(baselines) != settings['repeats'] or len(candidates) != settings['repeats']:
        return {'pass': False, 'reason': 'incomplete_repeats'}
    comparisons = []
    for index, region in enumerate(regions):
        old = {k: max(r['regions'][index][k] for r in baselines)
               for k in ('K_bound', 'Q_bound', 'nodal_rate_rms')}
        new = {k: max(r['regions'][index][k] for r in candidates) for k in old}
        target = main and region['target']
        k_ratio = 1-settings['acceptance']['curvature_reduction'] if target else 1.
        q_ratio = 1-settings['acceptance']['rate_rms_reduction'] if target else 1.
        passed = new['K_bound'] <= old['K_bound']*k_ratio+1e-8 and (
            new['Q_bound'] <= old['Q_bound']+1e-8) and (
            new['nodal_rate_rms'] <= old['nodal_rate_rms']*q_ratio+1e-8)
        comparisons.append({'region': region, 'before_worst': old, 'after_worst': new,
                            'pass': passed})
    invariants = all(c['joins_valid'] and c['stops'] <= b['stops'] and not c['fallback']
                     for b, c in zip(baselines, candidates))
    time_ok = max(c['time'] for c in candidates) <= max(b['time'] for b in baselines)*(
        1. if main else 1+settings['acceptance']['regression_time_loss'])
    return {'pass': all(r['pass'] for r in comparisons) and invariants and time_ok,
            'regions': comparisons, 'invariants_pass': invariants, 'time_pass': time_ok,
            'before_worst_time': max(b['time'] for b in baselines),
            'after_worst_time': max(c['time'] for c in candidates)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--stage', choices=['baseline', 'geometry'], default='baseline')
    parser.add_argument('--attempt')
    args = parser.parse_args()
    root = args.root
    if args.attempt and (args.stage != 'geometry' or Path(args.attempt).name != args.attempt):
        parser.error('Attempt must identify a geometry experiment directory')
    work_root = root/'attempts'/args.attempt if args.attempt else root
    check_protected(root)
    settings = json.loads((root/'joint_configuration.json').read_text())
    cases = json.loads((root/'inputs.json').read_text())
    regions_file = root/'joint_regions.json'
    regions = json.loads(regions_file.read_text()) if regions_file.exists() else {}
    analyses, gates = {}, {}
    for case in cases:
        name = case['name']
        analyses[name] = {'baseline': [], 'candidate': []}
        if name not in regions:
            _, first, rows = load_result(root, 'A0', f'{name}_1')
            regions[name] = fixed_regions(rows, first['report'])
        for repeat in range(settings['repeats']):
            run = f'{name}_{repeat+1}'
            control_root = work_root/'control' if args.stage == 'geometry' else root
            directory, result, rows = load_result(control_root, 'A0', run)
            _, speed_result, speed_rows = load_result(control_root, 'A', run)
            if not speed_result['speed_only'] or not same_geometry(rows, speed_rows):
                raise ValueError(f'Unpaired speed replay: {run}')
            analyses[name]['baseline'].append(summarize_run(directory, result, rows,
                                                          regions[name]))
            if args.stage == 'geometry':
                directory, result, rows = load_result(work_root, 'B0', run)
                analyses[name]['candidate'].append(summarize_run(directory, result, rows,
                                                               regions[name]))
        if args.stage == 'geometry':
            gates[name] = geometry_gate(analyses[name]['baseline'], analyses[name]['candidate'],
                                        regions[name], settings, name == 'latest')
    if not regions_file.exists():
        write_json(regions_file, regions)
    targets_present = any(r['target'] for r in regions['latest'])
    result = {'stage': args.stage, 'complete': True, 'targets_present': targets_present,
              'cases': gates, 'pass': targets_present and all(g['pass'] for g in gates.values())}
    write_json(work_root/f'{args.stage}_analysis.json', analyses)
    write_json(work_root/f'{args.stage}_acceptance.json', result)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
