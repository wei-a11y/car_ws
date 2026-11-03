#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Run A0/A/B0/B/C/D on immutable, already-certified geometry; never rerun Bezier."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import shutil

import yaml

from analyze_joint_bezier import load_result, same_geometry
from analyze_bezier_p1 import load_path
from run_bezier_refine import check_protected, sha, snapshot, write_json
from run_joint_bezier import execute
from joint_speed_profiles import reverse_profile, write_profile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path,
                        default=Path('src/plan/config/turn_speed_coordination.yaml'))
    parser.add_argument('--cases', nargs='+', help='Development probe; incomplete runs cannot pass')
    parser.add_argument('--include-reverse', action='store_true', help='Also validate the reversed main route')
    args = parser.parse_args()
    settings = yaml.safe_load(args.config.read_text())['bounded_coordination']
    joint = json.loads((args.root/'joint_configuration.json').read_text())
    geometry_root = args.root/'attempts'/settings['geometry_attempt']
    gate = json.loads((geometry_root/'geometry_acceptance.json').read_text())
    if not gate['pass']:
        parser.error('Geometry acceptance must pass before speed optimization')
    cases = json.loads((args.root/'inputs.json').read_text())
    regions = json.loads((args.root/'joint_regions.json').read_text())
    if args.include_reverse:
        main_case = next(c for c in cases if c['name'] == 'latest')
        motion = main_case['motion']['/**']['ros__parameters']
        if motion['linear_acceleration'] != motion['linear_deceleration']:
            parser.error('Reverse global reference time must be recalculated for asymmetric motion limits')
        reverse = deepcopy(main_case)
        reverse.update(name='latest_reverse', source_case='latest', reverse=True)
        reverse['scenario'] = {'start': main_case['scenario']['goal'],
                               'goal': main_case['scenario']['start']}
        cases.append(reverse)
        _, report, _ = load_result(geometry_root/'control', 'A0', 'latest_1')
        length = report['report']['geometry_reference_s'][-1]
        regions['latest_reverse'] = [dict(r, range=[length-r['range'][1], length-r['range'][0]],
                                          basis='reversed_'+r['basis'])
                                     for r in reversed(regions['latest'])]
    check_protected(args.root)
    from ament_index_python.packages import get_package_prefix
    binary = (Path(get_package_prefix('plan'))/'lib/plan/bezier_benchmark').resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    snapshot(args.output, binary)
    shutil.copy2(binary, args.output/'bezier_benchmark')
    write_json(args.output/'configuration.json', settings)
    write_json(args.output/'joint_configuration.json', joint)
    write_json(args.output/'inputs.json', cases)
    write_json(args.output/'regions.json', regions)
    write_json(args.output/'geometry_gate.json', gate)
    inputs = {}
    for repeat in range(joint['repeats']):
        for case in cases:
            if args.cases and case['name'] not in args.cases:
                continue
            if sha(case['input']) != case['input_sha256']:
                raise ValueError('Frozen map input changed')
            name = f'{case["name"]}_{repeat+1}'
            frozen = args.output/'geometry'/name
            frozen.mkdir(parents=True)
            sources = {}
            for group, base in (('A0', geometry_root/'control'), ('B0', geometry_root)):
                source_name = f'{case.get("source_case", case["name"])}_{repeat+1}'
                directory, result, rows = load_result(base, group, source_name)
                source = directory/f'case_{group}.csv'
                destination = frozen/f'{group}.csv'
                if case.get('reverse'):
                    rows, result = reverse_profile(rows, result)
                    write_profile(destination, rows)
                else:
                    shutil.copy2(source, destination)
                write_json(frozen/f'{group}_report.json', result)
                sources[group] = {'path': str(destination), 'sha256': sha(destination),
                                  'source': str(source), 'time_limit':
                                  result['report']['frozen_baseline_time'] *
                                  case['planner']['planner']['ros__parameters'][
                                      'post_processing']['window_max_time_ratio']}
            inputs[name] = sources
            write_json(args.output/'geometry_inputs.json', inputs)
            for group in ('A0', 'A', 'B0', 'B', 'C', 'D'):
                source = sources['B0' if group in ('B0', 'B', 'D') else 'A0']
                if sha(source['path']) != source['sha256']:
                    raise ValueError('Frozen geometry changed')
                overrides = deepcopy(settings['overrides']) if group in ('C', 'D') else {
                    'coordination_policy': 'legacy'}
                overrides.update(joint['geometry_overrides'] if group in ('B0', 'B', 'D') else {
                    'window_feature_refinement': False})
                output = args.output/group/name
                result_path = execute(binary, case, output, group, source['path'], overrides,
                                      speed_only=True, coordination=group not in ('A0', 'B0'),
                                      speed_time_limit=source['time_limit'])
                if not same_geometry(load_path(Path(source['path'])), load_path(result_path)):
                    raise ValueError('Speed-only experiment changed certified geometry')
            print(name, flush=True)
    check_protected(args.root)
    write_json(args.output/'completed.json', {'complete': True, 'selected_cases': args.cases})


if __name__ == '__main__':
    main()
