#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Freeze one safe input, then paired geometry and speed experiments with explicit switches."""
import argparse
from copy import deepcopy
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
from zoneinfo import ZoneInfo

import yaml

from run_bezier_refine import check_protected, prepare, sha, snapshot, write_json


def execute(binary, case, output, variant, profile, overrides=None, speed_only=False,
            coordination=False):
    output.mkdir(parents=True, exist_ok=False)
    planner = deepcopy(case['planner'])
    params = planner['planner']['ros__parameters']['post_processing']
    params.update(overrides or {})
    params.update(enable_window_bezier=True, enable_turn_speed_coordination=coordination,
                  window_geometry_policy='geometry_first', window_detailed_diagnostics=True)
    if (params['window_solver_seconds'], params['window_total_seconds'],
            params['window_iterations']) != (.25, 1.0, 50):
        raise ValueError('Acceptance requires unchanged default solver budgets')
    motion = deepcopy(case['motion'])
    motion['/**']['ros__parameters']['enable_dynamic_angular_budget'] = False
    (output/'planner.yaml').write_text(yaml.safe_dump(planner, sort_keys=False))
    (output/'motion.yaml').write_text(yaml.safe_dump(motion, sort_keys=False))
    command = [str(binary), '--ros-args', '--params-file', str((output/'planner.yaml').resolve()),
               '--params-file', str((output/'motion.yaml').resolve()),
               '-p', 'use_sim_time:=false', '-p', 'benchmark_p1:=true',
               '-p', 'benchmark_use_config_switches:=true',
               '-p', f'benchmark_speed_only:={str(speed_only).lower()}',
               '-p', f'benchmark_variant:={variant}', '-p', 'benchmark_input_inflated:=true',
               '-p', f'benchmark_preserve_polyline:={str(case["synthetic"]).lower()}',
               '-p', f'benchmark_input_file:={Path(case["input"]).resolve()}',
               '-p', f'benchmark_profile_input:={Path(profile).resolve()}',
               '-p', f'benchmark_output_dir:={output.resolve()}']
    write_json(output/'command.json', command)
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1',
               ROS_LOG_DIR=str((output/'ros_logs').resolve()))
    with (output/'benchmark.log').open('w') as stream:
        subprocess.run(command, env=env, stdout=stream, stderr=subprocess.STDOUT,
                       check=True, timeout=30)
    result = json.loads((output/'offline.jsonl').read_text())
    if result['flags'] != (5 if coordination else 1) or result['speed_only'] != speed_only:
        raise ValueError('Benchmark effective switches differ from requested experiment')
    return output/f'case_{variant}.csv'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=Path('src/plan/config/joint_bezier.yaml'))
    parser.add_argument('--stage', choices=['baseline', 'geometry'], default='baseline')
    parser.add_argument('--cases', nargs='+')
    args = parser.parse_args()
    settings = yaml.safe_load(args.config.read_text())['joint_bezier']
    config = yaml.safe_load(Path('src/plan/config/bezier_benchmark.yaml').read_text())['benchmark']
    config['refine']['records'] = {'latest': settings['main_record'],
                                   'old': settings['regression_record']}
    # Extra spacing regressions use the same safe-input freezing path.
    config['synthetic']['scenarios'] += [
        {'name': 'short_s_turns', 'points': [[1., 1.], [3., 1.], [3., 1.6], [5., 1.6]]},
        {'name': 'long_s_turns', 'points': [[1., 1.], [3., 1.], [3., 4.], [5., 4.]]}]
    from ament_index_python.packages import get_package_prefix
    binary = (Path(get_package_prefix('plan'))/'lib/plan/bezier_benchmark').resolve()
    root = args.root
    if not (root/'inputs.json').exists():
        if args.stage != 'baseline':
            parser.error('Run the baseline stage first')
        root.mkdir(parents=True, exist_ok=False)
        check_protected(root)
        cases = prepare(root, config, binary)
        write_json(root/'configuration.json', config)
        write_json(root/'joint_configuration.json', settings)
    else:
        cases = json.loads((root/'inputs.json').read_text())
        if settings != json.loads((root/'joint_configuration.json').read_text()):
            raise ValueError('Frozen experiment settings changed; use a new root')
    check_protected(root)
    stage = root/args.stage
    stage.mkdir(exist_ok=False)
    snapshot(stage, binary)
    write_json(stage/'started.json', {'time': datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
                                    'stage': args.stage, 'cases': args.cases})
    for repeat in range(settings['repeats']):
        for case in cases:
            if args.cases and case['name'] not in args.cases:
                continue
            if sha(case['input']) != case['input_sha256'] or (
                    sha(case['profile']) != case['profile_sha256']):
                raise ValueError('Frozen safe input changed')
            name = f'{case["name"]}_{repeat+1}'
            if args.stage == 'baseline':
                geometry = execute(binary, case, root/'A0'/name, 'A0', case['profile'])
                execute(binary, case, root/'A'/name, 'A', geometry,
                        speed_only=True, coordination=True)
            else:
                execute(binary, case, root/'B0'/name, 'B0', case['profile'],
                        settings['geometry_overrides'])
            print(args.stage, name, flush=True)
    check_protected(root)
    write_json(stage/'completed.json', {'complete': True})


if __name__ == '__main__':
    main()
