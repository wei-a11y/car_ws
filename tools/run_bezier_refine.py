#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Freeze pre-Bezier paths once, then run named geometry-only ablations."""
import argparse
from copy import deepcopy
from datetime import datetime
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
from zoneinfo import ZoneInfo

import numpy as np
import yaml


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def write_input(path, geometry, cells, grid, scenario):
    with path.open('w') as stream:
        stream.write(' '.join(map(str, geometry)) + '\n')
        keys = ('occupied_threshold', 'unknown_is_obstacle', 'robot_radius',
                'safety_margin', 'inflate_map_boundary', 'max_cells')
        stream.write(' '.join(str(int(grid[k])) if isinstance(grid[k], bool) else str(grid[k])
                              for k in keys) + '\n')
        np.savetxt(stream, cells, fmt='%d')
        points = scenario.get('points', [])
        stream.write(f'1\ncase {len(points)}\n')
        for point in points or [scenario['start'] + scenario['goal']]:
            stream.write(' '.join(map(str, point)) + '\n')


def execute(binary, case, output, planner, motion, variant, profile=None):
    output.mkdir(parents=True, exist_ok=False)
    planner_file, motion_file = output/'planner.yaml', output/'motion.yaml'
    planner_file.write_text(yaml.safe_dump(planner, sort_keys=False))
    motion_file.write_text(yaml.safe_dump(motion, sort_keys=False))
    command = [str(binary), '--ros-args', '--params-file', str(planner_file.resolve()),
               '--params-file', str(motion_file.resolve()), '-p', 'use_sim_time:=false',
               '-p', 'benchmark_p1:=true', '-p', f'benchmark_variant:={variant}',
               '-p', 'benchmark_input_inflated:=true',
               '-p', f'benchmark_preserve_polyline:={str(case["synthetic"]).lower()}',
               '-p', f'benchmark_input_file:={Path(case["input"]).resolve()}',
               '-p', f'benchmark_output_dir:={output.resolve()}']
    if profile:
        command += ['-p', f'benchmark_profile_input:={Path(profile).resolve()}']
    write_json(output/'command.json', command)
    env = dict(os.environ, ROS_LOG_DIR=str((output/'ros_logs').resolve()),
               ROS_LOCALHOST_ONLY='1')
    with (output/'benchmark.log').open('w') as stream:
        subprocess.run(command, check=True, env=env, stdout=stream, stderr=subprocess.STDOUT,
                       timeout=30)
    return output/'case_baseline.csv'


def prepare(root, config, binary):
    settings = config['refine']
    planner = yaml.safe_load(Path(config['planner_params_file']).read_text())
    motion = yaml.safe_load(Path(config['motion_limits_file']).read_text())
    motion['/**']['ros__parameters']['enable_dynamic_angular_budget'] = False
    grid = yaml.safe_load(Path(config['grid_params_file']).read_text())[
        'planning_grid']['ros__parameters']
    grid.update(yaml.safe_load(Path(config['deployment_params_file']).read_text())[
        'planning_grid']['ros__parameters'])
    cases = []
    for name, record in settings['records'].items():
        directory = root/'inputs'/name
        directory.mkdir(parents=True)
        run = Path(record)
        experiment_path = run/'path_0001/experiment.json'
        experiment = json.loads(experiment_path.read_text())
        captured = experiment['parameters']['/planner']['values']
        captured_params = {k[len('post_processing.'):]: v for k, v in captured.items()
                           if k.startswith('post_processing.')}
        case_planner = deepcopy(planner)
        case_planner['planner']['ros__parameters']['post_processing'].update(captured_params)
        params = case_planner['planner']['ros__parameters']['post_processing']
        params.update(enable_window_bezier=False, enable_turn_speed_coordination=False,
                      window_detailed_diagnostics=True)
        snapshot_file = (run/'path_0001'/experiment['map_snapshot']).resolve()
        snapshot = json.loads(gzip.decompress(snapshot_file.read_bytes()))
        meta = snapshot['map']
        q = meta['origin']['quaternion_xyzw']
        geometry = [meta['width'], meta['height'], meta['resolution'],
                    *meta['origin']['position'][:2], 2*math.atan2(q[2], q[3])]
        cells = np.asarray(snapshot['data']).reshape(meta['height'], meta['width'])
        endpoints = experiment['path_endpoints']
        scenario = {'start': endpoints['start']['position'][:2],
                    'goal': endpoints['end']['position'][:2]}
        source_files = (snapshot_file, experiment_path, run/'path_0001/planned_path.csv')
        case = {'name': name, 'synthetic': False, 'input': str(directory/'input.txt'),
                'source_record': str(run), 'source_hashes': {str(p): sha(p) for p in source_files},
                'scenario': scenario, 'planner': case_planner, 'motion': motion}
        write_input(Path(case['input']), geometry, cells, grid, scenario)
        if name == 'old':
            # P0 proves that this run adopted zero Bezier windows. Do not re-smooth latest output.
            report = json.loads((run/'path_0001/planning_report.json').read_text())
            if any(w['result'] == 'accepted' for w in report['windows']):
                raise ValueError('Old regression is not an unmodified pre-Bezier path')
            shutil.copy2(run/'path_0001/planned_path.csv', directory/'safe.csv')
            case['baseline_origin'] = 'historical_zero_adoption'
        else:
            frozen = execute(binary, case, directory/'freeze', case_planner, motion, 'baseline')
            shutil.copy2(frozen, directory/'safe.csv')
            case['baseline_origin'] = 'reconstructed_Astar_before_Bezier_not_historical_exact'
        case['profile'] = str(directory/'safe.csv')
        cases.append(case)
    spec = config['synthetic']
    for scenario in spec['scenarios']:
        directory = root/'inputs'/scenario['name']
        directory.mkdir(parents=True)
        case = {'name': scenario['name'], 'synthetic': True,
                'input': str(directory/'input.txt'), 'planner': deepcopy(planner),
                'motion': motion, 'scenario': scenario,
                'baseline_origin': 'frozen_synthetic_before_Bezier'}
        params = case['planner']['planner']['ros__parameters']['post_processing']
        params.update(enable_window_bezier=False, enable_turn_speed_coordination=False,
                      window_detailed_diagnostics=True)
        geometry = [spec['width'], spec['height'], spec['resolution'], *spec['origin']]
        write_input(Path(case['input']), geometry,
                    np.zeros((spec['height'], spec['width']), dtype=int), grid, scenario)
        frozen = execute(binary, case, directory/'freeze', case['planner'], motion, 'baseline')
        shutil.copy2(frozen, directory/'safe.csv')
        case['profile'] = str(directory/'safe.csv')
        cases.append(case)
    for case in cases:
        case['input_sha256'] = sha(case['input'])
        case['profile_sha256'] = sha(case['profile'])
    write_json(root/'inputs.json', cases)
    return cases


def snapshot(root, binary):
    files = subprocess.check_output([
        'git', 'ls-files', '--cached', '--others', '--exclude-standard', '-z',
        '--', 'src', 'tools', 'docs']).split(b'\0')
    paths = sorted({Path(p.decode()) for p in files if p})
    manifest = {'revision': subprocess.check_output(['git', 'rev-parse', 'HEAD']).decode().strip(),
                'binary': {'path': str(binary), 'sha256': sha(binary)},
                'files': {str(p): sha(p) for p in paths if p.is_file()}}
    with tarfile.open(root/'source.tar.gz', 'w:gz') as archive:
        for p in paths:
            if p.is_file():
                archive.add(p, arcname=str(p))
    (root/'source.patch').write_bytes(subprocess.check_output(['git', 'diff', 'HEAD', '--binary']))
    write_json(root/'source.json', manifest)


def is_protected(path):
    return path.startswith(('src/controller/', 'src/urdf/', 'src/common_msg/')) or path in (
        'src/plan/config/motion_limits.yaml', 'src/plan/include/plan/motion_limits.hpp',
        'src/plan/include/plan/speed_envelope.hpp')


def check_protected(root):
    manifest = root/'protected_sources.json'
    if not manifest.exists():
        # Older frozen roots already carry complete source manifests. New roots
        # capture their own protected sources, without depending on ignored task logs.
        sources = sorted(root.glob('*/source.json'))
        if sources:
            files = json.loads(sources[0].read_text())['files']
        else:
            paths = subprocess.check_output(['git', 'ls-files', 'src']).decode().splitlines()
            files = {p: sha(p) for p in paths}
        write_json(manifest, {p: h for p, h in files.items() if is_protected(p)})
    protected = json.loads(manifest.read_text())
    assert protected and all(sha(p) == h for p, h in protected.items())
    write_json(root/'protected_check.json', {'passed': True, 'files': len(protected)})


def prove_unchanged_scope(root, cases, config, binary, candidate):
    stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
    output = root/'equivalence'/stamp
    output.mkdir(parents=True)
    snapshot(output, binary)
    proof_file = root/'scope_equivalence.json'
    prior = json.loads(proof_file.read_text()) if proof_file.exists() else {}
    proof = {'binary_sha256': sha(binary), 'directory': str(output), 'cases': {},
             'purpose': 'work-complete equivalence only; not default-budget geometry acceptance',
             'settings': {'iterations': 4, 'window_seconds': 5.0, 'path_seconds': 20.0}}
    for case in cases:
        if not case['synthetic']:
            continue
        paths = {}
        for name in ('control', 'candidate'):
            planner = deepcopy(case['planner'])
            params = planner['planner']['ros__parameters']['post_processing']
            params.update(config['refine']['compatibility_defaults'])
            if name == 'candidate':
                params.update(config['refine']['variants'][candidate])
            params.update(window_solver_seconds=5.0, window_total_seconds=20.0,
                          window_iterations=4, enable_window_bezier=True,
                          enable_turn_speed_coordination=False,
                          window_geometry_policy='geometry_first',
                          window_detailed_diagnostics=True)
            directory = output/case['name']/name
            execute(binary, case, directory, planner, case['motion'], name, case['profile'])
            report = json.loads((directory/'offline.jsonl').read_text())['report']
            assert all(w['evaluation_basis'] == 'legacy_mixed' for w in report['windows'])
            paths[name] = (directory/f'case_{name}.csv').read_bytes()
        assert paths['control'] == paths['candidate']
        digest = hashlib.sha256(paths['control']).hexdigest()
        proof['cases'][case['name']] = {'passed': True, 'path_sha256': digest}
    if prior.get('binary_sha256') == proof['binary_sha256']:
        if 'original_binary_compatibility' in prior:
            proof['original_binary_compatibility'] = prior['original_binary_compatibility']
    if proof_file.exists():
        shutil.copy2(proof_file, output/'previous_proof.json')
    write_json(proof_file, proof)
    check_protected(root)
    print('Scope equivalence passed:', candidate, sorted(proof['cases']))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=Path('src/plan/config/bezier_benchmark.yaml'))
    parser.add_argument('--root', type=Path)
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--prove-unchanged-scope', metavar='VARIANT')
    parser.add_argument('--variants', nargs='+', default=['A'])
    parser.add_argument('--cases', nargs='+')
    parser.add_argument('--repeats', type=int)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())['benchmark']
    from ament_index_python.packages import get_package_prefix
    binary = (Path(get_package_prefix('plan'))/'lib/plan/bezier_benchmark').resolve()
    root = args.root or Path('log/bezier_refine')/datetime.now(ZoneInfo('Asia/Shanghai')).strftime(
        '%Y%m%d_%H%M%S')
    if not (root/'inputs.json').exists():
        root.mkdir(parents=True, exist_ok=False)
        check_protected(root)
        cases = prepare(root, config, binary)
        write_json(root/'configuration.json', config)
    else:
        cases = json.loads((root/'inputs.json').read_text())
    print(root, flush=True)
    check_protected(root)
    if args.prove_unchanged_scope:
        prove_unchanged_scope(root, cases, config, binary, args.prove_unchanged_scope)
        return
    if args.prepare_only:
        return
    repeats = args.repeats if args.repeats is not None else config['refine']['repeats']
    if repeats < 1:
        parser.error('repeats must be positive')
    for variant in args.variants:
        overrides = config['refine']['variants'][variant]
        group = root/variant
        group.mkdir(exist_ok=False)
        snapshot(group, binary)
        write_json(group/'overrides.json', overrides)
        for repeat in range(repeats):
            for case in cases:
                if args.cases and case['name'] not in args.cases:
                    continue
                assert sha(case['input']) == case['input_sha256']
                assert sha(case['profile']) == case['profile_sha256']
                planner = deepcopy(case['planner'])
                params = planner['planner']['ros__parameters']['post_processing']
                params.update(config['refine']['compatibility_defaults'])
                params.update(overrides)
                params.update(enable_window_bezier=True, enable_turn_speed_coordination=False,
                              window_detailed_diagnostics=True,
                              window_geometry_policy='geometry_first')
                if params['window_solver_seconds'] != .25 or params['window_total_seconds'] != 1.0:
                    raise ValueError('Default acceptance budgets changed')
                output = group/f'{case["name"]}_{repeat+1}'
                execute(binary, case, output, planner, case['motion'], variant, case['profile'])
                print(variant, case['name'], repeat+1, flush=True)
    check_protected(root)


if __name__ == '__main__':
    main()
