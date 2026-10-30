#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Freeze inputs and compare baseline, legacy Bezier and geometry-first (no speed changes)."""
import argparse
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
from PIL import Image
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['replay', 'map', 'synthetic'], default='replay')
    parser.add_argument('--budget', choices=['default', 'expanded'], default='default')
    parser.add_argument('--config', default='src/plan/config/bezier_benchmark.yaml')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    c = yaml.safe_load(Path(args.config).read_text())['benchmark']
    p1 = c['p1']
    root = args.output or Path('log/bezier_p1') / datetime.now(ZoneInfo('Asia/Shanghai')).strftime(
        f'%Y%m%d_%H%M%S_{args.mode}_{args.budget}')
    root.mkdir(parents=True, exist_ok=False)
    planner = yaml.safe_load(Path(c['planner_params_file']).read_text())
    motion = yaml.safe_load(Path(c['motion_limits_file']).read_text())
    params = planner['planner']['ros__parameters']['post_processing']
    params.update(enable_window_bezier=False, enable_turn_speed_coordination=False,
                  window_detailed_diagnostics=True)
    if args.budget == 'expanded':
        params.update(window_solver_seconds=p1['diagnostic_window_seconds'],
                      window_total_seconds=p1['diagnostic_path_seconds'])
    motion['/**']['ros__parameters']['enable_dynamic_angular_budget'] = False
    grid = yaml.safe_load(Path(c['grid_params_file']).read_text())[
        'planning_grid']['ros__parameters']
    grid.update(yaml.safe_load(Path(c['deployment_params_file']).read_text())[
        'planning_grid']['ros__parameters'])
    frozen_profile = None
    inputs = {}
    if args.mode == 'replay':
        run = Path(p1['frozen_run'])
        experiment = json.loads((run / 'path_0001/experiment.json').read_text())
        snapshot_file = (run / 'path_0001' / experiment['map_snapshot']).resolve()
        snapshot = json.loads(gzip.decompress(snapshot_file.read_bytes()))
        meta = snapshot['map']
        width, height, resolution = (meta[k] for k in ('width', 'height', 'resolution'))
        q = meta['origin']['quaternion_xyzw']
        origin = meta['origin']['position'][:2] + [2 * math.atan2(q[2], q[3])]
        cells = np.asarray(snapshot['data']).reshape(height, width)
        frozen_profile = root / 'frozen_profile.csv'
        shutil.copy2(run / 'path_0001/planned_path.csv', frozen_profile)
        inputs = {str(file): hashlib.sha256(file.read_bytes()).hexdigest() for file in (
            snapshot_file, run / 'path_0001/planned_path.csv', run / 'path_0001/experiment.json')}
        endpoints = experiment['path_endpoints']
        scenarios = [{'name': 'frozen', 'points': [
            endpoints['start']['position'][:2], endpoints['end']['position'][:2]]}]
    elif args.mode == 'synthetic':
        spec = c['synthetic']
        width, height, resolution, origin = (spec[k] for k in (
            'width', 'height', 'resolution', 'origin'))
        cells = np.zeros((height, width), dtype=np.int8)
        scenarios = spec['scenarios']
    else:
        spec = yaml.safe_load(Path(c['map_yaml']).read_text())
        pixels = np.asarray(Image.open(Path(c['map_yaml']).parent / spec['image']).convert('L'))
        occupancy = pixels / 255.0 if spec['negate'] else (255.0 - pixels) / 255.0
        raw = np.where(occupancy > spec['occupied_thresh'], 100,
                       np.where(occupancy < spec['free_thresh'], 0, -1))
        cells = np.flipud(raw).astype(np.int8)
        height, width = cells.shape
        resolution, origin = float(np.float32(spec['resolution'])), spec['origin']
        scenarios = [{'name': 'fixed_route', 'start': p1['start'], 'goal': p1['goal']}]
    file = root / 'frozen_input.txt'
    with file.open('w') as stream:
        stream.write(f'{width} {height} {resolution} ' + ' '.join(map(str, origin)) + '\n')
        keys = ('occupied_threshold', 'unknown_is_obstacle', 'robot_radius',
                'safety_margin', 'inflate_map_boundary', 'max_cells')
        stream.write(' '.join(str(int(grid[k])) if isinstance(grid[k], bool) else str(grid[k])
                              for k in keys) + '\n')
        np.savetxt(stream, cells, fmt='%d')
        stream.write(f'{len(scenarios)}\n')
        for scenario in scenarios:
            points = scenario.get('points', [])
            stream.write(f"{scenario['name']} {len(points)}\n")
            for point in points or [scenario['start'] + scenario['goal']]:
                stream.write(' '.join(map(str, point)) + '\n')
    planner_file, motion_file = root / 'planner.yaml', root / 'motion.yaml'
    planner_file.write_text(yaml.safe_dump(planner, sort_keys=False))
    motion_file.write_text(yaml.safe_dump(motion, sort_keys=False))
    command = ['ros2', 'run', 'plan', 'bezier_benchmark', '--ros-args',
               '--params-file', str(planner_file.resolve()),
               '--params-file', str(motion_file.resolve()),
               '-p', 'use_sim_time:=false', '-p', 'benchmark_p1:=true',
               '-p', f'benchmark_input_inflated:={str(args.mode == "replay").lower()}',
               '-p', f'benchmark_preserve_polyline:={str(args.mode == "synthetic").lower()}',
               '-p', f'benchmark_input_file:={file.resolve()}',
               '-p', f'benchmark_output_dir:={root.resolve()}']
    if frozen_profile:
        command += ['-p', f'benchmark_profile_input:={frozen_profile.resolve()}']
    manifest = {'mode': args.mode, 'budget': args.budget, 'configuration': c,
                'source_revision': subprocess.check_output([
                    'git', 'rev-parse', 'HEAD']).decode().strip(),
                'input_hashes': inputs, 'command': command,
                'prepared_input_sha256': hashlib.sha256(file.read_bytes()).hexdigest()}
    from ament_index_python.packages import get_package_prefix
    binary = Path(get_package_prefix('plan')) / 'lib/plan/bezier_benchmark'
    manifest['binary'] = {'path': str(binary),
                          'sha256': hashlib.sha256(binary.read_bytes()).hexdigest()}
    files = subprocess.check_output([
        'git', 'ls-files', '--cached', '--others', '--exclude-standard', '-z',
        '--', 'src', 'tools', 'docs']).split(b'\0')
    manifest['source_files'] = {}
    with tarfile.open(root / 'source_snapshot.tar.gz', 'w:gz') as archive:
        for file in sorted(set(p.decode() for p in files if p)):
            path = Path(file)
            if path.is_file():
                manifest['source_files'][file] = hashlib.sha256(path.read_bytes()).hexdigest()
                archive.add(path, arcname=file)
    (root / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    (root / 'source.patch').write_bytes(subprocess.check_output([
        'git', 'diff', 'HEAD', '--binary']))
    print(root, flush=True)
    env = dict(os.environ, ROS_LOG_DIR=str((root / 'ros_logs').resolve()))
    with (root / 'benchmark.log').open('w') as stream:
        subprocess.run(command, check=True, env=env, stdout=stream, stderr=subprocess.STDOUT)
    for input_file, sha in inputs.items():
        assert hashlib.sha256(Path(input_file).read_bytes()).hexdigest() == sha
    before = json.loads(Path('log/bezier_p1/before/manifest.json').read_text())['files']
    protected = {file: sha for file, sha in before.items() if file.startswith((
        'src/controller/', 'src/urdf/', 'src/common_msg/')) or file in (
            'src/plan/config/motion_limits.yaml', 'src/plan/include/plan/motion_limits.hpp',
            'src/plan/include/plan/speed_envelope.hpp')}
    assert all(hashlib.sha256(Path(file).read_bytes()).hexdigest() == sha
               for file, sha in protected.items())
    manifest['protected_files_unchanged'] = True
    (root / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print('Replay completed; original inputs unchanged.', flush=True)


if __name__ == '__main__':
    main()
