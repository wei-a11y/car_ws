#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Freeze production map/config inputs and run six offline algorithm variants."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
from zoneinfo import ZoneInfo

from PIL import Image
import numpy as np
import yaml


def load(path):
    return yaml.safe_load(Path(path).read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='src/plan/config/bezier_benchmark.yaml')
    parser.add_argument('--synthetic', action='store_true')
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    c = load(args.config)['benchmark']
    suffix = 'synthetic' if args.synthetic else 'map'
    directory = Path(c['output_root']) / (datetime.now(ZoneInfo('Asia/Shanghai')).strftime(
        '%Y%m%d_%H%M%S_%f') + '_' + suffix)
    directory.mkdir(parents=True)
    grid = load(c['grid_params_file'])['planning_grid']['ros__parameters']
    grid.update(load(c['deployment_params_file'])['planning_grid']['ros__parameters'])
    if args.synthetic:
        spec = c['synthetic']
        width, height, resolution, origin = (spec[k] for k in (
            'width', 'height', 'resolution', 'origin'))
        cells = np.zeros((height, width), dtype=np.int8)
        scenarios = spec['scenarios']
    else:
        spec = load(c['map_yaml'])
        if spec.get('mode', 'trinary') != 'trinary':
            raise ValueError('Benchmark importer supports the deployed trinary map mode only')
        pixels = np.asarray(Image.open(Path(c['map_yaml']).parent / spec['image']).convert('L'))
        occupancy = pixels / 255.0 if spec['negate'] else (255.0 - pixels) / 255.0
        raw = np.where(occupancy > spec['occupied_thresh'], 100,
                       np.where(occupancy < spec['free_thresh'], 0, -1))
        cells = np.flipud(raw).astype(np.int8)
        height, width = cells.shape
        resolution, origin = float(np.float32(spec['resolution'])), spec['origin']
        scenarios = c['scenarios']
    file = directory / 'frozen_input.txt'
    with file.open('w') as fp:
        fp.write(f'{width} {height} {resolution} ' + ' '.join(map(str, origin)) + '\n')
        fp.write(' '.join(str(int(grid[k])) if isinstance(grid[k], bool) else str(grid[k])
                          for k in ('occupied_threshold', 'unknown_is_obstacle', 'robot_radius',
                                    'safety_margin', 'inflate_map_boundary', 'max_cells')) + '\n')
        np.savetxt(fp, cells, fmt='%d')
        fp.write(str(len(scenarios)) + '\n')
        for scenario in scenarios:
            points = scenario.get('points', [])
            fp.write(f"{scenario['name']} {len(points)}\n")
            if points:
                for point in points:
                    fp.write(' '.join(map(str, point)) + '\n')
            else:
                fp.write(' '.join(map(str, scenario['start'] + scenario['goal'])) + '\n')
    command = ['ros2', 'run', 'plan', 'bezier_benchmark', '--ros-args',
               '--params-file', c['planner_params_file'], '--params-file', c['motion_limits_file'],
               '-p', f'benchmark_preserve_polyline:={str(args.synthetic).lower()}',
               '-p', 'use_sim_time:=false', '-p', f'benchmark_input_file:={file.resolve()}',
               '-p', f'benchmark_output_dir:={directory.resolve()}']
    patch = subprocess.check_output(['git', 'diff', 'HEAD', '--binary'])
    (directory / 'source.patch').write_bytes(patch)
    (directory / 'manifest.json').write_text(json.dumps({
        'kind': 'offline_geometry_and_speed_envelope; not Gazebo measurement',
        'configuration': c, 'grid_parameters': grid, 'scenarios': scenarios,
        'input_sha256': hashlib.sha256(file.read_bytes()).hexdigest(),
        'revision': subprocess.check_output(['git', 'rev-parse', 'HEAD']).decode().strip(),
        'tracked_diff_sha256': hashlib.sha256(patch).hexdigest(), 'command': command,
    }, indent=2) + '\n')
    for name in ('planner_params_file', 'motion_limits_file', 'grid_params_file',
                 'deployment_params_file'):
        (directory / Path(c[name]).name).write_bytes(Path(c[name]).read_bytes())
    print(directory, flush=True)
    if not args.prepare_only:
        environment = dict(os.environ, ROS_LOG_DIR=str((directory / "ros_logs").resolve()))
        subprocess.run(command, check=True, env=environment)


if __name__ == '__main__':
    main()
