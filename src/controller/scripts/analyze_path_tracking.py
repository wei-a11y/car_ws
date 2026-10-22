#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Chinese tracking plots, event metrics and explicit experiment comparisons."""

import argparse
import json
from pathlib import Path

import matplotlib
import numpy as np
import pandas as pd

from tracking_analysis import derive, load_config, recording_epochs, save_summary, summarize
from tracking_plots import (comparison_figure, export_plots, figure, gapped, prune_generated_plots,
                            save_figure, style)


def read_epoch(directory, config):
    directory = Path(directory)
    planned = pd.read_csv(directory / 'planned_path.csv')
    actual = pd.read_csv(directory / 'actual_path.csv')
    _, summary = summarize(planned, actual, config)
    data, _, _ = derive(planned, actual, config)
    return planned, data, summary


def analyze(directory, config, show=False):
    directory = Path(directory)
    planned, data, summary = read_epoch(directory, config)
    save_summary(directory / 'tracking_summary.json', summary)
    # Derived samples retain invalid rows and missing-interval separators, not just valid geometry.
    data.to_csv(directory / 'tracking_error.csv', index=False)
    export_plots(directory, planned, data, summary, config, show)
    m = summary['lateral_m']
    print(f"{directory}: RMSE={m['rmse']:.4f} m, P95={m['p95_abs']:.4f} m, "
          f"tracking low-speed stops={summary['unexpected_stop_count']}, "
          f"goal_reached={summary['goal_reached']}; independent plots in figures/")
    return summary


def recorded_metadata(directory):
    result = {}
    for name in ('path.json', 'experiment.json', 'planning_report.json'):
        path = directory / name
        result[name] = json.loads(path.read_text()) if path.is_file() else None
    return result


def differences(a, b, prefix=''):
    if isinstance(a, dict) and isinstance(b, dict):
        result = []
        for key in sorted(a.keys() | b.keys()):
            result += differences(a.get(key), b.get(key), prefix + '/' + str(key))
        return result
    return [] if a == b else [{'field': prefix, 'baseline': a, 'candidate': b}]


def metric_delta(a, b):
    result = {}
    for key in a.keys() & b.keys():
        x, y = a[key], b[key]
        if isinstance(x, dict) and isinstance(y, dict):
            result[key] = metric_delta(x, y)
        elif x is None or y is None:
            result[key] = None
        elif (isinstance(x, (int, float)) and not isinstance(x, bool) and
              isinstance(y, (int, float)) and not isinstance(y, bool)):
            result[key] = y - x
    return result


def compare(baseline, candidate, config, show=False, output=None):
    baseline, candidate = Path(baseline), Path(candidate)
    # Only explicit single-epoch paths are accepted; never infer paired routes from whole runs.
    for path in (baseline, candidate):
        if not (path / 'planned_path.csv').is_file() or not (path / 'actual_path.csv').is_file():
            raise ValueError(f'Comparison requires one recorded path epoch: {path}')
    left, right = read_epoch(baseline, config), read_epoch(candidate, config)
    output = Path(output) if output else candidate.parent / 'comparisons' / (
        baseline.parent.name + '_' + baseline.name + '_vs_' + candidate.name)
    output.mkdir(parents=True, exist_ok=True)
    meta_a, meta_b = recorded_metadata(baseline), recorded_metadata(candidate)
    endpoints = []
    for planned, _, _ in (left, right):
        endpoints.append({'start': planned[['x', 'y']].iloc[0].tolist(),
                          'end': planned[['x', 'y']].iloc[-1].tolist()})
    path_a, path_b = left[0][['x', 'y']].to_numpy(), right[0][['x', 'y']].to_numpy()
    frames = [(m['path.json'] or {}).get('frame') for m in (meta_a, meta_b)]
    same_frame = frames[0] is not None and frames[0] == frames[1]
    notes = ['No route reversal, spatial registration or same-location correspondence applied.']
    if meta_a['experiment.json'] is None or meta_b['experiment.json'] is None:
        notes.append('Historical experiment parameters/map snapshot not recorded; '
                     'not reconstructed.')
    if not same_frame:
        notes.append('Trajectory overlay skipped: common recorded coordinate frame unavailable.')
    metric_keys = ('lateral_m', 'heading_rad', 'position_m', 'tracking', 'duration_sec',
                   'progress_coverage_percent', 'unexpected_stop_count')
    report = {'baseline': str(baseline.resolve()), 'candidate': str(candidate.resolve()),
              'comparison_basis': 'independent own path progress; '
                                  'not point pairing',
              'notes': notes, 'frames': frames, 'endpoints': endpoints,
              'route_changed': path_a.shape != path_b.shape or not np.array_equal(path_a, path_b),
              'metadata_differences': differences(meta_a, meta_b),
              'baseline_summary': left[2], 'candidate_summary': right[2],
              'metric_changes_candidate_minus_baseline': metric_delta(
                  {k: left[2][k] for k in metric_keys}, {k: right[2][k] for k in metric_keys}),
              'files': [], 'skipped': []}
    with style(config):
        if config['plot_trajectory'] and same_frame:
            fig, ax = figure(config)
            for (planned, data, _), label, color in zip(
                    (left, right), ('基线实验', '待比较实验'), ('C0', 'C1')):
                ax.plot(planned['x'].to_numpy(), planned['y'].to_numpy(), '--', color=color,
                        label=label + '参考路径')
                valid = data['geometry_valid'].to_numpy() > 0
                x, y = data['x'].to_numpy().copy(), data['y'].to_numpy().copy()
                x[~valid], y[~valid] = np.nan, np.nan
                x, y = gapped(x, y, data['time'].to_numpy(), config['max_sample_gap'])
                ax.plot(x, y, color=color, label=label + '实际轨迹（TF 定位）')
            ax.set(xlabel='地图 X 坐标 [m]', ylabel='地图 Y 坐标 [m]',
                   title='显式路线对比（未进行空间配准）')
            ax.set_aspect('equal', adjustable='datalim')
            ax.grid(True, alpha=.3)
            ax.legend(loc='best')
            report['files'] += save_figure(fig, output, 'trajectory_comparison', config, show)
        for quantity in config['plot_quantities']:
            tracking_views = config['plot_tracking_errors'] and quantity.endswith('error')
            scopes = [False, True] if tracking_views else [False]
            for tracking in scopes:
                for axis in config['plot_axes']:
                    stem = quantity + ('_tracking' if tracking else '') + '_comparison_' + axis
                    fig = comparison_figure(left[1], right[1], quantity, axis, config, tracking)
                    if fig is None:
                        report['skipped'].append(
                            {'plot': stem, 'reason': 'values missing in one or both inputs'})
                    else:
                        report['files'] += save_figure(fig, output, stem, config, show)
    output.mkdir(parents=True, exist_ok=True)
    prune_generated_plots(output, report['files'], comparison=True)
    save_summary(output / 'comparison_summary.json', report)
    print(f'Explicit comparison saved in {output}')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, help='path_tracking.yaml')
    parser.add_argument('--data-dir', type=Path, help='one path epoch, a run, or recording root')
    parser.add_argument('--baseline', type=Path, help='explicit baseline path epoch')
    parser.add_argument('--candidate', type=Path, help='explicit candidate path epoch')
    parser.add_argument('--output-dir', type=Path, help='comparison output directory')
    parser.add_argument('--show', action=argparse.BooleanOptionalAction, default=True,
                        help='show each independent plot (default: enabled; --no-show saves only)')
    args = parser.parse_args()
    if bool(args.baseline) != bool(args.candidate):
        parser.error('--baseline and --candidate must be specified together')
    if args.baseline and args.data_dir:
        parser.error('--data-dir cannot be combined with comparison inputs')
    if args.output_dir and not args.baseline:
        parser.error('--output-dir is for explicit comparisons')
    if not args.show:
        matplotlib.use('Agg')
    try:
        config = load_config(args.config)
        if args.baseline:
            compare(args.baseline, args.candidate, config['analysis'], args.show, args.output_dir)
            return
        root = args.data_dir or Path(config['path_recorder']['ros__parameters']['output_dir'])
        epochs = recording_epochs(root)
        results = []
        for epoch in epochs:
            try:
                summary = analyze(epoch, config['analysis'], args.show)
                results.append({'epoch': epoch.name, **summary})
            except (ValueError, pd.errors.EmptyDataError) as error:
                print(f'{epoch}: skipped: {error}')
        if not results:
            parser.error('No analyzable epochs; keep the recorder running while following a path')
        save_summary(epochs[0].parent / 'run_summary.json', results)
    except (ValueError, OSError, pd.errors.EmptyDataError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
