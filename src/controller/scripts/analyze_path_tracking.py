#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Three views: route overlay, signed lateral error, signed heading error."""

import argparse
from pathlib import Path

import matplotlib
import numpy as np
import pandas as pd

from tracking_analysis import load_config, save_summary, summarize


def analyze(directory, config, show=False):
    import matplotlib.pyplot as plt

    planned = pd.read_csv(directory / 'planned_path.csv')
    actual = pd.read_csv(directory / 'actual_path.csv')
    valid, summary = summarize(planned, actual, config)
    save_summary(directory / 'tracking_summary.json', summary)
    valid.to_csv(directory / 'tracking_error.csv', index=False)
    axis = config['error_axis']
    if axis not in ('progress', 'time'):
        raise ValueError('error_axis must be progress or time')
    x = valid[axis].to_numpy()
    xlabel = 'Path progress [m]' if axis == 'progress' else 'Simulation elapsed time [s]'
    curve = np.abs(valid['curvature'].to_numpy()) >= config['curve_curvature_threshold']
    fig, axes = plt.subplots(3, 1, figsize=tuple(config['figure_size']))
    ax = axes[0]
    ax.plot(planned['x'].to_numpy(), planned['y'].to_numpy(), '--', label='Processed reference')
    ax.plot(actual['x'].to_numpy(), actual['y'].to_numpy(), linewidth=1, label='Tracked TF pose')
    color = ax.scatter(valid['x'], valid['y'], c=np.abs(valid['lateral_error']),
                       s=config['point_size'], cmap='viridis', vmin=0,
                       vmax=config['lateral_error_threshold'])
    fig.colorbar(color, ax=ax, label='Absolute lateral error [m]')
    if 'stop_required' in planned:
        stops = planned[planned['stop_required'].fillna(0) > 0]
        ax.scatter(stops['x'], stops['y'], marker='x', color='red', label='Required stops / goal')
    ax.scatter(planned['x'].iloc[0], planned['y'].iloc[0], marker='o', label='Start')
    ax.set(xlabel='X [m]', ylabel='Y [m]', title=f'{directory.name}: whole-route tracking')
    ax.set_aspect('equal', adjustable='datalim')
    ax.legend(loc='best')
    lateral_threshold = config['lateral_error_threshold']
    heading_threshold = np.degrees(config['heading_error_threshold'])
    for ax, values, threshold, label in (
            (axes[1], valid['lateral_error'].to_numpy(), lateral_threshold, 'Lateral error [m]'),
            (axes[2], np.degrees(valid['heading_error'].to_numpy()), heading_threshold,
             'Heading error [deg]')):
        ax.plot(x, values, linewidth=1)
        ax.axhline(0, color='black', linewidth=0.5)
        ax.axhline(threshold, linestyle='--', color='red', label='Allowed error')
        ax.axhline(-threshold, linestyle='--', color='red')
        ax.fill_between(x, 0, 1, where=curve, transform=ax.get_xaxis_transform(),
                        color='orange', alpha=0.12, label='Curve region')
        ax.set(xlabel=xlabel, ylabel=label)
        extent = max(float(np.max(np.abs(values))), float(threshold))
        ax.set_ylim(-extent * (1 + config['error_axis_margin']),
                    extent * (1 + config['error_axis_margin']))
        ax.legend(loc='best')
    for ax in axes:
        ax.grid(True, alpha=0.3)
    m = summary['lateral_m']
    fig.suptitle(f"RMSE {m['rmse']:.3f} m | P95 {m['p95_abs']:.3f} m | "
                 f"max {m['max_abs']:.3f} m | "
                 f"coverage {summary['progress_coverage_percent']:.1f}%")
    fig.tight_layout()
    fig.savefig(directory / 'tracking_overview.png', dpi=config['dpi'])
    if show:
        plt.show()
    plt.close(fig)
    print(f"{directory}: RMSE={m['rmse']:.4f} m, P95={m['p95_abs']:.4f} m, "
          f"unexpected stops={summary['unexpected_stop_count']}, "
          f"goal_reached={summary['goal_reached']}")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, help='path_tracking.yaml')
    parser.add_argument('--data-dir', type=Path,
                        help='one path epoch, a run, or the recording root')
    parser.add_argument('--show', action='store_true')
    args = parser.parse_args()
    config = load_config(args.config)
    root = args.data_dir or Path(config['path_recorder']['ros__parameters']['output_dir'])
    if not args.show:
        matplotlib.use('Agg')
    if (root / 'planned_path.csv').exists():
        epochs = [root]
    elif (root / 'manifest.json').exists():
        epochs = sorted(root.glob('path_*'))
    else:
        runs = sorted(root.glob('run_*'))
        if not runs:
            parser.error(f'No recording found in {root}')
        epochs = sorted(runs[-1].glob('path_*'))
    results = []
    for epoch in epochs:
        try:
            results.append({'epoch': epoch.name, **analyze(epoch, config['analysis'], args.show)})
        except (ValueError, pd.errors.EmptyDataError) as error:
            print(f'{epoch}: skipped: {error}')
    if not results:
        parser.error('No analyzable epochs; keep the recorder running while following a path')
    save_summary(epochs[0].parent / 'run_summary.json', results)


if __name__ == '__main__':
    main()
