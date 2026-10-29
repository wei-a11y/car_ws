#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Export the complete A/B trajectory and speed comparison as PNG and SVG."""
import argparse
import json
import os
from pathlib import Path
import sys

os.environ.setdefault(
    'MPLCONFIGDIR', str(Path(__file__).resolve().parents[1] / 'log/bezier_window/matplotlib'))

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/controller/scripts'))
from tracking_analysis import load_config  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-dir', type=Path, required=True)
    parser.add_argument('--config', default='src/controller/config/path_tracking.yaml')
    args = parser.parse_args()
    c = load_config(args.config)['analysis']
    plt.rcParams['font.sans-serif'] = c['font_families']
    plt.rcParams['axes.unicode_minus'] = False
    figure, axes = plt.subplots(2, 2, figsize=(14, 10), constrained_layout=True)
    colors = {'baseline': '#b34727', 'combined': '#176ca3'}
    names = {'baseline': '原方法', 'combined': '组合方案'}
    for row, (scenario, title) in enumerate((('outward', '去程'), ('inward', '回程'))):
        for variant in ('baseline', 'combined'):
            first = True
            for file in sorted(args.data_dir.glob(f'{variant}_{scenario}_*/result.json')):
                report = json.loads(file.read_text())
                epochs = sorted(file.parent.glob('record/run_*/path_*/actual_path.csv'))
                if len(epochs) != 1:
                    continue
                actual = pd.read_csv(epochs[0])
                planned = pd.read_csv(epochs[0].with_name('planned_path.csv'))
                color, label = colors[variant], names[variant] if first else None
                if first:
                    axes[row, 0].plot(
                        planned.x.to_numpy(), planned.y.to_numpy(), '--', color=color,
                        linewidth=1, alpha=.8)
                    axes[row, 1].plot(
                        actual.progress.to_numpy(), actual.v_ref.to_numpy(), '--', color=color,
                        linewidth=1, alpha=.6)
                axes[row, 0].plot(
                    actual.x.to_numpy(), actual.y.to_numpy(), color=color, linewidth=1,
                    alpha=.65, label=label)
                axes[row, 1].plot(
                    actual.progress.to_numpy(), actual.v_actual.to_numpy(), color=color,
                    linewidth=1, alpha=.65, label=label)
                if not report['goal_reached']:
                    axes[row, 0].plot(actual.x.iloc[-1], actual.y.iloc[-1], 'x', color='red')
                first = False
        axes[row, 0].set(title=f'{title}路径：虚线参考，实线 TF 定位轨迹', xlabel='x / m', ylabel='y / m')
        axes[row, 0].set_aspect('equal', adjustable='box')
        axes[row, 1].set(title=f'{title}速度：虚线参考，实线里程计速度',
                         xlabel='路径进度 / m', ylabel='速度 / (m/s)')
        for speed in (c['target_turn_speed_min'], c['target_turn_speed_preferred']):
            axes[row, 1].axhline(speed, color='gray', linewidth=.6, alpha=.5)
        for axis in axes[row]:
            axis.grid(alpha=.2)
            axis.legend(loc='best')
    figure.suptitle('固定地图与起终位姿下的 Gazebo A/B（每方向各三次）', fontsize=15)
    figure.savefig(args.data_dir / 'comparison.png', dpi=c['dpi'])
    figure.savefig(args.data_dir / 'comparison.svg')
    plt.close(figure)


if __name__ == '__main__':
    main()
