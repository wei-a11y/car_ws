#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Plot six-way fixed-geometry speed replay, including time/event tradeoffs."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from analyze_bezier_p1 import distances  # noqa: E402
from analyze_joint_bezier import load_result  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--case', default='latest')
    args = parser.parse_args()
    name = args.case+'_1'
    manifest = json.loads((args.root/'geometry_inputs.json').read_text())[name]
    analysis = json.loads((args.root/'analysis.json').read_text())[args.case]
    figure, axes = plt.subplots(3, 1, figsize=(12, 11), constrained_layout=True)
    labels = {'A0': 'Current / uncoordinated', 'A': 'Current / old coordination',
              'B0': 'Refined / uncoordinated', 'B': 'Refined / old coordination',
              'C': 'Current / bounded coordination', 'D': 'Refined / bounded coordination'}
    colors = {'A0': '#8196a6', 'A': '#af423d', 'B0': '#8196a6', 'B': '#af423d',
              'C': '#007f71', 'D': '#007f71'}
    for index, groups in enumerate((('A0', 'A', 'C'), ('B0', 'B', 'D'))):
        source = Path(manifest[groups[0]]['path']).with_name(groups[0]+'_report.json')
        mapping = json.loads(source.read_text())['report']['geometry_reference_s']
        for group in groups:
            directory, _, rows = load_result(args.root, group, name)
            speed = np.genfromtxt(directory/f'case_{group}_speed.csv', delimiter=',', names=True)
            s = np.interp(speed['distance'], distances(rows), mapping)
            axes[index].plot(s, speed['speed'], label=labels[group], color=colors[group],
                             linestyle='--' if group.endswith('0') else '-')
        axes[index].set(xlabel='Frozen safe-path reference progress (m)', ylabel='Speed (m/s)')
        axes[index].legend(loc='best')
        axes[index].grid(alpha=.2)
    for group in ('A0', 'A', 'B0', 'B', 'C', 'D'):
        x = [r[group]['time'] for r in analysis]
        y = [r[group]['events'] for r in analysis]
        axes[2].scatter(x, y, label=group, s=35)
        axes[2].annotate(group, (max(x), max(y)), xytext=(5, 5), textcoords='offset points')
    axes[2].set(xlabel='Estimated total time (s), all five replays', ylabel='Acceleration/deceleration events')
    axes[2].grid(alpha=.2)
    figure.suptitle(args.case+': frozen geometry and speed-only replay; not Gazebo execution')
    for extension in ('png', 'svg'):
        figure.savefig(args.root/f'speed_comparison_{args.case}.{extension}', dpi=180)
    plt.close(figure)


if __name__ == '__main__':
    main()
