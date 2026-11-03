#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Plot a paired default-budget replay; never substitute sampled curves for certificates."""
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
    parser.add_argument('--attempt', required=True)
    parser.add_argument('--case', default='latest')
    args = parser.parse_args()
    work = args.root/'attempts'/args.attempt
    analysis = json.loads((work/'geometry_analysis.json').read_text())[args.case]
    regions = json.loads((args.root/'joint_regions.json').read_text())[args.case]
    target = next((i for i, r in enumerate(regions) if r['target']), 0)
    repeat = max(range(len(analysis['candidate'])),
                 key=lambda i: analysis['candidate'][i]['regions'][target]['K_bound'])+1
    figure, axes = plt.subplots(4, 1, figsize=(12, 15), constrained_layout=True)
    for root, group, label, color in ((work/'control', 'A0', 'Current geometry', '#53697d'),
                                     (work, 'B0', 'Feature refinement', '#007f71')):
        directory, result, rows = load_result(root, group, f'{args.case}_{repeat}')
        s = np.asarray(result['report']['geometry_reference_s'])
        xy = np.array([[r['x'], r['y']] for r in rows])
        axes[0].plot(xy[:, 0], xy[:, 1], label=label, color=color, linewidth=1.5)
        axes[1].plot(s, [r['curvature'] for r in rows], color=color, label=label)
        axes[2].plot(s[:-1], [r['curvature_rate_bound'] for r in rows[:-1]], color=color,
                     label=label, drawstyle='steps-post')
        speed = np.genfromtxt(directory/f'case_{group}_speed.csv', delimiter=',', names=True)
        reference_s = np.interp(speed['distance'], distances(rows), s)
        axes[3].plot(reference_s, speed['speed'], color=color,
                     label=f'{label}: estimated T={result["executable_time"]:.2f} s')
    axes[0].set(xlabel='x (m)', ylabel='y (m)', aspect='equal')
    axes[1].set(ylabel='Signed nodal curvature (1/m)')
    axes[2].set(ylabel='Certified outgoing rate bound (1/m²)')
    axes[3].set(xlabel='Frozen safe-path reference progress (m)',
                ylabel='Uncoordinated feasible speed (m/s)')
    for axis in axes[1:]:
        for region in regions:
            if region['target']:
                axis.axvspan(*region['range'], color='#e2b948', alpha=.12)
    for axis in axes:
        axis.legend(loc='best')
        axis.grid(alpha=.2)
    figure.suptitle(f'{args.case}: paired repeat {repeat} (candidate worst target K bound)\n'
                    'Offline geometry and speed envelopes; not Gazebo tracking')
    for extension in ('png', 'svg'):
        figure.savefig(work/f'geometry_comparison_{args.case}.{extension}', dpi=180)
    plt.close(figure)


if __name__ == '__main__':
    main()
