#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Check P1 frozen-region gates; expanded-budget diagnostics never count as default acceptance."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import struct

import numpy as np


def load_path(file):
    with file.open() as stream:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    return rows


def distances(rows):
    return np.r_[0, np.cumsum([math.hypot(b['x']-a['x'], b['y']-a['y'])
                              for a, b in zip(rows, rows[1:])])]


def geometry_id(rows):
    value = 14695981039346656037
    for row in rows:
        for k in ('x', 'y', 'yaw', 'curvature', 'stop_required'):
            for byte in struct.pack('<d', row[k]):
                value = ((value ^ byte) * 1099511628211) & ((1 << 64)-1)
    return f'{value:016x}'


def region_metrics(rows, mapping, region):
    indices = [i for i in range(len(rows)-1) if mapping[i+1] > region[0]+1e-10
               and mapping[i] < region[1]-1e-10]
    if not indices:
        raise ValueError(f'No path coverage for fixed region {region}')
    peak = max(rows[i]['curvature_bound'] for i in indices)
    rate = max(rows[i]['curvature_rate_bound'] for i in indices)
    curvature_cost = rate_cost = 0
    for i in indices:
        a, b = rows[i], rows[i+1]
        ds = math.hypot(b['x']-a['x'], b['y']-a['y'])
        curvature_cost += ds * (a['curvature']**2 + a['curvature']*b['curvature'] +
                                b['curvature']**2)/3
        rate_cost += (b['curvature']-a['curvature'])**2/ds
    # Monotonic magnitude ramps; zero crossings/sign changes delimit ramps.
    ramps = []
    k = np.asarray([r['curvature'] for r in rows])
    s = distances(rows)
    begin = indices[0]
    trend = 0
    for i in range(begin+1, indices[-1]+2):
        delta = abs(k[i])-abs(k[i-1])
        sign = 1 if delta > 1e-8 else -1 if delta < -1e-8 else 0
        if (sign and trend and sign != trend) or k[i]*k[i-1] < 0:
            if i-1 > begin and abs(abs(k[i-1])-abs(k[begin])) > 1e-6:
                lo = min(abs(k[begin]), abs(k[i-1]))
                span = abs(abs(k[i-1])-abs(k[begin]))
                crossings = []
                for level in (.1, .9):
                    target = lo+level*span
                    for j in range(begin+1, i):
                        a, b = abs(k[j-1]), abs(k[j])
                        if a != b and min(a, b) <= target <= max(a, b):
                            crossings.append(s[j-1]+(target-a)/(b-a)*(s[j]-s[j-1]))
                            break
                if len(crossings) == 2:
                    ramps.append(abs(crossings[1]-crossings[0]))
            begin, trend = i-1, 0
        if sign:
            trend = sign
    return {'region': region, 'K': peak, 'Q': rate, 'curvature_integral': curvature_cost,
            'nodal_rate_integral': rate_cost, 'ramp_10_90_distances': ramps,
            'minimum_published_cap': min(rows[i]['speed_limit'] for i in indices)}


def analyze(root):
    manifest = json.loads((root/'manifest.json').read_text())
    settings = manifest['configuration']['p1']
    cases = [json.loads(line) for line in (root/'offline.jsonl').read_text().splitlines()]
    results = []
    for name in sorted({c['scenario'] for c in cases}):
        by_variant = {c['variant']: c for c in cases if c['scenario'] == name}
        assert set(by_variant) == {'baseline', 'legacy', 'geometry_first'}
        paths = {v: load_path(root/f'{name}_{v}.csv') for v in by_variant}
        baseline = paths['baseline']
        report = by_variant['geometry_first']['report']
        mapping = np.asarray(report['geometry_reference_s'])
        assert len(mapping) == len(paths['geometry_first'])
        assert np.all(np.diff(mapping) > 0)
        assert abs(mapping[0]) < 1e-8 and abs(mapping[-1]-distances(baseline)[-1]) < 1e-8
        assert geometry_id(paths['geometry_first']) == report['final_geometry_id']
        for row in report['windows']:
            assert all(key in row for key in ('id', 'parent', 'search_end', 'selected_candidate'))
            ids = [candidate['id'] for candidate in row['candidates']]
            assert len(set(ids)) == len(ids)
            if row['result'] == 'accepted':
                assert row['selected_candidate'] in ids
                chosen = next(a for a in row['candidates'] if a['id'] == row['selected_candidate'])
                assert chosen['result'] == 'accepted'
        for a, b in zip(paths['geometry_first'], paths['geometry_first'][1:]):
            ds = math.hypot(b['x']-a['x'], b['y']-a['y'])
            assert ds > 0 and a['curvature_bound']+1e-7 >= abs(a['curvature'])
            if not b['stop_required']:
                assert a['curvature_rate_bound']+1e-7 >= abs(b['curvature']-a['curvature'])/ds
        metrics = {}
        for v, rows in paths.items():
            source = mapping if v == 'geometry_first' else distances(rows)
            metrics[v] = {'length': distances(rows)[-1],
                          'time': by_variant[v]['executable_time'],
                          'processing_seconds': by_variant[v]['seconds']}
            # Fixed historical ROI applies only to the matching replay input.
            if manifest['mode'] == 'replay':
                metrics[v].update({key: region_metrics(rows, source, settings[f'{key}_region'])
                                   for key in ('front', 'bottleneck')})
        gate = None
        if manifest['mode'] == 'replay':
            old = metrics['baseline']['bottleneck']
            new = metrics['geometry_first']['bottleneck']
            front = settings['front_region']
            front_adopted = any(w['result'] == 'accepted' and w['baseline_begin_s'] < front[1]
                                and w['baseline_end_s'] > front[0] for w in report['windows'])
            gate = {'curvature_pass': new['K'] <= old['K']*(
                1-settings['required_curvature_reduction'])+1e-7,
                    'rate_pass': new['Q'] <= settings['required_rate_bound']+1e-7,
                    'front_adopted': front_adopted,
                    'length_pass': metrics['geometry_first']['length'] <=
                    metrics['baseline']['length']*1.1+1e-7,
                    'time_pass': metrics['geometry_first']['time'] <=
                    report['frozen_baseline_time']*1.1+1e-7,
                    'no_global_fallback': not report['geometry_fallback_reason']}
            gate = {key: bool(value) for key, value in gate.items()}
            gate['passed'] = all(gate.values())
            gate['counts_for_default_acceptance'] = manifest['budget'] == 'default'
        result = {'scenario': name, 'metrics': metrics, 'gate': gate,
                  'accepted_windows': [w['id'] for w in report['windows']
                                       if w['result'] == 'accepted'],
                  'final_geometry_id': report['final_geometry_id']}
        results.append(result)
        if manifest['mode'] == 'replay':
            plot(root, name, paths, mapping, settings)
    document = {'mode': manifest['mode'], 'budget': manifest['budget'], 'source_manifest_sha256':
                hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest(),
                'analysis_script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                'integrity_checks_passed': True, 'scenarios': results}
    (root/'p1_analysis.json').write_text(json.dumps(document, ensure_ascii=False, indent=2)+'\n')
    return document


def plot(root, name, paths, mapping, settings):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.family'] = ['Noto Sans CJK JP', 'DejaVu Sans']
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    labels = {'baseline': '原方法', 'legacy': '旧 Bézier', 'geometry_first': '新 Bézier'}
    for v, rows in paths.items():
        source = mapping if v == 'geometry_first' else distances(rows)
        for column, key in enumerate(('front', 'bottleneck')):
            lo, hi = settings[f'{key}_region']
            begin = max(0, np.searchsorted(source, lo)-1)
            end = min(len(rows), np.searchsorted(source, hi)+1)
            for axis, field in zip(axes[:, column], ('curvature', 'curvature_rate_bound')):
                values = [r[field] for r in rows[begin:end]]
                if field == 'curvature_rate_bound':
                    axis.step(source[begin:end], values, where='post', label=labels[v])
                else:
                    axis.plot(source[begin:end], values, label=labels[v])
                axis.set_xlim(lo, hi)
                axis.grid(alpha=.3)
                axis.set_xlabel('基线对应进度 / m（旧方案为自身进度）')
        axes[0, 0].set_ylabel('节点曲率 [1/m]')
        axes[1, 0].set_ylabel('变化率区间上界 [1/m^2]')
    axes[0, 0].set_title('前段固定区域')
    axes[0, 1].set_title('后段固定区域（包含回退部分）')
    for axis in axes.flat:
        axis.legend()
    fig.savefig(root/f'{name}_geometry_comparison.png', dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', type=Path, nargs='+')
    args = parser.parse_args()
    for directory in args.directories:
        result = analyze(directory)
        print(directory, json.dumps({
            'integrity_checks_passed': True, 'gates': [s['gate'] for s in result['scenarios']]},
            ensure_ascii=False))


if __name__ == '__main__':
    main()
