#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Independently inspect frozen-region geometry, including all retained baseline portions."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from analyze_bezier_p1 import distances, geometry_id, load_path


def points(rows):
    return np.asarray([[r['x'], r['y']] for r in rows])


def project(point, rows):
    xy = points(rows)
    delta = np.diff(xy, axis=0)
    lengths2 = np.sum(delta**2, axis=1)
    t = np.clip(np.sum((point-xy[:-1])*delta, axis=1)/lengths2, 0, 1)
    error = np.linalg.norm(xy[:-1]+t[:, None]*delta-point, axis=1)
    index = int(np.argmin(error))
    return float(distances(rows)[index]+t[index]*math.sqrt(lengths2[index])), float(error[index])


def curve_from_knots(a, b):
    chord = np.linalg.norm(b[:2]-a[:2])
    da = chord*a[3]*np.array([math.cos(a[2]), math.sin(a[2])])
    db = chord*b[3]*np.array([math.cos(b[2]), math.sin(b[2])])
    dda = (chord*a[3])**2*a[4]*np.array([-math.sin(a[2]), math.cos(a[2])])
    ddb = (chord*b[3])**2*b[4]*np.array([-math.sin(b[2]), math.cos(b[2])])
    return np.array([a[:2], a[:2]+da/5, a[:2]+2*da/5+dda/20,
                     b[:2]-2*db/5+ddb/20, b[:2]-db/5, b[:2]])


def value(control, t):
    n = len(control)-1
    basis = np.array([math.comb(n, i)*t**i*(1-t)**(n-i) for i in range(n+1)]).T
    return basis@control


def geometry(control, t):
    d = 5*np.diff(control, axis=0)
    a = 4*np.diff(d, axis=0)
    j = 3*np.diff(a, axis=0)
    v, acc, jerk = (value(c, t) for c in (d, a, j))
    speed = np.linalg.norm(v, axis=1)
    cross = v[:, 0]*acc[:, 1]-v[:, 1]*acc[:, 0]
    k = cross/speed**3
    q = (v[:, 0]*jerk[:, 1]-v[:, 1]*jerk[:, 0])/speed**4 - (
        3*cross*np.sum(v*acc, axis=1)/speed**6)
    return value(control, t), k, q, speed, np.arctan2(v[:, 1], v[:, 0])


def inspect_curves(report, output):
    """Dense samples are diagnostic estimates, never replacements for interval certificates."""
    records, joins, controls = [], [], []
    for window in report['windows']:
        if window['result'] != 'accepted':
            continue
        knots = np.asarray(window['knots']).reshape(-1, 5)
        source = np.asarray(window['knots_reference_s'])+window['baseline_begin_s']
        previous = None
        for segment, (a, b) in enumerate(zip(knots, knots[1:])):
            control = curve_from_knots(a, b)
            controls.append({'window': window['id'], 'segment': segment,
                             'control_points': control.tolist(),
                             'reference_begin': float(source[segment]),
                             'reference_end': float(source[segment+1]),
                             'origin': 'independently_reconstructed_from_this_replay_final_knots'})
            t = np.linspace(0, 1, 513)
            xy, k, q, speed, yaw = geometry(control, t)
            if previous is not None:
                joins.append({'window': window['id'], 'segment': segment,
                              'position_gap': float(np.linalg.norm(xy[0]-previous[0])),
                              'heading_gap': float(abs(np.remainder(
                                  yaw[0]-previous[1]+math.pi, 2*math.pi)-math.pi)),
                              'curvature_gap': float(abs(k[0]-previous[2])),
                              'rate_gap': float(abs(q[0]-previous[3]))})
            previous = (xy[-1], yaw[-1], k[-1], q[-1])
            for i, u in enumerate(t):
                records.append({'window': window['id'], 'segment': segment, 't': float(u),
                                'reference_s': float(source[segment]+u*(
                                    source[segment+1]-source[segment])),
                                'x': float(xy[i, 0]), 'y': float(xy[i, 1]),
                                'curvature': float(k[i]), 'rate': float(q[i]),
                                'ds_weight': float(speed[i]/512*(.5 if i in (0, 512) else 1))})
    if records:
        with (output/'analytic_dense.csv').open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(records[0]))
            writer.writeheader()
            writer.writerows(records)
    (output/'curve_controls.json').write_text(json.dumps(controls, indent=2)+'\n')
    return records, joins


def region_metrics(rows, mapping, region, dense):
    indices = [i for i in range(len(rows)-1)
               if mapping[i+1] > region[0]+1e-10 and mapping[i] < region[1]-1e-10]
    if not indices:
        raise ValueError('Missing fixed-region coverage')
    length = rate_cost = 0.0
    for i in indices:
        ds = math.hypot(rows[i+1]['x']-rows[i]['x'], rows[i+1]['y']-rows[i]['y'])
        fraction = (min(mapping[i+1], region[1])-max(mapping[i], region[0]))/(
            mapping[i+1]-mapping[i])
        length += ds*fraction
        rate_cost += (rows[i+1]['curvature']-rows[i]['curvature'])**2/ds*fraction
    values = [r for r in dense if region[0] <= r['reference_s'] <= region[1]]
    k = np.abs([rows[i]['curvature'] for i in indices])
    result = {'K_bound': max(rows[i]['curvature_bound'] for i in indices),
              'Q_bound': max(rows[i]['curvature_rate_bound'] for i in indices),
              'nodal_peak': float(max(k)), 'nodal_rate_rms': math.sqrt(rate_cost/length),
              'nodal_rate_integral': rate_cost,
              'spikes_over_target': int(sum(k[i] > 1/.7 and k[i] > k[i-1] and k[i] >= k[i+1]
                                            for i in range(1, len(k)-1))),
              'analytic_covered_reference_length': 0.0}
    if values:
        weights = np.array([r['ds_weight'] for r in values])
        rates = np.array([r['rate'] for r in values])
        peak = max(values, key=lambda r: abs(r['curvature']))
        result.update(analytic_peak_sampled=abs(peak['curvature']), peak_location=peak,
                      analytic_rate_peak_sampled=float(np.max(abs(rates))),
                      analytic_rate_rms_sampled=float(np.sqrt(
                          np.sum(weights*rates**2)/sum(weights))))
        spans = {}
        for r in values:
            spans.setdefault((r['window'], r['segment']), []).append(r['reference_s'])
        result['analytic_covered_reference_length'] = sum(max(s)-min(s) for s in spans.values())
    return result


def freeze_regions(root, cases, config):
    destination = root/'regions.json'
    if destination.exists():
        return json.loads(destination.read_text())
    frozen = {}
    for case in cases:
        if case['synthetic']:
            safe = load_path(Path(case['profile']))
            frozen[case['name']] = {'regions': [[0, float(distances(safe)[-1])]],
                                    'basis': 'entire_frozen_synthetic_path'}
        elif case['name'] == 'old':
            frozen['old'] = {'regions': [config['p1']['front_region'],
                                         config['p1']['bottleneck_region']],
                             'basis': 'original_P1_frozen_regions'}
        else:
            historical = load_path(Path(case['source_record'])/'path_0001/planned_path.csv')
            safe = load_path(Path(case['profile']))
            old_s = distances(historical)
            regions, errors = [], []
            for lo, hi in config['refine']['historical_regions']:
                ends, gaps = [], []
                for s in (lo, hi):
                    point = np.array([np.interp(s, old_s, points(historical)[:, j])
                                      for j in (0, 1)])
                    mapped, gap = project(point, safe)
                    ends.append(mapped)
                    gaps.append(gap)
                if ends[0] >= ends[1]:
                    raise ValueError('Ambiguous historical-region projection')
                regions.append(ends)
                errors.append(gaps)
            frozen[case['name']] = {'regions': regions, 'endpoint_projection_errors': errors,
                                    'basis': 'historical_positions_projected_to_frozen_safe_path'}
    destination.write_text(json.dumps(frozen, indent=2)+'\n')
    return frozen


def analyze(root, plots=True, plot_variants=None):
    cases = json.loads((root/'inputs.json').read_text())
    config = json.loads((root/'configuration.json').read_text())
    regions = freeze_regions(root, cases, config)
    results = []
    for case in cases:
        assert hashlib.sha256(Path(case['profile']).read_bytes()).hexdigest() == (
            case['profile_sha256'])
        assert hashlib.sha256(Path(case['input']).read_bytes()).hexdigest() == case['input_sha256']
        safe = load_path(Path(case['profile']))
        for summary in sorted(root.glob(f'*/{case["name"]}_*/offline.jsonl')):
            data = json.loads(summary.read_text())
            variant = data['variant']
            rows = load_path(summary.parent/f'case_{variant}.csv')
            report = data['report']
            mapping = np.asarray(report['geometry_reference_s'])
            assert len(mapping) == len(rows) and np.all(np.diff(mapping) > 0)
            assert abs(mapping[-1]-distances(safe)[-1]) < 1e-7
            assert geometry_id(rows) == report['final_geometry_id']
            assert not report['dynamic_angular_budget']
            params = yaml_parameters(summary.parent/'planner.yaml')
            assert not params['enable_turn_speed_coordination']
            assert params['window_solver_seconds'] == .25 and params['window_total_seconds'] == 1
            for window in report['windows']:
                if window['result'] == 'accepted':
                    chosen = next(c for c in window['candidates']
                                  if c['id'] == window['selected_candidate'])
                    assert chosen['result'] == 'accepted'
                for candidate in window['candidates']:
                    if candidate['seed_type'] != 'local_refinement':
                        continue
                    if 'final_knots' not in candidate:
                        continue  # Explicitly skipped before a solver could run.
                    initial = np.asarray(candidate['initial_knots']).reshape(-1, 5)
                    final = np.asarray(candidate['final_knots']).reshape(-1, 5)
                    a, b = candidate['active_begin_knot'], candidate['active_end_knot']
                    assert np.array_equal(initial[:a+1], final[:a+1])
                    assert np.array_equal(initial[b:], final[b:])
            dense, joins = inspect_curves(report, summary.parent)
            sampled_k, signed_rate = [], []
            for i, (a, b) in enumerate(zip(rows, rows[1:])):
                ds = math.hypot(b['x']-a['x'], b['y']-a['y'])
                assert ds > 0
                assert a['curvature_bound']+1e-7 >= max(abs(a['curvature']), abs(b['curvature']))
                if not b['stop_required']:
                    assert a['curvature_rate_bound']+1e-7 >= abs(b['curvature']-a['curvature'])/ds
                signed_rate.append((b['curvature']-a['curvature'])/ds)
            for a, b, c in zip(rows, rows[1:], rows[2:]):
                ab = np.array([b['x']-a['x'], b['y']-a['y']])
                bc = np.array([c['x']-b['x'], c['y']-b['y']])
                denom = np.linalg.norm(ab)*np.linalg.norm(bc)*np.linalg.norm(ab+bc)
                sampled_k.append(float(2*np.cross(ab, bc)/denom) if denom > 0 else 0)
            with (summary.parent/'discrete_curvature.csv').open('w') as stream:
                writer = csv.writer(stream)
                writer.writerow(['reference_s', 'three_point_curvature', 'published_curvature'])
                writer.writerows(zip(mapping[1:-1], sampled_k,
                                     [r['curvature'] for r in rows[1:-1]]))
            result = {'case': case['name'], 'variant': variant,
                      'repeat': int(summary.parent.name.rsplit('_', 1)[1]),
                      'directory': str(summary.parent), 'length': float(distances(rows)[-1]),
                      'time': data['executable_time'], 'processing_seconds': data['seconds'],
                      'windows_seconds': sum(w['seconds'] for w in report['windows']
                                             if w['parent'] == -1),
                      'all_internal_G2': all(j['position_gap'] < 1e-8 and
                                             j['heading_gap'] < 1e-8 and
                                             j['curvature_gap'] < 1e-7 for j in joins),
                      'joins': joins, 'fallback': report['geometry_fallback_reason'],
                      'all_windows_skipped_refinement': all(
                          w.get('evaluation_basis') == 'legacy_mixed' for w in report['windows']),
                      'adopted_windows': sum(w['result'] == 'accepted' for w in report['windows']),
                      'regions': [region_metrics(rows, mapping, r, dense)
                                  for r in regions[case['name']]['regions']]}
            if case['name'] == 'old':
                result['old_P1_gate'] = (result['regions'][1]['K_bound'] <= 2.25+1e-7 and
                                         result['regions'][1]['Q_bound'] <= 30+1e-7)
            result['hard_checks_passed'] = (result['all_internal_G2'] and not result['fallback']
                                            and result['length'] <= 1.1*distances(safe)[-1]+1e-7
                                            and result['time'] <=
                                            1.1*report['frozen_baseline_time']+1e-7)
            results.append(result)
            if (dense and plots and result['repeat'] == 1 and
                    (not plot_variants or variant in plot_variants)):
                make_plot(summary.parent, rows, mapping, dense, regions[case['name']]['regions'])
    document = {'regions': regions, 'results': results,
                'interpretation': 'Analytic/nodal: estimates. K/Q_bound: certificates.',
                'analysis_script_sha256': hashlib.sha256(
                    Path(__file__).read_bytes()).hexdigest()}
    (root/'analysis.json').write_text(json.dumps(document, ensure_ascii=False, indent=2)+'\n')
    for variant in sorted({r['variant'] for r in results}):
        for case in cases:
            group = [r for r in results if r['variant'] == variant and r['case'] == case['name']]
            if group:
                region = -1
                print(variant, case['name'], 'n=', len(group),
                      'K=', [round(r['regions'][region]['K_bound'], 4) for r in group],
                      'Q=', [round(r['regions'][region]['Q_bound'], 4) for r in group],
                      'all_hard=', all(r['hard_checks_passed'] for r in group))
    return document


def acceptance(root, document, candidate, baseline='A', unchanged_tolerance=0.0):
    """Evaluate all repeats, including worst retained regions; never select a best-only run."""
    expected = json.loads((root/'configuration.json').read_text())['refine']['repeats']
    checks = {}
    proof_file = root/'scope_equivalence.json'
    proof = json.loads(proof_file.read_text()) if proof_file.exists() else {}
    binary_matches = bool(proof) and all(json.loads((root/v/'source.json').read_text())[
        'binary']['sha256'] == proof['binary_sha256'] for v in (baseline, candidate))
    for case in json.loads((root/'inputs.json').read_text()):
        groups = {v: [r for r in document['results'] if r['case'] == case['name'] and
                      r['variant'] == v] for v in (baseline, candidate)}
        complete = all(len(g) == expected for g in groups.values())
        result = {'complete': complete}
        if complete:
            old, new = groups[baseline], groups[candidate]
            region_results = []
            for i in range(len(old[0]['regions'])):
                old_k = max(r['regions'][i]['K_bound'] for r in old)
                new_k = max(r['regions'][i]['K_bound'] for r in new)
                old_q = max(r['regions'][i]['Q_bound'] for r in old)
                new_q = max(r['regions'][i]['Q_bound'] for r in new)
                old_rms = max(r['regions'][i]['nodal_rate_rms'] for r in old)
                new_rms = max(r['regions'][i]['nodal_rate_rms'] for r in new)
                region_results.append({
                    'K_before': old_k, 'K_after': new_k, 'Q_before': old_q, 'Q_after': new_q,
                    'rms_before': old_rms, 'rms_after': new_rms,
                    'K_nonincreasing': new_k <= old_k+1e-6,
                    'Q_nonincreasing': new_q <= old_q+1e-6,
                    'rms_nonincreasing': new_rms <= old_rms+1e-6})
            result.update(dict(regions=region_results,
                          hard_checks=all(r['hard_checks_passed'] for r in new),
                          old_P1_gate=all(r.get('old_P1_gate', True) for r in new),
                          K_nonincreasing=all(r['K_nonincreasing'] for r in region_results),
                          Q_nonincreasing=all(r['Q_nonincreasing'] for r in region_results),
                          rms_nonincreasing=all(r['rms_nonincreasing'] for r in region_results),
                          length_ratio_to_A=max(r['length'] for r in new) /
                          max(r['length'] for r in old),
                          time_ratio_to_A=max(r['time'] for r in new)/max(r['time'] for r in old),
                          max_processing_seconds=max(r['processing_seconds'] for r in new)))
        result['strict_nonregression_passed'] = complete and all(result.get(k, False) for k in (
            'K_nonincreasing', 'Q_nonincreasing', 'rms_nonincreasing'))
        scope_equal = complete and binary_matches and proof.get('cases', {}).get(
            case['name'], {}).get('passed', False) and all(
                r.get('all_windows_skipped_refinement', False)
                for group in groups.values() for r in group)
        result['unchanged_scope_within_variation'] = bool(scope_equal and all(
            r[after] <= r[before]*(1+unchanged_tolerance)+1e-6
            for r in result.get('regions', [])
            for before, after in [('K_before', 'K_after'), ('Q_before', 'Q_after'),
                                  ('rms_before', 'rms_after')]))
        result['passed'] = (complete and result.get('hard_checks', False) and
                            result.get('old_P1_gate', False) and (
                                result['strict_nonregression_passed'] or
                                result['unchanged_scope_within_variation']))
        checks[case['name']] = result
    passed = all(r['passed'] for r in checks.values())
    report = {'candidate': candidate, 'baseline': baseline,
              'offline_pass': passed, 'cases': checks,
              'expanded_budget_counts_for_acceptance': False,
              'tracking_validated': False,
              'unchanged_scope_relative_tolerance': unchanged_tolerance,
              'scope_equivalence_sha256': hashlib.sha256(proof_file.read_bytes()).hexdigest()
              if proof else None,
              'source_analysis_sha256': hashlib.sha256(
                  (root/'analysis.json').read_bytes()).hexdigest()}
    (root/f'acceptance_{candidate}.json').write_text(json.dumps(report, indent=2)+'\n')
    print('Acceptance', candidate, passed, {k: r['passed'] for k, r in checks.items()})
    return report


def yaml_parameters(path):
    import yaml
    return yaml.safe_load(path.read_text())['planner']['ros__parameters']['post_processing']


def make_plot(output, rows, mapping, dense, regions):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), constrained_layout=True)
    source = np.array([r['reference_s'] for r in dense])
    for axis, key in zip(axes[:2], ('curvature', 'rate')):
        axis.plot(source, [r[key] for r in dense], label='dense analytic (adopted curves)', lw=.8)
        for lo, hi in regions:
            axis.axvspan(lo, hi, alpha=.08, color='green')
        axis.grid(alpha=.3)
        axis.legend()
    axes[0].plot(mapping, [r['curvature'] for r in rows], '.', ms=2, label='published nodes')
    axes[0].legend()
    axes[0].set_ylabel('curvature [1/m]')
    axes[1].set_ylabel('d curvature / ds [1/m^2]')
    axes[1].set_xlabel('frozen reference progress [m]')
    axes[2].plot([r['x'] for r in rows], [r['y'] for r in rows])
    axes[2].axis('equal')
    axes[2].grid(alpha=.3)
    axes[2].set_xlabel('x [m]')
    axes[2].set_ylabel('y [m]')
    fig.savefig(output/'geometry.png', dpi=140)
    plt.close(fig)


def comparison_plot(root, document, candidate, baseline):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for case in ('latest', 'old'):
        regions = document['regions'][case]['regions']
        fig, axes = plt.subplots(len(regions), 2, figsize=(12, 3*len(regions)),
                                 squeeze=False, constrained_layout=True)
        for variant, color in ((baseline, 'tab:gray'), (candidate, 'tab:blue')):
            directory = root/variant/f'{case}_1'
            data = json.loads((directory/'offline.jsonl').read_text())
            rows = load_path(directory/f'case_{variant}.csv')
            source = data['report']['geometry_reference_s']
            for i, region in enumerate(regions):
                axes[i, 0].plot(source, [r['curvature'] for r in rows], color=color,
                                label=variant+' published curvature')
                axes[i, 1].step(source, [r['curvature_rate_bound'] for r in rows],
                                where='post', color=color, label=variant+' certified rate bound')
                for axis in axes[i]:
                    axis.set_xlim(*region)
                    axis.grid(alpha=.3)
                    axis.legend(fontsize=8)
                    axis.set_xlabel('frozen reference progress [m]')
                axes[i, 0].set_ylabel('curvature [1/m]')
                axes[i, 1].set_ylabel('rate bound [1/m^2]')
        fig.savefig(root/f'comparison_{case}.png', dpi=160)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--no-plots', action='store_true')
    parser.add_argument('--candidate')
    parser.add_argument('--baseline-variant', default='A')
    parser.add_argument('--unchanged-scope-tolerance', type=float, default=0.0)
    args = parser.parse_args()
    if not 0 <= args.unchanged_scope_tolerance <= .01:
        parser.error('Unchanged-scope tolerance must be within [0, 0.01]')
    variants = {args.candidate, args.baseline_variant} if args.candidate else None
    document = analyze(args.root, not args.no_plots, variants)
    if args.candidate:
        acceptance(args.root, document, args.candidate, args.baseline_variant,
                   args.unchanged_scope_tolerance)
        if not args.no_plots:
            comparison_plot(args.root, document, args.candidate, args.baseline_variant)


if __name__ == '__main__':
    main()
