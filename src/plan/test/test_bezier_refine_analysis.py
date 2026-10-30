# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Independent geometry evaluation must include fallback portions and unequal spans."""
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'tools'))
from analyze_bezier_refine import (  # noqa: E402
    curve_from_knots, geometry, project, region_metrics,
)


def row(x, curvature=0, bound=0, rate_bound=0):
    return dict(x=x, y=0., yaw=0., curvature=curvature, curvature_bound=bound,
                curvature_rate_bound=rate_bound, stop_required=0., speed_limit=.2,
                segment_type=1.)


def test_rate_rms_uses_arc_length_and_clips_region_boundaries():
    rows = [row(0, 0, 2, 2), row(1, 2, 2, 1), row(3, 0, 0, 0)]
    result = region_metrics(rows, [0, 1, 3], [.5, 2], [])
    assert result['nodal_rate_integral'] == pytest.approx(3.)
    assert result['nodal_rate_rms'] == pytest.approx(math.sqrt(2.))
    assert result['K_bound'] == 2
    assert result['Q_bound'] == 2


def test_retained_interval_is_not_hidden_by_adopted_analytic_samples():
    rows = [row(0, 0, 1, 3), row(1, 1, 2.5, 62.5), row(2, 2.5)]
    dense = [dict(reference_s=.5, ds_weight=1, rate=.1, curvature=.5,
                  window=1, segment=0)]
    result = region_metrics(rows, [0, 1, 2], [0, 2], dense)
    assert result['K_bound'] == 2.5
    assert result['Q_bound'] == 62.5
    assert result['analytic_peak_sampled'] == .5
    assert result['analytic_covered_reference_length'] < 2


def test_projection_uses_segments_instead_of_nearest_node():
    reference = [row(0), row(2)]
    source, gap = project(np.array([.75, .2]), reference)
    assert source == pytest.approx(.75)
    assert gap == pytest.approx(.2)


def test_quintic_endpoints_and_curvature_match_shared_geometric_knots():
    a = np.array([0., 0., .1, .8, .2])
    b = np.array([.1, .01, .3, 1., 1.2])
    c = np.array([1., .5, .4, 1.1, -.2])
    left = geometry(curve_from_knots(a, b), np.array([0., 1.]))
    right = geometry(curve_from_knots(b, c), np.array([0., 1.]))
    np.testing.assert_allclose(left[0][-1], right[0][0], atol=1e-12)
    assert left[1][-1] == pytest.approx(b[4], abs=1e-10)
    assert right[1][0] == pytest.approx(b[4], abs=1e-10)
    assert left[4][-1] == pytest.approx(right[4][0], abs=1e-12)


def test_dense_analytic_rate_matches_finite_difference():
    a = np.array([0., 0., .1, 1., .2])
    b = np.array([1., .4, .3, 1., 1.2])
    curve = curve_from_knots(a, b)
    t, h = .37, 1e-6
    center = geometry(curve, np.array([t]))
    ends = geometry(curve, np.array([t-h, t+h]))
    numerical = (ends[1][1]-ends[1][0])/(2*h*center[3][0])
    assert center[2][0] == pytest.approx(numerical, abs=1e-6)


def test_acceptance_keeps_worst_repeat_instead_of_best_only(tmp_path):
    import json
    from analyze_bezier_refine import acceptance
    (tmp_path/'configuration.json').write_text(json.dumps({'refine': {'repeats': 2}}))
    (tmp_path/'inputs.json').write_text(json.dumps([{'name': 'latest'}]))
    (tmp_path/'analysis.json').write_text('{}')
    results = []
    for variant, peaks in [('A', [2., 2.]), ('E', [1., 3.])]:
        for k in peaks:
            results.append(dict(case='latest', variant=variant, hard_checks_passed=True,
                                regions=[dict(K_bound=k, Q_bound=5., nodal_rate_rms=1.)],
                                length=10., time=30., processing_seconds=.5))
    report = acceptance(tmp_path, {'results': results}, 'E')
    assert not report['offline_pass']
    assert report['cases']['latest']['regions'][0]['K_after'] == 3.
    assert not report['cases']['latest']['K_nonincreasing']


def test_runtime_variation_tolerance_never_applies_to_active_refinement(tmp_path):
    import json
    from analyze_bezier_refine import acceptance
    (tmp_path/'configuration.json').write_text(json.dumps({'refine': {'repeats': 1}}))
    (tmp_path/'inputs.json').write_text(json.dumps([{'name': 'latest'}]))
    (tmp_path/'analysis.json').write_text('{}')
    proof = dict(binary_sha256='binary', cases={'latest': {'passed': True}})
    (tmp_path/'scope_equivalence.json').write_text(json.dumps(proof))
    results = []
    for variant, k in [('A', 2.), ('E', 2.001)]:
        (tmp_path/variant).mkdir()
        (tmp_path/variant/'source.json').write_text(json.dumps({'binary': {'sha256': 'binary'}}))
        results.append(dict(case='latest', variant=variant, hard_checks_passed=True,
                            all_windows_skipped_refinement=False,
                            regions=[dict(K_bound=k, Q_bound=5., nodal_rate_rms=1.)],
                            length=10., time=30., processing_seconds=.5))
    report = acceptance(tmp_path, {'results': results}, 'E', unchanged_tolerance=.01)
    assert not report['offline_pass']
    assert not report['cases']['latest']['unchanged_scope_within_variation']
