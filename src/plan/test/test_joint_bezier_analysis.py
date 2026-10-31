# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Acceptance must reject incomplete or locally regressing experiments."""
from copy import deepcopy
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'tools'))
from analyze_joint_bezier import geometry_gate, same_geometry  # noqa: E402


def sample(k=2., q=10., rms=4.):
    return {'regions': [{'K_bound': k, 'Q_bound': q, 'nodal_rate_rms': rms}],
            'joins_valid': True, 'stops': 0, 'fallback': '', 'time': 40.}


SETTINGS = {'repeats': 5, 'acceptance': {'curvature_reduction': .1,
            'rate_rms_reduction': .15, 'regression_time_loss': .02}}
REGIONS = [{'range': [1., 4.], 'target': True}]


def test_rejects_missing_repeat_and_keeps_worst_run():
    old = [sample() for _ in range(5)]
    new = [sample(1.7, 9., 3.) for _ in range(5)]
    assert geometry_gate(old, new, REGIONS, SETTINGS, True)['pass']
    assert not geometry_gate(old, new[:-1], REGIONS, SETTINGS, True)['pass']
    new[-1]['regions'][0]['Q_bound'] = 11.
    assert not geometry_gate(old, new, REGIONS, SETTINGS, True)['pass']


def test_rejects_rate_regression_even_when_curvature_improves():
    old = [sample() for _ in range(5)]
    new = [sample(1.5, 8., 3.8) for _ in range(5)]
    assert not geometry_gate(old, new, REGIONS, SETTINGS, True)['pass']


def test_speed_pair_includes_continuous_certificates():
    row = {k: 0. for k in ('x', 'y', 'yaw', 'curvature', 'stop_required', 'segment_type',
                           'curvature_bound', 'curvature_rate_bound')}
    row['speed_limit'] = .3
    changed = deepcopy(row)
    changed['speed_limit'] = .2
    assert same_geometry([row], [changed])
    changed['curvature_rate_bound'] = 1.
    assert not same_geometry([row], [changed])
