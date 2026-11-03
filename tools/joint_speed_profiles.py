#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Direction reversal of certified curves for controlled simulation replay."""
from copy import deepcopy
import csv
import math

from analyze_bezier_p1 import geometry_id

FIELDS = ('index', 'x', 'y', 'yaw', 'curvature', 'stop_required', 'segment_type',
          'speed_limit', 'curvature_bound', 'curvature_rate_bound')


def reverse_profile(rows, result):
    if any(r['stop_required'] for r in rows[1:-1]):
        raise ValueError('Reverse replay requires geometry without internal stop discontinuities')
    output = []
    for i, source in enumerate(reversed(rows)):
        row = dict(source)
        row.update(index=i, yaw=math.remainder(source['yaw']+math.pi, 2*math.pi),
                   curvature=-source['curvature'], stop_required=int(i == len(rows)-1))
        if i+1 < len(rows):
            outgoing = rows[len(rows)-i-2]
            for key in ('segment_type', 'speed_limit', 'curvature_bound', 'curvature_rate_bound'):
                row[key] = outgoing[key]
        else:
            row.update(segment_type=0, speed_limit=0., curvature_bound=abs(row['curvature']),
                       curvature_rate_bound=0.)
        output.append(row)
    report = deepcopy(result)
    mapping = result['report']['geometry_reference_s']
    report['report']['geometry_reference_s'] = [mapping[-1]-s for s in reversed(mapping)]
    report['report']['final_geometry_id'] = geometry_id(output)
    # Original optimization candidates describe forward curves. Preserve them in
    # the source report, rather than presenting them as reverse optimization work.
    report['report']['windows'] = []
    report['derived_reverse'] = True
    return output, report


def write_profile(path, rows):
    with path.open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(FIELDS)
        for row in rows:
            writer.writerow([int(row[k]) if k in ('index', 'stop_required', 'segment_type')
                             else format(row[k], '.17g') for k in FIELDS])
