#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Fixed-work equivalence proof only; these runs never count as optimization gains."""
import argparse
import json
from pathlib import Path

from analyze_joint_bezier import load_result, same_geometry
from run_bezier_refine import check_protected, sha, snapshot, write_json
from run_joint_bezier import execute


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    from ament_index_python.packages import get_package_prefix
    binary = (Path(get_package_prefix('plan'))/'lib/plan/bezier_benchmark').resolve()
    cases = json.loads((args.root/'inputs.json').read_text())
    check_protected(args.root)
    args.output.mkdir(parents=True, exist_ok=False)
    snapshot(args.output, binary)
    proof = {'diagnostic_only': True, 'binary_sha256': sha(binary), 'cases': {}}
    for case in cases:
        if sha(case['input']) != case['input_sha256'] or (
                sha(case['profile']) != case['profile_sha256']):
            raise ValueError('Frozen safe input changed')
        execute(binary, case, args.output/'A0'/case['name'], 'A0', case['profile'], fixed_work=True)
        execute(binary, case, args.output/'B0'/case['name'], 'B0', case['profile'],
                {'window_feature_refinement': True}, fixed_work=True)
        _, _, before = load_result(args.output, 'A0', case['name'])
        _, result, after = load_result(args.output, 'B0', case['name'])
        active = any(w.get('feature_refinement') for w in result['report']['windows'])
        equal = same_geometry(before, after) and len(before) == len(after) and all(
            a['speed_limit'] == b['speed_limit'] for a, b in zip(before, after))
        proof['cases'][case['name']] = {'feature_active': active,
                                      'all_geometry_and_cap_fields_equal': equal,
                                      'pass': active or equal}
        write_json(args.output/'scope_equivalence.json', proof)
    proof['pass'] = all(c['pass'] for c in proof['cases'].values())
    write_json(args.output/'scope_equivalence.json', proof)
    print(json.dumps(proof, indent=2))


if __name__ == '__main__':
    main()
