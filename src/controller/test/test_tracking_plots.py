# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
import json
from pathlib import Path
import struct
import sys

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pytest

matplotlib.use('Agg')
SCRIPTS = str(Path(__file__).parents[1] / 'scripts')
sys.path.insert(0, SCRIPTS)
from analyze_path_tracking import analyze, compare  # noqa: E402
from tracking_analysis import derive, load_config, summarize  # noqa: E402
from tracking_plots import (quantity_figure, speed_position_figure,  # noqa: E402
                            style, trajectory_figure)


def epoch(path, projection=True, offset=0.):
    path.mkdir()
    n = 9
    pd.DataFrame({'x': [0., 2.], 'y': [0., 0.],
                  'stop_required': [0, 1]}).to_csv(path / 'planned_path.csv', index=False)
    data = pd.DataFrame({
        'time': np.arange(n) * .1, 'ros_time': np.arange(n) * .1 + 10,
        'x': np.linspace(0, 1, n), 'y': [.02 + offset] * n, 'yaw': [0.] * n,
        'progress': np.linspace(0, 1, n), 'remaining': np.linspace(2, 1, n),
        'lateral_error': [.02 + offset] * n, 'heading_error': [2., .1] + [.02] * (n - 2),
        'v_actual': [.1] * n, 'v_ref': [.12] * n, 'v_cmd': [.11] * n,
        'w_cmd': [.1] * n, 'w_actual': [.09] * n, 'w_ff': [.08] * n,
        'curvature': [0., 0., 1., 1., 1., 0., 0., 0., 0.],
        'state': [1, 1, 2, 2, 2, 2, 2, 2, 4], 'geometry_valid': [1] * n})
    if projection:
        data['nearest_x'], data['nearest_y'] = data['x'], 0.
    data.to_csv(path / 'actual_path.csv', index=False)
    (path / 'path.json').write_text('{"frame":"map"}')
    return path


def test_export_progress_png_and_vector_overlay_without_touching_raw(tmp_path):
    path = epoch(tmp_path / 'path_0001')
    cfg = load_config()['analysis']
    raw = (path / 'actual_path.csv').read_bytes()
    historical = path / 'tracking_overview.png'
    historical.write_bytes(b'historical')
    obsolete = [path / 'figures' / name for name in (
        'longitudinal_speed_time.png', 'longitudinal_speed_time.svg',
        'heading_error_tracking_time.pdf', 'turn_0001_longitudinal_speed_time.png',
        'turn_0001_position_error_progress.png', 'heading_error_tracking_progress.png',
        'lateral_error_tracking_progress.png', 'position_error_tracking_progress.png')]
    obsolete[0].parent.mkdir()
    for p in obsolete:
        p.write_text('obsolete generated export')
    unrelated = path / 'figures/user_notes.png'
    unrelated.write_bytes(b'user picture')
    summary = analyze(path, cfg, show=False)
    manifest = json.loads((path / 'figures/plots_manifest.json').read_text())
    files = [Path(p) for p in manifest['files']]
    stems = {p.stem for p in files}
    assert 'position_error_progress' in stems
    assert not any('_tracking_' in stem for stem in stems)
    assert not any(p.stem.endswith('_time') or p.stem.startswith('turn_') for p in files)
    assert all(p.suffix == '.png' for p in files if p.stem != 'speed_position_error_progress')
    svg = path / 'figures/speed_position_error_progress.svg'
    assert svg.exists() and '<image' not in svg.read_text()
    assert (path / 'figures/speed_position_error_progress.png').exists()
    assert manifest['label_language'] == 'zh-CN'
    assert not any(p.exists() for p in obsolete)
    assert unrelated.read_bytes() == b'user picture'
    png = path / 'figures/longitudinal_speed_progress.png'
    assert struct.unpack('>II', png.read_bytes()[16:24]) == (3600, 1500)
    assert summary['position_m']['max_abs'] == pytest.approx(.02)
    assert (path / 'actual_path.csv').read_bytes() == raw
    assert historical.read_bytes() == b'historical'
    assert not plt.get_fignums()


def test_one_quantity_axes_tracking_zoom_and_gaps(tmp_path):
    path = epoch(tmp_path / 'path_0001')
    cfg = load_config()['analysis']
    planned, raw = pd.read_csv(path / 'planned_path.csv'), pd.read_csv(path / 'actual_path.csv')
    raw.loc[4:, 'time'] += 1
    _, summary = summarize(planned, raw, cfg)
    data, _, _ = derive(planned, raw, cfg)
    full = quantity_figure(data, summary, 'heading_error', 'progress', cfg)
    tracking = quantity_figure(data, summary, 'heading_error', 'progress', cfg, tracking_only=True)
    assert len(full.axes) == len(tracking.axes) == 1
    assert tracking.axes[0].get_title() == '航向误差（正常跟踪阶段）'
    assert full.axes[0].get_xlabel() == '路径进度 [m]'
    assert full.axes[0].get_ylabel() == '航向误差 [度]'
    labels = [text.get_text() for text in full.axes[0].get_legend().get_texts()]
    assert '航向误差' in labels and '允许误差阈值' in labels
    from matplotlib.font_manager import FontProperties, findfont
    from matplotlib.ft2font import FT2Font
    with style(cfg):
        font = FT2Font(findfont(FontProperties()))
    assert all(font.get_char_index(ord(c)) for c in '纵向速度曲率横向误差航向位置减速起点')
    assert full.axes[0].get_ylim()[1] > tracking.axes[0].get_ylim()[1]
    assert np.isnan(tracking.axes[0].lines[0].get_ydata()).any()
    assert len(trajectory_figure(planned, data, cfg, 'test').axes) == 2  # data + colorbar
    plt.close('all')


def test_combined_full_route_preserves_stops_gaps_and_independent_units(tmp_path):
    path = epoch(tmp_path / 'path_0001')
    cfg = load_config()['analysis']
    planned, raw = pd.read_csv(path / 'planned_path.csv'), pd.read_csv(path / 'actual_path.csv')
    raw.loc[3, 'progress'] = raw.loc[2, 'progress']  # standstill: retain repeated progress
    raw.loc[3, 'v_actual'] = 0.
    raw.loc[4:, 'time'] += 1.
    raw.loc[6, 'geometry_valid'] = 0
    data, _, _ = derive(planned, raw, cfg)
    _, summary = summarize(planned, raw, cfg)
    fig = speed_position_figure(planned, data, summary, cfg, 'test')
    assert len(fig.axes) == 2
    speed, error = fig.axes
    assert speed.get_position().bounds == error.get_position().bounds
    assert speed.get_xlim() == pytest.approx((0., 2.))
    assert speed.get_ylabel() == '纵向速度 [m/s]'
    assert error.get_ylabel() == '位置距离误差 [m]'
    actual = next(line for line in speed.lines if line.get_label() == '实际速度（里程计）')
    assert actual.get_xdata()[2] == actual.get_xdata()[3]
    assert actual.get_ydata()[3] == 0.
    assert np.isnan(actual.get_ydata()[4])  # missing time interval is not joined
    assert np.isnan(actual.get_ydata()[7])  # invalid geometry is not plotted
    distance = error.lines[0].get_ydata()
    assert distance[np.isfinite(distance)] == pytest.approx(.02)
    assert error.lines[1].get_ydata()[0] == cfg['position_error_threshold']
    plt.close('all')


def test_legacy_skip_position_and_explicit_comparison(tmp_path):
    baseline = epoch(tmp_path / 'path_0001', projection=False)
    candidate = epoch(tmp_path / 'path_0002', projection=False, offset=.01)
    cfg = load_config()['analysis']
    cfg.update(formats=['png'], plot_quantities=['longitudinal_speed', 'position_error'],
               plot_tracking_errors=False)
    summary = analyze(baseline, cfg, show=False)
    assert summary['position_m'] is None
    assert not (baseline / 'figures/position_error_progress.png').exists()
    assert not (baseline / 'figures/speed_position_error_progress.svg').exists()
    comparison = tmp_path / 'comparison'
    comparison.mkdir()
    stale = comparison / 'longitudinal_speed_comparison_time.png'
    stale.write_bytes(b'old time-axis plot')
    report = compare(baseline, candidate, cfg, output=tmp_path / 'comparison')
    assert report['frames'] == ['map', 'map']
    assert len(report['skipped']) == 1
    assert not stale.exists()
    assert all(Path(p).stem.endswith('_progress') or Path(p).stem == 'trajectory_comparison'
               for p in report['files'])
    assert report['metric_changes_candidate_minus_baseline']['lateral_m']['mae'] == \
        pytest.approx(.01)
    assert report['baseline_summary']['position_m'] is None
    assert (tmp_path / 'comparison/comparison_summary.json').exists()
    assert report['notes']
    with pytest.raises(ValueError, match='one recorded path epoch'):
        compare(tmp_path, candidate, cfg)


def test_reverse_routes_are_not_registered_and_one_sided_distance_is_not_compared(tmp_path):
    baseline = epoch(tmp_path / 'path_0001', projection=False)
    candidate = epoch(tmp_path / 'path_0002')
    planned = pd.read_csv(candidate / 'planned_path.csv').iloc[::-1]
    planned.to_csv(candidate / 'planned_path.csv', index=False)
    cfg = load_config()['analysis']
    cfg.update(formats=['png'], plot_quantities=['position_error'],
               plot_tracking_errors=False)
    report = compare(baseline, candidate, cfg, output=tmp_path / 'reverse_comparison')
    assert report['route_changed']
    assert report['endpoints'][0]['start'] == [0., 0.]
    assert report['endpoints'][1]['start'] == [2., 0.]
    assert len(report['skipped']) == 1
    assert report['metric_changes_candidate_minus_baseline']['position_m'] is None
    assert not (tmp_path / 'reverse_comparison/position_error_comparison_progress.png').exists()


def test_cli_rejects_ambiguous_comparison_arguments(tmp_path):
    import subprocess
    script = Path(SCRIPTS) / 'analyze_path_tracking.py'
    result = subprocess.run(
        [sys.executable, str(script), '--baseline', str(tmp_path), '--no-show'],
        capture_output=True, text=True)
    assert result.returncode != 0
    assert 'must be specified together' in result.stderr
