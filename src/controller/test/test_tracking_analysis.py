# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
import importlib.util
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

SPEC = importlib.util.spec_from_file_location(
    'tracking_analysis', Path(__file__).parents[1] / 'scripts/tracking_analysis.py')
analysis = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analysis)


def test_recording_root_ignores_legacy_csvs_when_runs_exist(tmp_path):
    (tmp_path / 'planned_path.csv').write_text('index,x,y\n0,0,0\n')
    (tmp_path / 'actual_path.csv').write_text('time,x,y,yaw\n0,0,0,0\n')
    old_run = tmp_path / 'run_20261019_203326_360761'
    new_run = tmp_path / 'run_20261019_210334_835379'
    for run in (new_run, old_run):
        run.mkdir()
        (run / 'manifest.json').write_text('{}')
        (run / 'path_0001').mkdir()
    second = new_run / 'path_0002'
    second.mkdir()
    (tmp_path / 'run_99999999_note').write_text('not a run directory')
    (new_run / 'path_notes.txt').write_text('not a path directory')

    assert analysis.recording_epochs(tmp_path) == [new_run / 'path_0001', second]
    assert analysis.recording_epochs(old_run) == [old_run / 'path_0001']
    epoch = old_run / 'path_0001'
    (epoch / 'planned_path.csv').touch()
    assert analysis.recording_epochs(epoch) == [epoch]


def test_empty_latest_run_does_not_fall_back_to_old_data(tmp_path):
    (tmp_path / 'planned_path.csv').touch()
    latest = tmp_path / 'run_20261019_210334_835379'
    latest.mkdir()
    (latest / 'manifest.json').write_text('{}')
    assert analysis.recording_epochs(tmp_path) == []


def test_legacy_recording_and_missing_recording(tmp_path):
    with pytest.raises(ValueError, match='No recording found'):
        analysis.recording_epochs(tmp_path)
    (tmp_path / 'planned_path.csv').touch()
    assert analysis.recording_epochs(tmp_path) == [tmp_path]


def test_signed_errors_curves_incomplete_coverage_and_stop_duration():
    config = analysis.load_config()['analysis']
    planned = pd.DataFrame({'x': [0., 2.], 'y': [0., 0.]})
    actual = pd.DataFrame({
        'time': [0., .2, .4, .6, .8, 1.], 'x': [0., .2, .3, .3, .3, .4],
        'y': [0., -.02, .02, .04, .02, 0.], 'yaw': [0.] * 6,
        'progress': [0., .2, .3, .3, .3, .4], 'remaining': [2.] * 6,
        'lateral_error': [0., -.02, .02, .04, .02, 0.],
        'heading_error': [0., -.1, .1, .2, .1, 0.],
        'curvature': [0., 0., 1., 1., 1., 0.],
        'v_actual': [.1, .1, 0., 0., 0., .1], 'state': [2] * 6, 'geometry_valid': [1] * 6})
    valid, summary = analysis.summarize(planned, actual, config)
    assert valid['lateral_error'].min() < 0
    assert summary['lateral_m']['rmse'] == pytest.approx(np.sqrt(.0028 / 6))
    assert summary['curve_lateral_m']['max_abs'] == .04
    assert summary['progress_coverage_percent'] == 20.
    assert summary['unexpected_stop_count'] == 1
    assert summary['unexpected_stop_duration_sec'] == pytest.approx(.6)
    assert not summary['goal_reached']
    actual.loc[4, 'progress'] = .1
    with pytest.raises(ValueError, match='backwards'):
        analysis.summarize(planned, actual, config)


def test_duration_uses_timestamps_instead_of_record_count():
    assert analysis.duration_of_runs([0., .1, .4, .9], [False, True, True, False], .3) == [.8]
    assert analysis.duration_of_runs([0., .1, .2], [True, False, False], .3) == []


def samples(time=None):
    time = np.array(time if time is not None else [0., .1, .2, .3, .4, .5, .6])
    n = len(time)
    return pd.DataFrame({
        'time': time, 'ros_time': time + 100., 'x': np.linspace(.2, 1.2, n),
        'y': [.04] * n, 'yaw': [0.] * n, 'progress': np.linspace(.2, 1.2, n),
        'remaining': np.linspace(1.8, .8, n), 'lateral_error': [.04] * n,
        'heading_error': [.02] * n, 'curvature': [1.] * n,
        'v_actual': [.1] * n, 'v_ref': [.1] * n, 'v_cmd': [.1] * n,
        'w_actual': [.1] * n, 'w_cmd': [.1] * n, 'w_ff': [.1] * n,
        'state': [2] * n, 'geometry_valid': [1] * n,
        'nearest_x': np.linspace(.2, 1.2, n), 'nearest_y': [0.] * n})


def test_position_distance_uses_recorded_projection_and_old_csv_remains_unavailable():
    cfg = analysis.load_config()['analysis']
    planned = pd.DataFrame({'x': [0., 2.], 'y': [0., 0.]})
    data = samples()
    data['x'] += .03
    data.loc[0, 'lateral_error'] = -.04
    valid, summary = analysis.summarize(planned, data, cfg)
    assert valid['position_error'].iloc[0] == pytest.approx(.05)
    assert summary['position_m']['mae'] == pytest.approx(.05)
    assert summary['lateral_m']['mae'] == pytest.approx(.04)
    event = next(e for e in summary['events'] if e['type'] == 'maximum_position_error')
    assert event['position_error'] == pytest.approx(.05)
    _, old = analysis.summarize(planned, data.drop(columns=['nearest_x', 'nearest_y']), cfg)
    assert old['position_m'] is None
    assert old['tracking']['position_m'] is None
    assert old['availability']['position_error'].startswith('unavailable')


def test_acceleration_nonuniform_timestamps_and_gap_reset():
    cfg = analysis.load_config()['analysis']
    t = np.array([0., .1, .3, 1., 1.1])
    raw, smooth = analysis.acceleration(t, np.array([0., .1, .3, .3, .4]),
                                        np.ones(5, dtype=bool), cfg)
    assert np.isnan(raw[[0, 3]]).all()
    assert raw[[1, 2, 4]] == pytest.approx([1., 1., 1.])
    assert smooth[[1, 2, 4]] == pytest.approx([1., 1., 1.])


def test_time_metrics_exclude_invalid_samples_and_large_gaps():
    cfg = analysis.load_config()['analysis']
    data = samples([0., .1, .2, .9, 1.])
    data['lateral_error'] = .2
    data['geometry_valid'] = [1, 0, 1, 1, 1]
    planned = pd.DataFrame({'x': [0., 2.], 'y': [0., 0.]})
    _, summary = analysis.summarize(planned, data, cfg)
    assert summary['lateral_m']['valid_duration_sec'] == pytest.approx(.1)
    assert summary['lateral_m']['exceed_duration_sec'] == pytest.approx(.1)
    assert summary['lateral_m']['exceed_time_percent'] == pytest.approx(100.)


def test_turn_merge_breaks_at_braking_invalid_samples_and_planned_stops():
    cfg = analysis.load_config()['analysis']
    data = samples()
    data['progress'] = np.linspace(.1, .22, len(data))
    data['curvature'] = [1., 1., 0., 1., 0., 0., 1.]
    planned = pd.DataFrame({'x': [0., 2.], 'y': [0., 0.]})
    assert len(analysis.turn_intervals(data, planned, cfg)) == 1
    data.loc[4, 'state'] = 3
    assert len(analysis.turn_intervals(data, planned, cfg)) == 2
    data.loc[4, 'state'] = 2
    data.loc[4, 'geometry_valid'] = 0
    assert len(analysis.turn_intervals(data, planned, cfg)) == 2
    data.loc[4, 'geometry_valid'] = 1
    planned = pd.DataFrame({'x': [0., .18, 2.], 'y': [0., 0., 0.],
                            'stop_required': [0, 1, 1]})
    assert len(analysis.turn_intervals(data, planned, cfg)) == 2


def test_state_motion_and_geometry_are_independent_and_events_include_realign_goal():
    cfg = analysis.load_config()['analysis']
    data = samples(np.arange(12) * .1)
    data['v_actual'] = np.arange(12) * .02
    data['state'] = [1, 2, 2, 2, 2, 2, 3, 1, 1, 2, 2, 4]
    planned = pd.DataFrame({'x': [0., 2.], 'y': [0., 0.]})
    valid, summary = analysis.summarize(planned, data, cfg)
    assert ((valid['path_type'] == 'curve') & (valid['motion_action'] == 'accelerating')).any()
    kinds = [e['type'] for e in summary['events']]
    assert 'initial_alignment' in kinds and 'realignment' in kinds and 'goal_reached' in kinds
    assert summary['tracking']['samples'] < summary['valid_error_samples']
    assert summary['goal_reached']


@pytest.mark.parametrize('key,value', [
    ('max_sample_gap', 0), ('formats', ['jpg']),
    ('plot_axes', ['time']), ('combined_formats', ['jpg']), ('dpi', float('nan'))])
def test_invalid_plot_analysis_parameters(key, value):
    cfg = analysis.load_config()['analysis']
    cfg[key] = value
    with pytest.raises(ValueError):
        analysis.validate_config(cfg)
