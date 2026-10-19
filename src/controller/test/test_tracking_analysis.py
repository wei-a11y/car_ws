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
