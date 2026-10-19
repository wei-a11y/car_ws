# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""ROS-independent loading, validation and whole-run tracking metrics."""

import json
from pathlib import Path

import numpy as np
import yaml


def load_config(path=None):
    if path is None:
        path = Path(__file__).resolve().parents[1] / 'config/path_tracking.yaml'
        if not path.exists():
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory('controller')) / 'config/path_tracking.yaml'
    with Path(path).open() as fp:
        config = yaml.safe_load(fp)
    config['analysis'] = config['analysis']['ros__parameters']
    return config


def duration_of_runs(time, mask, minimum_duration):
    """Sample intervals, not sample counts; a run may span a simulation pause."""
    time = np.asarray(time, dtype=float)
    mask = np.asarray(mask, dtype=bool)
    runs = []
    start = None
    for i, active in enumerate(mask):
        if active and start is None:
            start = i
        if start is not None and (not active or i == len(mask) - 1):
            end = i if not active else len(mask) - 1
            duration = float(time[end] - time[start])
            if duration >= minimum_duration:
                runs.append(duration)
            start = None
    return runs


def error_metrics(errors, threshold):
    values = np.asarray(errors, dtype=float)
    if not len(values):
        return None
    return {
        'mae': float(np.mean(np.abs(values))),
        'rmse': float(np.sqrt(np.mean(values ** 2))),
        'max_abs': float(np.max(np.abs(values))),
        'p95_abs': float(np.percentile(np.abs(values), 95)),
        'exceed_percent': float(np.mean(np.abs(values) > threshold) * 100),
    }


def summarize(planned, actual, config):
    required = {'time', 'x', 'y', 'yaw', 'progress', 'remaining', 'lateral_error', 'heading_error',
                'curvature', 'v_actual', 'state', 'geometry_valid'}
    if not required.issubset(actual.columns):
        raise ValueError(
            'Recording has no synchronized controller errors; record with the new recorder')
    if len(planned) < 2 or not len(actual):
        raise ValueError('Need a path with at least two points and recorded controller samples')
    numeric = actual[list(required)].to_numpy(dtype=float)
    if not np.isfinite(numeric).all() or np.any(np.diff(actual['time']) <= 0):
        raise ValueError('Nonfinite data or non-increasing simulation timestamps')
    valid = actual[actual['geometry_valid'] > 0].copy()
    if not len(valid):
        raise ValueError('No valid path-projection samples in this epoch')
    if np.any(np.diff(valid['progress']) < -1e-9):
        raise ValueError('Progress moved backwards; do not mix different path epochs')
    # Invalid/fault samples remain in the CSV, but cannot dilute error statistics.
    curve = np.abs(valid['curvature']) >= config['curve_curvature_threshold']
    threshold = config['lateral_error_threshold']
    planned_length = float(np.hypot(np.diff(planned['x']), np.diff(planned['y'])).sum())
    if not np.isfinite(planned_length) or planned_length <= 0:
        raise ValueError('Reference path must have finite positive length')
    actual_length = float(np.hypot(np.diff(actual['x']), np.diff(actual['y'])).sum())
    states = actual['state'].to_numpy(dtype=int)
    tracking = states == 2
    unexpected = (tracking & (np.abs(actual['v_actual']) <= config['stopped_speed_threshold']) &
                  (actual['progress'] > config['stop_exclusion_distance']) &
                  (actual['remaining'] > config['stop_exclusion_distance']))
    stops = duration_of_runs(actual['time'], unexpected, config['stop_min_duration'])
    return valid, {
        'samples': len(actual), 'valid_error_samples': len(valid),
        'planned_length_m': planned_length, 'recorded_length_m': actual_length,
        'progress_coverage_percent': float(np.clip(
            valid['progress'].max() / planned_length * 100, 0, 100)),
        'goal_reached': bool(np.any(states == 4)),
        'duration_sec': float(actual['time'].iloc[-1] - actual['time'].iloc[0]),
        'lateral_m': error_metrics(valid['lateral_error'], threshold),
        'heading_rad': error_metrics(valid['heading_error'], config['heading_error_threshold']),
        'curve_lateral_m': error_metrics(valid.loc[curve, 'lateral_error'], threshold),
        'straight_lateral_m': error_metrics(valid.loc[~curve, 'lateral_error'], threshold),
        'unexpected_stop_count': len(stops), 'unexpected_stop_duration_sec': sum(stops),
        'thresholds': config,
        'pose_source': 'TF localization estimate; not Gazebo ground truth',
    }


def save_summary(path, summary):
    Path(path).write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
