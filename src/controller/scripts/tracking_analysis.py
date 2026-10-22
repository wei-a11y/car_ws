# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""ROS-independent synchronized tracking analysis; raw recordings are never rewritten."""

import json
from pathlib import Path

import numpy as np
import yaml


STATES = {0: 'WAIT_PATH', 1: 'ALIGNING', 2: 'TRACKING', 3: 'BRAKING',
          4: 'GOAL_REACHED', 5: 'TRACKING_ERROR'}
ERRORS = {'lateral_m': ('lateral_error', 'lateral_error_threshold'),
          'heading_rad': ('heading_error', 'heading_error_threshold'),
          'position_m': ('position_error', 'position_error_threshold')}


def load_config(path=None):
    default = Path(__file__).resolve().parents[1] / 'config/path_tracking.yaml'
    if not default.exists():
        from ament_index_python.packages import get_package_share_directory
        default = Path(get_package_share_directory('controller')) / 'config/path_tracking.yaml'
    with default.open() as fp:
        config = yaml.safe_load(fp)
    if path is not None:
        with Path(path).open() as fp:
            override = yaml.safe_load(fp)
        for section in ('path_recorder', 'analysis'):
            config[section]['ros__parameters'].update(
                override.get(section, {}).get('ros__parameters', {}))
    config['analysis'] = config['analysis']['ros__parameters']
    validate_config(config['analysis'])
    return config


def validate_config(c):
    positive = ('position_error_threshold', 'lateral_error_threshold', 'heading_error_threshold',
                'curve_curvature_threshold', 'stopped_speed_threshold', 'stop_min_duration',
                'max_sample_gap', 'acceleration_filter_tau', 'acceleration_threshold',
                'motion_min_duration', 'font_size', 'line_width', 'point_size', 'dpi')
    for name in positive:
        if not np.isfinite(c[name]) or c[name] <= 0:
            raise ValueError(f'{name} must be finite and positive')
    for name in ('turn_merge_distance', 'turn_merge_time',
                 'stop_exclusion_distance', 'error_axis_margin', 'event_label_spacing_fraction'):
        if not np.isfinite(c[name]) or c[name] < 0:
            raise ValueError(f'{name} must be finite and nonnegative')
    if len(c['figure_size']) != 2 or any(
            not np.isfinite(x) or x <= 0 for x in c['figure_size']):
        raise ValueError('figure_size must have two positive finite values')
    if (not c['font_families'] or not isinstance(c['font_families'], list) or
            any(not isinstance(name, str) or not name.strip() for name in c['font_families'])):
        raise ValueError('font_families must be a nonempty list of font names')
    if (type(c['combined_legend_columns']) is not int or c['combined_legend_columns'] < 1):
        raise ValueError('combined_legend_columns must be a positive integer')
    choices = {'formats': {'svg', 'pdf', 'png'},
               'combined_formats': {'svg', 'pdf', 'png'}, 'plot_axes': {'progress'},
               'plot_quantities': {'longitudinal_speed', 'angular_velocity', 'curvature',
                                   'acceleration', 'lateral_error', 'heading_error',
                                   'position_error'}}
    for key, allowed in choices.items():
        if not c[key] or len(c[key]) != len(set(c[key])) or not set(c[key]) <= allowed:
            raise ValueError(f'Invalid or duplicate {key}')


def recording_epochs(root):
    """Keep the established latest-run selection, including an empty latest run."""
    root = Path(root)
    if (root / 'manifest.json').is_file():
        return sorted(p for p in root.glob('path_*') if p.is_dir())
    runs = sorted(p for p in root.glob('run_*') if p.is_dir())
    if runs:
        return sorted(p for p in runs[-1].glob('path_*') if p.is_dir())
    if (root / 'planned_path.csv').is_file():
        return [root]
    raise ValueError(f'No recording found in {root}')


def duration_of_runs(time, mask, minimum_duration):
    # Retained public helper for existing callers; timestamp intervals, not sample counts.
    return [float(time[end] - time[begin]) for begin, end in intervals(time, mask)
            if time[end] - time[begin] >= minimum_duration - 1e-9]


def intervals(time, mask, maximum_gap=np.inf):
    """Return inclusive sample bounds; active left samples own [t_i, t_i+1)."""
    time, mask = np.asarray(time), np.asarray(mask, dtype=bool)
    result, start = [], None
    for i, active in enumerate(mask):
        gap = i > 0 and time[i] - time[i - 1] > maximum_gap + 1e-9
        if gap and start is not None:
            result.append((start, i - 1))
            start = None
        if active and start is None:
            start = i
        if start is not None and (not active or i == len(mask) - 1):
            result.append((start, i))
            start = None
    return result


def error_metrics(errors, threshold):
    values = np.asarray(errors, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return {'mae': float(np.mean(np.abs(values))),
            'rmse': float(np.sqrt(np.mean(values ** 2))),
            'max_abs': float(np.max(np.abs(values))),
            'p95_abs': float(np.percentile(np.abs(values), 95)),
            'exceed_percent': float(np.mean(np.abs(values) > threshold) * 100)}


def metric_group(data, mask, config):
    t = data['time'].to_numpy()
    dt = np.diff(t)
    connected = (dt <= config['max_sample_gap'] + 1e-9)
    valid = data['geometry_valid'].to_numpy() > 0
    mask = np.asarray(mask, dtype=bool) & valid
    result = {'samples': int(mask.sum())}
    for name, (column, threshold_key) in ERRORS.items():
        values = data[column].to_numpy()
        usable = mask & np.isfinite(values)
        m = error_metrics(values[usable], config[threshold_key])
        if m is not None:
            # Never count through an invalid sample, missing measurement or missing interval.
            spans = usable[:-1] & valid[1:] & np.isfinite(values[1:]) & connected
            duration = float(dt[spans].sum())
            exceed = spans & (np.abs(values[:-1]) > config[threshold_key])
            exceed_time = float(dt[exceed].sum())
            m.update(valid_samples=int(usable.sum()), valid_duration_sec=duration,
                     exceed_duration_sec=exceed_time,
                     exceed_time_percent=100 * exceed_time / duration if duration else None)
        result[name] = m
    return result


def acceleration(time, speed, valid, config):
    raw, filtered = np.full(len(time), np.nan), np.full(len(time), np.nan)
    previous = None
    for i in range(1, len(time)):
        dt = time[i] - time[i - 1]
        if (not valid[i] or not valid[i - 1] or not np.isfinite(speed[i:i + 1]).all() or
                not np.isfinite(speed[i - 1]) or dt > config['max_sample_gap'] + 1e-9):
            previous = None
            continue
        raw[i] = (speed[i] - speed[i - 1]) / dt
        alpha = dt / (config['acceleration_filter_tau'] + dt)
        previous = raw[i] if previous is None else previous + alpha * (raw[i] - previous)
        filtered[i] = previous
    return raw, filtered


def turn_intervals(data, planned, config):
    t, s = data['time'].to_numpy(), data['progress'].to_numpy()
    eligible = (data['geometry_valid'].to_numpy() > 0) & (data['state'].to_numpy() == 2)
    curved = eligible & (np.abs(data['curvature'].to_numpy()) >=
                         config['curve_curvature_threshold'])
    stops = np.array([])
    if 'stop_required' in planned:
        length = np.r_[0, np.cumsum(np.hypot(np.diff(planned['x']), np.diff(planned['y'])))]
        stops = length[planned['stop_required'].fillna(0).to_numpy() > 0]
    groups = []
    for begin, end in intervals(t, curved, config['max_sample_gap']):
        last = end if curved[end] else end - 1
        if groups:
            previous = groups[-1][1]
            between = slice(previous, begin + 1)
            stop_between = np.any((stops >= s[previous] - 1e-9) &
                                  (stops <= s[begin] + 1e-9))
            if (t[begin] - t[previous] <= config['turn_merge_time'] + 1e-9 and
                    s[begin] - s[previous] <= config['turn_merge_distance'] + 1e-9 and
                    eligible[between].all() and not stop_between and
                    np.all(np.diff(t[between]) <= config['max_sample_gap'] + 1e-9)):
                groups[-1] = (groups[-1][0], last)
                continue
        groups.append((begin, last))
    return groups


def event_at(data, i, kind, **extra):
    keys = ('time', 'ros_time', 'progress', 'x', 'y', 'v_ref', 'v_cmd', 'v_actual',
            'w_cmd', 'w_actual', 'curvature', 'lateral_error', 'heading_error', 'position_error')
    event = {'type': kind, 'state': STATES.get(int(data.iloc[i]['state']), 'UNKNOWN')}
    for key in keys:
        v = float(data.iloc[i][key]) if key in data else np.nan
        event[key] = v if np.isfinite(v) else None
    event.update(extra)
    return event


def derive(planned, actual, config):
    data = actual.copy().reset_index(drop=True)
    for name in ('v_ref', 'v_cmd', 'w_cmd', 'w_actual', 'w_ff', 'nearest_x', 'nearest_y'):
        if name not in data:
            data[name] = np.nan
    # Derive exclusively from the projection captured in the same controller message.
    data['position_error'] = np.hypot(data['x'] - data['nearest_x'],
                                      data['y'] - data['nearest_y'])
    valid = data['geometry_valid'].to_numpy() > 0
    t = data['time'].to_numpy()
    raw, filtered = acceleration(t, data['v_actual'].to_numpy(), valid, config)
    data['acceleration_raw'], data['acceleration_filtered'] = raw, filtered
    data['control_state'] = [STATES.get(int(x), 'UNKNOWN') for x in data['state']]
    data['path_type'] = np.where(np.abs(data['curvature']) >=
                                 config['curve_curvature_threshold'], 'curve', 'straight')
    data.loc[~valid, 'path_type'] = 'invalid'
    tracking = valid & (data['state'].to_numpy() == 2)
    data['motion_action'] = np.where(tracking, 'transient', 'not_tracking')
    events = []
    threshold = config['acceleration_threshold']
    masks = {'accelerating': filtered > threshold, 'decelerating': filtered < -threshold,
             'steady': np.abs(filtered) <= threshold}
    for action, mask in masks.items():
        for begin, end in intervals(t, mask & tracking, config['max_sample_gap']):
            if t[end] - t[begin] < config['motion_min_duration'] - 1e-9:
                continue
            last = end if (mask & tracking)[end] else end - 1
            data.loc[begin:last, 'motion_action'] = action
            events.append(event_at(data, begin, action, end_time=float(t[end]),
                                   duration_sec=float(t[end] - t[begin])))
    turns = []
    data['turn_id'] = 0
    for number, (begin, end) in enumerate(turn_intervals(data, planned, config), 1):
        data.loc[begin:end, 'turn_id'] = number
        minimum = int(data.iloc[begin:end + 1]['v_actual'].idxmin())
        turn = {'id': number, 'start_time': float(t[begin]), 'end_time': float(t[end]),
                'start_progress': float(data.iloc[begin]['progress']),
                'end_progress': float(data.iloc[end]['progress']),
                'minimum_speed': event_at(data, minimum, 'turn_minimum_speed')}
        turns.append(turn)
        events.append(dict(turn['minimum_speed'], turn_id=number))
    for name, (column, _) in ERRORS.items():
        usable = valid & np.isfinite(data[column].to_numpy())
        if usable.any():
            i = int(data.loc[usable, column].abs().idxmax())
            events.append(event_at(data, i, 'maximum_' + column))
    seen_tracking = False
    previous_state = None
    for i, state in enumerate(data['state'].astype(int)):
        if state == 1 and previous_state != 1:
            events.append(event_at(data, i, 'realignment' if seen_tracking else
                                   'initial_alignment'))
        if state == 4 and previous_state != 4:
            events.append(event_at(data, i, 'goal_reached'))
        if state == 2:
            seen_tracking = True
        previous_state = state
    # Stopped intervals include BRAKING/ALIGNING, not just TRACKING.
    stop_s = np.array([])
    if 'stop_required' in planned:
        ss = np.r_[0, np.cumsum(np.hypot(np.diff(planned['x']), np.diff(planned['y'])))]
        stop_s = ss[planned['stop_required'].fillna(0).to_numpy() > 0]
    stopped = valid & (np.abs(data['v_actual'].to_numpy()) <= config['stopped_speed_threshold'])
    for begin, end in intervals(t, stopped, config['max_sample_gap']):
        duration = float(t[end] - t[begin])
        if duration < config['stop_min_duration'] - 1e-9:
            continue
        s = float(data.iloc[begin]['progress'])
        initial = not (data.iloc[:begin]['state'] == 2).any()
        planned_stop = bool(np.any(np.abs(stop_s - s) <= config['stop_exclusion_distance']))
        kind = 'initial_stop' if initial else 'planned_stop' if planned_stop else 'unplanned_stop'
        events.append(event_at(data, begin, kind, end_time=float(t[end]), duration_sec=duration))
    return data, turns, sorted(events, key=lambda e: (e['time'], e['type']))


def summarize(planned, actual, config):
    required = {'time', 'x', 'y', 'yaw', 'progress', 'remaining', 'lateral_error', 'heading_error',
                'curvature', 'v_actual', 'state', 'geometry_valid'}
    if not required.issubset(actual.columns):
        raise ValueError('Recording has no synchronized controller errors; '
                         'record with new recorder')
    if len(planned) < 2 or not len(actual):
        raise ValueError('Need a path with at least two points and recorded controller samples')
    if not np.isfinite(actual[list(required)].to_numpy(dtype=float)).all() or \
            np.any(np.diff(actual['time']) <= 0):
        raise ValueError('Nonfinite data or non-increasing simulation timestamps')
    if not np.isfinite(planned[['x', 'y']].to_numpy(dtype=float)).all():
        raise ValueError('Nonfinite reference path')
    valid = actual[actual['geometry_valid'] > 0]
    if not len(valid):
        raise ValueError('No valid path-projection samples in this epoch')
    if np.any(np.diff(valid['progress']) < -1e-9):
        raise ValueError('Progress moved backwards; do not mix different path epochs')
    data, turns, events = derive(planned, actual, config)
    usable = data['geometry_valid'].to_numpy() > 0
    whole = metric_group(data, usable, config)
    tracking = usable & (data['state'].to_numpy() == 2)
    unexpected = (tracking & (np.abs(data['v_actual']) <= config['stopped_speed_threshold']) &
                  (data['progress'] > config['stop_exclusion_distance']) &
                  (data['remaining'] > config['stop_exclusion_distance']))
    stops = [data.iloc[e]['time'] - data.iloc[b]['time'] for b, e in
             intervals(data['time'].to_numpy(), unexpected, config['max_sample_gap'])
             if data.iloc[e]['time'] - data.iloc[b]['time'] >= config['stop_min_duration'] - 1e-9]
    length = float(np.hypot(np.diff(planned['x']), np.diff(planned['y'])).sum())
    if not np.isfinite(length) or length <= 0:
        raise ValueError('Reference path must have finite positive length')
    curve = data['path_type'].to_numpy() == 'curve'
    summary = {'schema_version': 2, 'samples': len(data), 'valid_error_samples': int(usable.sum()),
               'planned_length_m': length,
               'recorded_length_m': float(np.hypot(np.diff(data['x']), np.diff(data['y'])).sum()),
               'progress_coverage_percent': float(np.clip(valid['progress'].max() / length * 100,
                                                          0, 100)),
               'goal_reached': bool((data['state'] == 4).any()),
               'duration_sec': float(data['time'].iloc[-1] - data['time'].iloc[0]),
               **{k: whole[k] for k in ERRORS},
               'curve_lateral_m': metric_group(data, usable & curve, config)['lateral_m'],
               'straight_lateral_m': metric_group(data, usable & ~curve, config)['lateral_m'],
               'tracking': metric_group(data, tracking, config),
               'by_state': {s: metric_group(data, usable & (data['control_state'] == s), config)
                            for s in data['control_state'].unique()},
               'by_geometry': {s: metric_group(data, usable & (data['path_type'] == s), config)
                               for s in ('straight', 'curve')},
               'by_motion': {s: metric_group(data, tracking & (data['motion_action'] == s), config)
                             for s in ('accelerating', 'steady', 'decelerating', 'transient')},
               'turns': turns, 'events': events,
               'unexpected_stop_count': len(stops),
               'unexpected_stop_duration_sec': float(sum(stops)),
               'availability': {'position_error': 'recorded_projection' if
                                np.isfinite(data['position_error']).any() else
                                'unavailable: projection not recorded; no offline reconstruction'},
               'thresholds': config,
               'metric_basis': 'sample MAE/RMSE/P95; interval-held time metrics',
               'pose_source': 'TF localization estimate; not Gazebo ground truth'}
    for turn in turns:
        turn['metrics'] = metric_group(data, usable & (data['turn_id'] == turn['id']), config)
    return data.loc[usable].copy(), summary


def save_summary(path, summary):
    Path(path).write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
