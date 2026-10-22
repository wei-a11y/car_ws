# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Chinese progress plots and a full-route vector speed / position-error overlay."""

from pathlib import Path
import re

import matplotlib as mpl
import numpy as np

from tracking_analysis import intervals, save_summary


TITLES = {
    'longitudinal_speed': '纵向速度', 'angular_velocity': '角速度',
    'curvature': '参考曲率', 'acceleration': '纵向加速度估计',
    'lateral_error': '横向误差', 'heading_error': '航向误差',
    'position_error': '位置距离误差'}
AXIS_LABELS = {'time': '仿真运行时间 [s]', 'progress': '路径进度 [m]'}
EVENT_LABELS = {
    'maximum_lateral_error': '最大横向误差', 'maximum_heading_error': '最大航向误差',
    'maximum_position_error': '最大位置距离误差', 'realignment': '重新对准',
    'unplanned_stop': '非计划停车', 'planned_stop': '必要停车', 'goal_reached': '到达终点'}
SPECS = {
    'longitudinal_speed': ('纵向速度 [m/s]',
                           [('v_ref', '参考速度', 'C0', '--'),
                            ('v_cmd', '指令速度', 'C1', '-.'),
                            ('v_actual', '实际速度（里程计）', 'C2', '-')]),
    'angular_velocity': ('角速度 [rad/s]',
                         [('w_ff', '前馈角速度', 'C0', '--'),
                          ('w_cmd', '指令角速度', 'C1', '-.'),
                          ('w_actual', '实际角速度（里程计）', 'C2', '-')]),
    'curvature': ('前馈参考曲率 [1/m]', [('curvature', '参考曲率', 'C0', '-')]),
    'acceleration': ('纵向加速度估计 [m/s²]',
                     [('acceleration_raw', '原始差分估计', '0.75', '-'),
                      ('acceleration_filtered', '滤波估计', 'C0', '-')]),
    'lateral_error': ('带符号横向误差 [m]', [('lateral_error', '横向误差', 'C0', '-')]),
    'heading_error': ('航向误差 [度]', [('heading_error', '航向误差', 'C0', '-')]),
    'position_error': ('到选定投影点的距离 [m]',
                       [('position_error', '位置距离误差', 'C0', '-')])}

ACTUAL = {'longitudinal_speed': 'v_actual', 'angular_velocity': 'w_actual',
          'curvature': 'curvature', 'acceleration': 'acceleration_filtered',
          'lateral_error': 'lateral_error', 'heading_error': 'heading_error',
          'position_error': 'position_error'}


def style(config):
    return mpl.rc_context({'font.size': config['font_size'],
                           'font.family': 'sans-serif',
                           'font.sans-serif': config['font_families'],
                           'axes.unicode_minus': False,
                           'lines.linewidth': config['line_width'],
                           'svg.fonttype': 'path', 'pdf.fonttype': 42,
                           'path.simplify': False})


def figure(config):
    import matplotlib.pyplot as plt
    return plt.subplots(figsize=tuple(config['figure_size']))


def save_figure(fig, directory, stem, config, show):
    import matplotlib.pyplot as plt
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    files = []
    for extension in config['formats']:
        path = directory / (stem + '.' + extension)
        fig.savefig(path, dpi=config['dpi'])
        files.append(str(path))
    # Replace this generated plot's obsolete formats when the configured formats change.
    for extension in {'svg', 'pdf', 'png'} - set(config['formats']):
        (directory / (stem + '.' + extension)).unlink(missing_ok=True)
    if show:
        plt.show()
    plt.close(fig)
    return files


def gapped(x, y, time, maximum_gap):
    # Insert separators; never discard the first valid sample after a missing interval.
    split = np.flatnonzero(np.diff(time) > maximum_gap + 1e-9) + 1
    return np.insert(x, split, np.nan), np.insert(y, split, np.nan)


def threshold(quantity, config):
    keys = {'lateral_error': 'lateral_error_threshold',
            'heading_error': 'heading_error_threshold',
            'position_error': 'position_error_threshold'}
    if quantity not in keys:
        return None
    value = config[keys[quantity]]
    return float(np.degrees(value)) if quantity == 'heading_error' else value


def yvalues(data, column, mask):
    y = data[column].to_numpy(dtype=float).copy()
    if column == 'heading_error':
        y = np.degrees(y)
    y[~mask] = np.nan
    return y


def decorate(ax, data, summary, axis, config, selection):
    t = data['time'].to_numpy()
    x = data[axis].to_numpy()
    maximum_gap = config['max_sample_gap']
    active = np.asarray(selection, dtype=bool)
    if not active.any():
        return
    low, high = x[active].min(), x[active].max()
    used = set()
    for mask, color, label in (
            ((data['state'].to_numpy() == 1) & active, '0.7', '起步或原地对准'),
            ((data['state'].to_numpy() == 3) & active, 'lightcoral', '制动'),
            ((data['turn_id'].to_numpy() > 0) & active, 'orange', '转弯区间')):
        for begin, end in intervals(t, mask, maximum_gap):
            end_x = x[end] if mask[end] else x[max(begin, end - 1)]
            ax.axvspan(x[begin], end_x, color=color, alpha=.12,
                       label=label if label not in used else None)
            used.add(label)
    for action, color, marker in [('accelerating', 'green', '^'), ('decelerating', 'red', 'v')]:
        points = [e[axis] for e in summary['events'] if e['type'] == action and
                  low <= e[axis] <= high]
        if points:
            ax.scatter(points, [.025] * len(points), transform=ax.get_xaxis_transform(),
                       color=color, marker=marker, s=14,
                       label={'accelerating': '加速区间起点', 'decelerating': '减速区间起点'}[action])
    markers = {}
    for event in summary['events']:
        kind = event['type']
        selected_kinds = ('realignment', 'unplanned_stop', 'planned_stop', 'goal_reached')
        if kind.startswith('maximum_') or kind in selected_kinds:
            point = event[axis]
            if low <= point <= high:
                # The same stop often has a realignment event at the exact same sample.
                # Merge labels, not data, so their text cannot overprint one another.
                markers.setdefault(round(point, 6), []).append(
                    EVENT_LABELS.get(kind, kind))
    groups = []
    spacing = (high - low) * config['event_label_spacing_fraction']
    for point, labels in sorted(markers.items()):
        ax.axvline(point, color='0.45', alpha=.5, linestyle=':', linewidth=.7)
        if groups and point - groups[-1][0] <= spacing:
            groups[-1][1].extend(labels)
        else:
            groups.append((point, list(labels)))
    for point, labels in groups:
        ax.text(point, .98, ' / '.join(dict.fromkeys(labels)),
                transform=ax.get_xaxis_transform(), rotation=90,
                va='top', ha='right', fontsize=max(7, config['font_size'] - 4))


def quantity_figure(data, summary, quantity, axis, config, selection=None,
                    tracking_only=False, title=None):
    selection = np.ones(len(data), dtype=bool) if selection is None else np.asarray(selection)
    mask = selection & (data['geometry_valid'].to_numpy() > 0)
    if tracking_only:
        mask &= data['state'].to_numpy() == 2
    series = [(column, label, color, linestyle) for column, label, color, linestyle in
              SPECS[quantity][1] if np.isfinite(yvalues(data, column, mask)).any()]
    if not series:
        return None
    fig, ax = figure(config)
    x, time = data[axis].to_numpy(), data['time'].to_numpy()
    for column, label, color, linestyle in series:
        xx, yy = gapped(x, yvalues(data, column, mask), time, config['max_sample_gap'])
        ax.plot(xx, yy, label=label, color=color, linestyle=linestyle)
    limit = threshold(quantity, config)
    if limit is not None:
        ax.axhline(limit, linestyle='--', color='red', label='允许误差阈值')
        if quantity != 'position_error':
            ax.axhline(-limit, linestyle='--', color='red')
    ax.axhline(0, color='black', linewidth=.5)
    decorate(ax, data, summary, axis, config, mask)
    ax.set(xlabel=AXIS_LABELS[axis],
           ylabel=SPECS[quantity][0],
           title=title or TITLES[quantity] + ('（正常跟踪阶段）' if tracking_only else ''))
    active_x = x[mask]
    if active_x.max() > active_x.min():
        ax.set_xlim(active_x.min(), active_x.max())
    ax.margins(y=config['error_axis_margin'])
    ax.grid(True, alpha=.3)
    ax.legend(loc='best', fontsize=max(7, config['font_size'] - 2))
    return fig


def trajectory_figure(planned, data, config, name):
    fig, ax = figure(config)
    ax.plot(planned['x'].to_numpy(), planned['y'].to_numpy(), '--', label='规划参考路径')
    valid = data['geometry_valid'].to_numpy() > 0
    x = data['x'].to_numpy().copy()
    y = data['y'].to_numpy().copy()
    x[~valid], y[~valid] = np.nan, np.nan
    x, y = gapped(x, y, data['time'].to_numpy(), config['max_sample_gap'])
    ax.plot(x, y, color='C1', label='实际轨迹（TF 定位）')
    dots = ax.scatter(data.loc[valid, 'x'], data.loc[valid, 'y'],
                      c=data.loc[valid, 'lateral_error'].abs(), cmap='viridis',
                      s=config['point_size'], vmin=0, vmax=config['lateral_error_threshold'])
    colorbar = fig.colorbar(dots, ax=ax, label='横向误差绝对值 [m]')
    if colorbar.solids is not None:
        colorbar.solids.set_rasterized(False)
    if 'stop_required' in planned:
        stops = planned[planned['stop_required'].fillna(0) > 0]
        ax.scatter(stops['x'], stops['y'], marker='x', color='red', label='必要停车点或终点')
    ax.scatter(planned['x'].iloc[0], planned['y'].iloc[0], marker='o', label='起点')
    ax.scatter(planned['x'].iloc[-1], planned['y'].iloc[-1], marker='*', label='终点')
    ax.set(xlabel='地图 X 坐标 [m]', ylabel='地图 Y 坐标 [m]',
           title='路径记录 ' + name + '：规划与实际轨迹（TF 定位估计）')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=.3)
    ax.legend(loc='best', fontsize=max(7, config['font_size'] - 2))
    return fig


def speed_position_figure(planned, data, summary, config, name):
    """Overlay speeds and recorded projection distance without changing their units."""
    valid = data['geometry_valid'].to_numpy() > 0
    speed_series = [(column, label, color, line) for column, label, color, line in
                    SPECS['longitudinal_speed'][1]
                    if np.isfinite(yvalues(data, column, valid)).any()]
    if not speed_series or not np.isfinite(yvalues(data, 'position_error', valid)).any():
        return None
    fig, speed = figure(config)
    error = speed.twinx()
    x, time = data['progress'].to_numpy(), data['time'].to_numpy()
    for column, label, color, line in speed_series:
        xx, yy = gapped(x, yvalues(data, column, valid), time, config['max_sample_gap'])
        speed.plot(xx, yy, label=label, color=color, linestyle=line)
    xx, yy = gapped(x, yvalues(data, 'position_error', valid), time, config['max_sample_gap'])
    error.plot(xx, yy, color='C3', label='位置距离误差')
    error.axhline(config['position_error_threshold'], color='C3', linestyle=':',
                  label='位置距离误差允许阈值')
    decorate(speed, data, summary, 'progress', config, valid)
    # Cover the entire planned route, retaining recorded out-of-range progress if present.
    length = float(np.hypot(np.diff(planned['x']), np.diff(planned['y'])).sum())
    end = max(length, float(x[valid].max()))
    if end > 0:
        speed.set_xlim(0, end)
    speed.set(xlabel='路径进度（起点 → 终点）[m]', ylabel='纵向速度 [m/s]',
              title='路径记录 ' + name + '：全程纵向速度与位置距离误差')
    error.set_ylabel('位置距离误差 [m]', color='C3')
    error.tick_params(axis='y', labelcolor='C3')
    speed.margins(y=config['error_axis_margin'])
    error.margins(y=config['error_axis_margin'])
    error.set_ylim(bottom=0)
    speed.grid(True, alpha=.3)
    handles, labels = speed.get_legend_handles_labels()
    h_error, l_error = error.get_legend_handles_labels()
    count = len(speed_series)
    speed.legend(handles[:count] + h_error + handles[count:],
                 labels[:count] + l_error + labels[count:], loc='upper center',
                 bbox_to_anchor=(.5, -.18), ncol=config['combined_legend_columns'],
                 fontsize=max(7, config['font_size'] - 2))
    return fig


def prune_generated_plots(directory, keep, comparison=False):
    """Remove obsolete known plot names, leaving arbitrary files and raw recordings intact."""
    quantities = '|'.join(TITLES)
    if comparison:
        stem = rf'(trajectory_comparison|({quantities})(_tracking)?_comparison_(time|progress))'
    else:
        stem = (rf'(trajectory_overlay|speed_position_error_progress|'
                rf'({quantities})(_tracking)?_(time|progress)|'
                rf'turn_\d+_({quantities})_(time|progress))')
    known = re.compile(stem + r'\.(png|svg|pdf)')
    keep = {Path(path).name for path in keep}
    for path in Path(directory).iterdir():
        if path.is_file() and known.fullmatch(path.name) and path.name not in keep:
            path.unlink()


def export_plots(directory, planned, data, summary, config, show=False):
    output = Path(directory) / 'figures'
    manifest = {'files': [], 'skipped': [], 'formats': list(config['formats']),
                'combined_formats': list(config['combined_formats']),
                'label_language': 'zh-CN', 'dpi': config['dpi']}
    with style(config):
        if config['plot_trajectory']:
            fig = trajectory_figure(planned, data, config, Path(directory).name)
            manifest['files'] += save_figure(fig, output, 'trajectory_overlay', config, show)
        for quantity in config['plot_quantities']:
            scopes = [False]
            if config['plot_tracking_errors'] and quantity.endswith('error'):
                scopes.append(True)
            for tracking in scopes:
                for axis in config['plot_axes']:
                    stem = quantity + ('_tracking' if tracking else '') + '_' + axis
                    fig = quantity_figure(data, summary, quantity, axis, config,
                                          tracking_only=tracking,
                                          title='路径记录 ' + Path(directory).name + '：' +
                                          TITLES[quantity] +
                                          ('（正常跟踪阶段）' if tracking else ''))
                    if fig is None:
                        manifest['skipped'].append({'plot': stem, 'reason': 'no recorded values'})
                    else:
                        manifest['files'] += save_figure(fig, output, stem, config, show)
        if config['plot_speed_position']:
            stem = 'speed_position_error_progress'
            fig = speed_position_figure(planned, data, summary, config, Path(directory).name)
            if fig is None:
                manifest['skipped'].append({'plot': stem,
                                            'reason': 'speed or recorded projection unavailable'})
            else:
                combined = dict(config, formats=config['combined_formats'])
                manifest['files'] += save_figure(fig, output, stem, combined, show)
    output.mkdir(parents=True, exist_ok=True)
    prune_generated_plots(output, manifest['files'])
    save_summary(output / 'plots_manifest.json', manifest)
    return manifest


def comparison_figure(baseline, candidate, quantity, axis, config, tracking_only=False):
    column = ACTUAL[quantity]
    masks = [(d['geometry_valid'].to_numpy() > 0) for d in (baseline, candidate)]
    if tracking_only:
        masks = [m & (d['state'].to_numpy() == 2) for d, m in zip((baseline, candidate), masks)]
    if not all(np.isfinite(yvalues(d, column, m)).any()
               for d, m in zip((baseline, candidate), masks)):
        return None
    fig, ax = figure(config)
    for data, mask, label, color in zip(
            (baseline, candidate), masks, ('基线实验', '待比较实验'), ('C0', 'C1')):
        x, y = gapped(data[axis].to_numpy(), yvalues(data, column, mask),
                      data['time'].to_numpy(), config['max_sample_gap'])
        ax.plot(x, y, color=color, label=label)
    limit = threshold(quantity, config)
    if limit is not None:
        ax.axhline(limit, color='red', linestyle='--', label='允许误差阈值')
        if quantity != 'position_error':
            ax.axhline(-limit, color='red', linestyle='--')
    ax.set(xlabel=AXIS_LABELS[axis],
           ylabel=SPECS[quantity][0], title='实验对比：' + TITLES[quantity] +
           ('（正常跟踪阶段）' if tracking_only else ''))
    ax.grid(True, alpha=.3)
    ax.margins(y=config['error_axis_margin'])
    ax.legend(loc='best')
    return fig
