#!/usr/bin/env python3

import math
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ============================================================
# 点到线段距离
# ============================================================

def point_to_segment_distance(px, py, ax, ay, bx, by):

    abx = bx - ax
    aby = by - ay

    apx = px - ax
    apy = py - ay

    ab2 = abx * abx + aby * aby

    # A、B 重合
    if ab2 < 1e-12:
        dx = px - ax
        dy = py - ay

        return math.sqrt(
            dx * dx + dy * dy
        )

    # 投影比例
    t = (
        apx * abx +
        apy * aby
    ) / ab2

    t = max(
        0.0,
        min(1.0, t)
    )

    closest_x = ax + t * abx
    closest_y = ay + t * aby

    dx = px - closest_x
    dy = py - closest_y

    return math.sqrt(
        dx * dx + dy * dy
    )


# ============================================================
# 计算实际轨迹每个点到规划轨迹的最近距离
# ============================================================

def calculate_cross_track_error(
    actual_x,
    actual_y,
    planned_xy
):

    errors = []

    for px, py in zip(
        actual_x,
        actual_y
    ):

        min_dist = float('inf')

        # 遍历所有规划路径线段
        for i in range(
            len(planned_xy) - 1
        ):

            ax, ay = planned_xy[i]
            bx, by = planned_xy[i + 1]

            dist = point_to_segment_distance(
                px,
                py,
                ax,
                ay,
                bx,
                by
            )

            if dist < min_dist:
                min_dist = dist

        errors.append(min_dist)

    return np.array(errors)


# ============================================================
# 主程序
# ============================================================

def main():

    data_dir = Path('data/path_record')  # 相对于启动程序时的工作目录

    planned_file = (
        data_dir /
        'planned_path.csv'
    )

    actual_file = (
        data_dir /
        'actual_path.csv'
    )

    result_file = (
        data_dir /
        'tracking_error.csv'
    )

    print('======================================')
    print('DWA Path Tracking Analysis')
    print('======================================')

    # --------------------------------------------------------
    # 读取数据
    # --------------------------------------------------------

    planned = pd.read_csv(
        planned_file
    )

    actual = pd.read_csv(
        actual_file
    )

    if len(planned) < 2:
        raise RuntimeError(
            'Planned path has fewer than 2 points.'
        )

    if len(actual) == 0:
        raise RuntimeError(
            'Actual path is empty.'
        )

    planned_xy = list(
        zip(
            planned['x'].values,
            planned['y'].values
        )
    )

    # --------------------------------------------------------
    # 计算误差
    # --------------------------------------------------------

    errors = calculate_cross_track_error(
        actual['x'].values,
        actual['y'].values,
        planned_xy
    )

    actual['tracking_error'] = errors

    # 保存结果
    actual.to_csv(
        result_file,
        index=False
    )

    # --------------------------------------------------------
    # 统计指标
    # --------------------------------------------------------

    mae = np.mean(
        np.abs(errors)
    )

    rmse = np.sqrt(
        np.mean(errors ** 2)
    )

    max_error = np.max(
        errors
    )

    median_error = np.median(
        errors
    )

    p95_error = np.percentile(
        errors,
        95
    )

    # 误差超过阈值的比例
    threshold = 0.10

    exceed_ratio = (
        np.sum(errors > threshold)
        /
        len(errors)
        *
        100.0
    )

    print()
    print('--------------- Tracking Error ---------------')

    print(
        f'MAE        : {mae:.4f} m'
    )

    print(
        f'RMSE       : {rmse:.4f} m'
    )

    print(
        f'Max Error  : {max_error:.4f} m'
    )

    print(
        f'Median     : {median_error:.4f} m'
    )

    print(
        f'95% Error  : {p95_error:.4f} m'
    )

    print(
        f'> {threshold:.2f} m : '
        f'{exceed_ratio:.2f}%'
    )

    # --------------------------------------------------------
    # 计算实际轨迹长度
    # --------------------------------------------------------

    dx = np.diff(
        actual['x'].values
    )

    dy = np.diff(
        actual['y'].values
    )

    actual_length = np.sum(
        np.sqrt(
            dx * dx +
            dy * dy
        )
    )

    # --------------------------------------------------------
    # 计算规划轨迹长度
    # --------------------------------------------------------

    pxy = np.array(
        planned_xy
    )

    pdx = np.diff(
        pxy[:, 0]
    )

    pdy = np.diff(
        pxy[:, 1]
    )

    planned_length = np.sum(
        np.sqrt(
            pdx * pdx +
            pdy * pdy
        )
    )

    print()
    print('--------------- Path Length ---------------')

    print(
        f'Planned length : '
        f'{planned_length:.4f} m'
    )

    print(
        f'Actual length  : '
        f'{actual_length:.4f} m'
    )

    print(
        f'Length error   : '
        f'{actual_length - planned_length:.4f} m'
    )

    # --------------------------------------------------------
    # 绘制轨迹对比
    # --------------------------------------------------------

    plt.figure(
        figsize=(10, 8)
    )

    plt.plot(
        planned['x'].to_numpy(),
        planned['y'].to_numpy(),
        label='A* / planned path'
    )

    plt.plot(
        actual['x'].to_numpy(),
        actual['y'].to_numpy(),
        label='Actual trajectory'
    )

    plt.scatter(
        planned['x'].iloc[0],
        planned['y'].iloc[0],
        marker='o',
        s=60,
        label='Start'
    )

    plt.scatter(
        planned['x'].iloc[-1],
        planned['y'].iloc[-1],
        marker='x',
        s=80,
        label='Goal'
    )

    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')

    plt.title(
        'Planned Path vs Actual Trajectory'
    )

    plt.axis('equal')

    plt.grid(True)

    plt.legend()

    plt.tight_layout()

    trajectory_png = (
        data_dir /
        'trajectory_comparison.png'
    )

    plt.savefig(
        trajectory_png,
        dpi=200
    )

    plt.show()

    # --------------------------------------------------------
    # 绘制误差曲线
    # --------------------------------------------------------

    plt.figure(
        figsize=(10, 5)
    )

    plt.plot(
        actual['time'].to_numpy(),
        errors
    )

    plt.axhline(
        threshold,
        linestyle='--',
        label=f'{threshold:.2f} m threshold'
    )

    plt.xlabel(
        'Time [s]'
    )

    plt.ylabel(
        'Tracking Error [m]'
    )

    plt.title(
        'Path Tracking Error'
    )

    plt.grid(True)

    plt.legend()

    plt.tight_layout()

    error_png = (
        data_dir /
        'tracking_error.png'
    )

    plt.savefig(
        error_png,
        dpi=200
    )

    plt.show()

    print()
    print('Results saved to:')

    print(
        result_file
    )

    print(
        trajectory_png
    )

    print(
        error_png
    )


if __name__ == '__main__':
    main()
