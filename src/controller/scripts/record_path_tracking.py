#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Record synchronized controller snapshots, separated by process and planning epoch."""

import csv
from datetime import datetime
import json
import math
from pathlib import Path
import signal

from common_msg.msg import PathProfile
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64MultiArray, String

from tracking_analysis import load_config
from tracking_metadata import MetadataCapture


class PathRecorder(Node):
    """Record the exact TF pose and path progress used by the controller."""

    def __init__(self):
        super().__init__('path_recorder')
        defaults = load_config()['path_recorder']['ros__parameters']
        for key, value in defaults.items():
            if not self.has_parameter(key):
                self.declare_parameter(key, value)

        def param(name):
            return self.get_parameter(name).value
        self.frame = param('global_frame')
        self.flush_period = float(param('flush_period'))
        if not self.frame or not math.isfinite(self.flush_period) or self.flush_period <= 0.0:
            raise ValueError('global_frame and positive flush_period are required')
        root = Path(param('output_dir')).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        self.directory = root / datetime.now().strftime('run_%Y%m%d_%H%M%S_%f')
        self.directory.mkdir()
        self.epoch = 0
        self.path = None
        self.pending_profile = None
        self.finished = False
        self.seen_stamps = set()
        self.epoch_dir = None
        self.start = self.last_time = None
        self.fp = self.writer = None
        self.status = 'WAIT_PATH'
        self.files = []
        self.events_fp = self.open_csv(self.directory / 'events.csv')
        self.events = csv.writer(self.events_fp)
        self.events.writerow(['ros_time', 'epoch', 'event'])
        self.create_subscription(PathMsg, param('plan_topic'), self.on_path, 10)
        self.create_subscription(PathProfile, param('profile_topic'), self.on_profile, 10)
        self.create_subscription(Float64MultiArray, param('tracking_topic'), self.on_tracking, 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, param('status_topic'), self.on_status, qos)
        self.metadata = None
        if param('capture_metadata'):
            period = float(param('metadata_poll_period'))
            if not math.isfinite(period) or period <= 0:
                raise ValueError('metadata_poll_period must be positive')
            self.metadata = MetadataCapture(self, param('metadata_nodes'),
                                            float(param('metadata_timeout')))
            self.create_subscription(OccupancyGrid, param('map_topic'), self.metadata.on_map, qos)
            self.create_subscription(PoseStamped, param('goal_topic'), self.metadata.on_goal, 10)
            self.create_timer(period, self.metadata.poll,
                              clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(self.flush_period, self.flush)
        (self.directory / 'manifest.json').write_text(json.dumps({
            'frame': self.frame, 'pose_source': 'controller TF localization estimate',
            'epoch_policy': 'each received Path starts a separate comparison',
            'parameters': {name: param(name) for name in defaults},
        }, indent=2) + '\n')
        self.get_logger().info(f'Tracking output: {self.directory}')

    def open_csv(self, path):
        fp = path.open('w', newline='')
        self.files.append(fp)
        return fp

    def event(self, name):
        self.events.writerow([self.get_clock().now().nanoseconds / 1e9, self.epoch, name])

    def on_path(self, message):
        self.flush()
        if self.fp is not None:
            self.fp.close()
        self.fp = self.writer = None
        self.start = self.last_time = None
        self.path = None
        self.finished = False
        self.epoch += 1
        if not message.poses:
            self.event('PATH_CLEARED')
            return
        if message.header.frame_id != self.frame or any(
                p.header.frame_id != self.frame for p in message.poses):
            self.event('PATH_FRAME_REJECTED')
            return
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        if stamp in self.seen_stamps:
            self.event('PATH_STAMP_REUSED_RECORDING_REJECTED')
            return
        self.seen_stamps.add(stamp)
        self.path = message
        self.epoch_dir = self.directory / f'path_{self.epoch:04d}'
        self.epoch_dir.mkdir()
        self.save_plan(self.pending_profile if self.pending_profile is not None and
                       self.pending_profile.path == message else None)
        self.fp = self.open_csv(self.epoch_dir / 'actual_path.csv')
        self.writer = csv.DictWriter(self.fp, fieldnames=[
            'time', 'ros_time', 'x', 'y', 'yaw', 'progress', 'lateral_error', 'heading_error',
            'v_ref', 'v_cmd', 'w_cmd', 'v_actual', 'w_actual', 'curvature', 'w_ff',
            'remaining', 'state', 'geometry_valid', 'nearest_x', 'nearest_y', 'position_error'])
        self.writer.writeheader()
        self.event('PATH_RECEIVED')
        if self.metadata is not None:
            self.metadata.start(self.epoch_dir, message)
        self.get_logger().info(f'New path epoch: {self.epoch_dir}')

    def save_plan(self, profile=None):
        with (self.epoch_dir / 'planned_path.csv').open('w', newline='') as fp:
            writer = csv.writer(fp)
            writer.writerow(['index', 'x', 'y', 'yaw', 'curvature', 'stop_required',
                             'segment_type', 'speed_limit', 'curvature_bound',
                             'curvature_rate_bound'])
            for i, p in enumerate(self.path.poses):
                q = p.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))
                writer.writerow([i, p.pose.position.x, p.pose.position.y,
                                 profile.tangent_yaw[i] if profile else yaw,
                                 profile.curvature[i] if profile else '',
                                 int(profile.stop_required[i]) if profile else '',
                                 profile.segment_type[i] if profile and
                                 profile.segment_type else '',
                                 profile.speed_limit[i] if profile and profile.speed_limit else '',
                                 profile.curvature_bound[i] if profile and
                                 profile.curvature_bound else '',
                                 profile.curvature_rate_bound[i] if profile and
                                 profile.curvature_rate_bound else ''])
        (self.epoch_dir / 'path.json').write_text(json.dumps({
            'stamp_sec': self.path.header.stamp.sec,
            'stamp_nanosec': self.path.header.stamp.nanosec,
            'frame': self.frame, 'profile_received': profile is not None,
            'curvature_continuous': bool(profile.curvature_continuous) if profile else False,
        }, indent=2) + '\n')
        if profile and profile.processing_report:
            (self.epoch_dir / 'planning_report.json').write_text(profile.processing_report + '\n')

    def on_profile(self, message):
        n = len(message.path.poses)
        if not all(len(v) == n for v in (
                message.tangent_yaw, message.curvature, message.stop_required)):
            return
        extended = (message.segment_type, message.speed_limit,
                    message.curvature_bound, message.curvature_rate_bound)
        if any(len(v) for v in extended) and not all(len(v) == n for v in extended):
            return
        self.pending_profile = message
        if self.path is None or message.path != self.path:
            return
        n = len(self.path.poses)
        if all(len(v) == n for v in (
                message.tangent_yaw, message.curvature, message.stop_required)):
            self.save_plan(message)

    def on_status(self, message):
        if message.data != self.status:
            self.status = message.data
            self.event(self.status)

    def on_tracking(self, message):
        if self.path is None or self.writer is None or self.finished or not message.layout.dim:
            return
        values = dict(zip(message.layout.dim[0].label.split(','), message.data))
        required = {'ey', 'etheta', 'progress', 'remaining', 'v_ref', 'v_cmd', 'w_cmd',
                    'curvature', 'w_ff', 'v_actual', 'w_actual', 'pose_x', 'pose_y',
                    'pose_yaw', 'ros_time', 'path_sec', 'path_nanosec', 'state', 'geometry_valid'}
        if not required.issubset(values) or not all(math.isfinite(values[k]) for k in required):
            return
        stamp = self.path.header.stamp
        if (values['path_sec'], values['path_nanosec']) != (stamp.sec, stamp.nanosec):
            return
        now = values['ros_time']
        if self.last_time is not None and now <= self.last_time:
            self.event('CLOCK_RESET_OR_DUPLICATE_SAMPLE')
            self.path = None
            return
        if self.start is None:
            self.start = now
        self.last_time = now
        row = {k: values[k] for k in (
            'progress', 'remaining', 'v_ref', 'v_cmd', 'w_cmd',
            'v_actual', 'w_actual', 'curvature', 'w_ff', 'state', 'geometry_valid')}
        row.update(time=now - self.start, ros_time=now, x=values['pose_x'], y=values['pose_y'],
                   yaw=values['pose_yaw'], lateral_error=values['ey'],
                   heading_error=values['etheta'])
        projection = [values.get(key) for key in ('nearest_x', 'nearest_y')]
        if all(v is not None and math.isfinite(v) for v in projection):
            row.update(nearest_x=projection[0], nearest_y=projection[1],
                       position_error=math.hypot(values['pose_x'] - projection[0],
                                                 values['pose_y'] - projection[1]))
        self.writer.writerow(row)
        self.finished = int(values['state']) in (4, 5)

    def flush(self):
        for fp in self.files:
            if not fp.closed:
                fp.flush()

    def destroy_node(self):
        if self.metadata is not None:
            self.metadata.close()
        self.flush()
        for fp in self.files:
            if not fp.closed:
                fp.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # launch and a terminal can both deliver SIGINT; finish flushing once.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
