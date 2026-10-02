# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Read Phase 8 rosbag2 data and print per-goal metrics; never publish commands."""
import argparse
import json
import math

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def path_length(path):
    points = [p.pose.position for p in path.poses]
    return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(points, points[1:]))


def planar_pose(transform):
    t, q = transform.translation, transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return t.x, t.y, yaw


def compose(a, b):
    x, y, yaw = a
    return (x + math.cos(yaw) * b[0] - math.sin(yaw) * b[1],
            y + math.sin(yaw) * b[0] + math.cos(yaw) * b[1], yaw + b[2])


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else None


class Metrics:
    def __init__(self):
        self.current = None
        self.transforms = {}
        self.controller_state = ''

    def finish(self, result, stamp):
        run = self.current
        if run is None:
            return None
        run['result'] = result
        run['navigation_time_sec'] = (stamp - run.pop('start_ns')) * 1e-9
        lateral, heading = run.pop('lateral'), run.pop('heading')
        run['cross_track_rmse_m'] = rms(lateral)
        run['max_cross_track_error_m'] = max(map(abs, lateral), default=None)
        run['heading_rmse_rad'] = rms(heading)
        run['tracking_samples'] = len(lateral)
        endpoint = run.pop('endpoint', None)
        map_odom = self.transforms.get(('map', 'odom'))
        odom_base = self.transforms.get(('odom', 'base_footprint'))
        pose = self.transforms.get(('map', 'base_footprint'))
        if map_odom is not None and odom_base is not None:
            pose = compose(map_odom, odom_base)
        run['terminal_position_error_m'] = (
            math.hypot(pose[0] - endpoint[0], pose[1] - endpoint[1])
            if pose is not None and endpoint is not None else None)
        self.current = None
        return run

    def feed(self, topic, msg, stamp):
        completed = []
        if topic in ('/tf', '/tf_static'):
            for transform in msg.transforms:
                self.transforms[(transform.header.frame_id, transform.child_frame_id)] = (
                    planar_pose(transform.transform))
        if topic == '/goal_pose':
            if self.current:
                completed.append(self.finish('SUPERSEDED', stamp))
            self.controller_state = ''
            self.current = {'start_ns': stamp, 'goal_frame': msg.header.frame_id,
                            'goal_xy': [msg.pose.position.x, msg.pose.position.y],
                            'planning_latency_sec': None, 'raw_path_length_m': None,
                            'processed_path_length_m': None, 'lateral': [], 'heading': []}
        if self.current is None:
            return completed
        run = self.current
        if topic == '/plan/raw_path' and msg.poses:
            run['raw_path_length_m'] = path_length(msg)
        if topic == '/plan' and msg.poses:
            run['processed_path_length_m'] = path_length(msg)
            end = msg.poses[-1].pose.position
            run['endpoint'] = [end.x, end.y]
        if topic == '/plan/status':
            code = msg.data.split(':', 1)[0]
            if run['planning_latency_sec'] is None:
                run['planning_latency_sec'] = (stamp - run['start_ns']) * 1e-9
            if code != 'SUCCESS':
                completed.append(self.finish(code, stamp))
        if topic == '/controller/lqr/status':
            self.controller_state = msg.data.split(':', 1)[0]
            if self.controller_state in ('GOAL_REACHED', 'TRACKING_ERROR'):
                completed.append(self.finish(self.controller_state, stamp))
        if topic == '/controller/lqr/debug' and self.controller_state == 'TRACKING':
            if len(msg.data) >= 2 and all(math.isfinite(v) for v in msg.data[:2]):
                run['lateral'].append(msg.data[0])
                run['heading'].append(msg.data[1])
        return completed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='rosbag2 directory recorded with --use-sim-time')
    parser.add_argument('--storage-id', default='sqlite3')
    args = parser.parse_args()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=args.storage_id),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    selected = {'/goal_pose', '/plan/raw_path', '/plan', '/plan/status',
                '/controller/lqr/status', '/controller/lqr/debug', '/tf', '/tf_static'}
    metrics, runs, last_stamp = Metrics(), [], 0
    while reader.has_next():
        topic, data, last_stamp = reader.read_next()
        if topic in selected:
            runs.extend(metrics.feed(topic, deserialize_message(data, types[topic]), last_stamp))
    if metrics.current:
        runs.append(metrics.finish('INCOMPLETE', last_stamp))
    print(json.dumps({'pose_source': 'recorded TF estimate, not Gazebo ground truth',
                      'planning_time': 'goal-to-status receipt latency, not A* CPU time',
                      'time_source': 'bag timestamps; record with --use-sim-time',
                      'runs': runs}, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
