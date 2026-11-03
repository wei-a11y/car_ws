#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Closed-loop A/B/C/D on exact frozen profiles, including reverse-direction speed replay."""
import argparse
from copy import deepcopy
from datetime import datetime
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
from zoneinfo import ZoneInfo
from urllib.parse import urlparse

import yaml

from analyze_bezier_p1 import load_path
from analyze_joint_bezier import same_geometry
from run_bezier_gazebo import dual_switches_match, stop
from run_bezier_refine import sha, snapshot, write_json


def validate_capture(directory, rows, variant, map_hash, limits):
    """Missing recorder metadata invalidates a trial without losing its outcome."""
    result = dict.fromkeys(('configuration_matches', 'map_matches', 'profile_matches',
                            'tracking_bounds_match'), False)
    try:
        captures = list(directory.glob('record/run_*/path_0001/experiment.json'))
        if len(captures) != 1:
            raise ValueError(f'Expected one recorder epoch, found {len(captures)}')
        metadata = json.loads(captures[0].read_text())
        planner = metadata.get('parameters', {}).get('/planner', {}).get('values', {})
        result['configuration_matches'] = dual_switches_match(metadata) and (
            planner.get('post_processing.coordination_policy') ==
            ('bounded' if variant in ('C', 'D') else 'legacy') and
            planner.get('post_processing.window_feature_refinement') is (variant in ('B', 'D')))
        result['map_matches'] = (metadata.get('map') or {}).get('sha256') == map_hash
        if not result['map_matches']:
            result['capture_error'] = 'Recorder map metadata missing or different from frozen map'
        recorded = load_path(captures[0].parent/'planned_path.csv')
        result['profile_matches'] = same_geometry(rows, recorded) and all(
            a['speed_limit'] == b['speed_limit'] for a, b in zip(rows, recorded))
        import pandas as pd
        actual = pd.read_csv(captures[0].parent/'actual_path.csv')
        if not len(actual):
            raise ValueError('Recorder actual path is empty')
        for field in ('position_error', 'lateral_error'):
            if not all(math.isfinite(float(value)) for value in actual[field]):
                raise ValueError(f'Recorder {field} contains nonfinite samples')
        result['max_position_error'] = float(actual['position_error'].abs().max())
        result['max_lateral_error'] = float(actual['lateral_error'].abs().max())
        result['tracking_bounds_match'] = (
            result['max_position_error'] <= limits['max_position_error'] and
            result['max_lateral_error'] <= limits['max_lateral_error'])
    except (OSError, KeyError, TypeError, ValueError) as error:
        result['capture_error'] = str(error)
    return result


def profile_messages(node, rows, report, frame):
    from common_msg.msg import PathProfile
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as RosPath
    path = RosPath()
    path.header.frame_id = frame
    path.header.stamp = node.get_clock().now().to_msg()
    for i, row in enumerate(rows):
        pose = PoseStamped()
        pose.header = deepcopy(path.header)
        pose.pose.position.x, pose.pose.position.y = row['x'], row['y']
        a, b = (rows[i], rows[i+1]) if i+1 < len(rows) else (rows[i-1], rows[i])
        heading = math.atan2(b['y']-a['y'], b['x']-a['x'])
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
        path.poses.append(pose)
    profile = PathProfile()
    profile.path = path
    profile.tangent_yaw = [r['yaw'] for r in rows]
    profile.curvature = [r['curvature'] for r in rows]
    profile.stop_required = [bool(r['stop_required']) for r in rows]
    profile.segment_type = [int(r['segment_type']) for r in rows]
    for field in ('speed_limit', 'curvature_bound', 'curvature_rate_bound'):
        setattr(profile, field, [r[field] for r in rows])
    profile.curvature_continuous = True
    profile.processing_report = json.dumps(report)
    return path, profile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--offline-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--variants', nargs='+', choices=['A', 'B', 'C', 'D'], default=['A', 'B', 'C', 'D'])
    parser.add_argument('--limit', type=int, help='Development probe only; never full acceptance')
    args = parser.parse_args()
    offline = args.offline_dir
    gate = json.loads((offline/'acceptance.json').read_text())
    if not gate['pass']:
        parser.error('Complete bidirectional offline acceptance is required')
    cases = json.loads((offline/'inputs.json').read_text())
    by_name = {c['name']: c for c in cases}
    if 'latest_reverse' not in by_name:
        parser.error('Run offline speed replay with --include-reverse first')
    configuration = yaml.safe_load(Path('src/plan/config/bezier_benchmark.yaml').read_text())['benchmark']
    gazebo = configuration['gazebo']
    limits = yaml.safe_load(Path('src/plan/config/joint_tracking_validation.yaml').read_text())[
        'joint_tracking']
    # Never attach the reset command to somebody else's existing Gazebo master.
    address = urlparse(gazebo['master_uri'])
    if address.hostname not in ('127.0.0.1', 'localhost'):
        parser.error('Joint simulation requires a local Gazebo master')
    with socket.socket() as probe:
        try:
            probe.bind(('127.0.0.1', address.port))
        except OSError:
            probe.bind(('127.0.0.1', 0))
            gazebo['master_uri'] = f'http://127.0.0.1:{probe.getsockname()[1]}'
    original_file = Path(by_name['latest']['source_record'])/'path_0001/experiment.json'
    for file, expected in by_name['latest']['source_hashes'].items():
        if sha(file) != expected:
            parser.error('Original frozen map/metadata changed')
    original = json.loads(original_file.read_text())
    frame = original['map']['frame']
    base_frame = by_name['latest']['planner']['planner']['ros__parameters']['base_frame']
    root = args.output or Path('log/joint_gazebo')/datetime.now(ZoneInfo('Asia/Shanghai')).strftime(
        '%Y%m%d_%H%M%S_%f')
    root.mkdir(parents=True, exist_ok=False)
    from ament_index_python.packages import get_package_prefix
    binary = Path(get_package_prefix('plan'))/'lib/plan/bezier_benchmark'
    snapshot(root, binary.resolve())
    write_json(root/'experiment.json', {'configuration': configuration, 'offline_gate': gate,
              'offline_dir': str(offline.resolve()), 'map_sha256': original['map']['sha256'],
              'input_mode': 'frozen_profile_publication_without_goal_replanning',
              'tracking_limits': limits,
              'variants': args.variants, 'probe_limit': args.limit})
    environment = dict(os.environ, ROS_DOMAIN_ID=str(gazebo['ros_domain_id']),
                       ROS_LOCALHOST_ONLY='1', GAZEBO_MASTER_URI=gazebo['master_uri'],
                       GAZEBO_MODEL_DATABASE_URI='', GAZEBO_LOG_PATH=str((root/'gazebo').resolve()))
    os.environ.update(environment)
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
    from common_msg.msg import PathProfile
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    sys.path.insert(0, str(Path('src/controller/scripts').resolve()))
    from tracking_metadata import MetadataCapture
    rclpy.init()
    results = []
    print(root, flush=True)
    try:
        for variant in args.variants:
            for direction, case_name in (('outward', 'latest'), ('inward', 'latest_reverse')):
                for repeat in range(configuration['repeats_per_direction']):
                    if args.limit is not None and len(results) >= args.limit:
                        return
                    source = offline/variant/f'{case_name}_{repeat+1}'
                    source_path = source/f'case_{variant}.csv'
                    rows = load_path(source_path)
                    frozen = json.loads((source/'offline.jsonl').read_text())
                    directory = root/f'{variant}_{direction}_{repeat+1}'
                    directory.mkdir()
                    env = dict(environment, ROS_LOG_DIR=str((directory/'ros_logs').resolve()))
                    os.environ['ROS_LOG_DIR'] = env['ROS_LOG_DIR']
                    node = rclpy.create_node('joint_frozen_profile_driver', parameter_overrides=[
                        Parameter('use_sim_time', value=True)])
                    buffer = Buffer()
                    listener = TransformListener(buffer, node)
                    capture = MetadataCapture(node, [], 5.)
                    state = {'status': '', 'odom': None}
                    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                    subscriptions = [
                        node.create_subscription(String, '/controller/lqr/status',
                                                 lambda m: state.update(status=m.data), qos),
                        node.create_subscription(Odometry, '/odom',
                                                 lambda m: state.update(odom=m), 10),
                        node.create_subscription(OccupancyGrid, '/plan/inflated_grid', capture.on_map, qos)]
                    path_pub = node.create_publisher(RosPath, '/plan', qos)
                    profile_pub = node.create_publisher(PathProfile, '/plan/profile', qos)
                    initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
                    launch = recorder = None
                    report = {'variant': variant, 'direction': direction, 'repeat': repeat+1,
                              'source_profile': str(source_path), 'source_sha256': sha(source_path),
                              'goal_reached': False, 'timed_out': False, 'status': ''}
                    with (directory/'launch.log').open('w') as launch_log, (
                            directory/'recorder.log').open('w') as recorder_log:
                        try:
                            command = ['ros2', 'launch', 'car_description', 'navigation.launch.py',
                                       'gui:=false', 'rviz:=false', 'controller_type:=lqr',
                                       f'planner_params_file:={(source/"planner.yaml").resolve()}',
                                       f'motion_limits_file:={(source/"motion.yaml").resolve()}',
                                       f'map_yaml:={Path(configuration["map_yaml"]).resolve()}']
                            write_json(directory/'command.json', command)
                            launch = subprocess.Popen(command, env=env, stdout=launch_log,
                                stderr=subprocess.STDOUT, start_new_session=True)
                            deadline = time.monotonic()+gazebo['startup_timeout']
                            while time.monotonic() < deadline:
                                rclpy.spin_once(node, timeout_sec=.05)
                                if (state['odom'] is not None and capture.map is not None and
                                        node.get_clock().now().nanoseconds > 0 and buffer.can_transform(
                                            frame, base_frame, rclpy.time.Time())):
                                    break
                                if launch.poll() is not None:
                                    raise RuntimeError('Navigation launch exited during startup')
                            else:
                                raise RuntimeError('Simulation/localization did not become ready')
                            if capture.map['sha256'] != original['map']['sha256']:
                                raise RuntimeError('Live inflated map differs from frozen input')
                            start = [rows[0]['x'], rows[0]['y']]
                            world, mapped = gazebo['world_reference_pose'], gazebo['map_reference_pose']
                            angle = world[2]-mapped[2]
                            dx, dy = start[0]-mapped[0], start[1]-mapped[1]
                            x = world[0]+math.cos(angle)*dx-math.sin(angle)*dy
                            y = world[1]+math.sin(angle)*dx+math.cos(angle)*dy
                            subprocess.run(['gz', 'model', '-m', gazebo['model'], '-x', str(x),
                                '-y', str(y), '-z', str(gazebo['model_z']),
                                '-Y', str(configuration['initial_yaw']+angle)], env=env,
                                stdout=launch_log, stderr=subprocess.STDOUT, check=True,
                                timeout=gazebo['startup_timeout'])
                            initial = PoseWithCovarianceStamped()
                            initial.header.frame_id = frame
                            initial.pose.pose.position.x, initial.pose.pose.position.y = start
                            yaw = configuration['initial_yaw']
                            initial.pose.pose.orientation.z, initial.pose.pose.orientation.w = (
                                math.sin(yaw/2), math.cos(yaw/2))
                            initial.pose.covariance[0] = initial.pose.covariance[7] = (
                                gazebo['initial_position_variance'])
                            initial.pose.covariance[35] = gazebo['initial_heading_variance']
                            settled = time.monotonic()+gazebo['stabilization_seconds']
                            while time.monotonic() < settled:
                                initial.header.stamp = node.get_clock().now().to_msg()
                                initial_pub.publish(initial)
                                rclpy.spin_once(node, timeout_sec=.1)
                            tf = buffer.lookup_transform(frame, base_frame, rclpy.time.Time())
                            p, q = tf.transform.translation, tf.transform.rotation
                            measured_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                            if (math.hypot(p.x-start[0], p.y-start[1]) > gazebo['start_position_tolerance']
                                    or abs(math.remainder(measured_yaw-yaw, 2*math.pi)) >
                                    gazebo['start_heading_tolerance']):
                                raise RuntimeError('Reset pose not confirmed by localization')
                            report['confirmed_start'] = [p.x, p.y, measured_yaw]
                            recorder = subprocess.Popen([
                                'python3', 'src/controller/scripts/record_path_tracking.py', '--ros-args',
                                '--params-file', 'src/controller/config/path_tracking.yaml',
                                '-p', f'output_dir:={directory.resolve()}/record'], env=env,
                                stdout=recorder_log, stderr=subprocess.STDOUT, start_new_session=True)
                            deadline = time.monotonic()+gazebo['startup_timeout']
                            while min(path_pub.get_subscription_count(), profile_pub.get_subscription_count()) < 2:
                                if time.monotonic() >= deadline:
                                    raise RuntimeError('Controller/recorder subscriptions not ready')
                                rclpy.spin_once(node, timeout_sec=.05)
                            # Discovery precedes the recorder's transient-local map callback.
                            settled = time.monotonic()+gazebo['stabilization_seconds']
                            while time.monotonic() < settled:
                                if recorder.poll() is not None:
                                    raise RuntimeError('Recorder exited before profile publication')
                                rclpy.spin_once(node, timeout_sec=.05)
                            path, profile = profile_messages(node, rows, frozen['report'], frame)
                            state['status'] = ''
                            begun = node.get_clock().now().nanoseconds/1e9
                            profile_pub.publish(profile)
                            path_pub.publish(path)
                            deadline = time.monotonic()+gazebo['trial_timeout']
                            while time.monotonic() < deadline:
                                rclpy.spin_once(node, timeout_sec=.05)
                                if state['status'] in ('GOAL_REACHED', 'TRACKING_ERROR'):
                                    break
                                if launch.poll() is not None:
                                    raise RuntimeError('Navigation launch exited during replay')
                            report.update(status=state['status'], goal_reached=state['status']=='GOAL_REACHED',
                                timed_out=state['status'] not in ('GOAL_REACHED', 'TRACKING_ERROR'),
                                duration_ros=node.get_clock().now().nanoseconds/1e9-begun)
                        except Exception as error:
                            report['error'] = str(error)
                        finally:
                            stop(recorder)
                            stop(launch)
                            listener.unregister()
                            for subscription in subscriptions:
                                node.destroy_subscription(subscription)
                            node.destroy_node()
                    # Persist movement outcome even if post-run validation fails unexpectedly.
                    write_json(directory/'result.json', report)
                    report.update(validate_capture(directory, rows, variant, original['map']['sha256'], limits))
                    write_json(directory/'result.json', report)
                    results.append(report)
                    write_json(root/'results.json', results)
                    print(directory.name, json.dumps(report), flush=True)
                    if variant == 'A' and not all(report[k] for k in (
                            'goal_reached', 'configuration_matches', 'map_matches', 'profile_matches',
                            'tracking_bounds_match')):
                        raise RuntimeError('Baseline prerequisite failed; candidate trials stopped')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
