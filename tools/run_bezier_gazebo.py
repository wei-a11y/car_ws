#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Run isolated, resettable Gazebo A/B trials; stop if a baseline prerequisite fails."""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from zoneinfo import ZoneInfo

import yaml


def stop(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--offline-dir', type=Path,
                        help='Frozen offline map run; required for full acceptance')
    parser.add_argument('--config', default='src/plan/config/bezier_benchmark.yaml')
    parser.add_argument('--limit', type=int, help='Development probe only; not full acceptance')
    parser.add_argument('--variants', nargs='+', choices=['baseline', 'combined'],
                        default=['baseline', 'combined'])
    args = parser.parse_args()
    if args.limit is None and args.offline_dir is None:
        parser.error("Full A/B requires --offline-dir to freeze windows before all trials")
    c = yaml.safe_load(Path(args.config).read_text())['benchmark']
    g = c['gazebo']
    root = Path(c['output_root']) / datetime.now(ZoneInfo('Asia/Shanghai')).strftime(
        '%Y%m%d_%H%M%S_%f_gazebo')
    root.mkdir(parents=True)
    print(root, flush=True)
    if args.offline_dir is not None:
        subprocess.run(['python3', 'tools/analyze_bezier_results.py',
                        '--gazebo-dir', str(root), '--offline-dir', str(args.offline_dir),
                        '--freeze-only'], check=True)
    environment = dict(os.environ, ROS_DOMAIN_ID=str(g['ros_domain_id']),
                       ROS_LOCALHOST_ONLY='1', GAZEBO_MASTER_URI=g['master_uri'],
                       GAZEBO_MODEL_DATABASE_URI='',
                       GAZEBO_LOG_PATH=str((root / 'gazebo').resolve()))
    os.environ.update(environment)
    import rclpy
    import numpy as np
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    rclpy.init()
    map_spec = yaml.safe_load(Path(c['map_yaml']).read_text())
    resolution = float(np.float32(map_spec['resolution']))
    results = []
    try:
        trial_index = 0
        for variant in args.variants:
            planner = yaml.safe_load(Path(c['planner_params_file']).read_text())
            motion = yaml.safe_load(Path(c['motion_limits_file']).read_text())
            enabled = variant == 'combined'
            params = planner['planner']['ros__parameters']['post_processing']
            params['enable_window_bezier'] = enabled
            params['enable_turn_speed_coordination'] = enabled
            motion['/**']['ros__parameters']['enable_dynamic_angular_budget'] = enabled
            planner_file = root / f'{variant}_planner.yaml'
            motion_file = root / f'{variant}_motion.yaml'
            planner_file.write_text(yaml.safe_dump(planner, sort_keys=False))
            motion_file.write_text(yaml.safe_dump(motion, sort_keys=False))
            for scenario in c['scenarios']:
                for repeat in range(c['repeats_per_direction']):
                    if args.limit is not None and trial_index >= args.limit:
                        return
                    trial_index += 1
                    directory = root / f"{variant}_{scenario['name']}_{repeat+1}"
                    directory.mkdir()
                    environment['ROS_LOG_DIR'] = str((directory / 'ros_logs').resolve())
                    os.environ['ROS_LOG_DIR'] = environment['ROS_LOG_DIR']
                    node = rclpy.create_node('bezier_experiment_driver', parameter_overrides=[
                        Parameter('use_sim_time', value=True)])
                    buffer = Buffer()
                    listener = TransformListener(buffer, node)
                    state = {'status': '', 'odom': None, 'plan': ''}
                    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                    subscriptions = [
                        node.create_subscription(String, '/controller/lqr/status',
                                                 lambda m: state.update(status=m.data), qos),
                        node.create_subscription(String, '/plan/status',
                                                 lambda m: state.update(plan=m.data), qos),
                        node.create_subscription(Odometry, '/odom',
                                                 lambda m: state.update(odom=m), 10)]
                    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
                    initial_pub = node.create_publisher(
                        PoseWithCovarianceStamped, '/initialpose', 10)
                    launch = recorder = None
                    start = list(scenario['start'])
                    if g['snap_start_to_cell_center']:
                        for axis in range(2):
                            origin = map_spec['origin'][axis]
                            cell = math.floor((start[axis] - origin) / resolution)
                            start[axis] = origin + (cell + .5) * resolution
                    report = {'initialization_target': start, 'variant': variant,
                              'scenario': scenario, 'repeat': repeat + 1,
                              'initial_yaw': c['initial_yaw'], 'goal_reached': False}
                    with (directory / 'launch.log').open('w') as launch_log, \
                            (directory / 'recorder.log').open('w') as record_log:
                        try:
                            command = ['ros2', 'launch', 'car_description', 'navigation.launch.py',
                                       'gui:=false', 'rviz:=false', 'controller_type:=lqr',
                                       f'planner_params_file:={planner_file.resolve()}',
                                       f'motion_limits_file:={motion_file.resolve()}']
                            launch = subprocess.Popen(command, env=environment, stdout=launch_log,
                                                      stderr=subprocess.STDOUT,
                                                      start_new_session=True)
                            deadline = time.monotonic() + g['startup_timeout']
                            while time.monotonic() < deadline:
                                rclpy.spin_once(node, timeout_sec=.05)
                                if (state['odom'] is not None
                                        and node.get_clock().now().nanoseconds > 0
                                        and buffer.can_transform(
                                            'map', 'base_footprint', rclpy.time.Time())):
                                    break
                            else:
                                raise RuntimeError('Simulation/localization did not become ready')
                            world, mapped = g['world_reference_pose'], g['map_reference_pose']
                            angle = world[2] - mapped[2]
                            dx, dy = start[0]-mapped[0], start[1]-mapped[1]
                            x = world[0] + math.cos(angle)*dx - math.sin(angle)*dy
                            y = world[1] + math.sin(angle)*dx + math.cos(angle)*dy
                            subprocess.run(['gz', 'model', '-m', g['model'], '-x', str(x),
                                            '-y', str(y), '-z', str(g['model_z']),
                                            '-Y', str(c['initial_yaw']+angle)], env=environment,
                                           check=True, timeout=g['startup_timeout'],
                                           stdout=launch_log, stderr=subprocess.STDOUT)
                            initial = PoseWithCovarianceStamped()
                            initial.header.frame_id = 'map'
                            initial.pose.pose.position.x, initial.pose.pose.position.y = start
                            initial.pose.pose.orientation.z = math.sin(c['initial_yaw']/2)
                            initial.pose.pose.orientation.w = math.cos(c['initial_yaw']/2)
                            initial.pose.covariance[0] = g['initial_position_variance']
                            initial.pose.covariance[7] = g['initial_position_variance']
                            initial.pose.covariance[35] = g['initial_heading_variance']
                            settled = time.monotonic() + g['stabilization_seconds']
                            while time.monotonic() < settled:
                                initial.header.stamp = node.get_clock().now().to_msg()
                                initial_pub.publish(initial)
                                rclpy.spin_once(node, timeout_sec=.1)
                            tf = buffer.lookup_transform(
                                'map', 'base_footprint', rclpy.time.Time())
                            p, q = tf.transform.translation, tf.transform.rotation
                            yaw = math.atan2(2 * (q.w*q.z + q.x*q.y),
                                             1 - 2 * (q.y*q.y + q.z*q.z))
                            if (math.hypot(p.x-start[0], p.y-start[1]) >
                                    g['start_position_tolerance'] or
                                    abs(math.remainder(yaw-c['initial_yaw'], 2*math.pi)) >
                                    g['start_heading_tolerance']):
                                raise RuntimeError('Reset pose not confirmed by localization')
                            report['confirmed_start'] = [p.x, p.y, yaw]
                            recorder = subprocess.Popen([
                                'python3', 'src/controller/scripts/record_path_tracking.py',
                                '--ros-args',
                                '--params-file', 'src/controller/config/path_tracking.yaml',
                                '-p', f'output_dir:={directory.resolve()}/record'],
                                env=environment,
                                stdout=record_log, stderr=subprocess.STDOUT,
                                start_new_session=True)
                            deadline = time.monotonic() + g['startup_timeout']
                            while (goal_pub.get_subscription_count() == 0
                                   and time.monotonic() < deadline):
                                rclpy.spin_once(node, timeout_sec=.05)
                            # Allow the recorder to subscribe and receive the latched map.
                            settled = time.monotonic() + g['stabilization_seconds']
                            while time.monotonic() < settled:
                                rclpy.spin_once(node, timeout_sec=.05)
                            goal = PoseStamped()
                            goal.header.frame_id = 'map'
                            goal.header.stamp = node.get_clock().now().to_msg()
                            goal.pose.position.x, goal.pose.position.y = scenario['goal']
                            goal.pose.orientation.w = 1.0
                            state['status'] = ''
                            goal_pub.publish(goal)
                            begun = node.get_clock().now().nanoseconds / 1e9
                            deadline = time.monotonic() + g['trial_timeout']
                            while time.monotonic() < deadline:
                                rclpy.spin_once(node, timeout_sec=.05)
                                if state['status'] in ('GOAL_REACHED', 'TRACKING_ERROR'):
                                    break
                                if launch.poll() is not None:
                                    raise RuntimeError('Navigation launch exited during trial')
                            report.update(status=state['status'], planning_status=state['plan'],
                                          goal_reached=state['status'] == 'GOAL_REACHED',
                                          duration_ros=(node.get_clock().now().nanoseconds
                                                        / 1e9 - begun))
                        except Exception as error:
                            report['error'] = str(error)
                        finally:
                            stop(recorder)
                            stop(launch)
                            listener.unregister()
                            for subscription in subscriptions:
                                node.destroy_subscription(subscription)
                            node.destroy_node()
                    (directory / 'result.json').write_text(json.dumps(report, indent=2)+'\n')
                    results.append(report)
                    (root / 'results.json').write_text(json.dumps(results, indent=2)+'\n')
                    print(directory, json.dumps(report), flush=True)
                    if variant == 'baseline' and not report['goal_reached']:
                        raise RuntimeError('Baseline prerequisite failed; A/B acceptance stopped')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
