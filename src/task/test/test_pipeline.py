# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Real planner/LQR/Safety Gate and task, with ideal differential-drive sensors."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import yaml

from task_runtime.node import TaskManager


def test_real_navigation_round(tmp_path):
    os.environ['ROS_DOMAIN_ID'] = str(175 + os.getpid() % 20)
    points = tmp_path / 'points.yaml'
    points.write_text(yaml.safe_dump(dict(
        configured=True, frame='map',
        home=dict(name='home', x=0.25, y=0.25, yaw=0.0),
        pickup=dict(name='pickup', x=0.45, y=0.25, yaw=0.2),
        dropoffs=[dict(name='drop', x=0.65, y=0.25, yaw=-0.2)])))
    processes, logs = [], []
    for package, executable, config in (
            ('plan', 'astar_node', 'astar.yaml'),
            ('controller', 'lqr_tracker_node', 'lqr_tracker.yaml'),
            ('controller', 'safety_gate_node', 'safety_gate.yaml')):
        args = [str(Path(get_package_prefix(package)) / 'lib' / package / executable),
                '--ros-args', '--params-file',
                str(Path(get_package_share_directory(package)) / 'config' / config),
                '-p', 'use_sim_time:=false']
        if executable != 'safety_gate_node':
            args.extend(['-p', 'planning_frame:=map'])
        log = (tmp_path / f'{executable}.log').open('w')
        logs.append(log)
        processes.append(subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT))
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=false',
                     '-p', f'points_file:={points}', '-p', f'log_directory:={tmp_path / "logs"}',
                     '-p', 'pickup_delay_sec:=0.1', '-p', 'dropoff_delay_sec:=0.1'])
    task = TaskManager()
    sim = Node('ideal_base_test')
    executor = SingleThreadedExecutor()
    executor.add_node(task)
    executor.add_node(sim)
    dynamic = TransformBroadcaster(sim)
    static = StaticTransformBroadcaster(sim)
    transform = TransformStamped()
    transform.header.frame_id = 'base_footprint'
    transform.child_frame_id = 'laser_link'
    transform.transform.rotation.w = 1.0
    static.sendTransform(transform)
    odom_pub = sim.create_publisher(Odometry, '/odom', 10)
    scan_pub = sim.create_publisher(LaserScan, '/scan', 10)
    grid_pub = sim.create_publisher(OccupancyGrid, '/plan/inflated_grid', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 30
    grid.info.resolution = 0.1
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 900
    grid_pub.publish(grid)
    pose, velocity, previous = [0.25, 0.25, 0.0], [0.0, 0.0], [time.monotonic()]
    commands, events = [], []

    def command(msg):
        velocity[:] = [msg.linear.x, msg.angular.z]
        commands.append((msg.linear.x, msg.angular.z))

    sim.create_subscription(Twist, '/cmd_vel', command, 10)
    sim.create_subscription(String, '/task/events',
                            lambda msg: events.append(json.loads(msg.data)), 20)

    def tick():
        now = time.monotonic()
        dt = min(now - previous[0], 0.05)
        previous[0] = now
        pose[0] += velocity[0] * math.cos(pose[2]) * dt
        pose[1] += velocity[0] * math.sin(pose[2]) * dt
        pose[2] += velocity[1] * dt
        stamp = sim.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'map', 'base_footprint'
        tf.header.stamp = stamp
        tf.transform.translation.x, tf.transform.translation.y = pose[:2]
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
        dynamic.sendTransform(tf)
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
        odom.header.stamp = stamp
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = velocity
        odom_pub.publish(odom)
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = 'laser_link', stamp
        scan.angle_min, scan.angle_max = -math.pi, math.pi
        scan.angle_increment = 2 * math.pi / 360
        scan.range_min, scan.range_max = 0.12, 8.0
        scan.ranges = [8.0] * 361
        scan_pub.publish(scan)

    sim.create_timer(0.02, tick)
    client = sim.create_client(Trigger, '/task/start')
    try:
        deadline = time.monotonic() + 8
        future = None
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
            if (client.service_is_ready() and task.grid and task.odom_valid() and
                    task.inhibit.service_is_ready() and 'min_range' in task.snapshot and
                    'lqr_diagnostic' in task.snapshot):
                # Allow TF delivery independent of diagnostics.
                try:
                    task.ready()
                    future = client.call_async(Trigger.Request())
                    break
                except Exception:
                    pass
        assert future is not None, 'Navigation input discovery failed'
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
            if future.done():
                assert future.result().success, future.result().message
            assert task.mode not in ('FAILED', 'STOPPING'), task.failure
            if any(event['event'] == 'TASK_DONE' for event in events):
                break
        assert [e['event'] for e in events].count('TASK_DONE') == 1, task.phase
        assert [e['event'] for e in events].count('PICKUP_DONE') == 1
        assert [e['event'] for e in events].count('DROPOFF_DONE') == 1
        assert any(v > 0 for v, _ in commands)
        assert any(abs(w) > 0 for _, w in commands)
        assert math.hypot(pose[0] - 0.25, pose[1] - 0.25) <= 0.03
        assert abs(math.remainder(pose[2], 2 * math.pi)) <= 0.05
        assert velocity == [0.0, 0.0]
    finally:
        executor.shutdown()
        task.destroy_node()
        sim.destroy_node()
        rclpy.shutdown()
        for process in processes:
            process.send_signal(signal.SIGINT)
        for process, log in zip(processes, logs):
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
            log.close()
        assert all(p.returncode == 0 for p in processes)
