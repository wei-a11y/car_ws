# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Real A*/LQR/gate/recovery with ideal base and Gazebo-compatible state services.

This is a repeatable integration test, not a Gazebo physics acceptance result.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import yaml

from task_runtime.gazebo_backend import ros_pose, transform_pose
from task_runtime.recovery_core import Pose3
from task_runtime.recovery_node import RecoveryTaskManager


class World(Node):
    def __init__(self, scenario):
        super().__init__('recovery_integration_world')
        self.pose, self.velocity = [-1.55, 0.05, 0.0], [0.0, 0.0]
        self.previous = time.monotonic()
        self.events, self.commands, self.sets = [], [], []
        self.scenario = scenario
        self.object = None if scenario in ('clear', 'scan_lost') else Pose3(0.05, 0.05, 0.22, 0.)
        self.original = self.object
        self.identity = {'bag': 'bag', 'temporary': 'temporary', 'stool': 'stool',
                         'grasp_failure': 'grasp_failure'}.get(scenario)
        self.model = f'task_recovery_{self.identity}'
        self.size = (0.24, 0.24, 0.44) if scenario != 'stool' else (0.4, 0.4, 0.55)
        self.dynamic = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        transforms = []
        for child, x, z in [('laser_link', 0.12, 0.35), ('base_link', 0., 0.097)]:
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = 'base_footprint', child
            tf.transform.translation.x, tf.transform.translation.z = x, z
            tf.transform.rotation.w = 1.
            transforms.append(tf)
        self.static.sendTransform(transforms)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 100
        grid.info.resolution = 0.1
        grid.info.origin.position.x = grid.info.origin.position.y = -5.
        grid.info.origin.orientation.w = 1.
        grid.data = [0]*10000
        self.map_pub.publish(grid)
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.create_subscription(String, '/task/events',
                                 lambda m: self.events.append(json.loads(m.data)), 50)
        self.create_service(GetEntityState, '/task/gazebo/get_entity_state', self.get)
        self.create_service(SetEntityState, '/task/gazebo/set_entity_state', self.set)
        self.create_timer(0.01, self.tick)

    def reference(self):
        result = TransformStamped()
        result.transform.translation.x, result.transform.translation.y = self.pose[:2]
        result.transform.translation.z = 0.097
        result.transform.rotation.z = math.sin(self.pose[2]/2)
        result.transform.rotation.w = math.cos(self.pose[2]/2)
        return result.transform

    def get(self, request, response):
        response.header.stamp = self.get_clock().now().to_msg()
        response.header.frame_id = request.reference_frame
        if request.name == 'car::base_link':
            response.success = True
            response.state.pose = ros_pose(Pose3(*self.pose[:2], 0.097, self.pose[2]))
        elif self.object is not None and request.name == self.model:
            response.success = True
            response.state.pose = ros_pose(transform_pose(self.object, self.reference(), True))
        return response

    def set(self, request, response):
        if request.state.name != self.model:
            return response
        p, q = request.state.pose.position, request.state.pose.orientation
        self.object = transform_pose(Pose3(p.x, p.y, p.z, 2*math.atan2(q.z, q.w)),
                                     self.reference())
        self.sets.append(self.object)
        response.success = True
        return response

    def command(self, msg):
        self.velocity = [msg.linear.x, msg.angular.z]
        self.commands.append(tuple(self.velocity))

    def tick(self):
        now = time.monotonic()
        dt, self.previous = min(now-self.previous, 0.05), now
        v, w = self.velocity
        self.pose[0] += v*math.cos(self.pose[2])*dt
        self.pose[1] += v*math.sin(self.pose[2])*dt
        self.pose[2] += w*dt
        stamp = self.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'map', 'base_footprint'
        tf.header.stamp = stamp
        tf.transform = self.reference()
        tf.transform.translation.z = 0.
        self.dynamic.sendTransform(tf)
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id, odom.header.stamp = (
            'odom', 'base_footprint', stamp)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.velocity
        self.odom_pub.publish(odom)
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = 'laser_link', stamp
        scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi/360
        scan.range_min, scan.range_max = 0.12, 8.
        ranges = [7.0]*721
        if self.object and self.object.z-self.size[2]/2 <= 0.35 <= self.object.z+self.size[2]/2:
            sx = self.pose[0] + 0.12*math.cos(self.pose[2])
            sy = self.pose[1] + 0.12*math.sin(self.pose[2])
            for i in range(721):
                angle = self.pose[2] + scan.angle_min + i*scan.angle_increment
                near, far = 0., 8.
                for origin, direction, center, extent in (
                        (sx, math.cos(angle), self.object.x, self.size[0]/2),
                        (sy, math.sin(angle), self.object.y, self.size[1]/2)):
                    if abs(direction) < 1e-10:
                        if abs(origin-center) > extent:
                            far = -1.
                            break
                    else:
                        ends = sorted(((center-extent-origin)/direction,
                                       (center+extent-origin)/direction))
                        near, far = max(near, ends[0]), min(far, ends[1])
                if 0.12 < near <= far:
                    ranges[i] = near
        scan.ranges = ranges
        if self.scenario != 'scan_lost' or not any(v > 0 for v, _ in self.commands):
            self.scan_pub.publish(scan)


@pytest.mark.parametrize('scenario', ['clear', 'bag', 'temporary', 'stool',
                                      'grasp_failure', 'scan_lost'])
def test_complete_recovery_mission(tmp_path, scenario):
    os.environ['ROS_DOMAIN_ID'] = str(125+os.getpid() % 20)
    points = tmp_path/'points.yaml'
    points.write_text(yaml.safe_dump(dict(configured=True, frame='map',
        home=dict(name='home', x=1.75, y=0.05, yaw=0.0),
        pickup=dict(name='pickup', x=1.55, y=0.05, yaw=0.0),
        dropoffs=[dict(name='drop', x=1.65, y=0.05, yaw=0.0)])))
    source = Path(__file__).parents[1]
    cfg = yaml.safe_load((source/'config/recovery.yaml').read_text())
    cfg['cost'].update(grasp_seconds=0., place_seconds=0., angular_speed=1000., work_weight=0.)
    cfg['backend'].update(lift_speed=2., backend_poll_sec=0.03, action_step_sec=0.05)
    cfg['navigation']['map_settle_sec'] = 0.05
    recovery = tmp_path/'recovery.yaml'
    recovery.write_text(yaml.safe_dump(cfg))
    processes, logs = [], []
    for package, executable, config, extra in (
        ('plan', 'planning_grid_node', 'planning_grid.yaml',
         ['map_topic:=/task/working_map', 'safety_margin:=0.1']),
        ('plan', 'astar_node', 'astar.yaml',
         ['planning_frame:=map', 'path_topic:=/task/candidate_path']),
        ('controller', 'lqr_tracker_node', 'lqr_tracker.yaml',
         ['planning_frame:=map', 'max_tracking_error:=0.2']),
        ('controller', 'safety_gate_node', 'safety_gate.yaml', [])):
        command = [str(Path(get_package_prefix(package))/'lib'/package/executable),
                   '--ros-args', '--params-file',
                   str(Path(get_package_share_directory(package))/'config'/config),
                   '-p', 'use_sim_time:=false']
        for value in extra:
            command.extend(['-p', value])
        log = (tmp_path/f'{executable}.log').open('w')
        logs.append(log)
        processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=false',
                    '-p', 'path_topic:=/task/candidate_path',
                    '-p', f'points_file:={points}', '-p', f'recovery_file:={recovery}',
                    '-p', f'scenarios_file:={source}/config/recovery_scenarios.yaml',
                    '-p', f'log_directory:={tmp_path}/logs',
                    '-p', 'pickup_delay_sec:=0.1', '-p', 'dropoff_delay_sec:=0.1'])
    task, world = RecoveryTaskManager(), World(scenario)
    executor = SingleThreadedExecutor()
    executor.add_node(task)
    executor.add_node(world)
    client = world.create_client(Trigger, '/task/start')
    try:
        deadline, future = time.monotonic()+10, None
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.005)
            try:
                task.ready()
                if task.grid and client.service_is_ready():
                    future = client.call_async(Trigger.Request())
                    break
            except Exception:
                pass
        assert future is not None, f'Not ready: {task.backend.ready()}, grid={task.grid is not None}'
        deadline = time.monotonic()+100
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.005)
            if future.done():
                assert future.result().success, future.result().message
            if scenario == 'scan_lost':
                if task.mode == 'FAILED':
                    break
            else:
                assert task.mode not in ('STOPPING', 'FAILED'), task.failure
            if any(e['event'] == 'TASK_DONE' for e in world.events):
                break
        if scenario == 'scan_lost':
            assert task.mode == 'FAILED' and task.stop_confirmed
            assert task.failure['code'] == 'SCAN_UNAVAILABLE'
            assert not any(e['event'] == 'TASK_DONE' for e in world.events)
            assert world.velocity == [0., 0.]
            return
        assert any(e['event'] == 'TASK_DONE' for e in world.events), task.nav_state
        assert [e['event'] for e in world.events].count('PICKUP_DONE') == 1
        assert [e['event'] for e in world.events].count('DROPOFF_DONE') == 1
        if scenario != 'clear':
            choices = [e['candidate']['kind'] for e in world.events if e['event'] == 'SCHEME_SELECTED']
            assert {'bag': 'carry', 'temporary': 'temporary', 'stool': 'detour',
                    'grasp_failure': 'carry'}[scenario] in choices
        if scenario in ('bag', 'temporary', 'grasp_failure'):
            assert world.sets and not task.ledger.items and task.backend.held_id is None
            assert math.hypot(world.object.x-world.original.x,
                              world.object.y-world.original.y) <= 0.04
            assert any(e['event'] == 'OBJECT_RESTORED' for e in world.events)
        if scenario == 'grasp_failure':
            assert len([e for e in world.events if e['event'] == 'GRASP_STARTED']) == 3
        assert world.velocity == [0., 0.]
    finally:
        executor.shutdown()
        task.destroy_node()
        world.destroy_node()
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
