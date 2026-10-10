# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Explicit placement of allowlisted recovery fixtures; never deletes a model."""
import argparse
import json
import math
from pathlib import Path
import time
from xml.sax.saxutils import escape

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import GetEntityState, SpawnEntity
from geometry_msgs.msg import Transform, Vector3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
import yaml

from task_runtime.gazebo_backend import load_registry, ros_pose, stamp_ns, transform_pose
from task_runtime.recovery_core import Pose3


def model_sdf(item):
    """Gravity-free, collision-enabled fixture controlled only by this simulator."""
    name = escape(item['model_name'], {'"': '&quot;'})
    size = ' '.join(str(float(value)) for value in item['size'])
    mass = float(item.get('mass_kg', 1.0))
    if not math.isfinite(mass) or mass <= 0:
        raise ValueError('Fixture mass must be finite and positive')
    x, y, z = item['size']
    ix, iy, iz = mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12
    color = ' '.join(str(float(value)) for value in item.get('color', [0.2, 0.5, 0.8, 1.0]))
    return f'''<sdf version="1.6"><model name="{name}"><static>false</static>
<link name="fixture"><gravity>false</gravity><kinematic>true</kinematic>
<inertial><mass>{mass}</mass><inertia><ixx>{ix}</ixx><iyy>{iy}</iyy><izz>{iz}</izz>
<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
<collision name="collision"><geometry><box><size>{size}</size></box></geometry></collision>
<visual name="visual"><geometry><box><size>{size}</size></box></geometry>
<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
</link></model></sdf>'''


def wait_future(node, future, timeout):
    deadline = time.monotonic()+timeout
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    if not future.done():
        future.cancel()
        raise RuntimeError('Gazebo service timed out')
    return future.result()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['spawn', 'status'])
    parser.add_argument('scenario', choices=['bag', 'stool', 'temporary', 'occluded', 'grasp_failure'])
    parser.add_argument('--registry', default=str(
        Path(get_package_share_directory('task'))/'config'/'recovery_scenarios.yaml'))
    parser.add_argument('--x', type=float)
    parser.add_argument('--y', type=float)
    parser.add_argument('--z', type=float, help='Model centre height; defaults to half fixture height')
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--frame', choices=['map', 'world'], default='map')
    parser.add_argument('--reference-entity', default='car::base_link')
    parser.add_argument('--reference-frame', default='base_link')
    parser.add_argument('--get-service', default='/task/gazebo/get_entity_state')
    parser.add_argument('--spawn-service', default='/spawn_entity')
    parser.add_argument('--timeout', type=float, default=5.0)
    options, ros_args = parser.parse_known_args(args)
    if options.command == 'spawn' and (options.x is None or options.y is None):
        parser.error('spawn requires explicit --x and --y coordinates')
    if any(value is not None and not math.isfinite(value) for value in
           (options.x, options.y, options.z, options.yaw, options.timeout)) or options.timeout <= 0:
        parser.error('Coordinates must be finite and timeout positive')
    registry = load_registry(options.registry)
    with open(options.registry, encoding='utf-8') as stream:
        scenarios = yaml.safe_load(stream)['scenarios']
    placements = scenarios[options.scenario]
    for item in placements:
        if item['object_id'] not in registry:
            raise ValueError('Scenario references unregistered object')
    rclpy.init(args=ros_args)
    node = Node('task_recovery_scenario', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    get = node.create_client(GetEntityState, options.get_service)
    spawn = node.create_client(SpawnEntity, options.spawn_service)
    try:
        if not get.wait_for_service(timeout_sec=options.timeout):
            raise RuntimeError('Gazebo state plugin unavailable; use obstacle_recovery.launch.py')
        if options.command == 'spawn' and not spawn.wait_for_service(timeout_sec=options.timeout):
            raise RuntimeError('Gazebo spawn service unavailable')
        transform = world_reference = None
        if options.command == 'spawn' and options.frame == 'map':
            request = GetEntityState.Request(name=options.reference_entity, reference_frame='world')
            reference = wait_future(node, get.call_async(request), options.timeout)
            if not reference.success:
                raise RuntimeError('Gazebo reference entity unavailable')
            p = reference.state.pose.position
            world_reference = Transform(translation=Vector3(x=p.x, y=p.y, z=p.z),
                                        rotation=reference.state.pose.orientation)
            reference_time = Time(nanoseconds=stamp_ns(reference.header.stamp))
            deadline = time.monotonic()+options.timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
                try:
                    transform = buffer.lookup_transform('map', options.reference_frame, reference_time).transform
                    break
                except Exception:
                    pass
            if transform is None:
                raise RuntimeError('Timestamped map-to-reference TF unavailable; initialize localization first')
        for placement in placements:
            item = registry[placement['object_id']]
            if options.command == 'status':
                request = GetEntityState.Request(name=item['model_name'], reference_frame='world')
                response = wait_future(node, get.call_async(request), options.timeout)
                result = dict(object_id=item['object_id'], model_name=item['model_name'], present=response.success)
                if response.success:
                    p, q = response.state.pose.position, response.state.pose.orientation
                    result.update(frame='world', x=p.x, y=p.y, z=p.z, yaw=2*math.atan2(q.z, q.w))
                print(json.dumps(result, allow_nan=False))
                continue
            forward, lateral = placement.get('forward', 0.0), placement.get('lateral', 0.0)
            pose = Pose3(options.x+math.cos(options.yaw)*forward-math.sin(options.yaw)*lateral,
                         options.y+math.sin(options.yaw)*forward+math.cos(options.yaw)*lateral,
                         options.z if options.z is not None else item['size'][2]/2, options.yaw)
            # Resolve the requested map pose into the inertial world using the
            # same-time Gazebo reference and TF. A later-moving robot cannot
            # shift a fixture between coordinate conversion and SpawnEntity.
            relative = transform_pose(transform_pose(pose, transform, inverse=True), world_reference) if transform else pose
            request = SpawnEntity.Request(name=item['model_name'], xml=model_sdf(item),
                                          initial_pose=ros_pose(relative),
                                          reference_frame='world')
            response = wait_future(node, spawn.call_async(request), options.timeout)
            print(json.dumps(dict(object_id=item['object_id'], success=response.success,
                                  detail=response.status_message), ensure_ascii=False))
            if not response.success:
                raise RuntimeError(f'Fixture spawn failed: {response.status_message}; no existing model was modified')
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
