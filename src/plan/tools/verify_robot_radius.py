# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Recompute the conservative collision envelope of the current Gazebo robot."""
import itertools
import math
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET


def rotate(rotation, point):
    return tuple(sum(rotation[i][j] * point[j] for j in range(3)) for i in range(3))


def origin_pose(origin):
    xyz = tuple(map(float, origin.get('xyz', '0 0 0').split())) if origin is not None else (
        0.0, 0.0, 0.0)
    r, p, y = map(float, origin.get('rpy', '0 0 0').split()) if origin is not None else (
        0.0, 0.0, 0.0)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rotation = ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
                (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
                (-sp, cp * sr, cp * cr))
    return rotation, xyz


def compose(parent, child):
    a, at = parent
    b, bt = child
    rotation = tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3))
                           for j in range(3)) for i in range(3))
    translation = tuple(v + t for v, t in zip(rotate(a, bt), at))
    return rotation, translation


def stl_points(mesh, package_dir):
    uri = mesh.get('filename')
    prefix = 'package://car_description/'
    if not uri.startswith(prefix):
        raise ValueError('Unsupported mesh URI: ' + uri)
    data = (package_dir / uri[len(prefix):]).read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + count * 50 or count == 0:
        raise ValueError('Expected a non-empty binary STL: ' + uri)
    scale = tuple(map(float, mesh.get('scale', '1 1 1').split()))
    for triangle in range(count):
        for vertex in range(3):
            point = struct.unpack_from('<3f', data, 84 + triangle * 50 + 12 + vertex * 12)
            yield tuple(v * s for v, s in zip(point, scale))


def collision_bounds(root, package_dir):
    poses = {'base_footprint': origin_pose(None)}
    moving = set()
    joints = list(root.findall('joint'))
    while joints:
        before = len(joints)
        for joint in joints[:]:
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            if parent not in poses:
                continue
            if parent in moving or joint.get('type') not in ('fixed', 'continuous'):
                raise ValueError('Unsupported moving geometry: ' + joint.get('name'))
            if joint.get('type') == 'continuous':
                moving.add(child)
            poses[child] = compose(poses[parent], origin_pose(joint.find('origin')))
            joints.remove(joint)
        if len(joints) == before:
            raise ValueError('URDF is not a tree rooted at base_footprint')

    lower = [math.inf] * 2
    upper = [-math.inf] * 2
    for link in root.findall('link'):
        name = link.get('name')
        for collision in link.findall('collision'):
            local = origin_pose(collision.find('origin'))
            rotation, translation = compose(poses[name], local)
            shape = collision.find('geometry')
            mesh, cylinder, sphere, box = [shape.find(tag) for tag in (
                'mesh', 'cylinder', 'sphere', 'box')]
            if name in moving and (mesh is not None or box is not None or
                                   any(local[1])):
                raise ValueError('Cannot bound rotating offset geometry: ' + name)
            if mesh is not None or box is not None:
                if mesh is not None:
                    points = stl_points(mesh, package_dir)
                else:
                    halves = [float(v) / 2 for v in box.get('size').split()]
                    points = itertools.product(*[(-v, v) for v in halves])
                for point in points:
                    rotated = rotate(rotation, point)
                    for axis in range(2):
                        value = rotated[axis] + translation[axis]
                        lower[axis] = min(lower[axis], value)
                        upper[axis] = max(upper[axis], value)
            else:
                # A sphere encloses each cylinder at every wheel rotation angle.
                if cylinder is not None:
                    radius = math.hypot(float(cylinder.get('radius')),
                                        float(cylinder.get('length')) / 2)
                elif sphere is not None:
                    radius = float(sphere.get('radius'))
                else:
                    raise ValueError('Unsupported collision shape: ' + name)
                for axis in range(2):
                    lower[axis] = min(lower[axis], translation[axis] - radius)
                    upper[axis] = max(upper[axis], translation[axis] + radius)
    if not all(math.isfinite(value) for value in lower + upper):
        raise ValueError('Empty or non-finite collision bounds')
    return lower, upper


def main():
    repository = Path(__file__).resolve().parents[3]
    package_dir = repository / 'src/urdf'
    urdf = package_dir / 'urdf/car_urdf.urdf'
    lower, upper = collision_bounds(ET.parse(urdf).getroot(), package_dir)
    radius = math.hypot(*[max(abs(a), abs(b)) for a, b in zip(lower, upper)])
    config = repository / 'src/plan/config/planning_grid.yaml'
    configured = float(re.search(r'^\s+robot_radius:\s+(\S+)', config.read_text(), re.M)[1])
    if not math.isfinite(configured) or configured < math.ceil(radius * 1000) / 1000:
        raise ValueError('Configured radius is smaller than the confirmed collision envelope')
    print('URDF:', urdf)
    print('XY minimum:', lower, 'maximum:', upper)
    print('Conservative enclosing radius:', radius, 'm')
    print('Configured robot_radius:', configured, 'm: PASS')


if __name__ == '__main__':
    main()
