# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Best-effort, nonblocking per-epoch metadata using standard ROS parameter services."""

import hashlib
import json
import math
import time

from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.parameter import parameter_value_to_python


def stamp(header):
    return {'frame': header.frame_id, 'sec': header.stamp.sec,
            'nanosec': header.stamp.nanosec}


def pose_dict(pose):
    return {'position': [pose.position.x, pose.position.y, pose.position.z],
            'quaternion_xyzw': [pose.orientation.x, pose.orientation.y,
                                pose.orientation.z, pose.orientation.w]}


def json_parameter(value):
    """Preserve unset/nonfinite actuator limits without emitting invalid JSON numbers."""
    if isinstance(value, float) and not math.isfinite(value):
        return {'nonfinite_float': str(value)}
    if isinstance(value, (bytes, bytearray)):
        return {'bytes_hex': value.hex()}
    if isinstance(value, list):
        return [json_parameter(v) for v in value]
    return value


class MetadataCapture:
    def __init__(self, node, names, timeout):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('metadata_timeout must be finite and positive')
        if len(names) != len(set(names)) or any(
                not name.startswith('/') for name in names):
            raise ValueError('metadata_nodes must be unique absolute ROS node names')
        self.node, self.timeout = node, timeout
        self.clients = {name: (node.create_client(ListParameters, name + '/list_parameters'),
                               node.create_client(GetParameters, name + '/get_parameters'))
                        for name in names}
        self.map = None
        self.goal = None
        self.jobs = []

    def on_map(self, message):
        meta = message.info
        geometry = {'width': meta.width, 'height': meta.height, 'resolution': meta.resolution,
                    'origin': pose_dict(meta.origin), 'frame': message.header.frame_id}
        digest = hashlib.sha256(json.dumps(geometry, sort_keys=True).encode())
        digest.update(bytes((int(v) & 255 for v in message.data)))
        self.map = {**geometry, 'sha256': digest.hexdigest(), 'stamp': stamp(message.header)}

    def on_goal(self, message):
        self.goal = {'header': stamp(message.header), 'pose': pose_dict(message.pose),
                     'received_ros_time': self.node.get_clock().now().nanoseconds / 1e9,
                     'association': 'last received goal; not an atomic planner request ID'}

    def start(self, directory, path):
        document = {'schema_version': 1, 'path_stamp': stamp(path.header),
                    'path_endpoints': {'start': pose_dict(path.poses[0].pose),
                                       'end': pose_dict(path.poses[-1].pose)},
                    'map': self.map, 'last_received_goal': self.goal,
                    'capture_ros_time': self.node.get_clock().now().nanoseconds / 1e9,
                    'atomic_snapshot': False, 'parameters': {}}
        for name in self.clients:
            document['parameters'][name] = {'status': 'pending'}
        job = {'file': directory / 'experiment.json', 'document': document,
               'deadline': time.monotonic() + self.timeout,
               'pending': {name: {'stage': 'waiting', 'future': None} for name in self.clients}}
        self.jobs.append(job)
        self._save(job)

    @staticmethod
    def _save(job):
        job['file'].write_text(json.dumps(job['document'], indent=2, allow_nan=False) + '\n')

    def poll(self):
        for job in self.jobs[:]:
            for name, task in list(job['pending'].items()):
                try:
                    result = self._step(name, task)
                except Exception as error:  # A metadata failure must never drop tracking samples.
                    result = {'status': 'error', 'reason': str(error)}
                if result is None and time.monotonic() >= job['deadline']:
                    result = {'status': 'unavailable', 'reason': 'parameter service timeout'}
                if result is not None:
                    future = task['future']
                    if future is not None and not future.done():
                        future.cancel()
                    result['capture_ros_time'] = self.node.get_clock().now().nanoseconds / 1e9
                    job['document']['parameters'][name] = result
                    del job['pending'][name]
                    self._save(job)
            if not job['pending']:
                self.jobs.remove(job)

    def _step(self, name, task):
        list_client, get_client = self.clients[name]
        if task['stage'] == 'waiting':
            if list_client.service_is_ready() and get_client.service_is_ready():
                task['future'] = list_client.call_async(ListParameters.Request())
                task['stage'] = 'listing'
        elif task['future'].done():
            response = task['future'].result()
            if task['stage'] == 'listing':
                task['names'] = list(response.result.names)
                if not task['names']:
                    return {'status': 'ok', 'values': {}}
                request = GetParameters.Request(names=task['names'])
                task['future'] = get_client.call_async(request)
                task['stage'] = 'getting'
            else:
                if len(response.values) != len(task['names']):
                    raise ValueError('Parameter response length mismatch')
                values = {key: json_parameter(parameter_value_to_python(value))
                          for key, value in zip(task['names'], response.values)}
                json.dumps(values, allow_nan=False)
                return {'status': 'ok', 'values': values}
        return None

    def close(self):
        for job in self.jobs:
            for name, task in job['pending'].items():
                if task['future'] is not None and not task['future'].done():
                    task['future'].cancel()
                job['document']['parameters'][name] = {
                    'status': 'unavailable', 'reason': 'recorder stopped before capture completed'}
            self._save(job)
        self.jobs.clear()
