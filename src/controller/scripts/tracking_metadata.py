# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Best-effort, nonblocking per-epoch metadata using standard ROS parameter services."""

import gzip
import hashlib
import json
import math
from pathlib import Path
import subprocess
import struct
import tarfile
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
        self.map_cells = None
        self.source = None
        self.goal = None
        self.jobs = []

    def on_map(self, message):
        meta = message.info
        geometry = {'width': meta.width, 'height': meta.height, 'resolution': meta.resolution,
                    'origin': pose_dict(meta.origin), 'frame': message.header.frame_id}
        digest = hashlib.sha256(json.dumps(geometry, sort_keys=True).encode())
        digest.update(bytes((int(v) & 255 for v in message.data)))
        origin = meta.origin
        payload = struct.pack('<QQdddd', meta.width, meta.height, meta.resolution,
                              origin.position.x, origin.position.y,
                              2 * math.atan2(origin.orientation.z, origin.orientation.w))
        payload += bytes((int(v) & 255 for v in message.data))
        fingerprint = 14695981039346656037
        for value in payload:
            fingerprint = ((fingerprint ^ value) * 1099511628211) & ((1 << 64) - 1)
        self.map = {**geometry, 'sha256': digest.hexdigest(),
                    'fingerprint_fnv1a64': f'{fingerprint:016x}', 'stamp': stamp(message.header)}
        self.map_cells = [int(v) for v in message.data]

    def on_goal(self, message):
        self.goal = {'header': stamp(message.header), 'pose': pose_dict(message.pose),
                     'received_ros_time': self.node.get_clock().now().nanoseconds / 1e9,
                     'association': 'last received goal; not an atomic planner request ID'}

    def capture_source(self, directory):
        if self.source is not None:
            return self.source
        root = next((p for p in (Path.cwd(), *Path.cwd().parents)
                     if (p / '.git').exists()), None)
        if root is None:
            self.source = {'status': 'unavailable', 'reason': 'not launched in repository'}
            return self.source
        try:
            revision = subprocess.check_output(
                ['git', 'rev-parse', 'HEAD'], cwd=root, timeout=2).decode().strip()
            patch = subprocess.check_output(['git', 'diff', 'HEAD', '--binary'],
                                            cwd=root, timeout=2)
            files = subprocess.check_output(
                ['git', 'ls-files', '--cached', '--others', '--exclude-standard',
                 '--', 'src', 'docs'], cwd=root, timeout=2).decode().splitlines()
            digest = hashlib.sha256()
            inventory = []
            for name in sorted(set(files)):
                file = root / name
                if (file.is_file() and file.suffix in
                        ('.cpp', '.hpp', '.py', '.yaml', '.xml', '.txt', '.md')):
                    digest.update(name.encode())
                    digest.update(file.read_bytes())
                    inventory.append((name, file))
            with tarfile.open(directory.parent / 'source_snapshot.tar.gz', 'w:gz') as archive:
                for name, file in inventory:
                    archive.add(file, arcname=name, recursive=False)
            artifact = directory.parent / 'source.patch'
            artifact.write_bytes(patch)
            self.source = {'status': 'ok', 'revision': revision,
                           'tracked_diff_sha256': hashlib.sha256(patch).hexdigest(),
                           'source_tree_sha256': digest.hexdigest(),
                           'tracked_diff_file': '../source.patch',
                           'source_snapshot': '../source_snapshot.tar.gz'}
        except (OSError, subprocess.SubprocessError) as error:
            self.source = {'status': 'unavailable', 'reason': str(error)}
        return self.source

    def start(self, directory, path):
        snapshot = None
        if self.map is not None and self.map_cells is not None:
            filename = 'inflated_grid_' + self.map['sha256'] + '.json.gz'
            file = directory.parent / filename
            if not file.exists():
                with gzip.open(file, 'wt', encoding='utf-8') as fp:
                    json.dump({'map': self.map, 'data': self.map_cells}, fp)
            snapshot = '../' + filename
        document = {'schema_version': 2, 'source': self.capture_source(directory),
                    'map_snapshot': snapshot, 'path_stamp': stamp(path.header),
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
