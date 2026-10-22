# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
import csv
import importlib.util
import os
from pathlib import Path
import sys

from common_msg.msg import PathProfile
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from nav_msgs.msg import OccupancyGrid
import rclpy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension


def test_epochs_pairing_empty_path_and_queued_samples(tmp_path):
    scripts = str(Path(__file__).parents[1] / 'scripts')
    sys.path.insert(0, scripts)
    spec = importlib.util.spec_from_file_location(
        'recorder', Path(scripts) / 'record_path_tracking.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    os.environ['ROS_DOMAIN_ID'] = str(175 + os.getpid() % 20)
    rclpy.init(args=['--ros-args', '-p', f'output_dir:={tmp_path}', '-p', 'use_sim_time:=false'])
    node = module.PathRecorder()
    try:
        path = PathMsg()
        path.header.frame_id = 'map'
        path.header.stamp.sec = 1
        for x in (0., 1.):
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x = x
            p.pose.orientation.w = 1.
            path.poses.append(p)
        profile = PathProfile(path=path, tangent_yaw=[0., 0.], curvature=[0., 0.],
                              stop_required=[False, True], curvature_continuous=True,
                              segment_type=[0, 0], speed_limit=[.12, 0.],
                              curvature_bound=[0., 0.], curvature_rate_bound=[0., 0.],
                              processing_report='{"retained_corners": 0}')
        node.on_profile(profile)  # DDS may deliver profile before its Path.
        node.on_path(path)
        msg = Float64MultiArray()
        fields = ['ey', 'etheta', 'progress', 'remaining', 'v_ref', 'v_cmd', 'w_cmd',
                  'curvature', 'w_ff', 'v_actual', 'w_actual', 'pose_x', 'pose_y',
                  'pose_yaw', 'ros_time', 'path_sec', 'path_nanosec', 'state', 'geometry_valid',
                  'nearest_x', 'nearest_y']
        msg.layout.dim = [MultiArrayDimension(label=','.join(fields))]
        msg.data = [0.] * len(fields)
        values = dict(ros_time=10., path_sec=1., geometry_valid=1., state=2.,
                      pose_x=.3, pose_y=.4)
        for key, value in values.items():
            msg.data[fields.index(key)] = value
        node.on_tracking(msg)
        path.header.stamp.sec = 2
        node.on_path(path)
        node.on_tracking(msg)  # Old epoch must not enter the new CSV.
        msg.data[fields.index('path_sec')] = 2.
        msg.data[fields.index('ros_time')] = 11.
        msg.data[fields.index('state')] = 4.
        node.on_tracking(msg)
        msg.data[fields.index('ros_time')] = 12.
        node.on_tracking(msg)  # Goal sample is saved exactly once.
        node.on_path(PathMsg())
        node.on_tracking(msg)  # Empty path stops recording immediately.
        node.flush()
        epochs = sorted(node.directory.glob('path_*'))
        assert len(epochs) == 2
        for epoch in epochs:
            with (epoch / 'actual_path.csv').open() as fp:
                rows = list(csv.DictReader(fp))
                assert len(rows) == 1
                assert float(rows[0]['position_error']) == .5
        assert rows[0]['state'] == '4.0'
        assert '"profile_received": true' in (epochs[0] / 'path.json').read_text()
        with (epochs[0] / 'planned_path.csv').open() as fp:
            planned = list(csv.DictReader(fp))
        assert float(planned[0]['speed_limit']) == .12
        assert int(planned[0]['segment_type']) == 0
        report = (epochs[0] / 'planning_report.json').read_text().strip()
        assert report == profile.processing_report
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.path.remove(scripts)


def test_metadata_async_success_timeout_map_and_shutdown(tmp_path):
    import time
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    scripts = str(Path(__file__).parents[1] / 'scripts')
    sys.path.insert(0, scripts)
    from tracking_metadata import MetadataCapture, json_parameter
    os.environ['ROS_DOMAIN_ID'] = str(175 + os.getpid() % 20)
    rclpy.init()
    source = rclpy.create_node('tracking_parameter_source', parameter_overrides=[
        Parameter('v_nominal', value=.3), Parameter('array_test', value=[1., 2.])],
        automatically_declare_parameters_from_overrides=True)
    recorder = rclpy.create_node('tracking_metadata_test')
    capture = MetadataCapture(recorder, ['/tracking_parameter_source', '/tracking_missing'], 1.)
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(recorder)
    try:
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width, grid.info.height = 2, 1
        grid.info.resolution = .05
        grid.info.origin.orientation.w = 1.
        grid.data = [0, 100]
        capture.on_map(grid)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 1.
        goal.pose.orientation.w = 1.
        capture.on_goal(goal)
        path = PathMsg()
        path.header.frame_id = 'map'
        path.poses = [PoseStamped(), goal]
        first = tmp_path / 'path_0001'
        first.mkdir()
        capture.start(first, path)
        end = time.monotonic() + 3.
        while capture.jobs and time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
            capture.poll()
        import json
        document = json.loads((first / 'experiment.json').read_text())
        assert document['parameters']['/tracking_parameter_source']['status'] == 'ok'
        values = document['parameters']['/tracking_parameter_source']['values']
        assert values['v_nominal'] == .3 and values['array_test'] == [1., 2.]
        assert document['parameters']['/tracking_missing']['status'] == 'unavailable'
        assert len(document['map']['sha256']) == 64
        assert not document['atomic_snapshot']
        assert json_parameter([float('nan'), float('inf'), b'\x01']) == [
            {'nonfinite_float': 'nan'}, {'nonfinite_float': 'inf'}, {'bytes_hex': '01'}]
        assert document['last_received_goal']['pose']['position'][0] == 1.
        second = tmp_path / 'path_0002'
        second.mkdir()
        capture.start(second, path)
        capture.close()
        assert 'stopped before capture' in (second / 'experiment.json').read_text()
        assert (first / 'experiment.json').read_text() != (second / 'experiment.json').read_text()
    finally:
        capture.close()
        executor.shutdown()
        source.destroy_node()
        recorder.destroy_node()
        rclpy.shutdown()
        sys.path.remove(scripts)
