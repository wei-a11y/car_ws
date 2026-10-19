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
                              stop_required=[False, True], curvature_continuous=True)
        node.on_profile(profile)  # DDS may deliver profile before its Path.
        node.on_path(path)
        msg = Float64MultiArray()
        fields = ['ey', 'etheta', 'progress', 'remaining', 'v_ref', 'v_cmd', 'w_cmd',
                  'curvature', 'w_ff', 'v_actual', 'w_actual', 'pose_x', 'pose_y',
                  'pose_yaw', 'ros_time', 'path_sec', 'path_nanosec', 'state', 'geometry_valid']
        msg.layout.dim = [MultiArrayDimension(label=','.join(fields))]
        msg.data = [0.] * len(fields)
        values = dict(ros_time=10., path_sec=1., geometry_valid=1., state=2.)
        for key, value in values.items():
            msg.data[fields.index(key)] = value
        node.on_tracking(msg)
        path.header.stamp.sec = 2
        node.on_path(path)
        node.on_tracking(msg)  # Old epoch must not enter the new CSV.
        msg.data[fields.index('path_sec')] = 2.
        msg.data[fields.index('ros_time')] = 11.
        node.on_tracking(msg)
        node.on_path(PathMsg())
        node.on_tracking(msg)  # Empty path stops recording immediately.
        node.flush()
        epochs = sorted(node.directory.glob('path_*'))
        assert len(epochs) == 2
        for epoch in epochs:
            with (epoch / 'actual_path.csv').open() as fp:
                assert len(list(csv.DictReader(fp))) == 1
        assert '"profile_received": true' in (epochs[0] / 'path.json').read_text()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.path.remove(scripts)
