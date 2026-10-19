#!/usr/bin/env python3

import csv
from pathlib import Path

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path as PathMsg

import tf2_ros


class PathRecorder(Node):

    def __init__(self):
        super().__init__('path_recorder')

        # =========================
        # 参数
        # =========================
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('record_period', 0.05)
        self.declare_parameter(
            'output_dir',
            'data/path_record'  # 相对于启动程序时的工作目录
        )

        self.plan_topic = self.get_parameter(
            'plan_topic'
        ).value

        self.global_frame = self.get_parameter(
            'global_frame'
        ).value

        self.base_frame = self.get_parameter(
            'base_frame'
        ).value

        self.record_period = self.get_parameter(
            'record_period'
        ).value

        self.output_dir = Path(
            self.get_parameter('output_dir').value
        )

        self.output_dir.mkdir(
            parents=True,
            exist_ok=True
        )

        # =========================
        # TF
        # =========================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # =========================
        # 记录计划路径
        # =========================
        self.plan_sub = self.create_subscription(
            PathMsg,
            self.plan_topic,
            self.plan_callback,
            10
        )

        # =========================
        # 文件
        # =========================
        self.plan_file = self.output_dir / 'planned_path.csv'
        self.actual_file = self.output_dir / 'actual_path.csv'

        self.plan_points = []

        self.plan_received = False
        self.start_time = None

        # 创建 actual csv
        self.actual_fp = open(
            self.actual_file,
            'w',
            newline=''
        )

        self.actual_writer = csv.writer(self.actual_fp)

        self.actual_writer.writerow([
            'time',
            'x',
            'y',
            'yaw'
        ])

        # =========================
        # 定时记录实际轨迹
        # =========================
        self.timer = self.create_timer(
            self.record_period,
            self.record_actual_pose
        )

        self.get_logger().info(
            'Path recorder started.'
        )

        self.get_logger().info(
            f'Plan topic: {self.plan_topic}'
        )

        self.get_logger().info(
            f'TF: {self.global_frame} -> {self.base_frame}'
        )

        self.get_logger().info(
            f'Output: {self.output_dir}'
        )

    # =========================================================
    # 接收 A* / Phase4 路径
    # =========================================================

    def plan_callback(self, msg):

        if len(msg.poses) == 0:
            return

        self.plan_points = []

        for pose_stamped in msg.poses:

            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y

            self.plan_points.append((x, y))

        # 每次收到新的 plan，就重新保存
        with open(
            self.plan_file,
            'w',
            newline=''
        ) as fp:

            writer = csv.writer(fp)

            writer.writerow([
                'index',
                'x',
                'y'
            ])

            for i, (x, y) in enumerate(self.plan_points):

                writer.writerow([
                    i,
                    x,
                    y
                ])

        self.plan_received = True

        self.get_logger().info(
            f'Received plan: {len(self.plan_points)} points'
        )

    # =========================================================
    # 记录实际机器人位置
    # =========================================================

    def record_actual_pose(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time()
            )

        except Exception as e:

            self.get_logger().warn(
                f'TF lookup failed: {e}'
            )

            return

        now = self.get_clock().now()

        if self.start_time is None:
            self.start_time = now

        t = (
            now.nanoseconds -
            self.start_time.nanoseconds
        ) / 1e9

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        q = transform.transform.rotation

        yaw = self.quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        self.actual_writer.writerow([
            f'{t:.6f}',
            f'{x:.6f}',
            f'{y:.6f}',
            f'{yaw:.6f}'
        ])

        self.actual_fp.flush()

    # =========================================================
    # 四元数 -> yaw
    # =========================================================

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):

        import math

        siny_cosp = 2.0 * (
            w * z + x * y
        )

        cosy_cosp = 1.0 - 2.0 * (
            y * y + z * z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    # =========================================================
    # 关闭
    # =========================================================

    def destroy_node(self):

        self.get_logger().info(
            'Saving recorded data...'
        )

        try:
            self.actual_fp.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = PathRecorder()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
