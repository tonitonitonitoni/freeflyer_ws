#!/usr/bin/env python3

import csv
import math
from collections import deque

import rclpy #type: ignore
from rclpy.node import Node #type: ignore

from geometry_msgs.msg import PoseWithCovarianceStamped #type: ignore

from combined_pose_system.utils.utils_ros import yaw_from_quat

class TrajectoryComparisonNode(Node):
    """
    Collect time-aligned vision and Marvelmind poses for frame alignment.

    Output CSV columns:
      t,
      x_v, y_v, yaw_v,
      x_mm, y_mm, yaw_mm
    """

    def __init__(self):
        super().__init__('trajectory_comparison_node')

        # Parameters
        self.time_tolerance = 0.05  # seconds
        self.csv_path = 'trajectory_pairs.csv'

        # Buffers
        self.vision_buf = deque(maxlen=2000)
        self.mm_buf = deque(maxlen=2000)

        # CSV setup
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            't',
            'x_v', 'y_v', 'yaw_v',
            'x_mm', 'y_mm', 'yaw_mm'
        ])

        # Subscribers
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/vision_pose/raw',
            self.cb_vision,
            10
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/marvelmind/pose',
            self.cb_mm,
            10
        )

        self.get_logger().info(
            f"Trajectory comparison node started, writing to {self.csv_path}"
        )

    def stamp_to_sec(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def cb_vision(self, msg):
        t = self.stamp_to_sec(msg.header.stamp)
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.vision_buf.append((t,
                                 msg.pose.pose.position.x,
                                 msg.pose.pose.position.y,
                                 yaw))
        self.try_match()

    def cb_mm(self, msg):
        t = self.stamp_to_sec(msg.header.stamp)
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.mm_buf.append((t,
                            msg.pose.pose.position.x,
                            msg.pose.pose.position.y,
                            yaw))
        self.try_match()

    def try_match(self):
        if not self.vision_buf or not self.mm_buf:
            return

        tv, xv, yv, yawv = self.vision_buf[0]
        best_idx = None
        best_dt = self.time_tolerance

        for i, (tm, xm, ym, yawm) in enumerate(self.mm_buf):
            dt = abs(tm - tv)
            if dt < best_dt:
                best_dt = dt
                best_idx = i

        if best_idx is None:
            return

        tm, xm, ym, yawm = self.mm_buf[best_idx]

        # Write paired sample
        self.csv_writer.writerow([
            tv,
            xv, yv, yawv,
            xm, ym, yawm
        ])
        self.csv_file.flush()

        # Remove used samples
        self.vision_buf.popleft()
        for _ in range(best_idx + 1):
            self.mm_buf.popleft()


def main():
    rclpy.init()
    node = TrajectoryComparisonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.csv_file.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
