#!/usr/bin/env python3

import math
import numpy as np

import rclpy #type: ignore
from rclpy.node import Node #type: ignore
from types import SimpleNamespace
from geometry_msgs.msg import PoseWithCovarianceStamped #type: ignore

# Replace this import with the actual Marvelmind message type
# from marvelmind_ros2_msgs.msg import HedgeImuFusion
#
# NOTE:
# You MUST update the import below to match the message package
# used by your Marvelmind ROS2 driver.
#
# The field names are assumed to match the documentation:
#   timestamp_ms, x_m, y_m, z_m, qw, qx, qy, qz
#
from marvelmind_ros2_msgs.msg import HedgeImuFusion #type: ignore
from combined_pose_system.utils.utils_ros import yaw_from_quat


class MarvelmindPoseNode(Node):
    """
    Marvelmind planar pose adapter.

    Frames:
      - Input: Marvelmind internal map frame (mm_map)
      - Output: PoseWithCovarianceStamped in frame 'mm_map'

    Conventions:
      - +x, +y in table plane
      - +z up
      - yaw CCW about +z
    """

    def __init__(self):
        super().__init__('marvelmind_pose_node')

        self.frame_id = 'mm_map'

        # Subscriber
        self.sub = self.create_subscription(
            HedgeImuFusion,
            'hedgehog_imu_fusion',
            self.cb_imu_fusion,
            10
        )

        # Publisher
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'marvelmind/pose',
            10
        )

        # Tunable covariance parameters
        self.pos_var = 0.02 ** 2      # 2 cm std dev (m^2)
        self.yaw_var = math.radians(5.0) ** 2  # 5 deg std dev (rad^2)

        self.get_logger().info("Marvelmind pose node initialized")

    def cb_imu_fusion(self, msg: HedgeImuFusion):
        """
        Convert Marvelmind IMU fusion output to planar pose.
        """

        # Extract planar position
        x = msg.x_m
        y = msg.y_m

        # Extract yaw from quaternion
        qi = SimpleNamespace(x=msg.qx, y=msg.qy, z=msg.qz, w=msg.qw)
        yaw = yaw_from_quat(qi)

        pose = PoseWithCovarianceStamped()

        # Timestamp handling
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id

        # Position
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = 0.0

        # Orientation (yaw only)
        pose.pose.pose.orientation.x = 0.0
        pose.pose.pose.orientation.y = 0.0
        pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Covariance (planar)
        cov = [0.0] * 36
        cov[0] = self.pos_var    # x
        cov[7] = self.pos_var    # y
        cov[35] = self.yaw_var   # yaw

        pose.pose.covariance = cov

        self.pub.publish(pose)


def main():
    rclpy.init()
    node = MarvelmindPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
