

#!/usr/bin/env python3

import json, math, os
import numpy as np

import rclpy  #type: ignore
from rclpy.node import Node  #type: ignore

from geometry_msgs.msg import PoseWithCovarianceStamped  #type: ignore
from combined_pose_system.utils.utils_ros import yaw_from_quat, yaw_to_quat
from ament_index_python.packages import get_package_share_directory

pkg_dir = get_package_share_directory("combined_pose_system")
transform_path = os.path.join(pkg_dir, "config", "vision_to_mm_transform.json")

class VisionToMMTransformNode(Node):
    """
    Apply a fixed SE(2) transform from vision_map to mm_map.

    vision_map -> mm_map:
        [x_mm] = R * [x_v] + t
        [y_mm]       [y_v]

        yaw_mm = yaw_v + yaw_offset
    """

    def __init__(self):
        super().__init__('vision_mm_transform_node')

        # Path to transform JSON (hardcoded by design)
        # self.transform_path = 'vision_to_mm_transform.json'

        with open(transform_path, 'r') as f:
            tf = json.load(f)

        self.yaw_offset = tf['yaw_rad']
        self.tx = tf['tx']
        self.ty = tf['ty']

        self.cos_y = math.cos(self.yaw_offset)
        self.sin_y = math.sin(self.yaw_offset)

        self.get_logger().info(
            f"Loaded vision→mm transform: "
            f"yaw={math.degrees(self.yaw_offset):.2f} deg, "
            f"tx={self.tx:.3f}, ty={self.ty:.3f}"
        )

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'vision/pose_raw',
            self.cb_vision,
            10
        )

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'vision/pose_mm',
            10
        )

    def cb_vision(self, msg: PoseWithCovarianceStamped):
        # Extract vision pose
        xv = msg.pose.pose.position.x
        yv = msg.pose.pose.position.y
        yaw_v = yaw_from_quat(msg.pose.pose.orientation)

        # Transform position
        x_mm = self.cos_y * xv - self.sin_y * yv + self.tx
        y_mm = self.sin_y * xv + self.cos_y * yv + self.ty

        # Transform yaw
        yaw_mm = yaw_v + self.yaw_offset
        yaw_mm = (yaw_mm + math.pi) % (2.0 * math.pi) - math.pi

        qx, qy, qz, qw = yaw_to_quat(yaw_mm)

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'mm_map'

        out.pose.pose.position.x = x_mm
        out.pose.pose.position.y = y_mm
        out.pose.pose.position.z = 0.0

        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw

        # Rotate planar covariance
        cov_in = np.array(msg.pose.covariance).reshape(6, 6)

        R = np.array([
            [ self.cos_y, -self.sin_y ],
            [ self.sin_y,  self.cos_y ]
        ])

        cov_xy = cov_in[0:2, 0:2]
        cov_xy_rot = R @ cov_xy @ R.T

        cov_out = np.zeros((6, 6))
        cov_out[0:2, 0:2] = cov_xy_rot
        cov_out[5, 5] = cov_in[5, 5]  # yaw variance unchanged

        out.pose.covariance = cov_out.flatten().tolist()

        self.pub.publish(out)


def main():
    rclpy.init()
    node = VisionToMMTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()