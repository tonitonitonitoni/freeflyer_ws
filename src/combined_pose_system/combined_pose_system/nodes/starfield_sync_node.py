#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32

from message_filters import Subscriber, ApproximateTimeSynchronizer

import numpy as np
import os


class StarfieldSyncNode(Node):
    """
    Synchronize pose_cam images, Marvelmind position, and vision yaw.
    Produces an offline-ready pose log for starfield map building.
    """

    def __init__(self):
        super().__init__('starfield_sync_node')

        # ---- subscribers ----
        self.image_sub = Subscriber(
            self, CompressedImage,
            'pose_cam/image/compressed'
        )

        self.mm_pose_sub = Subscriber(
            self, PoseWithCovarianceStamped,
            'marvelmind/pose'
        )

        self.yaw_sub = Subscriber(
            self, Float32,
            'vision/yaw'
        )

        # ---- synchronizer ----
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.mm_pose_sub, self.yaw_sub],
            queue_size=40,
            slop=0.05
        )
        self.sync.registerCallback(self.cb)

        # ---- output dirs ----
        self.img_dir = "logged_frames"
        self.pose_dir = "logged_poses"
        os.makedirs(self.img_dir, exist_ok=True)
        os.makedirs(self.pose_dir, exist_ok=True)

        self.idx = 0
        self.get_logger().info("Starfield sync node running (with vision yaw)")

    def cb(self, img_msg, mm_pose_msg, yaw_msg):
        # -----------------------
        # Save image
        # -----------------------
        img_path = f"{self.img_dir}/frame_{self.idx:06d}.jpg"
        with open(img_path, "wb") as f:
            f.write(img_msg.data)

        # -----------------------
        # Save pose
        # -----------------------
        p = mm_pose_msg.pose.pose.position

        cam_pos_w = np.array([p.x, p.y, p.z], dtype=float)
        yaw = float(yaw_msg.data)

        stamp = (
            img_msg.header.stamp.sec
            + 1e-9 * img_msg.header.stamp.nanosec
        )

        pose_path = f"{self.pose_dir}/pose_{self.idx:06d}.npz"
        np.savez(
            pose_path,
            cam_pos_w=cam_pos_w,
            yaw=yaw,
            stamp=stamp,
        )

        self.get_logger().info(
            f"Saved frame {self.idx}: pos=({p.x:.2f},{p.y:.2f}), yaw={np.degrees(yaw):.1f} deg"
        )

        self.idx += 1


def main():
    rclpy.init()
    node = StarfieldSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
