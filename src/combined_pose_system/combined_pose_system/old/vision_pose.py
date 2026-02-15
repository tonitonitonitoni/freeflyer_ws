#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float32 
from sensor_msgs.msg import CompressedImage 
from geometry_msgs.msg import PoseWithCovarianceStamped 
from ament_index_python.packages import get_package_share_directory

import json, os, math
from combined_pose_system.utils.utils_ros import *

pkg_dir = get_package_share_directory("combined_pose_system")
cluster_path = os.path.join(pkg_dir, "config", "feb6_cluster_data.json")

# Launch with `ros2 run combined_pose_system vision_pose`
from combined_pose_system.utils.utils_pose import estimate_planar_pose, wrap_angle, confidence_from_sigma, yaw_to_quat, covariance_from_sigma, compute_planar_sigma

class StarfieldPoseNode(Node):

    def __init__(self):
        super().__init__('starfield_pose_node')

        self.frame_id = 'vision_map'

        if not os.path.exists(cluster_path):
            raise RuntimeError(f"Cluster data file not found: {cluster_path}")

        with open(cluster_path, 'r') as f:
            self.cluster_locs = json.load(f)

        # ROS interfaces
        self.sub = self.create_subscription(
            CompressedImage,
            'pose_cam/image/compressed',
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'vision/pose_raw',
            10
        )

        self.prev_yaw = None
        self.prev_stamp = None

        self.max_yaw_rate = math.radians(90.0)  # rad/s flip guard

        # Diagnostics
        self.frames_rx = 0
        self.frames_ok = 0
        self.frames_rejected = 0

        self.conf_pub = self.create_publisher(
            Float32,
            'vision_pose/confidence',
            10
        )

        self.get_logger().info("Starfield pose node initialized")

    def image_callback(self, msg: Image):
        self.frames_rx += 1
        try:
            img = ros_image_to_numpy(msg)
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}")
            return

        result = estimate_planar_pose(img, self.cluster_locs)

        if result is None:
            self.get_logger().debug("No valid pose detected")
            return

        x_cm, y_cm, yaw, err = result

        self.frames_ok += 1

        stamp = msg.header.stamp
        t_sec = stamp.sec + stamp.nanosec * 1e-9

        # Flip guard
        if self.prev_yaw is not None and self.prev_stamp is not None:
            dt = max(1e-3, t_sec - self.prev_stamp)
            dyaw = wrap_angle(yaw - self.prev_yaw)
            yaw_rate = abs(dyaw) / dt

            if yaw_rate > self.max_yaw_rate:
                self.frames_rejected += 1
                self.get_logger().warn(
                    f"Yaw flip rejected: rate={math.degrees(yaw_rate):.1f} deg/s"
                )
                return

        # Build PoseWithCovarianceStamped
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id

        pose.pose.pose.position.x = x_cm * 0.01
        pose.pose.pose.position.y = y_cm * 0.01
        pose.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quat(yaw)
        pose.pose.pose.orientation.x = qx
        pose.pose.pose.orientation.y = qy
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw

        sigma = compute_planar_sigma(err)
        pose.pose.covariance = covariance_from_sigma(sigma)

        self.pub.publish(pose)

        # Confidence (simple inverse error metric)
        conf = Float32()
        conf.data = confidence_from_sigma(sigma)
        self.conf_pub.publish(conf)

        self.prev_yaw = yaw
        self.prev_stamp = t_sec

def main():
    rclpy.init()
    node = StarfieldPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()