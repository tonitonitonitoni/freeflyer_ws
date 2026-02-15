#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float32 
from sensor_msgs.msg import CompressedImage 
from ament_index_python.packages import get_package_share_directory

import json, os, math
from combined_pose_system.utils.utils_ros import compressed_image_to_numpy

pkg_dir = get_package_share_directory("combined_pose_system")
cluster_path = os.path.join(pkg_dir, "config", "feb6_cluster_data.json")

# Launch with `ros2 run combined_pose_system vision_pose`
from combined_pose_system.utils.utils_pose import estimate_planar_yaw, wrap_angle, confidence_from_yaw_error
from std_msgs.msg import Float32



class VisionYawNode(Node):

    def __init__(self):
        super().__init__('vision_yaw_node')

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

        self.yaw_pub = self.create_publisher(
            Float32,
            'vision/yaw',
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

    def image_callback(self, msg: CompressedImage):
        self.frames_rx += 1
        try:
            img = compressed_image_to_numpy(msg)
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}")
            return

        result = estimate_planar_yaw(img, self.cluster_locs)

        if result is None:
            self.get_logger().debug("No valid pose detected")
            return

        yaw, err = result

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

        msg_yaw = Float32()
        msg_yaw.data = yaw
        self.yaw_pub.publish(msg_yaw)

        # Confidence (Gaussian error metric)
        conf = Float32()
        conf.data = confidence_from_yaw_error(err)
        self.conf_pub.publish(conf)

        self.prev_yaw = yaw
        self.prev_stamp = t_sec

def main():
    rclpy.init()
    node = VisionYawNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()