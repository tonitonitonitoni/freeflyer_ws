#!/usr/bin/env python3

import numpy as np
import rclpy #type: ignore
from rclpy.node import Node #type: ignore
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped #type: ignore
import sys

from combined_pose_system.utils.utils_ros import wrap_angle, yaw_from_quat

class PoseToTwistNode(Node):
    """
    Generic planar pose → twist adapter.

    Computes velocity by finite differencing:
      vx = dx/dt
      vy = dy/dt
      wz = dyaw/dt

    Includes:
      - timestamp guards
      - yaw wrapping
      - exponential smoothing
    """

    def __init__(
        self,
        node_name: str,
        pose_topic: str,
        twist_topic: str,
        alpha: float = 0.4,
    ):
        super().__init__(node_name)

        self.alpha = alpha

        self.prev = None  # (t, x, y, yaw)
        self.v_prev = np.zeros(3)

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            pose_topic,
            self.cb_pose,
            10
        )

        self.pub = self.create_publisher(
            TwistWithCovarianceStamped,
            twist_topic,
            10
        )

        self.get_logger().info(
            f"Pose→Twist adapter started: {pose_topic} → {twist_topic} "
            f"(alpha={alpha})"
        )

    def stamp_to_sec(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def cb_pose(self, msg: PoseWithCovarianceStamped):
        t = self.stamp_to_sec(msg.header.stamp)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)

        if self.prev is None:
            self.prev = (t, x, y, yaw)
            return

        t_prev, x_prev, y_prev, yaw_prev = self.prev
        dt = t - t_prev

        if dt <= 1e-4:
            return

        dx = x - x_prev
        dy = y - y_prev
        dyaw = wrap_angle(yaw - yaw_prev)

        v = np.array([
            dx / dt,
            dy / dt,
            dyaw / dt
        ])

        # Exponential smoothing
        v_filt = self.alpha * v + (1.0 - self.alpha) * self.v_prev
        self.v_prev = v_filt

        out = TwistWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id

        out.twist.twist.linear.x = v_filt[0]
        out.twist.twist.linear.y = v_filt[1]
        out.twist.twist.linear.z = 0.0

        out.twist.twist.angular.x = 0.0
        out.twist.twist.angular.y = 0.0
        out.twist.twist.angular.z = v_filt[2]

        # Conservative covariance (tunable later)
        cov = [0.0] * 36
        cov[0] = 0.2 ** 2    # vx
        cov[7] = 0.2 ** 2    # vy
        cov[35] = 0.3 ** 2   # yaw rate

        out.twist.covariance = cov

        self.pub.publish(out)

        self.prev = (t, x, y, yaw)


def main():
    """
    Usage:
      ros2 run starfieldProject pose_twist_node vision
      ros2 run starfieldProject pose_twist_node mm
    """

    rclpy.init()

    if len(sys.argv) < 2:
        print("Usage: pose_twist_node.py [vision|mm]")
        return

    mode = sys.argv[1]

    if mode == 'vision':
        node = PoseToTwistNode(
            node_name='vision_pose_to_twist',
            pose_topic='vision/pose_mm',
            twist_topic='/vision_twist',
            alpha=0.3
        )
    elif mode == 'mm':
        node = PoseToTwistNode(
            node_name='mm_pose_to_twist',
            pose_topic='/marvelmind/pose',
            twist_topic='/marvelmind/twist',
            alpha=0.6
        )
    else:
        raise RuntimeError("Mode must be 'vision' or 'mm'")

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
