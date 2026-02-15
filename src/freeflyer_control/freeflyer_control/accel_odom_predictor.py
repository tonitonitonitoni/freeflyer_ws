#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class AccelOdomPredictor(Node):
    """
    Predicts planar motion from IMU acceleration and yaw rate.

    Publishes nav_msgs/Odometry on /freeflyer/thruster_odom.
    """

    def __init__(self):
        super().__init__("thruster_odom_predictor")

        # ---- Parameters ----
        self.declare_parameter("update_rate", 200.0)         # Hz integration

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", False)

        self.rate = float(self.get_parameter("update_rate").value)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        if self.rate <= 0.0:
            raise ValueError("update_rate must be > 0")

        # ---- Internal state (odom/world frame) ----
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0

        # Angular state (used in RW model mode; in IMU mode we just mirror IMU)
        self.omega = 0.0  # yaw rate

        # Growing covariances for dead reckoning
        self.Px = 0.05
        self.Py = 0.05
        self.Pyaw = 0.1

        # IMU inputs
        self.yaw_rate_imu = 0.0
        self.ax_b_imu = 0.0
        self.ay_b_imu = 0.0
        self.have_imu = False

        # ---- Publishers ----
        self.odom_pub = self.create_publisher(Odometry, "/freeflyer/thruster_odom", 10)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None

        # ---- Subscriber (IMU) ----
        self.create_subscription(Imu, "/imu/data_raw", self._cb_imu, 50)

        # ---- Timer integration loop ----
        self.last_t = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate, self._step)

        self.get_logger().info(
            f"Accel odom predictor running. rate={self.rate}Hz "
            f"publish_tf={self.publish_tf}"
        )

    # ---------- IMU callback ----------
    def _cb_imu(self, msg: Imu):
        self.yaw_rate_imu = float(msg.angular_velocity.z)
        self.ax_b_imu = float(msg.linear_acceleration.x)
        self.ay_b_imu = float(msg.linear_acceleration.y)
        self.have_imu = True

    # ---------- Main integration step ----------
    def _step(self):
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        dt = min(dt, 0.05)  # clamp pauses
        self.last_t = now

        if not self.have_imu:
            return

        # 1) Yaw propagation from IMU
        yr = max(min(self.yaw_rate_imu, 10.0), -10.0)  # clamp spikes
        self.omega = yr
        self.yaw = wrap_to_pi(self.yaw + self.omega * dt)

        # 2) Rotate body-frame IMU linear acceleration to world/odom frame
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        ax = cy * self.ax_b_imu - sy * self.ay_b_imu
        ay = sy * self.ax_b_imu + cy * self.ay_b_imu

        # 3) Semi-implicit Euler
        self.vx += ax * dt
        self.vy += ay * dt
        self.x += self.vx * dt
        self.y += self.vy * dt

        # 4) Covariance growth (simple, “be honest about drift” model)
        self.Px += 0.02 * dt
        self.Py += 0.02 * dt
        self.Pyaw += 0.01 * dt

        self._publish(now)

    def _publish(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # yaw -> quaternion
        half = 0.5 * self.yaw
        odom.pose.pose.orientation.w = math.cos(half)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(half)

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.omega

        # Pose covariance
        odom.pose.covariance[0] = self.Px
        odom.pose.covariance[7] = self.Py
        odom.pose.covariance[35] = self.Pyaw

        # Twist covariance (still “meh” — EKF fuses anyway)
        odom.twist.covariance[0] = 0.1
        odom.twist.covariance[7] = 0.1
        odom.twist.covariance[35] = 1e-3

        self.odom_pub.publish(odom)

        if self.publish_tf and self.tf_br is not None:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_br.sendTransform(t)


def main():
    rclpy.init()
    node = AccelOdomPredictor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
