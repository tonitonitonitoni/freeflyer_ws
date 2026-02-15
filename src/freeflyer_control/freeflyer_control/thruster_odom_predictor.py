#!/usr/bin/env python3
import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def wrap_to_pi(a: float) -> float:
    # Wrap angle to [-pi, pi]
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class ThrusterState:
    px: bool = False
    nx: bool = False
    py: bool = False
    ny: bool = False


class ThrusterOdomPredictor(Node):
    def __init__(self):
        super().__init__("thruster_odom_predictor")

        # ---- Parameters ----
        self.declare_parameter("mass", 15.0)                 # kg
        self.declare_parameter("max_thrust", 2.0)            # N per thruster when ON
        self.declare_parameter("linear_drag", 0.0)           # N*s/m (0 = no drag)
        self.declare_parameter("update_rate", 200.0)         # Hz integration rate

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", False)

        self.m = float(self.get_parameter("mass").value)
        self.Fmax = float(self.get_parameter("max_thrust").value)
        self.c_lin = float(self.get_parameter("linear_drag").value)
        self.rate = float(self.get_parameter("update_rate").value)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        if self.m <= 0.0:
            raise ValueError("mass must be > 0")
        if self.rate <= 0.0:
            raise ValueError("update_rate must be > 0")

        # ---- Internal state (world/odom frame) ----
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0

        # covariances
        self.Px = 0.05
        self.Py = 0.05
        self.Pyaw = 0.1

        # From IMU (yaw rate)
        self.yaw_rate = 0.0
        self.have_imu = False

        # Thruster commands
        self.th = ThrusterState()

        # ---- Publishers ----
        self.odom_pub = self.create_publisher(Odometry, "/freeflyer/thruster_odom", 10)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None

        # ---- Subscribers (thrusters) ----
        self.create_subscription(Bool, "/freeflyer/thrusters/px", self._cb_px, 10)
        self.create_subscription(Bool, "/freeflyer/thrusters/nx", self._cb_nx, 10)
        self.create_subscription(Bool, "/freeflyer/thrusters/py", self._cb_py, 10)
        self.create_subscription(Bool, "/freeflyer/thrusters/ny", self._cb_ny, 10)

        # ---- Subscriber (IMU) ----
        # Uses yaw rate only (gyro z). This is your short-term heading propagation.
        self.create_subscription(Imu, "/imu/data_raw", self._cb_imu, 50)

        # ---- Timer integration loop ----
        self.last_t = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate, self._step)

        self.get_logger().info(
            f"Thruster odom predictor running. m={self.m}kg Fmax={self.Fmax}N "
            f"drag={self.c_lin} rate={self.rate}Hz publish_tf={self.publish_tf}"
        )

    # ---------- Thruster callbacks ----------
    def _cb_px(self, msg: Bool): self.th.px = bool(msg.data)
    def _cb_nx(self, msg: Bool): self.th.nx = bool(msg.data)
    def _cb_py(self, msg: Bool): self.th.py = bool(msg.data)
    def _cb_ny(self, msg: Bool): self.th.ny = bool(msg.data)

    # ---------- IMU callback ----------
    def _cb_imu(self, msg: Imu):
        # In ROS, +z is CCW yaw rate if frames are consistent (you already verified this)
        self.yaw_rate = float(msg.angular_velocity.z)
        self.have_imu = True

    # ---------- Main integration step ----------
    def _step(self):
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        # Cap dt to avoid huge jumps if the node pauses
        dt = min(dt, 0.05)
        self.last_t = now

        # 1) Integrate yaw from IMU (dead-reckoning)
        # If IMU not present yet, yaw stays constant.
        if self.have_imu:
            yr = max(min(self.yaw_rate, 10.0), -10.0)
            self.yaw = wrap_to_pi(self.yaw + yr * dt)

        # 2) Compute body-frame force from bang-bang thrusters
        # Fx_body = Fmax*(px - nx), Fy_body = Fmax*(py - ny)
        fx_b = self.Fmax * (1.0 if self.th.px else 0.0) - self.Fmax * (1.0 if self.th.nx else 0.0)
        fy_b = self.Fmax * (1.0 if self.th.py else 0.0) - self.Fmax * (1.0 if self.th.ny else 0.0)

        # 3) Rotate force into world/odom frame: Fw = R(yaw) * Fb
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        fx_w = cy * fx_b - sy * fy_b
        fy_w = sy * fx_b + cy * fy_b

        # 4) Acceleration with optional linear drag: a = F/m - (c/m)*v
        ax = fx_w / self.m - (self.c_lin / self.m) * self.vx
        ay = fy_w / self.m - (self.c_lin / self.m) * self.vy

        # 5) Semi-implicit Euler integration (stable enough for this)
        self.vx += ax * dt
        self.vy += ay * dt
        self.x += self.vx * dt
        self.y += self.vy * dt

        # 6) Dead-reckoning uncertainty grows roughly with time because small force errors accumulate
        process_pos = 0.02 * dt
        process_yaw = 0.01 * dt

        self.Px += process_pos
        self.Py += process_pos
        self.Pyaw += process_yaw

        # 6) Publish Odometry + optional TF
        self._publish(now)

    def _publish(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # yaw -> quaternion (z-axis only)
        half = 0.5 * self.yaw
        odom.pose.pose.orientation.w = math.cos(half)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(half)

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.yaw_rate if self.have_imu else 0.0

        # Reasonable-ish covariances for dead-reckoning (tune later / EKF will override)
        # Pose covariance: x,y,yaw somewhat uncertain, grows in reality (not modeled here)
        odom.pose.covariance[0] = self.Px
        odom.pose.covariance[7] = self.Py
        odom.pose.covariance[35] = self.Pyaw

        # Twist covariance: trust velocity moderately, yaw rate from IMU
        odom.twist.covariance[0] = 0.1    # vx
        odom.twist.covariance[7] = 0.1    # vy
        odom.twist.covariance[35] = 1e-3  # yaw_rate

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
    node = ThrusterOdomPredictor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
