#!/usr/bin/env python3
import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class ThrusterState:
    px: float = 0.0
    nx: float = 0.0
    py: float = 0.0
    ny: float = 0.0


@dataclass
class ReactionWheelCmd:
    cw: bool = False     # -tau
    ccw: bool = False    # +tau


class ThrusterOdomPredictor(Node):
    """
    Predicts planar motion from bang-bang thrusters (Fx,Fy) and yaw from either:
      A) IMU gyro z (recommended), or
      B) Reaction wheel torque model (useful in sim/testing).

    Publishes nav_msgs/Odometry on /freeflyer/thruster_odom.
    """

    def __init__(self):
        super().__init__("thruster_odom_predictor")

        # ---- Parameters ----
        self.declare_parameter("mass", 15.0)                 # kg
        self.declare_parameter("max_thrust", 2.0)            # N per thruster when ON
        self.declare_parameter("linear_drag", 0.0)           # N*s/m (viscous drag)
        self.declare_parameter("update_rate", 200.0)         # Hz integration

        # Reaction wheel model (only used if use_imu_yaw_rate=False)
        self.declare_parameter("use_imu_yaw_rate", True)
        self.declare_parameter("I_body", 0.625)              # kg*m^2 (approx from your URDF)
        self.declare_parameter("rw_torque", 0.05)            # N*m bang-bang torque magnitude
        self.declare_parameter("rw_damping", 0.0)            # N*m*s (optional viscous damping)

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", False)

        self.m = float(self.get_parameter("mass").value)
        self.Fmax = float(self.get_parameter("max_thrust").value)
        self.c_lin = float(self.get_parameter("linear_drag").value)
        self.rate = float(self.get_parameter("update_rate").value)

        self.use_imu_yaw_rate = bool(self.get_parameter("use_imu_yaw_rate").value)
        self.Ib = float(self.get_parameter("I_body").value)
        self.tau_rw = float(self.get_parameter("rw_torque").value)
        self.c_rot = float(self.get_parameter("rw_damping").value)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        if self.m <= 0.0:
            raise ValueError("mass must be > 0")
        if self.rate <= 0.0:
            raise ValueError("update_rate must be > 0")
        if self.Ib <= 0.0:
            raise ValueError("I_body must be > 0")
        if self.tau_rw < 0.0:
            raise ValueError("rw_torque must be >= 0")

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

        # IMU yaw rate input
        self.yaw_rate_imu = 0.0
        self.have_imu = False

        # Commands
        self.th = ThrusterState()
        self.rw = ReactionWheelCmd()

        # ---- Publishers ----
        self.odom_pub = self.create_publisher(Odometry, "/freeflyer/thruster_odom", 10)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None

        # ---- Subscribers (thrusters) ----
        self.create_subscription(Float32, "/freeflyer/thrusters/px", self._cb_px, 10)
        self.create_subscription(Float32, "/freeflyer/thrusters/nx", self._cb_nx, 10)
        self.create_subscription(Float32, "/freeflyer/thrusters/py", self._cb_py, 10)
        self.create_subscription(Float32, "/freeflyer/thrusters/ny", self._cb_ny, 10)

        # ---- Subscribers (reaction wheel bang-bang torque commands) ----
        self.create_subscription(Bool, "/freeflyer/reaction_wheel/cw", self._cb_rw_cw, 10)
        self.create_subscription(Bool, "/freeflyer/reaction_wheel/ccw", self._cb_rw_ccw, 10)

        # ---- Subscriber (IMU) ----
        self.create_subscription(Imu, "/imu/data_raw", self._cb_imu, 50)

        # ---- Timer integration loop ----
        self.last_t = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate, self._step)

        self.get_logger().info(
            f"Thruster odom predictor running. m={self.m}kg Fmax={self.Fmax}N "
            f"drag={self.c_lin} rate={self.rate}Hz "
            f"use_imu_yaw_rate={self.use_imu_yaw_rate} I_body={self.Ib} rw_torque={self.tau_rw} "
            f"publish_tf={self.publish_tf}"
        )

    # ---------- Thruster callbacks ----------
    def _cb_px(self, msg: Float32): self.th.px = max(0.0, min(1.0, float(msg.data)))
    def _cb_nx(self, msg: Float32): self.th.nx = max(0.0, min(1.0, float(msg.data)))
    def _cb_py(self, msg: Float32): self.th.py = max(0.0, min(1.0, float(msg.data)))
    def _cb_ny(self, msg: Float32): self.th.ny = max(0.0, min(1.0, float(msg.data)))

    # ---------- Reaction wheel callbacks ----------
    def _cb_rw_cw(self, msg: Bool): self.rw.cw = bool(msg.data)
    def _cb_rw_ccw(self, msg: Bool): self.rw.ccw = bool(msg.data)

    # ---------- IMU callback ----------
    def _cb_imu(self, msg: Imu):
        self.yaw_rate_imu = float(msg.angular_velocity.z)
        self.have_imu = True

    def _reaction_wheel_torque(self) -> float:
        # Bang-bang torque. If both pressed, cancel to 0 (safe default).
        if self.rw.cw and not self.rw.ccw:
            return -self.tau_rw
        if self.rw.ccw and not self.rw.cw:
            return +self.tau_rw
        return 0.0

    # ---------- Main integration step ----------
    def _step(self):
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        dt = min(dt, 0.05)  # clamp pauses
        self.last_t = now

        # 1) Yaw propagation
        if self.use_imu_yaw_rate:
            if self.have_imu:
                yr = max(min(self.yaw_rate_imu, 10.0), -10.0)  # clamp spikes
                self.omega = yr
                self.yaw = wrap_to_pi(self.yaw + self.omega * dt)
        else:
            # Reaction wheel rigid-body yaw dynamics:
            #   I * domega = tau - c_rot * omega
            tau = self._reaction_wheel_torque()
            alpha = (tau - self.c_rot * self.omega) / self.Ib
            self.omega += alpha * dt
            self.yaw = wrap_to_pi(self.yaw + self.omega * dt)

        # 2) Body-frame force from continuous one-sided thrusters
        fx_b = self.Fmax * (self.th.px - self.th.nx)
        fy_b = self.Fmax * (self.th.py - self.th.ny)

        # 3) Rotate to world/odom frame
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        fx_w = cy * fx_b - sy * fy_b
        fy_w = sy * fx_b + cy * fy_b

        # 4) Acceleration with optional viscous drag
        ax = fx_w / self.m - (self.c_lin / self.m) * self.vx
        ay = fy_w / self.m - (self.c_lin / self.m) * self.vy

        # 5) Semi-implicit Euler
        self.vx += ax * dt
        self.vy += ay * dt
        self.x += self.vx * dt
        self.y += self.vy * dt

        # 6) Covariance growth (simple, “be honest about drift” model)
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
    node = ThrusterOdomPredictor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
