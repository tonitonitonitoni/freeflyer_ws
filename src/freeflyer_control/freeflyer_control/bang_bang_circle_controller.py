#!/usr/bin/env python3
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


class BangBangCircleController(Node):
    def __init__(self):
        super().__init__("bang_bang_circle_controller")

        # Orbit guidance params (same structure as continuous controller).
        self.declare_parameter("r0", 1.5)
        self.declare_parameter("v_tan_des", 0.3)
        self.declare_parameter("k_r", 10.0)
        self.declare_parameter("k_dr", 4.0)
        self.declare_parameter("k_t", 8.0)

        # Bang-bang conversion params.
        self.declare_parameter("thruster_force", 2.0)   # [N], per axis thruster max
        self.declare_parameter("pwm_period", 0.2)       # [s]
        self.declare_parameter("deadband_force", 0.05)  # [N]

        self.r0 = float(self.get_parameter("r0").value)
        self.v_tan_des = float(self.get_parameter("v_tan_des").value)
        self.k_r = float(self.get_parameter("k_r").value)
        self.k_dr = float(self.get_parameter("k_dr").value)
        self.k_t = float(self.get_parameter("k_t").value)

        self.thruster_force = max(1e-6, float(self.get_parameter("thruster_force").value))
        self.pwm_period = max(1e-3, float(self.get_parameter("pwm_period").value))
        self.deadband_force = max(0.0, float(self.get_parameter("deadband_force").value))

        self.target_x = 0.0
        self.target_y = 0.0

        self.fx_cmd = 0.0
        self.fy_cmd = 0.0

        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)

        self.radial_err_pub = self.create_publisher(Float64, "/orbit/radial_error", 10)
        self.thrusters = {
            "px": self.create_publisher(Bool, "/freeflyer/thrusters/px", 10),
            "nx": self.create_publisher(Bool, "/freeflyer/thrusters/nx", 10),
            "py": self.create_publisher(Bool, "/freeflyer/thrusters/py", 10),
            "ny": self.create_publisher(Bool, "/freeflyer/thrusters/ny", 10),
        }

        self.t0 = self.get_clock().now()
        self.create_timer(0.02, self.control_tick)  # 50 Hz

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        rx = x - self.target_x
        ry = y - self.target_y
        r = math.hypot(rx, ry)
        if r < 1e-3:
            return

        radial_error = r - self.r0
        msg_err = Float64()
        msg_err.data = radial_error
        self.radial_err_pub.publish(msg_err)

        r_hat_x = rx / r
        r_hat_y = ry / r
        t_hat_x = -r_hat_y
        t_hat_y = r_hat_x

        v_rad = vx * r_hat_x + vy * r_hat_y
        v_tan = vx * t_hat_x + vy * t_hat_y

        f_r = -self.k_r * radial_error - self.k_dr * v_rad
        f_t = self.k_t * (self.v_tan_des - v_tan)

        fx_w = f_r * r_hat_x + f_t * t_hat_x
        fy_w = f_r * r_hat_y + f_t * t_hat_y

        c = math.cos(yaw)
        s = math.sin(yaw)
        self.fx_cmd = c * fx_w + s * fy_w
        self.fy_cmd = -s * fx_w + c * fy_w

    def control_tick(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9
        phase = t % self.pwm_period

        duty_x = min(1.0, abs(self.fx_cmd) / self.thruster_force)
        duty_y = min(1.0, abs(self.fy_cmd) / self.thruster_force)

        x_active = abs(self.fx_cmd) >= self.deadband_force and phase < duty_x * self.pwm_period
        y_active = abs(self.fy_cmd) >= self.deadband_force and phase < duty_y * self.pwm_period

        px = x_active and self.fx_cmd > 0.0
        nx = x_active and self.fx_cmd < 0.0
        py = y_active and self.fy_cmd > 0.0
        ny = y_active and self.fy_cmd < 0.0

        self.publish_thrusters(px, nx, py, ny)

    def publish_thrusters(self, px: bool, nx: bool, py: bool, ny: bool):
        msg = Bool()
        msg.data = px
        self.thrusters["px"].publish(msg)

        msg = Bool()
        msg.data = nx
        self.thrusters["nx"].publish(msg)

        msg = Bool()
        msg.data = py
        self.thrusters["py"].publish(msg)

        msg = Bool()
        msg.data = ny
        self.thrusters["ny"].publish(msg)


def main():
    rclpy.init()
    node = BangBangCircleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.publish_thrusters(False, False, False, False)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
