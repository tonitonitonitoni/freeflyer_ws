#!/usr/bin/env python3
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


class BangBangCircleController(Node):
    def __init__(self):
        super().__init__("bang_bang_circle_controller")

        self.declare_parameter("r0", 1.5)
        self.declare_parameter("v_tan_des", 0.3)
        self.declare_parameter("k_r", 1.0)
        self.declare_parameter("k_dr", 4.0)
        self.declare_parameter("k_t", 8.0)
        self.declare_parameter("radial_error_sat", 0.5)     # [m]
        self.declare_parameter("radial_vel_sat", 1.0)       # [m/s]
        self.declare_parameter("capture_radius", 0.5)       # [m] outside r0
        self.declare_parameter("force_hysteresis", 0.05)    # [N]
        self.declare_parameter("force_off_threshold", 0.02) # [N]

        # Bang-bang conversion params.
        self.declare_parameter("thruster_force", 1.0)   # [N], per axis thruster max
        self.declare_parameter("pwm_period", 0.01)       # [s]
        self.declare_parameter("deadband_force", 0.05)  # [N]

        self.r0 = float(self.get_parameter("r0").value)
        self.v_tan_des = float(self.get_parameter("v_tan_des").value)
        self.k_r = float(self.get_parameter("k_r").value)
        self.k_dr = float(self.get_parameter("k_dr").value)
        self.k_t = float(self.get_parameter("k_t").value)

        self.thruster_force = max(1e-6, float(self.get_parameter("thruster_force").value))
        self.pwm_period = max(1e-3, float(self.get_parameter("pwm_period").value))
        self.deadband_force = max(0.0, float(self.get_parameter("deadband_force").value))
        self.radial_error_sat = float(self.get_parameter("radial_error_sat").value)
        self.radial_vel_sat   = float(self.get_parameter("radial_vel_sat").value)
        self.capture_radius   = float(self.get_parameter("capture_radius").value)
        self.force_hyst_on    = float(self.get_parameter("force_hysteresis").value)
        self.force_hyst_off   = float(self.get_parameter("force_off_threshold").value)

        self.state_ready = False

        self.target_x = 0.0
        self.target_y = 0.0
        self.velocity_threshold = 0.2
        self.fx_cmd = 0.0
        self.fy_cmd = 0.0
        self.current_rx = 0.0
        self.current_ry = 0.0
        self.current_r  = 0.0
        self.current_v_rad = 0.0
        self.current_v_tan = 0.0
        self.current_r_hat_x = 1.0
        self.current_r_hat_y = 0.0
        self.current_t_hat_x = 0.0
        self.current_t_hat_y = 1.0
        self.current_cos_yaw = 1.0
        self.current_sin_yaw = 0.0


        self.create_subscription(Odometry, "/freeflyer/odometry/filtered", self.odom_cb, 10)

        self.radial_err_pub = self.create_publisher(Float64, "/orbit/radial_error", 10)
        
        self.thrusters = {
            "px": self.create_publisher(Bool, "/freeflyer/thrusters/px", 10),
            "nx": self.create_publisher(Bool, "/freeflyer/thrusters/nx", 10),
            "py": self.create_publisher(Bool, "/freeflyer/thrusters/py", 10),
            "ny": self.create_publisher(Bool, "/freeflyer/thrusters/ny", 10),
        }
        self.last_px = False
        self.last_nx = False
        self.last_py = False
        self.last_ny = False

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
            # Initialize a valid local frame at the orbit center so control can start from rest.
            c = math.cos(yaw)
            s = math.sin(yaw)
            self.current_rx = 0.0
            self.current_ry = 0.0
            self.current_r = 0.0
            self.current_v_rad = vx * c + vy * s
            self.current_v_tan = -vx * s + vy * c
            self.current_r_hat_x = c
            self.current_r_hat_y = s
            self.current_t_hat_x = -s
            self.current_t_hat_y = c
            self.current_cos_yaw = c
            self.current_sin_yaw = s
            self.state_ready = True

            msg_err = Float64()
            msg_err.data = -self.r0
            self.radial_err_pub.publish(msg_err)
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

        c = math.cos(yaw)
        s = math.sin(yaw)

        self.current_rx = rx
        self.current_ry = ry
        self.current_r  = r
        self.current_v_rad = v_rad
        self.current_v_tan = v_tan
        self.current_r_hat_x = r_hat_x
        self.current_r_hat_y = r_hat_y
        self.current_t_hat_x = t_hat_x
        self.current_t_hat_y = t_hat_y
        self.current_cos_yaw = c
        self.current_sin_yaw = s

        self.state_ready = True

    def control_tick(self):

        if not self.state_ready:
            return

        # --- Orbit state ---
        r = self.current_r
        v_rad = self.current_v_rad
        v_tan = self.current_v_tan

        radial_error = r - self.r0

        # --- Sliding surface parameters ---
        lam = 2.0
        eps_r = 0.2     # radial boundary layer
        eps_t = 0.1     # tangential boundary layer

        def sat(x):
            return max(-1.0, min(1.0, x))

        # --- Continuous sliding surface radial force ---
        s_r = v_rad + lam * radial_error
        f_r = -self.thruster_force * sat(s_r / eps_r)

        # --- Capture mode ---
        capture_mode = abs(radial_error) > self.capture_radius or abs(v_rad) > self.velocity_threshold


        if capture_mode:
            f_t = 0.0
        else:
            v_err = self.v_tan_des - v_tan
            f_t = self.thruster_force * sat(v_err / eps_t)

        # --- Convert (f_r, f_t) to WORLD frame ---
        r_hat_x = self.current_r_hat_x
        r_hat_y = self.current_r_hat_y
        t_hat_x = self.current_t_hat_x
        t_hat_y = self.current_t_hat_y

        fx_w = f_r * r_hat_x + f_t * t_hat_x
        fy_w = f_r * r_hat_y + f_t * t_hat_y

        # --- Rotate WORLD -> BODY frame ---
        c = self.current_cos_yaw
        s = self.current_sin_yaw

        fx_b = c * fx_w + s * fy_w
        fy_b = -s * fx_w + c * fy_w

        # --- PWM duty from continuous force ---
        duty_x = min(1.0, abs(fx_b) / self.thruster_force)
        duty_y = min(1.0, abs(fy_b) / self.thruster_force)

        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9
        phase = t % self.pwm_period

        x_active = duty_x > 0.01 and phase < duty_x * self.pwm_period
        y_active = duty_y > 0.01 and phase < duty_y * self.pwm_period

        # --- Hysteresis ---
        def hysteresis(active, last, force):
            if last:
                return active and abs(force) > self.force_hyst_off
            else:
                return active and abs(force) > self.force_hyst_on


        px = hysteresis(x_active and fx_b > 0.0, self.last_px, fx_b)
        nx = hysteresis(x_active and fx_b < 0.0, self.last_nx, fx_b)
        py = hysteresis(y_active and fy_b > 0.0, self.last_py, fy_b)
        ny = hysteresis(y_active and fy_b < 0.0, self.last_ny, fy_b)

        self.publish_thrusters(px, nx, py, ny)

        self.last_px = px
        self.last_nx = nx
        self.last_py = py
        self.last_ny = ny


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
