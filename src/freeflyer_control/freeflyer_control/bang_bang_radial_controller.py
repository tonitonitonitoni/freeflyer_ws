#!/usr/bin/env python3
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

class params:
    # Geometry / targets
    r0 = 1.5
    target_x = 0.0
    target_y = 0.0
    min_orbit_radius = 1e-3

    # Node timing
    control_period = 0.01

    # Sliding surfaces (body XY)
    lambda_bx = 0.9
    lambda_by = 0.9
    boundary_layer_bx = 0.11
    boundary_layer_by = 0.11
    gain_bx = 1.0
    gain_by = 1.0

    # Safety / shaping
    min_thruster_force = 1e-6
    max_axis_duty = 0.25
    max_axis_duty_with_tan = 0.65
    min_pulse_width = 0.02
    min_pulse_gap = 0.02
    impulse_leak = 0.995
    max_impulse_pulses = 3.0
    radial_settle_error = 0.06
    radial_settle_vrad = 0.05
    near_band_error = 0.18
    near_kp_scale = 0.28
    near_kd_scale = 1.3

    # Tangential controller
    tangential_gate_err_on = 0.10
    tangential_gate_err_off = 0.20
    tangential_gate_vrad_on = 0.10
    tangential_gate_vrad_off = 0.16
    tangential_boundary_layer = 0.10
    tangential_gain_scale = 0.70
    tangential_force_limit_scale = 0.55
    centripetal_ff_gain = 0.90


def sat(x: float) -> float:
    if x > 1.0:
        return 1.0
    if x < -1.0:
        return -1.0
    return x


class BangBangRadialController(Node):
    def __init__(self):
        super().__init__("bang_bang_radial_controller")
        self.declare_parameter("v_tan_des", 0.3)
        self.declare_parameter("thruster_force", 2.0)
        self.declare_parameter("mass", 15.0)
        self.v_tan_des = float(self.get_parameter("v_tan_des").value)
        self.mass = float(self.get_parameter("mass").value)
        self.thruster_force = max(
            params.min_thruster_force, float(self.get_parameter("thruster_force").value)
        )

        self.state_ready = False
        self.current_r = 0.0
        self.current_radial_error = 0.0
        self.current_v_rad = 0.0
        self.current_v_tan = 0.0
        self.current_r_hat_x = 1.0
        self.current_r_hat_y = 0.0
        self.current_t_hat_x = 0.0
        self.current_t_hat_y = 1.0
        self.current_cos_yaw = 1.0
        self.current_sin_yaw = 0.0
        self.tan_enabled = False

        self.imp_x = 0.0
        self.imp_y = 0.0
        self.pulse_x_time = 0.0
        self.pulse_y_time = 0.0
        self.pulse_x_gap_time = 0.0
        self.pulse_y_gap_time = 0.0
        self.pulse_x_dir = 0
        self.pulse_y_dir = 0

        self.create_subscription(Odometry, "/freeflyer/odometry/filtered", self.odom_cb, 10)
        self.radial_err_pub = self.create_publisher(Float64, "/orbit/radial_error", 10)
        self.sx_pub = self.create_publisher(Float64, "/orbit/switch_surface_x", 10)
        self.sy_pub = self.create_publisher(Float64, "/orbit/switch_surface_y", 10)
        self.thrusters = {
            "px": self.create_publisher(Bool, "/freeflyer/thrusters/px", 10),
            "nx": self.create_publisher(Bool, "/freeflyer/thrusters/nx", 10),
            "py": self.create_publisher(Bool, "/freeflyer/thrusters/py", 10),
            "ny": self.create_publisher(Bool, "/freeflyer/thrusters/ny", 10),
        }

        self.create_timer(params.control_period, self.control_tick)

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx_b = msg.twist.twist.linear.x
        vy_b = msg.twist.twist.linear.y

        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        c = math.cos(yaw)
        s = math.sin(yaw)

        # Odometry twist is body-frame; convert to world-frame before radial projection.
        vx_w = c * vx_b - s * vy_b
        vy_w = s * vx_b + c * vy_b

        rx = x - params.target_x
        ry = y - params.target_y
        r = math.hypot(rx, ry)
        if r < params.min_orbit_radius:
            self.current_r = 0.0
            self.current_radial_error = -params.r0
            self.current_v_rad = vx_b
            self.current_v_tan = vy_b
            self.current_r_hat_x = c
            self.current_r_hat_y = s
            self.current_t_hat_x = -s
            self.current_t_hat_y = c
            self.current_cos_yaw = c
            self.current_sin_yaw = s
            self.state_ready = True

            msg_err = Float64()
            msg_err.data = -params.r0
            self.radial_err_pub.publish(msg_err)
            return

        radial_error = r - params.r0
        r_hat_x = rx / r
        r_hat_y = ry / r
        t_hat_x = -r_hat_y
        t_hat_y = r_hat_x
        v_rad = vx_w * r_hat_x + vy_w * r_hat_y
        v_tan = vx_w * t_hat_x + vy_w * t_hat_y

        self.current_r = r
        self.current_radial_error = radial_error
        self.current_v_rad = v_rad
        self.current_v_tan = v_tan
        self.current_r_hat_x = r_hat_x
        self.current_r_hat_y = r_hat_y
        self.current_t_hat_x = t_hat_x
        self.current_t_hat_y = t_hat_y
        self.current_cos_yaw = c
        self.current_sin_yaw = s
        self.state_ready = True

        msg_err = Float64()
        msg_err.data = radial_error
        self.radial_err_pub.publish(msg_err)

    def control_tick(self):
        if not self.state_ready:
            return

        radial_error = self.current_radial_error
        v_rad = self.current_v_rad
        v_tan = self.current_v_tan
        r_hat_x = self.current_r_hat_x
        r_hat_y = self.current_r_hat_y
        t_hat_x = self.current_t_hat_x
        t_hat_y = self.current_t_hat_y
        thruster_force = self.thruster_force

        # Radial-only error/velocity vectors in world XY.
        e_wx = radial_error * r_hat_x
        e_wy = radial_error * r_hat_y
        v_wx = v_rad * r_hat_x
        v_wy = v_rad * r_hat_y

        # Rotate world -> body, then build independent body-axis surfaces.
        c = self.current_cos_yaw
        s = self.current_sin_yaw
        e_bx = c * e_wx + s * e_wy
        e_by = -s * e_wx + c * e_wy
        v_bx = c * v_wx + s * v_wy
        v_by = -s * v_wx + c * v_wy
        s_x = v_bx + params.lambda_bx * e_bx
        s_y = v_by + params.lambda_by * e_by

        sx_msg = Float64()
        sx_msg.data = s_x
        self.sx_pub.publish(sx_msg)

        sy_msg = Float64()
        sy_msg.data = s_y
        self.sy_pub.publish(sy_msg)

        k_x = params.gain_bx * thruster_force
        k_y = params.gain_by * thruster_force
        fx_b_smc = -k_x * sat(s_x / max(params.boundary_layer_bx, params.min_thruster_force))
        fy_b_smc = -k_y * sat(s_y / max(params.boundary_layer_by, params.min_thruster_force))

        # Near the setpoint, use smooth radial PD damping to reduce bang-bang limit cycles.
        if abs(radial_error) < params.near_band_error:
            kp = (
                params.near_kp_scale
                * thruster_force
                / max(params.near_band_error, params.min_thruster_force)
            )
            kd = (
                params.near_kd_scale
                * thruster_force
                / max(params.radial_settle_vrad, params.min_thruster_force)
            )
            f_r = -(kp * radial_error + kd * v_rad)
            f_r = max(-thruster_force, min(thruster_force, f_r))

            fx_w = f_r * r_hat_x
            fy_w = f_r * r_hat_y
            fx_b = c * fx_w + s * fy_w
            fy_b = -s * fx_w + c * fy_w
        else:
            fx_b = fx_b_smc
            fy_b = fy_b_smc

        # Tangential control with radial convergence gating + hysteresis.
        if self.tan_enabled:
            if (
                abs(radial_error) > params.tangential_gate_err_off
                or abs(v_rad) > params.tangential_gate_vrad_off
            ):
                self.tan_enabled = False
        else:
            if (
                abs(radial_error) < params.tangential_gate_err_on
                and abs(v_rad) < params.tangential_gate_vrad_on
            ):
                self.tan_enabled = True

        if self.tan_enabled:
            k_t = (
                params.tangential_gain_scale
                * thruster_force
                / max(params.tangential_boundary_layer, params.min_thruster_force)
            )
            f_t = k_t * (self.v_tan_des - v_tan)
            tangential_force_limit = params.tangential_force_limit_scale * thruster_force
            f_t = max(-tangential_force_limit, min(tangential_force_limit, f_t))

            fx_w_tan = f_t * t_hat_x
            fy_w_tan = f_t * t_hat_y
            fx_b += c * fx_w_tan + s * fy_w_tan
            fy_b += -s * fx_w_tan + c * fy_w_tan

        # Radial feedforward for circular motion (centripetal demand).
        r_eff = max(self.current_r, params.min_orbit_radius)
        f_r_ff = -params.centripetal_ff_gain * self.mass * (v_tan * v_tan) / r_eff
        fx_w_ff = f_r_ff * r_hat_x
        fy_w_ff = f_r_ff * r_hat_y
        fx_b += c * fx_w_ff + s * fy_w_ff
        fy_b += -s * fx_w_ff + c * fy_w_ff

        axis_duty = params.max_axis_duty_with_tan if self.tan_enabled else params.max_axis_duty
        axis_force_cap = axis_duty * thruster_force
        fx_b = max(-axis_force_cap, min(axis_force_cap, fx_b))
        fy_b = max(-axis_force_cap, min(axis_force_cap, fy_b))

        if abs(radial_error) < params.radial_settle_error and abs(v_rad) < params.radial_settle_vrad:
            self.imp_x = 0.0
            self.imp_y = 0.0

        dt = params.control_period
        i_pulse = thruster_force * params.min_pulse_width
        i_cap = params.max_impulse_pulses * i_pulse

        self.imp_x += fx_b * dt
        self.imp_y += fy_b * dt
        self.imp_x = max(-i_cap, min(i_cap, self.imp_x))
        self.imp_y = max(-i_cap, min(i_cap, self.imp_y))

        px = nx = py = ny = False
        self.pulse_x_gap_time = max(0.0, self.pulse_x_gap_time - dt)
        self.pulse_y_gap_time = max(0.0, self.pulse_y_gap_time - dt)

        if self.pulse_x_time > 0.0:
            self.pulse_x_time -= dt
            if self.pulse_x_dir > 0:
                px = True
            elif self.pulse_x_dir < 0:
                nx = True
        elif self.pulse_x_gap_time <= 0.0 and abs(self.imp_x) >= i_pulse:
            if self.imp_x > 0.0:
                self.pulse_x_dir = 1
                px = True
                self.imp_x -= i_pulse
            else:
                self.pulse_x_dir = -1
                nx = True
                self.imp_x += i_pulse
            self.pulse_x_time = params.min_pulse_width
            self.pulse_x_gap_time = params.min_pulse_gap

        if self.pulse_y_time > 0.0:
            self.pulse_y_time -= dt
            if self.pulse_y_dir > 0:
                py = True
            elif self.pulse_y_dir < 0:
                ny = True
        elif self.pulse_y_gap_time <= 0.0 and abs(self.imp_y) >= i_pulse:
            if self.imp_y > 0.0:
                self.pulse_y_dir = 1
                py = True
                self.imp_y -= i_pulse
            else:
                self.pulse_y_dir = -1
                ny = True
                self.imp_y += i_pulse
            self.pulse_y_time = params.min_pulse_width
            self.pulse_y_gap_time = params.min_pulse_gap

        self.imp_x *= params.impulse_leak
        self.imp_y *= params.impulse_leak

        if px and nx:
            px = nx = False
        if py and ny:
            py = ny = False

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
    node = BangBangRadialController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.publish_thrusters(False, False, False, False)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
