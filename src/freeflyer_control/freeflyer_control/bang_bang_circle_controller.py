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
    control_period = 0.01  # 100 Hz (unrealistic)
    pwm_period = 0.01

    # Bang-bang / actuator shaping
    deadband_force = 0.05
    force_hyst_on = 0.05
    force_hyst_off = 0.02
    min_thruster_force = 1e-6

    # Radial controller
    radial_error_to_v_gain = 0.9
    radial_velocity_limit = 0.14
    radial_velocity_gain_scale = 0.80
    radial_wall_guard_error = 0.35
    radial_wall_brake_force_scale = 0.55
    radial_settle_error = 0.06
    radial_settle_vrad = 0.05
    radial_damping_error_band = 0.50
    radial_near_kp_scale = 0.12
    radial_near_kd_scale = 0.70

    # Tangential controller
    tangential_gate_on = 0.12
    tangential_gate_off = 0.20
    tangential_radial_gate_on = 0.10
    tangential_radial_gate_off = 0.18
    tangential_vrad_gate_on = 0.10
    tangential_vrad_gate_off = 0.16
    tangential_boundary_layer = 0.08
    tangential_gain_scale = 0.5
    tangential_force_limit_scale = 0.5

    # PWM authority cap to reduce slam into the wall during transients.
    max_axis_duty = 0.25
    min_pulse_width = 0.02      # 20 ms realistic valve open time
    min_pulse_gap = 0.02        # keep anti-chatter without starving actuation
    impulse_leak = 0.995
    max_impulse_pulses = 3.0



class BangBangCircleController(Node):
    def __init__(self):
        super().__init__("bang_bang_circle_controller")

        self.declare_parameter("v_tan_des", 0.3)
        # Match simulated thruster max_thrust default in freeflyer.urdf.xacro.
        self.declare_parameter("thruster_force", 2.0)

        self.v_tan_des = float(self.get_parameter("v_tan_des").value)
        self.thruster_force = max(
            params.min_thruster_force, float(self.get_parameter("thruster_force").value)
        )

        self.state_ready = False
        self.tan_enabled = False

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

        self.imp_x = 0.0
        self.imp_y = 0.0

        self.pulse_x_time = 0.0
        self.pulse_y_time = 0.0
        self.pulse_x_gap_time = 0.0
        self.pulse_y_gap_time = 0.0
        self.pulse_x_dir = 0
        self.pulse_y_dir = 0
        self.in_damping_band_prev = False

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
        self.create_timer(params.control_period, self.control_tick)

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

        rx = x - params.target_x
        ry = y - params.target_y
        r = math.hypot(rx, ry)
        if r < params.min_orbit_radius:
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
            msg_err.data = -params.r0
            self.radial_err_pub.publish(msg_err)
            return

        radial_error = r - params.r0
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
        radial_error = r - params.r0
        thruster_force = self.thruster_force
        in_damping_band = abs(radial_error) < params.radial_damping_error_band

        # On entry to damping band, clear active pulse timers/directions.
        if in_damping_band and not self.in_damping_band_prev:
            self.pulse_x_time = 0.0
            self.pulse_y_time = 0.0
            self.pulse_x_dir = 0
            self.pulse_y_dir = 0

        # ---------------------------
        # 1) Radial velocity-governed command
        # ---------------------------
        v_rad_des = -params.radial_error_to_v_gain * radial_error
        v_rad_des = max(-params.radial_velocity_limit, min(params.radial_velocity_limit, v_rad_des))
        k_vr = (
            params.radial_velocity_gain_scale
            * thruster_force
            / max(params.radial_velocity_limit, params.min_thruster_force)
        )
        f_r = k_vr * (v_rad_des - v_rad)
        f_r = max(-thruster_force, min(thruster_force, f_r))

        # Near-orbit damping mode to dissipate oscillations from pulse quantization.
        if in_damping_band:
            k_near_p = (
                params.radial_near_kp_scale
                * thruster_force
                / max(params.radial_damping_error_band, params.min_thruster_force)
            )
            k_near_d = (
                params.radial_near_kd_scale
                * thruster_force
                / max(params.radial_settle_vrad, params.min_thruster_force)
            )
            f_r = -(k_near_p * radial_error + k_near_d * v_rad)
            f_r = max(-thruster_force, min(thruster_force, f_r))

        # Hard wall guards: if already far out and still moving outward, force braking inward.
        if radial_error > params.radial_wall_guard_error and v_rad > 0.0:
            f_r = min(f_r, -params.radial_wall_brake_force_scale * thruster_force)
        if radial_error < -params.radial_wall_guard_error and v_rad < 0.0:
            f_r = max(f_r, params.radial_wall_brake_force_scale * thruster_force)

        # ------------------------------------------
        # 3) Tangential control gating
        #    Enable only when both the radial surface and radial error are small.
        # ------------------------------------------
        s_r = v_rad + params.radial_error_to_v_gain * radial_error
        if not in_damping_band:
            self.tan_enabled = False
        elif self.tan_enabled:
            if (
                abs(s_r) > params.tangential_gate_off
                or abs(radial_error) > params.tangential_radial_gate_off
                or abs(v_rad) > params.tangential_vrad_gate_off
            ):
                self.tan_enabled = False
        else:
            if (
                abs(s_r) < params.tangential_gate_on
                and abs(radial_error) < params.tangential_radial_gate_on
                and abs(v_rad) < params.tangential_vrad_gate_on
            ):
                self.tan_enabled = True

        if self.tan_enabled:
            v_err = self.v_tan_des - v_tan
            k_vt = (
                params.tangential_gain_scale
                * thruster_force
                / max(params.tangential_boundary_layer, params.min_thruster_force)
            )
            f_t = k_vt * v_err
            tangential_force_limit = params.tangential_force_limit_scale * thruster_force
            f_t = max(-tangential_force_limit, min(tangential_force_limit, f_t))
        else:
            f_t = 0.0

        # ------------------------------------------
        # 4) (f_r, f_t) -> world -> body
        # ------------------------------------------
        r_hat_x = self.current_r_hat_x
        r_hat_y = self.current_r_hat_y
        t_hat_x = self.current_t_hat_x
        t_hat_y = self.current_t_hat_y

        fx_w = f_r * r_hat_x + f_t * t_hat_x
        fy_w = f_r * r_hat_y + f_t * t_hat_y

        # WORLD -> BODY
        c = self.current_cos_yaw
        s = self.current_sin_yaw
        fx_b = c * fx_w + s * fy_w
        fy_b = -s * fx_w + c * fy_w

        # Hard cap body-axis requested force to limit injected impulse per cycle.
        axis_force_cap = params.max_axis_duty * thruster_force
        fx_b = max(-axis_force_cap, min(axis_force_cap, fx_b))
        fy_b = max(-axis_force_cap, min(axis_force_cap, fy_b))

        # ------------------------------------------
        # 5) Pulse scheduling via impulse accumulator
        # ------------------------------------------

        # Near orbit: clear residual queued impulse, but do not disable control.
        if abs(radial_error) < params.radial_settle_error and abs(v_rad) < params.radial_settle_vrad:
            self.imp_x = 0.0
            self.imp_y = 0.0

        dt = params.control_period
        Fmax = thruster_force
        I_pulse = Fmax * params.min_pulse_width
        I_cap = params.max_impulse_pulses * I_pulse

        # --- Accumulate desired impulse ---
        self.imp_x += fx_b * dt
        self.imp_y += fy_b * dt
        self.imp_x = max(-I_cap, min(I_cap, self.imp_x))
        self.imp_y = max(-I_cap, min(I_cap, self.imp_y))

        # --- Handle active pulse timers ---
        px = nx = py = ny = False
        self.pulse_x_gap_time = max(0.0, self.pulse_x_gap_time - dt)
        self.pulse_y_gap_time = max(0.0, self.pulse_y_gap_time - dt)

        # X axis pulse timing (direction latched when pulse starts)
        if self.pulse_x_time > 0.0:
            self.pulse_x_time -= dt
            if self.pulse_x_dir > 0:
                px = True
            elif self.pulse_x_dir < 0:
                nx = True
        else:
            # No active pulse: check if enough impulse accumulated
            if self.pulse_x_gap_time <= 0.0 and abs(self.imp_x) >= I_pulse:
                if self.imp_x > 0.0:
                    self.pulse_x_dir = 1
                    px = True
                    self.imp_x -= I_pulse
                else:
                    self.pulse_x_dir = -1
                    nx = True
                    self.imp_x += I_pulse

                self.pulse_x_time = params.min_pulse_width
                self.pulse_x_gap_time = params.min_pulse_gap

        # Y axis pulse timing (direction latched when pulse starts)
        if self.pulse_y_time > 0.0:
            self.pulse_y_time -= dt
            if self.pulse_y_dir > 0:
                py = True
            elif self.pulse_y_dir < 0:
                ny = True
        else:
            if self.pulse_y_gap_time <= 0.0 and abs(self.imp_y) >= I_pulse:
                if self.imp_y > 0.0:
                    self.pulse_y_dir = 1
                    py = True
                    self.imp_y -= I_pulse
                else:
                    self.pulse_y_dir = -1
                    ny = True
                    self.imp_y += I_pulse

                self.pulse_y_time = params.min_pulse_width
                self.pulse_y_gap_time = params.min_pulse_gap
                
        self.imp_x *= params.impulse_leak
        self.imp_y *= params.impulse_leak
        # Never fire both on same axis
        if px and nx:
            px = nx = False
        if py and ny:
            py = ny = False

        self.publish_thrusters(px, nx, py, ny)


        self.last_px = px
        self.last_nx = nx
        self.last_py = py
        self.last_ny = ny
        self.in_damping_band_prev = in_damping_band


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
