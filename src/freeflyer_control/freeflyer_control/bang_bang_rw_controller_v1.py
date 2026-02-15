#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float64, Bool


def wrap_to_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class BangBangCircleController(Node):
    def __init__(self):
        super().__init__("bang_bang_circle_controller")

        # ---- Orbit guidance ----
        self.declare_parameter("r0", 1.5)
        self.declare_parameter("v_tan_des", 0.1)
        self.declare_parameter("k_r", 5.0)
        self.declare_parameter("k_dr", 2.7)
        self.declare_parameter("k_ir", 0.35)
        self.declare_parameter("radial_i_limit", 0.25)
        self.declare_parameter("radial_i_zone", 0.25)
        self.declare_parameter("radial_i_leak", 0.25)
        self.declare_parameter("k_t", 1.0)
        self.declare_parameter("k_cent", 1.0)
        self.declare_parameter("enable_tangential_control", True)
        self.declare_parameter("enable_centripetal_ff", True)
        self.declare_parameter("enable_radial_integral", True)
        self.declare_parameter("use_odom_twist", True)

        # ---- Attitude control ----
        self.declare_parameter("k_yaw", 5.0)
        self.declare_parameter("k_yaw_rate", 5.0)
        self.declare_parameter("yaw_deadband", 0.02)
        self.declare_parameter("yaw_on_threshold", 0.45)
        self.declare_parameter("yaw_off_threshold", 0.20)
        self.declare_parameter("yaw_rate_limit", 0.35)
        self.declare_parameter("yaw_rate_error_on_threshold", 0.015)
        self.declare_parameter("yaw_rate_error_off_threshold", 0.005)
        self.declare_parameter("yaw_surface_gain", 0.8)
        self.declare_parameter("yaw_surface_on_threshold", 0.02)
        self.declare_parameter("yaw_surface_off_threshold", 0.004)
        self.declare_parameter("yaw_ref_rate_limit", 0.15)
        self.declare_parameter("use_geometric_yaw_rate", True)
        self.declare_parameter("yaw_min_speed_for_control", 0.05)
        self.declare_parameter("attitude_enable_radial_tol", 0.12)
        self.declare_parameter("attitude_enable_vtan_tol", 0.12)

        # ---- Actuator limits ----
        self.declare_parameter("thruster_force", 2.0)
        self.declare_parameter("reaction_torque", 0.015)
        self.declare_parameter("enable_attitude_control", True)

        self.r0 = float(self.get_parameter("r0").value)
        self.v_tan_des = float(self.get_parameter("v_tan_des").value)
        self.k_r = float(self.get_parameter("k_r").value)
        self.k_dr = float(self.get_parameter("k_dr").value)
        self.k_ir = float(self.get_parameter("k_ir").value)
        self.radial_i_limit = max(0.0, float(self.get_parameter("radial_i_limit").value))
        self.radial_i_zone = max(0.0, float(self.get_parameter("radial_i_zone").value))
        self.radial_i_leak = max(0.0, float(self.get_parameter("radial_i_leak").value))
        self.k_t = float(self.get_parameter("k_t").value)
        self.k_cent = float(self.get_parameter("k_cent").value)
        self.enable_tangential_control = bool(
            self.get_parameter("enable_tangential_control").value
        )
        self.enable_centripetal_ff = bool(
            self.get_parameter("enable_centripetal_ff").value
        )
        self.enable_radial_integral = bool(
            self.get_parameter("enable_radial_integral").value
        )
        self.use_odom_twist = bool(self.get_parameter("use_odom_twist").value)

        self.k_yaw = float(self.get_parameter("k_yaw").value)
        self.k_yaw_rate = float(self.get_parameter("k_yaw_rate").value)
        self.yaw_deadband = float(self.get_parameter("yaw_deadband").value)
        self.yaw_on_threshold = abs(float(self.get_parameter("yaw_on_threshold").value))
        self.yaw_off_threshold = abs(float(self.get_parameter("yaw_off_threshold").value))
        self.yaw_rate_limit = abs(float(self.get_parameter("yaw_rate_limit").value))
        self.yaw_rate_error_on_threshold = abs(
            float(self.get_parameter("yaw_rate_error_on_threshold").value)
        )
        self.yaw_rate_error_off_threshold = abs(
            float(self.get_parameter("yaw_rate_error_off_threshold").value)
        )
        self.yaw_surface_gain = float(self.get_parameter("yaw_surface_gain").value)
        self.yaw_surface_on_threshold = abs(
            float(self.get_parameter("yaw_surface_on_threshold").value)
        )
        self.yaw_surface_off_threshold = abs(
            float(self.get_parameter("yaw_surface_off_threshold").value)
        )
        self.yaw_ref_rate_limit = abs(float(self.get_parameter("yaw_ref_rate_limit").value))
        self.use_geometric_yaw_rate = bool(self.get_parameter("use_geometric_yaw_rate").value)
        self.yaw_min_speed_for_control = max(
            0.0, float(self.get_parameter("yaw_min_speed_for_control").value)
        )
        self.attitude_enable_radial_tol = max(
            0.0, float(self.get_parameter("attitude_enable_radial_tol").value)
        )
        self.attitude_enable_vtan_tol = max(
            0.0, float(self.get_parameter("attitude_enable_vtan_tol").value)
        )
        if self.yaw_off_threshold > self.yaw_on_threshold:
            self.yaw_off_threshold = self.yaw_on_threshold

        self.Fmax = float(self.get_parameter("thruster_force").value)
        self.Tmax = float(self.get_parameter("reaction_torque").value)
        self.enable_attitude_control = bool(self.get_parameter("enable_attitude_control").value)

        self.target_x = 0.0
        self.target_y = 0.0

        self.fx_cmd = 0.0
        self.fy_cmd = 0.0
        self.tau_cmd = 0.0
        self.last_x = None
        self.last_y = None
        self.last_t = None
        self.last_guidance_t = None
        self.radial_error_int = 0.0
        self.yaw_error = 0.0
        self.yaw_rate = 0.0
        self.desired_yaw_rate = 0.0
        self.desired_yaw_cmd = None
        self.rw_dir = 0  # -1: cw, +1: ccw, 0: off
        self.attitude_ready = False

        # ---- Subscriptions ----
        self.create_subscription(Odometry, "/freeflyer/odometry/filtered", self.odom_cb, 10)

        # ---- Publishers ----
        self.radial_err_pub = self.create_publisher(Float64, "/orbit/radial_error", 10)
        self.yaw_err_pub = self.create_publisher(Float64, "/orbit/yaw_error", 10)
        self.debug_fx_w_pub = self.create_publisher(Float64, "/debug/fx_w", 10)
        self.debug_fy_w_pub = self.create_publisher(Float64, "/debug/fy_w", 10)
        self.debug_fr_pub = self.create_publisher(Float64, "/debug/f_r", 10)

        self.thrusters = {
            "px": self.create_publisher(Float32, "/freeflyer/thrusters/px", 10),
            "nx": self.create_publisher(Float32, "/freeflyer/thrusters/nx", 10),
            "py": self.create_publisher(Float32, "/freeflyer/thrusters/py", 10),
            "ny": self.create_publisher(Float32, "/freeflyer/thrusters/ny", 10),
        }

        self.rw_cw_pub = self.create_publisher(Bool, "/freeflyer/reaction_wheel/cw", 10)
        self.rw_ccw_pub = self.create_publisher(Bool, "/freeflyer/reaction_wheel/ccw", 10)

        self.create_timer(0.02, self.control_tick)

    # ---------------- ODOM ----------------

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        t = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)

        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        )
        c = math.cos(yaw)
        s = math.sin(yaw)

        if self.use_odom_twist:
            vx_m = msg.twist.twist.linear.x
            vy_m = msg.twist.twist.linear.y

            # Odometry twist is commonly reported in child_frame_id (body frame).
            # Convert to world frame for radial/tangential decomposition.
            child = (msg.child_frame_id or "").strip()
            if child == "" or child.endswith("base_link"):
                vx = c * vx_m - s * vy_m
                vy = s * vx_m + c * vy_m
            else:
                vx = vx_m
                vy = vy_m
        else:
            if self.last_t is None:
                self.last_x, self.last_y, self.last_t = x, y, t
                return

            dt = t - self.last_t
            if dt > 1e-4:
                vx = (x - self.last_x) / dt
                vy = (y - self.last_y) / dt
            else:
                vx = 0.0
                vy = 0.0
            self.last_x = x
            self.last_y = y
            self.last_t = t

        yaw_rate = msg.twist.twist.angular.z
        self.yaw_rate = yaw_rate

        # ---- Orbit guidance ----
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
        speed = math.hypot(vx, vy)
        self.attitude_ready = (
            abs(radial_error) <= self.attitude_enable_radial_tol
            and abs(speed - abs(self.v_tan_des)) <= self.attitude_enable_vtan_tol
            and speed >= self.yaw_min_speed_for_control
        )

        dt_guidance = 0.0
        if self.last_guidance_t is not None:
            dt_guidance = max(0.0, t - self.last_guidance_t)
        self.last_guidance_t = t

        if self.enable_radial_integral and dt_guidance > 0.0:
            leak_scale = max(0.0, 1.0 - self.radial_i_leak * dt_guidance)
            self.radial_error_int *= leak_scale

            # Integrate only near the target radius to avoid pumping energy
            # during large transients.
            if abs(radial_error) <= self.radial_i_zone:
                self.radial_error_int += radial_error * dt_guidance
            self.radial_error_int = max(
                -self.radial_i_limit,
                min(self.radial_i_limit, self.radial_error_int),
            )

        f_r = -self.k_r * radial_error - self.k_dr * v_rad
        if self.enable_radial_integral:
            f_r += -self.k_ir * self.radial_error_int
        if self.enable_centripetal_ff:
            # Inward feedforward to sustain circular motion without biasing
            # the radial-error loop. Mass is assumed 1 kg in this simplified model.
            f_r += -self.k_cent * (v_tan * v_tan / max(r, 1e-3))
        if self.enable_tangential_control:
            f_t = self.k_t * (self.v_tan_des - v_tan)
        else:
            f_t = 0.0

        fx_w = f_r * r_hat_x + f_t * t_hat_x
        fy_w = f_r * r_hat_y + f_t * t_hat_y
        msg_dbg = Float64()
        msg_dbg.data = f_r
        self.debug_fr_pub.publish(msg_dbg)
        msg_dbg = Float64()
        msg_dbg.data = fx_w
        self.debug_fx_w_pub.publish(msg_dbg)
        msg_dbg = Float64()
        msg_dbg.data = fy_w
        self.debug_fy_w_pub.publish(msg_dbg)

        # Rotate world force → body frame
        self.fx_cmd = c*fx_w + s*fy_w
        self.fy_cmd = -s*fx_w + c*fy_w

        # Desired attitude: point inward toward the orbit center (origin/target),
        # i.e., opposite of the radial unit vector.
        desired_yaw = math.atan2(-r_hat_y, -r_hat_x)
        if self.desired_yaw_cmd is None:
            self.desired_yaw_cmd = yaw
        self.desired_yaw_rate = 0.0

        if dt_guidance > 0.0 and speed >= self.yaw_min_speed_for_control:
            yaw_step = wrap_to_pi(desired_yaw - self.desired_yaw_cmd)
            max_step = self.yaw_ref_rate_limit * dt_guidance
            if abs(yaw_step) > max_step:
                yaw_step = math.copysign(max_step, yaw_step)
            self.desired_yaw_cmd = wrap_to_pi(self.desired_yaw_cmd + yaw_step)
            if self.use_geometric_yaw_rate:
                yr_des = v_tan / max(r, 1e-3)
                self.desired_yaw_rate = max(-self.yaw_ref_rate_limit, min(self.yaw_ref_rate_limit, yr_des))
            else:
                self.desired_yaw_rate = yaw_step / dt_guidance

        yaw_error = wrap_to_pi(self.desired_yaw_cmd - yaw)
        self.yaw_error = yaw_error
        msg_yaw_err = Float64()
        msg_yaw_err.data = yaw_error
        self.yaw_err_pub.publish(msg_yaw_err)

        # ---- Attitude control (optional) ----
        if self.enable_attitude_control:
            yaw_rate_error = self.desired_yaw_rate - yaw_rate
            tau = self.k_yaw * yaw_error + self.k_yaw_rate * yaw_rate_error
            tau = max(-self.Tmax, min(self.Tmax, tau))

            if abs(yaw_error) < self.yaw_deadband:
                tau = 0.0

            self.tau_cmd = tau
        else:
            self.tau_cmd = 0.0

    # ---------------- ACTUATION ----------------

    def control_tick(self):

        # --- Linear thrusters (continuous split command) ---
        fx = max(-self.Fmax, min(self.Fmax, self.fx_cmd))
        fy = max(-self.Fmax, min(self.Fmax, self.fy_cmd))

        self.publish_thruster_pair("px", "nx", fx)
        self.publish_thruster_pair("py", "ny", fy)

        # --- Reaction wheel bang-bang control (with hysteresis + rate limiting) ---
        cw_msg = Bool()
        ccw_msg = Bool()

        if not self.enable_attitude_control or not self.attitude_ready:
            self.rw_dir = 0
        else:
            yerr = self.yaw_error
            yr = self.yaw_rate
            yre = self.desired_yaw_rate - yr
            # Sliding surface: combine angle + rate error into one switching signal.
            s = yre + self.yaw_surface_gain * yerr

            if self.rw_dir == 0:
                if s > self.yaw_surface_on_threshold:
                    self.rw_dir = +1
                elif s < -self.yaw_surface_on_threshold:
                    self.rw_dir = -1
                elif yerr > self.yaw_on_threshold:
                    self.rw_dir = +1
                elif yerr < -self.yaw_on_threshold:
                    self.rw_dir = -1
            elif self.rw_dir > 0:
                # Stop when surface closes, rate is close enough, or spinning too fast.
                if (
                    s < self.yaw_surface_off_threshold
                    or yre < self.yaw_rate_error_off_threshold
                    or yr > self.yaw_rate_limit
                    or yerr < -self.yaw_off_threshold
                ):
                    self.rw_dir = 0
            else:
                # Stop when surface closes, rate is close enough, or spinning too fast.
                if (
                    s > -self.yaw_surface_off_threshold
                    or yre > -self.yaw_rate_error_off_threshold
                    or yr < -self.yaw_rate_limit
                    or yerr > self.yaw_off_threshold
                ):
                    self.rw_dir = 0

        cw_msg.data = self.rw_dir < 0
        ccw_msg.data = self.rw_dir > 0

        self.rw_cw_pub.publish(cw_msg)
        self.rw_ccw_pub.publish(ccw_msg)


    def publish_thruster_pair(self, pos_name, neg_name, force):
        # Convert desired signed force [-Fmax, +Fmax] into one-sided continuous
        # commands [0, 1] expected by the continuous Gazebo thruster plugin.
        pos_cmd = max(0.0, force) / self.Fmax
        neg_cmd = max(0.0, -force) / self.Fmax

        msg = Float32()
        msg.data = float(pos_cmd)
        self.thrusters[pos_name].publish(msg)

        msg = Float32()
        msg.data = float(neg_cmd)
        self.thrusters[neg_name].publish(msg)


def main():
    rclpy.init()
    node = BangBangCircleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
