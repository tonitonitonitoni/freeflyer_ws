#!/usr/bin/env python3
import os
import time
from collections import deque

import matplotlib
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

# Force interactive backend
if "MPLBACKEND" not in os.environ:
    try:
        matplotlib.use("TkAgg")
    except Exception:
        pass

import matplotlib.pyplot as plt


class ForceVelocityLivePlot(Node):
    def __init__(self):
        super().__init__("force_velocity_live_plot")

        self.get_logger().info(f"matplotlib backend: {matplotlib.get_backend()}")

        self.history_size = 1200
        self.last_time = None
        self.last_vx = None
        self.last_vy = None
        self.last_wz = None
        self.last_state_time = None

        # Thruster commands in [0, 1]
        self.px_cmd = 0.0
        self.nx_cmd = 0.0
        self.py_cmd = 0.0
        self.ny_cmd = 0.0
        self.rw_cw_cmd = False
        self.rw_ccw_cmd = False

        self.t0 = time.time()

        self.t  = deque(maxlen=self.history_size)
        self.fx = deque(maxlen=self.history_size)
        self.fy = deque(maxlen=self.history_size)
        self.vx = deque(maxlen=self.history_size)
        self.vy = deque(maxlen=self.history_size)
        self.ax = deque(maxlen=self.history_size)
        self.ay = deque(maxlen=self.history_size)
        self.rw_cw = deque(maxlen=self.history_size)
        self.rw_ccw = deque(maxlen=self.history_size)
        self.wz = deque(maxlen=self.history_size)
        self.az = deque(maxlen=self.history_size)

        # --- Subscriptions ---
        self.create_subscription(Float32, "/freeflyer/thrusters/px", self.cb_px, 10)
        self.create_subscription(Float32, "/freeflyer/thrusters/nx", self.cb_nx, 10)
        self.create_subscription(Float32, "/freeflyer/thrusters/py", self.cb_py, 10)
        self.create_subscription(Float32, "/freeflyer/thrusters/ny", self.cb_ny, 10)
        self.create_subscription(Bool, "/freeflyer/reaction_wheel/cw", self.cb_rw_cw, 10)
        self.create_subscription(Bool, "/freeflyer/reaction_wheel/ccw", self.cb_rw_ccw, 10)

        # ONLY correct state source
        self.create_subscription(Odometry, "/freeflyer/thruster_odom", self.cb_odom, 10)

        self.get_logger().info("Listening to /freeflyer/thruster_odom + thruster topics")

        # --- Plot setup ---
        plt.ion()
        self.fig, axs = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
        self.ax_force = axs[0, 0]
        self.ax_vel = axs[1, 0]
        self.ax_acc = axs[2, 0]
        self.ax_rw = axs[0, 1]
        self.ax_wz = axs[1, 1]
        self.ax_az = axs[2, 1]

        self.line_fx, = self.ax_force.plot([], [], label="Fx cmd", linewidth=1.8)
        self.line_fy, = self.ax_force.plot([], [], label="Fy cmd", linewidth=1.8)
        self.ax_force.set_ylabel("Force Cmd")
        self.ax_force.set_ylim(-1.2, 1.2)
        self.ax_force.grid(True)
        self.ax_force.legend()

        self.line_vx, = self.ax_vel.plot([], [], label="vx", linewidth=1.8)
        self.line_vy, = self.ax_vel.plot([], [], label="vy", linewidth=1.8)
        self.ax_vel.set_ylabel("Velocity")
        self.ax_vel.grid(True)
        self.ax_vel.legend()

        self.line_ax, = self.ax_acc.plot([], [], label="ax", linewidth=1.8)
        self.line_ay, = self.ax_acc.plot([], [], label="ay", linewidth=1.8)
        self.ax_acc.set_ylabel("Acceleration")
        self.ax_acc.set_xlabel("Time [s]")
        self.ax_acc.grid(True)
        self.ax_acc.legend()

        self.line_rw_cw, = self.ax_rw.plot([], [], label="rw_cw", linewidth=1.8, drawstyle="steps-post")
        self.line_rw_ccw, = self.ax_rw.plot([], [], label="rw_ccw", linewidth=1.8, drawstyle="steps-post")
        self.ax_rw.set_ylabel("RW Cmd")
        self.ax_rw.set_ylim(-0.1, 1.1)
        self.ax_rw.grid(True)
        self.ax_rw.legend()

        self.line_wz, = self.ax_wz.plot([], [], label="yaw rate wz", linewidth=1.8)
        self.ax_wz.set_ylabel("Yaw Rate")
        self.ax_wz.grid(True)
        self.ax_wz.legend()

        self.line_az, = self.ax_az.plot([], [], label="yaw accel az", linewidth=1.8)
        self.ax_az.set_ylabel("Yaw Accel")
        self.ax_az.set_xlabel("Time [s]")
        self.ax_az.grid(True)
        self.ax_az.legend()

        self.fig.suptitle("Thruster Commands vs Motion")
        self.fig.tight_layout()
        plt.show(block=False)

        self.create_timer(0.05, self.update_plot)
        self.create_timer(2.0, self.watchdog)

    # ---------------- Thruster callbacks ----------------

    def cb_px(self, msg): self.px_cmd = float(msg.data)
    def cb_nx(self, msg): self.nx_cmd = float(msg.data)
    def cb_py(self, msg): self.py_cmd = float(msg.data)
    def cb_ny(self, msg): self.ny_cmd = float(msg.data)
    def cb_rw_cw(self, msg): self.rw_cw_cmd = bool(msg.data)
    def cb_rw_ccw(self, msg): self.rw_ccw_cmd = bool(msg.data)

    # ---------------- Odom callback ----------------

    def cb_odom(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        self.push_state(vx, vy, wz)

    # ---------------- Data logic ----------------

    def push_state(self, vx, vy, wz):
        now = time.time() - self.t0
        self.last_state_time = now

        fx = self.px_cmd - self.nx_cmd
        fy = self.py_cmd - self.ny_cmd

        if self.last_time is None:
            ax = ay = az = 0.0
        else:
            dt = now - self.last_time
            ax = (vx - self.last_vx)/dt if dt > 1e-6 else 0.0
            ay = (vy - self.last_vy)/dt if dt > 1e-6 else 0.0
            az = (wz - self.last_wz)/dt if dt > 1e-6 else 0.0

        self.last_time = now
        self.last_vx = vx
        self.last_vy = vy
        self.last_wz = wz

        self.t.append(now)
        self.fx.append(fx)
        self.fy.append(fy)
        self.vx.append(vx)
        self.vy.append(vy)
        self.ax.append(ax)
        self.ay.append(ay)
        self.rw_cw.append(1.0 if self.rw_cw_cmd else 0.0)
        self.rw_ccw.append(1.0 if self.rw_ccw_cmd else 0.0)
        self.wz.append(wz)
        self.az.append(az)

    # ---------------- Watchdog ----------------

    def watchdog(self):
        if self.last_state_time is None:
            pubs = [
                name for name, types in self.get_topic_names_and_types()
                if "nav_msgs/msg/Odometry" in types
            ]
            self.get_logger().warn(f"No state yet. Odometry topics detected: {pubs}")

    # ---------------- Plot ----------------

    def update_plot(self):
        if not self.t:
            return

        self.line_fx.set_data(self.t, self.fx)
        self.line_fy.set_data(self.t, self.fy)
        self.line_vx.set_data(self.t, self.vx)
        self.line_vy.set_data(self.t, self.vy)
        self.line_ax.set_data(self.t, self.ax)
        self.line_ay.set_data(self.t, self.ay)
        self.line_rw_cw.set_data(self.t, self.rw_cw)
        self.line_rw_ccw.set_data(self.t, self.rw_ccw)
        self.line_wz.set_data(self.t, self.wz)
        self.line_az.set_data(self.t, self.az)

        for a in (self.ax_force, self.ax_vel, self.ax_acc, self.ax_wz, self.ax_az):
            a.relim()
            a.autoscale_view()

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main():
    rclpy.init()
    node = ForceVelocityLivePlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
