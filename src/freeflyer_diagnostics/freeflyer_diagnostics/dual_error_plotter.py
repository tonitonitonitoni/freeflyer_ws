#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
from collections import deque
import time

class DualErrorPlotter(Node):
    def __init__(self):
        super().__init__('dual_error_plotter')

        self.radial_sub = self.create_subscription(
            Float64,
            '/orbit/radial_error',
            self.radial_cb,
            10
        )
        self.yaw_sub = self.create_subscription(
            Float64,
            '/orbit/yaw_error',
            self.yaw_cb,
            10
        )

        # Storage
        self.t0 = time.time()
        self.radial_t = deque(maxlen=600)
        self.radial_err = deque(maxlen=600)
        self.yaw_t = deque(maxlen=600)
        self.yaw_err = deque(maxlen=600)

        # ---- Matplotlib setup ----
        plt.ion()
        self.fig, (self.radial_ax, self.yaw_ax) = plt.subplots(2, 1)
        self.radial_line, = self.radial_ax.plot([], [], lw=2)
        self.yaw_line, = self.yaw_ax.plot([], [], lw=2)

        self.radial_ax.set_title("Radial Tracking Error")
        self.radial_ax.set_xlabel("Time [s]")
        self.radial_ax.set_ylabel("Radial Error [m]")
        self.radial_ax.grid(True)
        self.radial_ax.set_ylim(-0.4, 1.0)

        self.yaw_ax.set_title("Yaw Tracking Error")
        self.yaw_ax.set_xlabel("Time [s]")
        self.yaw_ax.set_ylabel("Yaw Error [rad]")
        self.yaw_ax.grid(True)


        self.fig.show()
        self.fig.canvas.draw()

    def radial_cb(self, msg: Float64):
        now = time.time() - self.t0

        self.radial_t.append(now)
        self.radial_err.append(msg.data)

        self.radial_line.set_data(self.radial_t, self.radial_err)

        self.radial_ax.relim()
        self.radial_ax.autoscale_view()

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def yaw_cb(self, msg: Float64):
        now = time.time() - self.t0

        self.yaw_t.append(now)
        self.yaw_err.append(float(msg.data))

        self.yaw_line.set_data(self.yaw_t, self.yaw_err)

        self.yaw_ax.relim()
        self.yaw_ax.autoscale_view()

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

def main():
    rclpy.init()
    node = DualErrorPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
