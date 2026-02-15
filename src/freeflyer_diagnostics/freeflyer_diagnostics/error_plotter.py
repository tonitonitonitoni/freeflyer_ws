#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# import matplotlib
# matplotlib.use("TkAgg")   # Force a known-good backend
import matplotlib.pyplot as plt
from collections import deque
import time


class RadialErrorPlotter(Node):
    def __init__(self):
        super().__init__('radial_error_plotter')

        self.sub = self.create_subscription(
            Float64,
            '/orbit/radial_error',
            self.cb,
            10
        )

        # Storage
        self.t0 = time.time()
        self.t = deque(maxlen=600)
        self.err = deque(maxlen=600)

        # ---- Matplotlib setup ----
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], lw=2)

        self.ax.set_title("Radial Tracking Error")
        self.ax.set_xlabel("Time [s]")
        self.ax.set_ylabel("Radial Error [m]")
        self.ax.grid(True)
        # self.ax.set_ylim(0, 2.0)


        self.fig.show()
        self.fig.canvas.draw()

    def cb(self, msg: Float64):
        now = time.time() - self.t0

        self.t.append(now)
        self.err.append(msg.data)

        self.line.set_data(self.t, self.err)

        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main():
    rclpy.init()
    node = RadialErrorPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
