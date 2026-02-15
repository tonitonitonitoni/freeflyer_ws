#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from collections import deque
import threading


class AccelCompareNode(Node):

    def __init__(self):
        super().__init__("accel_compare_node")

        self.create_subscription(Imu, "/imu/data_accel", self.cb_raw, 50)
        self.create_subscription(Imu, "/imu/data_linear", self.cb_linear, 50)

        self.window = 500
        self.raw_x = deque(maxlen=self.window)
        self.raw_y = deque(maxlen=self.window)
        self.raw_z = deque(maxlen=self.window)

        self.lin_x = deque(maxlen=self.window)
        self.lin_y = deque(maxlen=self.window)
        self.lin_z = deque(maxlen=self.window)

        self.lock = threading.Lock()

        threading.Thread(target=self.plot_loop, daemon=True).start()

        self.get_logger().info("Accel comparison node running.")

    def cb_raw(self, msg: Imu):
        with self.lock:
            self.raw_x.append(msg.linear_acceleration.x)
            self.raw_y.append(msg.linear_acceleration.y)
            self.raw_z.append(msg.linear_acceleration.z)

    def cb_linear(self, msg: Imu):
        with self.lock:
            self.lin_x.append(msg.linear_acceleration.x)
            self.lin_y.append(msg.linear_acceleration.y)
            self.lin_z.append(msg.linear_acceleration.z)

    def plot_loop(self):
        plt.ion()
        fig, ax = plt.subplots()

        while rclpy.ok():
            with self.lock:
                ax.clear()
                ax.plot(self.raw_x, label="raw_x")
                ax.plot(self.raw_y, label="raw_y")
                ax.plot(self.raw_z, label="raw_z")

                ax.plot(self.lin_x, linestyle="--", label="lin_x")
                ax.plot(self.lin_y, linestyle="--", label="lin_y")
                ax.plot(self.lin_z, linestyle="--", label="lin_z")

            ax.legend()
            ax.set_title("Raw vs Linear Acceleration")
            ax.set_ylabel("m/s^2")
            plt.pause(0.05)


def main():
    rclpy.init()
    node = AccelCompareNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
