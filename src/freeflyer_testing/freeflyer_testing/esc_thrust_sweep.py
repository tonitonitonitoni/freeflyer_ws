#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu

import csv


class ThrustSweepNode(Node):

    def __init__(self):
        super().__init__('thrust_sweep_node')

        # ---- Publisher ----
        self.cmd_pub = self.create_publisher(
            Float32,
            '/freeflyer/thrusters/px',
            10
        )

        # ---- IMU ----
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            50
        )

        # ---- Sweep Config ----
        low = np.linspace(0.0, 0.3, 10)
        high = np.linspace(0.3, 1.0, 5)
        self.levels = list(low) + list(high[1:])

        self.hold_time = 4.0
        self.settle_time = 1.0

        self.current_index = 0
        self.current_level = self.levels[0]

        self.last_switch_time = self.get_clock().now()

        # ---- IMU Buffer ----
        self.imu_samples = []

        # ---- Timer ----
        self.timer = self.create_timer(0.02, self.timer_callback)

        # ---- CSV ----
        self.csv_file = open('thrust_sweep_log.csv', 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'cmd_level',
            'avg_ax',
            'avg_ay',
            'avg_az'
        ])

        self.get_logger().info("Improved thrust sweep started")

    # ------------------------------------------------
    # Timer loop
    # ------------------------------------------------
    def timer_callback(self):

        now = self.get_clock().now()
        elapsed = (now - self.last_switch_time).nanoseconds * 1e-9

        # Always publish current command
        msg = Float32()
        msg.data = self.current_level
        self.cmd_pub.publish(msg)

        # Finished?
        if self.current_index >= len(self.levels):
            return

        # Still holding?
        if elapsed < self.hold_time:
            return

        # ---- Compute steady-state average ----
        if len(self.imu_samples) > 0:

            avg_ax = sum(x[0] for x in self.imu_samples) / len(self.imu_samples)
            avg_ay = sum(x[1] for x in self.imu_samples) / len(self.imu_samples)
            avg_az = sum(x[2] for x in self.imu_samples) / len(self.imu_samples)

            self.writer.writerow([
                self.current_level,
                avg_ax,
                avg_ay,
                avg_az
            ])

            self.get_logger().info(
                f"Level {self.current_level:.2f} "
                f"avg_ax={avg_ax:.4f}"
            )

        # ---- Move to next level ----
        self.current_index += 1

        if self.current_index >= len(self.levels):
            self.get_logger().info("Sweep complete")
            self.cmd_pub.publish(Float32(data=0.0))
            self.csv_file.close()
            rclpy.shutdown()
            return

        self.current_level = self.levels[self.current_index]
        self.last_switch_time = now
        self.imu_samples = []

    # ------------------------------------------------
    # IMU callback
    # ------------------------------------------------
    def imu_callback(self, msg: Imu):

        now = self.get_clock().now()
        elapsed = (now - self.last_switch_time).nanoseconds * 1e-9

        # Ignore transient
        if elapsed < self.settle_time:
            return

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        self.imu_samples.append((ax, ay, az))


def main(args=None):
    rclpy.init(args=args)
    node = ThrustSweepNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
