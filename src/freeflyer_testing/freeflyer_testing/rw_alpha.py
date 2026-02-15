#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Imu

import csv


class ReactionWheelCalib(Node):

    def __init__(self):
        super().__init__('reaction_wheel_calibration')

        # -------- Publishers --------
        self.pub_cw = self.create_publisher(
            Bool,
            '/freeflyer/reaction_wheel/cw',
            10
        )

        self.pub_ccw = self.create_publisher(
            Bool,
            '/freeflyer/reaction_wheel/ccw',
            10
        )

        # -------- IMU subscription --------
        self.sub_imu = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            100
        )

        # -------- Experiment timing --------
        self.phase_sequence = [
            ('off', 2.0),
            ('cw',  1.5),
            ('off', 2.0),
            ('ccw', 1.5),
            ('off', 2.0)
        ]

        self.current_phase = 0
        self.phase_start_time = self.get_clock().now()

        # -------- Differentiation state --------
        self.prev_gyro = None
        self.prev_time = None

        # -------- CSV --------
        self.csv_file = open('rw_calibration_log.csv', 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'ros_time',
            'command',
            'gyro_z',
            'alpha_z'
        ])

        # -------- Timer --------
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info("Reaction wheel calibration started")

    # --------------------------------------------------
    # Timer handles command sequencing
    # --------------------------------------------------
    def timer_callback(self):

        now = self.get_clock().now()
        elapsed = (now - self.phase_start_time).nanoseconds * 1e-9

        phase_name, phase_duration = self.phase_sequence[self.current_phase]

        if elapsed > phase_duration:
            self.current_phase += 1

            if self.current_phase >= len(self.phase_sequence):
                self.get_logger().info("Experiment complete")
                self.send_command('off')
                self.csv_file.close()
                rclpy.shutdown()
                return

            self.phase_start_time = now
            phase_name, _ = self.phase_sequence[self.current_phase]

        self.send_command(phase_name)

    # --------------------------------------------------
    # Send wheel command
    # --------------------------------------------------
    def send_command(self, mode):

        msg_cw = Bool()
        msg_ccw = Bool()

        if mode == 'cw':
            msg_cw.data = True
            msg_ccw.data = False
            self.current_cmd = 1

        elif mode == 'ccw':
            msg_cw.data = False
            msg_ccw.data = True
            self.current_cmd = -1

        else:
            msg_cw.data = False
            msg_ccw.data = False
            self.current_cmd = 0

        self.pub_cw.publish(msg_cw)
        self.pub_ccw.publish(msg_ccw)

    # --------------------------------------------------
    # IMU callback
    # --------------------------------------------------
    def imu_callback(self, msg: Imu):

        now = self.get_clock().now().nanoseconds * 1e-9
        gyro_z = msg.angular_velocity.z

        if self.prev_time is not None:
            dt = now - self.prev_time

            if dt > 0.0005:  # avoid division by tiny dt
                alpha_z = (gyro_z - self.prev_gyro) / dt
            else:
                alpha_z = 0.0
        else:
            alpha_z = 0.0

        self.writer.writerow([
            now,
            getattr(self, 'current_cmd', 0),
            gyro_z,
            alpha_z
        ])

        self.prev_gyro = gyro_z
        self.prev_time = now


def main(args=None):
    rclpy.init(args=args)
    node = ReactionWheelCalib()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
