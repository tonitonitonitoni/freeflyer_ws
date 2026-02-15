#!/usr/bin/env python3

import sys
import math
from collections import deque

# Run with ros2 run combined_pose_system single_trajectory_plotter mm 
# (mm for Marvelmind, vision for /vision_pose/mm_map) 
import rclpy 
from rclpy.node import Node 

from geometry_msgs.msg import PoseWithCovarianceStamped 
from nav_msgs.msg import Odometry  

import matplotlib.pyplot as plt
import numpy as np

from combined_pose_system.utils.utils_ros import yaw_from_quat

class SingleTrajectoryPlotter(Node):
    """
    Live trajectory plotter for a single pose source.

    Usage:
      ros2 run combined_pose_system single_trajectory_plotter mm
      ros2 run combined_pose_system single_trajectory_plotter vision
      ros2 run combined_pose_system single_trajectory_plotter ekf
    """

    def __init__(self, mode: str):
        super().__init__('single_trajectory_plotter')
        self.cb = self.cb_pose
        
        if mode not in ('mm', 'vision', 'ekf'):
            raise RuntimeError("Mode must be 'mm', 'vision', or 'ekf'")

        if mode == 'mm':
            topic = '/marvelmind/pose'
            msg_type = PoseWithCovarianceStamped
            label = 'Marvelmind'
            color = 'r'

        elif mode == 'vision':
            topic = 'vision/pose_mm'
            msg_type = PoseWithCovarianceStamped
            label = 'Vision'
            color = 'b'

        else:  # ekf
            topic = '/odometry/filtered'
            msg_type = Odometry
            label = 'EKF'
            color = 'g'

        self.label = label
        self.color = color

        self.max_len = 2000
        self.traj = deque(maxlen=self.max_len)

        self.create_subscription(
            msg_type,
            topic,
            self.cb,
            10
        )

        # Plot setup
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.timer = self.create_timer(0.2, self.update_plot)

        self.get_logger().info(
            f"Single trajectory plotter started for {label} "
            f"({topic})"
        )

    def cb_pose(self, msg):
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.traj.append((
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        ))

    def cb_ekf(self, msg: Odometry):
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.traj.append((
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        ))

    def update_plot(self):
        self.ax.clear()

        if self.traj:
            x, y, yaw = zip(*self.traj)
            self.ax.plot(x, y, self.color + '-', label=self.label)
            self.draw_quiver(x, y, yaw)

        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlabel('x [m]')
        self.ax.set_ylabel('y [m]')
        self.ax.set_title(f'{self.label} Trajectory')
        self.ax.legend()
        self.ax.grid(True)

        plt.pause(0.001)

    def draw_quiver(self, x, y, yaw):
        step = max(1, len(x) // 30)
        xq = np.array(x[::step])
        yq = np.array(y[::step])
        yawq = np.array(yaw[::step])

        u = np.cos(yawq)
        v = np.sin(yawq)

        self.ax.quiver(
            xq, yq, u, v,
            angles='xy',
            scale_units='xy',
            scale=8.0,
            color=self.color,
            alpha=0.6
        )


def main():
    if len(sys.argv) < 2:
        print("Usage: single_trajectory_plotter.py [mm|vision|ekf]")
        sys.exit(1)

    mode = sys.argv[1]

    rclpy.init()
    node = SingleTrajectoryPlotter(mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
