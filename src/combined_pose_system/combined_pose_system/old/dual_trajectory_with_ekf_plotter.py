#!/usr/bin/env python3

from collections import deque

import rclpy 
from rclpy.node import Node 

from geometry_msgs.msg import PoseWithCovarianceStamped 
from nav_msgs.msg import Odometry 

import matplotlib.pyplot as plt
import numpy as np

from combined_pose_system.utils.utils_ros import yaw_from_quat

class TrajectoryPlotterNode(Node):
    """
    Live 2D trajectory plotter for sanity checking frame alignment.

    - Vision trajectory: /vision_pose/mm_map
    - Marvelmind trajectory: /marvelmind/pose
    - EKF trajectory: /odometry/filtered
    """

    def __init__(self):
        super().__init__('trajectory_plotter_node')

        # Buffers
        self.max_len = 2000
        self.vision_traj = deque(maxlen=self.max_len)
        self.mm_traj = deque(maxlen=self.max_len)
        self.ekf_traj = deque(maxlen=self.max_len)

        # Subscribers
        self.create_subscription(
            PoseWithCovarianceStamped,
            'vision/pose_mm',
            self.cb_vision,
            10
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/marvelmind/pose',
            self.cb_mm,
            10
        )

        self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.cb_ekf,
            10
        )

        # Plot setup
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.timer = self.create_timer(0.2, self.update_plot)

        self.get_logger().info("Trajectory plotter node started")

    def cb_vision(self, msg):
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.vision_traj.append((
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        ))

    def cb_mm(self, msg):
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.mm_traj.append((
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        ))

    def cb_ekf(self, msg: Odometry):
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.ekf_traj.append((
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        ))

    def update_plot(self):
        self.ax.clear()

        if self.vision_traj:
            xv, yv, yawv = zip(*self.vision_traj)
            self.ax.plot(xv, yv, 'b-', label='Vision')
            self.draw_quiver(xv, yv, yawv, color='b')

        if self.mm_traj:
            xm, ym, yawm = zip(*self.mm_traj)
            self.ax.plot(xm, ym, 'r-', label='Marvelmind')
            self.draw_quiver(xm, ym, yawm, color='r')

        if self.ekf_traj:
            xe, ye, yawe = zip(*self.ekf_traj)
            self.ax.plot(xe, ye, 'g-', label='EKF')
            self.draw_quiver(xe, ye, yawe, color='g')

        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlabel('x [m]')
        self.ax.set_ylabel('y [m]')
        self.ax.set_title('Trajectory Comparison (mm_map)')
        self.ax.legend()
        self.ax.grid(True)

        plt.pause(0.001)

    def draw_quiver(self, x, y, yaw, color):
        # Decimate arrows so the plot stays readable
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
            color=color,
            alpha=0.6
        )


def main():
    rclpy.init()
    node = TrajectoryPlotterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()