#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        self.path_pub = self.create_publisher(Path, '/freeflyer/path', 10)
        self.create_subscription(
            Odometry,
            '/freeflyer/odometry/filtered',
            self.odom_callback,
            10
        )

        self.path = Path()

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.header = msg.header
        self.path.poses.append(pose)

        self.path_pub.publish(self.path)

def main(args=None):
    rclpy.init()
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

