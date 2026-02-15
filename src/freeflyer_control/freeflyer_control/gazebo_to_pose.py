#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped


class GazeboTruthToPose(Node):

    def __init__(self):
        super().__init__("gazebo_truth_to_pose")

        self.declare_parameter("model_name", "freeflyer")
        self.declare_parameter("model_states_topic", "model_states")
        self.declare_parameter("frame_id", "map")

        self.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, True)
        ])

        self.model_name = self.get_parameter("model_name").value
        self.states_topic = self.get_parameter("model_states_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.xy_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/localization/xy",
            10
        )

        self.yaw_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/localization/yaw",
            10
        )

        self.sub = self.create_subscription(
            ModelStates,
            self.states_topic,
            self.cb,
            qos
        )

        self.get_logger().info("Gazebo truth → simulated sensors running")

    def cb(self, msg: ModelStates):
        try:
            i = msg.name.index(self.model_name)
        except ValueError:
            return

        pose = msg.pose[i]

        now = self.get_clock().now().to_msg()

        # -------------------
        # XY SENSOR
        # -------------------
        xy = PoseWithCovarianceStamped()
        xy.header.stamp = now
        xy.header.frame_id = self.frame_id
        xy.pose.pose.position.x = pose.position.x
        xy.pose.pose.position.y = pose.position.y

        cov = [0.0]*36
        cov[0] = 1e-4     # x noise
        cov[7] = 1e-4     # y noise
        cov[14] = 1e6     # z unknown
        cov[21] = 1e6
        cov[28] = 1e6
        cov[35] = 1e6
        xy.pose.covariance = cov

        self.xy_pub.publish(xy)

        # -------------------
        # YAW SENSOR
        # -------------------
        yaw = PoseWithCovarianceStamped()
        yaw.header.stamp = now
        yaw.header.frame_id = self.frame_id

        yaw.pose.pose.orientation = pose.orientation

        cov = [0.0]*36
        cov[0] = 1e6
        cov[7] = 1e6
        cov[14] = 1e6
        cov[21] = 1e6
        cov[28] = 1e6
        cov[35] = 1e-4   # yaw noise
        yaw.pose.covariance = cov

        self.yaw_pub.publish(yaw)


def main():
    rclpy.init()
    node = GazeboTruthToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
