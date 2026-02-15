#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class GstCamera(Node):
    def __init__(self):
        super().__init__('gst_camera')
        self.pub = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()

        gst = (
            "udpsrc port=5000 caps=application/x-rtp,media=video,encoding-name=H264,payload=96 ! "
            "rtph264depay ! avdec_h264 ! videoconvert ! appsink"
        )

        self.cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open GStreamer pipeline")

        self.timer = self.create_timer(0.033, self.grab_and_show)  # ~30 Hz

    def grab_and_show(self):

        ret, frame = self.cap.read()
        if not ret:
            return

        cv2.imshow("Flyer Camera", frame)
        cv2.waitKey(1)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "flyer_cam"
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = GstCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
