#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2

class PoseCamCompressed(Node):
    def __init__(self):
        super().__init__('pose_cam_compressed')

        self.pub = self.create_publisher(
            CompressedImage,
            'pose_cam/image/compressed',
            10
        )

        gst = (
            "udpsrc port=5000 caps=application/x-rtp,media=video,encoding-name=H264,payload=96 ! "
            "rtph264depay ! "
            "avdec_h264 ! "
            "videoconvert ! "
            "appsink" 
        )

        self.cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open Pi camera GStreamer pipeline")

        self.timer = self.create_timer(0.033, self.grab_and_publish)

        self.get_logger().info("Pose camera started")

    def grab_and_publish(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        cv2.imshow("Pose Camera", frame)
        cv2.waitKey(1)
        
        # Encode to JPEG
        ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pose_cam"
        msg.format = "jpeg"
        msg.data = jpeg.tobytes()

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PoseCamCompressed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
