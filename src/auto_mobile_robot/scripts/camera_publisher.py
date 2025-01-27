#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.rgb_publisher = self.create_publisher(Image, 'camera/rgb/image_raw', 10)
        self.rgbd_publisher = self.create_publisher(Image, 'camera/rgbd/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.publish_images)

    def publish_images(self):
        # Create a dummy RGB image
        rgb_image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(rgb_image, 'RGB Image', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # Create a dummy RGBD image (simulated depth)
        rgbd_image = np.zeros((480, 640, 4), dtype=np.uint8)
        rgbd_image[..., :3] = rgb_image  # Copy RGB data
        rgbd_image[..., 3] = 255  # Simulated depth channel

        # Publish RGB image
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='bgr8')
        self.rgb_publisher.publish(rgb_msg)

        # Publish RGBD image
        rgbd_msg = self.bridge.cv2_to_imgmsg(rgbd_image, encoding='rgba8')
        self.rgbd_publisher.publish(rgbd_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()