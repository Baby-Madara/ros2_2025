#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageFormatConverter(Node):
    def __init__(self):
        super().__init__("image_format_converter")
        self.declare_parameter("input_topic", "/rgb_camera/image_raw/compressedDepth")
        self.declare_parameter("output_topic", "/camera/depth_image_converted")
        self.declare_parameter("output_format", "32FC1")  # Options: '16UC1', '32FC1'

        # Get parameters
        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.output_format = self.get_parameter("output_format").get_parameter_value().string_value

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, self.output_topic, 10)
        self.subscription = self.create_subscription(Image, self.input_topic, self.image_callback, 10)

        self.get_logger().info(f"Subscribed to {self.input_topic}, publishing converted images to {self.output_topic}")

    def image_callback(self, msg):
        try:
            # Convert RGB image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

            # Convert to grayscale (simulating depth data)
            grayscale_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

            # Scale to match depth ranges (simulated example)
            if self.output_format == "16UC1":
                depth_image = np.uint16(grayscale_image * 256)  # Scale to 16-bit
            elif self.output_format == "32FC1":
                depth_image = np.float32(grayscale_image) / 255.0  # Normalize to 0-1 range
            else:
                self.get_logger().error(f"Unsupported output format: {self.output_format}")
                return

            # Convert back to ROS2 Image message
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding=self.output_format)
            depth_msg.header = msg.header  # Retain original message's header

            # Publish the converted image
            self.publisher.publish(depth_msg)
            self.get_logger().info("Published converted depth image.")
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageFormatConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


