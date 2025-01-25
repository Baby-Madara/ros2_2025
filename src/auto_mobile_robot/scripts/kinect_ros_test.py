#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from std_msgs.msg import Header
import numpy as np
import tf2_ros
import geometry_msgs.msg
import freenect
import cv2

class KinectPublisher(Node):
    def __init__(self):
        super().__init__('kinect_publisher')
        
        self.counter     = 0
        # Camera parameters
        self.frequency   = 30
        self.width       = 640               # Kinect resolution width
        self.height      = 480               # Kinect resolution height
        self.fx          = 575.8             # Approximate focal length for Kinect in x
        self.fy          = 575.8             # Approximate focal length for Kinect in y
        self.cx          = self.width / 2    # Principal point x
        self.cy          = self.height / 2   # Principal point y
        
        # Initialize publishers
        self.tf_broadcaster       = tf2_ros.TransformBroadcaster(self)
        self.image_pub            = self.create_publisher(Image,        'camera/rgb/image_raw',     10)
        self.camera_info_pub      = self.create_publisher(CameraInfo,   'camera/rgb/camera_info',   10)
        self.depth_image_pub      = self.create_publisher(Image,        'camera/depth/image_raw',   10)
        self.pc_pub               = self.create_publisher(PointCloud2,  'camera/depth/pointcloud',  10)
        self.timer                = self.create_timer(1 / self.frequency, self.publish_data)

    def get_video(self):
        frame, _ = freenect.sync_get_video()
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    def get_depth(self):
        depth, _ = freenect.sync_get_depth()
        # Normalize depth values to range [0, 1] and scale them to meters
        depth_normalized = depth.astype(np.float32) / 1000.0
        return depth_normalized

    def publish_data(self):
        # Capture Kinect frames
        rgb_frame    = self.get_video()
        depth_frame  = self.get_depth()

        self.publish_camera_info()
        self.publish_rgb_image(rgb_frame)
        self.publish_depth_image(depth_frame)  # Publish the depth image with color map
        self.publish_pointcloud(rgb_frame, depth_frame)
        self.publish_tf()

        self.counter+=1
        self.get_logger().info(f'Published Kinect data. {self.counter}')

    def publish_pointcloud(self, rgb_frame, depth_frame):
        # Generate 3D coordinates using depth data
        u, v = np.meshgrid(np.arange(self.width), np.arange(self.height))
        z = depth_frame.flatten()  # Depth in meters
        x = (u.flatten() - self.cx) * z / self.fx
        y = (v.flatten() - self.cy) * z / self.fy

        # Handle invalid or zero depth values (optional)
        mask = z > 0  # Filter out zero or invalid depth values
        x = x[mask]
        y = y[mask]
        z = z[mask]
        # self.get_logger().info(f'Valid points: {mask}')
        rgb = rgb_frame.reshape(-1, 3)[mask]

        # Pack RGB data
        rgb_packed = (rgb[:, 0].astype(np.uint32) |
                        np.left_shift(rgb[:, 1].astype(np.uint32), 8) |
                        np.left_shift(rgb[:, 2].astype(np.uint32), 16)
                    )

        # Combine into a point cloud array
        points = np.stack([x, y, z, rgb_packed.astype(np.float32)], axis=-1)

        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link'

        fields = [
            PointField(name='x',    offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',    offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',    offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb',  offset=12, datatype=PointField.UINT32,  count=1),
        ]

        self.pc_pub.publish(
            PointCloud2(
                header        = header,
                height        = 1,
                width         = len(points),
                fields        = fields,
                is_bigendian  = False,
                point_step    = 16,
                row_step      = 16 * len(points),
                is_dense      = True,
                data          = points.astype(np.float32).tobytes()
            )
        )

    def publish_rgb_image(self, rgb_frame):
        # Create Image message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link'

        # image_msg = 

        self.image_pub.publish(
            Image(
                header        = header,
                height        = self.height,
                width         = self.width,
                encoding      = 'bgr8',                 # Kinect gives BGR format
                is_bigendian  = False,
                step          = self.width * 3,
                data          = rgb_frame.tobytes(),
            )
        )

    def publish_depth_image(self, depth_frame):
        # Apply a color map to the depth image
        depth_colormap = cv2.applyColorMap((depth_frame * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)

        # Create Image message for depth
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link'

        self.depth_image_pub.publish(
            Image(
                header        = header,
                height        = self.height,
                width         = self.width,
                encoding      = 'bgr8',  # Depth as color-mapped image
                is_bigendian  = False,
                step          = self.width * 3,
                data          = depth_colormap.tobytes(),
            )
        )

    def publish_camera_info(self):
        # Create CameraInfo message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link'
        
        self.camera_info_pub.publish(
            CameraInfo(
                header              = header,
                height              = self.height,
                width               = self.width,
                distortion_model    = 'plumb_bob',
                d                   = [0.0, 0.0, 0.0, 0.0, 0.0],    # No distortion
                k                   = [ self.fx, 0.0, self.cx,
                                        0.0, self.fy, self.cy,
                                        0.0, 0.0, 1.0],             # Intrinsic matrix
                r                   = [ 1.0, 0.0, 0.0,
                                        0.0, 1.0, 0.0,
                                        0.0, 0.0, 1.0],             # Rectification matrix
                p                   = [ self.fx, 0.0, self.cx, 0.0,
                                        0.0, self.fy, self.cy, 0.0,
                                        0.0, 0.0, 1.0, 0.0],        # Projection matrix
            )
        )

    def publish_tf(self):
        # Create a transform from camera_link to world
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'camera_link'

        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 1.0

        transform.transform.rotation.x = +0.5
        transform.transform.rotation.y = -0.5
        transform.transform.rotation.z = +0.5
        transform.transform.rotation.w = -0.5

        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = KinectPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()