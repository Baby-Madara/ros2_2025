#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header
import numpy as np
import struct
import tf2_ros
import geometry_msgs.msg

class RGBDPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('rgbd_pointcloud_publisher')
        
        # Initialize publisher for PointCloud2 data
        self.pc_pub = self.create_publisher(PointCloud2, 'camera/pointcloud', 10)
        
        # Initialize TF broadcaster to publish transformations
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.frequency = 30
        self.width   = int(320/1)
        self.height  = int(240/1)

        self.counter = 0
        
        # Timer to publish the point cloud at 30 Hz
        self.timer = self.create_timer(1/self.frequency, self.publish_pointcloud)  # Publish at 30 Hz

    def publish_pointcloud(self):
        # Dummy RGB frame (5x5, red image)
        rgb_frame = np.zeros((self.width, self.height, 3), dtype=np.uint8)
        rgb_frame[:, :, 0] =  np.random.randint(255)  # R channel
        rgb_frame[:, :, 1] =  np.random.randint(255)  # G channel
        rgb_frame[:, :, 2] =  np.random.randint(255)  # B channel

        # Dummy Depth frame (320x240, random depth between 0.5 and 2.0 meters)
        depth_frame = np.random.uniform(0.5, 2.0, (self.height, self.width)).astype(np.float32)

        # Generate 3D coordinates
        u, v = np.meshgrid(np.arange(self.width), np.arange(self.height))
        z = depth_frame.flatten()
        x = (u.flatten() - self.width / 2)  * z * 0.01
        y = (v.flatten() - self.height / 2) * z * 0.01

        # Pack RGB data
        rgb = rgb_frame.reshape(-1, 3)
        rgb_packed = (rgb[:, 0].astype(np.uint32) |  # Red channel
                    np.left_shift(rgb[:, 1].astype(np.uint32), 8) |  # Green channel
                    np.left_shift(rgb[:, 2].astype(np.uint32), 16))  # Blue channel

        # Combine into a point cloud array
        points = np.stack([x, y, z, rgb_packed.astype(np.float32)], axis=-1)



        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]

        pointcloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            fields=fields,
            is_bigendian=False,
            point_step=16,
            row_step=16 * len(points),
            is_dense=True,
            data = points.astype(np.float32).tobytes()
        )

        # pointcloud_msg.header = header,
        # pointcloud_msg.height = 1,
        # pointcloud_msg.width = len(points),
        # pointcloud_msg.fields = fields,
        # pointcloud_msg.is_bigendian = False,
        # pointcloud_msg.point_step = 16,
        # pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width,
        # pointcloud_msg.is_dense = True,
        # pointcloud_msg.data = np.array(points, dtype=np.float32).tobytes(),

        # Publish the PointCloud2 message
        self.pc_pub.publish(pointcloud_msg)
        # self.counter += 1
        # self.get_logger().info(f'Published RGB-D PointCloud. counter = {self.counter}')

        # Create and publish the TF transformation (camera_link -> world)
        self.publish_tf()

    def publish_tf(self):
        # Create a transform from camera_link to world
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'camera_link'

        # Set translation (e.g., camera located at (1, 1, 1) in world frame)
        transform.transform.translation.x = 1.0
        transform.transform.translation.y = 1.0
        transform.transform.translation.z = 1.0

        # Set rotation (identity rotation, no rotation)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        # Send the transform
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info('Published TF: camera_link to world')

def main(args=None):
    rclpy.init(args=args)
    node = RGBDPointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
