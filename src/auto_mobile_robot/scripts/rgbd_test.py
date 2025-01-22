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
        
        # Timer to publish the point cloud at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_pointcloud)  # Publish at 1 Hz

    def publish_pointcloud(self):
        # Dummy RGB frame (5x5, red image)
        rgb_frame = np.zeros((5, 5, 3), dtype=np.uint8)
        rgb_frame[:, :, 0] =  255  # Red channel

        # Dummy Depth frame (5x5, random depth between 0.5 and 2.0 meters)
        depth_frame = np.random.uniform(0.5, 2.0, (5, 5)).astype(np.float32)

        # Create point cloud data
        points = []
        for v in range(depth_frame.shape[0]):
            for u in range(depth_frame.shape[1]):
                z = depth_frame[v, u]
                if z == 0:  # Skip points with zero depth
                    continue
                x = (u - depth_frame.shape[1] / 2) * z * 0.1  # Scale for visualization
                y = (v - depth_frame.shape[0] / 2) * z * 0.1
                r, g, b = rgb_frame[v, u]
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]  # Pack RGB into single int
                points.append([x, y, z, rgb])

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

        pointcloud_msg = PointCloud2()
        pointcloud_msg.header = header
        pointcloud_msg.height = 1
        pointcloud_msg.width = len(points)
        pointcloud_msg.fields = fields
        pointcloud_msg.is_bigendian = False
        pointcloud_msg.point_step = 16
        pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width
        pointcloud_msg.is_dense = True
        pointcloud_msg.data = np.array(points, dtype=np.float32).tobytes()

        # Publish the PointCloud2 message
        self.pc_pub.publish(pointcloud_msg)
        self.get_logger().info('Published RGB-D PointCloud')

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
