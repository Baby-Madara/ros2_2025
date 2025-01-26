#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from cv_bridge import CvBridge
from std_msgs.msg import Header
import numpy as np
import tf2_ros
import geometry_msgs.msg
import freenect
import cv2
import math








class KinectPublisher(Node):
    def __init__(self):
        super().__init__('kinect_publisher')
        self.counter     = 0
        # Camera parameters (calibrated values from research papers):cite[8]
        self.frequency   = 30
        self.width       = 640               # Kinect resolution width
        self.height      = 480               # Kinect resolution height
        self.fx          = 582.624481        # Calibrated focal length x:cite[8]
        self.fy          = 582.691032        # Calibrated focal length y:cite[8]
        self.cx          = 318.569184        # Calibrated principal point x:cite[8]
        self.cy          = 257.382996        # Calibrated principal point y:cite[8]
        
        self.centerX = int(self.width   / 2)
        self.centerY = int(self.height  / 2)
        self.tanW = math.tan(57/2*math.pi/180)
        self.tanH = math.tan(43/2*math.pi/180)
        
        
        # Precompute grid for faster point cloud generation:cite[1]:cite[9]
        self.u, self.v   = np.meshgrid(np.arange(self.width), np.arange(self.height))
        self.u_flat      = self.u.flatten()
        self.v_flat      = self.v.flatten()
        
        # Initialize publishers
        self.image_pub            = self.create_publisher(Image,        'camera/rgb/image_raw',     10)
        self.camera_info_pub      = self.create_publisher(CameraInfo,   'camera/rgb/camera_info',   10)
        self.depth_image_pub      = self.create_publisher(Image,        'camera/depth/image_raw',   10)
        self.pc_pub               = self.create_publisher(PointCloud2,  'camera/depth/pointcloud',  10)
        self.tf_broadcaster       = tf2_ros.TransformBroadcaster(self)
        self.timer                = self.create_timer(1 / self.frequency, self.publish_data)

    def get_video(self):
        frame, _ = freenect.sync_get_video()
        # return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame
        
        # # Generate dummy RGB frames
        # rgb_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # # # for different colors
        # # rgb_frame[:, :, 0] = np.random.randint(0, 255, (self.height, self.width))  # R channel
        # # rgb_frame[:, :, 1] = np.random.randint(0, 255, (self.height, self.width))  # G channel
        # # rgb_frame[:, :, 2] = np.random.randint(0, 255, (self.height, self.width))  # B channel
        
        # # for solid color
        # rgb_frame[:, :, 0] = 0b10000000 #0b00001111 #np.random.randint(255)  # R channel
        # rgb_frame[:, :, 1] = 0b10000000 #0b00110011 #np.random.randint(255)  # G channel
        # rgb_frame[:, :, 2] = 0b10000000 #0b01010101 #np.random.randint(255)  # B channel
        # return rgb_frame

    def get_depth(self):
        depth, _ = freenect.sync_get_depth()
        # registered_depth = freenect.depth_registration(depth)  # Align depth to RGB

        # Normalize depth values to range [0, 15] and scale them to meters
        # depth_normalized = depth.astype(np.float32) * (6.2/2047.0)
        # depth_normalized = (depth.astype(np.float32) * (6.2/2047.0) - 1.1)*1.3421 + 0.45
        depth_normalized = (depth.astype(np.float32) /246.0035) - 1.026316
        return depth_normalized

        # # Apply non-linear correction from research papers:cite[5]:cite[8]
        # depth_corrected = depth.astype(np.float32) * (4.0/2047.0)
        # depth_corrected = 1.2 * np.tan(depth_corrected / 2.8425 + 1.1863)  # Non-linear correction
        # return depth_corrected

    def publish_data(self):
        # Capture Kinect frames
        rgb_frame    = self.get_video()
        depth_frame  = self.get_depth()

        self.publish_camera_info()
        self.publish_rgb_image(rgb_frame)
        self.publish_depth_image(depth_frame)
        self.publish_pointcloud(rgb_frame, depth_frame)
        self.publish_tf()

        self.get_logger().info(f'Published Kinect data.')

    def publish_pointcloud(self, rgb_frame, depth_frame):
        # Use precomputed grid for faster calculations

        #### d = depth_frame.flatten()
        #### self.tanX = ({x} - self.centerX) * self.tanW / self.centerX
        #### self.tanY = ({y} - self.centerY) * self.tanH / self.centerY
        #### z = d / math.sqrt(1 + self.tanX**2 + self.tanY**2)
        #### x = self.tanX * z
        #### y = self.tanY * z

        # z = depth_frame.flatten()
        # x = (self.u_flat - self.cx) * z / self.fx
        # y = (self.v_flat - self.cy) * z / self.fy

        z = depth_frame.flatten()
        x = (self.u_flat - self.width/2)  /(self.width/2)  * z *self.tanW
        y = (self.v_flat - self.height/2) /(self.height/2) * z *self.tanH

        # Filter invalid points
        mask        = (z > 0.35) & (z < 4.0)
        x           = x[mask]
        y           = y[mask]
        z           = z[mask]
        rgb         = rgb_frame.reshape(-1, 3)[mask]

        rgb_packed = (
            np.left_shift(rgb[:,0].astype(np.uint32), 16)    |    # Red
            np.left_shift(rgb[:,1].astype(np.uint32),  8)    |    # Green
            rgb[:,2].astype(np.uint32)                            # Blue
        ).view(np.float32)
        points = np.stack([x, y, z, rgb_packed], axis=-1)

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

        self.image_pub.publish(
            Image(
                header        = header,
                height        = self.height,
                width         = self.width,
                encoding      = 'rgb8',
                is_bigendian  = False,
                step          = self.width * 3,
                data          = rgb_frame.tobytes(),
            )
        )

    def publish_depth_image(self, depth_frame):
        # Apply a color map to the depth image
        depth_colormap = cv2.applyColorMap((depth_frame * 255).astype(np.uint8), cv2.COLORMAP_JET) # cv2.COLORMAP_VIRIDIS | cv2.COLORMAP_JET

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
    try:
        rclpy.init(args=args)
        node = KinectPublisher()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        freenect.sync_stop()

if __name__ == '__main__':
    main()



