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
from scipy.ndimage import affine_transform
import matplotlib.pyplot as plt
import math
import logging
import time


class KinectPublisher(Node):
    def __init__(self):
        super().__init__('kinect_publisher')
        self.counter     = 0
        self.frequency   = 30  # Hz
        self.timestamps  = []  # List to store timestamps of published messages

        # fast may make the color pixel be projected twice
        self.fast = 'alg_2'
        self.fast = 'alg_3'
        self.fast = 'alg_1'


        # Camera parameters (calibrated values from research papers):cite[8]
        self.width                       = 640               # Kinect resolution width
        self.height                      = 480               # Kinect resolution height
        self.fx_ir                       = 580 #582.624481        # Calibrated focal length x:cite[8]
        self.fy_ir                       = 580 #582.691032        # Calibrated focal length y:cite[8]
        self.cx                          = 318.569184        # Calibrated principal point x:cite[8]
        self.cy                          = 257.382996        # Calibrated principal point y:cite[8]
        self.inv_fx_ir, self.inv_fy_ir   = 1 / self.fx_ir, 1 / self.fy_ir  # Precompute inverses
        self.fx_rgb = self.fy_rgb        = 534  # RGB focal length

        self.centerX = int(self.width   / 2)
        self.centerY = int(self.height  / 2)
        
        self.half_width  = self.width  // 2
        self.half_height = self.height // 2
        
        self.a, self.b, self.c, self.d = 0.08367558, 4072.170, 1.3016675, 0.061144379

        # Extrinsic Matrix (Depth → RGB)
        T              = np.eye(4, dtype=np.float32)

        R_rgb_to_depth = self.rpy_to_rotation_matrix(-2.5,-0.4, 0)
        T_rgb_to_depth = np.array([-0.0254, -0.00013, -0.00218 ])  # Translation in meters
        
        T[:3, :3]   = R_rgb_to_depth # np.eye(3, dtype=np.float32)
        T[:3, 3]    = T_rgb_to_depth
        self.T_inv  = T # np.linalg.inv(T)

        self.point_cloud_header = Header()
        self.point_cloud_header.frame_id = 'camera_link'
        self.fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32,  count=1),
        ]


        # Precompute grid for faster point cloud generation:cite[1]:cite[9]
        self.u, self.v   = np.meshgrid(np.arange(self.width), np.arange(self.height))
        self.u_flat      = self.u.flatten()
        self.v_flat      = self.v.flatten()
        self.crop_filter = (np.logical_and(np.logical_and(self.u_flat > 17, self.u_flat < 588), np.logical_and(self.v_flat > 45, self.v_flat < 479))).flatten() # 4, 595 | 40, 480

        
        # Initialize publishers
        self.image_pub            = self.create_publisher(Image,        'camera/rgb/image_raw',     10)
        self.camera_info_pub      = self.create_publisher(CameraInfo,   'camera/rgb/camera_info',   10)
        self.depth_image_pub      = self.create_publisher(Image,        'camera/depth/image_raw',   10)
        self.pc_pub               = self.create_publisher(PointCloud2,  'camera/depth/pointcloud',  10)
        self.tf_broadcaster       = tf2_ros.TransformBroadcaster(self)
        self.timer                = self.create_timer(1 / self.frequency, self.publish_data)





        '''
        # mean of the center rectangle
        # h, w = depth_frame.shape  # 480 (height), 640 (width)
        # h_start, h_end = h // 3, 2 * (h // 3)  # Vertical bounds
        # w_start, w_end = w // 3, 2 * (w // 3)  # Horizontal bounds
        # # Crop the middle rectangle
        # middle_rectangle = depth_frame[h_start:h_end, w_start:w_end]
        # self.get_logger().info(f'avg_z: {np.mean(middle_rectangle.flatten())}')
        '''

    def publish_pointcloud(self, rgb_frame, depth_frame):

        depth = depth_frame.astype(np.float32).ravel()
        self.z = np.abs(self.a * np.tan(depth / self.b + self.c) + self.d)

        self.x = (self.u_flat - self.half_width) * self.z * self.inv_fx_ir
        self.y = (self.v_flat - self.half_height) * self.z * self.inv_fy_ir



        if self.fast == 'alg_1':
            # Homogeneous transformation without extra arrays
            self.points3D = self.T_inv @ np.vstack((self.x, self.y, self.z, np.ones_like(self.z)))
            self.x_t, self.y_t, self.z_t = self.points3D[:3]

            self.proj_u = ((self.x_t * self.fx_rgb / self.z_t) + self.half_width  - 0.5).astype(np.int32)
            self.proj_v = ((self.y_t * self.fy_rgb / self.z_t) + self.half_height - 0.5).astype(np.int32)

            # Mask valid indices
            self.valid_mask = (self.proj_u >= 0) & (self.proj_u < self.width) & (self.proj_v >= 0) & (self.proj_v < self.height)

            # Pack RGB efficiently
            self.rgb = rgb_frame.reshape(-1, 3).astype(np.uint32)
            self.rgb_packed = np.zeros_like(self.z_t, dtype=np.float32)
            self.rgb_packed_view = ((self.rgb[:, 0] << 16) | (self.rgb[:, 1] << 8) | self.rgb[:, 2]).view(np.float32)
            self.rgb_packed[self.valid_mask] = self.rgb_packed_view[self.proj_v[self.valid_mask] * self.width + self.proj_u[self.valid_mask]]

            # Apply final mask (crop + depth range + validity check)
            self.final_mask = (self.z_t > 0.5) & (self.z_t < 9.0) & self.valid_mask #& self.crop_filter
            self.points3D = np.vstack((self.x_t[self.final_mask], self.y_t[self.final_mask], self.z_t[self.final_mask], self.rgb_packed[self.final_mask])).T  # Shape: (M, 4)
        
        elif self.fast == 'alg_2':
            # Homogeneous coordinates transformation (preallocating memory for speed)
            self.points3D = (self.T_inv @ np.column_stack((self.x, self.y, self.z, np.ones_like(self.z))).T).T  # Shape: (N, 4)

            # Efficient sorting by self.z values
            sorted_indices = np.argsort(self.points3D[:, 2])  # Can use np.argpartition for partial sorting
            self.points3D       = self.points3D[sorted_indices]  

            # Extract sorted coordinates
            self.x_t, self.y_t, self.z_t = self.points3D[:, 0], self.points3D[:, 1], self.points3D[:, 2]

            self.proj_u   = ((self.x_t * self.fx_rgb / self.z_t) + self.half_width - 0.5).astype(np.int32)
            self.proj_v   = ((self.y_t * self.fy_rgb / self.z_t) + self.half_height - 0.5).astype(np.int32)
            proj_uv  = np.column_stack((self.proj_u, self.proj_v))

            # Faster duplicate filtering using lexsort
            self.valid_mask      = (self.proj_u >= 0) & (self.proj_u < self.width) & (self.proj_v >= 0) & (self.proj_v < self.height)
            proj_uv_valid   = proj_uv[self.valid_mask]
            sort_order      = np.lexsort((proj_uv_valid[:, 1], proj_uv_valid[:, 0]))  # Sort by (u, v)
            sorted_proj_uv  = proj_uv_valid[sort_order]
            
            # Identify first occurrences
            unique_mask     = np.concatenate(([True], np.any(sorted_proj_uv[1:] != sorted_proj_uv[:-1], axis=1)))
            
            # Map back to original self.valid_mask indices
            first_occurrence_mask                           = np.zeros(self.valid_mask.sum(), dtype=bool)
            first_occurrence_mask[sort_order[unique_mask]]  = True
            self.valid_mask[self.valid_mask]                         &= first_occurrence_mask


            # Pack RGB (memory-efficient)
            self.rgb_packed       = np.zeros_like(self.z_t, dtype=np.float32)
            self.rgb              = rgb_frame.reshape(-1, 3).astype(np.uint32)
            self.rgb_packed_view  = ((self.rgb[:, 0] << 16) | (self.rgb[:, 1] << 8) | self.rgb[:, 2]).view(np.float32)

            # Assign colors only where valid
            valid_indices          = np.ravel_multi_index((self.proj_v[self.valid_mask], self.proj_u[self.valid_mask]), (self.height, self.width))
            self.rgb_packed[self.valid_mask] = self.rgb_packed_view[valid_indices]

            # Apply final depth filtering
            self.final_mask      = (self.z_t > 0.49) & (self.z_t < 9.0) & self.valid_mask & self.crop_filter
            self.points3D        = self.points3D[self.final_mask]
            self.points3D[:, 3]  = self.rgb_packed[self.final_mask]
        
        elif self.fast == 'alg_3':

            # Transform to RGB camera frame
            self.points3D = (self.T_inv @ np.column_stack((self.x, self.y, self.z, np.ones_like(self.z))).T).T  # Shape: (N, 4)
            self.x_t, self.y_t, self.z_t = self.points3D[:, 0], self.points3D[:, 1], self.points3D[:, 2]

            # Project to RGB frame
            self.proj_u = ((self.x_t * self.fx_rgb / self.z_t) + self.half_width - 0.5).astype(np.int32)
            self.proj_v = ((self.y_t * self.fy_rgb / self.z_t) + self.half_height - 0.5).astype(np.int32)

            # Create a 640x480x4 array initialized with (inf, inf, inf, 0)
            new_ar = np.full((self.height, self.width, 4), [np.inf, np.inf, np.inf, 0], dtype=np.float32)

            # Flatten RGB frame and pack color
            self.rgb = rgb_frame.reshape(-1, 3).astype(np.uint32)
            self.rgb_packed = ((self.rgb[:, 0] << 16) | (self.rgb[:, 1] << 8) | self.rgb[:, 2]).view(np.float32)

            # Flatten indices
            self.valid_mask = (self.proj_u >= 0) & (self.proj_u < self.width) & (self.proj_v >= 0) & (self.proj_v < self.height)
            flat_indices = np.ravel_multi_index((self.proj_v[self.valid_mask], self.proj_u[self.valid_mask]), (self.height, self.width))

            # Get unique indices while keeping the smallest Z
            sort_order = np.argsort(self.z_t[self.valid_mask])  # Sorting ensures nearest depth is prioritized
            sorted_indices = flat_indices[sort_order]
            _, unique_idx = np.unique(sorted_indices, return_index=True)  # Keep only first occurrence (nearest Z)

            # Map back to selected points
            final_idx = sort_order[unique_idx]
            proj_u_f, proj_v_f = self.proj_u[self.valid_mask][final_idx], self.proj_v[self.valid_mask][final_idx]

            # Assign values
            new_ar[proj_v_f, proj_u_f, :3] = np.column_stack((self.x_t[self.valid_mask][final_idx], 
                                                            self.y_t[self.valid_mask][final_idx], 
                                                            self.z_t[self.valid_mask][final_idx]))
            new_ar[proj_v_f, proj_u_f, 3] = self.rgb_packed[self.valid_mask][final_idx]  # Assign RGB packed color

            # Reshape to (N, 4) format
            mask = (new_ar[:, :, 2] > 0.5) & (new_ar[:, :, 2] < 9.0)  # Apply depth range filtering
            self.points3D = new_ar[mask].reshape(-1, 4)

        # Construct PointCloud2 message
        self.point_cloud_header.stamp = self.get_clock().now().to_msg()

        self.pc_pub.publish(
            PointCloud2(
                header        = self.point_cloud_header,
                height        = 1,
                width         = len(self.points3D),
                fields        = self.fields,
                is_bigendian  = False,
                point_step    = 16,
                row_step      = 16 * len(self.points3D),
                is_dense      = True,
                data          = self.points3D.astype(np.float32).tobytes()
            )
        )



    def publish_depth_image(self, depth_frame):
        # Apply a color map to the depth image
        # depth_colormap = cv2.applyColorMap((depth_frame* 255).astype(np.uint8), cv2.COLORMAP_JET) # cv2.COLORMAP_VIRIDIS | cv2.COLORMAP_JET
        depth_colormap = cv2.applyColorMap((depth_frame/1200.0 * 255).astype(np.uint8), cv2.COLORMAP_JET) # cv2.COLORMAP_VIRIDIS | cv2.COLORMAP_JET

        # Create Image message for depth
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link'

        self.depth_image_pub.publish(
            Image(
                header        = header,
                height        = self.height,
                width         = self.width,
                encoding      = 'bgr8',  # Depth as color-mapped image 'bgr8'
                is_bigendian  = False,
                step          = self.width * 3,
                data          = depth_colormap.tobytes(),
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

                d                   = [0.0,      0.0,     0.0,       0.0,      0.0],   # No distortion

                k                   = [ self.fx_ir, 0.0,     self.cx,
                                        0.0,     self.fy_ir, self.cy,
                                        0.0,     0.0,     1.0],                        # Intrinsic matrix

                r                   = [ 1.0,     0.0,     0.0,
                                        0.0,     1.0,     0.0,
                                        0.0,     0.0,     1.0],                        # Rectification matrix

                p                   = [ 535,  0,       self.width/2  ,0,
                                        0,        535, self.height/2 ,0,
                                        0,        0,       1,             0],

                # p=                 [ self.fx_ir, 0.0,     self.cx,   0.0,
                #                         0.0,     self.fy_ir, self.cy,   0.0,
                #                         0.0,     0.0,     1.0,       0.0]             # Projection matrix
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
        transform.transform.translation.z = 0.9

        transform.transform.rotation.x = +0.5
        transform.transform.rotation.y = -0.5
        transform.transform.rotation.z = +0.5
        transform.transform.rotation.w = -0.5

        self.tf_broadcaster.sendTransform(transform)

    def publish_data(self):
        # Capture Kinect frames
        rgb_frame    = self.get_video()
        depth_frame  = self.get_depth()

        self.fps_calculator()

        self.publish_camera_info()
        self.publish_rgb_image(rgb_frame)
        self.publish_depth_image(depth_frame)
        self.publish_pointcloud(rgb_frame, depth_frame)
        self.publish_tf()



    def get_video(self):
        frame, _ = freenect.sync_get_video()
        return frame
        '''
        # # Generate dummy RGB frames
        # rgb_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # # # for different colors
        # # rgb_frame[:, :, 0] = np.random.randint(0, 255, (self.height, self.width))  # R channel
        # # rgb_frame[:, :, 1] = np.random.randint(0, 255, (self.height, self.width))  # G channel
        # # rgb_frame[:, :, 2] = np.random.randint(0, 255, (self.height, self.width))  # B channel
        
        # for solid color
        rgb_frame[:, :, 0] = 0b10000000 #0b00001111 #np.random.randint(255)  # R channel
        rgb_frame[:, :, 1] = 0b10000000 #0b00110011 #np.random.randint(255)  # G channel
        rgb_frame[:, :, 2] = 0b10000000 #0b01010101 #np.random.randint(255)  # B channel
        return rgb_frame
        '''

    def get_depth(self):
        depth, _ = freenect.sync_get_depth()
        return depth

        # # correct depth by transformation
        # transformed_depth = self.align_depth_to_rgb(
        #     depth_frame  = depth,
        #     tx           = -9,
        #     ty           = 21,
        #     angle        = 0,
        #     scaleX       = 0.925,
        #     scaleY       = 0.925,
        #     centerX      = self.cx,
        #     centerY      = self.cy,
        # )
        # return transformed_depth

        # ~/ros2_2025/src/auto_mobile_robot/archives/
        # np.savetxt("depth.csv", transformed_depth.astype(np.uint16), delimiter=",")
        # transformed_depth8 = (transformed_depth/2048 * 255).astype(np.uint8)
        # cv2.imwrite('grayscale_image_opencv.png', transformed_depth8)



    def fps_calculator(self):
        samples = 10
        current_time = self.get_clock().now().nanoseconds / 1e9  # Convert to seconds
        self.timestamps.append(current_time)
        if len(self.timestamps) > samples:
            self.timestamps.pop(0)

        if len(self.timestamps) == samples:
            time_differences = [
                self.timestamps[i] - self.timestamps[i - 1]
                for i in range(1, len(self.timestamps))
            ]
            average_period = sum(time_differences) / len(time_differences)
            frequency = 1 / average_period if average_period > 0 else 0
            self.get_logger().info(f"frequency: {frequency:.2f} FPS")

    def align_depth_to_rgb(self, depth_frame: np.array, tx=0.0, ty=0.0, angle=0.0, scaleX=1, scaleY=1, centerX=0.0, centerY=0.0):
        """
        Align depth frame to RGB with translation, rotation, scaling, and scaling around a specific point.
        
        Parameters:
            depth_frame (np.array): Input depth frame.
            tx (float): Translation along the x-axis.
            ty (float): Translation along the y-axis.
            angle (float): Rotation angle in radians.
            scale (float): Scaling factor.
            centerX (float): x-coordinate of the center for scaling.
            centerY (float): y-coordinate of the center for scaling.
            
        Returns:
            np.array: Transformed depth frame.
        """
        # Translation to move scaling point to the origin
        translate_to_origin = np.array([
            [1, 0, -centerX],
            [0, 1, -centerY],
            [0, 0, 1]
        ])
        
        # Scaling and rotation
        scaling_rotation = np.array([
            [np.cos(angle) * scaleX, -np.sin(angle) * scaleY, 0],
            [np.sin(angle) * scaleX,  np.cos(angle) * scaleY, 0],
            [0, 0, 1]
        ])
        
        # Translation back from the origin
        translate_back = np.array([
            [1, 0, centerX],
            [0, 1, centerY],
            [0, 0, 1]
        ])
        
        # Combine transformations
        affine_params = translate_back @ scaling_rotation @ translate_to_origin
        
        # Add translation
        affine_params[0, 2] += tx
        affine_params[1, 2] += ty
        
        # self.get_logger().info(f'depth_frame: {depth_frame.shape}')
        
        # Apply transformation
        transformed_depth = cv2.warpAffine(depth_frame.astype(np.float32), affine_params[:2,:], dsize=(self.width, self.height))
        
        # self.get_logger().info(f'transformed_depth: {transformed_depth.shape}')
        return transformed_depth

    def rpy_to_rotation_matrix(self, roll, pitch, yaw, degrees=True):
        """
        Converts roll, pitch, yaw angles into a 3x3 rotation matrix.
        
        Args:
        - roll: Rotation around X-axis
        - pitch: Rotation around Y-axis
        - yaw: Rotation around Z-axis
        - degrees: If True, convert from degrees to radians
        
        Returns:
        - 3x3 NumPy rotation matrix
        """
        if degrees:
            roll = np.radians(roll)
            pitch = np.radians(pitch)
            yaw = np.radians(yaw)

        # Rotation matrices for each axis
        Rx = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

        # Combined rotation matrix (R = Rz * Ry * Rx)
        R = Rz @ Ry @ Rx
        return R





def main(args=None):
    try:
        
        rclpy.init(args=args)
        node = KinectPublisher()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        freenect.sync_stop()
        # freenect.shutdown(node.device)

if __name__ == '__main__':
    main()



