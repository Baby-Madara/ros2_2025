#!/usr/bin/python3

import os
from launch                    import LaunchDescription
from launch_ros.actions        import Node
from launch_ros.substitutions  import FindPackageShare


# ros2 run auto_mobile_robot kinect_ros_test.py
def generate_launch_description():

    # Set the path to different files and folders.
    pkg_share                       = FindPackageShare(package='auto_mobile_robot').find('auto_mobile_robot')
    default_rviz_config_path        = os.path.join(pkg_share, 'rviz/point_cloud_config.rviz')

    # Create the launch description and populate
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', default_rviz_config_path]
        )
    ])

