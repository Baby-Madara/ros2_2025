#!/usr/bin/env python3


#!/usr/bin/python3

import os
from launch                             import LaunchDescription
from launch.actions                     import IncludeLaunchDescription
from launch.conditions                  import IfCondition
from launch.launch_description_sources  import PythonLaunchDescriptionSource
from launch.substitutions               import Command
from launch_ros.actions                 import Node
from launch_ros.substitutions           import FindPackageShare
from launch.substitutions               import Command, LaunchConfiguration, PythonExpression


def generate_launch_description():
    # Set paths
    pkg_ign_gazebo_ros              = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')  # Ignition package
    pkg_share                       = FindPackageShare(package='auto_car').find('auto_car')
    model_path                      = os.path.join(pkg_share, 'models/auto_mobile_robot.urdf')
    rviz_config_path                = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    robot_localization_file_path    = os.path.join(pkg_share, 'config/ekf.yaml')
    world_path                      = os.path.join(pkg_share, 'worlds/smalltown.world')  # Ignition uses .sdf

    use_sim_time    = LaunchConfiguration('use_sim_time', default='true')
    use_simulator   = LaunchConfiguration('use_simulator', default='true')
    headless        = LaunchConfiguration('headless', default='false')

    # Start robot state publisher
    node_robot_state_publisher = Node(
        package      = 'robot_state_publisher',
        executable   = 'robot_state_publisher',
        parameters   = [{
            'use_sim_time': use_sim_time, 
            'robot_description': Command(['xacro ', model_path])
        }],
        arguments    = [model_path]
    )

    # Launch RViz
    node_rviz = Node(
        package      = 'rviz2',
        executable   = 'rviz2',
        name         = 'rviz2',
        output       = 'screen',
        arguments    = ['-d', rviz_config_path]
    )

    # Launch RQT Robot Steering
    node_rqt_robot_steering_node = Node(
        package      = 'rqt_robot_steering',
        executable   = 'rqt_robot_steering',
        name         = 'rqt_robot_steering',
        output       = 'screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription([
        node_robot_state_publisher,
        node_rqt_robot_steering_node,
        node_rviz,
    ])

    return ld
