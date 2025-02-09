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
    model_path                      = os.path.join(pkg_share, 'urdf/car.urdf.xacro')
    rviz_config_path                = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    robot_localization_file_path    = os.path.join(pkg_share, 'config/ekf.yaml')
    # world_path                      = os.path.join(pkg_share, 'worlds/ignition_worlds/actor_crowd.sdf')  # Ignition uses .sdf
    world_path                      = model_path  # Ignition uses .sdf

    use_sim_time    = LaunchConfiguration('use_sim_time', default='true')
    use_simulator   = LaunchConfiguration('use_simulator', default='true')
    headless        = LaunchConfiguration('headless', default='false')

    # Start Gazebo Ignition server
    launch_ignition_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ign_gazebo_ros, 'launch', 'gz_sim.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'ign_args': world_path, '-v': '4'}.items()  # Ignition uses `ign_args` to specify the world
    )

    # Start robot localization
    node_robot_localization = Node(
        package      = 'robot_localization',
        executable   = 'ekf_node',
        name         = 'ekf_filter_node',
        output       = 'screen',
        parameters   = [
            robot_localization_file_path,
            {'use_sim_time': use_sim_time}
        ]
    )

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

    # Bridge for Ignition topics
    node_ros_ign_bridge = Node(
        package      = 'ros_gz_bridge',
        executable   = 'parameter_bridge',
        name         = 'ros_ign_bridge',
        output       = 'screen',
        arguments    = [
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/auto_car/pose@geometry_msgs/msg/Pose[gz.msgs.Pose'
        ]
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
        launch_ignition_server,
        # node_robot_localization,
        node_robot_state_publisher,
        node_ros_ign_bridge,
        node_rqt_robot_steering_node,
        node_rviz,
    ])

    return ld
