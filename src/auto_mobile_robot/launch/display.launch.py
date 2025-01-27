#!/usr/bin/python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    pkg_share = FindPackageShare(package='auto_mobile_robot').find('auto_mobile_robot')
    xacro_file = os.path.join(pkg_share, 'models/static_robot_description/model.urdf')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True, 
                'robot_description': robot_description_raw}],
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'static_robot'],
            output='screen'
        ),
    ])