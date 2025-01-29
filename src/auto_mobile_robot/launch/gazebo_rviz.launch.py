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

''' 
# run this command to spawn the robot in gazebo (but first comment the launch_arguments in launch_gazebo_server, or comment the last include tag in 'smalltown.world'):
    ros2 run gazebo_ros spawn_entity.py -entity auto_mobile_robot -file /home/pita/ros2_2025/src/auto_mobile_robot/models/auto_mobile_robot_description/model.sdf -x 0 -y 0 -z 0.5
# to delete:
    ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'auto_mobile_robot'}"
'''


def generate_launch_description():
    # Set paths
    pkg_gazebo_ros                  = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_share                       = FindPackageShare(package='auto_mobile_robot').find('auto_mobile_robot')
    model_path                      = os.path.join(pkg_share, 'models/auto_mobile_robot.urdf')
    rviz_config_path                = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    robot_localization_file_path    = os.path.join(pkg_share, 'config/ekf.yaml')
    world_path                      = os.path.join(pkg_share, 'worlds/smalltown.world')
    # world_path                      = os.path.join(pkg_share, 'worlds/house.sdf')
    
    use_sim_time    = LaunchConfiguration('use_sim_time', default='True')
    use_simulator   = LaunchConfiguration('use_simulator', default='True')
    headless        = LaunchConfiguration('headless', default='False')

    
    # Start Gazebo server
    launch_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world_path}.items()
    )

    # Start Gazebo client
    launch_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition    = IfCondition(PythonExpression([use_simulator, ' and not ', headless]))
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
        launch_gazebo_server,
        launch_gazebo_client,
        node_robot_localization,
        node_robot_state_publisher,
        node_rqt_robot_steering_node,
        node_rviz,
    ])

    return ld
