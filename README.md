# `ROS2_2025` Workspace

## Goal

Make two robots:
 1. `auto_mobile_robot`
        3 lanuch files
         - `basic_mobile_robot.launch.py`    for rvis and joint testing
         - `gazebo_rviz.launch.py`           for launching Gazebo with RVIZ
         - `gazebo_rviz_nav2.launch.py`      for launching Gazebo with RVIZ & NAV2 & SLAM : `ros2 launch auto_mobile_robot gazebo_rviz_nav2.launch.py slam:=True` or no slam argument for loading an already saved map
 1. `robotic_arm`
        


 simulate it, and implement on RaspberryPi robot