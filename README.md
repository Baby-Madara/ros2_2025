# `ROS2_2025` Workspace

## Roadmap for What to Learn in ROS2 and the Related Activities

| ## | Topic                                                                                                 | Done | Description |
|----|-------------------------------------------------------------------------------------------------------|:----:|-------------|
| 00 | What are `ROS2`, `Node`, `Topic`, `Message`, `Publisher`, `Subscriber`, `URDF`, `SDF`, `Launch_File`  |  ✅  |             |
| 01 | visaulize robot in `RVIZ`                                                                             |  ✅  |             |
| 02 | Simulating robot in `Gazebo`                                                                          |  ✅  |             |
| 03 | Simulating robot in `Gazebo_Ignition`                                                                 |      |             |
| 04 | Adding RGBD camera in `RVIZ`                                                                          |  ✅  |             |
| 04 | Adding RGBD camera in `Gazebo`                                                                        |      |             |
| 04 | Adding RGBD camera in real `Kinect`                                                                   |      |             |
| 04 | Learning ROS `Actions`                                                                                |      |             |
| 04 | Learning ROS `Control`                                                                                |      |             |
| 04 | Learning ROS `Service & Servers`                                                                      |      |             |
| 05 | Implement a mobile robot (may use `NAV2`) in real world                                               |      | `nav2` is a tool for path planning in 2D mainly. It contains different algorithms for path planning and localization, in addition to different robot configurations like differential, omniwheel robot, legged robot, etc. |
| 06 | Implement a static robot (may use `MOVEIT2`) in real world                                            |      | `moveit2` is a tool mainly for making inverse kinematics solutions (numerical solutions) |
| 07 | Using `Docker`, and implementing on `RasbperryPi` once, then on `Nvidia JetsonNano`                   |      | `Docker` can make containers that run (for example) a code that needs Ubuntu 22 on a Ubuntu 18 device, or on Windows          |
| 08 | Making it IoT project                                                                                 |      |             |
| 09 | Using Computer vision by `YOLO` or `OpenCV`                                                           |      |             |
| 10 | Reinforcement Learning `DQN`, `PPO`, `DDPG`                                                           |      |             |
| 11 | NLP                                                                                                   |      |             |

## Achievements and Details

Make two robots:

 1. `auto_mobile_robot`
    3 lanuch files
       - `basic_mobile_robot.launch.py`    for rvis and joint testing
       - `gazebo_rviz.launch.py`           for launching Gazebo with RVIZ
       - `gazebo_rviz_nav2.launch.py`      for launching Gazebo with RVIZ & NAV2 & SLAM : `ros2 launch auto_mobile_robot gazebo_rviz_nav2.launch.py slam:=True` or no slam argument for loading an already saved map
       - `ignition_rviz.launch.py`         for launching Gazebo Ignition with RVIZ

## Steps for First Installation

```bash

# Install ROS Humble (or foxy, but humble has Long Term Support LTS until 2027)
# Install VS_Code (Most useful IDE on that bleak OS, can be installed from snap store)

sudo apt update
sudo apt install terminator

# for installing the various ros pkgs (including gazebo plugins, Moveit2, Nav2, ...etc):
sudo apt install ros-${ROS_DISTRO}-[WHATEVER THE PACKAGE]

cd ~
mkdir -p ros2_2025/src/
# then add the `auto_mobile_robot` inside src folder (manually or download it)

cd ~/ros2_2025
# echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc;
echo "source ~/ros2_2025/install/setup.bash" >> ~/.bashrc;

# FROM Now On, these two commands (build_symlink and sourcing_bashrc) shall be used together (use --symlink-install makes rebuilding unnecessary if a script file was modified)
colcon build --symlink-install; source ~/.bashrc;  

# Modify the paths in the files: launch, urdf, sdf, CmakeLists (I think), etc
ros2 launch auto_mobile_robot gazebo_rviz.launch.py

```
