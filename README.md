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

===

## For rgbd gazebo test

## Static Robot Project

This project implements a static robot represented as a box with a camera on top, designed for use with ROS 2 Humble. The robot publishes RGB and RGBD camera data, which can be visualized in RViz2.

### Project Structure

```md
static_robot_project
├── src
│   ├── launch
│   │   └── display.launch.py       # Launch file for the robot and camera
│   ├── urdf
│   │   └── robo_rgbd_gaz.urdf      # URDF file defining the robot and camera
│   ├── rviz
│   │   └── display.rviz             # RViz2 configuration for visualization
│   └── scripts
│       └── camera_publisher.py      # Python script for publishing camera data
├── package.xml                      # Package manifest for ROS 2
├── CMakeLists.txt                  # Build instructions for the package
└── README.md                        # Project documentation
```

### Setup Instructions

1. **Install ROS 2 Humble**: Follow the official ROS 2 installation guide for your operating system.

2. **Clone the Repository**: Clone this project to your local machine.

3. **Build the Package**:
   Navigate to the root of the project and run:

   ```bash
   colcon build --symlink-install
   ```

4. **Source the Setup File**:

   After building, source the setup file:

   ```bash
   source install/setup.bash
   ```

### Usage

1. **Launch the Robot and Camera**:
   Use the following command to launch the robot and camera publisher:

   ```bash
   ros2 launch static_robot_project display.launch.py
   ```

2. **Visualize in RViz2**:
   Open RViz2 in a new terminal:

   ```bash
   ros2 run rviz2 rviz2
   ```

   Load the `display.rviz` configuration to visualize the robot and camera data.

### Features

- Static robot model represented as a box.
- RGB and RGBD camera data publishing.
- Visualization in RViz2 for easy monitoring and debugging.

### Acknowledgments

This project is built on the ROS 2 framework and utilizes various ROS packages for camera handling and visualization.
