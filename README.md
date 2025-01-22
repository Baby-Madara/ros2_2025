# `ROS2_2025` Workspace

## Roadmap to What to Learn and the Related Activity

| No. | Topic                                                                               |Done | Description |
|-----|-------------------------------------------------------------------------------------|:---:|-------------|
| 01  | visaulize robot in `RVIZ`                                                           | [x] |             |
| 02  | simulating robot in `Gazebo`                                                        | [x] |             |
| 03  | simulating robot in `Gazebo_Ignition`                                               |     |             |
| 04  | adding RGBD camera in `RVIZ`                                                        | [x] |             |
| 04  | adding RGBD camera in `Gazebo`                                                      |     |             |
| 04  | adding RGBD camera in real `Kinect`                                                 |     |             |
| 04  | Learning ROS Actions                                                                |     |             |
| 04  | Learning ROS Control                                                                |     |             |
| 04  | Learning ROS Service & Servers                                                      |     |             |
| 05  | implement a mobile robot (may use `NAV2`) in real world                             |     | `nav2` is a tool for path planning in 2D mainly. It contains different algorithms for path planning and localization, in addition to different robot configurations like differential, omniwheel robot, legged robot, etc. |
| 06  | implement a static robot (may use `MOVEIT2`) in real world                          |     | `moveit2` is a tool mainly for making inverse kinematics solutions (numerical solutions) |
| 07  | using `Docker`, and implementing on `RasbperryPi` once, then on `Nvidia JetsonNano` |     | `Docker` can make containers that run (for example) a code that needs Ubuntu 22 on a Ubuntu 18 device, or on Windows          |
| 08  | making it IoT project                                                               |     |             |
| 09  | Using Computer vision by `YOLO` or `OpenCV`                                         |     |             |
| 10  | Reinforcement Learning DQN, PPO, DDPG                                               |     |             |
| 11  | NLP                                                                                 |     |             |

## Achievements and Details

Make two robots:

 1. `auto_mobile_robot`
    3 lanuch files
       - `basic_mobile_robot.launch.py`    for rvis and joint testing
       - `gazebo_rviz.launch.py`           for launching Gazebo with RVIZ
       - `gazebo_rviz_nav2.launch.py`      for launching Gazebo with RVIZ & NAV2 & SLAM : `ros2 launch auto_mobile_robot gazebo_rviz_nav2.launch.py slam:=True` or no slam argument for loading an already saved map
       - `ignition_rviz.launch.py`         for launching Gazebo Ignition with RVIZ
