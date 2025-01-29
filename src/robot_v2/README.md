# Robot V2 ROS2 Package

This package, `robot_v2`, is designed for simulating a robot in ROS2 Humble using Gazebo Ignition and RViz2. It includes a classic camera and a depth camera for enhanced perception capabilities.

## Package Structure

The package is organized as follows:

```tree
ros2_2025
└── src
    └── robot_v2
        ├── launch
        │   └── robot_launch.py       # Launch file for Gazebo and RViz2
        ├── urdf
        │   ├── robot.urdf.xacro      # Robot model definition
        │   └── sensors.xacro         # Sensor definitions
        ├── rviz
        │   └── robot_config.rviz      # RViz2 configuration
        ├── CMakeLists.txt            # Build instructions
        └── package.xml                # Package metadata
```

## Installation

To build the package, navigate to the root of your ROS2 workspace and run:

```bash
colcon build --packages-select robot_v2
```

## Running the Simulation

After building the package, you can launch the robot simulation using:

```bash
ros2 launch robot_v2 robot_launch.py
```

This command will start Gazebo Ignition and RViz2, allowing you to visualize the robot along with its sensors.

## Visualization

In RViz2, you will be able to see:

- RGB images from the classic camera
- Depth cloud data from the depth camera

## Dependencies

Ensure you have the following dependencies installed:

- ROS2 Humble
- Gazebo Ignition
- Any additional dependencies specified in `package.xml`

## Additional Information

For more details on the robot's capabilities and configurations, refer to the respective files in the `urdf` and `rviz` directories.