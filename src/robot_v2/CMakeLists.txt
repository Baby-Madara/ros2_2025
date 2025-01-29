cmake_minimum_required(VERSION 3.5)
project(robot_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(ignition-gazebo_DIR "/opt/ros/humble/share/ignition_cmake2_vendor")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_ign_gazebo REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(ignition-transport REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(visualization_msgs REQUIRED)

# # Include directories
# include_directories(include)

# Install launch files
install(DIRECTORY launch rviz scripts urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# ament_python_install_package(${PROJECT_NAME})

file(GLOB SCRIPT_FILES "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py")

install(
  PROGRAMS ${SCRIPT_FILES}
  DESTINATION lib/${PROJECT_NAME}
)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export package
ament_package()