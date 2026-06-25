cmake_minimum_required(VERSION 3.8)
project(six_dof_arm_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(arm_motion_node src/arm_motion_node.cpp)
target_include_directories(arm_motion_node PRIVATE src)
ament_target_dependencies(arm_motion_node
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  std_msgs
  moveit_ros_planning_interface
)

add_executable(differential_mixer src/differential_mixer.cpp)
target_include_directories(differential_mixer PRIVATE src)
ament_target_dependencies(differential_mixer
  rclcpp
  std_msgs
)

install(TARGETS
  arm_motion_node
  differential_mixer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/joy_to_arm.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()