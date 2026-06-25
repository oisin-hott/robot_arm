cmake_minimum_required(VERSION 3.8)
project(six_dof_arm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

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

# 1. Build the hardware interface as a shared library
add_library(six_dof_arm_hardware SHARED
  src/six_dof_arm_hardware.cpp
)

# 2. Tell the compiler where to find the header files
target_include_directories(six_dof_arm_hardware PUBLIC
  include
)

# 3. Link the dependencies into the library
ament_target_dependencies(six_dof_arm_hardware
  hardware_interface
  pluginlib
  rclcpp
)

# 4. Register the plugin XML with pluginlib
pluginlib_export_plugin_description_file(
  hardware_interface
  six_dof_arm_hardware.xml
)

# 5. Install the library and headers so other packages can find it
install(TARGETS six_dof_arm_hardware
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
