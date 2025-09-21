cmake_minimum_required(VERSION 3.8)
project(my_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rplidar_ros REQUIRED)
find_package(ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

install(DIRECTORY launch config ros2_control urdf
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY include/
  DESTINATION include/
)

set(PIGPIO_INCLUDE_DIRS "$ENV{HOME}/pigpio-master")
set(PIGPIO_LIBRARIES "$ENV{HOME}/pigpio-master/libpigpio.so")

add_library(
  my_robot_control
  SHARED
  src/diffbot_system.cpp
  src/motor_driver.cpp 
)

ament_target_dependencies(
  my_robot_control
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_include_directories(my_robot_control PRIVATE ${PIGPIO_INCLUDE_DIRS})
target_link_libraries(my_robot_control ${PIGPIO_LIBRARIES})

install(TARGETS my_robot_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(hardware_interface ros2_control_hardware.xml)

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

ament_package()
