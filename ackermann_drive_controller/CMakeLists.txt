cmake_minimum_required(VERSION 3.8)
project(ackermann_drive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(control_toolbox REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/ackermann_drive_controller.cpp
  src/odometry.cpp
  src/traction_limiter.cpp
  src/steering_limiter.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  ackermann_msgs
  builtin_interfaces
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  std_srvs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
  control_toolbox
)

pluginlib_export_plugin_description_file(controller_interface ackermann_drive_controller.xml)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(
  DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_ackermann_drive_controller
    test/test_ackermann_drive_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_ackermann_drive_controller.yaml)
  target_include_directories(test_ackermann_drive_controller PRIVATE include)
  target_link_libraries(test_ackermann_drive_controller
    ${PROJECT_NAME}
  )

  ament_target_dependencies(test_ackermann_drive_controller
    geometry_msgs
    hardware_interface
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    tf2
    tf2_msgs
  )

  ament_add_gmock(
    test_load_ackermann_drive_controller
    test/test_load_ackermann_drive_controller.cpp
  )
  target_include_directories(test_load_ackermann_drive_controller PRIVATE include)
  ament_target_dependencies(test_load_ackermann_drive_controller
    controller_manager
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  controller_interface
  geometry_msgs
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)

ament_package()
