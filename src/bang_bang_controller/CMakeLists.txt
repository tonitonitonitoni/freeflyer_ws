cmake_minimum_required(VERSION 3.8)
project(bang_bang_controller)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(bang_bang_circle_controller SHARED
  src/bang_bang_circle_controller.cpp
)
target_include_directories(bang_bang_circle_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(bang_bang_circle_controller
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  nav_msgs
  std_msgs
)
pluginlib_export_plugin_description_file(controller_interface bang_bang_controller.xml)


install(TARGETS bang_bang_circle_controller 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
