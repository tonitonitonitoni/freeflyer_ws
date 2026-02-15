cmake_minimum_required(VERSION 3.10)
project(freeflyer_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(gazebo_ros2_control REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(bang_bang_reaction_wheel SHARED
  plugins/bang_bang_reaction_wheel.cpp
)

target_link_libraries(bang_bang_reaction_wheel
  ${GAZEBO_LIBRARIES}
)

ament_target_dependencies(bang_bang_reaction_wheel
  rclcpp
  std_msgs
)

add_library(continuous_thruster SHARED
  plugins/continuous_thruster.cpp
)

target_link_libraries(continuous_thruster
  ${GAZEBO_LIBRARIES}
)

ament_target_dependencies(continuous_thruster
  rclcpp
  std_msgs
  nav_msgs
)
add_library(freeflyer_sim_system SHARED
  plugins/freeflyer_sim_system.cpp
)

target_include_directories(freeflyer_sim_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  
)

ament_target_dependencies(freeflyer_sim_system
  rclcpp
  pluginlib
  hardware_interface
  gazebo_ros2_control
)
target_include_directories(freeflyer_sim_system PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(freeflyer_sim_system ${GAZEBO_LIBRARIES})
pluginlib_export_plugin_description_file(gazebo_ros2_control freeflyer_sim_system.xml)

add_library(freeflyer_sim_no_rw SHARED
  plugins/freeflyer_sim_no_rw.cpp
)

target_include_directories(freeflyer_sim_no_rw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(freeflyer_sim_no_rw
  rclcpp
  pluginlib
  hardware_interface
  gazebo_ros2_control
)
target_include_directories(freeflyer_sim_no_rw PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(freeflyer_sim_no_rw ${GAZEBO_LIBRARIES})
pluginlib_export_plugin_description_file(gazebo_ros2_control freeflyer_sim_no_rw.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  freeflyer_sim_system.xml
  freeflyer_sim_no_rw.xml
  DESTINATION share/${PROJECT_NAME}
)
# ----------------------------
# Install plugin
# ----------------------------
install(
  TARGETS 
  bang_bang_reaction_wheel
  continuous_thruster
  freeflyer_sim_system
  freeflyer_sim_no_rw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ----------------------------
# Install launch + world files
# ----------------------------
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
