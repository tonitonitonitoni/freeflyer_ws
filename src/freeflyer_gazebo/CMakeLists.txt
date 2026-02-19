cmake_minimum_required(VERSION 3.10)
project(freeflyer_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(bang_bang_thruster SHARED
  plugins/bang_bang_thruster.cpp
)

target_link_libraries(bang_bang_thruster
  ${GAZEBO_LIBRARIES}
)

ament_target_dependencies(bang_bang_thruster
  rclcpp
  std_msgs
)

add_library(continuous_reaction_wheel SHARED
  plugins/continuous_reaction_wheel.cpp
)

target_link_libraries(continuous_reaction_wheel
  ${GAZEBO_LIBRARIES}
)

ament_target_dependencies(continuous_reaction_wheel
  rclcpp
  std_msgs
)
# ----------------------------
# Install plugin
# ----------------------------
install(
  TARGETS 
  bang_bang_thruster
  continuous_reaction_wheel
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
