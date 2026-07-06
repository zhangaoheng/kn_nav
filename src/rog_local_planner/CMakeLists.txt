cmake_minimum_required(VERSION 3.8)
project(rog_local_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-O3 -Wall -Wextra -Werror=return-type)

find_package(ament_cmake REQUIRED)
find_package(art_planner_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rog_map_ros2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(rog_local_planner_node src/rog_local_planner_node.cpp)
target_link_libraries(rog_local_planner_node rog_map_ros2::rog_map_ros2)
ament_target_dependencies(rog_local_planner_node
  art_planner_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

install(TARGETS rog_local_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
