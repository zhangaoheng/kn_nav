cmake_minimum_required(VERSION 3.8)
project(rog_map_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-O3 -Wall -Wextra -Werror=return-type -DQT_NO_VERSION_TAGGING)
add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")
add_definitions(-DORIGIN_AT_CORNER)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

file(GLOB_RECURSE ROG_MAP_SRC CONFIGURE_DEPENDS
  src/rog_map/*.cpp
  include/*.cpp
)

add_library(rog_map_ros2 SHARED ${ROG_MAP_SRC})
target_include_directories(rog_map_ros2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(rog_map_ros2
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  pcl_conversions
)
target_link_libraries(rog_map_ros2 ${PCL_LIBRARIES})

add_executable(rog_map_node src/rog_map_node.cpp)
target_link_libraries(rog_map_node rog_map_ros2)
ament_target_dependencies(rog_map_node rclcpp)

install(TARGETS rog_map_ros2
  EXPORT export_rog_map_ros2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS rog_map_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_targets(export_rog_map_ros2 HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  pcl_conversions
  Eigen3
  PCL
)

ament_package()
