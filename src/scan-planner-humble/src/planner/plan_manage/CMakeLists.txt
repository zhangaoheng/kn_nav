cmake_minimum_required(VERSION 3.8)
project(scan_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(traj_utils REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Bspline.msg
  msg/ControllerCommand.msg
  msg/DataDisp.msg
  msg/PlannerStatus.msg
  msg/PlanningRequest.msg
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    nav_msgs
    std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

set(SCAN_PLANNER_DEPS
  rclcpp
  bspline_opt
  cv_bridge
  geometry_msgs
  nav_msgs
  plan_env
  path_searching
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  traj_utils
  visualization_msgs
)

add_executable(scan_planner_node
  src/scan_planner_node.cpp
  src/scan_replan_fsm.cpp
  src/planner_manager.cpp
)
target_include_directories(scan_planner_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(scan_planner_node ${SCAN_PLANNER_DEPS})
target_link_libraries(scan_planner_node "${cpp_typesupport_target}" ${PCL_LIBRARIES})

add_executable(closed_loop_controller src/closed_loop_controller.cpp)
target_include_directories(closed_loop_controller PRIVATE
  include
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(closed_loop_controller rclcpp geometry_msgs nav_msgs std_msgs tf2 tf2_geometry_msgs bspline_opt)
target_link_libraries(closed_loop_controller "${cpp_typesupport_target}")

add_executable(go2_gait_publisher src/go2_gait_publisher.cpp)
ament_target_dependencies(go2_gait_publisher rclcpp nav_msgs sensor_msgs)

install(TARGETS
  scan_planner_node
  closed_loop_controller
  go2_gait_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
