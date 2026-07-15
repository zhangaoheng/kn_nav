cmake_minimum_required(VERSION 3.8)
project(traj_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(path_searching REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_library(traj_utils
  src/planning_visualization.cpp
  src/polynomial_traj.cpp
)

target_include_directories(traj_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(traj_utils
  rclcpp
  bspline_opt
  path_searching
  std_msgs
  visualization_msgs
)

target_link_libraries(traj_utils ${PCL_LIBRARIES})

install(TARGETS traj_utils
  EXPORT export_traj_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_traj_utils HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp bspline_opt path_searching std_msgs visualization_msgs)

ament_package()
