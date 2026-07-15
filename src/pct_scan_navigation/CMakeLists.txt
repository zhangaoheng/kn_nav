cmake_minimum_required(VERSION 3.8)
project(pct_scan_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(scan_planner REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/NavigationStatus.msg
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(pct_scan_coordinator src/pct_scan_coordinator.cpp)
ament_target_dependencies(pct_scan_coordinator
  geometry_msgs nav_msgs rclcpp scan_planner std_msgs std_srvs)
target_link_libraries(pct_scan_coordinator "${cpp_typesupport_target}")

install(TARGETS pct_scan_coordinator DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/global_path_follow_coordinator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pct_global_path_follow_coordinator
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_navigation_contract test/test_navigation_contract.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
