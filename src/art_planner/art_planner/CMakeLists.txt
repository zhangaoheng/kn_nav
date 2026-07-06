cmake_minimum_required(VERSION 3.8)
project(art_planner)

add_compile_options(-std=c++14)

find_package(ament_cmake REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(ompl REQUIRED)
find_package(ODE REQUIRED ode)
find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME}
  src/start.cpp
  src/goal.cpp
  src/map/map.cpp
  src/map/processors/basic.cpp
  src/map/processors/change.cpp
  src/map/processors/probability_distribution.cpp
  src/map/processors/sample_density.cpp
  src/params.cpp
  src/planner.cpp
  src/utils.cpp
  src/sampler.cpp
  src/validity_checker/validity_checker.cpp
  src/validity_checker/validity_checker_body.cpp
  src/validity_checker/validity_checker_feet.cpp
  src/validity_checker/height_map_box_checker.cpp
  src/objectives/path_length_objective.cpp
  src/objectives/motion_cost_objective.cpp
  src/planners/grid_astar.cpp
  src/planners/lazy_prm_star_min_update.cpp
  src/planners/prm_motion_cost.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OMPL_INCLUDE_DIRS}
    ${ODE_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  grid_map_core
)
target_link_libraries(${PROJECT_NAME}
  ${OMPL_LIBRARIES}
  ${ODE_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(grid_map_core)
ament_package()
