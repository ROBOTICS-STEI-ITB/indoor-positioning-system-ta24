cmake_minimum_required(VERSION 3.16)
project(ips_nodes_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ips_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

add_compile_options(
  -Wall -Wextra -Wpedantic
  -O3
  -march=native
)

# Algorithm library (compiled once, linked into compute nodes)
add_library(${PROJECT_NAME}_algorithms
  src/algorithms/sync_engine.cpp
  src/algorithms/chan_solver.cpp
  src/algorithms/position_kf.cpp
  src/algorithms/dzs_filter.cpp
  src/algorithms/predictive_tdoa_gate.cpp
)
target_include_directories(${PROJECT_NAME}_algorithms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_algorithms PUBLIC Eigen3::Eigen)

# clock_sync_node
add_executable(clock_sync_node src/clock_sync_node.cpp)
target_include_directories(clock_sync_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(clock_sync_node
  rclcpp std_msgs geometry_msgs builtin_interfaces ips_msgs)
target_link_libraries(clock_sync_node ${PROJECT_NAME}_algorithms)

# position_solver_node
add_executable(position_solver_node src/position_solver_node.cpp)
target_include_directories(position_solver_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(position_solver_node
  rclcpp std_msgs geometry_msgs builtin_interfaces ips_msgs)
target_link_libraries(position_solver_node ${PROJECT_NAME}_algorithms)

# wolf_position_node (NEW) — WoLF-EKF-CA (Duran-Martin et al. 2024)
# Parallel alternative to position_solver_node. Uses only chan_solver from
# the algorithm library (for one-time initialization).
add_executable(wolf_position_node src/wolf_position_node.cpp)
target_include_directories(wolf_position_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(wolf_position_node
  rclcpp std_msgs geometry_msgs builtin_interfaces ips_msgs)
target_link_libraries(wolf_position_node ${PROJECT_NAME}_algorithms)

# bias_compensator_node (NEW)
# Standalone — doesn't need the algorithm library, just Eigen + yaml-cpp
add_executable(bias_compensator_node src/bias_compensator_node.cpp)
target_include_directories(bias_compensator_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(bias_compensator_node
  rclcpp geometry_msgs builtin_interfaces ips_msgs)
target_link_libraries(bias_compensator_node
  Eigen3::Eigen
  yaml-cpp)


# imu_processor_node (IMU Tingkat 1 - compute, C++)
add_executable(imu_processor_node src/imu_processor_node.cpp)
target_include_directories(imu_processor_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(imu_processor_node rclcpp std_msgs geometry_msgs builtin_interfaces ips_msgs)

target_link_libraries(imu_processor_node Eigen3::Eigen)
# differentiator_node (dipindah dari Python ke C++)
add_executable(differentiator_node src/differentiator_node.cpp)
target_include_directories(differentiator_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(differentiator_node rclcpp std_msgs geometry_msgs builtin_interfaces ips_msgs)

target_link_libraries(differentiator_node Eigen3::Eigen)
install(TARGETS
  imu_processor_node
  differentiator_node
  ${PROJECT_NAME}_algorithms
  clock_sync_node
  position_solver_node
  wolf_position_node
  bias_compensator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

message(STATUS "----- ${PROJECT_NAME} configuration -----")
message(STATUS "  Build type   : ${CMAKE_BUILD_TYPE}")
message(STATUS "  Compiler     : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  Executables  : clock_sync_node, position_solver_node, wolf_position_node, bias_compensator_node")
message(STATUS "----------------------------------------")

ament_export_dependencies(rclcpp ips_msgs Eigen3 yaml-cpp)
ament_package()
