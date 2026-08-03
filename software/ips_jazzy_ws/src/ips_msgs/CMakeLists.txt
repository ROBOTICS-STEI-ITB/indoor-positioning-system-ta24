cmake_minimum_required(VERSION 3.8)
project(ips_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/UwbAnchorReport.msg"
  "msg/SessionEvent.msg"
  "msg/CorrectedToA.msg"
  "msg/SyncStatus.msg"
  "msg/SyncState.msg"
  "msg/ImuTelemetry.msg"
  "msg/BlinkObservation.msg"
  "msg/AntennaDelays.msg"
)

set(srv_files
  "srv/SetAnchorConfig.srv"
  "srv/RecordControl.srv"
  "srv/Calibrate.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
