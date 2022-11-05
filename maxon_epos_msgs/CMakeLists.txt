cmake_minimum_required(VERSION 3.5)
project(maxon_epos_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
  msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorState.msg"
  "msg/MotorStates.msg"
  DEPENDENCIES builtin_interfaces std_msgs
 )

 ament_export_dependencies(rosidl_default_runtime)

 ament_package()
