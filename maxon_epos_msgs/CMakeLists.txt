cmake_minimum_required(VERSION 3.5)
project(maxon_epos_msgs)


find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorState.msg"
  "msg/MotorStates.msg"
  DEPENDENCIES builtin_interfaces std_msgs
 )

