cmake_minimum_required(VERSION 3.5)
project(maxon_epos_msgs)


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED 
  message_generation
  std_msgs
)

find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

ament_package(

 AMENT_DEPENDS message_runtime std_msgs
)


include_directories(
# include
  ${ament_INCLUDE_DIRS}
)