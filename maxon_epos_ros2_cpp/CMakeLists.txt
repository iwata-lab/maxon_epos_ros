cmake_minimum_required(VERSION 3.5)
project(maxon_epos_ros2_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
)

include_directories(
  maxon_epos_msgs
  maxon_epos_driver
  maxon_epos_example
)

# Macro to create nodes easily
macro(add_node node_name node_src)

  add_executable("${node_name}" "${node_src}")
  ament_target_dependencies("${node_name}" ${dependencies})

endmacro() 

# Create Cpp executable, create executable nodes here
add_node(maxon_epos_driver ../maxon_epos_driver/src/node/maxon_bringup.cpp)
#add_executable(maxon_epos_driver src/node/maxon_bringup.cpp)
 
ament_export_dependencies(rclcpp)

#target_link_libraries(${PROJECT_NAME} maxon_epos_driver)


ament_package()


