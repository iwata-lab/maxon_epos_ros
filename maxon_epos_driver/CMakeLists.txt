cmake_minimum_required(VERSION 3.5)
project(maxon_epos_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(DEPENDENCIES
  "geometry_msgs"
  "rclcpp"
  "std_msgs"
)



include_directories(
    include
)

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/lib/EposMotor.cpp
    src/lib/EposManager.cpp
    src/lib/Device.cpp
    src/lib/utils/EposException.cpp
    src/lib/control/ControlModeBase.cpp
    src/lib/control/EposProfilePositionMode.cpp
    src/lib/control/EposProfileVelocityMode.cpp
    src/lib/control/EposCurrentMode.cpp
)

#add_executable(maxon_epos_driver src/node/maxon_bringup.cpp)
#ament_target_dependencies(maxon_bringup
#  "rclcpp_components"
#  "rclcpp"
#  "geometry_msgs"
#  "std_msgs"
#)


target_link_libraries(maxon_epos_driver
)

ament_package(
)

