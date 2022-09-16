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




include_directories(
    include
  ${ament_INCLUDE_DIRS}
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

ament_package(
)

