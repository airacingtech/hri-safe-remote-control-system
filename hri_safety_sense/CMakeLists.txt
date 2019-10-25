cmake_minimum_required(VERSION 3.5)
project(hri_safety_sense)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hri_safety_sense_msgs)
find_package(rclcpp)
#find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs)
find_package(std_msgs)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

#rosidl_generate_interfaces(${PROJECT_NAME}
#  "srv/EmergencyStop.srv"
#  "srv/KeyString.srv"
#  "srv/KeyValue.srv"
#)

include_directories(
  include
  #${Boost_INCLUDE_DIRS}
)

add_executable(safe_remote_control
  src/main.cpp
  src/VscProcess.cpp
  src/VehicleInterface.c
  src/JoystickHandler.cpp
  src/SerialInterface.c)
ament_target_dependencies(safe_remote_control
  hri_safety_sense_msgs
  rclcpp
  sensor_msgs
  std_msgs)
#target_link_libraries(safe_remote_control ${Boost_LIBRARIES})

install(TARGETS safe_remote_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
#ament_export_dependencies(rosidl_default_runtime)
ament_package()
