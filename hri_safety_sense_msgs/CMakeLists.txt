cmake_minimum_required(VERSION 3.5)
project(hri_safety_sense_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs)
find_package(std_msgs)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/EmergencyStop.srv"
  "srv/KeyString.srv"
  "srv/KeyValue.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
