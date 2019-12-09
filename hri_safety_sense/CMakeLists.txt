cmake_minimum_required(VERSION 3.5)
project(hri_safety_sense)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(hri_c_driver REQUIRED)
find_package(hri_safety_sense_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(safe_remote_control
  src/main.cpp
  src/VscProcess.cpp
  src/JoystickHandler.cpp)
ament_target_dependencies(safe_remote_control
  hri_c_driver
  hri_safety_sense_srvs
  rclcpp
  sensor_msgs
  std_msgs)

install(TARGETS safe_remote_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
