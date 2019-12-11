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
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

add_library(hri_joystick SHARED
  src/VscProcess.cpp
  src/JoystickHandler.cpp
)
ament_target_dependencies(hri_joystick
  hri_c_driver
  hri_safety_sense_srvs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
)

add_executable(hri_joystick_node src/main.cpp)
ament_target_dependencies(hri_joystick_node rclcpp)
target_link_libraries(hri_joystick_node hri_joystick)

install(TARGETS hri_joystick
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS hri_joystick_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch cfg
  DESTINATION share/${PROJECT_NAME}
)

rclcpp_components_register_nodes(hri_joystick
  "hri_safety_sense::VscProcess")

ament_package()
