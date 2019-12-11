cmake_minimum_required(VERSION 3.5)
project(hri_c_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

include_directories(
  include
)

add_library(hri_c_driver SHARED
  src/SerialInterface.c src/VehicleInterface.c
)

install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS hri_c_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(hri_c_driver)
ament_package()
