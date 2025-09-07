cmake_minimum_required(VERSION 3.8)
project(bldc_gz_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-plugin2 REQUIRED)

add_library(bldc_motor_plugin SHARED src/bldc_motor_plugin.cpp)

ament_target_dependencies(bldc_motor_plugin
  rclcpp
  geometry_msgs
  std_msgs
)

target_link_libraries(bldc_motor_plugin
  gz-sim8::gz-sim8
  gz-math7::gz-math7
  gz-common5::gz-common5
  gz-transport13::gz-transport13
  gz-plugin2::gz-plugin2
)

install(TARGETS bldc_motor_plugin
  LIBRARY DESTINATION lib
)

foreach(dir launch models worlds config)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

install(PROGRAMS
  scripts/gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
