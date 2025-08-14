cmake_minimum_required(VERSION 3.8)
project(bldc_gz_sim)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-common5 REQUIRED)

add_library(bldc_motor_plugin SHARED src/bldc_motor_plugin.cpp)
ament_target_dependencies(bldc_motor_plugin rclcpp)
target_link_libraries(bldc_motor_plugin
  gz-sim8::gz-sim8
  gz-math7::gz-math7
  gz-common5::gz-common5
)

install(TARGETS bldc_motor_plugin
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY launch models worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
