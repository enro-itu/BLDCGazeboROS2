cmake_minimum_required(VERSION 3.8)
project(bldc_gz_sim)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros_gz_sim REQUIRED)

install(
  DIRECTORY launch models worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
