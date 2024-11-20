cmake_minimum_required(VERSION 3.5)
project(image_pub)

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include ${OpenCV_INCLUDE_DIRS})

# Build the executable
add_executable(image_pub_node src/image_pub.cpp)
ament_target_dependencies(image_pub_node rclcpp sensor_msgs camera_info_manager OpenCV)

# Install the executable
install(TARGETS image_pub_node
  DESTINATION lib/${PROJECT_NAME})

# Install the launch file
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Export package
ament_package()
