cmake_minimum_required(VERSION 3.5)
project(rtmp_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)


add_executable(rtmp_node src/rtmp_node.cpp)
ament_target_dependencies(rtmp_node rclcpp sensor_msgs cv_bridge OpenCV)

target_link_libraries(rtmp_node ${OpenCV_LIBRARIES} avformat avcodec avutil swscale)
target_link_libraries(rtmp_node avformat)

install(TARGETS
  rtmp_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
