cmake_minimum_required(VERSION 3.5)
project(web_socket)
set(CMAKE_CXX_STANDARD 11)
# 找到必要的包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(websocketpp REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mycan_msgs REQUIRED)
find_package(tier4_planning_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
include_directories(${mycan_msgs_INCLUDE_DIRS})


# 添加可执行文件
add_executable(web_socket_node src/web_socket.cpp)

# 设置目标依赖项
ament_target_dependencies(web_socket_node rclcpp Boost websocketpp std_msgs  mycan_msgs sensor_msgs tier4_planning_msgs geometry_msgs autoware_adapi_v1_msgs)

# 指定链接库
target_link_libraries(web_socket_node ${Boost_LIBRARIES} OpenSSL::SSL OpenSSL::Crypto )
target_link_libraries(web_socket_node yaml-cpp )
install(TARGETS
  web_socket_node
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
