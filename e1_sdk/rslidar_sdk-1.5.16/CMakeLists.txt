cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(rslidar_sdk)


set(POINT_TYPE XYZI)
set(COMPILE_METHOD COLCON)
set(PROJECT_NAME rslidar_sdk)
add_definitions(-std=c++17)
add_compile_options(-Wall)
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")

add_definitions(-DPOINT_TYPE_XYZI)

add_definitions(-DPOINT_TYPE_XYZIRT)

#ROS2#
find_package(rclcpp QUIET)

add_definitions(-DROS2_FOUND)
include_directories(${rclcpp_INCLUDE_DIRS})
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rslidar_msg REQUIRED)
find_package(std_msgs REQUIRED)                      
find_package(rclcpp_components REQUIRED)  


#Others#
# 查找 包
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
# 添加项目源代码目录
include_directories(${PROJECT_SOURCE_DIR}/src)

# 查找并添加 rs_driver 依赖
add_subdirectory(src/rs_driver)
find_package(rs_driver REQUIRED)
include_directories(
 ${rs_driver_INCLUDE_DIRS}
)

add_library(rslidar_sdk SHARED
  src/manager/node_manager.cpp

)
target_link_libraries(rslidar_sdk yaml-cpp)
# 设置目标依赖项
ament_target_dependencies(rslidar_sdk
  rclcpp
  sensor_msgs
  rslidar_msg
  std_msgs
  yaml-cpp  # 确保依赖 yaml-cpp
  rclcpp_components
  PCL
  pcl_conversions
)

# 注册组件化节点
rclcpp_components_register_node(rslidar_sdk
  PLUGIN "robosense::lidar::NodeManager"
  EXECUTABLE rslidar_sdk_components
)

# 安装目标库
install(
  TARGETS rslidar_sdk
  EXPORT  rslidar_sdk_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
    plugin.xml
    DESTINATION share/rslidar_sdk
)
# 安装启动文件目录
install(DIRECTORY launch/
  DESTINATION share/rslidar_sdk
)

install(DIRECTORY config/
  DESTINATION share/rslidar_sdk
)

install(EXPORT rslidar_sdk_components
    DESTINATION share/rslidar_sdk/cmake
)

# 包裹 ROS 组件
ament_package()
