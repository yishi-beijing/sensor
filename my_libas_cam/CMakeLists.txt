cmake_minimum_required(VERSION 3.8)
project(my_libas_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Boost REQUIRED)
find_package(angles REQUIRED)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(Boost REQUIRED COMPONENTS thread)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(image_transport REQUIRED)



include_directories(
    include
  ${OPENCV_INCLUDE_DIRS}
  "/opt/LBAS/include"
)

include_directories(
    ${rclcpp_INCLUDE_DIRS}   
)


add_executable(my_libas_cam nodes/lbas_cam_node.cpp src/lbas_cam.cpp) 
ament_target_dependencies(my_libas_cam 

rclcpp 
std_msgs 
sensor_msgs 
image_transport
std_srvs  
${OpenCV_LIBRARIES} 
) 

include_directories(
    ${rclcpp_INCLUDE_DIRS}   
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


target_link_libraries(my_libas_cam
 /opt/LBAS/lib/64/libMvCameraControl.so
)


install(TARGETS
  my_libas_cam
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()   
