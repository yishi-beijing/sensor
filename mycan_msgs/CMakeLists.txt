cmake_minimum_required(VERSION 3.8)
project(mycan_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Can530.msg"
  "msg/Can531.msg"
  "msg/Can532.msg"
  "msg/Can534.msg"
  "msg/Can535.msg"
  "msg/Can539.msg"
  "msg/Can53A.msg"
  "msg/Can53B.msg"
  "msg/Can540.msg"
  "msg/Can541.msg"
  "msg/Can542.msg"
  "msg/Can543.msg"
  "msg/Can548.msg"
  "msg/Hwcan.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
