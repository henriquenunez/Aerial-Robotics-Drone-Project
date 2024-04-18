cmake_minimum_required(VERSION 3.8)
project(example_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(tello_msgs_DIR "${PROJECT_SOURCE_DIR}/../../install/tello_msgs/share/tello_msgs/cmake")

# message(tello_msgs_DIR="${tello_msgs_DIR}")

# find dependencies
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tello_msgs REQUIRED)

add_executable(tracker src/tracker.cpp)
ament_target_dependencies(tracker rclcpp OpenCV cv_bridge geometry_msgs sensor_msgs std_msgs tello_msgs)

include_directories(
  include
)

install(TARGETS
  tracker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
