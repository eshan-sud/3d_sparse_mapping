cmake_minimum_required(VERSION 3.8)
project(monocular_depth_estimator)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(depth_estimator_node src/depth_estimator_node.cpp)
ament_target_dependencies(depth_estimator_node rclcpp sensor_msgs cv_bridge image_transport OpenCV)

install(PROGRAMS
  src/midas_depth_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  depth_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
