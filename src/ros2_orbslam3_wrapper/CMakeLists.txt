cmake_minimum_required(VERSION 3.25)
project(ros2_orbslam3_wrapper)

# Enforce use of C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

# Check ORB-SLAM3 shared library exists
if(NOT EXISTS ${CMAKE_SOURCE_DIR}/ORB_SLAM3/lib/libORB_SLAM3.so)
    message(FATAL_ERROR "libORB_SLAM3.so not found! Did you build ORB-SLAM3?")
endif()

# Import required libraries
# ORB SLAM3
add_library(ORB_SLAM3_LIB SHARED IMPORTED)
set_target_properties(ORB_SLAM3_LIB PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/ORB_SLAM3/lib/libORB_SLAM3.so)
# DBoW2
add_library(DBoW2 SHARED IMPORTED)
set_target_properties(DBoW2 PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so)
# g2o
add_library(g2o SHARED IMPORTED)
set_target_properties(g2o PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so)

# Link ORB-SLAM3 Dependencies
target_link_libraries(ORB_SLAM3_LIB INTERFACE DBoW2 g2o pthread ${OpenCV_LIBS})
target_include_directories(ORB_SLAM3_LIB INTERFACE
    ${CMAKE_SOURCE_DIR}/ORB_SLAM3/include
    ${CMAKE_SOURCE_DIR}/ORB_SLAM3/include/CameraModels
    ${CMAKE_SOURCE_DIR}/ORB_SLAM3/Thirdparty
    ${CMAKE_SOURCE_DIR}/ORB_SLAM3/Thirdparty/DBoW2
    ${CMAKE_SOURCE_DIR}/ORB_SLAM3/Thirdparty/DBoW2/DBoW2
    ${CMAKE_SOURCE_DIR}/ORB_SLAM3/Thirdparty/Sophus
    ${CMAKE_SOURCE_DIR}/ORB_SLAM3/Thirdparty/g2o
)

# ORB-SLAM3 ROS2 Node
add_executable(orbslam3_ros2_node src/orbslam3_ros2_node.cpp)
target_link_libraries(orbslam3_ros2_node ORB_SLAM3_LIB ${OpenCV_LIBS} pthread)
ament_target_dependencies(orbslam3_ros2_node rclcpp sensor_msgs std_msgs cv_bridge image_transport OpenCV Eigen3 Pangolin)

# Camera Publisher Node
add_executable(camera_publisher_node src/camera_publisher_node.cpp)
target_link_libraries(camera_publisher_node ${OpenCV_LIBS})
ament_target_dependencies(camera_publisher_node rclcpp sensor_msgs cv_bridge image_transport OpenCV)

# Install the executables
install(TARGETS
  camera_publisher_node
  orbslam3_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

# Required for ROS 2 package
ament_package()
