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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(rosidl_default_generators REQUIRED)

list(APPEND CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/src/octomap_ros")
find_package(octomap REQUIRED)
find_package(octomap_ros REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/src/octomap/include
  ${PROJECT_SOURCE_DIR}/src/octomap_ros/include
  ${PROJECT_SOURCE_DIR}/src/octomap_msgs/include
)

# Check ORB-SLAM3 shared library exists
if(NOT EXISTS ${PROJECT_SOURCE_DIR}/../ORB_SLAM3/lib/libORB_SLAM3.so)
    message(FATAL_ERROR "libORB_SLAM3.so not found! Did you build ORB-SLAM3?")
endif()

# Import required libraries
# ORB SLAM3
add_library(ORB_SLAM3_LIB SHARED IMPORTED)
set_target_properties(ORB_SLAM3_LIB PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../ORB_SLAM3/lib/libORB_SLAM3.so)
# DBoW2
add_library(DBoW2 SHARED IMPORTED)
set_target_properties(DBoW2 PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so)
# g2o
add_library(g2o SHARED IMPORTED)
set_target_properties(g2o PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so)

# Link ORB-SLAM3 Dependencies
target_link_libraries(ORB_SLAM3_LIB INTERFACE DBoW2 g2o pthread ${OpenCV_LIBS})
target_include_directories(ORB_SLAM3_LIB INTERFACE
    ${PROJECT_SOURCE_DIR}/../ORB_SLAM3
    ${PROJECT_SOURCE_DIR}/../ORB_SLAM3/include
    ${PROJECT_SOURCE_DIR}/../ORB_SLAM3/include/CameraModels
    ${PROJECT_SOURCE_DIR}/../ORB_SLAM3/Thirdparty
    ${PROJECT_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/DBoW2
    ${PROJECT_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/DBoW2/DBoW2
    ${PROJECT_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/Sophus
    ${PROJECT_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/g2o
)

# ORB-SLAM3 ROS2 Node
add_executable(orbslam3_node src/orbslam3_node.cpp)
target_link_libraries(orbslam3_node ORB_SLAM3_LIB ${OpenCV_LIBS} pthread)
ament_target_dependencies(orbslam3_node rclcpp sensor_msgs std_msgs nav_msgs geometry_msgs cv_bridge image_transport OpenCV Eigen3 Pangolin)

# Camera Publisher Node
add_executable(camera_publisher_node src/camera_publisher_node.cpp)
target_link_libraries(camera_publisher_node ${OpenCV_LIBS})
ament_target_dependencies(camera_publisher_node rclcpp sensor_msgs cv_bridge image_transport OpenCV)

# RViz Launcher Node
add_executable(rviz_launcher_node src/rviz_launcher_node.cpp)
target_link_libraries(rviz_launcher_node ${OpenCV_LIBS})
ament_target_dependencies(rviz_launcher_node rclcpp sensor_msgs std_msgs)

# OctoMap Mapping Node
add_executable(octomap_node src/octomap_node.cpp)
target_include_directories(octomap_node PRIVATE ${OCTOMAP_ROS_INCLUDE_DIRS})
target_link_libraries(octomap_node ${OpenCV_LIBS} ${OCTOMAP_LIBRARIES} ${OCTOMAP_ROS_LIBRARIES})
ament_target_dependencies(octomap_node rclcpp nav_msgs octomap_ros)

# Trajectory Publisher Node (for visualizing KeyFrameTrajectory.txt)
#add_executable(trajectory_publisher_node src/trajectory_publisher_node.cpp)
#target_link_libraries(trajectory_publisher_node ${OpenCV_LIBS})
#ament_target_dependencies(trajectory_publisher_node rclcpp nav_msgs geometry_msgs)


# Install the executables
install(TARGETS
  orbslam3_node
  camera_publisher_node
  rviz_launcher_node
#  trajectory_publisher_node
  octomap_node
  DESTINATION lib/${PROJECT_NAME}
)



# Required for ROS2 package
ament_package()
