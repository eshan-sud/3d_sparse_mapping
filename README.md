
# 3D Sparse Mapping

A real-time Sparse mapping SLAM implementation of ORB-SLAM3 on Raspberry Pi 5 using ROS2 (Humble) with an external USB camera.

SLAM (Simultaneous Localisation and Mapping) enables a robot to construct a three-dimensional map of the environment while localising its own pose (position and orientation) within it. ORB-SLAM3 is a one of the most powerful SLAM implementations which have real-time, multi-map, and multi-mode support even on low-powered devices (such as the Raspberry Pi). I have implemented ORB-SLAM3 in ROS2 (Humble) on the Pi5, and built a full modular system that supports real-time monocular SLAM, RGB-D mapping, and dataset-based evaluation (TUM, EuRoC, and KITTI) with automated scripts and camera calibration tools. The system is capable of publishing 3D sparse maps, integrating with RViz2 for visualization, and supports live or recorded input via ROS 2 topics or native ORB-SLAM3 executables.

This is also my minor project for my Bachelor's of Technology degree at Maniapl University Jaipur


![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%205-blue)
![ROS2](https://img.shields.io/badge/ROS2-Humble-green)
![SLAM](https://img.shields.io/badge/SLAM-ORB_SLAM3-blue)
![OpenCV](https://img.shields.io/badge/OpenCV-4.6.0-green)
![License](https://img.shields.io/badge/license-MIT-blue)

- Real-time Monocular SLAM

https://github.com/user-attachments/assets/c1e514ce-84ce-493f-891f-064e3c87e71a

- Example TUM RGB-D Monocular SLAM

https://github.com/user-attachments/assets/fde1103e-1539-4830-b77d-c23ed27f7330



## Features
- Camera calibration
- Real-time execution in Monocular mode
- 3D Sparse maps generation
- Dataset-based SLAM evaluation (TUM, EuRoC MAV & KITTI Visual Odometry datasets)
- Conversion tools
- RViz visualisation support
- Modular bash automation



## Testing environment
Model : Raspberry Pi 5 Model B Rev 1.0
Operating system : Debian GNU/Linux 12 (bookworm)
Mode : Monocular
### Hardware Used
1. Raspberry Pi 5 Model B Rev 1.0
2. Raspberry Pi Fan & Heatsink
3. Raspberry Pi Power Adaptor (minimum 27 Watts)
4. 256 GB Micro SD Card
5. External USB Camera
### Software / Packages Used
1. Raspian OS (12 (bookworm))
2. Python (3.11.2)
3. CMake (3.25.1)
4. OpenCV (4.6.0)
5. OpenCV Contrib
6. Vision_opencv
7. ROS2 (Humble Hawksbill)
8. Eigen3 (3.3.7)
9. g2o
10. Sophus (v1.1.0)
11. DBoW2 (v1.1)
12. Pangolin (4.5.0)
13. OpenGL (3.1)
14. Mesa (23.2.1-1~bpo12+rpt3)
15. ORB-SLAM3
16. Octomap
17. Image Common
18. Message Filters
19. Rclcpp
20. Octomap_ros
21. Geometry2
22. Common_interfaces



## How to start?
- Steps on how to setup & execute this project
### Headless Connection with Raspberry Pi 5
1. Download Raspberry Pi Imager from the [official website](https://www.raspberrypi.com/software/).
2. Connect your microSD card to your laptop/PC.
3. Run Raspberry Pi Imager:
   - Select the appropriate Raspberry Pi device (e.g., Raspberry Pi 5).
   - Select the required operating system (e.g., Raspberry Pi OS (64-bit)).
   - Select the appropriate storage (your microSD card).
4. Press `Ctrl + Shift + X` to open Advanced Options:
   - Set a hostname (default: `raspberrypi`).
   - Enable SSH and set a username and password *(remember these)*.
   - Configure Wi-Fi settings (SSID, password, and country code).
   - Set the correct locale & keyboard layout.
     _To check your layout: press `Windows + Spacebar` or check your system settings._
   - Click Save.
5. Click Write to write the OS to the microSD card
   The Imager will automatically eject the card when done
6. Insert the microSD card into the Raspberry Pi and power it on
### Connect to Raspberry Pi 5 via SSH
1. Open Command Prompt (Windows) or Terminal (Mac/Linux).
2. Run the following command:
```
ssh <username>@<hostname>.local
```
- Example:
```
ssh eshan-sud@raspberrypi.local
```
### Increase swap space
- Swap space is increased for max performance from the raspberry pi 5
```
sudo nano /etc/dphys-swapfile
```
- Change swapsize to 2048
```
sudo systemctl restart dphys-swapfile
```
### Execute the Project's Folder
```
cd ./scripts/
chmod +x setup.sh
./setup/setup.sh
```
### Camera Calibration
- Use this checkerboard for calibration: <a href="https://markhedleyjones.com/projects/calibration-checkerboard-collection">checkerboard-patterns</a>
- To calibrate your camera and generate an ORB-SLAM3-compatible **.yaml** file:
```
./start_camera_calibration.sh
```
### Downloading the Datasets
- TUM RGB-D
```
cd ~/ros2_test/scripts/tum/
./download_tum_sequences.sh
```
- EuRoC MAV
```
cd ~/ros2_test/scripts/euroc/
./download_euroc_sequences.sh
```
- KITTI Visual Odometry
```
cd ~/ros2_test/scripts/kitti/
./download_kitti_sequences.sh
```
### Rosbag handling
#### Record Custom Rosbag
- Monocular
```
ros2 bag record /camera/image_raw
ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw
```
- RGB-D
```
ros2 bag record /camera/color/image_raw /camera/depth/image_raw
```
#### Converting Datasets to RosBag
- Convert any TUM dataset to a ROS2 bag:
```
cd ~/ros2_test/scripts/tum
python3 convert_tum_to_ros2_bag.py <tum_sequence_folder_path> <output_folder>
```
- Convert any euroc dataset to a ROS2 bag:
```
cd ~/ros2_test/scripts/euroc
python3 convert_euroc_to_ros2_bag.py <tum_sequence_folder_path> <output_folder>
```
- Example:
```
cd ~/ros2_test/scripts/tum
python3 convert_tum_to_ros2_bag.py \
 /home/{your_username}/ros2_test/datasets/TUM/rgbd_dataset_freiburg1_desk \
 /home/{your_username}/tum_ros2_bag
```
#### Check Rosbag Contents
```
ros2 bag info ~/ros2_test/{path_to_rosbag}/{rosbag_name}
```
#### Execute Rosbag
```
ros2 bag play ~/ros2_test/{path_to_rosbag}/{rosbag_name}
```
### Execution
- Executions will work only after successful setup of all libraries, packages & datasets (if executing on them)
- Check for updates:
```
sudo apt update && sudo apt upgrade -y
```
- Before executing, execute these on terminal(s)
```
export DISPLAY=:0
export LIBGL_ALWAYS_INDIRECT=1
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libGL.so
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ros2_test/src/ORB_SLAM3/lib
export LD_LIBRARY_PATH=~/ros2_test/src/ORB_SLAM3/Thirdparty/DBoW2/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=~/ros2_test/src/ORB_SLAM3/Thirdparty/g2o/lib:$LD_LIBRARY_PATH
source ~/ros2_test/install/local_setup.bash
```
#### Execute using pre-written script
- Real-time SLAM (Monocular)
```
~/ros2_test/ ./start_realtime_slam.sh
```
- Dataset-based SLAM
	- TUM - Monocular : ``` ./start_dataset_slam.sh tum_mono ```
	- TUM - RGB-D : ``` ./start_dataset_slam.sh tum_rgbd ```
	- EuRoC - Monocular : ``` ./start_dataset_slam.sh euroc_mono ```
	- Custom ROS2 Bag (RGB-D) : ``` ./start_dataset_slam.sh custom_rgbd ```
#### Native ORB-SLAM3 Executables (No ROS2)
- TUM RGB-D - Monocular
```
cd ~/ros2_test/src/ORB_SLAM3/
./Examples/Monocular/mono_tum \
 ./Vocabulary/ORBvoc.txt \
 ./Examples/Monocular/TUM1.yaml \
 ~/ros2_test/datasets/TUM/rgbd_dataset_freiburg1_desk
```
- TUM RGB-D - RGB-D
```
cd ~/ros2_test/src/ORB_SLAM3/
./Examples/RGB-D/rgbd_tum \
  Vocabulary/ORBvoc.txt \
  ./Examples/RGB-D/TUM1.yaml \
  ~/ros2_test/datasets/TUM/rgbd_dataset_freiburg1_desk \
  ~/ros2_test/datasets/TUM/rgbd_dataset_freiburg1_desk/associate.txt
```
- EuRoC MAV - Monocular
```
cd ~/ros2_test/src/ORB_SLAM3/
./Examples/Monocular/mono_euroc \
 ./Vocabulary/ORBvoc.txt \
 ./Examples/Monocular/EuRoC.yaml \
 ~/ros2_test/datasets/EuRoC/MH01 \
 ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt
```
- EuRoC MAV - Stereo
```
cd ~/ros2_test/src/ORB_SLAM3/
./Examples/Stereo/stereo_euroc \
  ./Vocabulary/ORBvoc.txt \
  ./Examples/Stereo/EuRoC.yaml \
  ~/ros2_test/datasets/EuRoC/MH01 \
  ~/ros2_test/datasets/EuRoC/MH01/mav0/MH01.txt
```
- EuRoC MAV - Mono-Inertial
```
cd ~/ros2_test/src/ORB_SLAM3/
./Examples/Monocular-Inertial/mono_inertial_euroc \
 ./Vocabulary/ORBvoc.txt \
 ./Examples/Monocular-Inertial/EuRoC.yaml \
 ~/ros2_test/datasets/EuRoC/MH01 \
 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt
```
- EuRoC MAV - Stereo-Inertial
```
cd ~/ros2_test/src/ORB_SLAM3/
./Examples/Stereo-Inertial/stereo_inertial_euroc \
 ./Vocabulary/ORBvoc.txt \
 ./Examples/Stereo-Inertial/EuRoC.yaml \
 ~/ros2_test/datasets/EuRoC/MH01 \
 ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt
```
- KITTI Visual Odometry - Monocular
```
cd ~/ros2_test/src/ORB_SLAM3/
./Examples/Monocular/mono_kitti \
 ./Vocabulary/ORBvoc.txt \
 ./Examples/Monocular/KITTI00-02.yaml \
 ~/ros2_test/datasets/KITTI/dataset/sequences/00 \
 00
```
- KITTI Visual Odometry - Stereo
```
cd ~/ros2_test/src/ORB_SLAM3/
./Examples/Stereo/stereo_kitti \
 ./Vocabulary/ORBvoc.txt \
 ./Examples/Stereo/KITTI00-02.yaml \
 ~/ros2_test/datasets/KITTI/dataset/sequences/00
```
#### ROS2 Node Execution (real-time)
- Monocular Node
```ros2 run ros2_orbslam3_wrapper monocular_node --ros-args -p input_mode:=live```
- Camera Publisher with Calibration YAML
```
ros2 run ros2_orbslam3_wrapper camera_publisher_node \
 /home/{your_username}/ros2_test/scripts/common/my_camera.yaml
```
- RViz Launcher (from SSH, not VNC)
> [!NOTE]
> RViz may not launch under VNC. For proper OpenGL rendering, use an SSH terminal session instead.
> Also, create a ros topic of map & set it to **ORB_SLAM3/map** 
```
ros2 run ros2_orbslam3_wrapper rviz_launcher_node
```
#### ROS2 Node Execution (rosbag)
- Monocular Node
```ros2 run ros2_orbslam3_wrapper monocular_node --ros-args -p input_mode:=recorded```
- RGB-D Node
```ros2 run ros2_orbslam3_wrapper rgbd_node --ros-args -p input_mode:=recorded```
- Camera Publisher with Calibration YAML
```
ros2 run ros2_orbslam3_wrapper camera_publisher_node \
 /home/{your_username}/ros2_test/scripts/common/my_camera.yaml
```
- RViz Launcher (from SSH, not VNC)
> [!NOTE]
> RViz may not launch under VNC. For proper OpenGL rendering, use an SSH terminal session instead.
> Also, create a ros topic of map & set it to **ORB_SLAM3/map** 
```
ros2 run ros2_orbslam3_wrapper rviz_launcher_node
```
- Play Rosbag
```
ros2 bag play ~/ros2_test/{path_to_rosbag}/{rosbag_name}
```

> [!Important]  
> You should now see the sparse map being created on the pangolin viewer & on the RViz viewer as well


## References (Repositories being used in this project):

1. <a href="https://github.com/opencv/opencv">OpenCV4</a>
2. <a href="https://github.com/opencv/opencv_contrib">opencv-contrib</a> (OpenCV dependency)
3. <a href="https://github.com/ros-perception/vision_opencv">vision-opencv</a> (ROS2-OpenCV dependency)
4. <a href="https://github.com/stevenlovegrove/Pangolin">Pangolin</a>
5. <a href="https://github.com/eshan-sud/ORB_SLAM3">ORB-SLAM3 [Forked]</a>
6. <a href="https://github.com/ozandmrz/ros2_raspberry_pi_5">ROS2</a>
7. <a href="https://github.com/ozandmrz/orb_slam3_ros2_mono_publisher">ROS2 ORB-SLAM3 Wrapper [Referenced from]</a>
8. <a href="https://github.com/ros2/rviz">rviz</a>
9. <a href="https://github.com/ros-perception/image_common">image-common</a>
10. <a href="https://github.com/ros2/message_filters">message-filters</a>
11. <a href="https://github.com/ros2/rclcpp">rclcpp</a>
12. <a href="https://github.com/OctoMap/octomap">octomap</a>
13. <a href="https://github.com/OctoMap/octomap_ros">octomap_ros</a>
14. <a href="https://github.com/ros2/geometry2">geometry2</a>
15. <a href="https://github.com/ros2/common_interfaces">common_Interfaces</a>


### Contact the author

[eshansud22@gmail.com](mailto:eshansud22_gmail.com)

