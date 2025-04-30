
# 3D Sparse Mapping

A real-time Sparse mapping SLAM implementation of ORB-SLAM3 on Raspberry Pi 5 using ROS2 (Humble) with an external USB camera

## Testing environment
Model : Raspberry Pi 5 Model B Rev 1.0
Operating system : Debian GNU/Linux 12 (bookworm)
Mode : Monocular

### Hardware Used
1. Raspberry Pi 5 Model B Rev 1.0
2. Raspberry Pi Fan & Heatsink
3. Raspberry Pi Power Adaptor (minimum 27 Watts)
4. 256 GB Mircro SD Card
5. External USB Camera

### Software Used
1. Raspian OS (12 (bookworm))
2. Python (3.11.2)
3. CMake (3.25.1)
4. OpenCV (4.6.0)
5. ROS2 (Humble Hawksbill)
6. Eigen3 (3.3.7)
7. g2o (20241228_git-9-ge97abe41)
8. Sophus (v1.1.0)
9. DBoW2 (v1.1)
10. Pangolin (4.5.0 or v0.9.2-24-g235519a7)
11. OpenGL (3.1 or )
12. Mesa (23.2.1-1~bpo12+rpt3)
13. ORB-SLAM3

## Features
- Camera calibration
- Real-time execution in Monocular mode
- 3D Sparse maps generation
- Dataset-based SLAM evaluation (TUM, EuRoC MAV & KITTI Visual Odometry datasets)
- Conversion tools
- RViz visualisation support
- Modular bash automation


## How to start?
- Steps on how to setup & exeucte this project

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

### Connect to Raspberry Pu 5 via SSH
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
- To calibrate your camera and generate an ORB-SLAM3-compatible **.yaml** file: ``` /start_camera_calibration.sh ```

### Execution Modes
- Executions will work only after successfull setup of all libraries and packages
- Check for updates
``` sudo apt update ```
- Real-Time SLAM (Monocular)
``` ~/ros2_test/ ./start_realtime_slam.sh ```
- Dataset-based SLAM Modes
	- TUM - Monocular : ``` ./start_dataset_slam.sh tum_mono ```
	- TUM - RGB-D : ``` ./start_dataset_slam.sh tum_rgbd ```
	- EuRoC - Monocular : ``` ./start_dataset_slam.sh euroc_mono ```
	- Custom ROS2 Bag (RGB-D) : ``` ./start_dataset_slam.sh custom_rgbd ```

### Downloading the Datasets
- TUM
```
cd ~/ros2_test/scripts/common
./
```
- EuRoC MAV
```
cd ~/ros2_test/scripts/common

```
- KITTI Visual Odometry
```
cd ~/ros2_test/scripts/common

```

### Native ORB-SLAM3 Executables (No ROS2)
- Before executing, run this on the terminal
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

- TUM - Monocular
```
cd ~/ros2_test/src/ORB_SLAM3/
./Examples/Monocular/mono_tum \
 ./Vocabulary/ORBvoc.txt \
 ./Examples/Monocular/TUM1.yaml \
 ~/ros2_test/datasets/TUM/rgbd_dataset_freiburg1_desk
```

- TUM - RGB-D
```
cd ~/ros2_test/src/ORB_SLAM3/
./Examples/RGB-D/rgbd_tum \
  Vocabulary/ORBvoc.txt \
  ./Examples/RGB-D/TUM1.yaml \
  ~/ros2_test/datasets/TUM/rgbd_dataset_freiburg1_desk \
  ~/ros2_test/datasets/TUM/rgbd_dataset_freiburg1_desk/associate.txt
```

- EuRoC - Monocular
```
cd ~/ros2_test/src/ORB_SLAM3/
./Examples/Monocular/mono_euroc \
 ./Vocabulary/ORBvoc.txt \
 ./Examples/Monocular/EuRoC.yaml \
 ~/ros2_test/datasets/EuRoC/MH01 \
 ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt
```

- EuRoC - Stereo
```
cd ~/ros2_test/src/ORB_SLAM3/
./Examples/Stereo/stereo_euroc \
  ./Vocabulary/ORBvoc.txt \
  ./Examples/Stereo/EuRoC.yaml \
  ~/ros2_test/datasets/EuRoC/MH01 \
  ~/ros2_test/datasets/EuRoC/MH01/mav0/MH01.txt

```

- EuRoC - Mono-Inertial
```
cd ~/ros2_test/src/ORB_SLAM3/
./Examples/Monocular-Inertial/mono_inertial_euroc \
 ./Vocabulary/ORBvoc.txt \
 ./Examples/Monocular-Inertial/EuRoC.yaml \
 ~/ros2_test/datasets/EuRoC/MH01 \
 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt
```

- EuRoC - Stereo-Inertial
```
cd ~/ros2_test/src/ORB_SLAM3/
./Examples/Stereo-Inertial/stereo_inertial_euroc \
 ./Vocabulary/ORBvoc.txt \
 ./Examples/Stereo-Inertial/EuRoC.yaml \
 ~/ros2_test/datasets/EuRoC/MH01 \
 ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt
```

- KITTI - Monocular
```
cd ~/ros2_test/src/ORB_SLAM3/
./Examples/Monocular/mono_kitti \
 ./Vocabulary/ORBvoc.txt \
 ./Examples/Monocular/KITTI00-02.yaml \
 ~/ros2_test/datasets/KITTI/dataset/sequences/00 \
 00
```

- KITTI - Stereo
```
cd ~/ros2_test/src/ORB_SLAM3/
./Examples/Stereo/stereo_kitti \
 ./Vocabulary/ORBvoc.txt \
 ./Examples/Stereo/KITTI00-02.yaml \
 ~/ros2_test/datasets/KITTI/dataset/sequences/00
```

### Rosbag handling

#### Recrord Custom Rosbag
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
- Convert any dataset to a ROS2 bag:
```python3 convert_tum_to_ros2_bag.py <tum_sequence_folder_path> <output_folder>```
- Example:
```
python3 convert_tum_to_ros2_bag.py \
 /home/minor-project/ros2_test/datasets/TUM/rgbd_dataset_freiburg1_desk \
 /home/minor-project/tum_ros2_bag
```

#### Execute Rosbag
- 

### Executions

#### ROS2 Node Execution (Real-time)
- Either execute this
```
cd ~/ros2_test/
chmod +x start_real_time_slam.sh
./start_real_time_slam.sh
```
##### OR
- Before executing, run this on each of the terminals
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
- ORB-SLAM3 Monocular Node (Live)
```ros2 run ros2_orbslam3_wrapper monocular_node --ros-args -p input_mode:=live```
- Camera Publisher with Calibration YAML
```
ros2 run ros2_orbslam3_wrapper camera_publisher_node \
 /home/{username}/scripts/common/my_camera.yaml
```
- RViz Launcher (from SSH, not VNC)
	- For some readon it RViz doesn't start on VNC connection
```
ros2 run ros2_orbslam3_wrapper rviz_launcher_node
```
#### Rosbag Execution
- Before executing, run this on each of the terminals
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
- TUM Monocular Node (Rosbag)
```ros2 run ros2_orbslam3_wrapper monocular_node --ros-args -p input_mode:=recorded```
- TUM RGB-D Node (Rosbag)
```ros2 run ros2_orbslam3_wrapper rgbd_node --ros-args -p input_mode:=recorded```
- Camera Publisher with Calibration YAML
```
ros2 run ros2_orbslam3_wrapper camera_publisher_node \
 /home/{username}/ros2_test/scripts/common/my_camera.yaml
```
- RViz Launcher (from SSH, not VNC)
	- For some readon it RViz doesn't start on VNC connection
 	- Also create a ros topic of map & set it to **ORB_SLAM3/map**  
```
ros2 run ros2_orbslam3_wrapper rviz_launcher_node
```
- Play Rosbag
```
ros2 bag play ~/ros2_test/{papth_to_rosbag}/{rosbag_name}
```

- You should see the sparse map being created on the pangolin viewer & on the RViz viewer as well





## References (Repositories being used in this project):

1. OpenCV4: <a href="https://github.com/opencv/opencv">opencv</a>
2. OpenCV dependency: <a href="https://github.com/opencv/opencv_contrib">opencv-contrib</a>
3. ROS2-OpenCV dependency): <a href="https://github.com/ros-perception/vision_opencv">vision-opencv</a>
4. Pangolin: <a href="https://github.com/stevenlovegrove/Pangolin">Pangolin</a>
5. ORB-SLAM3:https://github.com/eshan-sud/ORB_SLAM3	 			[Forked]

##### 6. Installation of ROS2:
https://github.com/ozandmrz/ros2_raspberry_pi_5

##### 7. ORBS_SLAM3 ROS2 Wrapper:
https://github.com/ozandmrz/orb_slam3_ros2_mono_publisher       [Referenced from]





### Contact the author

[Email me](mailto:eshansud22_gmail.com)

