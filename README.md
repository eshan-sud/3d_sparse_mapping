
# 3D Sparse Mapping using ORB-SLAM3

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

- Camera Calibration
- Real-time execution in Monocular mode
- Sparse maps generation
- Evaluation on TUM, EuRoC MAV & KITTI Visual Odometry datasets


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

   ```bash
   ssh <username>@<hostname>.local
   ```

- Example:
   ```
   ssh eshan-sud@raspberrypi.local
   ```

### Increase swap space

- Swap space is increased for max performance from the raspberry pi 5

	```bash
	sudo nano /etc/dphys-swapfile
	```

- Change swapsize to 2048

	```bash
	sudo systemctl restart dphys-swapfile
	```

### Execute these project's folder

	```bash
	cd ./scripts/
	chmod +x setup.sh
	./setup/setup.sh
	```

### Execution commands

- Executions will work only after successfull setup of all libraries and packages

- Check all executables

	```bash
	ros2 pkg executables ros2_orbslam3_wrapper

	```

- Check for updates

	```bash
	sudo apt update
	sudo apt install gnome-terminal
	```

- TUM dataset execution (with ROS2)

-- Monocular mode
	```bash
	cd ~/ros2_test/src/ORB_SLAM3/
	./Examples/Monocular/mono_tum \
	 ./Vocabulary/ORBvoc.txt \
	 ./Examples/Monocular/TUM1.yaml \
	 ~/minor-project/Datasets/TUM/rgbd_dataset_freiburg1_desk

	```
-- RGB-D mode
	```bash
	cd ~/ros2_test/src/ORB_SLAM3/
	./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt \
	  Examples/RGB-D/TUM1.yaml \
	  ~/ros2_test/Datasets/TUM/rgbd_dataset_freiburg1_desk \
	  ~/ros2_test/Datasets/TUM/rgbd_dataset_freiburg1_desk/associate.txt
	```


- TUM dataset execution (without ROS2) 

        ```bash
        
        ```


- EuRoC dataset execution




- KITTI dataset execution



- Real-time (Moncular)

```bash
cd ~/ros2_test/
./start_real_time_slam.sh
```



## References (Repositories being used in this project):

##### 1. OpenCV4:
        https://github.com/opencv/opencv

##### 2. OpenCV4 Contrib:
        https://github.com/opencv/opencv_contrib

##### 3. ROS2 OpenCV:
        https://github.com/ros-perception/vision_opencv

##### 4. Pangolin:
        https://github.com/stevenlovegrove/Pangolin

##### 5. ORB-SLAM3:
        https://github.com/eshan-sud/ORB_SLAM3	 			[Forked]

##### 6. ORBS_SLAM3 ROS2 Wrapper:
        https://github.com/ozandmrz/orb_slam3_ros2_mono_publisher       [Forked]

##### 6. Installation of ROS2:
        https://github.com/ozandmrz/ros2_raspberry_pi_5



## Contact the author:

eshansud22_gmail.com
