#!/bin/bash

# sudo apt install gnome-terminal

# Set environment variables

SETUP_FILE="/tmp/ros2_env_setup.sh"

cat <<EOF > $SETUP_FILE
#!/bin/bash
export DISPLAY=:0
export LIBGL_ALWAYS_INDIRECT=1
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libGL.so
export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:~/ros2_test/src/ORB_SLAM3/lib
export LD_LIBRARY_PATH=~/ros2_test/src/ORB_SLAM3/Thirdparty/DBoW2/lib:\$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=~/ros2_test/src/ORB_SLAM3/Thirdparty/g2o/lib:\$LD_LIBRARY_PATH
source ~/ros2_test/install/local_setup.bash
EOF

chmod +x $SETUP_FILE


# Launch Camera Publisher Node
gnome-terminal -- bash -c "source $SETUP_FILE; ros2 run ros2_orbslam3_wrapper camera_publisher_node /home/minor-project/minor-project/camera-calibration/calibration/my_camera.yaml; exec bash"

# Launch ORB-SLAM3 Monocular Node
gnome-terminal -- bash -c "source $SETUP_FILE; ros2 run ros2_orbslam3_wrapper monocular_node --ros-args -p input_mode:=live; exec bash"

# Launch Third Node (e.g., Viewer or another ROS node)
#gnome-terminal -- bash -c "source $SETUP_FILE; echo 'Third node started'; exec bash"
