#!/bin/bash

# Create a temporary environment setup script
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

# Choose your mode
MODE=$1

if [ -z "$MODE" ]; then
  echo "Usage: $0 [tum_mono | tum_rgbd | euroc_mono | custom_mono | custom_rgbd]"
  exit 1
fi

case $MODE in

  tum_mono)
    gnome-terminal -- bash -c "source $SETUP_FILE; ros2 run ros2_orbslam3_wrapper monocular_node --ros-args -p input_mode:=recorded; exec bash"
    ;;

  tum_rgbd)
    gnome-terminal -- bash -c "source $SETUP_FILE; ros2 run ros2_orbslam3_wrapper rgbd_node --ros-args -p input_mode:=recorded; exec bash"
    ;;

  euroc_mono)
    gnome-terminal -- bash -c "source $SETUP_FILE; ros2 run ros2_orbslam3_wrapper monocular_node \
      --ros-args \
      -p input_mode:=recorded \
      -p dataset_type:=EuRoC \
      -p vocabulary_path:=/home/minor-project/ros2_test/src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
      -p settings_path:=/home/minor-project/ros2_test/src/ORB_SLAM3/Examples/Monocular/EuRoC.yaml \
      -p dataset_path:=/home/minor-project/ros2_test/Datasets/EuRoC/MH01/; exec bash"
    ;;

  custom_mono)
    gnome-terminal -- bash -c "source $SETUP_FILE; ros2 run ros2_orbslam3_wrapper orbslam3_node \
      --ros-args \
      -p input_mode:=recorded \
      -p dataset_type:=custom \
      -p vocabulary_path:=/home/minor-project/ros2_test/src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
      -p settings_path:=/your/settings.yaml \
      -p image_topic:=/camera/image_raw; exec bash"
    ;;

  custom_rgbd)
    gnome-terminal -- bash -c "source $SETUP_FILE; ros2 run ros2_orbslam3_wrapper orbslam3_node \
      --ros-args \
      -p input_mode:=recorded \
      -p dataset_type:=custom \
      -p vocabulary_path:=/home/minor-project/ros2_test/src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
      -p settings_path:=/your/settings.yaml \
      -p image_topic:=/camera/color/image_raw \
      -p depth_topic:=/camera/depth/image_raw; exec bash"
    ;;

  *)
    echo "Unknown mode: $MODE"
    echo "Valid modes: tum_mono, tum_rgbd, euroc_mono, custom_mono, custom_rgbd"
    exit 1
    ;;

esac
