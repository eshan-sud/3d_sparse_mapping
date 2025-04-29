

echo ""
echo "-----------------Starting installation of dependencies...-----------------------"
echo ""
sudo apt-get update
sudo apt update && sudo apt upgrade -y
sudo apt install -y gnome-terminal
sudo apt install -y build-essential cmake git libgtk2.0-dev \
pkg-config libavcodec-dev libavformat-dev libswscale-dev \
libtbbmalloc2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev \
libxvidcore-dev libx264-dev libatlas-base-dev gfortran \
python3-dev libeigen3-dev libboost-all-dev libsuitesparse-dev \
libopencv-dev libglew-dev python3-dev libboost-python-dev \
libsdl2-dev mesa-utils glew-utils libepoxy-dev \
software-properties-common libboost-serialization-dev \
libsuitesparse-dev libboost-all-dev cmake g++ libssl-dev \
libgl1-mesa-glx
sudo apt install -y git colcon python3-rosdep2 vcstool wget \
python3-flake8-docstrings python3-pip python3-pytest-cov \
python3-flake8-blind-except python3-flake8-builtins \
python3-flake8-class-newline python3-flake8-comprehensions \
python3-flake8-deprecated python3-flake8-import-order \
python3-flake8-quotes python3-pytest-repeat python3-pytest-rerunfailures \
python3-vcstools libx11-dev libxrandr-dev libasio-dev \
libtinyxml2-dev python3-colcon-common-extensions
sudo apt install -y qtbase5-dev libqt5widgets5 libqt5core5a libqt5gui5 \
qttools5-dev-tools qttools5-dev libgl1-mesa-dev libogre-1.12-dev \
freeglut3-dev libxaw7-dev
sudo add-apt-repository universe
sudo apt update && sudo apt upgrade -y
# ROS2 additional dependencies
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-keys F42ED6FBAB17C654
echo "deb http://repo.ros2.org/ubuntu/main bookworm main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip python3-setuptools python3-colour python3-rosdep python3-ament-package \
python3-rospkg-modules python3-rosdistro-modules python3-catkin-pkg-modules
# In case of errors
sudo dpkg --remove --force-remove-reinstreq python3-rospkg python3-rosdistro python3-catkin-pkg python3-rosdep-modules
sudo dpkg --remove --force-all python3-catkin-pkg python3-rosdep-modules
sudo dpkg --remove --force-all python3-colcon-ros
sudo apt --fix-broken install
sudo apt install -y python3-pip python3-setuptools python3-colour python3-rosdep python3-ament-package \
python3-rospkg-modules python3-rosdistro-modules python3-catkin-pkg-modules
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo tee /etc/apt/trusted.gpg.d/ros.asc
sudo apt autoremove -y
echo ""
echo "-----------------Dependencies installed-----------------------------------------"
echo ""
echo ""
echo "-----------------Starting installation of OpenCV...-----------------------------"
echo ""
cd ~/ros2_test/src/OpenCV/opencv_contrib && git checkout master
cd ../opencv/ && git checkout master
mkdir -p build && cd build
cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=~/ros2_test/src/OpenCV/opencv_contrib/modules \
      -D ENABLE_CXX11=ON \
      -D ENABLE_PKG_CONFIG=ON ..
make -j 3
sudo make install
# Check OpenCV version
# opencv_version
echo ""
echo "-----------------OpenCV installed------------------------------------------------"
echo ""
echo ""
echo "-----------------Starting installations of CV_Bridge...--------------------------"
echo ""
cd ~/ros2_humble/src
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-opencv python3-rospkg libboost-python1.81.0
cd vision_opencv/cv_bridge
git checkout humble
cd ../../..
colcon build --symlink-install
echo ""
echo "-----------------Image Transport & CV_Bridge installed---------------------------"
echo ""
echo ""
echo "-----------------Starting installation of Pangolin...----------------------------"
echo ""
cd ~/ros2_test/src/Pangolin
./scripts/install_prerequisites.sh recommended
mkdir -p build && cd build
cmake .. -D CMAKE_BUILD_TYPE=Release && make -j $(nproc - 1)
sudo make install
# Verify pangolin & its dependencies installation
# cd ./examples/HelloPangolin/HelloPangolin # Pangolin
# glxinfo | grep "OpenGL version" # OpenGL
# dpkg -l | grep glew # GLEW
# ls /usr/include/eigen3/Eigen # eigen3
echo ""
echo "-----------------Pangolin installed----------------------------------------------"
echo ""
echo ""
echo "-----------------Starting installation of ORB_SLAM3...---------------------------"
echo ""
cd ~/ros2_test/src/ORB_SLAM3/
echo ""
echo "-----------------Configuring and building Thirdparty/DBoW2...--------------------"
echo ""
cd Thirdparty/DBoW2
mkdir -p build && cd build
cmake .. && make -j $(nproc - 1)
cd ../../g2o
echo ""
echo "-----------------Configuring and building Thirdparty/g2o...----------------------"
echo ""
mkdir -p build && cd build
cmake .. && make -j
cd ../../../
echo ""
echo "-----------------Uncompress vocabulary...----------------------------------------"
echo ""
cd Vocabulary && tar -xf ORBvoc.txt.tar.gz
cd ../
echo "Uncompressed..."
echo ""
echo "-----------------Configuring and building ORB-SLAM3...---------------------------"
echo ""
mkdir -p build && cd build
cmake .. && make -j $(nproc - 1)
echo ""
echo "-----------------ORB_SLAM3 installed---------------------------------------------"
echo ""
echo ""
echo "-----------------Starting installations of ROS2 (Humble Hawksbill)...------------"
echo ""
cd ~/ros2_humble
sudo apt purge 'ros-*' -y
sudo apt autoremove -y
sudo apt clean
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src
sudo apt upgrade -y
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers python3-vcstool"
colcon build --symlink-install
echo "source ~/ros2_humble/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
# ROS2 additional dependencies
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-keys F42ED6FBAB17C654
echo "deb http://repo.ros2.org/ubuntu/main bookworm main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip python3-setuptools python3-colour python3-rosdep python3-ament-package\
python3-rospkg-modules python3-rosdistro-modules python3-catkin-pkg-modules
# In case of errors
sudo dpkg --remove --force-remove-reinstreq python3-rospkg python3-rosdistro python3-catkin-pkg python3-rosdep-modules
sudo dpkg --remove --force-all python3-catkin-pkg python3-rosdep-modules
sudo dpkg --remove --force-all python3-colcon-ros
sudo apt --fix-broken install
sudo apt install -y python3-pip python3-setuptools python3-colour python3-rosdep python3-ament-package\
python3-rospkg-modules python3-rosdistro-modules python3-catkin-pkg-modules
echo ""
echo "-----------------ROS2 (Humble Hawksbill) installed-------------------------------"
echo ""

cd ~/ros2_test/
rosdep install --from-paths src --ignore-src -r -y
source ~/ros2_humble/install/local_setup.bash

