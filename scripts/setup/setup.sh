

cd ~/ros2_test # Change this to location of the folder you want it to be in

echo ""
echo "-----------------Starting Installation of SLAM for 3D Mapping...----------------"
echo ""
echo ""
echo "-----------------Starting downgradation of GCC version to 11...-----------------"
echo ""
sudo apt-get update
sudo apt-get install -y gcc-11 g++-11
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 60 --slave /usr/bin/g++ g++ /usr/bin/g++-11
sudo update-alternatives --set gcc /usr/bin/gcc-11 # Automated selection
# sudo update-alternatives --config gcc # Choose auto mode (0) to set /usr/bin/gcc-11
# Verify gcc version
# gcc --version
echo ""
echo "-----------------GCC version downgraded to 11-----------------------------------"
echo ""
echo ""
mkdir -p ~/ros2_humble/src
./install_dependencies.sh # Install dependenies
echo ""
echo "-----------------System installed-----------------------------------------------"
echo ""
