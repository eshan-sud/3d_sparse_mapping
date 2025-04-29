#!/bin/bash

# Directory to store KITTI dataset
DATASET_DIR="${HOME}/ros2_test/datasets/KITTI"
mkdir -p "$DATASET_DIR"
cd "$DATASET_DIR" || exit

echo "Downloading KITTI grayscale data..."
wget -c https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_gray.zip

echo "Downloading KITTI calibration data..."
wget -c https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_calib.zip

echo "Downloading KITTI groundtruth files..."
wget -c https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_poses.zip

echo "Extracting grayscale data..."
unzip -q data_odometry_gray.zip

echo "Extracting calibration data..."
unzip -q data_odometry_calib.zip

echo "Extracting groundtruth data..."
unzip -q data_odometry_poses.zip

echo "Cleaning up..."
rm -f data_odometry_gray.zip data_odometry_calib.zip data_odometry_poses.zip

echo "KITTI dataset downloaded and extracted successfully in $DATASET_DIR"
