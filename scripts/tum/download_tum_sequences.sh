#!/bin/bash

# Set the dataset directory
DATASET_DIR="${HOME}/ros2_test/datasets/TUM"

# Create and enter the dataset directory
mkdir -p $DATASET_DIR
cd $DATASET_DIR

echo "Starting TUM RGB-D Dataset Downloads..."

# Define a list of sequences
declare -a sequences=(
    # Testing and Debugging
    "freiburg1/rgbd_dataset_freiburg1_xyz"

    # Handheld SLAM
    "freiburg1/rgbd_dataset_freiburg1_360"
    "freiburg1/rgbd_dataset_freiburg1_floor"
    "freiburg1/rgbd_dataset_freiburg1_desk"
    "freiburg1/rgbd_dataset_freiburg1_desk2"
    "freiburg1/rgbd_dataset_freiburg1_room"
    "freiburg2/rgbd_dataset_freiburg2_360_hemisphere"
    "freiburg2/rgbd_dataset_freiburg2_360_kidnap"
    "freiburg2/rgbd_dataset_freiburg2_desk"
    "freiburg2/rgbd_dataset_freiburg2_large_no_loop"
    "freiburg2/rgbd_dataset_freiburg2_large_with_loop"
    "freiburg3/rgbd_dataset_freiburg3_long_office_household"

    # Structure vs. Texture
    "freiburg3/rgbd_dataset_freiburg3_nostructure_notexture_far"
    "freiburg3/rgbd_dataset_freiburg3_nostructure_notexture_near_withloop"
    "freiburg3/rgbd_dataset_freiburg3_nostructure_texture_far"
    "freiburg3/rgbd_dataset_freiburg3_nostructure_texture_near_withloop"
    "freiburg3/rgbd_dataset_freiburg3_structure_notexture_far"
    "freiburg3/rgbd_dataset_freiburg3_structure_notexture_near"
    "freiburg3/rgbd_dataset_freiburg3_structure_texture_far"
    "freiburg3/rgbd_dataset_freiburg3_structure_texture_near"

    # Dynamic Objects
    "freiburg2/rgbd_dataset_freiburg2_desk_with_person"
    "freiburg3/rgbd_dataset_freiburg3_sitting_static"
    "freiburg3/rgbd_dataset_freiburg3_sitting_xyz"
    "freiburg3/rgbd_dataset_freiburg3_sitting_halfsphere"
    "freiburg3/rgbd_dataset_freiburg3_sitting_rpy"
    "freiburg3/rgbd_dataset_freiburg3_walking_static"
    "freiburg3/rgbd_dataset_freiburg3_walking_xyz"
    "freiburg3/rgbd_dataset_freiburg3_walking_halfsphere"
    "freiburg3/rgbd_dataset_freiburg3_walking_rpy"
)

# Download and extract each sequence
for seq in "${sequences[@]}"; do
    FILE=$(basename $seq)
    URL="http://vision.in.tum.de/rgbd/dataset/${seq}.tgz"

    echo "Downloading: $FILE..."
    wget $URL

    echo "Extracting: $FILE.tgz..."
    tar -xvzf ${FILE}.tgz

    echo "Cleaning up: $FILE.tgz..."
    rm ${FILE}.tgz
done

echo "All TUM sequences downloaded and extracted."
