#!/bin/bash

# Path to your updated generate_associate.py script
ASSOCIATE_SCRIPT=~/ros2_test/scripts/tum/generate_associate.py
DATASET_DIR=~/ros2_test/Datasets/TUM

sequences=(
    freiburg1_xyz
    freiburg1_360
    freiburg1_floor
    freiburg1_desk
    freiburg1_desk2
    freiburg1_room
    freiburg2_360_hemisphere
    freiburg2_360_kidnap
    freiburg2_desk
    freiburg2_large_no_loop
    freiburg2_large_with_loop
    freiburg3_long_office_household
    freiburg3_nostructure_notexture_far
    freiburg3_nostructure_notexture_near_withloop
    freiburg3_nostructure_texture_far
    freiburg3_nostructure_texture_near_withloop
    freiburg3_structure_notexture_far
    freiburg3_structure_notexture_near
    freiburg3_structure_texture_far
    freiburg3_structure_texture_near
    freiburg2_desk_with_person
    freiburg3_sitting_static
    freiburg3_sitting_xyz
    freiburg3_sitting_halfsphere
    freiburg3_sitting_rpy
    freiburg3_walking_static
    freiburg3_walking_xyz
    freiburg3_walking_halfsphere
    freiburg3_walking_rpy
)

for seq in "${sequences[@]}"; do
    echo "Processing $seq..."
    SEQ_DIR="$DATASET_DIR/rgbd_dataset_$seq"
    RGB_FILE="$SEQ_DIR/rgb.txt"
    DEPTH_FILE="$SEQ_DIR/depth.txt"
    ASSOCIATE_FILE="$SEQ_DIR/associate.txt"

    if [[ -f "$RGB_FILE" && -f "$DEPTH_FILE" ]]; then
        python3 "$ASSOCIATE_SCRIPT" "$RGB_FILE" "$DEPTH_FILE" --output "$ASSOCIATE_FILE"
    else
        echo "  Skipping $seq: Missing rgb.txt or depth.txt"
    fi
done

echo "All associate.txt files generated (where possible)."
