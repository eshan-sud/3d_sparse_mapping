#!/bin/bash

# Base directory where EuRoC sequences are stored
DATASET_DIR=~/ros2_test/Datasets/EuRoC
SCRIPT_PATH=~/ros2_test/scripts/euroc/generate_times.py

sequences=(
    MH01
    MH02
    MH03
    MH04
    MH05
    V101
    V102
    V103
    V201
    V202
    V203
)

for seq in "${sequences[@]}"; do
    echo "Generating timestamps for $seq..."
    python3 "$SCRIPT_PATH" "$DATASET_DIR/$seq/mav0"
done

echo "All times.txt files generated for EuRoC."
