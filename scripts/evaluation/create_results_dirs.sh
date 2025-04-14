#!/bin/bash

# Base directory
BASE_DIR="${HOME}/ros2_test/results"

# Dataset groups with their valid modes
declare -A dataset_modes
dataset_modes=(
  ["TUM"]="Monocular RGBD"
  ["EUROC"]="Monocular Stereo Mono-Inertial Stereo-Inertial"
  ["KITTI"]="Monocular Stereo"
)

# TUM sequences
source "$HOME/ros2_test/scripts/common/tum_sequences.sh"
TUM_SEQUENCES=("${TUM_SEQUENCES[@]}")
# EuRoC sequences
source "$HOME/ros2_test/scripts/common/euroc_sequences.sh"
EUROC_SEQUENCES=("${EUROC_SEQUENCES[@]}")
# KITTI sequences
source "$HOME/ros2_test/scripts/common/kitti_sequences.sh"
KITTI_SEQUENCES=("${KITTI_SEQUENCES[@]}")

# Loop through dataset groups
for group in "TUM" "EUROC" "KITTI"; do
  # Get sequences for this group
  varname="${group}_SEQUENCES[@]"
  sequences=("${!varname}")
  modes=${dataset_modes[$group]}

  for sequence in "${sequences[@]}"; do
    for mode in $modes; do
      for viewer in "with_viewer" "without_viewer"; do
        for i in {1..5}; do
          mkdir -p "$BASE_DIR/$group/$sequence/$mode/$viewer/run_$i"
        done
      done
    done
  done
done
