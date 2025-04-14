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

# Load sequences from respective files
source "$HOME/ros2_test/scripts/common/tum_sequences.sh"
TUM_SEQUENCES=("${TUM_SEQUENCES[@]}")

source "$HOME/ros2_test/scripts/common/euroc_sequences.sh"
EUROC_SEQUENCES=("${EUROC_SEQUENCES[@]}")

source "$HOME/ros2_test/scripts/common/kitti_sequences.sh"
KITTI_SEQUENCES=("${KITTI_SEQUENCES[@]}")

# Viewers
#viewers=("with_viewer" "without_viewer")
viewers=("without_viewers")

# Create result directory structure
for group in "TUM" "EUROC" "KITTI"; do
  varname="${group}_SEQUENCES[@]"
  sequences=("${!varname}")
  modes=${dataset_modes[$group]}
  for sequence in "${sequences[@]}"; do
    for mode in $modes; do
      for viewer in "${viewers[@]}"; do
        for i in {1..5}; do
          mkdir -p "$BASE_DIR/$group/$sequence/$mode/$viewer/run_$i"
        done
      done
    done
  done

  # Create the summary and failure files
  group_lower=$(echo "$group" | tr '[:upper:]' '[:lower:]')
  summary_file="${BASE_DIR}/${group}/${group_lower}_final_results.csv"
  failed_file="${BASE_DIR}/${group}/${group_lower}_failed_runs.log"

  echo "Sequence,Mode,Viewer,Run,RMSE,Mean,Median,StdDev,Precision,Accuracy,R2_Score,Failure_Rate,Avg_FPS,Exec_Time,CPU_Usage,RAM_Usage" > "$summary_file"
  touch "$failed_file"

  echo "[INFO] Created: $summary_file"
  echo "[INFO] Created: $failed_file"
done

echo "[DONE] All result directories and summary files created."
