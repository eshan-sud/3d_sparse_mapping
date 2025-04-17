#!/bin/bash

# Configuration
VOCAB_PATH="$HOME/ros2_test/src/ORB_SLAM3/Vocabulary/ORBvoc.txt"
CONFIG_PATH="../../Examples/Monocular/TUM1.yaml"
DATASET_DIR_BASE="$HOME/ros2_test/Datasets/TUM"
RESULTS_DIR_BASE="$HOME/ros2_test/results/TUM"
NUM_RUNS=5

# Sequences to evaluate
source "$HOME/ros2_test/scripts/common/tum_sequences.sh"
sequences=("${TUM_SEQUENCES[@]}")

# Modes and viewer options
modes=("Monocular" "RGBD")
#viewers=("with_viewer" "without_viewer")
viewers=("without_viewer")

# Function to run SLAM and evaluate
run_and_evaluate() {
  local sequence=$1
  local mode=$2
  local viewer_flag=$3
  local run_id=$4

  local dataset_path="$DATASET_DIR_BASE/$sequence"
  local result_dir="$RESULTS_DIR_BASE/$sequence/$mode/$viewer_flag/run_$run_id"
  local kf_output="$result_dir/KeyFrameTrajectory.txt"
  local log_output="$result_dir/resource_log.txt"

  rm -rf "$result_dir" # Resets the previous iterations data
  mkdir -p "$result_dir"
  echo "[RUNNING] $sequence | $mode | $viewer_flag | run_$run_id"

  # Determine mode-specific paths
  if [ "$mode" == "Monocular" ]; then
    EXEC_DIR="$HOME/ros2_test/src/ORB_SLAM3/Examples/Monocular"
    BIN="./mono_tum"
    CONFIG="$EXEC_DIR/TUM1.yaml"
    CMD="$BIN $VOCAB_PATH $CONFIG $dataset_path"
  elif [ "$mode" == "RGBD" ]; then
    EXEC_DIR="$HOME/ros2_test/src/ORB_SLAM3/Examples/RGB-D"
    BIN="./rgbd_tum"
    CONFIG="$EXEC_DIR/TUM1.yaml"
    ASSOC="${dataset_path}/associate.txt"
    CMD="$BIN $VOCAB_PATH $CONFIG $dataset_path $ASSOC"
  else
    echo "[ERROR] Unknown mode: $mode"
    return
  fi

  cd "$EXEC_DIR" || { echo "[ERROR] Failed to cd to $EXEC_DIR"; exit 1; }

  # Determine viewer argument
  viewer_arg="true"
  if [ "$viewer_flag" == "without_viewer" ]; then
      viewer_arg="false"
  fi

  full_cmd="$CMD $viewer_arg"
  echo "[DEBUG] Executing: $full_cmd" # -- DEBUGGING --

  # Run command with resource tracking
  /usr/bin/time -f "ExecutionTime=%e\nCPUUsage=%P\nMemoryKB=%M" -o "$log_output" \
    bash -c "$full_cmd"

  if [ ! -f KeyFrameTrajectory.txt ] ; then
    echo "[WARNING] ORB-SLAM3 failed or crashed. No KeyFrameTrajectory.txt produced. Logging failure."
    echo "[FAILED] $sequence | $mode | $viewer_flag | run_$run_id" >> "$HOME/ros2_test/results/TUM/tum_failed_runs.log"
    return
  fi
  mv KeyFrameTrajectory.txt "$kf_output"
  echo "[INFO] Saved KeyFrameTrajectory.txt to $kf_output"
#  rm -f KeyFrameTrajectory.txt kf_true.txt f_false.txt kf_false.txt f_true.txt CameraTrajectory.txt

  GT_FILE="$dataset_path/groundtruth.txt"
  if [ -f "$GT_FILE" ]; then
    # echo "[DEBUG] Groundtruth path: $GT_FILE" # -- Debugging --
    METRICS_SCRIPT="$HOME/ros2_test/scripts/tum/generate_tum_metrics.py"
    METRICS_OUTPUT="$result_dir/metrics.json"
    python3 "$METRICS_SCRIPT" --gt "$GT_FILE" --est "$kf_output" --out "$METRICS_OUTPUT"
    if [ -f "$METRICS_OUTPUT" ]; then
      rmse=$(jq '.rmse' "$METRICS_OUTPUT")
      mean=$(jq '.mean' "$METRICS_OUTPUT")
      median=$(jq '.median' "$METRICS_OUTPUT")
      stddev=$(jq '.stddev' "$METRICS_OUTPUT")
      precision=$(jq '.precision' "$METRICS_OUTPUT")
      accuracy=$(jq '.accuracy' "$METRICS_OUTPUT")
      r2_score=$(jq '.r2_score' "$METRICS_OUTPUT")
      failure_rate=$(jq '.failure_rate' "$METRICS_OUTPUT")
      avg_fps=$(jq '.avg_fps' "$METRICS_OUTPUT")
    else
      echo "[WARNING] Metrics output not found!"
      rmse=mean=median=stddev=precision=accuracy=r2_score=failure_rate=avg_fps="N/A"
    fi
  else
    echo "[WARNING] Ground truth not found at $gt_path"
    rmse=mean=median=stddev=precision=accuracy=r2_score=failure_rate=avg_fps="N/A"
  fi

  # Read time/memory/cpu from log
  exec_time=$(grep "ExecutionTime" "$log_output" | cut -d'=' -f2)
  cpu_usage=$(grep "CPUUsage" "$log_output" | cut -d'=' -f2)
  ram_usage=$(grep "MemoryKB" "$log_output" | cut -d'=' -f2)

  # Append to master CSV
  SUMMARY_FILE="$RESULTS_DIR_BASE/tum_final_results.csv"
  if [ ! -f "$SUMMARY_FILE" ]; then
    echo "Sequence,Mode,Viewer,Run,RMSE,Mean,Median,StdDev,Precision,Accuracy,R2_Score,FailureRate,AvgFPS,Exec_Time(s),CPU_Usage(%),RAM_Usage(KB)" > "$SUMMARY_FILE"
  fi
  echo "$sequence,$mode,$viewer_flag,run_$run_id,$rmse,$mean,$median,$stddev,$precision,$accuracy,$r2_score,$failure_rate,$avg_fps,$exec_time,$cpu_usage,$ram_usage" >> "$SUMMARY_FILE"
}

# Main loop
for sequence in "${sequences[@]}"; do
  for mode in "${modes[@]}"; do
    for viewer_flag in "${viewers[@]}"; do
      for run_id in $(seq 1 $NUM_RUNS); do
        run_and_evaluate "$sequence" "$mode" "$viewer_flag" "$run_id"
      done
    done
    sleep 160  # 2 mins in between each mode of execution
  done
done

echo "All TUM runs are completed. Results are in $RESULTS_DIR_BASE."
