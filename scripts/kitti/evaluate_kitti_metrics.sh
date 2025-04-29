#!/bin/bash

# Configuration
VOCAB_PATH="$HOME/ros2_test/src/ORB_SLAM3/Vocabulary/ORBvoc.txt"
KITTI_YAML_MONO="$HOME/ros2_test/src/ORB_SLAM3/Examples/Monocular/KITTI00-02.yaml"
KITTI_YAML_STEREO="$HOME/ros2_test/src/ORB_SLAM3/Examples/Stereo/KITTI00-02.yaml"
KITTI_DATASET_DIR="$HOME/ros2_test/datasets/KITTI/dataset/sequences"
KITTI_GT_DIR="$HOME/ros2_test/Datasets/KITTI/dataset/poses"
RESULTS_DIR_BASE="$HOME/ros2_test/results/KITTI"
METRICS_SCRIPT="$HOME/ros2_test/scripts/kitti/generate_kitti_metrics.py"
SUMMARY_FILE="$RESULTS_DIR_BASE/kitti_final_results.csv"
NUM_RUNS=5

source "$HOME/ros2_test/scripts/common/kitti_sequences.sh"
sequences=("${KITTI_SEQUENCES[@]}")

# Modes and viewers
modes=("Monocular" "Stereo")
#viewers=("with_viewer", "without_viewer")
viewers=("without_viewer")

run_and_evaluate() {
  echo "[RUNNING] $sequence | $mode | $viewer_flag | run_$run_id"

  mkdir -p "$result_dir"
  cd "$exec_dir" || { echo "[ERROR] Could not cd to $exec_dir"; return; }

  viewer_arg="true"
  [ "$viewer_flag" == "without_viewer" ] && viewer_arg="false"

  /usr/bin/time -f "ExecutionTime=%e\nCPUUsage=%P\nMemoryKB=%M" -o "$log_output" \
    bash -c "$bin_path $VOCAB_PATH $config_path $dataset_path $viewer_arg"

  # Determine which trajectory file is available
  if [ -f KeyFrameTrajectory.txt ]; then
    traj_file="KeyFrameTrajectory.txt"
  elif [ -f CameraTrajectory.txt ]; then
    traj_file="CameraTrajectory.txt"
  else
    echo "[FAILED] No trajectory file produced"
    echo "[FAILED] $sequence | $mode | $viewer_flag | run_$run_id" >> "$RESULTS_DIR_BASE/kitti_failed_runs.log"
    return
  fi
  mv "$traj_file" "$kf_output"
  echo "[INFO] Saved trajectory ($traj_file) to $kf_output"
  rm -f KeyFrameTrajectory.txt kf_true.txt f_false.txt kf_false.txt f_true.txt CameraTrajectory.txt

  if [ -f "$gt_path" ]; then
    python3 "$METRICS_SCRIPT" --gt "$gt_path" --est "$kf_output" --out "$metrics_output"

    if [ -f "$metrics_output" ]; then
      rmse=$(jq '.rmse' "$metrics_output")
      mean=$(jq '.mean' "$metrics_output")
      median=$(jq '.median' "$metrics_output")
      stddev=$(jq '.stddev' "$metrics_output")
      precision=$(jq '.precision' "$metrics_output")
      accuracy=$(jq '.accuracy' "$metrics_output")
      r2_score=$(jq '.r2_score' "$metrics_output")
      failure_rate=$(jq '.failure_rate' "$metrics_output")
      avg_fps=$(jq '.avg_fps' "$metrics_output")
    else
      echo "[WARNING] No metrics.json produced!"
      rmse=mean=median=stddev=precision=accuracy=r2_score=failure_rate=avg_fps="N/A"
    fi
  else
    echo "[WARNING] Ground truth not found at $gt_path"
    rmse=mean=median=stddev=precision=accuracy=r2_score=failure_rate=avg_fps="N/A"
  fi

  exec_time=$(grep "ExecutionTime" "$log_output" | cut -d'=' -f2)
  cpu_usage=$(grep "CPUUsage" "$log_output" | cut -d'=' -f2)
  ram_usage=$(grep "MemoryKB" "$log_output" | cut -d'=' -f2)

  if [ ! -f "$SUMMARY_FILE" ]; then
    echo "Sequence,Mode,Viewer,Run,RMSE,Mean,Median,StdDev,Precision,Accuracy,R2_Score,Failure_Rate,Avg_FPS,Exec_Time,CPU_Usage,RAM_Usage" > "$SUMMARY_FILE"
  fi
  echo "$sequence,$mode,$viewer_flag,run_$run_id,$rmse,$mean,$median,$stddev,$precision,$accuracy,$r2_score,$failure_rate,$avg_fps,$exec_time,$cpu_usage,$ram_usage" >> "$SUMMARY_FILE"
}
# Main loop
for sequence in "${sequences[@]}"; do
  for mode in "${modes[@]}"; do
    for viewer_flag in "${viewers[@]}"; do
      for run_id in $(seq 1 $NUM_RUNS); do

        dataset_path="$KITTI_DATASET_DIR/$sequence"
        gt_path="$KITTI_GT_DIR/$sequence.txt"
        exec_dir="$HOME/ros2_test/src/ORB_SLAM3/Examples/$mode"
        result_dir="$RESULTS_DIR_BASE/$sequence/$mode/$viewer_flag/run_$run_id"
        kf_output="$result_dir/KeyFrameTrajectory.txt"
        log_output="$result_dir/resource_log.txt"
        metrics_output="$result_dir/metrics.json"

        if [ "$mode" == "Monocular" ]; then
          bin_path="$exec_dir/mono_kitti"
          config_path="$KITTI_YAML_MONO"
        elif [ "$mode" == "Stereo" ]; then
          bin_path="$exec_dir/stereo_kitti"
          config_path="$KITTI_YAML_STEREO"
        else
          echo "[ERROR] Unknown mode: $mode"
          continue
        fi
        rm -rf "$result_dir"
        run_and_evaluate
      done
    done
    sleep 160  # 2 mins in between each mode of execution
  done
done

echo "All KITTI runs are completed. Results are saved in $RESULTS_DIR_BASE."
