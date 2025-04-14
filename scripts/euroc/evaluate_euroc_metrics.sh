#!/bin/bash

# Configuration
VOCAB_PATH="$HOME/ros2_test/src/ORB_SLAM3/Vocabulary/ORBvoc.txt"
TIMESTAMPS_DIR_BASE="$HOME/ros2_test/src/ORB_SLAM3/Examples"
DATASET_DIR_BASE="$HOME/ros2_test/Datasets/EuRoC"
RESULTS_DIR_BASE="$HOME/ros2_test/results/EUROC"
NUM_RUNS=5

# Sequences to evaluate
source "$HOME/ros2_test/scripts/common/euroc_sequences.sh"
sequences=("${EUROC_SEQUENCES[@]}")

# Modes and viewers
modes=("Monocular" "Stereo" "Mono-Inertial" "Stereo-Inertial")
#viewers=("with_viewer" "without_viewer")
viwers=("without_viewer")

# Mode configurations
declare -A MODE_BINARIES=(
  ["Monocular"]="mono_euroc"
  ["Stereo"]="stereo_euroc"
  ["Mono-Inertial"]="mono_inertial_euroc"
  ["Stereo-Inertial"]="stereo_inertial_euroc"
)

declare -A MODE_PATHS=(
  ["Monocular"]="Monocular"
  ["Stereo"]="Stereo"
  ["Mono-Inertial"]="Monocular-Inertial"
  ["Stereo-Inertial"]="Stereo-Inertial"
)

declare -A MODE_CONFIGS=(
  ["Monocular"]="EuRoC.yaml"
  ["Stereo"]="EuRoC.yaml"
  ["Mono-Inertial"]="EuRoC.yaml"
  ["Stereo-Inertial"]="EuRoC.yaml"
)

run_and_evaluate() {
  local sequence=$1
  local mode=$2
  local viewer_flag=$3
  local run_id=$4

  local bin_name=${MODE_BINARIES[$mode]}
  local mode_path=${MODE_PATHS[$mode]}
  local config_file=${MODE_CONFIGS[$mode]}
  local exec_dir="$HOME/ros2_test/src/ORB_SLAM3/Examples/$mode_path"

  local dataset_path="$DATASET_DIR_BASE/$sequence"
  local timestamp_file="$TIMESTAMPS_DIR_BASE/$mode_path/EuRoC_TimeStamps/${sequence}.txt"

  local result_dir="$RESULTS_DIR_BASE/$sequence/$mode/$viewer_flag/run_$run_id"
  local kf_output="$result_dir/kf_true.txt"
  local log_output="$result_dir/resource_log.txt"

  rm -rf "$result_dir"
  mkdir -p "$result_dir"

  echo "[RUNNING] $sequence | $mode | $viewer_flag | run_$run_id"

  cd "$exec_dir" || { echo "[ERROR] Failed to cd to $exec_dir"; exit 1; }

  local viewer_arg="true"
  if [ "$viewer_flag" == "without_viewer" ]; then
    viewer_arg="false"
  fi

  local cmd="./$bin_name $VOCAB_PATH $exec_dir/$config_file $dataset_path $timestamp_file $viewer_arg"
  echo "[DEBUG] Executing: $cmd"

  /usr/bin/time -f "ExecutionTime=%e\nCPUUsage=%P\nMemoryKB=%M" -o "$log_output" \
    bash -c "$cmd"

  if [ ! -f kf_true.txt ]; then
    echo "[WARNING] ORB-SLAM3 failed. No kf_true.txt. Logging failure."
    echo "[FAILED] $sequence | $mode | $viewer_flag | run_$run_id" >> "$HOME/ros2_test/results/euroc_failed_runs.log"
    return
  fi

  mv kf_true.txt "$kf_output"
  echo "[INFO] Saved kf_true.txt to $kf_output"

  # Evaluation
  local GT_FILE="$dataset_path/mav0/state_groundtruth_estimate0/data.csv"
  local METRICS_OUTPUT="$result_dir/metrics.json"
  local METRICS_SCRIPT="$HOME/ros2_test/scripts/euroc/generate_euroc_metrics.py"

  if [ -f "$GT_FILE" ]; then
    exec_time=$(grep "ExecutionTime" "$log_output" | cut -d'=' -f2)
    cpu_usage=$(grep "CPUUsage" "$log_output" | cut -d'=' -f2)
    ram_usage=$(grep "MemoryKB" "$log_output" | cut -d'=' -f2)

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
  fi

  SUMMARY_FILE="$RESULTS_DIR_BASE/euroc_final_results.csv"
  if [ ! -f "$SUMMARY_FILE" ]; then
    echo "Sequence,Mode,Viewer,Run,RMSE,Mean,Median,StdDev,Precision,Accuracy,R2_Score,FailureRate,AvgFPS,Exec_Time(s),CPU_Usage(%),RAM_Usage(KB)" > "$SUMMARY_FILE"
  fi

  echo "$sequence,$mode,$viewer_flag,run_$run_id,$rmse,$mean,$median,$stddev,$precision,$accuracy,$r2_score,$failure_rate,$avg_fps,$exec_time,$cpu_usage,$ram_usage" >> "$SUMMARY_FILE"
}

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

echo "All EuRoC runs completed. Results saved in $RESULTS_DIR_BASE."

