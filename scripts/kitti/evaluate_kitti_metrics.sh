#!/bin/bash

# Configuration
VOCAB_PATH="$HOME/ros2_test/src/ORB_SLAM3/Vocabulary/ORBvoc.txt"
KITTI_YAML_MONO="$HOME/ros2_test/src/ORB_SLAM3/Examples/Monocular/KITTI00-02.yaml"
KITTI_YAML_STEREO="$HOME/ros2_test/src/ORB_SLAM3/Examples/Stereo/KITTI00-02.yaml"
KITTI_DATASET_DIR="$HOME/ros2_test/Datasets/KITTI/dataset/sequences"
KITTI_GT_DIR="$HOME/ros2_test/Datasets/KITTI/dataset/poses"
RESULTS_DIR_BASE="$HOME/ros2_test/results/KITTI"
NUM_RUNS=1  # 5

# Sequences to evaluate
source "$HOME/ros2_test/scripts/common/kitti_sequences.sh"
sequences=("08")

# Modes and viewer options
modes=("Monocular" "Stereo")
#viewers=("with_viewer" "without_viewer")
viewers=("with_viewer")

# Main loop
for sequence in "${sequences[@]}"; do
  for mode in "${modes[@]}"; do
    for viewer_flag in "${viewers[@]}"; do
      for run_id in $(seq 1 $NUM_RUNS); do

        dataset_path="$KITTI_DATASET_DIR/$sequence"
        gt_path="$KITTI_GT_DIR/$sequence.txt"
        exec_dir="$HOME/ros2_test/src/ORB_SLAM3/Examples/$mode"
        result_dir="$RESULTS_DIR_BASE/$sequence/$mode/$viewer_flag/run_$run_id"

        # Binary and config
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

        kf_output="$result_dir/KeyFrameTrajectory.txt"
        log_output="$result_dir/resource_log.txt"
        metrics_output="$result_dir/metrics.json"

        rm -rf "$result_dir"
        mkdir -p "$result_dir"

        echo "[RUNNING] $sequence | $mode | $viewer_flag | run_$run_id"
        cd "$exec_dir" || {
          echo "[ERROR] Could not cd to $exec_dir"
          continue
        }

        viewer_arg="true"
        [ "$viewer_flag" == "without_viewer" ] && viewer_arg="false"

        echo "[DEBUG] Executing: $bin_path $VOCAB_PATH $config_path $dataset_path $viewer_arg"
        /usr/bin/time -f "ExecutionTime=%e\nCPUUsage=%P\nMemoryKB=%M" -o "$log_output" \
          bash -c "$bin_path $VOCAB_PATH $config_path $dataset_path $viewer_arg"

        if [ ! -f KeyFrameTrajectory.txt ]; then
          echo "[FAILED] No KeyFrameTrajectory.txt produced"
          echo "[FAILED] $sequence | $mode | $viewer_flag | run_$run_id" >> "$RESULTS_DIR_BASE/kitti_failed_runs.log"
          continue
        fi

        mv KeyFrameTrajectory.txt "$kf_output"
        echo "[INFO] Saved trajectory to $kf_output"

        METRICS_SCRIPT="$HOME/ros2_test/scripts/kitti/generate_kitti_metrics.py"
        if [ -f "$gt_path" ]; then
          python3 "$METRICS_SCRIPT" --gt "$gt_path" --est "$kf_output" --out "$metrics_output"
          if [ -f "$metrics_output" ]; then
            rmse=$(jq '.rmse' "$metrics_output")
            mean=$(jq '.mean' "$metrics_output")
            median=$(jq '.median' "$metrics_output")
          else
            echo "[WARNING] No metrics.json produced!"
            rmse="N/A"
            mean="N/A"
            median="N/A"
          fi
        else
          echo "[WARNING] Ground truth not found at $gt_path"
          rmse="N/A"
          mean="N/A"
          median="N/A"
        fi

        SUMMARY_FILE="$RESULTS_DIR_BASE/kitti_final_results.csv"
        if [ ! -f "$SUMMARY_FILE" ]; then
          echo "Sequence,Mode,Viewer,Run,RMSE,Mean,Median,Exec_Time(s),CPU_Usage(%),RAM_Usage(KB)" > "$SUMMARY_FILE"
        fi
        exec_time=$(grep "ExecutionTime" "$log_output" | cut -d'=' -f2)
        cpu_usage=$(grep "CPUUsage" "$log_output" | cut -d'=' -f2)
        ram_usage=$(grep "MemoryKB" "$log_output" | cut -d'=' -f2)
        echo "$sequence,$mode,$viewer_flag,run_$run_id,$rmse,$mean,$median,$exec_time,$cpu_usage,$ram_usage" >> "$SUMMARY_FILE"

      done
    done
  done
done

echo "All KITTI evaluations completed. Results are saved in $RESULTS_DIR_BASE."
