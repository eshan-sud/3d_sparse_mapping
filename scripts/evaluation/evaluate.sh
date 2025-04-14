#!/bin/bash

# Paths
RESULTS_DIR="$HOME/ros2_test/results"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EVALUATION_DIR="$SCRIPT_DIR"
ROOT_DIR="$(dirname "$EVALUATION_DIR")"

KITTI_SCRIPT="$ROOT_DIR/kitti/evaluate_kitti_metrics.sh"
TUM_SCRIPT="$ROOT_DIR/tum/evaluate_tum_metrics.sh"
EUROC_SCRIPT="$ROOT_DIR/euroc/evaluate_euroc_metrics.sh"
CREATE_DIRS_SCRIPT="$EVALUATION_DIR/create_results_dirs.sh"

# Function to sleep and log
sleep_and_log() {
    echo "[INFO] Sleeping for 5 minutes to let system cool down..."
    sleep 300
    echo "[INFO] Resuming..."
}

# 1. Clean up old results
echo "[INFO] Removing old results directory at $RESULTS_DIR"
rm -rf "$RESULTS_DIR"

# 2. Recreate results directory structure
echo "[INFO] Creating new results directories..."
bash "$CREATE_DIRS_SCRIPT" || { echo "[ERROR] Failed to create result directories."; exit 1; }

# 3. Evaluate EuRoC
echo "[INFO] Starting EuRoC evaluation..."
bash "$EUROC_SCRIPT" || { echo "[ERROR] EuRoC evaluation failed."; exit 1; }
sleep_and_log

# 4. Evaluate KITTI
echo "[INFO] Starting KITTI evaluation..."
bash "$KITTI_SCRIPT" || { echo "[ERROR] KITTI evaluation failed."; exit 1; }
sleep_and_log

# 5. Evaluate TUM
echo "[INFO] Starting TUM evaluation..."
bash "$TUM_SCRIPT" || { echo "[ERROR] TUM evaluation failed."; exit 1; }
sleep_and_log

echo "[SUCCESS] All evaluations completed successfully."


