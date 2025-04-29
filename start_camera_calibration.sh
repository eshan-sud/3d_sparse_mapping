

#!/bin/bash

echo ""
echo "================= CAMERA CALIBRATION SCRIPT ================="
echo ""
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Step 1: Run camera_calibration.py
echo "[1/2] Running camera calibration using OpenCV..."
python3 "$SCRIPT_DIR/camera_calibration.py"
# Step 2: Convert .npz to ORB-SLAM3 .yaml format
echo "[2/2] Converting .npz to ORB-SLAM3-compatible .yaml..."
python3 "$SCRIPT_DIR/convert_npz_to_yaml.py"
echo ""
echo "Calibration completed. YAML saved in ~/ros2_test/scripts/common/ directory"
echo ""
echo "================= CAMERA CALIBRATION END ===================="
echo ""
