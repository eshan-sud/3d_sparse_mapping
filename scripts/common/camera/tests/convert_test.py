import numpy as np
import yaml

# Load camera calibration data
calib_data = np.load("../calibration/camera_calibration.npz")
camera_matrix = calib_data["camera_matrix"]
dist_coeffs = calib_data["dist_coeffs"]

# Convert NumPy arrays to standard Python floats
fx, fy = float(camera_matrix[0, 0]), float(camera_matrix[1, 1])
cx, cy = float(camera_matrix[0, 2]), float(camera_matrix[1, 2])
k1, k2, p1, p2, k3 = map(float, dist_coeffs.flatten())

# Camera settings (modify if needed)
camera_width = 640   # Change based on your camera resolution
camera_height = 480
camera_fps = 30.0

# Baseline times focal length (only for stereo, keep as default for monocular)
camera_bf = 40.0  

# Create a dictionary for the YAML file
orb_slam3_config = {
    "Camera.type": "PinHole",
    "Camera.fx": fx,
    "Camera.fy": fy,
    "Camera.cx": cx,
    "Camera.cy": cy,
    "Camera.k1": k1,
    "Camera.k2": k2,
    "Camera.p1": p1,
    "Camera.p2": p2,
    "Camera.k3": k3,
    "Camera.width": camera_width,
    "Camera.height": camera_height,
    "Camera.fps": camera_fps,
    "Camera.bf": camera_bf,
    "Camera.RGB": 1,  # 1 = RGB, 0 = BGR
    "ThDepth": 40.0,  # Depth threshold
    "ORBextractor.nFeatures": 1000,
    "ORBextractor.scaleFactor": 1.2,
    "ORBextractor.nLevels": 8,
    "ORBextractor.iniThFAST": 20,
    "ORBextractor.minThFAST": 7,
    "Viewer.KeyFrameSize": 0.05,
    "Viewer.ViewpointX": 0,
    "Viewer.ViewpointY": -0.7,
    "Viewer.ViewpointZ": -1.8,
    "Viewer.ViewpointF": 500
}

# Save as a properly formatted YAML file
yaml_filename = "test.yaml"
with open(yaml_filename, "w") as file:
    yaml.dump(orb_slam3_config, file, default_flow_style=False)

print(f"ORB-SLAM3 YAML file saved as: {yaml_filename}")
print("Open the file and verify the parameters before running ORB-SLAM3!!")
