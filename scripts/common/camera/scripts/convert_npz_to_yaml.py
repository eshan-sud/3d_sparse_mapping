
# Converts .npz calibraito file to .yaml file as per the ORB-SLAM3 format

import numpy as np

# Proper YAML formatting function
def format_yaml_line(key, value):
    """Formats YAML lines ensuring correct spacing and float precision"""
    return f"{key}: {value:.6f}" if isinstance(value, float) else f"{key}: {value}"

# Load camera calibration data
calib_data = np.load("../calibration/camera_calibration.npz")
camera_matrix = calib_data["camera_matrix"]
dist_coeffs = calib_data["dist_coeffs"]

# Convert NumPy arrays to standard Python floats
fx, fy = float(camera_matrix[0, 0]), float(camera_matrix[1, 1])
cx, cy = float(camera_matrix[0, 2]), float(camera_matrix[1, 2])
k1, k2, p1, p2, k3 = map(float, dist_coeffs.flatten())

# Camera settings
camera_width = 640   # Modify based on your camera resolution
camera_height = 480
camera_fps = 30.0

# Create YAML content as a list of lines
yaml_lines = [
    "%YAML:1.0",
    "",
    "#--------------------------------------------------------------------------------------------",
    "# Camera Parameters. Adjust them!",
    "#--------------------------------------------------------------------------------------------",
    'Camera.type: "PinHole"',
    "",
    "# Camera calibration and distortion parameters (OpenCV)",
    format_yaml_line("Camera.fx", fx),
    format_yaml_line("Camera.fy", fy),
    format_yaml_line("Camera.cx", cx),
    format_yaml_line("Camera.cy", cy),
    "",
    format_yaml_line("Camera.k1", k1),
    format_yaml_line("Camera.k2", k2),
    format_yaml_line("Camera.p1", p1),
    format_yaml_line("Camera.p2", p2),
    format_yaml_line("Camera.k3", k3),
    "",
    "# Camera frames per second",
    format_yaml_line("Camera.fps", camera_fps),
    "",
    "# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)",
    format_yaml_line("Camera.RGB", 1),
    "",
    "# Camera resolution",
    format_yaml_line("Camera.width", camera_width),
    format_yaml_line("Camera.height", camera_height),
    "",
    "#--------------------------------------------------------------------------------------------",
    "# ORB Parameters",
    "#--------------------------------------------------------------------------------------------",
    "",
    "# ORB Extractor: Number of features per image",
    format_yaml_line("ORBextractor.nFeatures", 1000),
    "",
    "# ORB Extractor: Scale factor between levels in the scale pyramid",
    format_yaml_line("ORBextractor.scaleFactor", 1.2),
    "",
    "# ORB Extractor: Number of levels in the scale pyramid",
    format_yaml_line("ORBextractor.nLevels", 8),
    "",
    "# ORB Extractor: Fast threshold",
    "# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.",
    "# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST",
    "# You can lower these values if your images have low contrast",
    format_yaml_line("ORBextractor.iniThFAST", 20),
    format_yaml_line("ORBextractor.minThFAST", 7),
    "",
    "#--------------------------------------------------------------------------------------------",
    "# Viewer Parameters",
    "#--------------------------------------------------------------------------------------------",
    format_yaml_line("Viewer.KeyFrameSize", 0.05),
    format_yaml_line("Viewer.KeyFrameLineWidth", 1),
    format_yaml_line("Viewer.GraphLineWidth", 0.9),
    format_yaml_line("Viewer.PointSize", 2),
    format_yaml_line("Viewer.CameraSize", 0.08),
    format_yaml_line("Viewer.CameraLineWidth", 3),
    format_yaml_line("Viewer.ViewpointX", 0),
    format_yaml_line("Viewer.ViewpointY", -0.7),
    format_yaml_line("Viewer.ViewpointZ", -1.8),
    format_yaml_line("Viewer.ViewpointF", 500),
]

# Save YAML file
yaml_filename = "../../../my_camera.yaml"
with open(yaml_filename, "w") as file:
    file.write("\n".join(yaml_lines))

print(f"ORB-SLAM3 YAML file saved as: {yaml_filename}")
print("Open the file and verify the parameters before running ORB-SLAM3!!")
