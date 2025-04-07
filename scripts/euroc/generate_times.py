
# Script to generate the MH01.txt times file required to execute ORB-SLAM3 on EuRoC data

import os

# Path setup
dataset_root = '/home/minor-project/ros2_test/Datasets/EuRoC/MH01/mav0'
output_file = os.path.join(dataset_root, 'MH01.txt')

# Full paths to image folders
cam0_dir = os.path.join(dataset_root, 'cam0', 'data')
cam1_dir = os.path.join(dataset_root, 'cam1', 'data')

# Path to timestamps file (EuRoC format)
csv_path = os.path.join(dataset_root, 'cam0', 'data.csv')
with open(csv_path, 'r') as f:
    lines = f.readlines()

# Skip header if present
if 'timestamp' in lines[0]:
    lines = lines[1:]

valid_count = 0
with open(output_file, 'w') as out:
    for line in lines:
        timestamp = line.strip().split(',')[0]
        img0 = os.path.join(cam0_dir, f"{timestamp}.png")
        img1 = os.path.join(cam1_dir, f"{timestamp}.png")

        if os.path.isfile(img0) and os.path.isfile(img1):
            out.write(f"{timestamp}\n")
            valid_count += 1

print(f"Saved {valid_count} valid stereo timestamps to {output_file}")
