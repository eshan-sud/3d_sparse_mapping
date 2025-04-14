# generate_times.py
import os
import sys

if len(sys.argv) < 2:
    print("Usage: python3 generate_times.py <path_to_mav0>")
    sys.exit(1)

dataset_root = sys.argv[1]
output_file = os.path.join(dataset_root, os.path.basename(os.path.dirname(dataset_root)) + ".txt")

cam0_dir = os.path.join(dataset_root, 'cam0', 'data')
cam1_dir = os.path.join(dataset_root, 'cam1', 'data')
csv_path = os.path.join(dataset_root, 'cam0', 'data.csv')

with open(csv_path, 'r') as f:
    lines = f.readlines()

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

print(f"[{os.path.basename(os.path.dirname(dataset_root))}] Saved {valid_count} valid stereo timestamps to {output_file}")
