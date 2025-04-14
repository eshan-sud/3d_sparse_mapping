#!/usr/bin/env python3
import numpy as np
import argparse
import json
import sys

def load_kitti_groundtruth(file):
    data = []
    with open(file, 'r') as f:
        for idx, line in enumerate(f):
            parts = line.strip().split()
            if len(parts) != 12:
                continue
            try:
                # 3x4 transformation matrix: extract translation only
                tx, ty, tz = float(parts[3]), float(parts[7]), float(parts[11])
                data.append((idx, np.array([tx, ty, tz])))
            except ValueError:
                continue
    print(f"[DEBUG] Loaded {len(data)} poses from GT file: {file}")
    return data

def load_estimated_trajectory(file):
    data = []
    with open(file, 'r') as f:
        for idx, line in enumerate(f):
            parts = line.strip().split()
            if len(parts) >= 4:
                try:
                    tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
                    data.append((idx, np.array([tx, ty, tz])))
                except ValueError:
                    continue
    print(f"[DEBUG] Loaded {len(data)} poses from estimated file: {file}")
    return data

def compute_metrics(gt_traj, est_traj):
    min_len = min(len(gt_traj), len(est_traj))
    if min_len == 0:
        raise ValueError("[ERROR] No overlapping trajectory frames found.")

    errors = []
    for i in range(min_len):
        _, gt_pose = gt_traj[i]
        _, est_pose = est_traj[i]
        error = np.linalg.norm(est_pose - gt_pose)
        errors.append(error)

    errors = np.array(errors)
    return {
        'rmse': round(float(np.sqrt(np.mean(errors**2))), 3),
        'mean': round(float(np.mean(errors)), 3),
        'median': round(float(np.median(errors)), 3)
    }

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--gt', required=True, help='Groundtruth file (KITTI format)')
    parser.add_argument('--est', required=True, help='Estimated trajectory (KeyFrameTrajectory.txt)')
    parser.add_argument('--out', required=True, help='Output JSON file')
    args = parser.parse_args()

    gt_traj = load_kitti_groundtruth(args.gt)
    est_traj = load_estimated_trajectory(args.est)

    try:
        metrics = compute_metrics(gt_traj, est_traj)
        with open(args.out, 'w') as f:
            json.dump(metrics, f, indent=2)
        print(f"[INFO] Saved metrics to {args.out}")
    except Exception as e:
        print(str(e))
        print("[WARNING] Metrics output not generated!")
        sys.exit(1)
