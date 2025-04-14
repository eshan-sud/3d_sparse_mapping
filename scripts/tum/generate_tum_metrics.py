#!/usr/bin/env python3
import numpy as np
import argparse
import json

def load_est_trajectory(file):
    data = []
    with open(file, 'r') as f:
        for line in f:
            if line.startswith("#") or len(line.strip()) == 0:
                continue
            parts = line.strip().split()
            if len(parts) != 8:
                continue
            timestamp = float(parts[0])
            pose = np.array([float(p) for p in parts[1:4]])  # tx, ty, tz
            data.append((timestamp, pose))
    return data

def load_gt_trajectory(file):
    data = []
    with open(file, 'r') as f:
        for line in f:
            if line.startswith("#") or len(line.strip()) == 0:
                continue
            parts = line.strip().split() # ','
            if len(parts) < 4:
                continue
            timestamp = float(parts[0])
#            timestamp = float(parts[0]) * 1e-9  # convert from nanoseconds to seconds
            pose = np.array([float(parts[1]), float(parts[2]), float(parts[3])])  # x, y, z
            data.append((timestamp, pose))
    return data

def compute_metrics(gt_traj, est_traj):
    gt_dict = dict(gt_traj)
    errors = []
    for t_est, p_est in est_traj:
        # Find closest GT timestamp
        closest_t = min(gt_dict.keys(), key=lambda t: abs(t - t_est))
        p_gt = gt_dict[closest_t]
        error = np.linalg.norm(p_est - p_gt)
        errors.append(error)
    errors = np.array(errors)
    return {
        'rmse': round(float(np.sqrt(np.mean(errors**2))), 3),
        'mean': round(float(np.mean(errors)), 3),
        'median': round(float(np.median(errors)), 3)
    }

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--gt', required=True, help='Groundtruth file (CSV)')
    parser.add_argument('--est', required=True, help='Estimated trajectory (kf_true.txt)')
    parser.add_argument('--out', required=True, help='Output JSON file')
    args = parser.parse_args()

    gt_traj = load_gt_trajectory(args.gt)
    est_traj = load_est_trajectory(args.est)
    metrics = compute_metrics(gt_traj, est_traj)

    with open(args.out, 'w') as f:
        json.dump(metrics, f, indent=2)
