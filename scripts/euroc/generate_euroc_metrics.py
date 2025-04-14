#!/usr/bin/env python3
import numpy as np
import argparse
import json
import sys

def load_trajectory(file):
    data = []
    with open(file, 'r') as f:
        for line in f:
            if line.startswith("#") or len(line.strip()) == 0:
                continue
            line = line.replace(',', ' ')  # Handles CSV-style input
            parts = line.strip().split()

            try:
                if len(parts) >= 4:  # Accept both estimated and groundtruth
                    timestamp = int(float(parts[0]))
                    pose = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
                    data.append((timestamp, pose))
            except ValueError:
                continue
    print(f"[DEBUG] Loaded {len(data)} poses from {file}")
    return data

def compute_metrics(gt_traj, est_traj, max_time_diff_ns=5e7):  # 50 ms tolerance
    if not gt_traj:
        raise ValueError("[ERROR] Ground truth trajectory is empty.")
    if not est_traj:
        raise ValueError("[ERROR] Estimated trajectory is empty.")

    gt_dict = dict(gt_traj)
    errors = []
    unmatched = 0

    for t_est, p_est in est_traj:
        # Find closest GT timestamp within tolerance
        possible_matches = [t for t in gt_dict if abs(t - t_est) < max_time_diff_ns]
        if not possible_matches:
            unmatched += 1
            continue
        closest_t = min(possible_matches, key=lambda t: abs(t - t_est))
        p_gt = gt_dict[closest_t]
        error = np.linalg.norm(p_est - p_gt)
        errors.append(error)

    if not errors:
        raise ValueError("[ERROR] No timestamp matches found within the allowed tolerance.")

    errors = np.array(errors)
    print(f"[INFO] Matched {len(errors)} timestamps, skipped {unmatched} (no close groundtruth match)")
    return {
        'rmse': round(float(np.sqrt(np.mean(errors**2))), 3),
        'mean': round(float(np.mean(errors)), 3),
        'median': round(float(np.median(errors)), 3)
    }

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--gt', required=True, help='Groundtruth file')
    parser.add_argument('--est', required=True, help='Estimated trajectory')
    parser.add_argument('--out', required=True, help='Output JSON file')
    args = parser.parse_args()

    gt_traj = load_trajectory(args.gt)
    est_traj = load_trajectory(args.est)

    try:
        metrics = compute_metrics(gt_traj, est_traj)
        with open(args.out, 'w') as f:
            json.dump(metrics, f, indent=2)
        print(f"[INFO] Saved metrics to {args.out}")
    except Exception as e:
        print(str(e))
        print("[WARNING] Metrics output not generated!")
        sys.exit(1)
