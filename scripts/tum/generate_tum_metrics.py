#!/usr/bin/env python3
import numpy as np
import argparse
import json

def compute_r2_score(y_true, y_pred):
    ss_res = np.sum((y_true - y_pred) ** 2)
    ss_tot = np.sum((y_true - np.mean(y_true, axis=0)) ** 2)
    return 1 - (ss_res / ss_tot) if ss_tot != 0 else 0.0

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
            parts = line.strip().split()  # CSVs are whitespace-delimited for TUM ground truth
            if len(parts) < 4:
                continue
            timestamp = float(parts[0])
            pose = np.array([float(parts[1]), float(parts[2]), float(parts[3])])  # x, y, z
            data.append((timestamp, pose))
    return data

def compute_metrics(gt_traj, est_traj):
    gt_dict = dict(gt_traj)
    errors = []
    matched_gt = []
    matched_est = []

    for t_est, p_est in est_traj:
        if not gt_dict:
            continue
        closest_t = min(gt_dict.keys(), key=lambda t: abs(t - t_est))
        p_gt = gt_dict[closest_t]
        error = np.linalg.norm(p_est - p_gt)
        errors.append(error)
        matched_gt.append(p_gt)
        matched_est.append(p_est)

    errors = np.array(errors)
    matched_gt = np.array(matched_gt)
    matched_est = np.array(matched_est)

    # Compute metrics
    rmse = float(np.sqrt(np.mean(errors**2)))
    mean = float(np.mean(errors))
    median = float(np.median(errors))
    stddev = float(np.std(errors))
    precision = 1.0 / stddev if stddev != 0 else 0.0
    max_possible_error = np.linalg.norm(np.max(matched_gt, axis=0) - np.min(matched_gt, axis=0))
    accuracy = 1.0 - (rmse / max_possible_error) if max_possible_error != 0 else 0.0

    # R² score on 3D coordinates
    r2 = float(compute_r2_score(np.array(matched_gt), np.array(matched_est)))


    # Failure rate
    failure_rate = float((len(gt_traj) - len(est_traj)) / len(gt_traj)) if len(gt_traj) > 0 else 1.0  # <-- ADDED

    # Avg FPS – assuming all timestamps are ordered and span a period
    if len(est_traj) >= 2:
        duration = est_traj[-1][0] - est_traj[0][0]
        fps = float(len(est_traj) / duration) if duration > 0 else 0.0  # <-- ADDED
    else:
        fps = 0.0

    return {
        'rmse': round(rmse, 3),
        'mean': round(mean, 3),
        'median': round(median, 3),
        'stddev': round(stddev, 3),           # <-- ADDED
        'precision': round(precision, 3),     # <-- ADDED
        'accuracy': round(accuracy, 3),       # <-- ADDED
        'r2_score': round(r2, 3),             # <-- ADDED
        'failure_rate': round(failure_rate, 3), # <-- ADDED
        'avg_fps': round(fps, 3)              # <-- ADDED
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
