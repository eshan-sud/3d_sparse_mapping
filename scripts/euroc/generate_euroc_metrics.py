#!/usr/bin/env python3
import numpy as np
import argparse
import json

def compute_r2_score(y_true, y_pred):
    ss_res = np.sum((y_true - y_pred) ** 2)
    ss_tot = np.sum((y_true - np.mean(y_true, axis=0)) ** 2)
    return 1 - (ss_res / ss_tot) if ss_tot != 0 else 0.0

def load_trajectory(file, is_gt=False):
    data = []
    with open(file, 'r') as f:
        for line in f:
            if line.startswith("#") or len(line.strip()) == 0:
                continue
            parts = line.strip().split(",") if is_gt else line.strip().split()
            try:
                timestamp = float(parts[0]) / 1e9 if is_gt else float(parts[0])
                pose = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
                data.append((timestamp, pose))
            except:
                continue
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

    if not matched_gt:
        raise ValueError("[ERROR] No matching ground truth and estimation data available.")

    errors = np.array(errors)
    matched_gt = np.array(matched_gt)
    matched_est = np.array(matched_est)

    rmse = float(np.sqrt(np.mean(errors**2)))
    mean = float(np.mean(errors))
    median = float(np.median(errors))
    stddev = float(np.std(errors))
    precision = 1.0 / stddev if stddev != 0 else 0.0
    max_possible_error = np.linalg.norm(np.max(matched_gt, axis=0) - np.min(matched_gt, axis=0))
    accuracy = 1.0 - (rmse / max_possible_error) if max_possible_error != 0 else 0.0
    r2 = float(compute_r2_score(np.array(matched_gt), np.array(matched_est)))
    failure_rate = float((len(gt_traj) - len(est_traj)) / len(gt_traj)) if len(gt_traj) > 0 else 1.0
    if len(est_traj) >= 2:
        duration = est_traj[-1][0] - est_traj[0][0]
        fps = float(len(est_traj) / duration) if duration > 0 else 0.0
    else:
        fps = 0.0

    return {
        'rmse': round(rmse, 3),
        'mean': round(mean, 3),
        'median': round(median, 3),
        'stddev': round(stddev, 3),
        'precision': round(precision, 3),
        'accuracy': round(accuracy, 3),
        'r2_score': round(r2, 3),
        'failure_rate': round(failure_rate, 3),
        'avg_fps': round(fps, 3)
    }

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--gt', required=True)
    parser.add_argument('--est', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    gt_traj = load_trajectory(args.gt, is_gt=True)
    est_traj = load_trajectory(args.est, is_gt=False)
    metrics = compute_metrics(gt_traj, est_traj)

    with open(args.out, 'w') as f:
        json.dump(metrics, f, indent=2)

