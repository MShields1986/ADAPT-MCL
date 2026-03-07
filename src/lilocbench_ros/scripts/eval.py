#!/usr/bin/env python3
"""TUM pose evaluator — takes GT and EST paths as arguments."""
import sys
import numpy as np

def load_tum(path):
    ts, poses = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            v = list(map(float, line.split()))
            if len(v) < 8:
                continue
            ts.append(v[0])
            poses.append(v[1:8])  # x y z qx qy qz qw
    return np.array(ts, dtype=float), np.array(poses, dtype=float)

def interp_poses(est_ts, est_poses, gt_ts):
    out = []
    for t in gt_ts:
        idx = np.searchsorted(est_ts, t)
        if idx == 0:
            out.append(est_poses[0])
        elif idx >= len(est_ts):
            out.append(est_poses[-1])
        else:
            t0, t1 = est_ts[idx-1], est_ts[idx]
            alpha = (t - t0) / (t1 - t0)
            out.append(est_poses[idx-1] * (1 - alpha) + est_poses[idx] * alpha)
    return np.array(out)

def evaluate(gt_path, est_path, start_time=0.0):
    gt_ts, gt_poses = load_tum(gt_path)
    est_ts, est_poses = load_tum(est_path)

    # Apply start_time offset relative to gt start
    abs_start = gt_ts[0] + start_time

    mask = (gt_ts >= max(abs_start, est_ts[0])) & (gt_ts <= est_ts[-1])
    gt_ts_c = gt_ts[mask]
    gt_poses_c = gt_poses[mask]

    if len(gt_ts_c) == 0:
        print("No overlapping GT timestamps found.")
        return

    est_interp = interp_poses(est_ts, est_poses, gt_ts_c)
    pos_err = np.linalg.norm(gt_poses_c[:, :2] - est_interp[:, :2], axis=1)

    duration = gt_ts_c[-1] - gt_ts_c[0]
    print(f"start_time={start_time:.0f}s  GT range: {duration:.1f}s  ({len(pos_err)} samples)")
    print(f"  mean={np.mean(pos_err)*100:.2f}cm  median={np.median(pos_err)*100:.2f}cm  "
          f"p95={np.percentile(pos_err,95)*100:.2f}cm  max={np.max(pos_err)*100:.2f}cm")
    thresholds = [0.02, 0.04, 0.06, 0.08, 0.10, 0.20]
    scores     = [100,  80,   60,   40,   20,   10]
    parts = []
    for t, s in zip(thresholds, scores):
        frac = np.mean(pos_err < t) * 100
        parts.append(f"<{t*100:.0f}cm:{frac:.0f}%")
    print("  " + "  ".join(parts))
    return np.mean(pos_err)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <gt.txt> <est.txt> [start_time1 start_time2 ...]")
        sys.exit(1)

    gt_path  = sys.argv[1]
    est_path = sys.argv[2]
    starts   = [float(s) for s in sys.argv[3:]] if len(sys.argv) > 3 else [0.0]

    print(f"GT:  {gt_path}")
    print(f"EST: {est_path}")
    for st in starts:
        evaluate(gt_path, est_path, st)
