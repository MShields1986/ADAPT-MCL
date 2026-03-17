#!/usr/bin/env python3
"""Simple TUM-format localization evaluator (Python 3.8 compatible)."""
import sys
import math
import numpy as np

def load_tum(path):
    ts, xs, ys = [], [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            ts.append(float(parts[0]))
            xs.append(float(parts[1]))
            ys.append(float(parts[2]))
    return np.array(ts), np.array(xs), np.array(ys)

def interp_pose(ts_ref, ts_est, xs_est, ys_est):
    xs_i = np.interp(ts_ref, ts_est, xs_est)
    ys_i = np.interp(ts_ref, ts_est, ys_est)
    return xs_i, ys_i

def main():
    if len(sys.argv) < 3:
        sys.exit("Usage: eval_simple.py <gt.txt> <est.txt> [start_times...]")
    gt_path  = sys.argv[1]
    est_path = sys.argv[2]
    start_times = [float(s) for s in sys.argv[3:]] if len(sys.argv) > 3 else [None]

    ts_gt, xs_gt, ys_gt   = load_tum(gt_path)
    ts_est, xs_est, ys_est = load_tum(est_path)

    if len(ts_est) == 0:
        print("ERROR: estimate file is empty")
        return

    # Crop GT to EST time range
    t0 = max(ts_gt[0], ts_est[0])
    t1 = min(ts_gt[-1], ts_est[-1])
    mask = (ts_gt >= t0) & (ts_gt <= t1)
    ts_gt  = ts_gt[mask]
    xs_gt  = xs_gt[mask]
    ys_gt  = ys_gt[mask]

    if len(ts_gt) == 0:
        print("ERROR: no overlapping time range between GT and estimate")
        return

    xs_i, ys_i = interp_pose(ts_gt, ts_est, xs_est, ys_est)
    errs = np.sqrt((xs_gt - xs_i)**2 + (ys_gt - ys_i)**2) * 100  # cm

    print(f"Estimate: {len(ts_est)} poses, GT overlap: {len(ts_gt)} samples")
    print(f"  Mean:   {errs.mean():.2f} cm")
    print(f"  Median: {np.median(errs):.2f} cm")
    print(f"  p95:    {np.percentile(errs, 95):.2f} cm")
    print(f"  Max:    {errs.max():.2f} cm")
    print(f"  <10cm:  {(errs < 10).mean()*100:.1f}%")
    print(f"  <25cm:  {(errs < 25).mean()*100:.1f}%")
    print(f"  <50cm:  {(errs < 50).mean()*100:.1f}%")

if __name__ == "__main__":
    main()
