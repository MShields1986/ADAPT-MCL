#!/usr/bin/env python3
"""
Phase 0.2: Extract keyframes from the mapping run at regular spatial intervals.

For each keyframe, stores:
  - pose (x, y, theta) from GT interpolated to camera timestamp
  - paths to color and depth images for front/left/right cameras
  - timestamp

Output: keyframes.pkl — list of dicts
"""

import os
import math
import pickle
import argparse
import bisect
import numpy as np

DATA_DIR = os.environ.get("LILOCBENCH_DATA", "/home/matthew/Desktop/LILocBench/data") + "/mapping"
OUTPUT_DIR = os.environ.get("LILOCBENCH_MODELS", "/home/matthew/Desktop/LILocBench/lilocbench_ws/models/vpr")


def load_gt_poses(path):
    """Load TUM-format GT poses. Returns (timestamps, x, y, theta) arrays."""
    data = np.loadtxt(path)
    ts = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    # Extract yaw from quaternion: qz=col6, qw=col7
    qz = data[:, 6]
    qw = data[:, 7]
    theta = 2.0 * np.arctan2(qz, qw)
    return ts, x, y, theta


def interpolate_pose(gt_ts, gt_x, gt_y, gt_theta, query_t):
    """Linearly interpolate GT pose at query timestamp."""
    idx = bisect.bisect_right(gt_ts, query_t) - 1
    if idx < 0:
        idx = 0
    if idx >= len(gt_ts) - 1:
        idx = len(gt_ts) - 2

    t0, t1 = gt_ts[idx], gt_ts[idx + 1]
    if t1 == t0:
        alpha = 0.0
    else:
        alpha = (query_t - t0) / (t1 - t0)
    alpha = max(0.0, min(1.0, alpha))

    x = gt_x[idx] + alpha * (gt_x[idx + 1] - gt_x[idx])
    y = gt_y[idx] + alpha * (gt_y[idx + 1] - gt_y[idx])

    # Interpolate angle with wrapping
    dtheta = gt_theta[idx + 1] - gt_theta[idx]
    dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))  # wrap to [-pi, pi]
    theta = gt_theta[idx] + alpha * dtheta

    return x, y, theta


def get_sorted_timestamps(image_dir):
    """Get sorted list of timestamps from image filenames."""
    files = os.listdir(image_dir)
    timestamps = []
    for f in files:
        if f.endswith('.png'):
            t = float(f.replace('.png', ''))
            timestamps.append((t, f))
    timestamps.sort()
    return timestamps


def find_nearest(sorted_ts_list, query_t, max_dt=0.05):
    """Find nearest timestamp in sorted list. Returns (timestamp, filename) or None."""
    ts_only = [t for t, _ in sorted_ts_list]
    idx = bisect.bisect_right(ts_only, query_t)

    best = None
    best_dt = max_dt
    for i in [idx - 1, idx]:
        if 0 <= i < len(sorted_ts_list):
            dt = abs(sorted_ts_list[i][0] - query_t)
            if dt < best_dt:
                best_dt = dt
                best = sorted_ts_list[i]
    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--spacing", type=float, default=0.5,
                        help="Spatial interval between keyframes in metres (default: 0.5)")
    parser.add_argument("--max-rotation-rate", type=float, default=0.3,
                        help="Max heading change per frame in rad to accept (default: 0.3 ~17deg)")
    parser.add_argument("--data-dir", type=str, default=DATA_DIR)
    parser.add_argument("--output", type=str, default=os.path.join(OUTPUT_DIR, "keyframes.pkl"))
    args = parser.parse_args()

    print(f"Loading GT poses from {args.data_dir}/gt_poses.txt ...")
    gt_ts, gt_x, gt_y, gt_theta = load_gt_poses(os.path.join(args.data_dir, "gt_poses.txt"))
    print(f"  {len(gt_ts)} poses, {gt_ts[-1]-gt_ts[0]:.1f}s")

    # Load camera timestamps for all 3 cameras, color + depth
    cameras = ["camera_front", "camera_left", "camera_right"]
    cam_timestamps = {}
    for cam in cameras:
        for modality in ["color", "depth"]:
            key = f"{cam}/{modality}"
            img_dir = os.path.join(args.data_dir, cam, modality, "images")
            ts_list = get_sorted_timestamps(img_dir)
            cam_timestamps[key] = ts_list
            print(f"  {key}: {len(ts_list)} images")

    # Use front color timestamps as the primary timeline
    front_color = cam_timestamps["camera_front/color"]
    print(f"\nSelecting keyframes with {args.spacing}m spacing ...")

    keyframes = []
    last_kf_x, last_kf_y = None, None
    skipped_rotation = 0
    prev_theta = None

    for t_cam, fname in front_color:
        # Skip if outside GT range
        if t_cam < gt_ts[0] or t_cam > gt_ts[-1]:
            continue

        x, y, theta = interpolate_pose(gt_ts, gt_x, gt_y, gt_theta, t_cam)

        # Check rotation rate (skip frames during fast turns)
        if prev_theta is not None:
            dtheta = abs(math.atan2(math.sin(theta - prev_theta), math.cos(theta - prev_theta)))
            if dtheta > args.max_rotation_rate:
                skipped_rotation += 1
                prev_theta = theta
                continue
        prev_theta = theta

        # Check spatial distance from last keyframe
        if last_kf_x is not None:
            dist = math.sqrt((x - last_kf_x) ** 2 + (y - last_kf_y) ** 2)
            if dist < args.spacing:
                continue

        # This is a keyframe — find matching images for all cameras
        kf = {
            "timestamp": t_cam,
            "pose": (x, y, theta),
            "images": {},
        }

        all_found = True
        for cam in cameras:
            for modality in ["color", "depth"]:
                key = f"{cam}/{modality}"
                match = find_nearest(cam_timestamps[key], t_cam)
                if match is None:
                    all_found = False
                    break
                img_path = os.path.join(args.data_dir, cam, modality, "images", match[1])
                kf["images"][key] = {
                    "path": img_path,
                    "timestamp": match[0],
                    "dt": match[0] - t_cam,
                }
            if not all_found:
                break

        if not all_found:
            continue

        keyframes.append(kf)
        last_kf_x, last_kf_y = x, y

    print(f"  Selected {len(keyframes)} keyframes ({skipped_rotation} frames skipped for rotation)")

    # Summary statistics
    if len(keyframes) > 1:
        dists = []
        for i in range(1, len(keyframes)):
            dx = keyframes[i]["pose"][0] - keyframes[i - 1]["pose"][0]
            dy = keyframes[i]["pose"][1] - keyframes[i - 1]["pose"][1]
            dists.append(math.sqrt(dx * dx + dy * dy))
        print(f"  Inter-keyframe distance: mean={np.mean(dists):.2f}m, "
              f"min={np.min(dists):.2f}m, max={np.max(dists):.2f}m")
        total_path = sum(dists)
        print(f"  Total path length: {total_path:.1f}m")

    # Save
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "wb") as f:
        pickle.dump(keyframes, f)
    print(f"\nSaved to {args.output}")


if __name__ == "__main__":
    main()
