#!/usr/bin/env python3
"""
Phase 4.1: Evaluate place retrieval recall on the mapping sequence.

Leave-one-out evaluation: for each keyframe, query against the database
excluding keyframes within an exclusion radius (to avoid trivial matches
from the same location on the same trajectory).

Reports Recall@1, @3, @5 at various distance thresholds.
"""

import os
import math
import pickle
import argparse
import numpy as np

GLOBAL_LOC_DIR = os.environ.get("LILOCBENCH_MODELS", "/home/matthew/Desktop/LILocBench/lilocbench_ws/models/vpr")


def pose_distance(p1, p2):
    """Euclidean distance between two (x, y, theta) poses."""
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def heading_error(p1, p2):
    """Absolute heading error in degrees."""
    dtheta = p1[2] - p2[2]
    dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
    return math.degrees(abs(dtheta))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--database", type=str,
                        default=os.path.join(GLOBAL_LOC_DIR, "database.pkl"))
    parser.add_argument("--exclusion-radius", type=float, default=2.0,
                        help="Exclude database keyframes within this distance (m) of query")
    parser.add_argument("--correct-threshold", type=float, nargs="+",
                        default=[1.0, 2.0, 3.0, 5.0],
                        help="Distance thresholds for 'correct' retrieval (m)")
    parser.add_argument("--k-values", type=int, nargs="+", default=[1, 3, 5])
    args = parser.parse_args()

    with open(args.database, "rb") as f:
        keyframes = pickle.load(f)
    print(f"Loaded {len(keyframes)} keyframes")

    # Build descriptor matrix
    descriptors = np.stack([kf["descriptor"] for kf in keyframes]).astype(np.float32)
    poses = [kf["pose"] for kf in keyframes]

    import faiss
    n, d = descriptors.shape
    print(f"Descriptor dimension: {d}")

    # For each query, find matches excluding nearby keyframes
    max_k = max(args.k_values) + 50  # retrieve extra to handle exclusions

    results = {thresh: {k: 0 for k in args.k_values} for thresh in args.correct_threshold}
    n_valid = 0
    pos_errors_at_1 = []
    heading_errors_at_1 = []

    for qi in range(n):
        query = descriptors[qi:qi + 1]
        query_pose = poses[qi]

        # Search against full index
        index = faiss.IndexFlatL2(d)
        index.add(descriptors)
        D, I = index.search(query, max_k)

        # Filter out keyframes within exclusion radius
        filtered = []
        for j in range(max_k):
            db_idx = I[0][j]
            if db_idx == qi:
                continue
            dist_to_query = pose_distance(query_pose, poses[db_idx])
            if dist_to_query < args.exclusion_radius:
                continue
            filtered.append((db_idx, D[0][j]))
            if len(filtered) >= max(args.k_values):
                break

        if len(filtered) < max(args.k_values):
            continue  # not enough non-excluded candidates

        n_valid += 1

        # Check recall at various k and thresholds
        for thresh in args.correct_threshold:
            for k in args.k_values:
                top_k = filtered[:k]
                for db_idx, _ in top_k:
                    if pose_distance(query_pose, poses[db_idx]) < thresh:
                        results[thresh][k] += 1
                        break

        # Position and heading error of top-1 match
        top1_idx = filtered[0][0]
        pos_errors_at_1.append(pose_distance(query_pose, poses[top1_idx]))
        heading_errors_at_1.append(heading_error(query_pose, poses[top1_idx]))

    print(f"\nValid queries: {n_valid}/{n} (excl. radius={args.exclusion_radius}m)")

    # Print recall table
    print(f"\n{'Threshold':>10s}", end="")
    for k in args.k_values:
        print(f"  Recall@{k:d}", end="")
    print()
    print("-" * (10 + 10 * len(args.k_values)))
    for thresh in args.correct_threshold:
        print(f"{thresh:>8.1f}m ", end="")
        for k in args.k_values:
            recall = results[thresh][k] / n_valid * 100
            print(f"  {recall:6.1f}%", end="")
        print()

    # Position error stats for top-1
    pos_errors = np.array(pos_errors_at_1)
    head_errors = np.array(heading_errors_at_1)
    print(f"\nTop-1 match position error:")
    print(f"  mean={pos_errors.mean():.2f}m, median={np.median(pos_errors):.2f}m, "
          f"p95={np.percentile(pos_errors, 95):.2f}m, max={pos_errors.max():.2f}m")
    print(f"Top-1 match heading error:")
    print(f"  mean={head_errors.mean():.1f}°, median={np.median(head_errors):.1f}°, "
          f"p95={np.percentile(head_errors, 95):.1f}°, max={head_errors.max():.1f}°")


if __name__ == "__main__":
    main()
