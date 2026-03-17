#!/usr/bin/env python3
"""
cpd_validation.py — Validate whether a CPD-style GMM objective discriminates
the true pose from the aliased pose on lt_changes sequences.

The key question: does CPD's explicit outlier component give a better score
to the TRUE pose than the ALIASED pose, even though the naive likelihood-field
score favours the aliased one?

Run inside the Docker container (data must be attached):
  python3 /catkin_ws/src/lilocbench_ros/scripts/cpd_validation.py

Optionally pass a sequence name:
  python3 ... lt_changes_0          (default)
  python3 ... lt_changes_dynamics_0
"""

import sys
import math
import numpy as np
import yaml
from pathlib import Path

try:
    from scipy.spatial import KDTree
except ImportError:
    sys.exit("scipy not found — run: pip3 install scipy")

try:
    import rosbag
    from sensor_msgs.msg import LaserScan
except ImportError:
    sys.exit("rosbag not found — must run inside the Docker container")

# ── Hardcoded per-sequence data ──────────────────────────────────────────────

SEQ_DATA = {
    "lt_changes_0": {
        "true_pose":    (-2.738933, -4.032920,  0.037187),
        "aliased_pose": (-9.68,     -4.03,       0.037),
        "bag":          "/data/lt_changes_0/lt_changes_0_no_cams.bag",
    },
    "lt_changes_dynamics_0": {
        "true_pose":    (-1.639154, -3.787429,  3.099951),
        "aliased_pose": ( 2.88,     -2.05,       3.10),
        "bag":          "/data/lt_changes_dynamics_0/lt_changes_dynamics_0_no_cams.bag",
    },
}

MAP_YAML   = "/data/map_office/map_office.yaml"
SCAN_TOPIC = "/laser_scan_front/scan"

# Sensor offsets: front laser → base_link (from transformations.yaml)
FRONT_SX = 0.304559
FRONT_SY = -0.005977
FRONT_ST = 0.019208   # rad

# CPD sweep parameters
W_VALUES     = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
SIGMA_VALUES = [0.10, 0.20, 0.30, 0.50]   # map-point Gaussian bandwidth (metres)
MAP_DOWNSAMPLE = 4   # keep every Nth obstacle point (speed vs accuracy trade-off)
SCAN_SUBSAMPLE = 1   # keep every Nth scan ray


# ── Map loading ───────────────────────────────────────────────────────────────

def load_map(yaml_path: str):
    """Return (obstacle_points [N×2], kdtree) in world frame (metres)."""
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)

    img_path = Path(yaml_path).parent / cfg["image"]
    resolution = float(cfg["resolution"])
    origin = cfg["origin"]          # [x, y, yaw]
    ox, oy = float(origin[0]), float(origin[1])
    negate  = int(cfg.get("negate", 0))
    occ_thr = float(cfg.get("occupied_thresh", 0.65))

    # Load as greyscale
    try:
        from PIL import Image as PILImage
        img = np.array(PILImage.open(str(img_path)).convert("L"))
    except ImportError:
        try:
            import cv2
            img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
            if img is None:
                raise RuntimeError(f"cv2 could not open {img_path}")
        except ImportError:
            sys.exit("Need Pillow or opencv-python to read the map image")

    height, width = img.shape

    # ROS convention: p_occ = (255 - px)/255  when negate=0
    # occupied if p_occ > occ_thr  →  px < (1-occ_thr)*255
    px_float = img.astype(np.float32) / 255.0
    if negate:
        p_occ = px_float
    else:
        p_occ = 1.0 - px_float

    occ_mask = p_occ > occ_thr
    rows, cols = np.where(occ_mask)

    # Pixel → world: row 0 is max-y (image top = map top)
    wx = ox + cols * resolution
    wy = oy + (height - 1 - rows) * resolution

    pts = np.column_stack([wx, wy]).astype(np.float32)

    # Downsample for speed
    if MAP_DOWNSAMPLE > 1:
        pts = pts[::MAP_DOWNSAMPLE]

    print(f"Map loaded: {len(pts)} obstacle points (after {MAP_DOWNSAMPLE}× downsample)")
    return pts, KDTree(pts)


# ── Scan loading ──────────────────────────────────────────────────────────────

def load_scan_at_offset(bag_path: str, topic: str, time_offset_s: float = 0.0):
    """Return (angles_rad, ranges_m) for the first scan at or after time_offset_s from bag start."""
    bag = rosbag.Bag(bag_path, "r")
    start_time = None
    for _, msg, t in bag.read_messages(topics=[topic]):
        if start_time is None:
            start_time = t.to_sec()
        if t.to_sec() - start_time >= time_offset_s:
            angles = np.arange(msg.angle_min,
                               msg.angle_min + len(msg.ranges) * msg.angle_increment,
                               msg.angle_increment,
                               dtype=np.float32)
            ranges = np.array(msg.ranges, dtype=np.float32)
            valid  = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
            elapsed = t.to_sec() - start_time
            bag.close()
            print(f"  Scan at t={elapsed:.1f}s (offset requested: {time_offset_s:.1f}s)")
            return angles[valid][::SCAN_SUBSAMPLE], ranges[valid][::SCAN_SUBSAMPLE]
    bag.close()
    raise RuntimeError(f"No messages found on {topic} in {bag_path}")


# ── Coordinate transforms ─────────────────────────────────────────────────────

def scan_to_baselink(angles, ranges, sx, sy, st):
    """Laser-frame endpoints → base_link frame (2D)."""
    lx = ranges * np.cos(angles)
    ly = ranges * np.sin(angles)
    c, s = math.cos(st), math.sin(st)
    bx = sx + c * lx - s * ly
    by = sy + s * lx + c * ly
    return np.column_stack([bx, by]).astype(np.float32)


def transform_to_map(pts_bl, pose):
    """base_link points → map frame given (x, y, theta) robot pose."""
    x, y, theta = pose
    c, s = math.cos(theta), math.sin(theta)
    mx = x + c * pts_bl[:, 0] - s * pts_bl[:, 1]
    my = y + s * pts_bl[:, 0] + c * pts_bl[:, 1]
    return np.column_stack([mx, my]).astype(np.float32)


# ── Scoring functions ─────────────────────────────────────────────────────────

def likelihood_field_score(scan_map, kdtree, sigma=0.15):
    """
    Standard likelihood-field score: sum of log N(dist | 0, sigma).
    Returns mean log-weight per ray (comparable across different N).
    """
    dists, _ = kdtree.query(scan_map, k=1, workers=-1)
    log_w = -0.5 * (dists / sigma) ** 2
    return float(log_w.mean())


def cpd_gmm_log_likelihood(scan_map, map_pts, kdtree, sigma, w, k_nn=20):
    """
    Evaluate the CPD GMM log-likelihood at a FIXED pose (no optimisation).

    For each scan point x_n:
        p(x_n) = (1-w)/M * sum_m N(x_n | y_m, sigma^2 I)  +  w / A

    where A = spatial area of the map (normalisation for uniform outlier).
    We approximate the full sum via K nearest neighbours.

    Returns mean log p(x_n) per scan point.
    """
    M = len(map_pts)
    N = len(scan_map)

    # Estimate spatial area from map bounding box
    area = (map_pts[:, 0].ptp() * map_pts[:, 1].ptp()) or 1.0

    # K-NN query for each scan point
    k = min(k_nn, M)
    dists, _ = kdtree.query(scan_map, k=k, workers=-1)   # N × k

    # Gaussian mixture contribution (unnormalised per scan point)
    gauss_coeff = 1.0 / (2.0 * math.pi * sigma ** 2)
    mixture = gauss_coeff * np.exp(-0.5 * (dists / sigma) ** 2).sum(axis=1)  # N,

    # Scale by (1-w)/M  and add outlier floor w/A
    if w > 0.0:
        p_n = (1.0 - w) / M * mixture + w / area
    else:
        p_n = mixture / M

    # Avoid log(0) for degenerate cases
    p_n = np.maximum(p_n, 1e-300)
    return float(np.log(p_n).mean())


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    seq = sys.argv[1] if len(sys.argv) > 1 else "lt_changes_0"
    if seq not in SEQ_DATA:
        sys.exit(f"Unknown sequence '{seq}'. Choose from: {list(SEQ_DATA)}")
    time_offset_s = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0

    info = SEQ_DATA[seq]
    true_pose    = info["true_pose"]
    aliased_pose = info["aliased_pose"]
    bag_path     = info["bag"]

    print(f"\n{'='*60}")
    print(f"  CPD Validation — {seq}  (scan offset: {time_offset_s:.1f}s)")
    print(f"{'='*60}")
    print(f"  True pose:    ({true_pose[0]:.3f}, {true_pose[1]:.3f}, {math.degrees(true_pose[2]):.1f}°)")
    print(f"  Aliased pose: ({aliased_pose[0]:.3f}, {aliased_pose[1]:.3f}, {math.degrees(aliased_pose[2]):.1f}°)")
    print(f"  Bag:          {bag_path}")

    # Load data
    print("\nLoading map ...")
    map_pts, kdtree = load_map(MAP_YAML)

    print("Loading scan ...")
    angles, ranges = load_scan_at_offset(bag_path, SCAN_TOPIC, time_offset_s)
    print(f"  Scan: {len(ranges)} valid rays")

    # Convert scan to base_link then to map frame at each test pose
    pts_bl = scan_to_baselink(angles, ranges, FRONT_SX, FRONT_SY, FRONT_ST)
    scan_true    = transform_to_map(pts_bl, true_pose)
    scan_aliased = transform_to_map(pts_bl, aliased_pose)

    # ── Likelihood-field baseline ─────────────────────────────────────────────
    lf_sigma = 0.15
    lf_true    = likelihood_field_score(scan_true,    kdtree, sigma=lf_sigma)
    lf_aliased = likelihood_field_score(scan_aliased, kdtree, sigma=lf_sigma)

    print(f"\n{'─'*60}")
    print(f"  Likelihood-field score (sigma={lf_sigma}m)")
    print(f"    True:    {lf_true:+.4f}")
    print(f"    Aliased: {lf_aliased:+.4f}")
    lf_diff = lf_true - lf_aliased
    verdict = "TRUE favoured ✓" if lf_diff > 0 else "ALIASED favoured ✗"
    print(f"    Diff (true−aliased): {lf_diff:+.4f}  →  {verdict}")

    # ── CPD sweep ─────────────────────────────────────────────────────────────
    print(f"\n{'─'*60}")
    print("  CPD GMM log-likelihood sweep (mean per ray)")
    print(f"  {'sigma':>6}  {'w':>5}  {'true':>10}  {'aliased':>10}  {'diff':>10}  verdict")
    print(f"  {'─'*6}  {'─'*5}  {'─'*10}  {'─'*10}  {'─'*10}  {'─'*7}")

    best = None   # track best discriminating configuration

    for sigma in SIGMA_VALUES:
        for w in W_VALUES:
            cpd_true    = cpd_gmm_log_likelihood(scan_true,    map_pts, kdtree, sigma, w)
            cpd_aliased = cpd_gmm_log_likelihood(scan_aliased, map_pts, kdtree, sigma, w)
            diff = cpd_true - cpd_aliased
            ok   = "TRUE ✓" if diff > 0 else "ALIAS ✗"
            print(f"  {sigma:6.2f}  {w:5.2f}  {cpd_true:10.4f}  {cpd_aliased:10.4f}  {diff:+10.4f}  {ok}")
            if best is None or abs(diff) > abs(best[2]):
                best = (sigma, w, diff, ok)

    print(f"\n  Best discriminating config: sigma={best[0]}, w={best[1]}, diff={best[2]:+.4f}  ({best[3]})")
    print(f"{'='*60}\n")

    # ── Per-ray breakdown at best config ──────────────────────────────────────
    if best[2] > 0:
        print("  Per-ray inlier analysis at best config:")
        sigma, w = best[0], best[1]
        M = len(map_pts)
        area = (map_pts[:, 0].ptp() * map_pts[:, 1].ptp()) or 1.0
        k = min(20, M)

        for label, scan_map in [("True", scan_true), ("Aliased", scan_aliased)]:
            dists, _ = kdtree.query(scan_map, k=k, workers=-1)
            nn_dist = dists[:, 0]   # nearest-neighbour distance per ray

            inlier_thresh = 3 * sigma   # rays within 3-sigma of any obstacle
            n_inlier = (nn_dist < inlier_thresh).sum()
            mean_nn  = nn_dist.mean()
            print(f"    {label:7s}: nn_dist mean={mean_nn*100:.1f}cm, "
                  f"inliers(<{inlier_thresh*100:.0f}cm)={n_inlier}/{len(nn_dist)} "
                  f"({100*n_inlier/len(nn_dist):.1f}%)")


if __name__ == "__main__":
    main()
