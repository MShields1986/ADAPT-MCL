#!/usr/bin/env python3
"""
Generate VPR candidates for a sequence using multi-camera MixVPR.

Queries the database with all 3 cameras from the first frame of the test
sequence. For each camera, retrieves top-k candidates. Then merges and
deduplicates (poses within merge_radius are combined, keeping the best score).

Usage:
    docker compose run --rm vpr scripts/vpr/publish_vpr_candidates_multicam.py \
        --sequence static_0 --top-k 10 --output-file /data/static_0/vpr_candidates.txt
"""

import os
import sys
import math
import bisect
import pickle
import argparse
import numpy as np
import torch
import torchvision.transforms as T
from PIL import Image

DATA_BASE = os.environ.get("LILOCBENCH_DATA", "/data")
MODELS_DIR = os.environ.get("LILOCBENCH_MODELS", "/models/vpr")
MIXVPR_REPO = os.environ.get("MIXVPR_REPO", "/workspace/mixvpr_repo")

CAMERAS = ["camera_front", "camera_left", "camera_right"]


def build_model(backbone="resnet18", out_channels=256, out_rows=4,
                mix_depth=4, layers_to_freeze=0):
    sys.path.insert(0, MIXVPR_REPO)
    from models.helper import get_backbone, get_aggregator

    bb = get_backbone(backbone, pretrained=True,
                      layers_to_freeze=layers_to_freeze, layers_to_crop=[4])
    with torch.no_grad():
        feat = bb(torch.randn(1, 3, 480, 640))
        in_ch, in_h, in_w = feat.shape[1], feat.shape[2], feat.shape[3]

    agg = get_aggregator("mixvpr", {
        "in_channels": in_ch, "in_h": in_h, "in_w": in_w,
        "out_channels": out_channels, "out_rows": out_rows,
        "mix_depth": mix_depth, "mlp_ratio": 1,
    })

    class Model(torch.nn.Module):
        def __init__(self, backbone, aggregator):
            super().__init__()
            self.backbone = backbone
            self.aggregator = aggregator
        def forward(self, x):
            return self.aggregator(self.backbone(x))

    return Model(bb, agg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sequence", type=str, required=True)
    parser.add_argument("--database", type=str,
                        default=os.path.join(MODELS_DIR, "database_mixvpr_multicam.pkl"))
    parser.add_argument("--index", type=str,
                        default=os.path.join(MODELS_DIR, "place_index_mixvpr_multicam.faiss"))
    parser.add_argument("--weights", type=str,
                        default=os.path.join(MODELS_DIR, "mixvpr_multicam_finetuned.pth"))
    parser.add_argument("--backbone", type=str, default="resnet18")
    parser.add_argument("--out-channels", type=int, default=256)
    parser.add_argument("--out-rows", type=int, default=4)
    parser.add_argument("--mix-depth", type=int, default=4)
    parser.add_argument("--top-k", type=int, default=10,
                        help="Final number of candidates after merging")
    parser.add_argument("--per-cam-k", type=int, default=10,
                        help="Candidates to retrieve per camera before merging")
    parser.add_argument("--merge-radius", type=float, default=1.0,
                        help="Merge candidates within this distance (m)")
    parser.add_argument("--output-file", type=str, required=True)
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    import faiss

    with open(args.database, "rb") as f:
        db_entries = pickle.load(f)
    index = faiss.read_index(args.index)
    print(f"Database: {len(db_entries)} entries, index: {index.ntotal}")

    model = build_model(args.backbone, args.out_channels, args.out_rows, args.mix_depth)
    if os.path.exists(args.weights):
        model.load_state_dict(torch.load(args.weights, map_location=device))
        print(f"Loaded weights: {args.weights}")
    model = model.to(device).eval()

    transform = T.Compose([
        T.Resize((480, 640)), T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])

    # Query with each camera
    all_candidates = []  # (x, y, theta, score, camera, l2_dist)

    for cam in CAMERAS:
        img_dir = os.path.join(DATA_BASE, args.sequence, cam, "color", "images")
        first_img = sorted(os.listdir(img_dir))[0]
        img_path = os.path.join(img_dir, first_img)
        print(f"\nQuery {cam}: {first_img}")

        img = Image.open(img_path).convert("RGB")
        with torch.no_grad():
            desc = model(transform(img).unsqueeze(0).to(device)).cpu().numpy()
        desc = desc / (np.linalg.norm(desc) + 1e-8)

        D, I = index.search(desc.astype(np.float32), args.per_cam_k)

        for j in range(args.per_cam_k):
            idx = I[0][j]
            entry = db_entries[idx]
            p = entry["pose"]
            score = 1.0 / (1.0 + D[0][j])
            all_candidates.append({
                "x": p[0], "y": p[1], "theta": p[2],
                "score": score, "l2": D[0][j],
                "query_cam": cam, "db_cam": entry["camera"],
                "kf_idx": entry["keyframe_idx"],
            })
            print(f"  {j+1}. ({p[0]:.2f}, {p[1]:.2f}) "
                  f"db:{entry['camera']} kf{entry['keyframe_idx']} "
                  f"L2={D[0][j]:.3f} score={score:.3f}")

    # Sort by score (best first)
    all_candidates.sort(key=lambda c: c["score"], reverse=True)

    # Merge candidates within merge_radius — keep the best-scoring one
    merged = []
    for cand in all_candidates:
        is_dup = False
        for m in merged:
            dist = math.sqrt((cand["x"] - m["x"])**2 + (cand["y"] - m["y"])**2)
            if dist < args.merge_radius:
                is_dup = True
                break
        if not is_dup:
            merged.append(cand)

    # Take top-k after merging
    merged = merged[:args.top_k]

    print(f"\n--- Merged candidates ({len(merged)}) ---")
    for i, c in enumerate(merged):
        print(f"  {i+1}. ({c['x']:.2f}, {c['y']:.2f}, {math.degrees(c['theta']):.1f}deg) "
              f"score={c['score']:.3f} query:{c['query_cam']} db:{c['db_cam']}")

    # Write output
    os.makedirs(os.path.dirname(args.output_file), exist_ok=True)
    with open(args.output_file, "w") as f:
        for c in merged:
            f.write(f"{c['x']} {c['y']} {c['theta']} {c['score']}\n")

    print(f"\nWrote {len(merged)} candidates to {args.output_file}")


if __name__ == "__main__":
    main()
