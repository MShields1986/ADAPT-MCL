#!/usr/bin/env python3
"""
Generate VPR candidates for a sequence using MixVPR.

Usage:
    docker compose run vpr scripts/vpr/publish_vpr_candidates_mixvpr.py \
        --sequence static_0 --top-k 10 --output-file /data/static_0/vpr_candidates.txt
"""

import os
import sys
import math
import pickle
import argparse
import numpy as np
import torch
import torchvision.transforms as T
from PIL import Image

DATA_BASE = os.environ.get("LILOCBENCH_DATA", "/data")
MODELS_DIR = os.environ.get("LILOCBENCH_MODELS", "/models/vpr")
MIXVPR_REPO = os.environ.get("MIXVPR_REPO", "/workspace/mixvpr_repo")


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
    parser.add_argument("--camera", type=str, default="camera_front")
    parser.add_argument("--database", type=str,
                        default=os.path.join(MODELS_DIR, "database_mixvpr.pkl"))
    parser.add_argument("--index", type=str,
                        default=os.path.join(MODELS_DIR, "place_index_mixvpr.faiss"))
    parser.add_argument("--weights", type=str,
                        default=os.path.join(MODELS_DIR, "mixvpr_finetuned.pth"))
    parser.add_argument("--backbone", type=str, default="resnet18")
    parser.add_argument("--out-channels", type=int, default=256)
    parser.add_argument("--out-rows", type=int, default=4)
    parser.add_argument("--mix-depth", type=int, default=4)
    parser.add_argument("--top-k", type=int, default=10)
    parser.add_argument("--output-file", type=str, required=True)
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    import faiss

    # Load database and index
    with open(args.database, "rb") as f:
        keyframes = pickle.load(f)
    index = faiss.read_index(args.index)
    poses = [kf["pose"] for kf in keyframes]

    # Build and load model
    model = build_model(args.backbone, args.out_channels, args.out_rows, args.mix_depth)
    if os.path.exists(args.weights):
        model.load_state_dict(torch.load(args.weights, map_location=device))
        print(f"Loaded weights: {args.weights}")
    model = model.to(device).eval()

    transform = T.Compose([
        T.Resize((480, 640)), T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])

    # Get first image from the sequence
    img_dir = os.path.join(DATA_BASE, args.sequence, args.camera, "color", "images")
    first_img = sorted(os.listdir(img_dir))[0]
    img_path = os.path.join(img_dir, first_img)
    print(f"Query: {img_path}")

    img = Image.open(img_path).convert("RGB")
    with torch.no_grad():
        desc = model(transform(img).unsqueeze(0).to(device)).cpu().numpy()
    desc = desc / (np.linalg.norm(desc) + 1e-8)

    D, I = index.search(desc.astype(np.float32), args.top_k)

    # Write candidates
    os.makedirs(os.path.dirname(args.output_file), exist_ok=True)
    with open(args.output_file, "w") as f:
        for j in range(args.top_k):
            idx = I[0][j]
            p = poses[idx]
            w = 1.0 / (1.0 + D[0][j])
            f.write(f"{p[0]} {p[1]} {p[2]} {w}\n")

    print(f"\nTop-{args.top_k} candidates:")
    for j in range(args.top_k):
        idx = I[0][j]
        p = poses[idx]
        w = 1.0 / (1.0 + D[0][j])
        print(f"  {j+1}. ({p[0]:.2f}, {p[1]:.2f}, {math.degrees(p[2]):.1f}) "
              f"w={w:.3f} L2={D[0][j]:.3f}")

    print(f"\nWrote to {args.output_file}")


if __name__ == "__main__":
    main()
