#!/usr/bin/env python3
"""
Evaluate MixVPR retrieval recall on the mapping keyframes.

Leave-one-out: for each keyframe, query against the database excluding
±5 sequential neighbours. Reports Recall@k at various distance thresholds.

Also generates VPR candidates for all 4 test sequences and reports
distances to GT initial poses.

Usage:
    docker compose run vpr scripts/vpr/eval_retrieval_mixvpr.py
    docker compose run vpr scripts/vpr/eval_retrieval_mixvpr.py --weights /models/vpr/mixvpr_finetuned.pth
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


def pose_dist(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--keyframes", type=str,
                        default=os.path.join(MODELS_DIR, "keyframes.pkl"))
    parser.add_argument("--weights", type=str,
                        default=os.path.join(MODELS_DIR, "mixvpr_finetuned.pth"))
    parser.add_argument("--backbone", type=str, default="resnet18")
    parser.add_argument("--out-channels", type=int, default=256)
    parser.add_argument("--out-rows", type=int, default=4)
    parser.add_argument("--mix-depth", type=int, default=4)
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # Load keyframes
    with open(args.keyframes, "rb") as f:
        keyframes = pickle.load(f)
    poses = [kf["pose"] for kf in keyframes]
    n = len(keyframes)

    # Build and load model
    model = build_model(args.backbone, args.out_channels, args.out_rows, args.mix_depth)
    if os.path.exists(args.weights):
        model.load_state_dict(torch.load(args.weights, map_location=device))
        print(f"Loaded: {args.weights}")
    else:
        print("Using pretrained backbone (no fine-tuned weights)")
    model = model.to(device).eval()

    transform = T.Compose([
        T.Resize((480, 640)), T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])

    # Remap paths from host to container
    data_dir = os.environ.get("LILOCBENCH_DATA", "/data")
    for kf in keyframes:
        for key, info in kf["images"].items():
            p = info["path"]
            for marker in ["/data/", "/LILocBench/data/"]:
                idx = p.find(marker)
                if idx >= 0:
                    info["path"] = os.path.join(data_dir, p[idx + len(marker):])
                    break

    # Extract descriptors
    print(f"Extracting descriptors for {n} keyframes...")
    descs = []
    for i in range(0, n, 16):
        batch = []
        for j in range(i, min(i + 16, n)):
            img = Image.open(keyframes[j]["images"]["camera_front/color"]["path"]).convert("RGB")
            batch.append(transform(img))
        with torch.no_grad():
            d = model(torch.stack(batch).to(device)).cpu().numpy()
        descs.append(d)
    descs = np.concatenate(descs).astype(np.float32)
    norms = np.linalg.norm(descs, axis=1, keepdims=True)
    descs = descs / (norms + 1e-8)

    # Build index and evaluate
    import faiss
    index = faiss.IndexFlatL2(descs.shape[1])
    index.add(descs)
    D, I = index.search(descs, 60)

    EXCL_IDX = 5
    correct = {t: {k: 0 for k in [1, 5, 10]} for t in [2, 3, 5]}
    pos_errs = []

    for qi in range(n):
        filtered = [(I[qi][j], D[qi][j]) for j in range(60)
                    if abs(I[qi][j] - qi) > EXCL_IDX][:10]
        if not filtered:
            continue
        pos_errs.append(pose_dist(poses[qi], poses[filtered[0][0]]))
        for t in [2, 3, 5]:
            for k in [1, 5, 10]:
                if any(pose_dist(poses[qi], poses[idx]) < t for idx, _ in filtered[:k]):
                    correct[t][k] += 1

    pos_errs = np.array(pos_errs)
    print(f"\nMixVPR Retrieval ({descs.shape[1]}d):")
    print(f"  Top-1 error: mean={pos_errs.mean():.2f}m, med={np.median(pos_errs):.2f}m")
    print(f"  {'thresh':>6s}  {'R@1':>6s}  {'R@5':>6s}  {'R@10':>6s}")
    for t in [2, 3, 5]:
        vals = [f"{correct[t][k]/n*100:5.1f}%" for k in [1, 5, 10]]
        print(f"  {t:5.0f}m  {'  '.join(vals)}")

    # Cross-sequence evaluation
    gt_init = {
        "static_0":              (9.971633, -3.639337),
        "dynamics_0":            (0.359727, -7.046163),
        "lt_changes_0":         (-2.738933, -4.032920),
        "lt_changes_dynamics_0":(-1.639154, -3.787429),
    }

    print("\nCross-sequence candidate quality:")
    for seq, gt in gt_init.items():
        img_dir = os.path.join(DATA_BASE, seq, "camera_front", "color", "images")
        if not os.path.exists(img_dir):
            print(f"  {seq}: no images found, skipping")
            continue
        first_img = os.path.join(img_dir, sorted(os.listdir(img_dir))[0])
        img = Image.open(first_img).convert("RGB")
        with torch.no_grad():
            desc = model(transform(img).unsqueeze(0).to(device)).cpu().numpy()
        desc = desc / (np.linalg.norm(desc) + 1e-8)
        D_q, I_q = index.search(desc.astype(np.float32), 10)

        errors = [math.sqrt((poses[I_q[0][j]][0]-gt[0])**2 +
                            (poses[I_q[0][j]][1]-gt[1])**2) for j in range(10)]
        best3 = min(errors[:3])
        best10 = min(errors)
        print(f"  {seq}: top1={errors[0]:.2f}m, best3={best3:.2f}m, best10={best10:.2f}m")


if __name__ == "__main__":
    main()
