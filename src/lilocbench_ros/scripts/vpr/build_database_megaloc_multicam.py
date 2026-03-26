#!/usr/bin/env python3
"""
Build VPR database using MegaLoc descriptors from all 3 cameras.

Each keyframe produces 3 descriptors (front, left, right). The FAISS index
has 3x entries, and each entry maps back to the same (x, y, theta) pose.

Usage:
    docker compose run --rm vpr scripts/vpr/build_database_megaloc_multicam.py \
        --weights /models/vpr/megaloc_multicam_finetuned.pth
"""

import os
import sys
import pickle
import argparse
import numpy as np
import torch
import torchvision.transforms as T
from PIL import Image

MODELS_DIR = os.environ.get("LILOCBENCH_MODELS", "/models/vpr")

CAMERAS = ["camera_front", "camera_left", "camera_right"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--keyframes", type=str,
                        default=os.path.join(MODELS_DIR, "keyframes.pkl"))
    parser.add_argument("--weights", type=str, default="",
                        help="Path to fine-tuned weights")
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--output-db", type=str,
                        default=os.path.join(MODELS_DIR, "database_megaloc_multicam.pkl"))
    parser.add_argument("--output-index", type=str,
                        default=os.path.join(MODELS_DIR, "place_index_megaloc_multicam.faiss"))
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    with open(args.keyframes, "rb") as f:
        keyframes = pickle.load(f)
    print(f"Loaded {len(keyframes)} keyframes")

    # Remap paths
    data_dir = os.environ.get("LILOCBENCH_DATA", "/data")
    for kf in keyframes:
        for key, info in kf["images"].items():
            p = info["path"]
            for marker in ["/data/", "/LILocBench/data/"]:
                idx = p.find(marker)
                if idx >= 0:
                    info["path"] = os.path.join(data_dir, p[idx + len(marker):])
                    break

    # Load model
    print("Loading MegaLoc...")
    model = torch.hub.load("gmberton/MegaLoc", "get_trained_model", trust_repo=True)

    if args.weights and os.path.exists(args.weights):
        state = torch.load(args.weights, map_location=device)
        model.load_state_dict(state)
        print(f"Loaded fine-tuned weights: {args.weights}")
    else:
        print("Using pretrained weights (no fine-tuned weights)")

    model = model.to(device).eval()

    # Detect descriptor dimension
    with torch.no_grad():
        dummy = torch.randn(1, 3, 480, 640).to(device)
        desc_dim = model(dummy).shape[1]
    print(f"Descriptor dimension: {desc_dim}")

    transform = T.Compose([
        T.Resize((480, 640)), T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])

    # Extract descriptors for all cameras
    db_entries = []
    all_descs = []

    for cam_key in ["camera_front/color", "camera_left/color", "camera_right/color"]:
        cam_name = cam_key.split("/")[0]
        print(f"Extracting {cam_name} descriptors...")
        cam_descs = np.zeros((len(keyframes), desc_dim), dtype=np.float32)

        for i in range(0, len(keyframes), args.batch_size):
            batch = []
            for j in range(i, min(i + args.batch_size, len(keyframes))):
                img = Image.open(keyframes[j]["images"][cam_key]["path"]).convert("RGB")
                batch.append(transform(img))
            tensor = torch.stack(batch).to(device)
            with torch.no_grad():
                descs = model(tensor).cpu().numpy()
            cam_descs[i:i+len(batch)] = descs
            if (i + args.batch_size) % 100 < args.batch_size:
                print(f"  {min(i+args.batch_size, len(keyframes))}/{len(keyframes)}")

        # L2 normalise
        norms = np.linalg.norm(cam_descs, axis=1, keepdims=True)
        cam_descs = cam_descs / (norms + 1e-8)

        for i, kf in enumerate(keyframes):
            db_entries.append({
                "pose": kf["pose"],
                "camera": cam_name,
                "keyframe_idx": i,
            })
        all_descs.append(cam_descs)

    all_descs = np.vstack(all_descs)
    print(f"\nTotal database entries: {len(db_entries)} ({len(keyframes)} keyframes x {len(CAMERAS)} cameras)")

    # Build FAISS index
    import faiss
    index = faiss.IndexFlatL2(desc_dim)
    index.add(all_descs)
    faiss.write_index(index, args.output_index)
    print(f"Saved index to {args.output_index}")

    for i, entry in enumerate(db_entries):
        entry["descriptor"] = all_descs[i]

    with open(args.output_db, "wb") as f:
        pickle.dump(db_entries, f)
    print(f"Saved database to {args.output_db}")

    # Sanity check
    D, I = index.search(all_descs[:5], k=3)
    print(f"\nSanity (first 5 entries, top-3):")
    for i in range(5):
        entry = db_entries[i]
        matches = []
        for j in range(3):
            m = db_entries[I[i][j]]
            matches.append(f"kf{m['keyframe_idx']}:{m['camera']} L2={D[i][j]:.3f}")
        print(f"  kf{entry['keyframe_idx']}:{entry['camera']}: {matches}")


if __name__ == "__main__":
    main()
