#!/usr/bin/env python3
"""
Phase 0.3 + Phase 1: Build the place recognition database.

For each keyframe:
1. Extract CosPlace RGB descriptor (512-d) for each camera
2. Store descriptors + poses in a FAISS index

Output:
  - database.pkl  — keyframes with descriptors added
  - place_index.faiss — FAISS index over concatenated multi-camera descriptors
"""

import os
import pickle
import argparse
import numpy as np
import torch
import torchvision.transforms as T
from PIL import Image

GLOBAL_LOC_DIR = os.environ.get("LILOCBENCH_MODELS", "/home/matthew/Desktop/LILocBench/lilocbench_ws/models/vpr")


def load_model(backbone="ResNet18", fc_output_dim=512):
    """Load pretrained CosPlace model."""
    model = torch.hub.load(
        "gmberton/cosplace", "get_trained_model",
        backbone=backbone, fc_output_dim=fc_output_dim,
        trust_repo=True,
    )
    model.eval()
    return model


def get_transform():
    """CosPlace standard preprocessing."""
    return T.Compose([
        T.Resize((480, 640)),  # keep native resolution
        T.ToTensor(),
        T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])


def extract_descriptor(model, image_path, transform, device):
    """Extract a single descriptor from an image."""
    img = Image.open(image_path).convert("RGB")
    tensor = transform(img).unsqueeze(0).to(device)
    with torch.no_grad():
        desc = model(tensor)
    return desc.squeeze().cpu().numpy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--keyframes", type=str,
                        default=os.path.join(GLOBAL_LOC_DIR, "keyframes.pkl"))
    parser.add_argument("--output-db", type=str,
                        default=os.path.join(GLOBAL_LOC_DIR, "database.pkl"))
    parser.add_argument("--output-index", type=str,
                        default=os.path.join(GLOBAL_LOC_DIR, "place_index.faiss"))
    parser.add_argument("--cameras", type=str, nargs="+",
                        default=["camera_front", "camera_left", "camera_right"])
    parser.add_argument("--backbone", type=str, default="ResNet18")
    parser.add_argument("--fc-dim", type=int, default=512)
    parser.add_argument("--batch-size", type=int, default=32)
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    # Load keyframes
    with open(args.keyframes, "rb") as f:
        keyframes = pickle.load(f)
    print(f"Loaded {len(keyframes)} keyframes")

    # Load model
    print(f"Loading CosPlace ({args.backbone}, {args.fc_dim}d) ...")
    model = load_model(args.backbone, args.fc_dim).to(device)
    transform = get_transform()

    # Extract descriptors for each camera
    n_cams = len(args.cameras)
    desc_dim = args.fc_dim
    all_descriptors = np.zeros((len(keyframes), n_cams * desc_dim), dtype=np.float32)

    for cam_idx, cam in enumerate(args.cameras):
        print(f"\nExtracting descriptors for {cam} ...")
        cam_key = f"{cam}/color"

        # Batch extraction
        batch_paths = []
        batch_indices = []
        for kf_idx, kf in enumerate(keyframes):
            path = kf["images"][cam_key]["path"]
            batch_paths.append(path)
            batch_indices.append(kf_idx)

            if len(batch_paths) == args.batch_size or kf_idx == len(keyframes) - 1:
                # Process batch
                tensors = []
                for p in batch_paths:
                    img = Image.open(p).convert("RGB")
                    tensors.append(transform(img))
                batch_tensor = torch.stack(tensors).to(device)

                with torch.no_grad():
                    descs = model(batch_tensor).cpu().numpy()

                for i, bi in enumerate(batch_indices):
                    start = cam_idx * desc_dim
                    all_descriptors[bi, start:start + desc_dim] = descs[i]

                if (kf_idx + 1) % 100 == 0 or kf_idx == len(keyframes) - 1:
                    print(f"  {kf_idx + 1}/{len(keyframes)}")

                batch_paths = []
                batch_indices = []

    # L2-normalize the full concatenated descriptor
    norms = np.linalg.norm(all_descriptors, axis=1, keepdims=True)
    norms[norms == 0] = 1.0
    all_descriptors = all_descriptors / norms

    # Store descriptors in keyframes
    for i, kf in enumerate(keyframes):
        kf["descriptor"] = all_descriptors[i]

    # Build FAISS index
    print(f"\nBuilding FAISS index ({all_descriptors.shape}) ...")
    import faiss
    index = faiss.IndexFlatL2(all_descriptors.shape[1])
    index.add(all_descriptors)
    faiss.write_index(index, args.output_index)
    print(f"  Saved index to {args.output_index}")

    # Save database
    with open(args.output_db, "wb") as f:
        pickle.dump(keyframes, f)
    print(f"  Saved database to {args.output_db}")

    # Quick sanity check: query each keyframe against itself
    D, I = index.search(all_descriptors[:5], k=3)
    print(f"\nSanity check (first 5 keyframes, top-3 matches):")
    for i in range(5):
        matches = [(I[i][j], D[i][j]) for j in range(3)]
        print(f"  KF {i}: matches={matches}")


if __name__ == "__main__":
    main()
