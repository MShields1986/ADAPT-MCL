#!/usr/bin/env python3
"""
Fine-tune EigenPlaces on the mapping sequence using all 3 cameras.

Images from all cameras at the same pose are treated as views of the same location.
This teaches the model view-invariant features and 3x the training data.

Triplet loss with hard negative mining — positives can be from any camera at the
same pose, negatives are from any camera at a distant pose.
"""

import os
import math
import bisect
import argparse
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torchvision.transforms as T
from PIL import Image
from torch.utils.data import Dataset, DataLoader

DATA_DIR = os.environ.get("LILOCBENCH_DATA", "/home/matthew/Desktop/LILocBench/data") + "/mapping"
MODELS_DIR = os.environ.get("LILOCBENCH_MODELS", "/home/matthew/Desktop/LILocBench/lilocbench_ws/models/vpr")

CAMERAS = ["camera_front", "camera_left", "camera_right"]


def load_gt_poses(path):
    data = np.loadtxt(path)
    ts = data[:, 0]
    x, y = data[:, 1], data[:, 2]
    qz, qw = data[:, 6], data[:, 7]
    theta = 2.0 * np.arctan2(qz, qw)
    return ts, x, y, theta


def interpolate_pose(gt_ts, gt_x, gt_y, query_t):
    idx = bisect.bisect_right(gt_ts, query_t) - 1
    idx = max(0, min(idx, len(gt_ts) - 2))
    t0, t1 = gt_ts[idx], gt_ts[idx + 1]
    alpha = max(0.0, min(1.0, (query_t - t0) / (t1 - t0 + 1e-10)))
    return gt_x[idx] + alpha * (gt_x[idx + 1] - gt_x[idx]), \
           gt_y[idx] + alpha * (gt_y[idx + 1] - gt_y[idx])


class MultiCamTrajectoryDataset(Dataset):
    """Dataset that loads images from all 3 cameras along the trajectory.

    Each item has a pose (x, y) and a camera index. Images from different
    cameras at the same trajectory position share the same pose, making
    them valid positive pairs for triplet mining.
    """

    def __init__(self, data_dir, cameras, gt_path, transform, subsample=5):
        self.transform = transform
        gt_ts, gt_x, gt_y, _ = load_gt_poses(gt_path)

        front_dir = os.path.join(data_dir, "camera_front", "color", "images")
        front_files = sorted(os.listdir(front_dir))

        cam_files = {}
        for cam in cameras:
            img_dir = os.path.join(data_dir, cam, "color", "images")
            files = sorted(os.listdir(img_dir))
            cam_files[cam] = {float(f.replace(".png", "")): f for f in files if f.endswith(".png")}

        self.images = []
        self.poses = []
        self.cam_indices = []

        for i, fname in enumerate(front_files):
            if i % subsample != 0:
                continue
            t_front = float(fname.replace(".png", ""))
            if t_front < gt_ts[0] or t_front > gt_ts[-1]:
                continue

            x, y = interpolate_pose(gt_ts, gt_x, gt_y, t_front)

            self.images.append(os.path.join(front_dir, fname))
            self.poses.append((x, y))
            self.cam_indices.append(0)

            for cam_idx, cam in enumerate(cameras[1:], 1):
                img_dir = os.path.join(data_dir, cam, "color", "images")
                best_f, best_dt = None, 0.05
                for t_cam, f_cam in cam_files[cam].items():
                    dt = abs(t_cam - t_front)
                    if dt < best_dt:
                        best_dt = dt
                        best_f = f_cam
                if best_f is not None:
                    self.images.append(os.path.join(img_dir, best_f))
                    self.poses.append((x, y))
                    self.cam_indices.append(cam_idx)

        self.poses = np.array(self.poses, dtype=np.float32)
        self.cam_indices = np.array(self.cam_indices, dtype=np.int64)

        n_per_cam = [(self.cam_indices == i).sum() for i in range(len(cameras))]
        print(f"  Multi-cam dataset: {len(self.images)} total images")
        for i, cam in enumerate(cameras):
            print(f"    {cam}: {n_per_cam[i]}")

    def __len__(self):
        return len(self.images)

    def __getitem__(self, idx):
        img = Image.open(self.images[idx]).convert("RGB")
        return self.transform(img), self.poses[idx], idx


def mine_triplets(embeddings, poses, indices, pos_thresh=2.0, neg_thresh=10.0):
    """Mine hard triplets. Cross-camera positives at same pose allowed."""
    n = len(embeddings)
    pose_dists = torch.cdist(poses, poses)
    emb_dists = torch.cdist(embeddings, embeddings)

    anchors, positives, negatives = [], [], []
    for i in range(n):
        pos_mask = (pose_dists[i] < pos_thresh) & (pose_dists[i] > 0.01)
        neg_mask = pose_dists[i] > neg_thresh

        if not pos_mask.any() or not neg_mask.any():
            continue

        pe = emb_dists[i].clone()
        pe[~pos_mask] = -1
        ne = emb_dists[i].clone()
        ne[~neg_mask] = float("inf")

        anchors.append(i)
        positives.append(pe.argmax().item())
        negatives.append(ne.argmin().item())

    return anchors, positives, negatives


def main():
    parser = argparse.ArgumentParser(description="Fine-tune EigenPlaces (multi-camera)")
    parser.add_argument("--epochs", type=int, default=15)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--lr", type=float, default=1e-5)
    parser.add_argument("--margin", type=float, default=0.3)
    parser.add_argument("--pos-thresh", type=float, default=2.0)
    parser.add_argument("--neg-thresh", type=float, default=10.0)
    parser.add_argument("--subsample", type=int, default=5)
    parser.add_argument("--freeze-backbone", action="store_true", default=True)
    parser.add_argument("--no-freeze-backbone", dest="freeze_backbone",
                        action="store_false")
    parser.add_argument("--output", type=str,
                        default=os.path.join(MODELS_DIR, "eigenplaces_multicam_finetuned.pth"))
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    print("Loading pretrained EigenPlaces...")
    model = torch.hub.load(
        "gmberton/eigenplaces", "get_trained_model",
        backbone="ResNet50", fc_output_dim=512, trust_repo=True,
    ).to(device)

    if args.freeze_backbone:
        frozen = 0
        for name, param in model.named_parameters():
            if "aggregation" not in name and "fc" not in name:
                param.requires_grad = False
                frozen += 1
        trainable = sum(p.numel() for p in model.parameters() if p.requires_grad)
        total = sum(p.numel() for p in model.parameters())
        print(f"Total params: {total/1e6:.1f}M, trainable: {trainable/1e6:.1f}M (aggregation only)")
    else:
        trainable = sum(p.numel() for p in model.parameters() if p.requires_grad)
        print(f"All {trainable/1e6:.1f}M params trainable")

    transform = T.Compose([
        T.Resize((480, 640)),
        T.ColorJitter(brightness=0.3, contrast=0.3, saturation=0.2, hue=0.1),
        T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])

    gt_path = os.path.join(DATA_DIR, "gt_poses.txt")
    print("Loading multi-camera dataset...")
    dataset = MultiCamTrajectoryDataset(DATA_DIR, CAMERAS, gt_path, transform,
                                         subsample=args.subsample)
    loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=True,
                        num_workers=4, pin_memory=True, drop_last=True)

    optimizer = optim.Adam(filter(lambda p: p.requires_grad, model.parameters()),
                           lr=args.lr)
    triplet_loss = nn.TripletMarginLoss(margin=args.margin, p=2)

    print(f"\nTraining {args.epochs} epochs, bs={args.batch_size}, lr={args.lr}")
    best_loss = float("inf")

    for epoch in range(args.epochs):
        model.train()
        epoch_loss, n_triplets = 0.0, 0

        for imgs, poses, indices in loader:
            imgs = imgs.to(device)
            poses = poses.to(device)

            emb = model(imgs)
            emb = F.normalize(emb, p=2, dim=1)

            a, p, ng = mine_triplets(emb.detach(), poses, indices,
                                     pos_thresh=args.pos_thresh,
                                     neg_thresh=args.neg_thresh)
            if not a:
                continue

            loss = triplet_loss(emb[a], emb[p], emb[ng])
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            epoch_loss += loss.item() * len(a)
            n_triplets += len(a)

        avg = epoch_loss / max(n_triplets, 1)
        print(f"  Epoch {epoch+1}/{args.epochs}: loss={avg:.4f} ({n_triplets} triplets)")
        if avg < best_loss and n_triplets > 0:
            best_loss = avg
            torch.save(model.state_dict(), args.output)
            print(f"    -> Saved (loss={best_loss:.4f})")

    print(f"\nDone. Best loss: {best_loss:.4f}")
    print(f"Model saved to {args.output}")


if __name__ == "__main__":
    main()
