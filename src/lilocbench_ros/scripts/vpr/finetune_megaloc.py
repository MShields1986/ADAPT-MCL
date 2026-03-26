#!/usr/bin/env python3
"""
Fine-tune MegaLoc on the mapping sequence.
Freezes the DINOv2 backbone, only trains the aggregator.
Uses mixed precision to fit in 8GB GPU.
"""

import os
import math
import bisect
import argparse
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torchvision.transforms as T
from PIL import Image
from torch.utils.data import Dataset, DataLoader
from torch.cuda.amp import autocast, GradScaler

DATA_DIR = os.environ.get("LILOCBENCH_DATA", "/home/matthew/Desktop/LILocBench/data") + "/mapping"
GLOBAL_LOC_DIR = os.environ.get("LILOCBENCH_MODELS", "/home/matthew/Desktop/LILocBench/lilocbench_ws/models/vpr")


def load_gt_poses(path):
    data = np.loadtxt(path)
    return data[:, 0], data[:, 1], data[:, 2]


def interpolate_pose(gt_ts, gt_x, gt_y, query_t):
    idx = bisect.bisect_right(gt_ts, query_t) - 1
    idx = max(0, min(idx, len(gt_ts) - 2))
    t0, t1 = gt_ts[idx], gt_ts[idx + 1]
    alpha = max(0.0, min(1.0, (query_t - t0) / (t1 - t0 + 1e-10)))
    return gt_x[idx] + alpha * (gt_x[idx + 1] - gt_x[idx]), \
           gt_y[idx] + alpha * (gt_y[idx + 1] - gt_y[idx])


class TrajectoryDataset(Dataset):
    def __init__(self, data_dir, camera, gt_path, transform, subsample=5):
        self.transform = transform
        img_dir = os.path.join(data_dir, camera, "color", "images")
        all_files = sorted(os.listdir(img_dir))
        gt_ts, gt_x, gt_y = load_gt_poses(gt_path)
        self.images, self.poses = [], []
        for i, fname in enumerate(all_files):
            if i % subsample != 0:
                continue
            t = float(fname.replace(".png", ""))
            if t < gt_ts[0] or t > gt_ts[-1]:
                continue
            x, y = interpolate_pose(gt_ts, gt_x, gt_y, t)
            self.images.append(os.path.join(img_dir, fname))
            self.poses.append((x, y))
        self.poses = np.array(self.poses, dtype=np.float32)
        print(f"  {camera}: {len(self.images)} images")

    def __len__(self):
        return len(self.images)

    def __getitem__(self, idx):
        img = Image.open(self.images[idx]).convert("RGB")
        return self.transform(img), self.poses[idx], idx


def mine_triplets(embeddings, poses, indices, pos_thresh=2.0, neg_thresh=10.0, min_gap=3):
    n = len(embeddings)
    pose_dists = torch.cdist(poses, poses)
    emb_dists = torch.cdist(embeddings, embeddings)
    idx_gap = torch.abs(indices.unsqueeze(0) - indices.unsqueeze(1))
    anchors, positives, negatives = [], [], []
    for i in range(n):
        pos_mask = (pose_dists[i] < pos_thresh) & (pose_dists[i] > 0.1) & (idx_gap[i] >= min_gap)
        neg_mask = pose_dists[i] > neg_thresh
        if not pos_mask.any() or not neg_mask.any():
            continue
        pe = emb_dists[i].clone(); pe[~pos_mask] = -1
        ne = emb_dists[i].clone(); ne[~neg_mask] = float("inf")
        anchors.append(i)
        positives.append(pe.argmax().item())
        negatives.append(ne.argmin().item())
    return anchors, positives, negatives


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--epochs", type=int, default=15)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--lr", type=float, default=1e-5)
    parser.add_argument("--margin", type=float, default=0.3)
    parser.add_argument("--subsample", type=int, default=5)
    parser.add_argument("--output", type=str,
                        default=os.path.join(GLOBAL_LOC_DIR, "megaloc_finetuned.pth"))
    args = parser.parse_args()

    device = torch.device("cuda")
    model = torch.hub.load("gmberton/MegaLoc", "get_trained_model", trust_repo=True).to(device)

    # Freeze backbone
    for name, param in model.named_parameters():
        if name.startswith("backbone"):
            param.requires_grad = False
    trainable = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"Trainable params: {trainable/1e6:.1f}M (aggregator only)")

    transform = T.Compose([
        T.Resize((480, 640)),
        T.ColorJitter(brightness=0.3, contrast=0.3, saturation=0.2, hue=0.1),
        T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])

    dataset = TrajectoryDataset(DATA_DIR, "camera_front",
                                os.path.join(DATA_DIR, "gt_poses.txt"),
                                transform, subsample=args.subsample)
    loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=True,
                        num_workers=4, pin_memory=True, drop_last=True)

    optimizer = optim.Adam(filter(lambda p: p.requires_grad, model.parameters()), lr=args.lr)
    triplet_loss = nn.TripletMarginLoss(margin=args.margin, p=2)
    scaler = GradScaler()

    print(f"\nTraining {args.epochs} epochs, bs={args.batch_size}, mixed precision")
    best_loss = float("inf")

    for epoch in range(args.epochs):
        model.train()
        epoch_loss, n_triplets = 0.0, 0

        for imgs, poses, indices in loader:
            imgs = imgs.to(device)
            poses = poses.to(device)
            indices = indices.to(device).float()

            with autocast():
                emb = model(imgs)
                emb = nn.functional.normalize(emb, p=2, dim=1)

            a, p, ng = mine_triplets(emb.detach().float(), poses, indices)
            if not a:
                continue

            with autocast():
                loss = triplet_loss(emb[a], emb[p], emb[ng])

            optimizer.zero_grad()
            scaler.scale(loss).backward()
            scaler.step(optimizer)
            scaler.update()

            epoch_loss += loss.item() * len(a)
            n_triplets += len(a)

        avg = epoch_loss / max(n_triplets, 1)
        print(f"  Epoch {epoch+1}/{args.epochs}: loss={avg:.4f} ({n_triplets} triplets)")
        if avg < best_loss:
            best_loss = avg
            torch.save(model.state_dict(), args.output)
            print(f"    -> Saved (loss={best_loss:.4f})")

    print(f"\nDone. Best loss: {best_loss:.4f}")


if __name__ == "__main__":
    main()
