#!/usr/bin/env python3
"""
Fine-tune EigenPlaces on the mapping sequence for indoor place recognition.

Uses triplet loss with hard negative mining:
- Anchor: random image from trajectory
- Positive: image from within pos_thresh metres (different timestamp)
- Negative: hardest negative from batch (closest descriptor but >neg_thresh metres)

Only fine-tunes the last few layers (aggregation head) to avoid catastrophic
forgetting of the pretrained features.
"""

import os
import math
import bisect
import random
import argparse
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torchvision.transforms as T
from PIL import Image
from torch.utils.data import Dataset, DataLoader

DATA_DIR = os.environ.get("LILOCBENCH_DATA", "/home/matthew/Desktop/LILocBench/data") + "/mapping"
GLOBAL_LOC_DIR = os.environ.get("LILOCBENCH_MODELS", "/home/matthew/Desktop/LILocBench/lilocbench_ws/models/vpr")


def load_gt_poses(path):
    """Load GT poses, return (timestamps, x, y, theta)."""
    data = np.loadtxt(path)
    ts = data[:, 0]
    x, y = data[:, 1], data[:, 2]
    qz, qw = data[:, 6], data[:, 7]
    theta = 2.0 * np.arctan2(qz, qw)
    return ts, x, y, theta


def interpolate_pose(gt_ts, gt_x, gt_y, query_t):
    """Interpolate (x, y) at query_t."""
    idx = bisect.bisect_right(gt_ts, query_t) - 1
    idx = max(0, min(idx, len(gt_ts) - 2))
    t0, t1 = gt_ts[idx], gt_ts[idx + 1]
    alpha = (query_t - t0) / (t1 - t0 + 1e-10)
    alpha = max(0.0, min(1.0, alpha))
    x = gt_x[idx] + alpha * (gt_x[idx + 1] - gt_x[idx])
    y = gt_y[idx] + alpha * (gt_y[idx + 1] - gt_y[idx])
    return x, y


class TrajectoryDataset(Dataset):
    """Dataset of images with poses from the mapping trajectory.

    Subsamples to every Nth image for manageable size.
    """

    def __init__(self, data_dir, camera, gt_path, transform, subsample=5):
        self.transform = transform
        img_dir = os.path.join(data_dir, camera, "color", "images")
        all_files = sorted(os.listdir(img_dir))

        gt_ts, gt_x, gt_y, _ = load_gt_poses(gt_path)

        self.images = []
        self.poses = []
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
        print(f"  {camera}: {len(self.images)} images (subsampled {subsample}x)")

    def __len__(self):
        return len(self.images)

    def __getitem__(self, idx):
        img = Image.open(self.images[idx]).convert("RGB")
        tensor = self.transform(img)
        return tensor, self.poses[idx], idx


class TripletSampler:
    """Generates triplets with hard negative mining from a batch."""

    def __init__(self, pos_thresh=2.0, neg_thresh=10.0, min_time_gap=3):
        self.pos_thresh = pos_thresh
        self.neg_thresh = neg_thresh
        self.min_time_gap = min_time_gap

    def mine_triplets(self, embeddings, poses, indices):
        """Find hard triplets within a batch.

        Returns: (anchor_idx, positive_idx, negative_idx) lists into the batch.
        """
        n = len(embeddings)
        # Pairwise distances in pose space
        pose_dists = torch.cdist(poses, poses)
        # Pairwise distances in embedding space
        emb_dists = torch.cdist(embeddings, embeddings)

        # Index gap (temporal distance in dataset ordering)
        idx_gap = torch.abs(indices.unsqueeze(0) - indices.unsqueeze(1))

        anchors, positives, negatives = [], [], []
        for i in range(n):
            # Valid positives: close in space, far enough in time
            pos_mask = (pose_dists[i] < self.pos_thresh) & \
                       (pose_dists[i] > 0.1) & \
                       (idx_gap[i] >= self.min_time_gap)
            if not pos_mask.any():
                continue

            # Valid negatives: far in space
            neg_mask = pose_dists[i] > self.neg_thresh
            if not neg_mask.any():
                continue

            # Pick hardest positive (farthest in embedding space among positives)
            pos_emb_dists = emb_dists[i].clone()
            pos_emb_dists[~pos_mask] = -1
            hard_pos = pos_emb_dists.argmax()

            # Pick hardest negative (closest in embedding space among negatives)
            neg_emb_dists = emb_dists[i].clone()
            neg_emb_dists[~neg_mask] = float("inf")
            hard_neg = neg_emb_dists.argmin()

            anchors.append(i)
            positives.append(hard_pos.item())
            negatives.append(hard_neg.item())

        return anchors, positives, negatives


def load_model(backbone="ResNet50", fc_output_dim=512):
    """Load pretrained EigenPlaces model."""
    model = torch.hub.load(
        "gmberton/eigenplaces", "get_trained_model",
        backbone=backbone, fc_output_dim=fc_output_dim,
        trust_repo=True,
    )
    return model


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--epochs", type=int, default=10)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-5)
    parser.add_argument("--margin", type=float, default=0.3,
                        help="Triplet loss margin")
    parser.add_argument("--pos-thresh", type=float, default=2.0,
                        help="Max distance for positive pair (m)")
    parser.add_argument("--neg-thresh", type=float, default=10.0,
                        help="Min distance for negative pair (m)")
    parser.add_argument("--subsample", type=int, default=5,
                        help="Use every Nth image (5 = ~2400 images)")
    parser.add_argument("--camera", type=str, default="camera_front")
    parser.add_argument("--freeze-backbone", action="store_true", default=True,
                        help="Freeze ResNet backbone, only train aggregation")
    parser.add_argument("--no-freeze-backbone", dest="freeze_backbone",
                        action="store_false")
    parser.add_argument("--output", type=str,
                        default=os.path.join(GLOBAL_LOC_DIR, "eigenplaces_finetuned.pth"))
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    # Load model
    print("Loading pretrained EigenPlaces...")
    model = load_model().to(device)

    # Freeze backbone if requested (only train aggregation head)
    if args.freeze_backbone:
        frozen = 0
        for name, param in model.named_parameters():
            if "aggregation" not in name and "fc" not in name:
                param.requires_grad = False
                frozen += 1
        trainable = sum(1 for p in model.parameters() if p.requires_grad)
        print(f"  Frozen {frozen} params, {trainable} trainable (aggregation only)")
    else:
        # Unfreeze last few ResNet layers + aggregation
        trainable = sum(1 for p in model.parameters() if p.requires_grad)
        print(f"  All {trainable} params trainable")

    # Data
    transform = T.Compose([
        T.Resize((480, 640)),
        T.ColorJitter(brightness=0.3, contrast=0.3, saturation=0.2, hue=0.1),
        T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])

    gt_path = os.path.join(DATA_DIR, "gt_poses.txt")
    print("Loading dataset...")
    dataset = TrajectoryDataset(DATA_DIR, args.camera, gt_path, transform,
                                subsample=args.subsample)

    loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=True,
                        num_workers=4, pin_memory=True, drop_last=True)

    # Training
    optimizer = optim.Adam(filter(lambda p: p.requires_grad, model.parameters()),
                           lr=args.lr)
    triplet_loss = nn.TripletMarginLoss(margin=args.margin, p=2)
    sampler = TripletSampler(pos_thresh=args.pos_thresh, neg_thresh=args.neg_thresh)

    print(f"\nTraining for {args.epochs} epochs, batch_size={args.batch_size}")
    print(f"  pos_thresh={args.pos_thresh}m, neg_thresh={args.neg_thresh}m, margin={args.margin}")

    best_loss = float("inf")
    for epoch in range(args.epochs):
        model.train()
        epoch_loss = 0.0
        n_triplets = 0
        n_batches = 0

        for batch_imgs, batch_poses, batch_indices in loader:
            batch_imgs = batch_imgs.to(device)
            batch_poses = batch_poses.to(device)
            batch_indices = batch_indices.to(device).float()

            # Forward pass
            with torch.no_grad() if args.freeze_backbone else torch.enable_grad():
                embeddings = model(batch_imgs)

            # If backbone is frozen, we need to recompute with grad for aggregation
            if args.freeze_backbone:
                embeddings = model(batch_imgs)

            # L2 normalize
            embeddings = nn.functional.normalize(embeddings, p=2, dim=1)

            # Mine hard triplets
            anchors, positives, negatives = sampler.mine_triplets(
                embeddings.detach(), batch_poses, batch_indices)

            if len(anchors) == 0:
                n_batches += 1
                continue

            # Compute loss
            anchor_emb = embeddings[anchors]
            pos_emb = embeddings[positives]
            neg_emb = embeddings[negatives]

            loss = triplet_loss(anchor_emb, pos_emb, neg_emb)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            epoch_loss += loss.item() * len(anchors)
            n_triplets += len(anchors)
            n_batches += 1

        avg_loss = epoch_loss / max(n_triplets, 1)
        print(f"  Epoch {epoch+1}/{args.epochs}: loss={avg_loss:.4f} "
              f"({n_triplets} triplets from {n_batches} batches)")

        if avg_loss < best_loss:
            best_loss = avg_loss
            torch.save(model.state_dict(), args.output)
            print(f"    -> Saved best model (loss={best_loss:.4f})")

    # Final save
    torch.save(model.state_dict(), args.output)
    print(f"\nFinal model saved to {args.output}")


if __name__ == "__main__":
    main()
