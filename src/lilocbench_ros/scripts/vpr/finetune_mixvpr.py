#!/usr/bin/env python3
"""
Fine-tune MixVPR (ResNet18 backbone) on the mapping sequence.

Uses ResNet18 + MixVPR aggregator (~12M params total) which fits
comfortably in 8 GB VRAM for full backbone + aggregator training.

Triplet loss with hard negative mining, same approach as EigenPlaces/MegaLoc.
"""

import os
import sys
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
MIXVPR_REPO = os.environ.get("MIXVPR_REPO", "/workspace/mixvpr_repo")


class MixVPRModel(nn.Module):
    """Wrapper that combines ResNet backbone + MixVPR aggregator."""

    def __init__(self, backbone_arch="resnet18", layers_to_freeze=0,
                 out_channels=256, out_rows=4, mix_depth=4):
        super().__init__()
        sys.path.insert(0, MIXVPR_REPO)
        from models.helper import get_backbone, get_aggregator

        # ResNet18 with layer4 cropped → layer3 output: 256ch
        # ResNet50 with layer4 cropped → layer3 output: 1024ch
        self.backbone = get_backbone(
            backbone_arch, pretrained=True,
            layers_to_freeze=layers_to_freeze, layers_to_crop=[4])

        # Determine spatial size from a test forward pass
        with torch.no_grad():
            test = torch.randn(1, 3, 480, 640)
            feat = self.backbone(test)
            in_channels = feat.shape[1]
            in_h = feat.shape[2]
            in_w = feat.shape[3]

        self.aggregator = get_aggregator("mixvpr", {
            "in_channels": in_channels,
            "in_h": in_h,
            "in_w": in_w,
            "out_channels": out_channels,
            "out_rows": out_rows,
            "mix_depth": mix_depth,
            "mlp_ratio": 1,
        })

        self.out_dim = out_channels * out_rows
        print(f"MixVPR: {backbone_arch} → {in_channels}ch × {in_h}×{in_w} → "
              f"MixVPR({out_channels}ch, {out_rows}rows, depth={mix_depth}) → {self.out_dim}-d")

    def forward(self, x):
        x = self.backbone(x)
        x = self.aggregator(x)
        return x


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
        print(f"  {camera}: {len(self.images)} images (subsampled {subsample}x)")

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
    parser = argparse.ArgumentParser(description="Fine-tune MixVPR on mapping data")
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--lr", type=float, default=5e-5)
    parser.add_argument("--margin", type=float, default=0.3)
    parser.add_argument("--pos-thresh", type=float, default=2.0)
    parser.add_argument("--neg-thresh", type=float, default=10.0)
    parser.add_argument("--subsample", type=int, default=5)
    parser.add_argument("--camera", type=str, default="camera_front")
    parser.add_argument("--backbone", type=str, default="resnet18",
                        choices=["resnet18", "resnet50"])
    parser.add_argument("--out-channels", type=int, default=256)
    parser.add_argument("--out-rows", type=int, default=4)
    parser.add_argument("--mix-depth", type=int, default=4)
    parser.add_argument("--layers-to-freeze", type=int, default=0,
                        help="Backbone layers to freeze (0=train all)")
    parser.add_argument("--output", type=str,
                        default=os.path.join(MODELS_DIR, "mixvpr_finetuned.pth"))
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    # Build model
    model = MixVPRModel(
        backbone_arch=args.backbone,
        layers_to_freeze=args.layers_to_freeze,
        out_channels=args.out_channels,
        out_rows=args.out_rows,
        mix_depth=args.mix_depth,
    ).to(device)

    total_params = sum(p.numel() for p in model.parameters())
    trainable = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"Total params: {total_params/1e6:.1f}M, trainable: {trainable/1e6:.1f}M")
    print(f"Descriptor dimension: {model.out_dim}")

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

    optimizer = optim.Adam(model.parameters(), lr=args.lr)
    triplet_loss = nn.TripletMarginLoss(margin=args.margin, p=2)

    print(f"\nTraining {args.epochs} epochs, bs={args.batch_size}, lr={args.lr}")
    best_loss = float("inf")

    for epoch in range(args.epochs):
        model.train()
        epoch_loss, n_triplets = 0.0, 0

        for imgs, poses, indices in loader:
            imgs = imgs.to(device)
            poses = poses.to(device)
            indices = indices.to(device).float()

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
    print(f"Descriptor dim: {model.out_dim}")


if __name__ == "__main__":
    main()
