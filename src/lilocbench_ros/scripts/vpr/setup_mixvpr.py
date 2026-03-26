#!/usr/bin/env python3
"""
Setup script for MixVPR model.

Clones the MixVPR repository and downloads pretrained weights.
Run once inside the VPR Docker container:
    python scripts/vpr/setup_mixvpr.py

Pretrained configs available (all ResNet50 backbone):
  - 512-d:  out_channels=512,  out_rows=2, mix_depth=4 (smallest)
  - 4096-d: out_channels=1024, out_rows=4, mix_depth=4 (default, best)

We use ResNet18 backbone for fine-tuning (smaller, fits full training in 8GB).
"""

import os
import subprocess
import sys

MODELS_DIR = os.environ.get("LILOCBENCH_MODELS", "/models/vpr")
MIXVPR_DIR = os.path.join(MODELS_DIR, "mixvpr_repo")


def clone_repo():
    """Clone MixVPR repo for model definitions."""
    if os.path.exists(os.path.join(MIXVPR_DIR, "models")):
        print(f"MixVPR repo already exists at {MIXVPR_DIR}")
        return

    print(f"Cloning MixVPR to {MIXVPR_DIR}...")
    subprocess.run(
        ["git", "clone", "--depth=1",
         "https://github.com/amaralibey/MixVPR.git", MIXVPR_DIR],
        check=True,
    )
    print("Done.")


def download_pretrained_weights():
    """Download pretrained ResNet50+MixVPR-4096 weights from Google Drive."""
    weights_path = os.path.join(MODELS_DIR, "mixvpr_resnet50_4096_pretrained.ckpt")
    if os.path.exists(weights_path):
        print(f"Pretrained weights already exist at {weights_path}")
        return weights_path

    # Google Drive file ID for resnet50_MixVPR_4096_channels(1024)_rows(4).ckpt
    file_id = "1DQnefjk1hVICOEYPwE4-CZAZOvi1NSJz"

    try:
        import gdown
    except ImportError:
        subprocess.run([sys.executable, "-m", "pip", "install", "gdown"], check=True)
        import gdown

    print(f"Downloading pretrained weights to {weights_path}...")
    gdown.download(id=file_id, output=weights_path, quiet=False)
    print("Done.")
    return weights_path


def verify_model():
    """Verify model can be constructed and loaded."""
    sys.path.insert(0, MIXVPR_DIR)
    from models.helper import get_backbone, get_aggregator
    import torch

    # Build ResNet18 + MixVPR (what we'll use for fine-tuning)
    backbone = get_backbone("resnet18", pretrained=True, layers_to_freeze=0, layers_to_crop=[4])
    # ResNet18 layer3 output: 256 channels, spatial size depends on input
    # With 480x640 input and layers_to_crop=[4]: layer3 output is 256 x 30 x 40
    aggregator = get_aggregator("mixvpr", {
        "in_channels": 256,
        "in_h": 30,
        "in_w": 40,
        "out_channels": 256,
        "out_rows": 4,
        "mix_depth": 4,
        "mlp_ratio": 1,
    })

    # Test forward pass
    x = torch.randn(1, 3, 480, 640)
    with torch.no_grad():
        features = backbone(x)
        print(f"Backbone output shape: {features.shape}")
        desc = aggregator(features)
        print(f"Descriptor shape: {desc.shape}")  # should be (1, 1024)

    total_params = sum(p.numel() for p in backbone.parameters()) + \
                   sum(p.numel() for p in aggregator.parameters())
    trainable = sum(p.numel() for p in backbone.parameters() if p.requires_grad) + \
                sum(p.numel() for p in aggregator.parameters() if p.requires_grad)
    print(f"Total params: {total_params/1e6:.1f}M, trainable: {trainable/1e6:.1f}M")

    # Also test ResNet50 with pretrained weights if available
    weights_path = os.path.join(MODELS_DIR, "mixvpr_resnet50_4096_pretrained.ckpt")
    if os.path.exists(weights_path):
        print(f"\nVerifying pretrained ResNet50 weights...")
        backbone50 = get_backbone("resnet50", pretrained=True, layers_to_freeze=2, layers_to_crop=[4])
        aggregator50 = get_aggregator("mixvpr", {
            "in_channels": 1024,
            "in_h": 30,
            "in_w": 40,
            "out_channels": 1024,
            "out_rows": 4,
            "mix_depth": 4,
            "mlp_ratio": 1,
        })

        # Load checkpoint (PyTorch Lightning format)
        ckpt = torch.load(weights_path, map_location="cpu")
        state_dict = ckpt.get("state_dict", ckpt)
        # Strip 'backbone.' and 'aggregator.' prefixes if present
        bb_state = {k.replace("backbone.", ""): v for k, v in state_dict.items() if k.startswith("backbone.")}
        agg_state = {k.replace("aggregator.", ""): v for k, v in state_dict.items() if k.startswith("aggregator.")}
        backbone50.load_state_dict(bb_state, strict=False)
        aggregator50.load_state_dict(agg_state, strict=False)
        print("Pretrained weights loaded successfully.")


if __name__ == "__main__":
    clone_repo()
    download_pretrained_weights()
    verify_model()
