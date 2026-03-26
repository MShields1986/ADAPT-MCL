#!/usr/bin/env python3
"""
One-shot VPR candidate publisher for offline testing.

Reads the first camera image from extracted data, runs VPR retrieval,
and publishes candidates on /vpr_candidates as a latched message.

Run this BEFORE starting the localizer + bag playback.

Usage (from host, with ROS_MASTER_URI pointing to Docker):
    ROS_MASTER_URI=http://localhost:11311 python3 publish_vpr_candidates.py \
        --sequence mapping --top-k 10
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

# Add parent dir to path
sys.path.insert(0, os.path.dirname(__file__))

DATA_BASE = os.environ.get("LILOCBENCH_DATA", "/home/matthew/Desktop/LILocBench/data")
GLOBAL_LOC_DIR = os.environ.get("LILOCBENCH_MODELS", "/home/matthew/Desktop/LILocBench/lilocbench_ws/models/vpr")


def get_first_image(sequence, camera="camera_front"):
    """Get the first extracted image path for a sequence."""
    img_dir = os.path.join(DATA_BASE, sequence, camera, "color", "images")
    images = sorted(os.listdir(img_dir))
    if not images:
        raise RuntimeError(f"No images in {img_dir}")
    return os.path.join(img_dir, images[0])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sequence", type=str, default="mapping")
    parser.add_argument("--camera", type=str, default="camera_front")
    parser.add_argument("--database", type=str,
                        default=os.path.join(GLOBAL_LOC_DIR, "database.pkl"))
    parser.add_argument("--index", type=str,
                        default=os.path.join(GLOBAL_LOC_DIR, "place_index_front_finetuned.faiss"))
    parser.add_argument("--weights", type=str,
                        default=os.path.join(GLOBAL_LOC_DIR, "eigenplaces_finetuned.pth"),
                        help="Fine-tuned model weights (empty=pretrained)")
    parser.add_argument("--top-k", type=int, default=10)
    parser.add_argument("--output-file", type=str, default=None,
                        help="Write candidates to file instead of publishing to ROS")
    args = parser.parse_args()

    import faiss

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    # Load database
    with open(args.database, "rb") as f:
        keyframes = pickle.load(f)
    index = faiss.read_index(args.index)
    poses = [kf["pose"] for kf in keyframes]

    # Load model
    model = torch.hub.load(
        "gmberton/eigenplaces", "get_trained_model",
        backbone="ResNet50", fc_output_dim=512, trust_repo=True,
    )
    if args.weights and os.path.exists(args.weights):
        model.load_state_dict(torch.load(args.weights, map_location=device))
        print(f"  Loaded fine-tuned weights: {args.weights}")
    model = model.to(device).eval()
    transform = T.Compose([
        T.Resize((480, 640)), T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])

    # Get first image
    img_path = get_first_image(args.sequence, args.camera)
    print(f"Query image: {img_path}")

    img = Image.open(img_path).convert("RGB")
    tensor = transform(img).unsqueeze(0).to(device)
    with torch.no_grad():
        desc = model(tensor).squeeze().cpu().numpy()
    desc = desc / (np.linalg.norm(desc) + 1e-8)
    desc = desc.astype(np.float32).reshape(1, -1)

    D, I = index.search(desc, args.top_k)

    candidates = []
    for j in range(args.top_k):
        idx = I[0][j]
        dist = D[0][j]
        pose = poses[idx]
        weight = 1.0 / (1.0 + dist)
        candidates.append({
            "x": pose[0], "y": pose[1], "theta": pose[2],
            "weight": weight, "db_idx": idx, "l2_dist": float(dist),
        })

    print(f"\nTop-{args.top_k} candidates:")
    for i, c in enumerate(candidates):
        print(f"  {i+1}. ({c['x']:.2f}, {c['y']:.2f}, "
              f"{math.degrees(c['theta']):.1f}°) w={c['weight']:.3f}")

    if args.output_file:
        # Write candidates to a simple text file for the C++ node to read
        with open(args.output_file, "w") as f:
            for c in candidates:
                f.write(f"{c['x']} {c['y']} {c['theta']} {c['weight']}\n")
        print(f"\nWrote candidates to {args.output_file}")
        return

    # Publish to ROS
    try:
        import rospy
        from geometry_msgs.msg import PoseArray, Pose
    except ImportError:
        print("\nROS not available. Use --output-file to write candidates to file.")
        return

    rospy.init_node("vpr_publisher", anonymous=True)
    pub = rospy.Publisher("/vpr_candidates", PoseArray, queue_size=1, latch=True)

    pa = PoseArray()
    pa.header.stamp = rospy.Time.now()
    pa.header.frame_id = "map"
    for c in candidates:
        p = Pose()
        p.position.x = c["x"]
        p.position.y = c["y"]
        p.position.z = c["weight"]
        p.orientation.z = math.sin(c["theta"] / 2)
        p.orientation.w = math.cos(c["theta"] / 2)
        pa.poses.append(p)

    pub.publish(pa)
    print(f"\nPublished {len(candidates)} candidates on /vpr_candidates (latched)")
    print("Ctrl+C to exit when done.")
    rospy.spin()


if __name__ == "__main__":
    main()
