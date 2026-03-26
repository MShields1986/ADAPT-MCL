#!/usr/bin/env python3
"""
VPR Global Localization Node.

Subscribes to camera images, runs place retrieval against the database,
and publishes candidate poses on /vpr_candidates for the particle filter.

This runs OUTSIDE the Docker container (needs PyTorch + CUDA).
It connects to ROS master inside the container via ROS_MASTER_URI.

Usage:
    ROS_MASTER_URI=http://localhost:11311 python3 vpr_node.py \
        --database global_loc/database.pkl \
        --index global_loc/place_index.faiss \
        --top-k 10 \
        --spread-pos 1.5 \
        --spread-angle 0.5
"""

import os
import math
import pickle
import argparse
import numpy as np
import torch
import torchvision.transforms as T
from PIL import Image

GLOBAL_LOC_DIR = os.environ.get("LILOCBENCH_MODELS", "/home/matthew/Desktop/LILocBench/lilocbench_ws/models/vpr")


def load_database(db_path, index_path):
    """Load keyframe database and FAISS index."""
    import faiss
    with open(db_path, "rb") as f:
        keyframes = pickle.load(f)
    index = faiss.read_index(index_path)
    return keyframes, index


def load_model():
    """Load EigenPlaces model (best performer from our evaluation)."""
    model = torch.hub.load(
        "gmberton/eigenplaces", "get_trained_model",
        backbone="ResNet50", fc_output_dim=512,
        trust_repo=True,
    )
    model.eval()
    return model


def get_transform():
    return T.Compose([
        T.Resize((480, 640)),
        T.ToTensor(),
        T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])


class VPRLocalizer:
    def __init__(self, keyframes, index, model, transform, device,
                 top_k=10, camera="camera_front"):
        self.keyframes = keyframes
        self.index = index
        self.model = model
        self.transform = transform
        self.device = device
        self.top_k = top_k
        self.camera = camera
        self.poses = [kf["pose"] for kf in keyframes]

    def query(self, rgb_image_np):
        """Run VPR on an RGB image (numpy HxWx3 uint8).

        Returns list of (x, y, theta, score) candidates.
        """
        img = Image.fromarray(rgb_image_np)
        tensor = self.transform(img).unsqueeze(0).to(self.device)

        with torch.no_grad():
            desc = self.model(tensor).squeeze().cpu().numpy()

        # Normalize
        desc = desc / (np.linalg.norm(desc) + 1e-8)
        desc = desc.astype(np.float32).reshape(1, -1)

        D, I = self.index.search(desc, self.top_k)

        candidates = []
        for j in range(self.top_k):
            idx = I[0][j]
            dist = D[0][j]
            pose = self.poses[idx]
            # Convert L2 distance to a weight (closer = higher)
            weight = 1.0 / (1.0 + dist)
            candidates.append({
                "x": pose[0],
                "y": pose[1],
                "theta": pose[2],
                "weight": weight,
                "db_idx": idx,
                "l2_dist": float(dist),
            })

        return candidates


def query_from_file(localizer, image_path):
    """Run VPR from a file path (for offline testing)."""
    img = np.array(Image.open(image_path).convert("RGB"))
    return localizer.query(img)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--database", type=str,
                        default=os.path.join(GLOBAL_LOC_DIR, "database.pkl"))
    parser.add_argument("--index", type=str,
                        default=os.path.join(GLOBAL_LOC_DIR, "place_index_front.faiss"))
    parser.add_argument("--top-k", type=int, default=10)
    parser.add_argument("--spread-pos", type=float, default=1.5,
                        help="Position spread for PF seeding (m)")
    parser.add_argument("--spread-angle", type=float, default=0.5,
                        help="Angle spread for PF seeding (rad)")
    parser.add_argument("--test-image", type=str, default=None,
                        help="Offline test: query this image instead of subscribing")
    parser.add_argument("--test-gt", type=str, default=None,
                        help="GT pose 'x y theta' for test evaluation")
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    print("Loading database...")
    keyframes, index = load_database(args.database, args.index)
    print(f"  {len(keyframes)} keyframes, index dim={index.d}")

    print("Loading model...")
    model = load_model().to(device)
    transform = get_transform()

    localizer = VPRLocalizer(keyframes, index, model, transform, device,
                             top_k=args.top_k)

    if args.test_image:
        # Offline test mode
        print(f"\nQuerying: {args.test_image}")
        candidates = query_from_file(localizer, args.test_image)

        print(f"\nTop-{args.top_k} candidates:")
        for i, c in enumerate(candidates):
            print(f"  {i+1}. ({c['x']:.2f}, {c['y']:.2f}, "
                  f"{math.degrees(c['theta']):.1f}°) "
                  f"w={c['weight']:.3f} L2={c['l2_dist']:.3f} "
                  f"db_idx={c['db_idx']}")

        if args.test_gt:
            gt = [float(v) for v in args.test_gt.split()]
            print(f"\nGT pose: ({gt[0]:.2f}, {gt[1]:.2f}, {math.degrees(gt[2]):.1f}°)")
            for i, c in enumerate(candidates):
                err = math.sqrt((c["x"] - gt[0]) ** 2 + (c["y"] - gt[1]) ** 2)
                print(f"  {i+1}. error={err:.2f}m")
        return

    # ROS mode
    try:
        import rospy
        from geometry_msgs.msg import PoseArray, Pose
        from sensor_msgs.msg import Image as RosImage
        from cv_bridge import CvBridge
    except ImportError:
        print("ERROR: ROS packages not available. Use --test-image for offline mode.")
        return

    rospy.init_node("vpr_localizer", anonymous=True)
    bridge = CvBridge()
    pub = rospy.Publisher("/vpr_candidates", PoseArray, queue_size=1, latch=True)

    triggered = [False]

    def on_image(msg):
        if triggered[0]:
            return
        triggered[0] = True

        rospy.loginfo("VPR: received first image, running retrieval...")
        cv_img = bridge.imgmsg_to_cv2(msg, "rgb8")
        candidates = localizer.query(cv_img)

        # Publish as PoseArray (weight in position.z)
        pa = PoseArray()
        pa.header.stamp = rospy.Time.now()
        pa.header.frame_id = "map"
        for c in candidates:
            p = Pose()
            p.position.x = c["x"]
            p.position.y = c["y"]
            p.position.z = c["weight"]  # piggyback weight
            # Set orientation from theta
            p.orientation.z = math.sin(c["theta"] / 2)
            p.orientation.w = math.cos(c["theta"] / 2)
            pa.poses.append(p)

        pub.publish(pa)
        rospy.loginfo(f"VPR: published {len(candidates)} candidates")
        for i, c in enumerate(candidates):
            rospy.loginfo(f"  {i+1}. ({c['x']:.2f}, {c['y']:.2f}, "
                          f"{math.degrees(c['theta']):.1f}°) w={c['weight']:.3f}")

    sub = rospy.Subscriber("/camera_front/color/image_raw", RosImage,
                           on_image, queue_size=1)
    rospy.loginfo("VPR node ready, waiting for camera images...")
    rospy.spin()


if __name__ == "__main__":
    main()
