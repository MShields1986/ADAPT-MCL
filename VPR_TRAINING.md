# VPR Global Localisation — Training Procedure and Results

This document describes the visual place recognition (VPR) pipeline used by AdAPT-MCL for cold-start global localisation without a known initial pose.

---

## Overview

The VPR pipeline replaces the need for a known initial pose by using onboard RGB cameras (front, left, right) to retrieve the robot's approximate location from a pre-built database of images with known poses. The particle filter is then seeded at the retrieved locations and converges to the correct pose using LiDAR observations.

```
3 camera images → VPR retrieval (per camera) → merge & deduplicate → top-k candidates → PF seeding → LiDAR tracking
```

### Models evaluated

Four VPR configurations were benchmarked end-to-end through the full localisation pipeline:

| Model | Backbone | Descriptor dim | Training | Multi-cam |
|-------|----------|----------------|----------|-----------|
| MixVPR R18 | ResNet18 | 1024 | Full fine-tune (14.4M params) | Yes |
| MixVPR R50 | ResNet50 | 1024 | Full fine-tune (25M params) | Yes |
| EigenPlaces | ResNet50 | 512 | Head only (backbone frozen) | Yes |
| **MegaLoc** | DINOv2-ViT | 8448 | Aggregator only (backbone frozen) | **Yes** |

**MegaLoc is the recommended model.** It is the only model that successfully localises on all 4 benchmark sequences. All other models fail catastrophically on 1–3 sequences despite having good candidate quality, because the particle filter locks onto aliased hypotheses from incorrect top-ranked candidates.

All models are pre-trained on outdoor street-level imagery and must be **fine-tuned** on the target indoor environment for acceptable retrieval recall. Multi-camera training (using all 3 cameras as cross-camera positive pairs) provides 3× the training data and teaches view-invariant features.

---

## Training Procedure

All VPR training runs on a GPU via the `vpr` Docker service. The mapping sequence (`data/mapping/`) provides the training data: ~12,000 extracted camera images per camera with ground-truth poses.

### Prerequisites

- NVIDIA GPU with >= 16 GB VRAM (24 GB recommended for larger batch sizes)
- `nvidia-container-toolkit` installed
- Extracted camera images from the mapping run at `data/mapping/camera_{front,left,right}/color/images/`
- Ground-truth poses at `data/mapping/gt_poses.txt`

### Step 1: Build the VPR container

```bash
cd lilocbench_ws/docker
docker compose build vpr
```

### Step 2: Extract keyframes

Select images at 0.5 m spatial intervals along the mapping trajectory. Requires time-synchronised images from all 3 cameras (412 keyframes from ~12,000 images):

```bash
docker compose run --rm vpr scripts/vpr/extract_keyframes.py \
    --data-dir /data/mapping --output /models/vpr/keyframes.pkl
```

Output: `models/vpr/keyframes.pkl`

### Step 3: Fine-tune MegaLoc (multi-camera, recommended)

Fine-tuning uses triplet loss with hard negative mining across all 3 cameras. Images from different cameras at the same pose form cross-camera positive pairs (min distance threshold 0.01 m), teaching view-invariant features. The DINOv2 backbone is frozen; only the aggregator is trained. Mixed precision (AMP) is used to fit in GPU memory.

- **Positive pairs**: images within 2 m (any camera, including cross-camera)
- **Negative pairs**: hardest in-batch negative beyond 10 m
- **Augmentation**: colour jitter (brightness 0.3, contrast 0.3, saturation 0.2, hue 0.1)

```bash
docker compose run --rm vpr scripts/vpr/finetune_megaloc_multicam.py \
    --epochs 15 --batch-size 24 --lr 1e-5 --margin 0.3
```

Training takes ~20 minutes on an RTX 4090 (24 GB). Loss decreases from 0.19 to 0.001 — converges rapidly due to the strong DINOv2 backbone.

Output: `models/vpr/megaloc_multicam_finetuned.pth`

### Step 4: Build the multi-camera database

Extract MegaLoc descriptors for all 3 cameras at each keyframe and build a FAISS flat-L2 index. The index contains 3×412 = 1236 entries, each mapping back to its keyframe pose:

```bash
docker compose run --rm vpr scripts/vpr/build_database_megaloc_multicam.py \
    --weights /models/vpr/megaloc_multicam_finetuned.pth
```

Output: `models/vpr/database_megaloc_multicam.pkl`, `models/vpr/place_index_megaloc_multicam.faiss`

### Step 5: Generate VPR candidates for each sequence

Queries the database with all 3 cameras independently (top-k per camera), then merges candidates within 1.0 m radius (keeping the best score):

```bash
for seq in static_0 dynamics_0 lt_changes_0 lt_changes_dynamics_0; do
    docker compose run --rm vpr scripts/vpr/publish_vpr_candidates_megaloc_multicam.py \
        --sequence $seq --top-k 10 \
        --output-file /data/$seq/vpr_candidates_megaloc_multicam.txt
done
```

### Optional: Train alternative models

**MixVPR R18 multi-camera** (full backbone fine-tuning):
```bash
docker compose run --rm vpr scripts/vpr/finetune_mixvpr_multicam.py \
    --epochs 20 --batch-size 96 --lr 5e-5 --margin 0.3 \
    --backbone resnet18 --out-channels 256 --out-rows 4 --mix-depth 4
```

**MixVPR R50 multi-camera** (full backbone fine-tuning):
```bash
docker compose run --rm vpr scripts/vpr/finetune_mixvpr_multicam.py \
    --epochs 20 --batch-size 32 --lr 3e-5 --margin 0.3 \
    --backbone resnet50 --layers-to-freeze 0 --out-channels 256 --out-rows 4 --mix-depth 4 \
    --output /models/vpr/mixvpr_r50_multicam_finetuned.pth
```

**EigenPlaces multi-camera** (backbone frozen, head only):
```bash
docker compose run --rm vpr scripts/vpr/finetune_eigenplaces_multicam.py \
    --epochs 15 --batch-size 48 --lr 1e-5 --margin 0.3
```

---

## Candidate Quality (VPR retrieval accuracy)

Retrieval accuracy on the first image of each test sequence. Format: best candidate distance to GT pose (rank of that candidate).

### Multi-camera models (3 cameras: front, left, right)

| Model | static_0 | dynamics_0 | lt_changes_0 | lt_changes_dyn_0 |
|-------|----------|------------|--------------|-------------------|
| MixVPR R18 | 0.22 m (#2) | 0.96 m (#2) | 0.37 m (#5) | 0.80 m (#1) |
| MixVPR R50 | 0.22 m (#1) | 0.87 m (#9) | 0.29 m (#1) | 0.29 m (#4) |
| EigenPlaces | 0.22 m (#1) | 0.56 m (#2) | 0.42 m (#2) | 0.30 m (#7) |
| **MegaLoc** | **0.22 m (#1)** | **0.96 m (#1)** | **0.37 m (#1)** | **0.29 m (#1)** |

### Single-camera models (front camera only, for comparison)

| Model | static_0 | dynamics_0 | lt_changes_0 | lt_changes_dyn_0 |
|-------|----------|------------|--------------|-------------------|
| EigenPlaces (1cam) | 0.22 m (#1) | 0.56 m (#3) | 0.29 m (#1) | 0.73 m (#3) |
| MegaLoc (1cam) | 0.21 m (#10) | 0.87 m (#5) | 0.29 m (#2) | 0.21 m (#4) |

### Analysis

**MegaLoc multicam is the only model with rank #1 on all 4 sequences**, meaning the correct location is always the top-scoring candidate. Other models place the correct candidate at rank #2–#9, causing the particle filter to lock onto incorrect hypotheses.

Multi-camera querying dramatically improves MegaLoc's ranking: single-camera MegaLoc places the correct candidate at ranks #2–#10, while multicam achieves rank #1 across the board. The additional viewpoints from the left and right cameras resolve ambiguities that a single front camera cannot.

MegaLoc also provides differentiated scores (0.87–0.69) while MixVPR scores saturate at 1.000, giving the particle filter no useful weighting information.

---

## End-to-End Localisation Results

VPR candidates are fed to the particle filter via `vpr_candidates_file` parameter. The filter uses:
- Uniform heading distribution (keyframe heading is unreliable)
- 5,000 particles across candidates
- Temperature annealing for lt_changes sequences (20 scans, β: 0.05 → 1.0)

### Full benchmark (all models × all sequences, mean position error)

| Model | static_0 | dynamics_0 | lt_changes_0 | lt_changes_dyn_0 |
|-------|----------|------------|--------------|-------------------|
| MixVPR R18 | **1297 cm** | 9.27 cm | **309 cm** | 15.47 cm |
| MixVPR R50 | **358 cm** | 8.35 cm | 10.57 cm | 10.14 cm |
| EigenPlaces | 5.10 cm | **628 cm** | **1248 cm** | **1505 cm** |
| **MegaLoc** | **5.30 cm** | **6.11 cm** | **10.41 cm** | **11.64 cm** |
| *Tracking (ref)* | *4.93 cm* | *4.52 cm* | *9.26 cm* | *8.60 cm* |
| *AMCL (ref)* | *3.12 cm* | *3.21 cm* | *13.51 cm* | *11.13 cm* |

Bold values indicate failure (>100 cm mean = localisation completely lost).

**MegaLoc is the only model that localises successfully on all 4 sequences.** The others fail catastrophically on 1–3 sequences despite having sub-1 m candidate quality — the particle filter locks onto incorrect top-ranked candidates and never recovers.

### MegaLoc vs tracking mode

| Sequence | MegaLoc VPR | Tracking | Delta |
|----------|-------------|----------|-------|
| static_0 | 5.30 cm | 4.93 cm | +0.37 cm |
| dynamics_0 | 6.11 cm | 4.52 cm | +1.59 cm |
| lt_changes_0 | 10.41 cm | 9.26 cm | +1.15 cm |
| lt_changes_dynamics_0 | 11.64 cm | 8.60 cm | +3.04 cm |

VPR global localisation adds 0.4–3.0 cm mean error compared to tracking mode with a known initial pose. The lt_changes_dynamics_0 sequence has the largest gap (3.0 cm) with a 723 cm max error spike from briefly locking onto an aliased candidate before recovering.

---

## Key Design Decisions

### MegaLoc over MixVPR/EigenPlaces

Despite MixVPR having better retrieval recall in isolation and EigenPlaces having competitive candidate quality, **only MegaLoc produces consistent rank-#1 candidates across all sequences**. The particle filter is highly sensitive to the top-ranked candidate — a rank-#2 correct candidate often leads to catastrophic failure because the filter commits to the wrong hypothesis early and cannot recover once the aliased position dominates the likelihood.

MegaLoc's DINOv2 backbone provides fundamentally stronger visual features that generalise better across environmental changes (lighting, furniture, dynamic objects). The aggregator-only fine-tuning converges to loss 0.001 (well below the 0.3 margin) in 10 epochs, while MixVPR saturates at the margin boundary (0.3001).

### Multi-camera training and querying

Training with all 3 cameras (front, left, right) provides:
- **3× training data** from the same trajectory
- **Cross-camera positive pairs** that teach view-invariant features
- **Multi-camera querying** at runtime: 3 independent database lookups with candidate merging

The multi-camera database has 3×N entries (one per camera per keyframe). At query time, each camera's query can match any camera's database entry, allowing the left camera to find a match with the right camera's descriptor if the view is similar. Candidates within 1.0 m are merged, keeping the best score.

### Uniform heading distribution

VPR keyframe heading reflects the mapping trajectory direction, not the query robot's orientation. The same location may have been traversed facing 90° during mapping but the query robot faces 260°. Using the keyframe heading as a Gaussian prior causes total failure. Uniform heading over [-π, π] is essential — the LiDAR resolves heading ambiguity within a few scan updates.

### Temperature annealing for changed maps

In long-term-change sequences, the occupancy grid map no longer matches the environment at some locations. The aliased hypothesis (wrong position, higher LiDAR score) immediately dominates the correct hypothesis (true position, lower LiDAR score due to map changes). Temperature annealing scales log-weights by β < 1 for the first N scans, keeping all hypotheses alive until odometry-integrated evidence disambiguates them.

### Depth-based refinement abandoned

ICP refinement between depth point clouds was implemented and tested with three approaches (keyframe-to-keyframe 3D ICP, multi-heading ICP, likelihood field scoring). All failed to discriminate correct from incorrect candidates — ICP fitness is uniformly high (~0.88) regardless of correctness because the indoor office environment has walls and furniture everywhere. The LiDAR particle filter is far more effective at pose discrimination.
