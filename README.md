# AdAPT-MCL
## Adaptive Alpha Particle Tracker — Monte Carlo Localisation

A particle filter localiser for the [LILocBench](https://www.ipb.uni-bonn.de/html/projects/localisation_benchmark/) indoor localisation benchmark, designed to be robust to both dynamic objects and long-term structural map changes.

---

## Approach
AdAPT-MCL is a likelihood-field particle filter with a per-particle soft-EM sensor model.
The key insight is that each particle independently estimates how well its current pose hypothesis explains the observed LiDAR scan, making the filter naturally robust to:

- Dynamic objects: rays that don't match the map are absorbed by the z_short component (`γ`) and the inlier fraction `α` adapts to ignore moving obstacles
- Long-term map changes: partial map disagreement is handled gracefully — a particle with 60% map coverage still beats a particle with 30%
- Both simultaneously: per-particle independence prevents global map-normalisation failure modes (contrast with ENM-MCL which fails on `lt_changes_dynamics`)

### Sensor model (soft EM + z_short)
For each particle, given `N` stratified LiDAR ray endpoints in the map frame:
```
Three-component mixture per ray:
  p(z_k | x_i) = α_i · p_hit(d_k) + γ_i · p_short(z_k) + (1 - α_i - γ_i) · p_uniform

  p_hit(d)    = exp(-d² / 2σ²)                Gaussian hit model (σ = 0.15 m)
  p_short(z)  = λ · exp(-λ·z)                 Short-reading model (λ = 2.0 /m)
  p_uniform   = 1/z_max                        Uniform outlier model

EM update (1 iteration per particle):
  r_hit_k   = α · p_hit(k)   / denom          (E-step responsibilities)
  r_short_k = γ · p_short(k) / denom
  α_new = (Σ r_hit   + α_prior) / (N + α_prior + β_prior + γ_prior)   (M-step)
  γ_new = (Σ r_short + γ_prior) / (N + α_prior + β_prior + γ_prior)

Log weight:
  log w_i = (1 / N^0.57) · Σ_k log[α_i · p_hit(k) + γ_i · p_short(k) + (1-α_i-γ_i) · p_unif]
```

The `N^0.57` normalisation (slightly stronger than `√N = N^0.5`) was found to be optimal via grid search over static and changed-map sequences. It balances discrimination strength against particle diversity.

### VPR global localisation

For cold-start initialisation (no known initial pose), AdAPT-MCL uses a two-stage visual place recognition (VPR) pipeline with multi-camera queries:

1. **Database construction** (offline, once per environment): Extract keyframes at 0.5 m intervals from a mapping run. For each keyframe, compute a global image descriptor per camera (front, left, right) using a fine-tuned [MegaLoc](https://github.com/gmberton/MegaLoc) (DINOv2-ViT backbone) model. Index all 3×N descriptors in a FAISS flat-L2 index — each entry maps back to its keyframe pose.

2. **Runtime initialisation**: On the first camera frame, query the database with all 3 cameras independently (top-k per camera), then merge and deduplicate candidates within 1.0 m radius. Each candidate provides a 2D position but **no heading** (the keyframe heading is unreliable — it reflects the mapping trajectory direction, not the query robot's orientation). The particle filter is seeded with Gaussian position clusters (σ = 0.5–2.0 m) and **uniform heading** over [-π, π]. Temperature annealing (β ramping from 0.05 to 1.0 over 20 scans) is applied on long-term-change sequences to prevent the changed-map LiDAR likelihood from immediately collapsing particles to an aliased hypothesis.

MegaLoc is fine-tuned on the target environment's mapping data using triplet loss with multi-camera training — images from all 3 cameras at the same pose serve as cross-camera positive pairs, teaching view-invariant features. See [VPR_TRAINING.md](VPR_TRAINING.md) for the full training procedure, retrieval results, and design decisions.

---

## Performance Against [LILOC Benchmark](https://www.ipb.uni-bonn.de/html/projects/localisation_benchmark/)

### Tracking mode (known initial pose)
| Sequence type                          | AdAPT-MCL    | AMCL        | ENM-MCL   | LocNDF     |
|----------------------------------------|--------------|-------------|-----------|------------|
| Static (static_0)                      |    4.93 cm   | **3.12 cm** |   3.32 cm |    2.94 cm |
| Dynamic people (dynamics_0)            |    4.52 cm   | **3.21 cm** |   3.40 cm |    3.31 cm |
| Long-term changes (lt_changes_0)       |  **9.26 cm** |  13.51 cm   |   9.51 cm |  660 cm    |
| LTC + dynamics (lt_changes_dynamics_0) |  **8.60 cm** |  11.13 cm   | 898 cm    |  972 cm    |
| *Competition-weighted average*         |  **6.49 cm** |  ~7.06 cm   | ~94.72 cm | ~297.11 cm |

### VPR global localisation (no known initial pose)
| Sequence              | MegaLoc multicam | Tracking | AMCL    | Convergence |
|-----------------------|------------------|----------|---------|-------------|
| static_0              |  5.30 cm         | 4.93 cm  | 3.12 cm | ~8 s        |
| dynamics_0            |  6.11 cm         | 4.52 cm  | 3.21 cm | <1 s        |
| lt_changes_0          | 10.41 cm         | 9.26 cm  | 13.51 cm | <1 s       |
| lt_changes_dynamics_0 | 11.64 cm         | 8.60 cm  | 11.13 cm | ~5 s       |

Notes:
- MegaLoc multicam is the only VPR model that successfully localises on all 4 sequences
- VPR results include full trajectory (no convergence window excluded)
- Beats AMCL on lt_changes_0 despite starting with zero pose knowledge
- MixVPR and EigenPlaces were also evaluated but fail catastrophically on 1–3 sequences each
- AMCL, ENM-MCL and LocNDF baselines are category-level aggregates from the benchmark papers

---

## Running

### Prerequisites
- Docker + Docker Compose
- NVIDIA GPU + nvidia-container-toolkit (for VPR training/inference)
- Sequence bags at `data/<seq>/<seq>_no_cams.bag`
- Extracted camera images at `data/<seq>/camera_front/color/images/` (for VPR)
- Map at `data/map_office/map_office.yaml`

### Tracking mode (known initial pose)
```bash
cd lilocbench_ws/docker

# Run individual sequences
docker compose --profile train up static_0
docker compose --profile train up dynamics_0
docker compose --profile train up lt_changes_0
docker compose --profile train up lt_changes_dynamics_0
```

### VPR global localisation (no known initial pose)

#### 1. Build the VPR container
```bash
cd lilocbench_ws/docker
docker compose build vpr
```

#### 2. Extract keyframes (one-time, from the mapping run)

Extract keyframes from the mapping trajectory at 0.5 m intervals (requires extracted camera images from all 3 cameras):
```bash
docker compose run --rm vpr scripts/vpr/extract_keyframes.py \
    --data-dir /data/mapping --output /models/vpr/keyframes.pkl
```

#### 3. Fine-tune MegaLoc (multi-camera)

Fine-tuning uses triplet loss with hard negative mining across all 3 cameras. Images from different cameras at the same pose form cross-camera positive pairs, teaching view-invariant features:

```bash
docker compose run --rm vpr scripts/vpr/finetune_megaloc_multicam.py \
    --epochs 15 --batch-size 24 --lr 1e-5 --margin 0.3
```

Output: `models/vpr/megaloc_multicam_finetuned.pth`

#### 4. Build the multi-camera database

Compute MegaLoc descriptors for all 3 cameras at each keyframe and build a FAISS index (3×412 = 1236 entries):

```bash
docker compose run --rm vpr scripts/vpr/build_database_megaloc_multicam.py \
    --weights /models/vpr/megaloc_multicam_finetuned.pth
```

#### 5. Generate VPR candidates for each sequence

Queries the database with all 3 cameras, retrieves top-k per camera, then merges within 1.0 m radius:

```bash
for seq in static_0 dynamics_0 lt_changes_0 lt_changes_dynamics_0; do
    docker compose run --rm vpr scripts/vpr/publish_vpr_candidates_megaloc_multicam.py \
        --sequence $seq --top-k 10 \
        --output-file /data/$seq/vpr_candidates_megaloc_multicam.txt
done
```

#### 6. Run localisation with VPR seeding
```bash
cd lilocbench_ws/docker
docker compose --profile dev up -d
docker exec lilocbench-dev-1 bash -c "
    source /catkin_ws/devel/setup.bash &&
    timeout --signal=INT 220 roslaunch lilocbench_ros static_0_vpr.launch launch_rviz:=false"
```

### Evaluate
```bash
python3 src/lilocbench_ros/scripts/eval.py \
    /path/to/data/<seq>/gt_poses.txt \
    docker/output/results/<seq>/run_1.txt
```

---

## Parameters
### Core Filter
| Parameter                | Value     | Description |
|--------------------------|-----------|-------------|
| `use_kld_sampling`       | `true`    | Adaptive particle count (Fox 2001) |
| `kld_max_particles`      | 5000      | Initial and maximum particle count |
| `kld_min_particles`      | 200       | Minimum particle count after resample |
| `kld_bin_size_m`         | 0.20 m    | KLD position bin width |
| `kld_bin_size_rad`       | 0.20 rad  | KLD heading bin width |
| `ess_resample_threshold` | 0.5       | Resample when ESS/N < this |
| `ess_recovery_threshold` | 0.02      | Inject random particles only below this (very low — ESS-based recovery backfires at higher values) |
| `roughening_pos_m`       | 0.005 m   | Position jitter after resampling |
| `roughening_angle_rad`   | 0.005 rad | Angle jitter after resampling |

### Sensor Model
| Parameter       | Value  | Description |
|-----------------|--------|-------------|
| `n_rays`        | 600    | Stratified ray subsampling per scan update |
| `sigma_hit`     | 0.15 m | Gaussian std dev for likelihood field (map resolution 5cm limits benefit of sharper values) |
| `p_uniform`     | 0.033  | Uniform outlier density (≈ 1/30 m) |
| `alpha_prior`   | 8.0    | Beta prior on inlier fraction (prior mean = 0.8) |
| `beta_prior`    | 2.0    | Beta prior denominator component |
| `em_iters`      | 1      | EM iterations per particle per scan (0=no EM, 1=best balance, 2=hurts dynamics) |
| `norm_exponent` | 0.57   | Log-weight normalisation exponent (tuned; 0.5=sqrt was default) |
| `use_z_short`   | `true` | Enable short-reading component for dynamic obstacles |
| `lambda_short`  | 2.0 /m | Exponential decay for short-reading model |
| `gamma_prior`   | 1.0    | Prior pseudo-count for short-reading fraction |

### Motion Model (Omnidirectional)
| Parameter | Value | Description |
|-----------|-------|-------------|
| `alpha1`  | 0.005 | Rotation noise from rotation |
| `alpha2`  | 0.01  | Rotation noise from translation |
| `alpha3`  | 0.01  | Translation noise from translation |
| `alpha4`  | 0.01  | Translation noise from rotation |
| `alpha5`  | 0.01  | Lateral translation noise from translation |

### VPR Initialisation
| Parameter                | Value     | Description |
|--------------------------|-----------|-------------|
| `vpr_candidates_file`   | `""`      | Path to VPR candidates file (empty = disabled) |
| `init_spread_pos_m`     | 0.5–2.0 m | Position Gaussian spread per candidate |
| `n_particles`           | 5000      | Particle count for VPR seeding |
| `init_temperature`      | 0.05      | Starting temperature for annealing (lt_changes only) |
| `init_temperature_steps`| 20        | Scans over which temperature ramps to 1.0 |
