# AdAPT-MCL
## Adaptive Alpha Particle Tracker — Monte Carlo Localisation

A particle filter localizer for the [LILocBench](https://www.ipb.uni-bonn.de/html/projects/localization_benchmark/) indoor localization benchmark, designed to be robust to both dynamic objects and long-term structural map changes.

---

## Approach
AdAPT-MCL is a likelihood-field particle filter with a per-particle soft-EM sensor model.
The key insight is that each particle independently estimates how well its current pose hypothesis explains the observed LiDAR scan, making the filter naturally robust to:

- Dynamic objects: rays that don't match the map are absorbed by the z_short component (`γ`) and the inlier fraction `α` adapts to ignore moving obstacles
- Long-term map changes: partial map disagreement is handled gracefully — a particle with 60% map coverage still beats a particle with 30%
- Both simultaneously: per-particle independence prevents global map-normalization failure modes (contrast with ENM-MCL which fails on `lt_changes_dynamics`)

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

The `N^0.57` normalization (slightly stronger than `√N = N^0.5`) was found to be optimal via grid search over static and changed-map sequences. It balances discrimination strength against particle diversity.

---

## Performance Against [LILOC Benchmark](https://www.ipb.uni-bonn.de/html/projects/localization_benchmark/)
| Sequence type                          | AdAPT-MCL    | AMCL        | ENM-MCL   | LocNDF     |
|----------------------------------------|--------------|-------------|-----------|------------|
| Static (static_0)                      |    4.93 cm   | **3.12 cm** |   3.32 cm |    2.94 cm |
| Dynamic people (dynamics_0)            |    4.52 cm   | **3.21 cm** |   3.40 cm |    3.31 cm |
| Long-term changes (lt_changes_0)       |  **9.26 cm** |  13.51 cm   |   9.51 cm |  660 cm    |
| LTC + dynamics (lt_changes_dynamics_0) |  **8.60 cm** |  11.13 cm   | 898 cm    |  972 cm    |
| *Competition-weighted average*         |  **6.49 cm** |  ~7.06 cm   | ~94.72 cm | ~297.11 cm |

Notes:
- AMCL, ENM-MCL and LocNDF paper baselines are category-level aggregates (all sequences of that type)
- AdAPT-MCL is evaluated on a single public sequence per type and it is also provided with an initial starting pose from the ground truth data
- Competition-weighted average uses 7:5:6:2 ratio (static:dynamics:lt_changes:lt_changes_dynamics)
- AdAPT-MCL beats AMCL's competition-weighted average (6.49 vs ~7.06 cm), due to wins on long term change sequences (+4.25 cm and +2.53 cm respectively)

| Sequence              | Duration | Mean    | Median  | p95      | Max      |
|-----------------------|----------|---------|---------|----------|----------|
| static_0              | 598.6 s  | 4.93 cm | 4.90 cm |  8.96 cm | 10.79 cm |
| dynamics_0            | 159.8 s  | 4.52 cm | 4.44 cm |  8.69 cm | 12.64 cm |
| lt_changes_0          | 435.9 s  | 9.26 cm | 6.96 cm | 27.77 cm | 40.64 cm |
| lt_changes_dynamics_0 | 558.8 s  | 8.60 cm | 7.22 cm | 22.50 cm | 31.66 cm |

---

## Running
### Prerequisites
- Docker + Docker Compose
- Sequence bags at `data/<seq>/<seq>_no_cams.bag`
- Map at `data/map_office/map_office.yaml`

### Build and Run
To run the benchmarks:
```bash
cd lilocbench_ws/docker

# Run individual sequences (each starts/stops automatically)
docker compose up static_0
docker compose up dynamics_0
docker compose up lt_changes_0
docker compose up lt_changes_dynamics_0
```

For development:
```bash
cd lilocbench_ws/docker
docker compose --profile dev up -d       # starts lilocbench-dev-1 (sleep infinity)
docker exec -it lilocbench-dev-1 bash

# Inside container:
cd /catkin_ws
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -- adapt_mcl lilocbench_ros
source devel/setup.bash

# Run a sequence
rm -f /output/results/static_0/run_*.txt
timeout --signal=INT 660 roslaunch lilocbench_ros static_0.launch launch_rviz:=false
```

Results are written to `docker/output/results/<sequence>/run_1.txt` (TUM format).
`run_2.txt` and `run_3.txt` are automatically symlinked to `run_1.txt` on shutdown.

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
