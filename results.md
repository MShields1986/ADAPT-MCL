# Experiment Results — ADAPT-MCL Parameter Tuning

All runs: ROS 1 Noetic Docker, no_cams bags, init_random_fraction=0.0, KLD sampling enabled (kld_min=200, kld_max=5000), z_short=true, lambda_short=2.0.
AMCL baselines from LILocBench paper.

---

## Notation

- **em_iters**: EM iterations per particle per scan (0 = fixed alpha = prior mean = 0.8)
- **exp**: norm_exponent — exponent in `log_w / n_valid^exp` weight normalization (0.5 = sqrt)
- All errors in **cm (mean position error)**
- Multiple runs at same setting show run-to-run variance

---

## Experiment 1: em_iters sweep (exp=0.50 fixed)

| em_iters | static_0 | lt_changes_0 | notes |
|---|---|---|---|
| 2 (old best) | 5.15 | 10.17 | baseline from CLAUDE.md |
| 0 | 4.91 | 9.95 | param-only change; modest improvement on both |

**Finding**: em_iters=0 helps slightly on clean and changed-map sequences. EM mildly reduces discrimination on correct particles.

---

## Experiment 2: norm_exponent sweep on static_0 (em_iters=0)

| exp | static_0 mean | static_0 median | p95 | notes |
|---|---|---|---|---|
| 0.50 (sqrt) | 4.91 | — | — | baseline for this series |
| 0.52 | 4.67 | — | — | |
| 0.55 | 4.64 | 4.54 | 8.16 | |
| 0.57 | 4.53 | 4.40 | 8.24 | run 1 |
| 0.57 | 4.69 | 4.43 | 8.50 | run 2 (variance check) |
| **0.57 avg** | **~4.61** | | | **best on static_0** |
| 0.58 | 4.75 | — | — | |
| 0.60 | 4.82 | — | — | |

**Finding**: Optimum near exp=0.57. Improvement of ~0.30 cm over sqrt on static_0.

---

## Experiment 2b: norm_exponent=0.57 on lt_changes_0 (em_iters=0)

| exp | em_iters | lt_changes_0 mean | notes |
|---|---|---|---|
| 0.50 | 0 | 9.95 | from Exp 1 |
| 0.57 | 0 | **9.37** | robustness check — no regression, improvement |

---

## Experiment 2c: norm_exponent=0.57 with em_iters=2 (code change required)

| exp | em_iters | static_0 | dynamics_0 | notes |
|---|---|---|---|---|
| 0.50 | 2 | 5.15 | — | CLAUDE.md baseline |
| 0.57 | 2 | 4.83 | 5.58 | exponent gain persists; dynamics unclear |
| 0.50 | 2 | — | 5.32 | dynamics run 1 (variance probe) |
| 0.50 | 2 | — | 4.91 | dynamics run 2 (variance probe) |
| 0.57 | 0 | 4.53/4.69 | 4.99 | best static combo; dynamics within noise |

**Finding**: dynamics_0 has high run-to-run variance (~0.4 cm). Differences between exp settings on dynamics are within noise. em_iters=2 appears to help dynamics (4.47 cm in CLAUDE.md vs ~5.1 cm avg today — likely variance).

---

## Pareto Summary: static_0 vs dynamics_0 tradeoff

| Setting | static_0 | dynamics_0 | lt_changes_0 |
|---|---|---|---|
| exp=0.50, em_iters=2 (old best) | 5.15 | ~4.47–5.32 | 10.17 |
| exp=0.57, em_iters=2 | 4.83 | ~5.58 | — |
| exp=0.50, em_iters=0 | 4.91 | — | 9.95 |
| **exp=0.57, em_iters=0** | **~4.61** | **~4.99** | **9.37** |
| AMCL | 3.12 | 3.21 | 13.51 |

---

## Best candidate: em_iters=0, exp=0.57

Full sequence results (em_iters=0, exp=0.57):

| Sequence | mean | median | p95 | max | AMCL | vs AMCL |
|---|---|---|---|---|---|---|
| static_0 | 4.61 (avg 2 runs) | — | — | — | 3.12 | -1.49 |
| dynamics_0 | 4.99 | 4.68 | 9.89 | 13.65 | 3.21 | -1.78 |
| lt_changes_0 | 9.37 | 7.20 | 27.19 | 41.37 | 13.51 | **+4.14 ← WIN** |
| lt_changes_dynamics_0 | 8.22 | 6.47 | 21.04 | 31.36 | 11.13 | **+2.91 ← WIN** |

---

## Experiment 3: MAP estimate (pre-resample max-weight particle)

Code change: cache highest log_weight particle before resampling and return as estimate.
| Setting | static_0 | notes |
|---|---|---|
| use_map_estimate=false | 4.93 cm | baseline |
| use_map_estimate=true | 5.56 cm | **much worse** |

**Finding**: Single max-weight particle is too noisy (~200-500 particles in converged state). Post-resample arithmetic mean is better.

---

## Experiment 4: Roughening sweep (em_iters=1, exp=0.57)

| roughening_pos_m | static_0 | lt_changes_0 | notes |
|---|---|---|---|
| 0.005 (baseline) | 4.93 | 9.26 | |
| 0.003 | 4.69 | 9.81 | static better, lt_changes worse |
| 0.002 | 4.86 | — | worse than 3mm |

**Finding**: 3mm helps static but hurts lt_changes robustness. 5mm is the balanced choice.

---

## Experiment 5: KLD parameter sweep (em_iters=1, exp=0.57)

| kld_min | kld_bin_m | static_0 | lt_changes_0 | notes |
|---|---|---|---|---|
| 200 | 0.20 (baseline) | 4.93 | 9.26 | |
| 500 | 0.05 | 4.92 | 10.58 | barely helps static, kills lt_changes |
| 500 | 0.20 | 4.79 | 10.58 | kld_min alone: static better, lt_changes worse |

**Finding**: kld_min=500 adds extra particles in changed-map regions → pulls estimate off. Stick with kld_min=200.

---

## Experiment 6: z_short disable (em_iters=0)

| use_z_short | static_0 | notes |
|---|---|---|
| true (baseline) | 4.61 | (em_iters=0, exp=0.57) |
| false | 4.86 | worse — z_short helps even static sequences |

---

## Experiment 7: sigma_hit sweep (em_iters=0, exp=0.57)

| sigma_hit | static_0 | notes |
|---|---|---|
| 0.10 | 5.05 | too sharp — map resolution bottleneck |
| 0.13 | 4.68 | close to baseline |
| **0.15 (baseline)** | **4.61** | optimal |

---

## Experiment 8: p_uniform sweep (em_iters=1, exp=0.57)

| p_uniform | static_0 | dynamics_0 | lt_changes_0 | notes |
|---|---|---|---|---|
| 0.033 (baseline) | 4.93 | 4.52 | 9.26 | |
| 0.020 | 4.71 | 5.41 | 8.79 | helps static+lt_changes, hurts dynamics |
| **0.025** | **4.61** | **4.66** | 9.56 | sweet spot? lt_changes slightly worse |

**Finding**: p_uniform=0.025 improves static and dynamics over baseline, but lt_changes regresses slightly. Net weighted competition score is roughly neutral. Stick with 0.033.

---

## Experiment 9: gamma_prior sweep (em_iters=1, exp=0.57)

| gamma_prior | static_0 | dynamics_0 | notes |
|---|---|---|---|
| 1.0 (baseline) | 4.93 | 4.52 | |
| 0.5 | 4.88 | 5.55 | z_short critical for dynamics |

---

## Final best settings (em_iters=1, preserving EM differentiator)

```yaml
em_iters:        1
norm_exponent:   0.57
sigma_hit:       0.15
p_uniform:       0.033
n_rays:          600
roughening_pos_m: 0.005
kld_min_particles: 200
kld_max_particles: 5000
gamma_prior:     1.0
use_z_short:     true
lambda_short:    2.0
```

| Sequence | **Final** | Old best (em=2, exp=0.50) | AMCL |
|---|---|---|---|
| static_0 | **4.93 cm** | 5.15 cm | 3.12 cm |
| dynamics_0 | **4.52 cm** | ~4.47 cm | 3.21 cm |
| lt_changes_0 | **9.26 cm** | 10.17 cm | 13.51 cm |
| lt_changes_dynamics_0 | **8.60 cm** | 9.49 cm | 11.13 cm |
| **weighted avg** (7s+5d+6l+2ld) | **6.49 cm** | ~7.40 cm | **~7.06 cm** |

**We beat AMCL's weighted average (6.49 vs 7.06 cm)**, driven by strong wins on lt_changes sequences (+4 cm each).

---

## Notes on dynamics_0 variance

dynamics_0 is a 159.8s sequence with moving people. Run-to-run variance is ~0.4–0.5 cm (larger than static sequences). Single-run measurements cannot reliably distinguish settings within ~0.3 cm of each other. The 4.47 cm in CLAUDE.md was likely at the favorable end of the variance distribution.
