#!/usr/bin/env bash
# run_trials.sh — 3 trials × 2 configs × 4 sequences
# Config A: current best  (tracking for lt_changes, global+CAER for static/dynamics)
# Config B: global + CPD primary (no prior pose, CPD seeds all sequences)

set -e
RESULTS_DIR=/home/matthew/Desktop/LILocBench/lilocbench_ws/docker/output/results
GT_DIR=/home/matthew/Desktop/LILocBench/data
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EVAL="${SCRIPT_DIR}/eval_simple.py"
N_TRIALS=3

run_and_eval() {
  local seq=$1 timeout=$2 label=$3
  shift 3
  local extra_args="$@"

  docker exec lilocbench-dev-1 bash -c "
    rm -f /output/results/${seq}/run_1.txt &&
    source /catkin_ws/devel/setup.bash &&
    timeout --signal=INT ${timeout} roslaunch lilocbench_ros ${seq}.launch launch_rviz:=false \
      ${extra_args} 2>&1 | tail -1
  "

  # Extract mean error
  local mean
  mean=$(python3 $EVAL $GT_DIR/${seq}/gt_poses.txt $RESULTS_DIR/${seq}/run_1.txt \
    2>/dev/null | grep "Mean:" | awk '{print $2}')
  echo "${label},${seq},${mean}"
}

echo "label,sequence,mean_cm"

for trial in 1 2 3; do

  # ── Config A: current best ──────────────────────────────────────────────────

  # static_0: global + CAER (no scan_match_seed, global_init_steps=60)
  run_and_eval static_0 220 "A_trial${trial}" \
    "init_random_fraction:=1.0 kld_max_particles:=100000 global_init_steps:=60 use_caer_rerank:=true"

  # dynamics_0: global + seed + CAER
  run_and_eval dynamics_0 80 "A_trial${trial}" \
    "init_random_fraction:=1.0 kld_max_particles:=100000 global_init_steps:=60 use_scan_match_seed:=true use_caer_rerank:=true"

  # lt_changes_0: tracking (known pose)
  run_and_eval lt_changes_0 200 "A_trial${trial}" \
    ""

  # lt_changes_dynamics_0: tracking (known pose)
  run_and_eval lt_changes_dynamics_0 220 "A_trial${trial}" \
    ""

  # ── Config B: global + CPD primary ─────────────────────────────────────────

  run_and_eval static_0 220 "B_trial${trial}" \
    "init_random_fraction:=1.0 kld_max_particles:=100000 global_init_steps:=0 use_scan_match_seed:=true use_cpd_rerank:=true use_caer_rerank:=false cpd_sigma:=0.10 cpd_w:=0.20"

  run_and_eval dynamics_0 80 "B_trial${trial}" \
    "init_random_fraction:=1.0 kld_max_particles:=100000 global_init_steps:=0 use_scan_match_seed:=true use_cpd_rerank:=true use_caer_rerank:=false cpd_sigma:=0.10 cpd_w:=0.20"

  run_and_eval lt_changes_0 200 "B_trial${trial}" \
    "init_random_fraction:=1.0 kld_max_particles:=100000 global_init_steps:=0 use_scan_match_seed:=true use_cpd_rerank:=true use_caer_rerank:=false cpd_sigma:=0.10 cpd_w:=0.20"

  run_and_eval lt_changes_dynamics_0 220 "B_trial${trial}" \
    "init_random_fraction:=1.0 kld_max_particles:=100000 global_init_steps:=0 use_scan_match_seed:=true use_cpd_rerank:=true use_caer_rerank:=false cpd_sigma:=0.10 cpd_w:=0.20"

done

echo "--- DONE ---"
