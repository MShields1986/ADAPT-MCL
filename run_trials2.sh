#!/usr/bin/env bash
# run_trials2.sh — Config A (global+CAER) vs Config B (global+CPD) across
# all GT sequences at multiple bag start offsets to test initialisation robustness.
#
# Config A: global + scan-match seed + CAER re-ranking (no prior pose)
# Config B: global + CPD primary grid scorer (outlier-aware GMM, no prior pose)
# Both configs apply identically to every sequence.

RESULTS_DIR=/home/matthew/Desktop/LILocBench/lilocbench_ws/docker/output/results
GT_DIR=/home/matthew/Desktop/LILocBench/data
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EVAL="${SCRIPT_DIR}/eval_simple.py"

CAER_ARGS="init_random_fraction:=1.0 kld_max_particles:=100000 global_init_steps:=0 use_scan_match_seed:=true use_caer_rerank:=true use_cpd_rerank:=false"
CPD_ARGS="init_random_fraction:=1.0 kld_max_particles:=100000 global_init_steps:=0 use_scan_match_seed:=true use_cpd_rerank:=true use_caer_rerank:=false cpd_sigma:=0.10 cpd_w:=0.20"

run_and_eval() {
  local seq=$1 timeout=$2 label=$3 start_s=$4
  shift 4
  local extra_args="$@"

  docker exec lilocbench-dev-1 bash -c "
    rm -f /output/results/${seq}/run_1.txt &&
    source /catkin_ws/devel/setup.bash &&
    timeout --signal=INT ${timeout} roslaunch lilocbench_ros ${seq}.launch launch_rviz:=false \
      bag_start_s:=${start_s} ${extra_args} 2>&1 | tail -1
  "

  local result
  result=$(python3 $EVAL $GT_DIR/${seq}/gt_poses.txt $RESULTS_DIR/${seq}/run_1.txt 2>/dev/null)
  local mean=$(echo "$result" | grep "Mean:"   | awk '{print $2}')
  local max=$( echo "$result" | grep "Max:"    | awk '{print $2}')
  local p95=$( echo "$result" | grep "p95:"    | awk '{print $2}')
  echo "${label},${seq},${start_s},${mean},${p95},${max}"
}

echo "label,sequence,start_s,mean_cm,p95_cm,max_cm"

# ── static_0 (598s) — offsets: 0, 30, 60, 120 ──────────────────────────────
for S in 0 30 60 120; do
  T=$(( 220 - S/4 ))
  run_and_eval static_0 $T "A" $S "$CAER_ARGS"
  run_and_eval static_0 $T "B" $S "$CPD_ARGS"
done

# ── dynamics_0 (56s) — offsets: 0, 15, 30 ───────────────────────────────────
for S in 0 15 30; do
  run_and_eval dynamics_0 80 "A" $S "$CAER_ARGS"
  run_and_eval dynamics_0 80 "B" $S "$CPD_ARGS"
done

# ── lt_changes_0 (435s) — offsets: 0, 30, 60, 120 ───────────────────────────
for S in 0 30 60 120; do
  T=$(( 200 - S/4 ))
  run_and_eval lt_changes_0 $T "A" $S "$CAER_ARGS"
  run_and_eval lt_changes_0 $T "B" $S "$CPD_ARGS"
done

# ── lt_changes_dynamics_0 (558s) — offsets: 0, 30, 60, 120 ─────────────────
for S in 0 30 60 120; do
  T=$(( 220 - S/4 ))
  run_and_eval lt_changes_dynamics_0 $T "A" $S "$CAER_ARGS"
  run_and_eval lt_changes_dynamics_0 $T "B" $S "$CPD_ARGS"
done

# ── mapping (811s) — offsets: 0, 30, 60, 120, 180 ───────────────────────────
for S in 0 30 60 120 180; do
  T=$(( 240 - S/4 ))
  run_and_eval mapping $T "A" $S "$CAER_ARGS"
  run_and_eval mapping $T "B" $S "$CPD_ARGS"
done

echo "--- DONE ---"
