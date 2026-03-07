#!/usr/bin/env bash
# Run the localizer for every sequence.
#
# Usage (inside Docker):
#   ./run_all_sequences.sh \
#       <bags_dir>       — dir containing <seq>/<seq>.bag files \
#       <data_dir>       — dir containing <seq>/init_pose.txt and map_office/ \
#       <output_dir>     — where results/ will be written
#
# The shared map is expected at: <data_dir>/map_office/map_office.yaml
# Per-sequence init pose:        <data_dir>/<seq>/init_pose.txt  ("x y theta")
#
# Produces:
#   <output_dir>/results/<seq>/run_1.txt
#   <output_dir>/results/<seq>/run_2.txt  (symlink → run_1.txt)
#   <output_dir>/results/<seq>/run_3.txt  (symlink → run_1.txt)
#   <output_dir>/results/description.json

set -euo pipefail

BAGS_DIR="${1:?Usage: $0 <bags_dir> <data_dir> <output_dir>}"
DATA_DIR="${2:?}"
OUTPUT_DIR="${3:?}"

MAP_YAML="${DATA_DIR}/map_office/map_office.yaml"
RESULTS_DIR="${OUTPUT_DIR}/results"
mkdir -p "${RESULTS_DIR}"

if [[ ! -f "${MAP_YAML}" ]]; then
    echo "ERROR: map not found at ${MAP_YAML}" >&2
    exit 1
fi

# Write description.json (edit before CodaBench submission).
cat > "${RESULTS_DIR}/description.json" << 'JSON'
{
  "name": "Adaptive Soft-EM Particle Filter",
  "pdf_url": "",
  "code_url": "",
  "short_description": "Per-particle soft EM sensor model with Beta-regularized inlier fraction estimation, built on a likelihood field particle filter. Dual SICK TIM LiDAR, Dingo differential drive odometry."
}
JSON

# Known sequences with their bag filenames.
SEQUENCES=(mapping static_0 dynamics_0 lt_changes_0 lt_changes_dynamics_0)

for SEQ in "${SEQUENCES[@]}"; do
    # Find the bag file (try <seq>.bag and then any .bag in the dir).
    BAG_FILE="${BAGS_DIR}/${SEQ}/${SEQ}.bag"
    if [[ ! -f "${BAG_FILE}" ]]; then
        BAG_FILE=$(ls "${BAGS_DIR}/${SEQ}"/*.bag 2>/dev/null | head -1 || true)
    fi
    if [[ -z "${BAG_FILE}" ]]; then
        echo "WARNING: no .bag found for ${SEQ} in ${BAGS_DIR}/${SEQ} — skipping" >&2
        continue
    fi

    INIT_FILE="${DATA_DIR}/${SEQ}/init_pose.txt"
    INIT_X=0.0; INIT_Y=0.0; INIT_THETA=0.0
    if [[ -f "${INIT_FILE}" ]]; then
        read -r INIT_X INIT_Y INIT_THETA < "${INIT_FILE}"
    else
        echo "WARNING: ${INIT_FILE} not found — using (0, 0, 0) for ${SEQ}" >&2
    fi

    echo "==> ${SEQ}: init=(${INIT_X}, ${INIT_Y}, ${INIT_THETA}), bag=${BAG_FILE}"

    roslaunch lilocbench_ros localize.launch \
        sequence:="${SEQ}" \
        bag:="${BAG_FILE}" \
        map:="${MAP_YAML}" \
        output_dir:="${RESULTS_DIR}" \
        init_x:="${INIT_X}" \
        init_y:="${INIT_Y}" \
        init_theta:="${INIT_THETA}"

    echo "==> Done: ${SEQ}"
done

echo ""
echo "All sequences complete. Results in: ${RESULTS_DIR}"
echo ""
echo "Score locally:"
echo "  python3 /LILocBench/src/evaluate.py \\"
echo "    --config-file /LILocBench/config/config_public.yaml \\"
echo "    --gt-poses-dir ${DATA_DIR} \\"
echo "    --localizer-dir ${RESULTS_DIR} \\"
echo "    --output-dir ${OUTPUT_DIR}/eval_output"
