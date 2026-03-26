#!/bin/bash
# Run all VPR model × sequence combinations and evaluate
set -e

EVAL=/home/matthew/Desktop/LILocBench/lilocbench_ws/src/lilocbench_ros/scripts/eval.py
DATA=/home/matthew/Desktop/LILocBench/data
OUTPUT_BASE=/home/matthew/Desktop/LILocBench/lilocbench_ws/docker/output

MODELS=(
    "mixvpr_r18:vpr_candidates_mixvpr_r18_multicam.txt"
    "mixvpr_r50:vpr_candidates_mixvpr_r50_multicam.txt"
    "eigenplaces:vpr_candidates_eigenplaces_multicam.txt"
    "megaloc:vpr_candidates_megaloc_multicam.txt"
)

SEQUENCES=(
    "static_0:static_0_vpr.launch:220"
    "dynamics_0:dynamics_0_vpr.launch:80"
    "lt_changes_0:lt_changes_0_vpr.launch:200"
    "lt_changes_dynamics_0:lt_changes_dynamics_0_vpr.launch:220"
)

RESULTS_FILE=/home/matthew/Desktop/LILocBench/vpr_benchmark_results.txt
echo "VPR Benchmark Results" > "$RESULTS_FILE"
echo "=====================" >> "$RESULTS_FILE"
echo "" >> "$RESULTS_FILE"

for model_entry in "${MODELS[@]}"; do
    IFS=':' read -r model_name candidates_file <<< "$model_entry"
    echo "=== Model: $model_name ==="

    for seq_entry in "${SEQUENCES[@]}"; do
        IFS=':' read -r seq launch timeout <<< "$seq_entry"
        echo "  Running $seq with $model_name..."

        output_dir="/output/results_vpr_${model_name}"
        cand_path="/data/${seq}/${candidates_file}"

        # Clean previous output
        docker exec lilocbench-dev-1 bash -c "rm -f ${output_dir}/${seq}/run_1.txt" 2>/dev/null || true

        # Run
        docker exec lilocbench-dev-1 bash -c \
            "source /catkin_ws/devel/setup.bash && \
             timeout --signal=INT ${timeout} roslaunch lilocbench_ros ${launch} \
                launch_rviz:=false \
                output_dir:=${output_dir} \
                vpr_candidates_file:=${cand_path} \
             2>&1 | tail -5"

        # Evaluate
        host_result="${OUTPUT_BASE}/results_vpr_${model_name}/${seq}/run_1.txt"
        if [ -f "$host_result" ]; then
            result=$(python3 "$EVAL" "${DATA}/${seq}/gt_poses.txt" "$host_result" 2>&1)
            echo "  $result"
            echo "${model_name} | ${seq}: $result" >> "$RESULTS_FILE"
        else
            echo "  ERROR: No result file at $host_result"
            echo "${model_name} | ${seq}: NO RESULT" >> "$RESULTS_FILE"
        fi
        echo ""
    done
done

echo "Results saved to $RESULTS_FILE"
