#!/bin/bash

eval_script="src/lilocbench_ros/scripts/eval.py"
gt_data_root="../data"

sequences=(
	"mapping"
	"static_0"
	# "static_1"
	# "static_2"
	# "static_3"
	# "static_4"
	# "static_5"
	# "static_6"
	"dynamics_0"
	# "dynamics_1"
	# "dynamics_2"
	# "dynamics_3"
	# "dynamics_4"
	"lt_changes_0"
	# "lt_changes_1"
	# "lt_changes_2"
	# "lt_changes_3"
	# "lt_changes_4"
	# "lt_changes_5"
	"lt_changes_dynamics_0"
	# "lt_changes_dynamics_1"
)

echo "Starting evlauation..."

for s in "${sequences[@]}"; do
    echo "---------------------------------------------------------"
    python3 ${eval_script} ${gt_data_root}/${s}/gt_poses.txt docker/output/results/${s}/run_1.txt
done

echo "---------------------------------------------------------"
echo "Finished evaluating."
