#!/bin/bash

map="https://www.ipb.uni-bonn.de/html/projects/localization_benchmark/data/map_office.zip"
inidividual_files_url="https://www.ipb.uni-bonn.de/html/projects/localization_benchmark/data/data_files_zipped/" # .zip
bags_w_cams_url="https://www.ipb.uni-bonn.de/html/projects/localization_benchmark/data/bagfiles/" # .bag
bags_wo_cams_url="https://www.ipb.uni-bonn.de/html/projects/localization_benchmark/data/bagfiles_no_cams/" # .bag
gt_public_url="https://www.ipb.uni-bonn.de/html/projects/localization_benchmark/data/gt_poses_public/" # /gt_poses.txt
# gt_private_url="https://www.ipb.uni-bonn.de/html/projects/localization_benchmark/data/gt_poses_private/mapping/gt_poses.txt"

train_sequences=(
	"mapping"
	"static_0"
	"dynamics_0"
	"lt_changes_0"
	"lt_changes_dynamics_0"
)

test_sequences=(
	"static_1"
	"static_2"
	"static_3"
	"static_4"
	"static_5"
	"static_6"
	"dynamics_1"
	"dynamics_2"
	"dynamics_3"
	"dynamics_4"
	"lt_changes_1"
	"lt_changes_2"
	"lt_changes_3"
	"lt_changes_4"
	"lt_changes_5"
	"lt_changes_dynamics_1"
)

echo "Pulling map..."
mkdir -p map_office && cd map_office
wget ${map} && unzip map_office.zip && rm map_office.zip
cd ..
echo "Pulled map."
echo "---------------------------------------------------------"

echo "Pulling data..."
for s in "${train_sequences[@]}"; do
    echo "---------------------------------------------------------"
    mkdir -p ${s} && cd ${s}
    wget "${inidividual_files_url}${s}.zip" && unzip ${s}.zip && rm ${s}.zip
    # wget "${bags_w_cams_url}${s}.bag"
    wget -O "${s}_no_cams.bag" "${bags_wo_cams_url}${s}.bag"
    wget "${gt_public_url}${s}/gt_poses.txt"
    cd ..
done

echo "---------------------------------------------------------"
echo "Finished pulling data."
