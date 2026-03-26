XAUTHORITY=/dev/null docker compose run --rm vpr scripts/vpr/extract_keyframes.py \
      --data-dir /data/mapping \
      --output /models/vpr/keyframes.pkl

---

  1. Update .env

  cd /home/matthew/Desktop/LILocBench/ADAPT-MCL/docker
  sed -i 's|BAGS_DIR=.*|BAGS_DIR=/home/matthew/Desktop/LILocBench/data|' .env
  sed -i 's|DATA_DIR=.*|DATA_DIR=/home/matthew/Desktop/LILocBench/data|' .env
  cat .env  # verify it looks right

  2. Build VPR container

  cd /home/matthew/Desktop/LILocBench/ADAPT-MCL/docker
  XAUTHORITY=/dev/null docker compose build vpr

  3. Train R18 multi-camera

  XAUTHORITY=/dev/null docker compose run --rm vpr scripts/vpr/finetune_mixvpr_multicam.py \
      --epochs 20 --batch-size 96 --lr 5e-5 --margin 0.3 \
      --backbone resnet18 --out-channels 256 --out-rows 4 --mix-depth 4

  4. Train R50 multi-camera (full backbone, use that 24 GB)

  XAUTHORITY=/dev/null docker compose run --rm vpr scripts/vpr/finetune_mixvpr_multicam.py \
      --epochs 20 --batch-size 32 --lr 3e-5 --margin 0.3 \
      --backbone resnet50 --layers-to-freeze 0 --out-channels 256 --out-rows 4 --mix-depth 4 \
      --output /models/vpr/mixvpr_r50_multicam_finetuned.pth

  5. Build databases (one per model)

  # R18 multi-cam database
  XAUTHORITY=/dev/null docker compose run --rm vpr scripts/vpr/build_database_mixvpr_multicam.py \
      --weights /models/vpr/mixvpr_multicam_finetuned.pth

  # R50 multi-cam database
  XAUTHORITY=/dev/null docker compose run --rm vpr scripts/vpr/build_database_mixvpr_multicam.py \
      --weights /models/vpr/mixvpr_r50_multicam_finetuned.pth \
      --backbone resnet50 \
      --output-db /models/vpr/database_mixvpr_r50_multicam.pkl \
      --output-index /models/vpr/place_index_mixvpr_r50_multicam.faiss

  6. Generate candidates for all sequences (can do on the remote too)

for seq in static_0 dynamics_0 lt_changes_0 lt_changes_dynamics_0; do
      XAUTHORITY=/dev/null docker compose run --rm vpr scripts/vpr/publish_vpr_candidates_multicam.py \
          --sequence $seq --top-k 10 \
          --output-file /data/$seq/vpr_candidates_mixvpr_r18_multicam.txt
done


for seq in static_0 dynamics_0 lt_changes_0 lt_changes_dynamics_0; do
      XAUTHORITY=/dev/null docker compose run --rm vpr scripts/vpr/publish_vpr_candidates_multicam.py \
          --sequence $seq --top-k 10 \
          --weights /models/vpr/mixvpr_r50_multicam_finetuned.pth \
          --backbone resnet50 \
          --database /models/vpr/database_mixvpr_r50_multicam.pkl \
          --index /models/vpr/place_index_mixvpr_r50_multicam.faiss \
          --output-file /data/$seq/vpr_candidates_mixvpr_r50_multicam.txt
done

  7. Copy results back here

  # From the remote, scp back:
  #   models/vpr/mixvpr_multicam_finetuned.pth
  #   models/vpr/mixvpr_r50_multicam_finetuned.pth
  #   models/vpr/database_mixvpr_multicam.pkl
  #   models/vpr/place_index_mixvpr_multicam.faiss
  #   models/vpr/database_mixvpr_r50_multicam.pkl
  #   models/vpr/place_index_mixvpr_r50_multicam.faiss
  #   data/*/vpr_candidates_multicam.txt

  Steps 3 and 4 can run back-to-back. Step 5 needs the weights from the respective training run. Step 6 needs the database from step 5. Let me know what the build output
  looks like — the Docker build is the most likely place for issues.

---

  EigenPlaces multi-camera pipeline:

  # 1. Train EigenPlaces multicam
  XAUTHORITY=/dev/null docker compose run --rm vpr scripts/vpr/finetune_eigenplaces_multicam.py \
      --epochs 15 --batch-size 160 --lr 1e-5 --margin 0.3

  # 2. Build database
  XAUTHORITY=/dev/null docker compose run --rm vpr scripts/vpr/build_database_eigenplaces_multicam.py \
      --weights /models/vpr/eigenplaces_multicam_finetuned.pth

  # 3. Generate candidates
  for seq in static_0 dynamics_0 lt_changes_0 lt_changes_dynamics_0; do
      XAUTHORITY=/dev/null docker compose run --rm vpr scripts/vpr/publish_vpr_candidates_eigenplaces_multicam.py \
      --sequence $seq --top-k 10 \
      --output-file /data/$seq/vpr_candidates_eigenplaces_multicam.txt
  done

  MegaLoc multi-camera pipeline:
  # 1. Train MegaLoc multicam (smaller batch due to DINOv2 backbone)
  XAUTHORITY=/dev/null docker compose run --rm vpr scripts/vpr/finetune_megaloc_multicam.py \
      --epochs 15 --batch-size 24 --lr 1e-5 --margin 0.3

  # 2. Build database
  XAUTHORITY=/dev/null docker compose run --rm vpr scripts/vpr/build_database_megaloc_multicam.py \
      --weights /models/vpr/megaloc_multicam_finetuned.pth

  # 3. Generate candidates
  for seq in static_0 dynamics_0 lt_changes_0 lt_changes_dynamics_0; do
      XAUTHORITY=/dev/null docker compose run --rm vpr scripts/vpr/publish_vpr_candidates_megaloc_multicam.py \
          --sequence $seq --top-k 10 \
          --output-file /data/$seq/vpr_candidates_megaloc_multicam.txt
  done

  Files to scp back afterwards:
  - models/vpr/eigenplaces_multicam_finetuned.pth
  - models/vpr/database_eigenplaces_multicam.pkl
  - models/vpr/place_index_eigenplaces_multicam.faiss
  - models/vpr/megaloc_multicam_finetuned.pth
  - models/vpr/database_megaloc_multicam.pkl
  - models/vpr/place_index_megaloc_multicam.faiss
  - data/*/vpr_candidates_eigenplaces_multicam.txt
  - data/*/vpr_candidates_megaloc_multicam.txt

  EigenPlaces and MegaLoc can run back-to-back. MegaLoc needs the smaller batch size (8) since DINOv2 is much larger than ResNet50, but with 24GB you should be fine.
