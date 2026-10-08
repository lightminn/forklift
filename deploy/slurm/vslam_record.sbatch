#!/bin/bash
#SBATCH --job-name=vslam_record
#SBATCH --partition=compute
#SBATCH --gres=shard:1
#SBATCH --qos=admin
#SBATCH --cpus-per-task=8
#SBATCH --mem=16G
#SBATCH --time=04:00:00
# Isaac survey recording with the RGB-D rig (docs/plans/2026-10-07-visual-slam-and-fusion.md).
# Usage: EXP=<experiment dir> sbatch --output=<log> vslam_record.sbatch <name> <seed> [run_slam_drive args...]
# EXP holds source/ (the snapshot) and env/ (venv: editable core + the shared Isaac runtime).
set -uo pipefail
EXP=${EXP:?set EXP to the experiment directory}
NAME=$1; SEED=$2; shift 2
export OMNI_KIT_ACCEPT_EULA=YES OMP_NUM_THREADS=6
cd "$EXP/source"
mkdir -p "$EXP/records"
nvidia-smi --query-gpu=name,driver_version,memory.total,memory.used --format=csv
echo "=== $NAME seed $SEED start $(date -Is)"
timeout 12000 "$EXP/env/bin/python" sim/isaac/run_slam_drive.py \
  --base-scene /home/projects/forklift/artifacts/base_scene_warehouse_forklift/scene.usda \
  --forklift-urdf sim/models/dls08_provisional/forklift.urdf \
  --output "$EXP/records/$NAME" --seed "$SEED" \
  --depth-rig config/isaac_depth_rig.yaml "$@" \
  --portable-root /home/projects/forklift/cache/isaac_v2 > "$EXP/records/$NAME.log" 2>&1
STATUS=$?
echo "=== $NAME seed $SEED exit $STATUS $(date -Is)"
exit $STATUS
