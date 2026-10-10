#!/bin/bash
#SBATCH --job-name=world
#SBATCH --partition=compute
#SBATCH --gres=shard:1
#SBATCH --cpus-per-task=2
#SBATCH --mem=14G
# Measured 2026-10-09 (8 trucks, video): box and dressed rack/multi runs MaxRSS 5.0-7.6 GB and
# 1.5-1.8 cores busy (TotalCPU / elapsed); a dressed run was OOM-killed at 9G during start-up.
# Dressed shop varied: one run peaked at 11.36 GB under an 11G limit, one sat at the 11G limit at
# 0.4 cores busy until the 4 h wall limit, and the same code and seed rerun under 14G peaked at
# 6.2 GB in 43 min -- why is not known. 14G covers the highest seen; it is not a measured need.
#SBATCH --time=04:00:00
# Fleet on a random factory floor in Isaac (docs/plans/2026-10-09-fleet-random-worlds.md, W6).
# Usage: EXP=<experiment dir> BASE_SCENE=<measured base scene.usda> PORTABLE_ROOT=<Kit cache> \
#        sbatch --output=<log> world_run.sbatch <name> <rack|shop|multi> <seed> [run_world args...]
# EXP holds source/ (snapshot) and env/ (venv: the snapshot's src + the shared Isaac runtime).
set -uo pipefail
EXP=${EXP:?set EXP}; BASE_SCENE=${BASE_SCENE:?set BASE_SCENE}; PORTABLE_ROOT=${PORTABLE_ROOT:?set PORTABLE_ROOT}
NAME=$1; STRUCTURE=$2; SEED=$3; shift 3
mkdir -p "$EXP/runs"
export OMNI_KIT_ACCEPT_EULA=YES OMP_NUM_THREADS=2
echo "start $(date -Is) $NAME $STRUCTURE seed=$SEED $*" > "$EXP/runs/$NAME.job.txt"
SRC=$(readlink -f "$EXP/source")  # pinned now: relinking source later does not reach this run
echo "source $SRC" >> "$EXP/runs/$NAME.job.txt"
cd "$SRC"
PYTHONPATH="$SRC/src" timeout 20000 "$EXP/env/bin/python" sim/isaac/run_world.py --base-scene "$BASE_SCENE" \
  --structure "$STRUCTURE" --seed "$SEED" --output "$EXP/runs/$NAME" "$@" \
  --portable-root "$PORTABLE_ROOT" > "$EXP/runs/$NAME.log" 2>&1
STATUS=$?
echo "end $(date -Is) exit $STATUS" >> "$EXP/runs/$NAME.job.txt"
exit $STATUS
