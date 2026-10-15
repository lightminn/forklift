#!/bin/bash
#SBATCH --job-name=recog_isaac
#SBATCH --partition=compute
#SBATCH --gres=shard:1
#SBATCH --qos=admin
#SBATCH --cpus-per-task=4
#SBATCH --mem=6G
#SBATCH --time=01:00:00
#SBATCH --output=%x_%j.log
# Usage: sbatch --chdir=<run dir> --export=ALL,SRC=<source snapshot>,OUT=<out dir>,ARGS="<render args>" \
#        recognition_isaac_render.sbatch   (--mem and --time are overridden on the command line from measured runs)
set -euo pipefail
cd "${SRC:?}"
export OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH="$SRC/src:$SRC" PYTHONDONTWRITEBYTECODE=1
nvidia-smi --query-gpu=name,driver_version,memory.used --format=csv > "${OUT:?}.gpu_start.csv"
/home/projects/forklift/.venv/bin/python sim/isaac/render_recognition_scenes.py --out "$OUT" ${ARGS:-}
