#!/bin/bash
#SBATCH --job-name=recog-2a-eval
#SBATCH --partition=compute
#SBATCH --cpus-per-task=8
#SBATCH --mem=2G
#SBATCH --time=00:10:00
#SBATCH --output=%x-%j.log
# Stage 2A dev evaluation: oracle-fit and baseline on the stage 1B Isaac datasets.
# Usage: sbatch --chdir=<run dir> deploy/slurm/recognition_stage2a_evaluate.sbatch <source dir> <isaac run dir> <manifest>
# (1B evaluation job 2876: 8 processes, MaxRSS 1.8 GB, 6 min -> 8G and 30 min with headroom.)
# Every process is waited for by PID and must leave a metrics.json: a bare `wait` hides failures.
set -uo pipefail
src=$1; run=$2; manifest=$3
PY=/home/projects/forklift/.venv/bin/python
export PYTHONPATH="$src/src:$src" PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
pids=(); names=()
for ds in clean low mid high; do
  for det in oracle-fit baseline; do
    $PY "$src/tools/evaluate_recognition.py" --dataset "$run/isaac_$ds" --split dev --detector "$det" \
      --output "out/${ds}_${det}" --diagnostics --source-manifest "$manifest" > "out_${ds}_${det}.log" 2>&1 &
    pids+=($!); names+=("${ds}_${det}")
  done
done
rc=0
for i in "${!pids[@]}"; do
  if ! wait "${pids[$i]}"; then echo "FAILED ${names[$i]}"; rc=1; fi
  [ -s "out/${names[$i]}/metrics.json" ] || { echo "MISSING out/${names[$i]}/metrics.json"; rc=1; }
done
echo "ALL_DONE rc=$rc"
exit $rc
