#!/bin/bash
#SBATCH --job-name=recog-2b-eval
#SBATCH --partition=compute
#SBATCH --cpus-per-task=2
#SBATCH --mem=1G
#SBATCH --time=00:45:00
#SBATCH --output=%x-%j.log
# Stage 2B dev evaluation: oracle-fit only (the baseline detector is unchanged since 2A), two
# datasets at a time. 1B/2A evaluation processes peaked near 230 MB each (job 2876: 8 processes,
# MaxRSS 1.8 GB), so two fit in 1G, which ws1 could still schedule on 2026-10-16.
# Usage: sbatch --chdir=<run dir> deploy/slurm/recognition_stage2b_evaluate.sbatch <source dir> <isaac run dir> <manifest>
# Every process is waited for by PID and must leave a metrics.json: a bare `wait` hides failures.
set -uo pipefail
src=$1; run=$2; manifest=$3
PY=/home/projects/forklift/.venv/bin/python
export PYTHONPATH="$src/src:$src" PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
rc=0
for pair in "clean low" "mid high"; do
  pids=(); names=()
  for ds in $pair; do
    $PY "$src/tools/evaluate_recognition.py" --dataset "$run/isaac_$ds" --split dev --detector oracle-fit \
      --output "out/${ds}_oracle-fit" --diagnostics --source-manifest "$manifest" > "out_${ds}_oracle-fit.log" 2>&1 &
    pids+=($!); names+=("${ds}_oracle-fit")
  done
  for i in "${!pids[@]}"; do
    if ! wait "${pids[$i]}"; then echo "FAILED ${names[$i]}"; rc=1; fi
    [ -s "out/${names[$i]}/metrics.json" ] || { echo "MISSING out/${names[$i]}/metrics.json"; rc=1; }
  done
done
echo "ALL_DONE rc=$rc"
exit $rc
