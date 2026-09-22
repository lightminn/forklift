#!/usr/bin/env bash
# Run a batch of measurement commands on the school HPC (gate1.hpc) instead of locally.
#
#   tools/hpc_sweep.sh jobs.tsv [partition] [cores]
#
# jobs.tsv: one job per line, "<name><TAB><command>".  The command runs from the
# repository root with PYTHONPATH=src:. and `python` bound to the remote venv.
# Results land in artifacts/hpc/<run-id>/out/<name>.txt locally.
#
# Why CPU and not GPU/JAX: every figure in docs/decisions/0003 must reproduce the
# *committed* detector bit for bit, so the numerics may not change.  The parallelism
# is across grid cells and seeds, which is embarrassingly parallel on CPU.  See
# ~/ENV.md ("학교 HPC") for the cluster's partitions and the account's limits.
set -euo pipefail

JOBS=${1:?usage: hpc_sweep.sh jobs.tsv [partition] [cores]}
PARTITION=${2:-cpu1}
CORES=${3:-48}
HOST=gate1_External
REMOTE=forklift
RUN_ID=$(date -u +%Y%m%dT%H%M%SZ)
LOCAL_OUT="artifacts/hpc/$RUN_ID"

[ -s "$JOBS" ] || { echo "empty job file: $JOBS" >&2; exit 1; }
mkdir -p "$LOCAL_OUT"

echo "[1/4] sync repo -> $HOST:~/$REMOTE"
rsync -az --exclude='.git/' --exclude='artifacts/' --exclude='data/' \
  --exclude='__pycache__/' --exclude='*.pyc' --exclude='.pytest_cache/' \
  --exclude='presentation' --exclude='ros2/build/' --exclude='ros2/install/' \
  --exclude='ros2/log/' --exclude='.venv/' --exclude='build/' \
  ./ "$HOST:$REMOTE/"
rsync -az "$JOBS" "$HOST:$REMOTE/hpc-jobs-$RUN_ID.tsv"

echo "[2/4] submit $(awk 'NF && $0 !~ /^#/' "$JOBS" | wc -l) jobs -> $PARTITION, $CORES cores"
JOB_ID=$(ssh -o BatchMode=yes "$HOST" \
  "mkdir -p '$REMOTE/hpc/$RUN_ID' \
   && mv '$REMOTE/hpc-jobs-$RUN_ID.tsv' '$REMOTE/hpc/$RUN_ID/jobs.tsv' \
   && cd '$REMOTE' \
   && sbatch --parsable --partition='$PARTITION' --cpus-per-task='$CORES' \
        --output='hpc/$RUN_ID/slurm-%j.log' \
        --export=ALL,RUN_ID='$RUN_ID',CORES='$CORES' \
        deploy/hpc/run_sweep.sbatch")
echo "    slurm job $JOB_ID   run-id $RUN_ID"

echo "[3/4] wait for $JOB_ID"
ssh -o BatchMode=yes "$HOST" \
  "while squeue -h -j $JOB_ID -o %T 2>/dev/null | grep -qE 'PENDING|RUNNING|CONFIGURING|COMPLETING'; do sleep 20; done"

echo "[4/4] fetch -> $LOCAL_OUT"
rsync -az "$HOST:$REMOTE/hpc/$RUN_ID/" "$LOCAL_OUT/"
echo "results in $LOCAL_OUT/out/"
ls -1 "$LOCAL_OUT/out/" 2>/dev/null || echo "(no output — check $LOCAL_OUT/slurm-*.log)"
