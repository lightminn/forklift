#!/bin/bash
# Runs every line of jobs.tsv in parallel on one node.  RUN_ID and CORES come from
# --export on the sbatch call.  One job per line: <name><TAB><command>.
#SBATCH --job-name=forklift-sweep
#SBATCH --nodes=1
set -u
cd "$SLURM_SUBMIT_DIR"
export PYTHONPATH=src:.
# keep BLAS single-threaded: the parallelism is across jobs, not inside numpy
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
PY="$HOME/forklift-venv/bin/python"
OUT="hpc/$RUN_ID/out"
mkdir -p "$OUT"

run_one() {
  local line="$1" name cmd
  name=${line%%$'\t'*}
  cmd=${line#*$'\t'}
  cmd=${cmd//python3 /"$PY" }
  cmd=${cmd//python /"$PY" }
  ( eval "$cmd" ) > "$OUT/$name.txt" 2>&1
  echo "done $name (rc=$?)"
}
export -f run_one
export OUT PY

awk 'NF && $0 !~ /^#/' "hpc/$RUN_ID/jobs.tsv" \
  | tr '\n' '\0' \
  | xargs -0 -P "${CORES:-48}" -I{} bash -c 'run_one "$1"' _ {}
echo "ALL DONE"
