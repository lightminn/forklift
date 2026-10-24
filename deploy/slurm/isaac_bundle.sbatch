#!/bin/bash
#SBATCH --job-name=isaac_bundle
#SBATCH --partition=compute
#SBATCH --gres=shard:4
#SBATCH --cpus-per-task=8
#SBATCH --mem=27G
#SBATCH --time=02:00:00
#SBATCH --signal=B:TERM@120
# Several Isaac runs in one job (tools/isaac_bundle.py, docs/plans/2026-10-24-ws1-isaac-bundle.md): a run list of
# JSON lines ({"name", "argv", "env", "cwd"}), as many runs at once as the job has shards, starts 60 s apart, each
# run its own process group, log and exit code; one failing run leaves the others running. Each run gets SESSION_ID,
# BRIDGE_CPU and PORT (deploy/console/isaac_live.sbatch and deploy/riibotics/riib_scene.sbatch name their bridge
# container and pin it with them), the list's env overriding.
#
#   sbatch $(python3 tools/isaac_bundle.py sizes live|riib|bd [--slots N]) --time=T deploy/slurm/isaac_bundle.sbatch RUNS.jsonl OUT
#   (sizes prints --gres/--cpus-per-task/--mem from the measured per-session RAM, cores and GPU memory, in proportion
#   to the sessions; riib holds 3 per GPU -- 4.15-4.37 GB each with its video)
#   optional: BUNDLE_PY (python3; the runner needs only the standard library), BUNDLE_ARGS (more tools/isaac_bundle.py
#   options, split on spaces: --stagger-s, --run-timeout-s, --slots, --session-vram-mib ...)
#
# Sizes per kind of run, from ws1 measurements (2026-10-24, the plan's section 1 and 2.4): shard = runs at once,
#   live session with learned proposals  4 runs: shard:4, 8 CPUs, 27G (1.0-1.2 cores, <= 6.51 GiB, 3.9 GB GPU each);
#                                        BUNDLE_ARGS="--pace-floor 0.345 --pace-gap-p99-s 0.12" with a pace_log per run
#   batch riib_scene-like run            4 runs: shard:4, 8 CPUs, 28G (3.1 cores of 4 alone -- slower at 2, <= 6.8 GiB)
# Four sessions fill a GPU (4 x 3,915 MiB of 16,303); the runner's device free-memory check holds a start that would
# not fit. Long operator sessions (isaac_live.sbatch's 8 h) stay single jobs: the memory figures cover runs up to 70 min.
# --time: the list's runs in waves of N slots plus a minute of stagger per run, with room. The batch shell gets TERM
# 120 s before the limit and passes it to the runner, which TERMs every run and KILLs what is left after 110 s.
#
# Run from the repository root.
set -u
LIST=$(realpath "${1:?the run list (JSON lines)}")
OUT=$(realpath -m "${2:?the output directory}")
ROOT=$(pwd)
mkdir -p "$OUT"
echo "host=$(hostname) job=${SLURM_JOB_ID:-} shards=${SLURM_SHARDS_ON_NODE:-?} gpus=${CUDA_VISIBLE_DEVICES:-unset}" \
  "cpus=$(taskset -cp $$ | awk -F': ' '{print $2}') list=$LIST out=$OUT $(date -Is)"
nvidia-smi --query-gpu=index,memory.used,memory.total --format=csv 2>/dev/null
# shellcheck disable=SC2086 -- BUNDLE_ARGS is a list of words by design
"${BUNDLE_PY:-python3}" "$ROOT/tools/isaac_bundle.py" --list "$LIST" --out "$OUT" ${BUNDLE_ARGS:-} &
RUNNER=$!
trap 'kill -TERM "$RUNNER" 2>/dev/null' TERM INT  # Slurm's TERM: the runner ends its runs
wait "$RUNNER"
rc=$?
if kill -0 "$RUNNER" 2>/dev/null; then  # the wait was cut by the trap: wait for the runner's own end
  wait "$RUNNER"
  rc=$?
fi
echo "bundle exit $rc $(date -Is)"
exit "$rc"
