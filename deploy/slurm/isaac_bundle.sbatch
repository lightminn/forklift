#!/bin/bash
#SBATCH --job-name=isaac_bundle
#SBATCH --partition=compute
#SBATCH --gres=shard:4
#SBATCH --cpus-per-task=8
#SBATCH --mem=24G
#SBATCH --time=02:00:00
#SBATCH --signal=B:TERM@120
# Several Isaac runs in one job (tools/isaac_bundle.py, docs/plans/2026-10-24-ws1-isaac-bundle.md): a run list of
# JSON lines ({"name", "argv", "env", "cwd"}), as many runs at once as the job has shards, starts 60 s apart, each
# run its own process group, log and exit code; one failing run leaves the others running. Each run gets SESSION_ID,
# BRIDGE_CPU and PORT (deploy/console/isaac_live.sbatch and deploy/riibotics/riib_scene.sbatch name their bridge
# container and pin it with them), the list's env overriding.
#
#   sbatch $(python3 tools/isaac_bundle.py sizes KIND [--slots N] --runs R --jobs J [--run-min M]) \
#     deploy/slurm/isaac_bundle.sbatch RUNS.jsonl OUT KIND
#   (KIND = live|riib|bd: the runner takes the slots and the session GPU memory from it; without it, slots = shards)
#   (sizes prints --gres/--cpus-per-task/--mem from the measured per-session RAM, cores and GPU memory, in proportion
#   to the sessions; riib holds 3 per GPU -- 4.15-4.37 GB each with its video)
#   optional: BUNDLE_PY (python3; the runner needs only the standard library), BUNDLE_ARGS (more tools/isaac_bundle.py
#   options, split on spaces: --stagger-s, --run-timeout-s, --slots, --session-vram-mib ...)
#
# The list (before submitting): python3 tools/isaac_bundle.py check --list RUNS --runner SNAP/sim/isaac/run_transport.py
#   --kind KIND --jobs J. Two or more jobs on one list share BUNDLE_ARGS="--claim-dir OUT/claims" (the runner refuses
#   a second job on a list without the same claim dir). Put the longest runs first, and make the number of runs a
#   multiple of J x the kind's slots (live 4, riib 3 per job; the node holds 8 live or 6 riib): a last wave of half the
#   slots or fewer holds the other GPUs idle for about a run, and check warns about it.
#
# Sizes per kind of run, from ws1 measurements (2026-10-24, the plan's section 1 and 2.4): shard = runs at once,
#   live session with learned proposals  4 runs: shard:4, 8 CPUs, 24G (1.0-1.2 cores, anon ~5.45 GiB, 3.9 GB GPU each);
#                                        BUNDLE_ARGS="--pace-floor 0.335 --pace-gap-p99-s 0.12" with a pace_log per run
#                                        (judged from 540 s: a session runs 0.22-0.33 for its first 4 min)
#   batch riib_scene-like run            3 runs: shard:4, 6 CPUs, 20G (4.15-4.47 GB GPU each with its video)
#   (isaac_bundle.py sizes prints these; the figures above it are the source)
# Four sessions fill a GPU (4 x 3,915 MiB of 16,303); the runner's device free-memory check holds a start that would
# not fit. Long operator sessions (isaac_live.sbatch's 8 h) stay single jobs: the memory figures cover runs up to 70 min.
# --time (sizes --runs prints it): the waves the list's jobs make together x a run's p95 (the kind's measured, or
# --run-min M -- then give the runner the same: BUNDLE_ARGS="... --run-est-min M") + a wave's stagger + 5 min, +15 %.
# The runner starts no run with less than its p95 (+ the early TERM and grace) left in the job: a first attempt is
# recorded time_left and left unclaimed for another job on the list, a retry of its own time_left_retry (exit 1 either
# way: check summary.json for runs nobody ran). The batch shell
# gets TERM 120 s before the limit and passes it to the runner, which TERMs every run and KILLs what is left after
# 110 s.
#
# Run from the repository root.
set -u
LIST=$(realpath "${1:?the run list (JSON lines)}")
OUT=$(realpath -m "${2:?the output directory}")
KIND_ARGS=()
[ -n "${3:-}" ] && KIND_ARGS=(--kind "$3")
ROOT=$(pwd)
mkdir -p "$OUT"
echo "host=$(hostname) job=${SLURM_JOB_ID:-} shards=${SLURM_SHARDS_ON_NODE:-?} gpus=${CUDA_VISIBLE_DEVICES:-unset}" \
  "cpus=$(taskset -cp $$ | awk -F': ' '{print $2}') list=$LIST out=$OUT $(date -Is)"
nvidia-smi --query-gpu=index,memory.used,memory.total --format=csv 2>/dev/null
# shellcheck disable=SC2086 -- BUNDLE_ARGS is a list of words by design
"${BUNDLE_PY:-python3}" "$ROOT/tools/isaac_bundle.py" --list "$LIST" --out "$OUT" "${KIND_ARGS[@]}" ${BUNDLE_ARGS:-} &
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
