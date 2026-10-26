#!/bin/bash
#SBATCH --job-name=isaac_array
#SBATCH --partition=compute
#SBATCH --gres=shard:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=8192M
#SBATCH --time=00:55:00
#SBATCH --signal=B:TERM@120
# One run of a run list per Slurm array task (docs/plans/2026-10-26-ws1-isaac-array.md): a task's shard, CPUs and
# memory go back the moment its run ends, so a list whose runs differ in length (S3: 21-36 min next to 184-209 min, a
# 4-slot bundle used 54 % of what it held) holds nothing for the long ones. Each task runs line SLURM_ARRAY_TASK_ID
# (0-based, blank lines not counted) of the list with the bundle runner (tools/isaac_bundle.py --slots 1): its SESSION_ID, port and bridge
# cleanup, infrastructure retry, stall watchdog with diagnostics, time-limit fit and records stay; what a bundle adds
# on top -- start stagger, VRAM gate across sessions, pace back-off, one shared page cache -- does not apply. A task is
# sized as a single job (sizes --array): with 4 CPUs the runner's --bridge-cpus auto puts the SLAM bridge on a core of
# its own; at 2 CPUs (one core shared by session and bridge) the 11590 pilot lost 3 of 3 runs to slam_link_failed.
#
#   sbatch $(python3 tools/isaac_bundle.py sizes KIND --array --runs N --run-min M) \
#     deploy/slurm/isaac_array.sbatch RUNS.jsonl OUT KIND
#   (sizes prints --array=0-(N-1)%K -- K the tasks at once, live 5 riib 3 -- and one task's --gres/--cpus-per-task/--mem/
#   --time; both arms of a comparison go in one list and one array)
#   BUNDLE_ARGS="--run-est-min M" (the same M; more options as for isaac_bundle.sbatch). M is the batch's own longest
#   normal run: one kind's runs differ by hours (S3: 21-209 min). Afterwards: isaac_bundle.py collect OUT --expect N.
#
# Records: OUT/task_<i>/run.jsonl (the line), OUT/task_<i>/bundle/{results.jsonl,summary.json,runs/}. A task asked for
# again needs its OUT/task_<i>/bundle moved aside first (the runner refuses an output that holds results).
#
# Run from the repository root.
set -u
LIST=$(realpath "${1:?the run list (JSON lines)}")
OUT=$(realpath -m "${2:?the output directory}")
KIND_ARGS=()
[ -n "${3:-}" ] && KIND_ARGS=(--kind "$3")
I=${SLURM_ARRAY_TASK_ID:?an array task: submit with --array}
TASK="$OUT/task_$I"
ROOT=$(pwd)
mkdir -p "$TASK"
# the I-th non-empty line, as load_runs counts them (it skips blank lines)
grep -v '^[[:space:]]*$' "$LIST" | sed -n "$((I + 1))p" > "$TASK/run.jsonl"
if [ ! -s "$TASK/run.jsonl" ]; then
  echo "no line $I in $LIST" >&2
  exit 2
fi
echo "host=$(hostname) job=${SLURM_JOB_ID:-} array=${SLURM_ARRAY_JOB_ID:-}_$I shards=${SLURM_SHARDS_ON_NODE:-?}" \
  "node_gpu=${SLURM_JOB_GPUS:-?} cpus=$(taskset -cp $$ | awk -F': ' '{print $2}') list=$LIST out=$TASK $(date -Is)"
# shellcheck disable=SC2086 -- BUNDLE_ARGS is a list of words by design
# a GPU that stays full while this task runs nothing is held by someone else: give up after GPU_FULL_GIVE_UP_MIN
# (status gpu_full, resubmit the task) instead of holding the shard, CPUs and memory until time_left
"${BUNDLE_PY:-python3}" "$ROOT/tools/isaac_bundle.py" --list "$TASK/run.jsonl" --out "$TASK/bundle" --slots 1 \
  --gpu-full-give-up-min "${GPU_FULL_GIVE_UP_MIN:-10}" "${KIND_ARGS[@]}" ${BUNDLE_ARGS:-} &
RUNNER=$!
trap 'kill -TERM "$RUNNER" 2>/dev/null' TERM INT  # Slurm's TERM: the runner ends its run
wait "$RUNNER"
rc=$?
if kill -0 "$RUNNER" 2>/dev/null; then  # the wait was cut by the trap: wait for the runner's own end
  wait "$RUNNER"
  rc=$?
fi
echo "task exit $rc $(date -Is)"
exit "$rc"
