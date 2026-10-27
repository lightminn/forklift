#!/bin/bash
#SBATCH --job-name=slam_cost
#SBATCH --partition=compute
#SBATCH --cpus-per-task=2
#SBATCH --mem=1G
#SBATCH --time=00:30:00
# (sizes from sacct, 68 jobs to 2026-10-26: MaxRSS <= 0.23 GB, longest 22.7 min; the slam_toolbox container runs
#  outside the job's cgroup. A paced long replay asks for its own --time: the wall budget is read from squeue.)
# slam_link_failed plan E1 (docs/plans/2026-10-25-slam-link-failed.md 3): a run's recorded scans replayed into a fresh
# mapping bridge (tools/slam_remap_drive.py) with a params variant, slam_toolbox on one CPU of the job and the driver
# on the other, then tools/slam_cost_summary.py. Records the CPU state (the SLAM CPU's SMT siblings, node load, the
# container's cpu.pressure) beside the result.
#   sbatch deploy/slurm/slam_cost_replay.sbatch OUT RUN_DIR "key=value ..." [driver args, e.g. --noise-seed 1
#       --allow-unanswered-tail 8 --pace-live --splice-to-sim-s 1500]
#   LIVE_EPOCH (environment): the live run's epoch folder, for the summary's validity check
#   SLAM_NODE=sync|lifelong (environment, default sync): the slam_toolbox node; LIFELONG_PARAMS=1 adds the lifelong
#   keys at the deployed mapper_params_lifelong.yaml values (slam-bounded-graph plan candidate B); DUMP_GRAPH=1
#   serializes and dumps the final graph (nodes, edges) for the connectivity check; BASELINE (environment): a baseline
#   replay's drive.jsonl of the same input for the excess-error summary. The node's effective parameters are always
#   saved (bridge/params_effective.yaml).
#   SLAM_NODE=localization: MAP_BUNDLE (a bundle folder, tools/slam_replay_bundle.py), MAP_ID, MAP_START_POSE "x,y,yaw"
#   and LOC_SETS "key=value ..." (applied to the localization params) -- candidate A; no graph dump (localization
#   refuses serialize_map).
#   A paced replay runs at the live wall rate (0.4 live factor: sim 1,500 s ~ 63 min) -- give it --time; the driver
#   refuses a plan longer than the job's remaining time less 5 min (--wall-budget-s).
# Run from the repository root (a snapshot); docker image forklift/ros2-dev:jazzy.
set -u
OUT=$(realpath -m "$1"); RUN=$(realpath "$2"); SETS=$3; shift 3
ROOT=$(pwd)
PY=${VENV_PY:-/home/projects/forklift/.venv/bin/python}
if [ -e "$OUT" ] && [ -n "$(ls -A "$OUT" 2>/dev/null)" ]; then echo "output $OUT is not empty"; exit 2; fi
mkdir -p "$OUT/bridge" "$OUT/sock"
echo '{"state": "pending"}' > "$OUT/status.json"
JOB=${SLURM_JOB_ID:-local$$}
CPUS=$(taskset -cp $$ | awk -F': ' '{print $2}')
CPU0=$(echo "$CPUS" | cut -d, -f1 | cut -d- -f1)
DRV_CPU=$(echo "$CPUS" | tr ',' '\n' | sed 's/-/\n/' | tail -1)
YAML=$OUT/slam.yaml
cp "$ROOT/ros2/src/forklift_bringup/config/slam_toolbox_isaac_online.yaml" "$YAML"
if [ "${LIFELONG_PARAMS:-0}" = 1 ]; then
  # mapper_params_lifelong.yaml 22-29 (slam_toolbox 2.8.5): set explicitly, the C++ defaults differ
  cat >> "$YAML" <<'EOF'
    lifelong_search_use_tree: false
    lifelong_minimum_score: 0.1
    lifelong_iou_match: 0.85
    lifelong_node_removal_score: 0.04
    lifelong_overlap_score_scale: 0.06
    lifelong_constraint_multiplier: 0.08
    lifelong_nearby_penalty: 0.001
    lifelong_candidates_scale: 0.03
EOF
fi
for kv in $SETS; do
  k=${kv%%=*}; v=${kv#*=}
  grep -qE "^    $k:" "$YAML" || { echo "unknown param $k"; exit 2; }
  sed -i "s/^    $k: .*/    $k: $v/" "$YAML"
done
NAME=slam_cost_${JOB}
trap 'docker rm -f "$NAME" >/dev/null 2>&1' EXIT
{
  echo "job=$JOB run=$RUN sets=[$SETS] args=[$*] cpus=$CPUS slam_cpu=$CPU0 driver_cpu=$DRV_CPU"
  echo "slam_cpu_siblings=$(cat /sys/devices/system/cpu/cpu$CPU0/topology/thread_siblings_list 2>/dev/null)"
  echo "load_start=$(cut -d' ' -f1-3 /proc/loadavg) start=$(date -Is)"
} > "$OUT/env.txt"
LOC_DOCKER=()
if [ "${SLAM_NODE:-sync}" = localization ]; then
  LOC_DOCKER=(-v "$(realpath "${MAP_BUNDLE:?the bundle}"):/bundle:ro" -e SLAM_EXPECTED_MAP_ID="${MAP_ID:?the map id}"
              -e SLAM_MAP_START_POSE="${MAP_START_POSE:?the start pose}" -e LOC_SETS="${LOC_SETS:-}")
fi
docker run -d --name "$NAME" --network none --cpuset-cpus "$CPU0" --memory 4g --group-add 2000 "${LOC_DOCKER[@]}" \
  -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$ROOT:/workspace:ro" -v "$YAML:/params.yaml:ro" -v "$OUT/bridge:/out" \
  -v "$OUT/sock:/run/slam" forklift/ros2-dev:jazzy bash /workspace/tools/slam_cost_container.sh /run/slam/slam.sock /out /params.yaml \
  "${SLAM_NODE:-sync}" "$([ "${DUMP_GRAPH:-0}" = 1 ] && echo graph)" >/dev/null
CID=$(docker inspect -f '{{.Id}}' "$NAME" 2>/dev/null)
LEFT=$(squeue -h -j "$JOB" -o %L 2>/dev/null)
secs() { local d=0 h=0 m=0 s=0 t=$1; [[ $t == *-* ]] && { d=${t%%-*}; t=${t#*-}; }
  IFS=: read -r -a p <<< "$t"; case ${#p[@]} in 3) h=${p[0]}; m=${p[1]}; s=${p[2]};; 2) m=${p[0]}; s=${p[1]};; 1) s=${p[0]};; esac
  echo $(( 10#$d * 86400 + 10#$h * 3600 + 10#$m * 60 + 10#$s )); }
BUDGET=()
[ -n "$LEFT" ] && [ "$LEFT" != "UNLIMITED" ] && BUDGET=(--wall-budget-s $(( $(secs "$LEFT") - 300 )))
taskset -c "$DRV_CPU" "$PY" "$ROOT/tools/slam_remap_drive.py" "$RUN" "$OUT/sock/slam.sock" "$OUT/drive.jsonl" "${BUDGET[@]}" "$@" \
  > "$OUT/drive.log" 2>&1
DRV=$?
{
  echo "load_end=$(cut -d' ' -f1-3 /proc/loadavg) end=$(date -Is)"
  echo "cpu_pressure=$(cat /sys/fs/cgroup/system.slice/docker-$CID.scope/cpu.pressure 2>/dev/null | tr '\n' ' ')"
} >> "$OUT/env.txt"
# the container ends by itself once the bridge returns and the dump is written (up to 10 min with a graph dump)
CON=$(timeout 600 docker wait "$NAME")
[ -n "$CON" ] || { docker stop -t 20 "$NAME" >/dev/null 2>&1; CON=$(timeout 60 docker wait "$NAME"); }
docker logs "$NAME" > "$OUT/container.log" 2>&1
SUMMARY_ARGS=()
[ -n "${LIVE_EPOCH:-}" ] && SUMMARY_ARGS+=(--live-epoch "$LIVE_EPOCH")
[ -n "${BASELINE:-}" ] && SUMMARY_ARGS+=(--baseline "$BASELINE")
[ "${DUMP_GRAPH:-0}" = 1 ] && SUMMARY_ARGS+=(--graph-dump "$OUT/bridge/dump")
"$PY" "$ROOT/tools/slam_cost_summary.py" "$OUT/drive.jsonl" "${SUMMARY_ARGS[@]}" > "$OUT/summary.json"
SUM=$?
SENT=$(grep -oE "^sent [0-9]+" "$OUT/drive.log" | cut -d' ' -f2)
printf '{"state": "done", "driver_exit": %s, "container_exit": %s, "summary_exit": %s, "sent": %s}\n' \
  "$DRV" "${CON:-null}" "$SUM" "${SENT:-null}" > "$OUT/status.json"
cat "$OUT/status.json"
