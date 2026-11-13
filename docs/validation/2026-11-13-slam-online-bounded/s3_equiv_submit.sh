#!/usr/bin/env bash
# Online-SLAM plan stage S1 equivalence (가) (docs/plans/2026-11-13-slam-online-bounded.md 5, docs/design/
# 2026-11-13-slam-patch-build.md 4): the S2 input (four line_m runs spliced) cut at sim 1,500 s, fast replay, graph
# dumped, through three images -- R0 the deployed one, U0 the unpatched source build, P0 the patch series with every
# switch off -- twice each, and P0 once more with the trace on (the trace must not change a reply). Serial (afterany),
# CPU 2 each, so no two replays share the node while their cost is read.
#   bash s3_equiv_submit.sh SNAP OUT P1_IMAGE [FIRST_DEPENDENCY] [-- NAME ...]
#   P1_IMAGE: forklift/ros2-slam:jazzy-2.8.5-p1-<series> (deploy/slurm/slam_patch_build.sbatch prints it)
# Judge with tools/slam_equivalence.py: r0_1 vs p0_1 (the verdict), r0_1 vs u0_1 and u0_1 vs p0_1 (the cause),
# each arm's two runs against each other, and p0_1 vs p0t.
set -euo pipefail
SNAP=$1 OUT=$2 P1_IMAGE=$3
shift 3
DEP=""
if [ $# -gt 0 ] && [ "$1" != "--" ]; then DEP=$1; shift; fi
if [ $# -gt 0 ] && [ "$1" = "--" ]; then shift; fi
L=/home/projects/forklift/test_runs/live
RUN=$L/ct2_624d0616_auto_line_m_s1/live/epoch_1/run
SPLICE=($L/ct2_624d0616_fixed_line_m_s3/live/epoch_1/run $L/ct2_624d0616_auto_line_m_s4/live/epoch_1/run
        $L/ct2_41a2a7e8_fixed_line_m_s2/live/epoch_1/run)
ARGS=(--noise-seed 1 --allow-unanswered-tail 8 --splice-to-sim-s 1500 --splice-runs "${SPLICE[@]}")
SETS="map_update_interval=1.0"
U0_IMAGE=forklift/ros2-slam:jazzy-2.8.5-u0
NAMES=("$@")
[ ${#NAMES[@]} -gt 0 ] || NAMES=(r0_1 u0_1 p0_1 r0_2 u0_2 p0_2 p0t)
docker image inspect "$U0_IMAGE" >/dev/null && docker image inspect "$P1_IMAGE" >/dev/null
cd "$SNAP"
mkdir -p "$OUT"
prev=$DEP
for NAME in "${NAMES[@]}"; do
  [ -e "$OUT/$NAME" ] && { echo "$OUT/$NAME exists"; exit 2; }
  DEPARG=()
  [ -n "$prev" ] && DEPARG=(--dependency=afterany:"$prev")
  case $NAME in
    r0_*) ENV=() ;;
    u0_*) ENV=(SLAM_IMAGE="$U0_IMAGE" SLAM_IMAGE_KIND=u0) ;;
    p0_*) ENV=(SLAM_IMAGE="$P1_IMAGE" SLAM_IMAGE_KIND=patched) ;;
    p0t) ENV=(SLAM_IMAGE="$P1_IMAGE" SLAM_IMAGE_KIND=patched SLAM_PATCH_ENV=FORKLIFT_SLAM_TRACE=/out/slam_trace.jsonl) ;;
    *) echo "unknown arm $NAME"; exit 2 ;;
  esac
  # every variable the replay reads is set here or cleared, so nothing exported by the caller runs an arm as another
  # (Codex code review 3 P2-2): the image, its kind and switches, the node and its localization inputs
  prev=$(env -u SLAM_IMAGE -u SLAM_IMAGE_KIND -u SLAM_PATCH_ENV -u LIFELONG_PARAMS -u LOC_SETS -u MAP_BUNDLE \
    -u MAP_ID -u MAP_START_POSE -u LIVE_EPOCH -u BASELINE -u SUMMARY -u BUDGET -u KEEP_MAP_SUBSCRIPTION \
    $(env | grep -oE '^FORKLIFT_SLAM_[A-Z0-9_]+' | sed 's/^/-u /') \
    SLAM_NODE=sync DUMP_GRAPH=1 "${ENV[@]}" sbatch --parsable "${DEPARG[@]}" --job-name="s3e_$NAME" \
    --output="$OUT/$NAME-%j.log" --cpus-per-task=2 --mem=1G --time=00:30:00 \
    deploy/slurm/slam_cost_replay.sbatch "$OUT/$NAME" "$RUN" "$SETS" "${ARGS[@]}")
  echo "$NAME job $prev"
done
