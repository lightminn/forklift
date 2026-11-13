#!/usr/bin/env bash
# Online-SLAM plan stage S3 (docs/plans/2026-11-13-slam-online-bounded.md 5, 6; docs/design/2026-11-13-slam-patch-
# build.md 2): the C++ arms on the S2 input -- the four line_m runs spliced to sim 5,000 s, noise seed 1,
# --allow-unanswered-tail 8, map_update_interval 1.0, fast replay, graph dumped at the end. One replay per job, CPU 2,
# serial (afterany) so no two replays share the node while their cost is read. The judged runs have the trace off; the
# map measurement runs (*_mapq: a mapping bridge already keeps /map, so each is the same replay again) have it on, and
# their cost is not judged.
#   p0_1, p0_2         P0: the patch image, every switch off (its equivalence to R0 is S1 (가))
#   cwm_1, cwm_2       C-WM: FORKLIFT_SLAM_CWM_S=600
#   c600_1, c600_2     C: FORKLIFT_SLAM_DELETE_OLDER_S=600
#   c1200_1, c1200_2   C: FORKLIFT_SLAM_DELETE_OLDER_S=1200
#   s1_1, s1_2         S1: FORKLIFT_SLAM_S1=1, FORKLIFT_SLAM_S1_BUFFER=10, from S1_IMAGE (the image with patch 0004)
#   p0_mapq, cwm_mapq, c600_mapq, c1200_mapq, s1_mapq   the same with FORKLIFT_SLAM_TRACE on
#   bash s3_submit.sh SNAP OUT P1_IMAGE [FIRST_DEPENDENCY] [-- NAME ...]      (S1_IMAGE in the environment for s1_*)
# The image of an arm and its P0 must be the same patch series (a P0 of another series is not this arm's control).
set -euo pipefail
SNAP=$1 OUT=$2 P1_IMAGE=$3
shift 3
DEP=""
if [ $# -gt 0 ] && [ "$1" != "--" ]; then DEP=$1; shift; fi
if [ $# -gt 0 ] && [ "$1" = "--" ]; then shift; fi
S1_IMAGE=${S1_IMAGE:-}
L=/home/projects/forklift/test_runs/live
RUN=$L/ct2_624d0616_auto_line_m_s1/live/epoch_1/run
SPLICE=($L/ct2_624d0616_fixed_line_m_s3/live/epoch_1/run $L/ct2_624d0616_auto_line_m_s4/live/epoch_1/run
        $L/ct2_41a2a7e8_fixed_line_m_s2/live/epoch_1/run)
ARGS=(--noise-seed 1 --allow-unanswered-tail 8 --splice-to-sim-s 5000 --splice-runs "${SPLICE[@]}")
SETS="map_update_interval=1.0"
TRACE=FORKLIFT_SLAM_TRACE=/out/slam_trace.jsonl
NAMES=("$@")
[ ${#NAMES[@]} -gt 0 ] || NAMES=(p0_1 cwm_1 c600_1 c1200_1 p0_2 cwm_2 c600_2 c1200_2
                                 p0_mapq cwm_mapq c600_mapq c1200_mapq)
for f in "$RUN" "${SPLICE[@]}"; do [ -e "$f" ] || { echo "missing $f"; exit 2; }; done
docker image inspect "$P1_IMAGE" >/dev/null
cd "$SNAP"
mkdir -p "$OUT"
prev=$DEP
for NAME in "${NAMES[@]}"; do
  [ -e "$OUT/$NAME" ] && { echo "$OUT/$NAME exists"; exit 2; }
  DEPARG=()
  [ -n "$prev" ] && DEPARG=(--dependency=afterany:"$prev")
  IMAGE=$P1_IMAGE
  case $NAME in
    p0_mapq) SWITCHES="$TRACE" ;;
    p0_*) SWITCHES="" ;;
    cwm_mapq) SWITCHES="FORKLIFT_SLAM_CWM_S=600 $TRACE" ;;
    cwm_*) SWITCHES="FORKLIFT_SLAM_CWM_S=600" ;;
    c600_mapq) SWITCHES="FORKLIFT_SLAM_DELETE_OLDER_S=600 $TRACE" ;;
    c600_*) SWITCHES="FORKLIFT_SLAM_DELETE_OLDER_S=600" ;;
    c1200_mapq) SWITCHES="FORKLIFT_SLAM_DELETE_OLDER_S=1200 $TRACE" ;;
    c1200_*) SWITCHES="FORKLIFT_SLAM_DELETE_OLDER_S=1200" ;;
    s1_*)
      [ -n "$S1_IMAGE" ] || { echo "$NAME needs S1_IMAGE (the image with patch 0004)"; exit 2; }
      docker image inspect "$S1_IMAGE" >/dev/null
      IMAGE=$S1_IMAGE
      SWITCHES="FORKLIFT_SLAM_S1=1 FORKLIFT_SLAM_S1_BUFFER=10"
      [ "$NAME" = s1_mapq ] && SWITCHES="$SWITCHES $TRACE" ;;
    *) echo "unknown arm $NAME"; exit 2 ;;
  esac
  ENV=(SLAM_IMAGE="$IMAGE" SLAM_IMAGE_KIND=patched)
  [ -n "$SWITCHES" ] && ENV+=(SLAM_PATCH_ENV="$SWITCHES")
  # every variable the replay reads is set here or cleared, so nothing exported by the caller runs an arm as another
  prev=$(env -u SLAM_IMAGE -u SLAM_IMAGE_KIND -u SLAM_PATCH_ENV -u LIFELONG_PARAMS -u LOC_SETS -u MAP_BUNDLE \
    -u MAP_ID -u MAP_START_POSE -u LIVE_EPOCH -u BASELINE -u SUMMARY -u BUDGET -u KEEP_MAP_SUBSCRIPTION -u S1_IMAGE \
    $(env | grep -oE '^FORKLIFT_SLAM_[A-Z0-9_]+' | sed 's/^/-u /') \
    SLAM_NODE=sync DUMP_GRAPH=1 "${ENV[@]}" sbatch --parsable "${DEPARG[@]}" --job-name="s3_$NAME" \
    --output="$OUT/$NAME-%j.log" --cpus-per-task=2 --mem=1G --time=00:40:00 \
    deploy/slurm/slam_cost_replay.sbatch "$OUT/$NAME" "$RUN" "$SETS" "${ARGS[@]}")
  echo "$NAME job $prev"
done
