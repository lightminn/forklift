#!/usr/bin/env bash
# Online-SLAM plan stage S2 (docs/plans/2026-11-13-slam-online-bounded.md 6): the arms without C++ on the graph
# plan's S2 input -- the four line_m runs spliced to sim 5,000 s, noise seed 1, --allow-unanswered-tail 8 (graph plan
# 6.2; the restored runs at their own paths). One replay per job, CPU 2, serial (afterany) so no two replays share the
# node's CPUs while their cost is measured.
#   r0_1, r0_2  R0: deployed online config (R4 + R5 + R7), sync node (deploy/slurm/slam_cost_replay.sbatch)
#   a_1         A: localization on the graph plan's map bundle (lf_67694d16 unit 0, rebuilt byte-identical 2026-11-13,
#               map id 66a7f00f...), variant J, start -2.612,0,0 (a0's loc_j)
#   h_1, h_2    H: deploy/slurm/slam_h_switch.sbatch (unit 0 mapped, then localized on its own bundle)
#   r0_mapq     R0 map measurement: the same replay again, its cost not judged (plan 4: a mapping bridge already keeps
#               /map, so it is an R0 replay whose maps are read)
#   bash s2_submit.sh SNAP OUT [FIRST_DEPENDENCY] [-- NAME ...]
set -euo pipefail
SNAP=$1 OUT=$2
shift 2
DEP=""
if [ $# -gt 0 ] && [ "$1" != "--" ]; then DEP=$1; shift; fi
if [ $# -gt 0 ] && [ "$1" = "--" ]; then shift; fi
L=/home/projects/forklift/test_runs/live
RUN=$L/ct2_624d0616_auto_line_m_s1/live/epoch_1/run
SPLICE=($L/ct2_624d0616_fixed_line_m_s3/live/epoch_1/run $L/ct2_624d0616_auto_line_m_s4/live/epoch_1/run
        $L/ct2_41a2a7e8_fixed_line_m_s2/live/epoch_1/run)
ARGS=(--noise-seed 1 --allow-unanswered-tail 8 --splice-to-sim-s 5000 --splice-runs "${SPLICE[@]}")
SETS="map_update_interval=1.0"  # the deployed value, as the graph plan's jobs passed it
A_BUNDLE=$L/slam_online/a_rebuild/bundle
A_ID=66a7f00f54e0d9e1ff342ed12dce0b076ec58badf34c2ab0485fa085809b1511
NAMES=("$@")
[ ${#NAMES[@]} -gt 0 ] || NAMES=(r0_1 a_1 h_1 r0_2 h_2 r0_mapq)
for f in "$RUN" "${SPLICE[@]}" "$A_BUNDLE/manifest.json"; do [ -e "$f" ] || { echo "missing $f"; exit 2; }; done
grep -q "\"map_id\": \"$A_ID\"" "$A_BUNDLE/manifest.json" || { echo "A bundle map id differs"; exit 2; }
cd "$SNAP"
mkdir -p "$OUT"
prev=$DEP
for NAME in "${NAMES[@]}"; do
  [ -e "$OUT/$NAME" ] && { echo "$OUT/$NAME exists"; exit 2; }
  DEPARG=()
  [ -n "$prev" ] && DEPARG=(--dependency=afterany:"$prev")
  case $NAME in
    r0_*) JOB=(deploy/slurm/slam_cost_replay.sbatch "$OUT/$NAME" "$RUN" "$SETS" "${ARGS[@]}"); ENV=() ;;
    a_*) JOB=(deploy/slurm/slam_cost_replay.sbatch "$OUT/$NAME" "$RUN" "$SETS" "${ARGS[@]}")
         ENV=(SLAM_NODE=localization MAP_BUNDLE="$A_BUNDLE" MAP_ID="$A_ID" MAP_START_POSE="-2.612,0,0") ;;
    h_*) JOB=(deploy/slurm/slam_h_switch.sbatch "$OUT/$NAME" "$RUN" "$SETS" "${ARGS[@]}"); ENV=() ;;
    *) echo "unknown arm $NAME"; exit 2 ;;
  esac
  prev=$(env "${ENV[@]}" sbatch --parsable "${DEPARG[@]}" --job-name="s2_$NAME" --output="$OUT/$NAME-%j.log" \
    --cpus-per-task=2 --mem=1G --time=00:40:00 "${JOB[@]}")
  echo "$NAME job $prev"
done
