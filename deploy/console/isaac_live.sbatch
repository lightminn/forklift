#!/bin/bash
#SBATCH --job-name=console_live
#SBATCH --partition=compute
#SBATCH --gres=shard:1
#SBATCH --cpus-per-task=6
#SBATCH --mem=8G
#SBATCH --time=08:00:00
#SBATCH --signal=B:TERM@120
# A persistent operator console on the Isaac live runner (docs/plans/2026-10-18-persistent-live-console.md D9): the
# supervisor (tools/live_supervisor.py) runs one epoch after another -- its own SLAM bridge and runner -- until the
# operator ends the session (exit 0); a scene reset (exit 3) starts the next epoch at once, a failure after a
# backoff, three failures of one cause in a row stop it. The console keeps one port across epochs. Slurm's TERM
# (before the time limit) is passed to the supervisor, which ends the runner and its bridge and starts no more.
#
#   BASE_SCENE=<scene.usda> FORKLIFT_URDF=<forklift.urdf> VENV_PY=<python> sbatch deploy/console/isaac_live.sbatch OUT
#   optional: PORT (8798), FACTOR (0.4), SEED (0), ENCODER_MODEL (team_spec_quantized, or reported; each takes its own
#   obstacle-layer age table), EXTRA_ARGS (split on spaces), STANDBY_LINGER_S (300), MAP_BUNDLE (the SLAM mode,
#   deploy/console/slam_choice.sh -- unset, the default since the user's decision of 2026-10-27 (live-default plan 5d):
#   localization on the accepted survey bundle of SCENE_FILE and ENCODER_MODEL from config/slam_map_bundles.yaml under
#   SLAM_BUNDLE_ROOT, its map id held against the folder, and online mapping as a recorded fallback where the registry
#   has no row; registry: that bundle or no launch; none: mapping; a folder: that bundle with MAP_START_POSE "x,y,yaw",
#   map <- base_link at the scene's start), SLAM_BUNDLE_ROOT (where the registry's bundle folders are),
#   SESSION_ID / BRIDGE_CPU (set by tools/isaac_bundle.py when several sessions share the job),
#   LAYER_BUDGET (config/layer_latency_budget.yaml: --layer-latency-budget, B 0.10 s and the 1.0 s end, the lead's
#   decision 2026-11-11, docs/plans/2026-11-11-layer-latency-budget.md; off: none) -- records made before this default
#   ran without the budget: a comparison's two arms use the same LAYER_BUDGET, and results with and without it are
#   not mixed,
#   D7_HOLD (on: --d7-reapproach-hold, the SLAM corrections held over a D7 re-approach until its retry match -- its
#   registered V5 batch passed, docs/validation/2026-10-25-reapproach-slam-hold.md; off as before; an EXTRA_ARGS
#   --d7-reapproach-hold comes later and wins),
#   RETURN_START (boxed: --return-start-backoff boxed) and NEAR_END_STAND (on: --near-end-stand on) -- ④⑤, the
#   lead's promotion decision of 2026-11-11 after rv4 (docs/validation/2026-11-11-live-riib-rv4-results.md); off as
#   before (NEAR_END_STAND also shadow); a --near-tracking run defaults both off and refuses a return-start backoff other
#   than off; an EXTRA_ARGS value comes later and wins; records made before this default ran with both off: do not mix
#   them with later ones; measurement wrappers that call this launcher inherit the defaults unless they name the two,
#   so a measurement's registration names them,
#   NEAR_OBSERVE_ARRIVAL (0.05,0.05 when the near-end stand applied -- NEAR_END_STAND or an EXTRA_ARGS --near-end-stand
#   -- is on, else off: --near-observe-arrival, K1, docs/plans/2026-11-13-k1-near-loose-arrival.md; its registered
#   80-run comparison judged K1 better, docs/validation/2026-11-14-k1-comparison.md, lead decision 2026-11-14; off as
#   before; an explicit value needs the near-end stand on; an EXTRA_ARGS --near-observe-arrival comes later and wins;
#   records made before this default ran it off: a comparison names both arms' value and earlier off results are not
#   pooled with later on ones),
#   FAR_EXPECTED (on: --far-capture-expected on, the far capture without far recognition looks only in the mission's
#   adoption rectangle, so a neighbouring pallet is not adopted -- the lead's exception decision of 2026-10-28 after
#   docs/validation/2026-10-28-far-target-binding.md, which missed its pre-registered "no false refusal"; inert with
#   --recognition far; off as before; an EXTRA_ARGS --far-capture-expected comes later and wins; live records made
#   before this default -- CT-1, D7 and console baselines -- ran with it off: do not mix them with later ones),
#   START_POSE (B-P2 start relocalization, docs/plans/2026-10-29-start-relocalization.md: "x,y,yaw" world base_link --
#   the truck is placed there and the session finds itself on the survey map before its SLAM link opens: the runner
#   gets --start-pose/--start-relocalize, the supervisor --relocalize, the bridge the runner's estimate as its
#   map_start_pose instead of MAP_START_POSE; needs a survey map; the console shows the relocalization from the
#   start and takes the operator's pick or retry (B-P2c); RELOC_UNATTENDED=1: a start needing the operator ends the
#   session instead (unattended batches, B-P2d); RELOC_WAIT_S (600): the supervisor's whole wait, the operator's
#   time 60 s less), MAX_EPOCHS (unset: epochs until the session ends; 1 for a validation session that must not be
#   retried, B-P2d),
#   START_RELOC (on|off; start relocalization plan 7.1, lead 2026-10-31): unset, on for an operator's session and off
#   in a bundle's run (BUNDLE_TOKEN). On without START_POSE, on the registry's survey map: the truck placed at the
#   registry's scene start and relocalized (START_MODE in env.sh says what the start is and why) -- by default only
#   where the registry row is marked relocalizable at its start, otherwise the known start with the reason added to
#   SLAM_CHOICE (the console shows it); without that map, the known start (on asked for by name: no launch). START_POSE
#   relocalizes whatever the default; off with it: no launch.
#   SCENE_FILE (a tools/riibotics_isaac_scenes.py scene: --layout scene on its layout as the prior map -- no
#   --grid-planning, --min-obstacle-height-m or --prism-colliders -- instead of the factory hall)
#   The far recognition (docs/plans/2026-10-22-live-far-recognition.md) goes in EXTRA_ARGS: --recognition far and,
#   for the learned proposals, the model and interpreter of that host -- then submit with --gres=shard:2: the
#   session's Isaac (3.4 GB) and the learned worker (0.5 GB) peaked at 3.9 GB of a 4 GB shard (2026-10-22), too
#   close for a GPU shared with other jobs (a worker's warm-up ran out of GPU memory once).
#   POSE_CHECK (off|shadow|on; docs/plans/2026-11-06-unconfirmed-validation.md 2): the unconfirmed position check,
#   with START_POSE; on needs POSE_CHECK_BOUNDS (the development values' JSON file), shadow takes none.
#   POSE_CHECK_CONTACTS=1 records every forklift body's contacts. RELOC_INJECT (test only: a wrong-estimate injection,
#   reloc_phase.INJECT_KINDS) only in an unattended relocalization (RELOC_UNATTENDED=1) with the check on or shadow.
#   F1_AUTONOMY (block|allow; docs/plans/2026-11-08-f1-vehicle-block.md, lead 2026-11-08): with START_POSE, unset is
#   block -- an operator-placed start (or one of unknown source) refuses AUTO and missions for the epoch, teleop open;
#   allow only for an approved development run (the value is in the launch log and the runner's recorded arguments).
#   START_POSE_KNOWN (diagnostic; docs/plans/2026-11-09-l4c-repro-cancel-other-target.md 1.3, the repro batch's H0):
#   "x,y,yaw" world base_link -- the truck placed there and its start known there, no relocalization: the runner gets
#   --start-pose/--start-known, the bridge it as its MAP_START_POSE. Needs a survey map; never with START_POSE or
#   START_RELOC=on (it turns the start relocalization off); no F1 block (nothing is placed by an operator).
#
# Run from the repository root. Open the console through   ssh -N -L 8798:127.0.0.1:$PORT <host>.
set -u
OUT=$(realpath -m "${1:?output directory}")
ROOT=$(pwd)
: "${BASE_SCENE:?the base scene (.usda)}" "${FORKLIFT_URDF:?the forklift URDF the scene was built from}"
: "${VENV_PY:?the Isaac Python}"
# In a bundle (deploy/slurm/isaac_bundle.sbatch) several sessions share one Slurm job: SESSION_ID names this one's
# bridge containers and BRIDGE_CPU pins its bridge to a CPU of its own; alone, the job ID and the job's first CPU.
# The SLAM odometry's encoders (plan 2026-10-25-team-spec-default, lead decision): the joint-angle model by default,
# reported on request (its own baselines; never mixed with the default's), nothing else (no other age table).
ENCODER_MODEL=${ENCODER_MODEL:-team_spec_quantized}
case "$ENCODER_MODEL" in reported|team_spec_quantized) ;; *) echo "ENCODER_MODEL must be reported or team_spec_quantized"; exit 2;; esac
D7_HOLD=${D7_HOLD:-on}
case "$D7_HOLD" in on|off) ;; *) echo "D7_HOLD must be on or off"; exit 2;; esac
FAR_EXPECTED=${FAR_EXPECTED:-on}
case "$FAR_EXPECTED" in on|off) ;; *) echo "FAR_EXPECTED must be on or off"; exit 2;; esac
# The G3 scan-step changes (real-time plan 5.26-5.27; the lead's decision of 2026-11-10 after the 1b, 1c and 1d stage
# checks and the 1e judgement run): on by default in this launcher -- FORKLIFT_GRID_CLEAR_KERNEL=1 (1b, the grid's
# beam-cell clearing), FORKLIFT_SHADOW_KERNEL=1 (1c, the shadow memory's array path) and LIDAR_CAST=fast (1d, the LiDAR
# loop with a per-body own cache); each gives the same output as off (docs/validation/2026-11-10-g3-promote.md). An
# explicit value wins (0/1, loop/fast/oracle -- a measurement list's 0 stays 0) and an EXTRA_ARGS --lidar-cast comes
# later and wins. A --near-tracking run keeps all three off: its contract compares lidar_cast loop and cannot see the
# two environment switches, so its defaults are 0/0/loop and an explicit on is refused. The runner records what it
# applied (result.json scan_switches). Live records made before this default (the rv3 and ④⑤ batches, earlier console
# baselines) ran with them off at another realized sim/wall rate: do not mix them with later ones.
g3_words() { local IFS=$' \t\n'; set -f; printf '%s\n' ${EXTRA_ARGS:-}; set +f; }
G3_NEAR=0
G3_EXTRA_CAST=""
G3_EXTRA_RSB=""  # an EXTRA_ARGS --return-start-backoff's last value (other than off refused with --near-tracking)
G3_EXTRA_NES=""  # an EXTRA_ARGS --near-end-stand's last value (K1's default follows the value applied)
G3_EXTRA_NOA=""  # an EXTRA_ARGS --near-observe-arrival's last value
g3_next=0
rsb_next=0
nes_next=0
noa_next=0
while IFS= read -r w; do
  if [ "$g3_next" = 1 ]; then G3_EXTRA_CAST=$w; g3_next=0; continue; fi
  if [ "$rsb_next" = 1 ]; then G3_EXTRA_RSB=$w; rsb_next=0; continue; fi
  if [ "$nes_next" = 1 ]; then G3_EXTRA_NES=$w; nes_next=0; continue; fi
  if [ "$noa_next" = 1 ]; then G3_EXTRA_NOA=$w; noa_next=0; continue; fi
  case "$w" in
    --near-tracking) G3_NEAR=1;;
    --lidar-cast) g3_next=1;;
    --lidar-cast=*) G3_EXTRA_CAST=${w#--lidar-cast=};;
    --return-start-backoff) rsb_next=1;;
    --return-start-backoff=*) G3_EXTRA_RSB=${w#--return-start-backoff=};;
    --near-end-stand) nes_next=1;;  # comes after the launcher's and wins (argparse keeps the last)
    --near-end-stand=*) G3_EXTRA_NES=${w#--near-end-stand=};;
    --near-observe-arrival) noa_next=1;;  # K1: the launcher then adds none of its own
    --near-observe-arrival=*) G3_EXTRA_NOA=${w#--near-observe-arrival=};;
    --?*)  # argparse also takes an unambiguous abbreviation: refused here, so what is printed is what runs
      o=${w%%=*}
      for full in --near-tracking --lidar-cast --return-start-backoff --near-end-stand --near-observe-arrival; do
        case "$full" in "$o"*) echo "EXTRA_ARGS $o: write $full in full"; exit 2;; esac
      done;;
  esac
done < <(g3_words)
if [ "$G3_NEAR" = 1 ]; then
  for v in "${FORKLIFT_GRID_CLEAR_KERNEL:-0}" "${FORKLIFT_SHADOW_KERNEL:-0}"; do
    [ "$v" = 0 ] || { echo "--near-tracking keeps FORKLIFT_GRID_CLEAR_KERNEL and FORKLIFT_SHADOW_KERNEL at 0"; exit 2; }
  done
  [ "${LIDAR_CAST:-loop}" = loop ] && [ "${G3_EXTRA_CAST:-loop}" = loop ] \
    || { echo "--near-tracking keeps the LiDAR cast at loop"; exit 2; }
fi
G3_DEFAULT=1
[ "$G3_NEAR" = 1 ] && G3_DEFAULT=0
FORKLIFT_GRID_CLEAR_KERNEL=${FORKLIFT_GRID_CLEAR_KERNEL:-$G3_DEFAULT}
FORKLIFT_SHADOW_KERNEL=${FORKLIFT_SHADOW_KERNEL:-$G3_DEFAULT}
if [ "$G3_NEAR" = 1 ]; then LIDAR_CAST=${LIDAR_CAST:-loop}; else LIDAR_CAST=${LIDAR_CAST:-fast}; fi
case "$FORKLIFT_GRID_CLEAR_KERNEL" in 0|1) ;; *) echo "FORKLIFT_GRID_CLEAR_KERNEL must be 0 or 1"; exit 2;; esac
case "$FORKLIFT_SHADOW_KERNEL" in 0|1) ;; *) echo "FORKLIFT_SHADOW_KERNEL must be 0 or 1"; exit 2;; esac
case "$LIDAR_CAST" in loop|fast|oracle) ;; *) echo "LIDAR_CAST must be loop, fast or oracle"; exit 2;; esac
case "$G3_EXTRA_CAST" in ""|loop|fast|oracle) ;; *) echo "EXTRA_ARGS --lidar-cast must be loop, fast or oracle"; exit 2;; esac
echo "SCAN_SWITCHES grid_clear_kernel=$FORKLIFT_GRID_CLEAR_KERNEL shadow_kernel=$FORKLIFT_SHADOW_KERNEL" \
  "lidar_cast=${G3_EXTRA_CAST:-$LIDAR_CAST}${G3_EXTRA_CAST:+ (EXTRA_ARGS)}"
# ④⑤ (the lead's promotion decision of 2026-11-11 after rv4, docs/validation/2026-11-11-live-riib-rv4-results.md):
# on by default in this launcher -- --return-start-backoff boxed (RETURN_START) and --near-end-stand on
# (NEAR_END_STAND); an explicit value wins and an EXTRA_ARGS --return-start-backoff / --near-end-stand comes later and
# wins (abbreviations refused, so what is printed is what runs). A --near-tracking run defaults both to off: its contract
# records the return-start backoff off (near_tracking.py), so a value other than off is refused there; the near-end
# stand is a run-specific argument of that contract and may be named.
# The runner's own defaults stay off. Live records made before this default (rv4's arm A, earlier console baselines)
# ran with both off: do not mix them with later ones.
if [ "$G3_NEAR" = 1 ]; then RETURN_START=${RETURN_START:-off}; NEAR_END_STAND=${NEAR_END_STAND:-off}
else RETURN_START=${RETURN_START:-boxed}; NEAR_END_STAND=${NEAR_END_STAND:-on}; fi
case "$RETURN_START" in on|boxed|off) ;; *) echo "RETURN_START must be on, boxed or off"; exit 2;; esac
case "$NEAR_END_STAND" in on|shadow|off) ;; *) echo "NEAR_END_STAND must be on, shadow or off"; exit 2;; esac
if [ "$G3_NEAR" = 1 ] && { [ "$RETURN_START" != off ] || { [ -n "$G3_EXTRA_RSB" ] && [ "$G3_EXTRA_RSB" != off ]; }; }; then
  echo "--near-tracking keeps the return-start backoff off (RETURN_START and any EXTRA_ARGS --return-start-backoff off)"
  exit 2
fi
# K1 follows the near-end stand applied (the runner refuses it without --near-end-stand on); an explicit value needs on
NES_APPLIED=${G3_EXTRA_NES:-$NEAR_END_STAND}
if [ -n "$G3_EXTRA_NOA" ]; then NOA_APPLIED=$G3_EXTRA_NOA
elif [ -n "${NEAR_OBSERVE_ARRIVAL:-}" ]; then NOA_APPLIED=$NEAR_OBSERVE_ARRIVAL
elif [ "$NES_APPLIED" = on ]; then NOA_APPLIED=0.05,0.05
else NOA_APPLIED=off; fi
if [ "$NOA_APPLIED" != off ] && [ "$NES_APPLIED" != on ]; then
  echo "NEAR_OBSERVE_ARRIVAL (or EXTRA_ARGS --near-observe-arrival) needs the near-end stand on, not $NES_APPLIED"; exit 2
fi
# the shape only: the numbers are the runner's check (parse_bounds_pair takes .05 and 5e-2 too)
[ "$NOA_APPLIED" = off ] || [[ "$NOA_APPLIED" =~ ^[^,[:space:]]+,[^,[:space:]]+$ ]] \
  || { echo "NEAR_OBSERVE_ARRIVAL must be M,RAD (metres, radians) or off"; exit 2; }
# what the launcher passes: nothing when off or when EXTRA_ARGS names its own (the runner reads one)
NEAR_OBSERVE_ARRIVAL=$NOA_APPLIED
[ -n "$G3_EXTRA_NOA" ] && NEAR_OBSERVE_ARRIVAL=off
echo "RIIB_DEFAULTS return_start_backoff=$RETURN_START near_end_stand=$NES_APPLIED near_observe_arrival=$NOA_APPLIED"
# The SLAM mode (live-default plan S3 D2, 5d): MAP_BUNDLE unset (the registry, mapping as a recorded fallback) / none /
# registry / a folder, deploy/console/slam_choice.sh; the choice goes to the supervisor's log (--slam-choice)
. "$ROOT/deploy/console/slam_choice.sh"
# the repro batch's H0 (diagnostic): a known start is the bridge's start pose on any survey map, the registry's or one
# given by hand (checked with the start choice below)
[ -n "${START_POSE_KNOWN:-}" ] && [ -n "${MAP_BUNDLE:-}" ] && MAP_START_POSE=$START_POSE_KNOWN
if [ -n "${MAP_BUNDLE:-}" ]; then
  # a bundle folder that is not there stops the launch (an empty realpath would have read as mapping)
  MAP_BUNDLE=$(realpath -e "$MAP_BUNDLE") || { echo "no SLAM bundle folder ${MAP_BUNDLE}" >&2; exit 2; }
  : "${MAP_START_POSE:?the start pose x,y,yaw the bridge tells slam_toolbox (map <- base_link)}"
fi
[[ -z "${MAX_EPOCHS:-}" || "$MAX_EPOCHS" =~ ^[1-9][0-9]*$ ]] || { echo "MAX_EPOCHS must be a whole number >= 1"; exit 2; }
RELOC_WAIT_S=${RELOC_WAIT_S:-600}
case "$RELOC_WAIT_S" in ''|*[!0-9]*) echo "RELOC_WAIT_S must be whole seconds"; exit 2;; esac
[ "$RELOC_WAIT_S" -gt 60 ] || { echo "RELOC_WAIT_S must be over 60 s (the operator's time is 60 s less)"; exit 2; }
POSE_CHECK=${POSE_CHECK:-off}
case "$POSE_CHECK" in off|shadow|on) ;; *) echo "POSE_CHECK must be off, shadow or on" >&2; exit 2;; esac
# the vehicle-side F1 block (docs/plans/2026-11-08-f1-vehicle-block.md 2.5, lead 2026-11-08): unset, block; allow only
# for approved development runs (an unconfirmed-position run with a mission after a placement); set but empty: refused
F1_AUTONOMY=${F1_AUTONOMY-block}
case "$F1_AUTONOMY" in block|allow) ;; *) echo "F1_AUTONOMY must be block or allow" >&2; exit 2;; esac
if [ "$POSE_CHECK" != off ] && [ -z "${START_POSE:-}" ]; then
  echo "POSE_CHECK=$POSE_CHECK needs START_POSE (the check follows the start relocalization)" >&2; exit 2
fi
if [ "$POSE_CHECK" = on ]; then
  [ -f "${POSE_CHECK_BOUNDS:-}" ] || { echo "POSE_CHECK=on needs POSE_CHECK_BOUNDS (a JSON file)" >&2; exit 2; }
  POSE_CHECK_BOUNDS=$(realpath "$POSE_CHECK_BOUNDS")
elif [ -n "${POSE_CHECK_BOUNDS:-}" ]; then
  echo "POSE_CHECK_BOUNDS goes with POSE_CHECK=on" >&2; exit 2
fi
case "${POSE_CHECK_CONTACTS:-}" in ''|0) POSE_CHECK_CONTACTS= ;; 1) ;; *) echo "POSE_CHECK_CONTACTS must be 1" >&2; exit 2;; esac
if [ -n "$POSE_CHECK_CONTACTS" ] && [ "$POSE_CHECK" = off ]; then
  echo "POSE_CHECK_CONTACTS goes with POSE_CHECK on or shadow" >&2; exit 2
fi
if [ -n "${RELOC_INJECT:-}" ] && { [ "$POSE_CHECK" = off ] || [ "${RELOC_UNATTENDED:-}" != 1 ]; }; then
  echo "RELOC_INJECT is for unattended relocalization batches with POSE_CHECK on or shadow" >&2; exit 2
fi
if [ -n "${START_POSE:-}" ] && [ -z "${MAP_BUNDLE:-}" ]; then
  echo "START_POSE relocalizes on a survey map: it needs one (slam: $SLAM_CHOICE)" >&2
  exit 2
fi
if [ -n "${START_POSE_KNOWN:-}" ]; then  # the repro batch's H0 (diagnostic): a known start at a placed pose
  [ -n "${START_POSE:-}" ] && { echo "START_POSE_KNOWN and START_POSE are two starts: give one" >&2; exit 2; }
  if [ -z "${MAP_BUNDLE:-}" ]; then
    echo "START_POSE_KNOWN is a known start on a survey map: it needs one (slam: $SLAM_CHOICE)" >&2; exit 2
  fi
  if [ -n "${START_RELOC+set}" ] && [ "$START_RELOC" != off ]; then
    echo "START_RELOC=${START_RELOC} contradicts START_POSE_KNOWN (a known start, no relocalization)" >&2; exit 2
  fi
  START_RELOC=off
fi
# The start (lead 2026-10-31, start relocalization plan 7.1): the operator's console relocalizes by default -- placed at
# the registry's scene start (the survey's own settled start, ~80 um from the scene's nominal one; the localization
# never reads it, plan 1.1) and found on its survey map, the operator picking or retrying when it is ambiguous. A
# bundle's run (BUNDLE_TOKEN: unattended) keeps the known start, as riib and the replay gate do. START_POSE
# relocalizes as before; a map folder given by hand keeps the known start (its MAP_START_POSE is the SLAM's start
# estimate, not where the truck stands: Codex design review P2).
if [ -n "${START_RELOC+set}" ]; then  # given, even empty: checked below (Codex B-P2e review P3)
  START_RELOC_GIVEN=${START_RELOC:-empty}
else
  START_RELOC_GIVEN=
  START_RELOC=$([ -n "${BUNDLE_TOKEN:-}" ] && echo off || echo on)
fi
case "$START_RELOC" in on|off) ;; *) echo "START_RELOC must be on or off"; exit 2;; esac
if [ -n "${START_POSE:-}" ]; then
  [ "$START_RELOC_GIVEN" = off ] && { echo "START_RELOC=off contradicts START_POSE (which relocalizes)" >&2; exit 2; }
  START_MODE="relocalization (START_POSE)"
elif [ "$START_RELOC" = off ]; then
  if [ -n "$START_RELOC_GIVEN" ]; then START_MODE="known (START_RELOC=off)"; else START_MODE="known (a bundle's run: unattended)"; fi
elif [[ "$SLAM_CHOICE" == "localization (registry: "* ]]; then
  # lead 2026-10-31 (a): by default only on a row the Isaac smoke marked relocalizable at its start (the registry's
  # start_relocalizable); elsewhere the known start, its reason in the log, the supervisor's and the console's
  # slam.choice. START_RELOC=on asks for the relocalization all the same
  if [ "$MAP_START_RELOC" = yes ]; then
    START_POSE=$MAP_START_POSE
    START_MODE="relocalization (default: placed at the registry's scene start)"
  elif [ -n "$START_RELOC_GIVEN" ]; then
    START_POSE=$MAP_START_POSE
    START_MODE="relocalization (START_RELOC=on: the registry row is not marked relocalizable at its start)"
  else
    START_MODE="known (the registry row is not marked relocalizable at its start)"
    SLAM_CHOICE="$SLAM_CHOICE; start known: the row is not marked relocalizable"
  fi
else
  if [ -n "$START_RELOC_GIVEN" ]; then
    echo "START_RELOC=on needs the registry's survey map to relocalize on (slam: $SLAM_CHOICE)" >&2
    exit 2
  fi
  if [ -n "${MAP_BUNDLE:-}" ]; then
    START_MODE="known (a map folder given by hand: START_POSE relocalizes on it)"
  else
    START_MODE="known (no survey map to relocalize on)"
  fi
fi
if [ -n "${START_POSE_KNOWN:-}" ]; then  # where the truck stands is where the SLAM starts (map = world, set above)
  START_MODE="known at START_POSE_KNOWN (diagnostic: placed there, no relocalization)"
fi
echo "slam: $SLAM_CHOICE"
echo "start: $START_MODE"
[ -n "${START_POSE:-}" ] && echo "F1_AUTONOMY=$F1_AUTONOMY (an operator-placed start: block refuses AUTO and missions)"
JOB=${SESSION_ID:-${SLURM_JOB_ID:-local$$}}
mkdir -p "$OUT"
export OMNI_KIT_ACCEPT_EULA=YES OMP_NUM_THREADS=2 PYTHONPATH=$ROOT/src
# The CPUs (1e (c), docs/validation/2026-11-11-g3-1e-c-results.md, the lead's decision of 2026-11-11): alone, the job's
# CPUs split as a single-slot bundle splits them (tools/isaac_bundle.py split_bridge_cpus / auto_bridge_cpus) -- a whole
# core for the SLAM bridge, the session pinned to the rest: the default job CPU 6 gives the session two physical cores,
# CPU 4 one; fewer than a session core plus a bridge core and nothing is split (the bridge on the first CPU, the session
# unpinned, as before). In a bundle BRIDGE_CPU comes from the bundle runner, which pins the session itself.
# LIVE_TEST_CPUS / LIVE_TEST_SIBLINGS (tests only) stand in for the job's affinity and the CPU topology.
SESSION_CPUS=""
if [ -n "${BRIDGE_CPU:-}" ]; then
  CPU0=$BRIDGE_CPU
else
  read -r SPLIT_BRIDGE SPLIT_SESSION < <("$VENV_PY" -c '
import json, os, sys
sys.path.insert(0, os.path.join(sys.argv[1], "tools"))
import isaac_bundle as b
cpus = sorted(int(x) for x in os.environ["LIVE_TEST_CPUS"].split(",")) if os.environ.get("LIVE_TEST_CPUS") \
    else sorted(os.sched_getaffinity(0))
sib = json.loads(os.environ["LIVE_TEST_SIBLINGS"]) if os.environ.get("LIVE_TEST_SIBLINGS") else None
siblings = (lambda c: tuple(sib.get(str(c), [c]))) if sib is not None else b._siblings
bridge, session = b.split_bridge_cpus(cpus, b.auto_bridge_cpus(len(cpus), 1), siblings)
print(",".join(map(str, bridge)) or "-", ",".join(map(str, session)) or "-")
' "$ROOT")
  if [ -n "${SPLIT_BRIDGE:-}" ] && [ "$SPLIT_BRIDGE" != - ]; then
    CPU0=$SPLIT_BRIDGE
    SESSION_CPUS=$SPLIT_SESSION
  else
    CPU0=$(taskset -cp $$ | awk -F': ' '{print $2}' | cut -d, -f1 | cut -d- -f1)
  fi
fi
echo "cpus: bridge $CPU0, session ${SESSION_CPUS:-unpinned}"
# Every value the epoch scripts use, quoted for the shell once here; the scripts below expand nothing at writing.
{
  printf 'ROOT=%q\nBASE_SCENE=%q\nFORKLIFT_URDF=%q\nVENV_PY=%q\nJOB=%q\nCPU0=%q\n' \
    "$ROOT" "$BASE_SCENE" "$FORKLIFT_URDF" "$VENV_PY" "$JOB" "$CPU0"
  printf 'SESSION_CPUS=%q\n' "$SESSION_CPUS"  # the session's pinning (empty: unpinned, or the bundle's)
  printf 'PORT=%q\nFACTOR=%q\nSEED=%q\nEXTRA_ARGS=%q\n' "${PORT:-8798}" "${FACTOR:-0.4}" "${SEED:-0}" "${EXTRA_ARGS:-}"
  printf 'MAP_BUNDLE=%q\nMAP_START_POSE=%q\n' "${MAP_BUNDLE:-}" "${MAP_START_POSE:-}"
  printf 'SCENE_FILE=%q\n' "$([ -n "${SCENE_FILE:-}" ] && realpath "$SCENE_FILE")"
  printf 'ENCODER_MODEL=%q\n' "$ENCODER_MODEL"
  printf 'D7_HOLD=%q\n' "$D7_HOLD"
  printf 'FAR_EXPECTED=%q\n' "$FAR_EXPECTED"
  # the G3 scan-step switches (real-time plan 5.26-5.27): the runner exports the two and passes --lidar-cast
  printf 'FORKLIFT_GRID_CLEAR_KERNEL=%q\nFORKLIFT_SHADOW_KERNEL=%q\nLIDAR_CAST=%q\n' \
    "$FORKLIFT_GRID_CLEAR_KERNEL" "$FORKLIFT_SHADOW_KERNEL" "$LIDAR_CAST"
  printf 'RETURN_START=%q\nNEAR_END_STAND=%q\n' "$RETURN_START" "$NEAR_END_STAND"  # ④⑤ (the runner passes them)
  printf 'NEAR_OBSERVE_ARRIVAL=%q\n' "$NEAR_OBSERVE_ARRIVAL"  # K1 (the runner passes it unless off)
  printf 'START_POSE=%q\n' "${START_POSE:-}"  # B-P2: the placed start (empty: the scene's, known)
  printf 'START_POSE_KNOWN=%q\n' "${START_POSE_KNOWN:-}"  # the repro batch's H0: placed and known (empty: off)
  printf 'START_RELOC=%q\nSTART_MODE=%q\n' "$START_RELOC" "$START_MODE"  # plan 7.1: what the start is, and why
  printf 'RELOC_UNATTENDED=%q\nRELOC_WAIT_S=%q\n' "${RELOC_UNATTENDED:-}" "$RELOC_WAIT_S"  # B-P2c
  printf 'F1_AUTONOMY=%q\n' "$F1_AUTONOMY"  # the vehicle-side F1 block (plan 2026-11-08 2.5)
  printf 'SLAM_CHOICE=%q\n' "$SLAM_CHOICE"  # the runner shows it on the console (live-default plan 5d)
  printf 'POSE_CHECK=%q\nPOSE_CHECK_BOUNDS=%q\nPOSE_CHECK_CONTACTS=%q\nRELOC_INJECT=%q\n' "$POSE_CHECK" \
    "${POSE_CHECK_BOUNDS:-}" "$POSE_CHECK_CONTACTS" "${RELOC_INJECT:-}"  # plan 2026-11-06-unconfirmed-validation 2
  # the SLAM bridge's image: the tag by default; a judged batch names it by digest (B-P2d, Codex freeze review P2)
  printf 'BRIDGE_IMAGE=%q\n' "${BRIDGE_IMAGE:-forklift/ros2-dev:jazzy}"
  printf 'REQUIREMENTS_PROFILE=%q\nSCENE_REQUIREMENTS_OVERRIDE=%q\n' "${REQUIREMENTS_PROFILE:-}" \
    "${SCENE_REQUIREMENTS_OVERRIDE:-}"  # scene requirements plan
} > "$OUT/env.sh"

cat > "$OUT/bridge.sh" <<'EOT'
#!/bin/bash
# epoch $1, its directory $2: the SLAM bridge takes one connection, so each epoch has its own; with MAP_BUNDLE the
# session's map id $3 and bundle $4 (the supervisor's) -- the bundle mounted read-only, localization (plan L4); with
# START_POSE the runner's estimate $5 (B-P2: the supervisor's {map_start_pose}) is the bridge's start pose
. "$(dirname "$0")/env.sh"
mkdir -p "$2/bridge" "$2/sock"
LOC=()
START=$MAP_START_POSE
[ -n "${START_POSE:-}" ] && START=${5:?the estimate x,y,yaw from the runner}
[ -n "${MAP_BUNDLE:-}" ] && LOC=(-v "$4:/bundle:ro" -e SLAM_MAP_BUNDLE=/bundle -e SLAM_EXPECTED_MAP_ID="$3"
                                 -e SLAM_MAP_START_POSE="$START")
docker run -d --name "slam_bridge_${JOB}_e$1" --network none --cpuset-cpus "$CPU0" --memory 4g --group-add 2000 \
  -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$ROOT:/workspace:ro" -v "$2/bridge:/out" -v "$2/sock:/run/slam" "${LOC[@]}" \
  "$BRIDGE_IMAGE" bash /workspace/tools/slam_bridge_container.sh /run/slam/slam.sock /out "-0.12,0.0,1.05,0.0" persistent >/dev/null
# the image id this epoch's container runs, kept beside its records (a tag can move during a batch)
docker inspect --format '{{.Image}}' "slam_bridge_${JOB}_e$1" > "$2/bridge/image_id"
EOT
cat > "$OUT/stop.sh" <<'EOT'
#!/bin/bash
. "$(dirname "$0")/env.sh"
# a missing container is no failure; a removal that fails is (the supervisor logs it)
if [ -n "$(docker ps -aq --filter "name=^slam_bridge_${JOB}_e$1\$")" ]; then
  # the bridge ends by itself once the runner closed its socket, writing its records and final status first
  # (persistent console plan D8b): give it that time, then remove the container either way
  timeout 45 docker wait "slam_bridge_${JOB}_e$1" >/dev/null 2>&1
  docker rm -f "slam_bridge_${JOB}_e$1" >/dev/null
fi
EOT
cat > "$OUT/runner.sh" <<'EOT'
#!/bin/bash
# epoch $1, its directory $2, the port file $3, the session's map id $4 and survey bundle $5 (with MAP_BUNDLE)
. "$(dirname "$0")/env.sh"
cd "$ROOT"
# the G3 scan-step switches (env.sh): read by the modules at import; an EXTRA_ARGS --lidar-cast comes later and wins
export FORKLIFT_GRID_CLEAR_KERNEL="${FORKLIFT_GRID_CLEAR_KERNEL:-0}" FORKLIFT_SHADOW_KERNEL="${FORKLIFT_SHADOW_KERNEL:-0}"
LOC=()  # SLAM localization plan L7: localize on the survey bundle from the start (unset: online mapping)
[ -n "${MAP_BUNDLE:-}" ] && LOC=(--slam-map-bundle "$5" --expected-map-id "$4")
LAYOUT=(--min-obstacle-height-m 1.15 --prism-colliders --layout factory --grid-planning)
[ -n "${SCENE_FILE:-}" ] && LAYOUT=(--layout scene --scene-file "$SCENE_FILE")
# the SLAM odometry's encoders and the obstacle layer's age table drawn from the same model (the runner refuses a
# reported table for a team-spec model; PR #108's team-spec table)
LAYER=config/obstacle_layer_single.yaml
[ "$ENCODER_MODEL" = reported ] || LAYER=config/obstacle_layer_single_team_spec.yaml
# the layer latency budget (docs/plans/2026-11-11-layer-latency-budget.md, the lead's decision 2026-11-11): a file kept
# apart from the obstacle-layer one (a survey bundle's shared input); LAYER_BUDGET=off runs without it
LAYER_BUDGET=${LAYER_BUDGET:-config/layer_latency_budget.yaml}
BUDGET=(--layer-latency-budget "$LAYER_BUDGET")
# K1 (env.sh): the near-leg loose arrival narrowed unless off (an EXTRA_ARGS --near-observe-arrival left it off here)
NOA_ARGS=()
[ "${NEAR_OBSERVE_ARRIVAL:-off}" = off ] || NOA_ARGS=(--near-observe-arrival "$NEAR_OBSERVE_ARRIVAL")
[ "$LAYER_BUDGET" = off ] && BUDGET=()
RELOC=()  # B-P2b: placed at START_POSE, the start found on the survey map; the epoch folder holds the estimate
# one word: a pose with a negative x would read as an option (Codex B-P2b review P2)
[ -n "${START_POSE:-}" ] && RELOC=("--start-pose=$START_POSE" --start-relocalize --reloc-dir "$2"
                                  --reloc-operator-timeout-s "$((RELOC_WAIT_S - 60))")
[ -n "${START_POSE:-}" ] && [ "${RELOC_UNATTENDED:-}" = 1 ] && RELOC+=(--reloc-unattended)
[ -n "${START_POSE:-}" ] && RELOC+=(--operator-placed-auto "${F1_AUTONOMY:-block}")  # plan 2026-11-08 2.5
# the repro batch's H0 (diagnostic): placed at START_POSE_KNOWN and known there; one word, as above
[ -n "${START_POSE_KNOWN:-}" ] && RELOC=("--start-pose=$START_POSE_KNOWN" --start-known)
POSE=()  # the unconfirmed position check (plan 2026-11-06-unconfirmed-validation 2); off: as before
[ "${POSE_CHECK:-off}" != off ] && POSE=(--pose-check "$POSE_CHECK")
[ "${POSE_CHECK:-off}" = on ] && POSE+=(--pose-check-bounds "$POSE_CHECK_BOUNDS")
[ -n "${POSE_CHECK_CONTACTS:-}" ] && POSE+=(--pose-check-contacts)
[ -n "${RELOC_INJECT:-}" ] && POSE+=("--reloc-inject=$RELOC_INJECT")  # one word: the kinds hold a space
REQ=()  # scene requirements plan: a profile to compare with and an override reason, both recorded by the runner
[ -n "${REQUIREMENTS_PROFILE:-}" ] && REQ+=(--requirements-profile "$REQUIREMENTS_PROFILE")
[ -n "${SCENE_REQUIREMENTS_OVERRIDE:-}" ] && REQ+=(--scene-requirements-override "$SCENE_REQUIREMENTS_OVERRIDE")
# shellcheck disable=SC2086 -- EXTRA_ARGS is a list of words by design
ARGS=(
  --base-scene "$BASE_SCENE" \
  --pallet-urdf sim/models/epal6_pallet/pallet.urdf --pallet-geometry config/pallet_geometry_epal6.yaml \
  --forklift-urdf "$FORKLIFT_URDF" --settings config/isaac_transport_measured.yaml \
  --delivery-straight-m 1.5 --alignment-straight-m 2.1 --withdrawal-m 0.55 --insertion-reserve-m 0.016 \
  "${LAYOUT[@]}" --fps 60 \
  --use-perception --pallet-prior config/pallet_prior_epal6.yaml --planning-target perception \
  --perception-mount carriage_low_measured --depth-quantize-mm 1 --record-slam --return-home \
  --obstacle-layer "$LAYER" ${BUDGET[@]+"${BUDGET[@]}"} --encoder-model "$ENCODER_MODEL" --obstacle-act --seed "$SEED" \
  --slam-feedback "$2/sock/slam.sock" --slam-map-dir "$2/bridge" --slam-noise-seed "$SEED" \
  --d7-planner --d7-docking --d7-reapproach-hold "$D7_HOLD" --far-capture-expected "$FAR_EXPECTED" \
  --live --live-supervised --live-port "$PORT" --live-port-file "$3" \
  --live-factor "$FACTOR" --live-web-root "$ROOT/console/web" --live-epoch "$1" --slam-choice "$SLAM_CHOICE" \
  --lidar-cast "${LIDAR_CAST:-loop}" \
  --return-start-backoff "${RETURN_START:-off}" --near-end-stand "${NEAR_END_STAND:-off}" \
  ${NOA_ARGS[@]+"${NOA_ARGS[@]}"} \
  $EXTRA_ARGS "${LOC[@]}" "${RELOC[@]}" "${POSE[@]}" "${REQ[@]}" \
  --output "$2/run")
if [ -n "${RUNNER_CHECK:-}" ]; then  # the launcher's preflight: these arguments, then the runner's own checks on them
  # an older runner would take --check-args for a Kit argument and start Isaac
  grep -q -- '"--check-args"' "$ROOT/sim/isaac/run_transport.py" \
    || { echo "preflight: $ROOT's runner has no --check-args: refused" >&2; exit 2; }
  echo RUNNER_ARGS
  printf '%s\n' "${ARGS[@]}"
  exec "$VENV_PY" "$ROOT/sim/isaac/run_transport.py" "${ARGS[@]}" --check-args
fi
PIN=()  # the session's CPUs when the launcher split them (env.sh SESSION_CPUS)
[ -n "${SESSION_CPUS:-}" ] && PIN=(taskset -c "$SESSION_CPUS")
exec "${PIN[@]}" "$VENV_PY" "$ROOT/sim/isaac/run_transport.py" "${ARGS[@]}" > "$2/run.log" 2>&1
EOT
chmod +x "$OUT/bridge.sh" "$OUT/stop.sh" "$OUT/runner.sh"
# Preflight (docs/plans/2026-10-27-ws1-preflight.md): the session's SLAM bundle as the supervisor will load it, and the
# runner's own checks on the arguments runner.sh builds -- before any container, supervisor or Isaac. DRY_RUN=1 ends
# here with their verdict (tools/isaac_bundle.py check); a real run goes on only when they pass, so a launch refused
# here ends the job in seconds instead of leaving the console probe waiting for a port file (lvram, d7oor 2026-10-27).
preflight() {
  local map_id="" sock="$OUT/epoch_999/sock/slam.sock"
  # an epoch's SLAM socket is a Unix socket: its path fits in 107 bytes or the runner dies at connect (AF_UNIX path
  # too long -- the lvram random_m refill, 11840, at 112)
  if [ "$(printf %s "$sock" | wc -c)" -gt 107 ]; then
    echo "preflight: the SLAM socket path $sock is $(printf %s "$sock" | wc -c) bytes (AF_UNIX allows 107): shorten OUT" >&2
    return 2
  fi
  if [ -n "${MAP_BUNDLE:-}" ]; then
    map_id=$("$VENV_PY" -c 'import sys; from pathlib import Path
from forklift_core.localization.slam_map import load_bundle
print(load_bundle(Path(sys.argv[1]), sys.argv[2] or None).map_id)' "$MAP_BUNDLE" "${EXPECTED_MAP_ID:-}") \
      || { echo "preflight: the SLAM bundle $MAP_BUNDLE does not load as map ${EXPECTED_MAP_ID:-(any)}" >&2; return 2; }
    echo "SLAM_BUNDLE $MAP_BUNDLE $map_id"
  fi
  echo "SLAM_CHOICE $SLAM_CHOICE"
  mkdir -p "$OUT/preflight"
  RUNNER_CHECK=1 bash "$OUT/runner.sh" 0 "$OUT/preflight" "$OUT/preflight/live_port.json" "$map_id" "${MAP_BUNDLE:-}"
}
if [ "${DRY_RUN:-0}" = 1 ]; then
  preflight
  exit $?
fi
if ! preflight > "$OUT/preflight.log" 2>&1; then
  echo "preflight: the launch is refused (its log follows)"
  cat "$OUT/preflight.log"
  exit 2
fi
json() { "$VENV_PY" -c 'import json, sys; print(json.dumps(sys.argv[1:]))' "$@"; }
trap 'for c in $(docker ps -aq --filter "name=slam_bridge_${JOB}_e"); do docker rm -f "$c" >/dev/null; done' EXIT
echo "host=$(hostname) job=$JOB out=$OUT"
SUP_LOC=()
[ -n "${MAP_BUNDLE:-}" ] && SUP_LOC=(--slam-map-bundle "$MAP_BUNDLE")
[ -n "${MAP_BUNDLE:-}" ] && [ -n "$EXPECTED_MAP_ID" ] && SUP_LOC+=(--expected-map-id "$EXPECTED_MAP_ID")
# B-P2: the runner first, the bridge at its estimate
BRIDGE=(bash "$OUT/bridge.sh" '{epoch}' '{epoch_dir}' '{expected_map_id}' '{map_bundle}')
[ -n "${START_POSE:-}" ] && SUP_LOC+=(--relocalize --reloc-wait-s "$RELOC_WAIT_S") && BRIDGE+=('{map_start_pose}')
# Slurm's TERM (or reloc_session.sh ending the launch when its probe fails) ends the supervisor, which ends the runner
# and the bridge. The trap is set before the supervisor starts: a TERM between its start and a later trap killed the
# launcher alone and left the supervisor starting sessions (found in #322's review). A TERM before SUP=$! is latched
# and passed on once the pid is known.
SUP=""
TERMED=0
on_term() { TERMED=1; [ -n "$SUP" ] && kill -TERM "$SUP" 2>/dev/null; }
trap on_term TERM INT
# Between runners the supervisor serves the console's port itself (contract 6): the page and /api/v1/session, so a
# start check that fails before the runner's server opens is shown; after a failed end it stays up STANDBY_LINGER_S
# (300 s: the job holds its slot that long; 0 turns it off).
"$VENV_PY" "$ROOT/tools/live_supervisor.py" --out "$OUT" --port-file "$OUT/live_port.json" "${SUP_LOC[@]}" \
  ${MAX_EPOCHS:+--max-epochs "$MAX_EPOCHS"} \
  --slam-choice "$SLAM_CHOICE" \
  --standby-port "${PORT:-8798}" --web-root "$ROOT/console/web" --standby-linger-s "${STANDBY_LINGER_S:-300}" \
  --bridge "$(json "${BRIDGE[@]}")" \
  --bridge-stop "$(json bash "$OUT/stop.sh" '{epoch}')" \
  --bridge-ready "{epoch_dir}/bridge/bridge_ready" --exit-file "{epoch_dir}/run/exit_code" \
  --runner "$(json bash "$OUT/runner.sh" '{epoch}' '{epoch_dir}' '{port_file}' '{expected_map_id}' '{map_bundle}')" &
if [ -n "${LIVE_TEST_HOLD_FILE:-}" ]; then  # tests only (test_isaac_live_term.py): hold between the start and SUP=$!
  touch "$LIVE_TEST_HOLD_FILE.at"
  for _ in $(seq 1 600); do [ -e "$LIVE_TEST_HOLD_FILE" ] && break; sleep 0.05; done
fi
SUP=$!
[ "$TERMED" = 1 ] && kill -TERM "$SUP" 2>/dev/null  # a TERM that came before its pid was known
# Progress watchdog (tools/stall_watch.py): when no epoch's run/live_log.jsonl has changed for STALL_MIN minutes
# (default 10, 0 off) the runner gets SIGUSR1 -- its stack goes to run.stall_stack.txt -- and the supervisor TERM, as
# on Slurm's TERM; $OUT/stall.jsonl says so. A live session never went 3.3 s without a record in 30 runs (first record
# 38-50 s after its start); 11119 (2026-10-26) sat silent for 34 min mid-transport until its time limit.
"$VENV_PY" "$ROOT/tools/stall_watch.py" --glob "$OUT/epoch_*/run/live_log.jsonl" --stall-min "${STALL_MIN:-10}" \
  --term-pid "$SUP" --marker "$OUT/stall.jsonl" &
# A trapped TERM cuts a wait short: wait again until the supervisor has ended (a second TERM too), then take its own code
while kill -0 "$SUP" 2>/dev/null; do
  wait "$SUP"
done
wait "$SUP"
rc=$?
echo "supervisor exit $rc $(date -Is)"
exit "$rc"
