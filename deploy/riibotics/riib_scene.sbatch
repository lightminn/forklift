#!/bin/bash
#SBATCH --job-name=riib_scene
#SBATCH --partition=compute
#SBATCH --gres=shard:2
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=01:30:00
# One Riibotics demo scene (tools/riibotics_isaac_scenes.py) as an Isaac autonomous mission (--layout scene: the
# scene's layout is the planner's prior map). Defaults (user decision 2026-10-22, deploy/riibotics/README.md):
#   MODE=slam      far recognition (learned proposals), SLAM feedback through the slam_toolbox bridge, D7 docking
#   NEAR=on        PR #88's near re-capture: --near-capture-expected on, --near-capture-lower waive (plan
#                  2026-10-27-bottomless-and-bundled-pallets 2.1 c: no lower gate, a bottomless pallet is a target),
#                  --near-capture-redraw always, --near-capture-redraw-lateral-m 0.10 (slam mode only: they need the
#                  perception planning target)
#   NEAR_REDRAW_LATERAL=0.10  the near re-draw's lateral box (--near-capture-redraw-lateral-m); 0.05 is the only value
#                  --near-tracking takes (near tracking re-measure, docs/plans/2026-10-27-insertion-lateral-drift.md
#                  5.3); needs NEAR=on
#   NEAR_RECAPTURE_GAP=  (empty) C3: with a gap in metres, a second near capture where the estimated fork tip is that
#                  far before the face (--near-recapture-gap-m; docs/plans/2026-10-27-insertion-lateral-drift.md T2); needs
#                  NEAR=on
#   MARKERS=on     line_* / random_* run their slot-marker copy <scene>_m.json (PR #99); the other scenes have none
#   RACK=on        aisle / aisle_out run their rack copy <scene>_rack.json: rack uprights, wider slot gaps and the
#                  0.20 m back gap (user decision 2026-10-22, docs/plans/2026-10-22-riibotics-aisle.md D-A, D-C)
#   KEEPOUT=0      --lidar-keepout-m: the planner's searches keep the LiDAR-visible props this much wider (P1c,
#                  docs/plans/2026-10-23-planner-permission-clearance.md); 0 plans as before. 0.15 by default for the
#                  aisle's rack copy only (aisle_rack, release off, within the same batches: transports passed 0/9 at 0
#                  against 7/14 at 0.15, docs/validation/2026-10-24-start-band-release.md); every other scene 0 (trailer:
#                  no gain shown). KEEPOUT=<metres> overrides it for any scene
#   NOISE=const    the obstacle layer config/obstacle_layer_single.yaml (sigma 0.02, cut 0.06 at every range). NOISE=a2
#                  runs config/obstacle_layer_single_a2.yaml: LiDAR range noise and the grid's range bound from Slamtec's A2
#                  spec rows (P3a, same plan). It also feeds SLAM and the scan docking (shared scan), whose acceptance under
#                  it is not settled -- const by default (user decision 2026-10-24, docs/validation/2026-10-24-p3a-a2-range-bound.md)
#   ENCODER_MODEL=team_spec_quantized  the SLAM odometry's encoders: the joint-angle model (MT6701 counts, KTH7801 steering;
#                  plan 2026-10-25-team-spec-default, lead decision); reported on request -- the engine's joint velocities, the
#                  model of every Riibotics result before 2026-10-25 (never mixed with the default's). The obstacle layer
#                  takes the model's own odometry age table (the runner refuses a mismatched one)
#   START_BAND=off --start-band-release: an occupied cell the stopped truck already stands within START_BAND_M of
#                  does not block moving away from it (docs/plans/2026-10-24-start-band-release.md); START_BAND_M=0.03
#                  --start-band-m when on. Off: its Isaac comparison with ESCAPE_PRESTEER (C4,
#                  docs/validation/2026-10-28-escape-presteer.md) did not pass
#   ESCAPE_PRESTEER=off --escape-presteer: no escape qualifying at the held steering, the steering is set standing to
#                  the side whose escape qualifies and the escape evaluated again at the measured steering
#                  (docs/plans/2026-10-27-escape-presteer.md D2, D3); off as the C4 comparison decided
#   ESCAPE_START=off --escape-start-release: the P6 escape's sweep alone passes the trigger's rectangles the stop
#                  already stands within the sweep pad of while it moves away; the permission keeps its own start rule
#                  (docs/plans/2026-10-25-escape-start-release.md); the default follows its Isaac comparison
#   RESTART_CONFIRM=3 --restart-confirm-scans: after an occupied stop, this many new scans in a row must allow a
#                  restart (docs/plans/2026-10-25-restart-confirmation.md); 3: in the rack aisle it held the stops it
#                  armed on with no new failure and no fewer completions (docs/validation/2026-10-26-restart-confirmation.md);
#                  RESTART_CONFIRM=0 as before
#   LOW_CLEARANCE=on --low-clearance-guard: the props below the LiDAR plane watched along the stop arc of the
#                  estimated pose, the speed capped before the planned clearance is used up
#                  (docs/plans/2026-10-25-low-clearance-guard.md); on: no low-prop overlap and no fewer completions
#                  in the on/off comparison (docs/validation/2026-10-25-low-clearance-guard.md); LOW_CLEARANCE=off as before
#   D7_HOLD=on    --d7-reapproach-hold: the SLAM corrections held over a D7 re-approach until its retry match
#                  (docs/plans/2026-10-25-reapproach-slam-hold.md); on: its registered V5 batch passed
#                  (docs/validation/2026-10-25-reapproach-slam-hold.md); D7_HOLD=off as before
#   EXIT_EXEMPT=on (MODE=slam; off for MODE=truth) --exit-prefix-exempt: the transport's exit prefix driven under the
#                  extract rules (docs/plans/2026-10-28-exit-prefix-extract-rules.md); on: its registered batch passed
#                  rule ① in the slam mode (docs/validation/2026-10-31-riib-default-batch2.md); MODE=truth was not in
#                  that batch and keeps it off unless asked; EXIT_EXEMPT=off as before. The runner refuses it with
#                  --near-tracking, so EXTRA_ARGS=--near-tracking needs EXIT_EXEMPT=off.
#   RETURN_START=boxed (MODE=slam; off for MODE=truth) --return-start-backoff: a return planned from a stop the planner
#                  refuses (invalid_start, or a start boxed in at its root) backs off straight first and plans again
#                  (docs/plans/2026-11-02-return-start-backoff-boxed.md); boxed: its registered F2'' batch passed in the
#                  slam mode (docs/validation/2026-11-02-return-start-backoff-boxed-batch.md); MODE=truth was not in that
#                  batch and keeps it off unless asked; RETURN_START=on (invalid_start only) or off as before.
#   NEAR_END_STAND=on (MODE=slam; off for MODE=truth) --near-end-stand: the near re-observation leg's end stand outside
#                  the loose bounds is planned again to the same target (C2, docs/plans/2026-11-02-near-leg-end-stand-c2.md);
#                  on: its natural batch passed conditions 2-4 (#267) and its injection batch conditions 1-2
#                  (docs/validation/2026-11-03-near-end-stand-c2-injection-results.md); the runner's default stays off.
#                  NEAR_END_STAND=shadow|off as before; an explicit --near-end-stand in EXTRA_ARGS wins (the runner takes
#                  the last one) and riib_defaults.txt records the value applied. Earlier riib batches ran it off: keep their
#                  snapshots and settings, name both arms' C2 value in every comparison from now on, and never pool
#                  earlier off results with new on results in one denominator (lead decision 2026-11-03).
#   NEAR_OBSERVE_ARRIVAL=0.05,0.05 when the near-end stand applied is on, else off --near-observe-arrival: K1, the
#                  near re-observation leg's loose arrival and C2's outside judgement narrowed to M,RAD until C2 was
#                  tried on the leg (docs/plans/2026-11-13-k1-near-loose-arrival.md); 0.05,0.05: its registered 80-run
#                  comparison judged K1 better (docs/validation/2026-11-14-k1-comparison.md, lead decision 2026-11-14);
#                  the runner's default stays off. NEAR_OBSERVE_ARRIVAL=off as before; an explicit value needs the
#                  near-end stand on; an EXTRA_ARGS --near-observe-arrival wins and riib_defaults.txt records the value
#                  applied. Earlier batches ran it off: name both arms' K1 value in every comparison from now on and never
#                  pool earlier off results with new on results in one denominator.
#   RETURN_CUSP_REPLANS=2 (MODE=slam; 0 for MODE=truth) --return-cusp-replans: a return_home cusp heading failure is
#                  braked and the return planned again from the stand, at most N times per return
#                  (docs/plans/2026-10-31-return-cusp-replan.md D1); 2: its re-registered batch passed in the slam mode
#                  (docs/validation/2026-11-04-return-cusp-replan-reregistration-results.md); the runner's default stays 0.
#                  RETURN_CUSP_REPLANS=0 is the earlier behaviour; an explicit --return-cusp-replans in EXTRA_ARGS wins (the
#                  runner takes the last one; the launcher then adds none of its own) and riib_defaults.txt records the value
#                  applied. Earlier riib batches ran 0: name both arms' value in every comparison and never pool 0 results
#                  with 2 results in one denominator. The near-tracking contract was measured at 0, so EXTRA_ARGS=--near-tracking
#                  needs RETURN_CUSP_REPLANS=0 (the runner refuses another value); bundle lists name the full flag.
#   DOCKING_DEGENERATE_CONTINUE=on only for MODE=slam with the final scene NAME line_2_m (an allow list; off for every
#                  other scene -- the other lines, random, plain copies, trailer, conveyor, the aisles, unregistered
#                  scenes -- and for MODE=truth) --docking-degenerate-continue: a first round-1 delivery match refused as
#                  degenerate at every start drives to the 0.6 m round-2 stop and matches there
#                  (docs/plans/2026-11-04-line2-delivery-robustness.md); on: a limited defence against a rare failure,
#                  taken on the 7 rescued pairs under the along-track prior-error stress (+8 cm) of its registered batch
#                  (docs/validation/2026-11-05-line2-robustness-v3.md, lead decision 2026-11-05); the runner's default
#                  stays off. A scene joins the list only with its own verified stop -> match -> re-approach result, not
#                  by its name or an estimated margin. An explicit --docking-degenerate-continue in EXTRA_ARGS wins (the
#                  runner takes the last one; the launcher then adds none of its own) and riib_defaults.txt records the
#                  value applied. on needs --d7-docking (MODE=slam), and the near-tracking contract was measured off, so
#                  EXTRA_ARGS=--near-tracking on line_2_m needs DOCKING_DEGENERATE_CONTINUE=off. Earlier riib line_2_m
#                  batches ran off: name both arms' value in every comparison and never pool off results with on results.
# Regression runs choose the older combinations explicitly: MODE=truth (truck and pallet poses as truth, no camera,
# no SLAM -- the fleet demo setting), NEAR=off, MARKERS=off, RACK=off, KEEPOUT=0. Anything else goes through EXTRA_ARGS.
# shard:2 -- Isaac with the learned proposal worker peaks at 3.9 GB of a 4 GB shard (PR #98); a MODE=truth run may ask
# for one with sbatch --gres=shard:1.
# Several runs in one job (deploy/slurm/isaac_bundle.sbatch, docs/plans/2026-10-24-ws1-isaac-bundle.md): the bundle runner
# sets SESSION_ID (this run's bridge container name) and BRIDGE_CPU (its cpuset); sizes riib = 3 runs per GPU (a run
# with its video holds 4.15-4.47 GB). The runner goes in the background so that a TERM reaches the trap at once.
# DRY_RUN=1 prints the runner's arguments, one per line, and starts nothing.
#
# Usage (paths of this machine come from the caller, nothing host-specific is kept here):
#   SNAP=<code checkout> SCENE_DIR=<scene files> SCENE=<name> BASE_SCENE=<scene.usda> FORKLIFT_URDF=<urdf> \
#   VENV_PY=<python> [ONNX=<proposal weights> LEARNED_PY=<python with onnxruntime> BRIDGE_IMAGE=<ros2 image>] \
#   [MODE=slam|truth] [NEAR=on|off] [NEAR_REDRAW_LATERAL=0.10] [VIDEO=on|off] [VIEWS=quarter,chase|none|...] [MARKERS=on|off] [RACK=on|off] [KEEPOUT=0] [NOISE=const|a2] [START_BAND=off|on] [START_BAND_M=0.03] [ESCAPE_PRESTEER=off|on] [ESCAPE_START=off|on] [D7_HOLD=on|off] [EXIT_EXEMPT=on|off] [RETURN_START=on|boxed|off] [NEAR_END_STAND=on|shadow|off] [NEAR_OBSERVE_ARRIVAL=0.05,0.05|off] [RETURN_CUSP_REPLANS=2] [DOCKING_DEGENERATE_CONTINUE=on|off] [GYRO=off|measured] [SEED=0] [MAX_SIM=1500] [EXTRA_ARGS=...] \
#   sbatch --output=<dir>/riib-%j.log deploy/riibotics/riib_scene.sbatch OUT
set -u
OUT=${1:?the run directory}
: "${SNAP:?the code checkout to run}" "${SCENE_DIR:?the scene files}" "${SCENE:?the scene name}"
# Every path absolute before the cd into SNAP below (a relative one would name another directory there).
OUT=$(realpath -m "$OUT"); SNAP=$(realpath -m "$SNAP"); SCENE_DIR=$(realpath -m "$SCENE_DIR")
source "$SNAP/deploy/lib/term_relay.sh" || { echo "no $SNAP/deploy/lib/term_relay.sh" >&2; exit 2; }
# Path display plan (docs/plans/2026-11-14-path-display-perception-leak.md 2, lead decision 2026-11-14): the scene's
# display prims are seen by the perception cameras, so --path-display stage is diagnostic only. The diagnostic
# variable (stage or both) passes on to the runner here only: the S3 alignment smoke; a bundle's check refuses it.
pd_prev=""
for pd_w in $(IFS=$' \t\n'; set -f; printf '%s\n' ${EXTRA_ARGS:-}); do
  pd_o=${pd_w%%=*}
  if { [ "$pd_w" != "$pd_o" ] && [ "${pd_w#*=}" = stage ] && [ ${#pd_o} -ge 4 ] && [[ --path-display == "$pd_o"* ]]; } \
     || { [ "$pd_w" = stage ] && [ ${#pd_prev} -ge 4 ] && [[ --path-display == "$pd_prev"* ]]; }; then
    echo "EXTRA_ARGS --path-display stage: diagnostic only, never in this launcher's runs"; exit 2
  fi
  pd_prev=$pd_w
done
if [ -n "${FORKLIFT_PATH_DISPLAY_DIAGNOSTIC:-}" ]; then  # passed on to the runner, which applies it
  echo "PATH_DISPLAY_DIAGNOSTIC $FORKLIFT_PATH_DISPLAY_DIAGNOSTIC (a diagnostic run, not an evaluation)"
fi
: "${BASE_SCENE:?the base scene (.usda)}" "${FORKLIFT_URDF:?the forklift URDF the scene was built from}"
: "${VENV_PY:?the Isaac python}"
MODE=${MODE:-slam}; NEAR=${NEAR:-on}; MARKERS=${MARKERS:-on}; RACK=${RACK:-on}; NOISE=${NOISE:-const}; START_BAND=${START_BAND:-off}; START_BAND_M=${START_BAND_M:-0.03}; ESCAPE_PRESTEER=${ESCAPE_PRESTEER:-off}; ESCAPE_START=${ESCAPE_START:-off}; LOW_CLEARANCE=${LOW_CLEARANCE:-on}; RESTART_CONFIRM=${RESTART_CONFIRM:-3}; D7_HOLD=${D7_HOLD:-on}
case "$MODE" in slam|truth) ;; *) echo "MODE must be slam or truth"; exit 2;; esac
case "$NEAR" in on|off) ;; *) echo "NEAR must be on or off"; exit 2;; esac
case "$MARKERS" in on|off) ;; *) echo "MARKERS must be on or off"; exit 2;; esac
case "$RACK" in on|off) ;; *) echo "RACK must be on or off"; exit 2;; esac
ENCODER_MODEL=${ENCODER_MODEL:-team_spec_quantized}
case "$ENCODER_MODEL" in reported) AGE=;; team_spec_quantized) AGE=_team_spec;;
  *) echo "ENCODER_MODEL must be reported or team_spec_quantized"; exit 2;; esac
case "$NOISE" in a2) LAYER=config/obstacle_layer_single_a2$AGE.yaml;; const) LAYER=config/obstacle_layer_single$AGE.yaml;;
  *) echo "NOISE must be a2 or const"; exit 2;; esac
# GYRO=measured (T2's C1 arm, docs/plans/2026-10-27-insertion-lateral-drift.md): the odometry heading from the simulated
# D435i gyro on the model fitted to the 2026-10-23 static IMU record, with the age table made for that model -- an
# exploratory comparison, not an adoption (the gyro plan's G4 is open). It needs the perception run (MODE=slam), the
# team-spec encoders and the constant noise (no a2 gyro table).
GYRO=${GYRO:-off}
GYRO_ARGS=()
case "$GYRO" in
  off) ;;
  measured)
    [ "$MODE" = slam ] || { echo "GYRO=measured needs MODE=slam (the runner's gyro heading needs --use-perception)"; exit 2; }
    [ "$ENCODER_MODEL" = team_spec_quantized ] || { echo "GYRO=measured needs ENCODER_MODEL=team_spec_quantized"; exit 2; }
    [ "$NOISE" = const ] || { echo "GYRO=measured has no NOISE=$NOISE age table"; exit 2; }
    LAYER=config/obstacle_layer_single_team_spec_gyro_d435i_static.yaml
    GYRO_ARGS=(--odometry-heading gyro --gyro-model config/gyro_model_d435i_static_20261023.yaml) ;;
  *) echo "GYRO must be off or measured"; exit 2;; esac
# the layer latency budget (docs/plans/2026-11-11-layer-latency-budget.md, the lead's decision 2026-11-11): a file kept
# apart from the obstacle-layer one (a survey bundle's shared input); LAYER_BUDGET=off runs without it
LAYER_BUDGET=${LAYER_BUDGET:-config/layer_latency_budget.yaml}
BUDGET=(--layer-latency-budget "$LAYER_BUDGET")
[ "$LAYER_BUDGET" = off ] && BUDGET=()
case "$START_BAND" in on|off) ;; *) echo "START_BAND must be on or off"; exit 2;; esac
[[ "$START_BAND_M" =~ ^[0-9]+([.][0-9]+)?$ ]] && awk -v b="$START_BAND_M" 'BEGIN{exit !(b > 0)}' \
  || { echo "START_BAND_M must be a positive number of metres"; exit 2; }
case "$ESCAPE_PRESTEER" in on|off) ;; *) echo "ESCAPE_PRESTEER must be on or off"; exit 2;; esac
case "$ESCAPE_START" in on|off) ;; *) echo "ESCAPE_START must be on or off"; exit 2;; esac
case "$LOW_CLEARANCE" in on|off) ;; *) echo "LOW_CLEARANCE must be on or off"; exit 2;; esac
case "$D7_HOLD" in on|off) ;; *) echo "D7_HOLD must be on or off"; exit 2;; esac
if [ "$MODE" = slam ]; then EXIT_EXEMPT=${EXIT_EXEMPT:-on}; else EXIT_EXEMPT=${EXIT_EXEMPT:-off}; fi
case "$EXIT_EXEMPT" in on|off) ;; *) echo "EXIT_EXEMPT must be on or off"; exit 2;; esac
if [ "$MODE" = slam ]; then RETURN_START=${RETURN_START:-boxed}; else RETURN_START=${RETURN_START:-off}; fi
case "$RETURN_START" in on|boxed|off) ;; *) echo "RETURN_START must be on, boxed or off"; exit 2;; esac
if [ "$MODE" = slam ]; then NEAR_END_STAND=${NEAR_END_STAND:-on}; else NEAR_END_STAND=${NEAR_END_STAND:-off}; fi
case "$NEAR_END_STAND" in on|shadow|off) ;; *) echo "NEAR_END_STAND must be on, shadow or off"; exit 2;; esac
NEAR_END_STAND_FROM=launcher  # or EXTRA_ARGS, when it names its own (the runner takes the last --near-end-stand)
if [ "$MODE" = slam ]; then RETURN_CUSP_REPLANS=${RETURN_CUSP_REPLANS:-2}; else RETURN_CUSP_REPLANS=${RETURN_CUSP_REPLANS:-0}; fi
[[ "$RETURN_CUSP_REPLANS" =~ ^[0-9]+$ ]] || { echo "RETURN_CUSP_REPLANS must be a count (0 = off)"; exit 2; }
RETURN_CUSP_REPLANS_FROM=launcher  # or EXTRA_ARGS (the runner takes the last --return-cusp-replans)
[[ "$RESTART_CONFIRM" =~ ^[0-9]+$ ]] || { echo "RESTART_CONFIRM must be 0 (off) or a number of scans"; exit 2; }
# near tracking re-measure (docs/plans/2026-10-27-insertion-lateral-drift.md 5.3): empty = 0.10, the default before it.
NEAR_REDRAW_LATERAL=${NEAR_REDRAW_LATERAL:-}
if [ -n "$NEAR_REDRAW_LATERAL" ]; then
  [[ "$NEAR_REDRAW_LATERAL" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "NEAR_REDRAW_LATERAL must be a positive number of metres"; exit 2; }
  { [ "$MODE" = slam ] && [ "$NEAR" = on ]; } || { echo "NEAR_REDRAW_LATERAL needs MODE=slam and NEAR=on"; exit 2; }
fi
# C3 (docs/plans/2026-10-27-insertion-lateral-drift.md T2): empty = off; a gap needs the near capture it follows.
NEAR_RECAPTURE_GAP=${NEAR_RECAPTURE_GAP:-}
if [ -n "$NEAR_RECAPTURE_GAP" ]; then
  [[ "$NEAR_RECAPTURE_GAP" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "NEAR_RECAPTURE_GAP must be a positive number of metres"; exit 2; }
  { [ "$MODE" = slam ] && [ "$NEAR" = on ]; } || { echo "NEAR_RECAPTURE_GAP needs MODE=slam and NEAR=on"; exit 2; }
fi
NAME=$SCENE
case "$SCENE" in
  *_m) [ "$MARKERS" = off ] && { echo "SCENE=$SCENE is a marker copy but MARKERS=off"; exit 2; } ;;
  line_*|random_*) [ "$MARKERS" = on ] && NAME=${SCENE}_m ;;
  *_rack) [ "$RACK" = off ] && { echo "SCENE=$SCENE is a rack copy but RACK=off"; exit 2; } ;;
  aisle|aisle_out) [ "$RACK" = on ] && NAME=${SCENE}_rack ;;
esac
# the degenerate-continue allow list is on the final NAME (line_2 with its marker copy), in the slam mode only
if [ "$MODE" = slam ] && [ "$NAME" = line_2_m ]; then DDC_DEFAULT=on; else DDC_DEFAULT=off; fi
DOCKING_DEGENERATE_CONTINUE=${DOCKING_DEGENERATE_CONTINUE:-$DDC_DEFAULT}
case "$DOCKING_DEGENERATE_CONTINUE" in on|off) ;; *) echo "DOCKING_DEGENERATE_CONTINUE must be on or off"; exit 2;; esac
DOCKING_DEGENERATE_CONTINUE_FROM=launcher  # or EXTRA_ARGS (the runner takes the last --docking-degenerate-continue)
# the aisle's rack copy keeps its uprights 0.15 m wider in the searches; every other scene plans as before
[ "$NAME" = aisle_rack ] && KEEPOUT=${KEEPOUT:-0.15}
KEEPOUT=${KEEPOUT:-0}
[[ "$KEEPOUT" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "KEEPOUT must be a non-negative number of metres"; exit 2; }
# scene requirements (config/scene_requirements.yaml, docs/plans/2026-10-27-scene-requirements.md): trailer's loaded
# transport needs an exit prefix -- the fixed 0.30 m (PR #91) unless the keepout builds its own, or EXTRA_ARGS sets
# either (it comes last and decides); the runner refuses a trailer run with neither before Isaac starts
# the extra videos beside the overview (lead 2026-10-27): quarter,chase as every earlier batch; a comparison batch that
# needs no videos sets VIEWS=none in its list for both arms (one encoder and one camera render less per view)
# VIDEO=off (lead 2026-10-28, near tracking re-measure): no video at all -- no overview and no extra views, so
# nothing but the near camera renders (with video the near camera's frame period wandered 10-15 ticks; a real truck
# renders no video either). The bundle still watches run/frames.jsonl: --frame-log writes its pose line per frame tick.
VIDEO=${VIDEO:-on}
case "$VIDEO" in on|off) ;; *) echo "VIDEO must be on or off"; exit 2;; esac
if [ "$VIDEO" = off ]; then
  case "${VIEWS:-none}" in none) ;; *) echo "VIDEO=off has no extra views: VIEWS must be none or unset"; exit 2;; esac
  VIEWS=none
fi
VIEWS=${VIEWS:-quarter,chase}
case "$VIEWS" in none|quarter|chase|quarter,chase|chase,quarter) ;; *) echo "VIEWS must be none, quarter, chase or quarter,chase"; exit 2;; esac
EXIT_CLEARANCE=${EXIT_CLEARANCE:-}
EXTRA_SETS_EXIT=0  # EXTRA_ARGS's words, split as the runner gets them (spaces, tabs, newlines), not globbed here
set -f
NES_NEXT=0
NOA_NEXT=0
NEAR_OBSERVE_ARRIVAL_FROM=launcher  # or EXTRA_ARGS, when it names its own (the runner takes the last one)
RCR_NEXT=0
DDC_NEXT=0
EXTRA_NEAR_TRACKING=0  # --near-tracking among EXTRA_ARGS's words (spaces, tabs or newlines around it)
# shellcheck disable=SC2086
for w in ${EXTRA_ARGS:-}; do
  # C2: an explicit --near-end-stand in EXTRA_ARGS is the value applied (the runner's parser takes a unique prefix and
  # the last occurrence); recorded below as what the run got
  if [ "$NES_NEXT" = 1 ]; then NEAR_END_STAND=$w; NEAR_END_STAND_FROM=EXTRA_ARGS; NES_NEXT=0; continue; fi
  k=${w%%=*}
  if [[ "$k" == --near-e* && "--near-end-stand" == "$k"* ]]; then
    if [[ "$w" == *=* ]]; then NEAR_END_STAND=${w#*=}; NEAR_END_STAND_FROM=EXTRA_ARGS; else NES_NEXT=1; fi
    continue
  fi
  # K1: the same for --near-observe-arrival (--near-o... can only be that)
  if [ "$NOA_NEXT" = 1 ]; then NEAR_OBSERVE_ARRIVAL=$w; NEAR_OBSERVE_ARRIVAL_FROM=EXTRA_ARGS; NOA_NEXT=0; continue; fi
  if [[ "$k" == --near-o* && "--near-observe-arrival" == "$k"* ]]; then
    if [[ "$w" == *=* ]]; then NEAR_OBSERVE_ARRIVAL=${w#*=}; NEAR_OBSERVE_ARRIVAL_FROM=EXTRA_ARGS; else NOA_NEXT=1; fi
    continue
  fi
  # the same for the return cusp replan budget (--return-c... can only be --return-cusp-replans)
  if [ "$RCR_NEXT" = 1 ]; then RETURN_CUSP_REPLANS=$w; RETURN_CUSP_REPLANS_FROM=EXTRA_ARGS; RCR_NEXT=0; continue; fi
  if [[ "$k" == --return-c* && "--return-cusp-replans" == "$k"* ]]; then
    if [[ "$w" == *=* ]]; then RETURN_CUSP_REPLANS=${w#*=}; RETURN_CUSP_REPLANS_FROM=EXTRA_ARGS; else RCR_NEXT=1; fi
    continue
  fi
  # and the degenerate-continue switch (--docking-d... can only be --docking-degenerate-continue)
  if [ "$DDC_NEXT" = 1 ]; then DOCKING_DEGENERATE_CONTINUE=$w; DOCKING_DEGENERATE_CONTINUE_FROM=EXTRA_ARGS; DDC_NEXT=0; continue; fi
  if [[ "$k" == --docking-d* && "--docking-degenerate-continue" == "$k"* ]]; then
    if [[ "$w" == *=* ]]; then DOCKING_DEGENERATE_CONTINUE=${w#*=}; DOCKING_DEGENERATE_CONTINUE_FROM=EXTRA_ARGS; else DDC_NEXT=1; fi
    continue
  fi
  case "$w" in --lidar-keepout-m|--lidar-keepout-m=*|--exit-clearance-m|--exit-clearance-m=*) EXTRA_SETS_EXIT=1;; esac
  # SCENE (its final NAME) is the scene: the degenerate-continue allow list and riib_defaults.txt read it, so a scene
  # file given past it would run another scene under line_2_m's defaults (Codex promotion review P2); --scene-f... can
  # only be --scene-file
  if [[ "$k" == --scene-f* && "--scene-file" == "$k"* ]]; then
    echo "SCENE= sets the scene, not EXTRA_ARGS's --scene-file (the defaults and riib_defaults.txt follow SCENE)"; exit 2
  fi
  # the runner's parser takes a unique prefix too: --exit-p... can only be --exit-prefix-exempt
  k=${w%%=*}
  if [[ "$k" == --exit-p* && "--exit-prefix-exempt" == "$k"* ]]; then
    echo "EXIT_EXEMPT=on|off sets the exit-prefix exemption, not EXTRA_ARGS (riib_defaults.txt records the switch)"; exit 2
  fi
  if [[ "$k" == --return-s* && "--return-start-backoff" == "$k"* ]]; then
    echo "RETURN_START=on|boxed|off sets the return-start backoff, not EXTRA_ARGS (riib_defaults.txt records the switch)"; exit 2
  fi
  case "$w" in --near-tracking)
    EXTRA_NEAR_TRACKING=1
    [ "$EXIT_EXEMPT" = off ] || { echo "--near-tracking needs EXIT_EXEMPT=off (the runner refuses the exemption with it)"; exit 2; };; esac
done
set +f
case "$NEAR_END_STAND" in on|shadow|off) ;; *) echo "--near-end-stand in EXTRA_ARGS must be on, shadow or off"; exit 2;; esac
# K1 follows the near-end stand applied (the runner refuses it without --near-end-stand on); an explicit value needs on
if [ -z "${NEAR_OBSERVE_ARRIVAL:-}" ]; then
  if [ "$NEAR_END_STAND" = on ]; then NEAR_OBSERVE_ARRIVAL=0.05,0.05; else NEAR_OBSERVE_ARRIVAL=off; fi
elif [ "$NEAR_OBSERVE_ARRIVAL" != off ] && [ "$NEAR_END_STAND" != on ]; then
  echo "NEAR_OBSERVE_ARRIVAL (or EXTRA_ARGS --near-observe-arrival) needs the near-end stand on, not $NEAR_END_STAND"; exit 2
fi
# the shape only: the numbers are the runner's check (parse_bounds_pair takes .05 and 5e-2 too)
[ "$NEAR_OBSERVE_ARRIVAL" = off ] || [[ "$NEAR_OBSERVE_ARRIVAL" =~ ^[^,[:space:]]+,[^,[:space:]]+$ ]] \
  || { echo "NEAR_OBSERVE_ARRIVAL must be M,RAD (metres, radians) or off"; exit 2; }
[[ "$RETURN_CUSP_REPLANS" =~ ^[0-9]+$ ]] || { echo "--return-cusp-replans in EXTRA_ARGS must be a count (0 = off)"; exit 2; }
case " ${EXTRA_ARGS:-} " in *" --near-tracking "*)
  [ "$RETURN_CUSP_REPLANS" = 0 ] || { echo "--near-tracking needs RETURN_CUSP_REPLANS=0 (its contract was measured at 0)"; exit 2; };; esac
case "$DOCKING_DEGENERATE_CONTINUE" in on|off) ;; *) echo "--docking-degenerate-continue in EXTRA_ARGS must be on or off"; exit 2;; esac
if [ "$DOCKING_DEGENERATE_CONTINUE" = on ]; then
  # the round-2 stop and its retry are D7 docking's, which only the slam mode builds (the runner refuses it otherwise)
  [ "$MODE" = slam ] || { echo "DOCKING_DEGENERATE_CONTINUE=on needs MODE=slam (--d7-docking)"; exit 2; }
  [ "$EXTRA_NEAR_TRACKING" = 0 ] \
    || { echo "--near-tracking needs DOCKING_DEGENERATE_CONTINUE=off (its contract was measured off)"; exit 2; }
fi
if [ "$NAME" = trailer ] && [ -z "$EXIT_CLEARANCE" ] && [ "$EXTRA_SETS_EXIT" = 0 ] \
    && awk -v k="$KEEPOUT" 'BEGIN { exit !(k + 0 == 0) }'; then
  EXIT_CLEARANCE=0.30
fi
[ -z "$EXIT_CLEARANCE" ] || [[ "$EXIT_CLEARANCE" =~ ^[0-9]+([.][0-9]+)?$ ]] \
  || { echo "EXIT_CLEARANCE must be a positive number of metres"; exit 2; }
SCENE_FILE=$SCENE_DIR/$NAME.json
[ -f "$SCENE_FILE" ] || { echo "no scene file $SCENE_FILE (tools/riibotics_isaac_scenes.py --slot-markers / --rack write the _m / _rack copies)"; exit 2; }
export OMNI_KIT_ACCEPT_EULA=YES OMP_NUM_THREADS=2
# A run directory is used once: an earlier run's bridge_ready would pass for this bridge's, its records would mix.
if [ "${DRY_RUN:-0}" != 1 ] && { [ -e "$OUT/run" ] || [ -e "$OUT/bridge/bridge_ready" ]; }; then
  echo "refusing: $OUT holds an earlier run"; exit 2
fi
mkdir -p "$OUT/bridge" "$OUT/sock"
export PYTHONPATH=$SNAP/src
cd "$SNAP"
echo "gpu ${CUDA_VISIBLE_DEVICES:-unset} job ${SLURM_JOB_ID:-} snapshot $SNAP scene $NAME mode $MODE near $NEAR markers $MARKERS rack $RACK keepout $KEEPOUT noise $NOISE start_band $START_BAND $(date -Is)"
printf 'MODE=%s NEAR=%s MARKERS=%s RACK=%s KEEPOUT=%s NOISE=%s START_BAND=%s START_BAND_M=%s ESCAPE_PRESTEER=%s ESCAPE_START=%s LOW_CLEARANCE=%s RESTART_CONFIRM=%s D7_HOLD=%s EXIT_EXEMPT=%s RETURN_START=%s ENCODER_MODEL=%s GYRO=%s LAYER_BUDGET=%s SCENE_FILE=%s\n' \
  "$MODE" "$NEAR" "$MARKERS" "$RACK" "$KEEPOUT" "$NOISE" "$START_BAND" "$START_BAND_M" "$ESCAPE_PRESTEER" "$ESCAPE_START" "$LOW_CLEARANCE" "$RESTART_CONFIRM" "$D7_HOLD" "$EXIT_EXEMPT" "$RETURN_START" "$ENCODER_MODEL" "$GYRO" "$LAYER_BUDGET" \
  "$SCENE_FILE" > "$OUT/riib_defaults.txt"
# C3: recorded only when set, so every earlier run's record reads as before
[ -n "$NEAR_RECAPTURE_GAP" ] && printf 'NEAR_RECAPTURE_GAP=%s\n' "$NEAR_RECAPTURE_GAP" >> "$OUT/riib_defaults.txt"
# C2: the value applied and where it came from (the launcher's default or switch, or EXTRA_ARGS)
printf 'NEAR_END_STAND=%s NEAR_END_STAND_FROM=%s\n' "$NEAR_END_STAND" "$NEAR_END_STAND_FROM" >> "$OUT/riib_defaults.txt"
# K1: the value applied and where it came from
printf 'NEAR_OBSERVE_ARRIVAL=%s NEAR_OBSERVE_ARRIVAL_FROM=%s\n' "$NEAR_OBSERVE_ARRIVAL" "$NEAR_OBSERVE_ARRIVAL_FROM" \
  >> "$OUT/riib_defaults.txt"
# return cusp replan budget: the value applied and where it came from
printf 'RETURN_CUSP_REPLANS=%s RETURN_CUSP_REPLANS_FROM=%s\n' "$RETURN_CUSP_REPLANS" "$RETURN_CUSP_REPLANS_FROM" >> "$OUT/riib_defaults.txt"
printf 'DOCKING_DEGENERATE_CONTINUE=%s DOCKING_DEGENERATE_CONTINUE_FROM=%s\n' "$DOCKING_DEGENERATE_CONTINUE" \
  "$DOCKING_DEGENERATE_CONTINUE_FROM" >> "$OUT/riib_defaults.txt"
[ -n "$NEAR_REDRAW_LATERAL" ] && printf 'NEAR_REDRAW_LATERAL=%s\n' "$NEAR_REDRAW_LATERAL" >> "$OUT/riib_defaults.txt"
# scene requirements: the same, for the exit prefix, a profile and an override reason
[ -n "$EXIT_CLEARANCE" ] && printf 'EXIT_CLEARANCE=%s\n' "$EXIT_CLEARANCE" >> "$OUT/riib_defaults.txt"
[ "$VIEWS" != quarter,chase ] && printf 'VIEWS=%s\n' "$VIEWS" >> "$OUT/riib_defaults.txt"
[ "$VIDEO" = off ] && printf 'VIDEO=off\n' >> "$OUT/riib_defaults.txt"
[ -n "${REQUIREMENTS_PROFILE:-}" ] && printf 'REQUIREMENTS_PROFILE=%s\n' "$REQUIREMENTS_PROFILE" >> "$OUT/riib_defaults.txt"
[ -n "${SCENE_REQUIREMENTS_OVERRIDE:-}" ] \
  && printf 'SCENE_REQUIREMENTS_OVERRIDE=%q\n' "$SCENE_REQUIREMENTS_OVERRIDE" >> "$OUT/riib_defaults.txt"
# The SLAM mode, said and recorded (live-default plan 5d): this launcher maps online. The switch to localization on an
# accepted survey bundle is the live console's (deploy/console/slam_choice.sh); no Riibotics demo scene has one.
if [ "$MODE" = truth ]; then RIIB_SLAM="none (MODE=truth)"
else RIIB_SLAM="mapping (riib_scene.sbatch maps online; no Riibotics demo scene has an accepted survey bundle)"; fi
echo "slam: $RIIB_SLAM"
printf 'SLAM=%s\n' "$RIIB_SLAM" >> "$OUT/riib_defaults.txt"
read QE QT < <("$VENV_PY" -c "import json,sys;v=json.load(open(sys.argv[1]))['views'];print(','.join(map(str,v['quarter_eye'])),','.join(map(str,v['quarter_target'])))" "$SCENE_FILE")
COMMON=(--base-scene "$BASE_SCENE"
  --pallet-urdf sim/models/epal6_pallet/pallet.urdf --pallet-geometry config/pallet_geometry_epal6.yaml
  --forklift-urdf "$FORKLIFT_URDF" --settings config/isaac_transport_measured.yaml
  --delivery-straight-m 1.5 --alignment-straight-m 2.1 --withdrawal-m 0.55 --insertion-reserve-m 0.016
  --layout scene --scene-file "$SCENE_FILE" --fps 60 --max-sim-seconds "${MAX_SIM:-1500}"
  --record-slam --return-home --obstacle-layer "$LAYER" ${BUDGET[@]+"${BUDGET[@]}"} --encoder-model "$ENCODER_MODEL" ${GYRO_ARGS[@]+"${GYRO_ARGS[@]}"} --obstacle-act --seed "${SEED:-0}"
  --lidar-keepout-m "$KEEPOUT" --start-band-release "$START_BAND" --escape-start-release "$ESCAPE_START"
  --escape-presteer "$ESCAPE_PRESTEER"
  --low-clearance-guard "$LOW_CLEARANCE" --restart-confirm-scans "$RESTART_CONFIRM" --d7-reapproach-hold "$D7_HOLD"
  --exit-prefix-exempt "$EXIT_EXEMPT" --return-start-backoff "$RETURN_START")
# C2: the launcher's value only when EXTRA_ARGS names none, so the runner's arguments read one --near-end-stand
[ "$NEAR_END_STAND_FROM" = launcher ] && COMMON+=(--near-end-stand "$NEAR_END_STAND")
# K1: the launcher's value only when EXTRA_ARGS names none, and nothing when off (the runner has no off value)
[ "$NEAR_OBSERVE_ARRIVAL_FROM" = launcher ] && [ "$NEAR_OBSERVE_ARRIVAL" != off ] \
  && COMMON+=(--near-observe-arrival "$NEAR_OBSERVE_ARRIVAL")
# the launcher's budget only when EXTRA_ARGS names none, so the runner's arguments read one --return-cusp-replans
[ "$RETURN_CUSP_REPLANS_FROM" = launcher ] && COMMON+=(--return-cusp-replans "$RETURN_CUSP_REPLANS")
# and one --docking-degenerate-continue
[ "$DOCKING_DEGENERATE_CONTINUE_FROM" = launcher ] && COMMON+=(--docking-degenerate-continue "$DOCKING_DEGENERATE_CONTINUE")
[ "$VIDEO" = on ] && COMMON+=(--video)
case "$VIEWS" in
  none) COMMON+=(--frame-log) ;;  # the bundle watches run/frames.jsonl for progress
  *quarter*) COMMON+=(--extra-views "$VIEWS" --quarter-eye="$QE" --quarter-target="$QT") ;;
  *) COMMON+=(--extra-views "$VIEWS") ;;
esac
[ -n "$EXIT_CLEARANCE" ] && COMMON+=(--exit-clearance-m "$EXIT_CLEARANCE")
[ "$START_BAND" = on ] && COMMON+=(--start-band-m "$START_BAND_M")  # the runner takes a width only with the release
[ -n "${REQUIREMENTS_PROFILE:-}" ] && COMMON+=(--requirements-profile "$REQUIREMENTS_PROFILE")
[ -n "${SCENE_REQUIREMENTS_OVERRIDE:-}" ] && COMMON+=(--scene-requirements-override "$SCENE_REQUIREMENTS_OVERRIDE")
if [ "$MODE" = truth ]; then
  MODE_ARGS=(--d7-planner)
  [ "$NEAR" = on ] && echo "NEAR=on has no effect in MODE=truth (the near re-capture needs the perception target)" >&2
else
  : "${ONNX:?the learned proposal weights (MODE=slam)}" "${LEARNED_PY:?the python with onnxruntime (MODE=slam)}"
  MODE_ARGS=(--use-perception --pallet-prior config/pallet_prior_epal6.yaml --planning-target perception
    --perception-mount carriage_low_measured --depth-quantize-mm 1
    --slam-feedback "$OUT/sock/slam.sock" --slam-map-dir "$OUT/bridge" --slam-noise-seed "${SEED:-0}"
    --d7-planner --d7-docking --recognition far --recognition-proposals learned
    --recognition-onnx "$ONNX" --recognition-learned-python "$LEARNED_PY" --recognition-tau 0.3 --recognition-conf 0.05
    --recognition-pixel-lag-steps 24 --recognition-providers CUDAExecutionProvider,CPUExecutionProvider
    --recognition-device-id 0 --recognition-threads 4)
  if [ "$NEAR" = on ]; then
    MODE_ARGS+=(--near-capture-expected on --near-capture-lower waive
      --near-capture-redraw always --near-capture-redraw-lateral-m "${NEAR_REDRAW_LATERAL:-0.10}")
    # C3 (docs/plans/2026-10-27-insertion-lateral-drift.md T2): NEAR_RECAPTURE_GAP=<m> adds the second near capture.
    [ -n "$NEAR_RECAPTURE_GAP" ] && MODE_ARGS+=(--near-recapture-gap-m "$NEAR_RECAPTURE_GAP")
  fi
fi
# the SLAM socket is a Unix socket: its path fits in 107 bytes or the runner dies at connect (AF_UNIX path too long)
if [ "$MODE" != truth ] && [ "$(printf %s "$OUT/sock/slam.sock" | wc -c)" -gt 107 ]; then
  echo "preflight: the SLAM socket path $OUT/sock/slam.sock is longer than AF_UNIX allows (107 bytes): shorten OUT" >&2
  exit 2
fi
# the preflight's runner must know --check-args: an older one would take it for a Kit argument and start Isaac
if ! grep -q -- '"--check-args"' "$SNAP/sim/isaac/run_transport.py"; then
  if [ "${DRY_RUN:-0}" = 1 ] && [ "${DRY_RUN_CHECK:-1}" != 1 ]; then :; else
    echo "preflight: $SNAP's runner has no --check-args (an older snapshot): refused" >&2
    exit 2
  fi
fi
# shellcheck disable=SC2086  # EXTRA_ARGS is a word list on purpose
if [ "${DRY_RUN:-0}" = 1 ]; then
  echo RUNNER_ARGS
  printf '%s\n' "${COMMON[@]}" "${MODE_ARGS[@]}" ${EXTRA_ARGS:-}
  # preflight (docs/plans/2026-10-27-ws1-preflight.md): the runner's own checks on these arguments, ARGS_OK or exit 2;
  # DRY_RUN_CHECK=0 prints the arguments alone (this launcher's tests of how it builds them)
  [ "${DRY_RUN_CHECK:-1}" = 1 ] || exit 0
  exec "$VENV_PY" "$SNAP/sim/isaac/run_transport.py" "${COMMON[@]}" "${MODE_ARGS[@]}" ${EXTRA_ARGS:-} \
    --output "$OUT/run" --check-args
fi
# the same check before the runner starts: arguments it would refuse end the job here, in seconds
# shellcheck disable=SC2086
if ! "$VENV_PY" "$SNAP/sim/isaac/run_transport.py" "${COMMON[@]}" "${MODE_ARGS[@]}" ${EXTRA_ARGS:-} --output "$OUT/run" \
    --check-args > "$OUT/check_args.log" 2>&1; then
  echo "preflight: the runner refuses these arguments (its check follows)"
  cat "$OUT/check_args.log"
  exit 2
fi
# the SLAM bridge only after the check: arguments refused above start no container
if [ "$MODE" != truth ]; then
  NAME_B=slam_bridge_${SESSION_ID:-${SLURM_JOB_ID:-$$}}_riib
  CPU0=${BRIDGE_CPU:-$(taskset -cp $$ | awk -F': ' '{print $2}' | cut -d, -f1 | cut -d- -f1)}
  docker run -d --name "$NAME_B" --network none --cpuset-cpus "$CPU0" --memory 4g --group-add 2000 -u "$(id -u):$(id -g)" \
    -e HOME=/tmp -v "$SNAP":/workspace:ro -v "$OUT/bridge":/out -v "$OUT/sock":/run/slam \
    "${BRIDGE_IMAGE:-forklift/ros2-dev:jazzy}" bash /workspace/tools/slam_bridge_container.sh /run/slam/slam.sock /out "-0.12,0.0,1.05,0.0" >/dev/null \
    || { echo "bridge container did not start"; exit 1; }
  # The bridge saves its records after the runner disconnects: stop it with time to do so before removing it.
  trap 'docker stop -t "${BRIDGE_STOP_S:-60}" "$NAME_B" >/dev/null 2>&1; docker rm -f "$NAME_B" >/dev/null 2>&1' EXIT
  for i in $(seq 1 120); do [ -f "$OUT/bridge/bridge_ready" ] && break; sleep 1; done
  [ -f "$OUT/bridge/bridge_ready" ] || { echo "bridge not ready"; exit 1; }
fi
# timeout passes a TERM on to the runner: armed before the start, so a TERM however early reaches it
# (deploy/lib/term_relay.sh); before this, a TERM ends the launcher and its EXIT trap stops the bridge, as it did
term_relay_arm
# shellcheck disable=SC2086  # EXTRA_ARGS is a word list on purpose
timeout "${RUN_TIMEOUT:-6600}" "$VENV_PY" "$SNAP/sim/isaac/run_transport.py" "${COMMON[@]}" "${MODE_ARGS[@]}" ${EXTRA_ARGS:-} \
  --output "$OUT/run" > "$OUT/run.log" 2>&1 &
term_relay_hold runner
RUNNER=$!
term_relay_add "$RUNNER"
term_relay_wait "$RUNNER"
rc=$TERM_RELAY_RC
echo "runner exit $rc $(date -Is)"
echo "OUT_OF_DEVICE lines: $(grep -c OUT_OF_DEVICE "$OUT/run.log")"
# The runner also records its own result code as JSON {"code": .., "reason": ..} (Isaac can leave the process with 0
# after a failure).
if [ "$rc" -eq 0 ] && [ -f "$OUT/run/exit_code" ]; then
  recorded=$("$VENV_PY" -c 'import json,sys; print(int(json.load(open(sys.argv[1])).get("code") or 0))' "$OUT/run/exit_code") \
    || recorded=1
  if [ "$recorded" != 0 ]; then rc=$recorded; echo "runner recorded exit code $rc"; fi
fi
exit "$rc"
