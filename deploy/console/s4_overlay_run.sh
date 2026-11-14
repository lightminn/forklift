#!/bin/bash
#SBATCH --job-name=s4_overlay
#SBATCH --partition=compute
#SBATCH --gres=shard:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=7G
#SBATCH --time=01:15:00
#SBATCH --signal=B:TERM@120
# One S4 live session (docs/plans/2026-11-15-s4-live-overlay-registration.md): the console launcher
# (deploy/console/isaac_live.sbatch) with a known start on the registry's survey map, one epoch, no standby; the console
# probe's cycle scenario runs one mission and ends the session (tools/live_console_probe.py); tools/live_frames.py
# capture saves the console's camera frames from the port the running session wrote, and ends when the runner closes
# (1001). Modelled on deploy/console/reloc_session.sh, which needs START_POSE and has no cycle mode.
#
#   ARM=A|B VENV_PY=.. BASE_SCENE=.. FORKLIFT_URDF=.. SCENE_FILE=.. SLAM_BUNDLE_ROOT=.. \
#     sbatch --chdir=<snapshot> deploy/console/s4_overlay_run.sh OUT
#   ARM A: --path-display at its default (overlay); B: --path-display off; D: the both diagnostic (a *diag* snapshot
#   only, FORKLIFT_PATH_DISPLAY_DIAGNOSTIC=both). All: --max-sim-seconds 1200 and
#   FORKLIFT_LIVE_TICK_TRACE=1. Optional: SEED (0), FACTOR (0.4), CYCLE (P-4:D-1, random_m's first CT leg), MISSION_TIMEOUT_S (2400),
#   WAIT_PORT_S (1500), END_GRACE_S (300), CAPTURE_END_S (60),
#   FIRST_FRAME_S (120). DRY_RUN=1: print what would run, run and write nothing.
# Run from the snapshot root.
set -euo pipefail
OUT=$(realpath -m "${1:?output directory}")
ROOT=$(pwd)
ARM=${ARM:?ARM=A (overlay, the default) or B (--path-display off)}
case "$ARM" in
  A) ARM_ARGS="" ;;
  B) ARM_ARGS="--path-display off" ;;
  D)
    # the S4 both diagnostic (docs/plans/2026-11-15-s4-both-diagnostic.md, lead approval for one run): display prims
    # rendered in the scene + the 1 px magenta overlay at lag 0, from a diag snapshot whose launcher copy has its
    # diagnostic-variable refusal removed (DIAG_PATCH.txt there); never from an operating checkout
    ARM_ARGS=""
    case "$(basename "$ROOT")" in *diag*) ;; *) echo "ARM=D runs only from a snapshot named *diag*" >&2; exit 2 ;; esac
    [ -f "$ROOT/DIAG_PATCH.txt" ] || { echo "ARM=D: no $ROOT/DIAG_PATCH.txt" >&2; exit 2; }
    if grep -q 'FORKLIFT_PATH_DISPLAY_DIAGNOSTIC is diagnostic only' "$ROOT/deploy/console/isaac_live.sbatch"; then
      echo "ARM=D: this snapshot's launcher still refuses the diagnostic variable" >&2; exit 2
    fi
    # the launcher must be the one the patch record names (after_sha256), not any copy without the refusal
    want=$(awk '$1 == "after_sha256" {print $2}' "$ROOT/DIAG_PATCH.txt")
    have=$(sha256sum "$ROOT/deploy/console/isaac_live.sbatch" | awk '{print $1}')
    [ -n "$want" ] && [ "$want" = "$have" ] || {
      echo "ARM=D: the launcher's sha256 $have is not DIAG_PATCH.txt's after_sha256 ${want:-<none>}" >&2; exit 2; }
    export FORKLIFT_PATH_DISPLAY_DIAGNOSTIC=both ;;
  *) echo "ARM must be A, B or D" >&2; exit 2 ;;
esac
if [ -n "${EXTRA_ARGS:-}" ]; then
  echo "EXTRA_ARGS is this wrapper's (per ARM): unset it" >&2; exit 2
fi
: "${VENV_PY:?the Isaac Python}" "${BASE_SCENE:?}" "${FORKLIFT_URDF:?}" "${SCENE_FILE:?a registry scene file}"
: "${SLAM_BUNDLE_ROOT:?where the registry bundle folders are}"
CYCLE=${CYCLE:-P-4:D-1}  # random_m's CT order starts P-4:D-1; P-1 has no valid docking goal from the known start
MISSION_TIMEOUT_S=${MISSION_TIMEOUT_S:-2400}
WAIT_PORT_S=${WAIT_PORT_S:-1500}
END_GRACE_S=${END_GRACE_S:-300}
CAPTURE_END_S=${CAPTURE_END_S:-60}
FIRST_FRAME_S=${FIRST_FRAME_S:-120}
export VENV_PY BASE_SCENE FORKLIFT_URDF SCENE_FILE SLAM_BUNDLE_ROOT
export START_RELOC=off MAP_BUNDLE=registry MAX_EPOCHS=1 STANDBY_LINGER_S=0 FORKLIFT_LIVE_TICK_TRACE=1
export SEED=${SEED:-0} FACTOR=${FACTOR:-0.4}
# a scene-file session needs --near-capture-redraw always (the runner refuses otherwise; as the B-P2e smoke did)
export EXTRA_ARGS="${ARM_ARGS:+$ARM_ARGS }--near-capture-redraw always --max-sim-seconds 1200"

LAUNCHER=(bash "$ROOT/deploy/console/isaac_live.sbatch" "$OUT")
PROBE=("$VENV_PY" "$ROOT/tools/live_console_probe.py" --port-file "$OUT/live_port.json" --report "$OUT/probe.json"
       --scenario cycle --cycle "$CYCLE" --end-session --mission-timeout-s "$MISSION_TIMEOUT_S"
       --wait-port-s "$WAIT_PORT_S")
CAPTURE=("$VENV_PY" "$ROOT/tools/live_frames.py" capture --out "$OUT/frames")

if [ "${DRY_RUN:-0}" = 1 ]; then
  echo "ARM=$ARM ROOT=$ROOT OUT=$OUT"
  for v in START_RELOC MAP_BUNDLE MAX_EPOCHS STANDBY_LINGER_S FORKLIFT_LIVE_TICK_TRACE SEED FACTOR EXTRA_ARGS \
           SCENE_FILE SLAM_BUNDLE_ROOT BASE_SCENE FORKLIFT_URDF VENV_PY; do
    printf '%s=%q\n' "$v" "${!v}"
  done
  printf 'FORKLIFT_PATH_DISPLAY_DIAGNOSTIC=%s\n' "${FORKLIFT_PATH_DISPLAY_DIAGNOSTIC-unset}"
  printf 'launcher:'; printf ' %q' "${LAUNCHER[@]}"; echo " > $OUT/launcher.log"
  printf 'capture (once %s says running):' "$OUT/live_port.json"; printf ' %q' "${CAPTURE[@]}"
  echo " --url ws://<host>:<port> > $OUT/capture.log"
  printf 'probe (after the first captured frame, at most %s s):' "$FIRST_FRAME_S"; printf ' %q' "${PROBE[@]}"
  echo " --launcher-pid <launcher pid> > $OUT/probe.log"
  exit 0
fi

mkdir -p "$OUT"
if [ -n "$(find "$OUT" -mindepth 1 -print -quit)" ]; then
  echo "$OUT is not empty" >&2; exit 2
fi
source "$ROOT/deploy/lib/term_relay.sh" || { echo "no $ROOT/deploy/lib/term_relay.sh" >&2; exit 2; }
log() { echo "$(date -Is) $*" | tee -a "$OUT/s4_wrapper.log"; }
{
  for v in ARM START_RELOC MAP_BUNDLE MAX_EPOCHS STANDBY_LINGER_S FORKLIFT_LIVE_TICK_TRACE SEED FACTOR EXTRA_ARGS \
           SCENE_FILE SLAM_BUNDLE_ROOT BASE_SCENE FORKLIFT_URDF VENV_PY CYCLE MISSION_TIMEOUT_S; do
    printf '%s=%q\n' "$v" "${!v}"
  done
  printf 'FORKLIFT_LIVE_CATCHUP_STEPS=%s\n' "${FORKLIFT_LIVE_CATCHUP_STEPS-unset}"
  printf 'FORKLIFT_LIVE_CAMERA_PHASE=%s\n' "${FORKLIFT_LIVE_CAMERA_PHASE-unset}"
  printf 'FORKLIFT_PATH_DISPLAY_DIAGNOSTIC=%s\n' "${FORKLIFT_PATH_DISPLAY_DIAGNOSTIC-unset}"
  [ ! -f "$ROOT/DIAG_PATCH.txt" ] || cat "$ROOT/DIAG_PATCH.txt"
  sha256sum "$0" "$ROOT/deploy/console/isaac_live.sbatch" "$ROOT/tools/live_console_probe.py" \
    "$ROOT/tools/live_frames.py" "$SCENE_FILE"
} > "$OUT/args.txt"
log "host=$(hostname) job=${SLURM_JOB_ID:-} arm=$ARM out=$OUT"

port_info() {  # "state host port" of the port file, or nothing
  "$VENV_PY" - "$OUT/live_port.json" <<'EOF' 2>/dev/null || true
import json, sys
try:
    d = json.load(open(sys.argv[1]))
    print(d.get("state"), d.get("host"), d.get("port"))
except Exception:
    pass
EOF
}

term_relay_arm
"${LAUNCHER[@]}" > "$OUT/launcher.log" 2>&1 &
L=$!
term_relay_add "$L"

# the capture starts once the runner's own server is up (the port file says running): the supervisor's standby on the
# same port serves no media socket. The probe (which submits the mission) starts only after the capture has saved its
# first frame, so the mission's frames are all captured (Codex S4 review P2-1)
C=""
deadline=$(( SECONDS + WAIT_PORT_S ))
while [ "$SECONDS" -lt "$deadline" ] && [ "$TERM_RELAY_GOT" = 0 ] && kill -0 "$L" 2>/dev/null; do
  read -r st host port <<< "$(port_info)" || true
  if [ "${st:-}" = running ] && [ -n "${port:-}" ]; then
    "${CAPTURE[@]}" --url "ws://${host:-127.0.0.1}:$port" > "$OUT/capture.log" 2>&1 &
    C=$!
    term_relay_add "$C"
    log "capture started on ws://${host:-127.0.0.1}:$port"
    break
  fi
  sleep 1
done
[ -n "$C" ] || log "no running session within ${WAIT_PORT_S} s (or the launcher ended): no capture"
if [ -n "$C" ]; then
  for _ in $(seq 1 $(( FIRST_FRAME_S * 10 ))); do
    grep -q '"t": "frame"' "$OUT/frames/index.jsonl" 2>/dev/null && break
    kill -0 "$C" 2>/dev/null || break
    sleep 0.1
  done
  if grep -q '"t": "frame"' "$OUT/frames/index.jsonl" 2>/dev/null; then
    log "capture has its first frame: starting the probe"
  else
    log "no captured frame within ${FIRST_FRAME_S} s: starting the probe anyway (the run is void, registration 6)"
  fi
fi
"${PROBE[@]}" --launcher-pid "$L" > "$OUT/probe.log" 2>&1 &
P=$!
term_relay_add "$P"
log "probe started"

end_launcher() {
  kill -TERM "$L" 2>/dev/null || true
  for _ in $(seq 1 1200); do kill -0 "$L" 2>/dev/null || return 0; sleep 0.1; done
  kill -KILL "$L" 2>/dev/null || true
}

term_relay_wait "$P" || true
prc=$TERM_RELAY_RC
log "probe exit $prc"
if [ "$prc" -ne 0 ] && [ "$prc" -ne 3 ]; then
  log "probe failed: ending the launcher"
  end_launcher
elif [ "$TERM_RELAY_GOT" = 0 ]; then
  for _ in $(seq 1 $(( END_GRACE_S * 10 ))); do kill -0 "$L" 2>/dev/null || break; sleep 0.1; done
  if kill -0 "$L" 2>/dev/null; then
    log "launcher still up ${END_GRACE_S} s after the probe: ending it"
    end_launcher
  fi
fi
term_relay_wait "$L" || true
lrc=$TERM_RELAY_RC
log "launcher exit $lrc"

crc=0
if [ -n "$C" ]; then
  # the capture ends by itself on the runner's 1001 close; else TERM (it writes its end row on TERM)
  for _ in $(seq 1 $(( CAPTURE_END_S * 10 ))); do kill -0 "$C" 2>/dev/null || break; sleep 0.1; done
  if kill -0 "$C" 2>/dev/null; then
    log "capture still up ${CAPTURE_END_S} s after the launcher: TERM"
    kill -TERM "$C" 2>/dev/null || true
  fi
  term_relay_wait "$C" || true
  crc=$TERM_RELAY_RC
  log "capture exit $crc; end row: $(tail -n1 "$OUT/frames/index.jsonl" 2>/dev/null || echo none)"
fi

if [ "$prc" -ne 0 ] && [ "$prc" -ne 3 ]; then exit 4; fi
if [ "$lrc" -ne 0 ]; then exit "$lrc"; fi
if [ -z "$C" ] || [ "$crc" -ne 0 ]; then exit 5; fi
exit 0
