#!/bin/bash
# One unattended start relocalization session for B-P2d (docs/plans/2026-10-29-start-relocalization.md 4.4): the console
# launcher (deploy/console/isaac_live.sbatch) with START_POSE, RELOC_UNATTENDED=1 and MAX_EPOCHS=1 -- a failure is the
# session's outcome, never retried -- and the console probe's hold scenario: it waits for the session to run (the port
# file says running only once localized), stands HOLD_SIM_S simulated seconds and ends the session; a session that ends
# before (any relocalization ending) ends the probe with it. The verdict is read afterwards from the records
# (tools/reloc_judge.py). A probe that fails ends the launcher at once (nothing else would end a live session), and a
# launcher still up RELOC_END_GRACE_S after the probe's normal end is ended too (Codex B-P2d tooling review P2).
#
#   START_POSE=x,y,yaw <isaac_live.sbatch's environment> bash deploy/console/reloc_session.sh OUT
#   optional: HOLD_SIM_S (10), WAIT_PORT_S (1500), RELOC_END_GRACE_S (300), RELOC_END_KILL_S (120); RELOC_LAUNCHER
#   (tests only)
#   RELOC_PROBE (hold): place -- F1 (docs/plans/2026-10-31-operator-placed-start.md 4.2 b): the session is attended
#   (no RELOC_UNATTENDED: the console waits for the operator) and the probe is the operator, placing the click model's
#   draws around START_POSE (CLICK_SEED, CLICK_INDEX: the start's index in its list) and confirming the first preview;
#   CLICK_KIND (development only) places that record click (tools/f1_place_eval.py STRESS_KINDS) instead
#   reverse / reverse_place (docs/plans/2026-11-06-unconfirmed-validation.md 2.1): with POSE_CHECK on or shadow, the
#   probe backs the truck up by the pressing rule REVERSE_KIND (full | half | chatter) and, with MISSION_AFTER_CONFIRM=1,
#   runs the first pallet mission once confirmed (the shadow: after the strokes) and stands POST_MISSION_SIM_S (20)
#   after it -- unattended, or after the placement as place does (reverse_place)
#
# Run from the repository root.
set -u
OUT=$(realpath -m "${1:?output directory}")
ROOT=$(pwd)
source "$ROOT/deploy/lib/term_relay.sh" || { echo "no $ROOT/deploy/lib/term_relay.sh" >&2; exit 2; }
: "${START_POSE:?the placed start x,y,yaw (world base_link)}" "${VENV_PY:?the Isaac Python}"
LAUNCHER=${RELOC_LAUNCHER:-$ROOT/deploy/console/isaac_live.sbatch}
PROBE_MODE=${RELOC_PROBE:-hold}
case "$PROBE_MODE" in
  hold) UNATTENDED=1; PROBE_ARGS=(--scenario hold) ;;
  place) UNATTENDED=; PROBE_ARGS=(--scenario place "--truth=$START_POSE" --click-seed "${CLICK_SEED:-0}"
                                  --click-index "${CLICK_INDEX:-0}" --place-wait-s "${PLACE_WAIT_S:-600}"
                                  ${CLICK_KIND:+--click-kind "$CLICK_KIND"}) ;;
  # plan 2026-11-06-unconfirmed-validation 2.1: the unconfirmed position check's strokes (REVERSE_KIND full|half|
  # chatter, MISSION_AFTER_CONFIRM=1 for the first pallet mission after them), unattended or after the placement
  reverse) UNATTENDED=1; PROBE_ARGS=(--scenario reverse) ;;
  reverse_place) UNATTENDED=; PROBE_ARGS=(--scenario place --after-place reverse "--truth=$START_POSE"
                                          --click-seed "${CLICK_SEED:-0}" --click-index "${CLICK_INDEX:-0}"
                                          --place-wait-s "${PLACE_WAIT_S:-600}"
                                          ${CLICK_KIND:+--click-kind "$CLICK_KIND"}) ;;
  *) echo "RELOC_PROBE must be hold, place, reverse or reverse_place" >&2; exit 2 ;;
esac
case "$PROBE_MODE" in
  reverse|reverse_place)
    case "${REVERSE_KIND:-full}" in full|half|chatter) ;; *) echo "REVERSE_KIND must be full, half or chatter" >&2; exit 2;; esac
    PROBE_ARGS+=(--reverse-kind "${REVERSE_KIND:-full}" --mission-timeout-s "${MISSION_TIMEOUT_S:-2400}")
    [ "${MISSION_AFTER_CONFIRM:-}" = 1 ] && PROBE_ARGS+=(--mission-after-confirm \
      --post-mission-sim-s "${POST_MISSION_SIM_S:-20}")
    [ -n "${POSE_CHECK:-}" ] && [ "$POSE_CHECK" != off ] \
      || { echo "RELOC_PROBE=$PROBE_MODE needs POSE_CHECK on or shadow" >&2; exit 2; } ;;
esac
GRACE=${RELOC_END_GRACE_S:-300}
if [ "${DRY_RUN:-0}" = 1 ]; then
  # tools/isaac_bundle.py check: the launcher's own DRY_RUN (its preflight, then RUNNER_ARGS and ARGS_OK) with the
  # session's environment; no probe, no session
  RELOC_UNATTENDED=$UNATTENDED MAX_EPOCHS=1 STANDBY_LINGER_S=0 exec bash "$LAUNCHER" "$OUT"
fi
mkdir -p "$OUT"
# The wrapper's own TERM (Slurm's, or whoever runs it), at any step from the launcher's start on: the launcher and the
# probe get it at once, the launcher is killed if still up RELOC_END_KILL_S after the first TERM (a repeated TERM does
# not restart that time), and the wrapper exits 143 (deploy/lib/term_relay.sh; before, the trap came after the
# launcher's start and the probe ran in the foreground, which held the TERM until the probe ended).
L=""
reloc_cancel() {
  [ -n "$L" ] || return 0  # latched: acted on once the launcher's pid is known
  kill -TERM $(jobs -p) 2>/dev/null  # the probe too, if its pid is not known yet
  while kill -0 "$L" 2>/dev/null && [ "$(term_relay_since_us)" -lt $(( ${RELOC_END_KILL_S:-120} * 1000000 )) ]; do
    sleep 0.1
  done
  kill -KILL $(jobs -p) 2>/dev/null
  echo "terminated: launcher ended $(date -Is)"
  exit 143
}
term_relay_arm reloc_cancel
RELOC_UNATTENDED=$UNATTENDED MAX_EPOCHS=1 STANDBY_LINGER_S=0 bash "$LAUNCHER" "$OUT" > "$OUT/launcher.log" 2>&1 &
term_relay_hold launcher
L=$!
term_relay_add "$L"
[ "$TERM_RELAY_GOT" = 1 ] && reloc_cancel  # a TERM that came before its pid was known
end_launcher() {
  kill -TERM "$L" 2>/dev/null
  for _ in $(seq 1 1200); do kill -0 "$L" 2>/dev/null || return 0; sleep 0.1; done  # its own TERM handling
  kill -KILL "$L" 2>/dev/null
}
"$VENV_PY" "$ROOT/tools/live_console_probe.py" --port-file "$OUT/live_port.json" --report "$OUT/probe.json" \
  "${PROBE_ARGS[@]}" --hold-sim-s "${HOLD_SIM_S:-10}" --live-factor "${FACTOR:-0.4}" --launcher-pid "$L" \
  --wait-port-s "${WAIT_PORT_S:-1500}" > "$OUT/probe.log" 2>&1 &
term_relay_hold probe
PROBE=$!
term_relay_add "$PROBE"
term_relay_wait "$PROBE"
prc=$TERM_RELAY_RC
echo "probe exit $prc $(date -Is)"
# 3: the launcher ended before its session ran (a relocalization ending) -- it is already gone
if [ "$prc" -ne 0 ] && [ "$prc" -ne 3 ]; then
  echo "probe failed: ending the launcher"
  end_launcher
else
  for _ in $(seq 1 $((GRACE * 10))); do kill -0 "$L" 2>/dev/null || break; sleep 0.1; done
  if kill -0 "$L" 2>/dev/null; then
    echo "launcher still up ${GRACE} s after the probe: ending it"
    end_launcher
  fi
fi
term_relay_wait "$L"
rc=$TERM_RELAY_RC
echo "launcher exit $rc $(date -Is)"
[ "$prc" -ne 0 ] && [ "$prc" -ne 3 ] && exit 4
exit "$rc"
