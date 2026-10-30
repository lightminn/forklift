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
#   optional: HOLD_SIM_S (10), WAIT_PORT_S (1500), RELOC_END_GRACE_S (300); RELOC_LAUNCHER (tests only)
#   RELOC_PROBE (hold): place -- F1 (docs/plans/2026-10-31-operator-placed-start.md 4.2 b): the session is attended
#   (no RELOC_UNATTENDED: the console waits for the operator) and the probe is the operator, placing the click model's
#   draws around START_POSE (CLICK_SEED, CLICK_INDEX: the start's index in its list) and confirming the first preview
#
# Run from the repository root.
set -u
OUT=$(realpath -m "${1:?output directory}")
ROOT=$(pwd)
: "${START_POSE:?the placed start x,y,yaw (world base_link)}" "${VENV_PY:?the Isaac Python}"
LAUNCHER=${RELOC_LAUNCHER:-$ROOT/deploy/console/isaac_live.sbatch}
PROBE_MODE=${RELOC_PROBE:-hold}
case "$PROBE_MODE" in
  hold) UNATTENDED=1; PROBE_ARGS=(--scenario hold) ;;
  place) UNATTENDED=; PROBE_ARGS=(--scenario place --truth "$START_POSE" --click-seed "${CLICK_SEED:-0}"
                                  --click-index "${CLICK_INDEX:-0}" --place-wait-s "${PLACE_WAIT_S:-600}") ;;
  *) echo "RELOC_PROBE must be hold or place" >&2; exit 2 ;;
esac
GRACE=${RELOC_END_GRACE_S:-300}
if [ "${DRY_RUN:-0}" = 1 ]; then
  # tools/isaac_bundle.py check: the launcher's own DRY_RUN (its preflight, then RUNNER_ARGS and ARGS_OK) with the
  # session's environment; no probe, no session
  RELOC_UNATTENDED=$UNATTENDED MAX_EPOCHS=1 STANDBY_LINGER_S=0 exec bash "$LAUNCHER" "$OUT"
fi
mkdir -p "$OUT"
RELOC_UNATTENDED=$UNATTENDED MAX_EPOCHS=1 STANDBY_LINGER_S=0 bash "$LAUNCHER" "$OUT" > "$OUT/launcher.log" 2>&1 &
L=$!
end_launcher() {
  kill -TERM "$L" 2>/dev/null
  for _ in $(seq 1 1200); do kill -0 "$L" 2>/dev/null || return 0; sleep 0.1; done  # its own TERM handling
  kill -KILL "$L" 2>/dev/null
}
trap 'end_launcher; exit 143' TERM INT
"$VENV_PY" "$ROOT/tools/live_console_probe.py" --port-file "$OUT/live_port.json" --report "$OUT/probe.json" \
  "${PROBE_ARGS[@]}" --hold-sim-s "${HOLD_SIM_S:-10}" --live-factor "${FACTOR:-0.4}" --launcher-pid "$L" \
  --wait-port-s "${WAIT_PORT_S:-1500}" > "$OUT/probe.log" 2>&1
prc=$?
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
wait "$L"
rc=$?
echo "launcher exit $rc $(date -Is)"
[ "$prc" -ne 0 ] && [ "$prc" -ne 3 ] && exit 4
exit "$rc"
