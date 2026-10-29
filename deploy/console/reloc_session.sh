#!/bin/bash
# One unattended start relocalization session for B-P2d (docs/plans/2026-10-29-start-relocalization.md 4.4): the console
# launcher (deploy/console/isaac_live.sbatch) with START_POSE, RELOC_UNATTENDED=1 and MAX_EPOCHS=1 -- a failure is the
# session's outcome, never retried -- and the console probe's hold scenario: it waits for the session to run (the port
# file says running only once localized), stands HOLD_SIM_S simulated seconds and ends the session; a session that ends
# before (any relocalization ending) ends the probe with it. The verdict is read afterwards from the records
# (tools/reloc_judge.py).
#
#   START_POSE=x,y,yaw <isaac_live.sbatch's environment> bash deploy/console/reloc_session.sh OUT
#   optional: HOLD_SIM_S (10), WAIT_PORT_S (1500)
#
# Run from the repository root.
set -u
OUT=$(realpath -m "${1:?output directory}")
ROOT=$(pwd)
: "${START_POSE:?the placed start x,y,yaw (world base_link)}" "${VENV_PY:?the Isaac Python}"
mkdir -p "$OUT"
RELOC_UNATTENDED=1 MAX_EPOCHS=1 STANDBY_LINGER_S=0 bash "$ROOT/deploy/console/isaac_live.sbatch" "$OUT" \
  > "$OUT/launcher.log" 2>&1 &
L=$!
trap 'kill -TERM "$L" 2>/dev/null' TERM INT
"$VENV_PY" "$ROOT/tools/live_console_probe.py" --port-file "$OUT/live_port.json" --report "$OUT/probe.json" \
  --scenario hold --hold-sim-s "${HOLD_SIM_S:-10}" --live-factor "${FACTOR:-0.4}" --launcher-pid "$L" \
  --wait-port-s "${WAIT_PORT_S:-1500}" > "$OUT/probe.log" 2>&1
echo "probe exit $? $(date -Is)"
wait "$L"
rc=$?
if kill -0 "$L" 2>/dev/null; then  # the wait was cut by the trap: wait for the launcher's own end
  wait "$L"
  rc=$?
fi
echo "launcher exit $rc $(date -Is)"
exit "$rc"
