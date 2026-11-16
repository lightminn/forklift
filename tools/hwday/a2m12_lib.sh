# 11-17 (plan A): sourced by the a2m12_*.sh scripts. Each run gets its own folder (never judged on an older run's
# file, Codex P2), the driver container is started by name and stopped on every exit (the docker client's pid is
# waited for too), a signal ends the script after cleanup, and a driver that does not deliver scans within 30 s fails
# the run at once instead of after the full measurement.
set -euo pipefail
A_REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
cd "$A_REPO"
A_PY="${FORKLIFT_PY:-python3}"
[[ -x "$A_REPO/.venv/bin/python" ]] && A_PY="$A_REPO/.venv/bin/python"
[[ -x /home/light/anaconda3/bin/python ]] && A_PY=/home/light/anaconda3/bin/python
export PYTHONPATH="$A_REPO/src"
a_rundir() {  # a_rundir <label> <what> -> new folder data/a2m12_check/2026-11-17_<label>/<what>_<HHMMSS>
  local d="data/a2m12_check/2026-11-17_$1/$2_$(date +%H%M%S)"; mkdir -p "$d"; echo "$d"
}
a_device() {
  if [[ -z "${DEV:-}" ]]; then
    mapfile -t cands < <(find /dev/serial/by-id -maxdepth 1 -type l 2>/dev/null | grep -viE 'forklift-mcu|stm|weact' || true)
    if (( ${#cands[@]} != 1 )); then
      echo "USB-serial candidates (${#cands[@]}):" >&2; printf '  %s\n' "${cands[@]}" >&2
      echo "give the A2M12's one: DEV=/dev/serial/by-id/<name> bash $0 ..." >&2
      exit 2
    fi
    DEV="${cands[0]}"
  fi
}
A_NAME="hwday-a2m12-$$"; A_SOCK_DIR="/tmp/forklift-scan-$$"; A_CLIENT=""
a_cleanup() {
  docker stop -t 3 "$A_NAME" >/dev/null 2>&1 || true
  [[ -n "$A_CLIENT" ]] && { kill "$A_CLIENT" 2>/dev/null || true; wait "$A_CLIENT" 2>/dev/null || true; }
  docker stop -t 3 "$A_NAME" >/dev/null 2>&1 || true   # in case the container was still being created
  rm -rf "$A_SOCK_DIR"
}
trap a_cleanup EXIT
trap 'a_cleanup; exit 130' INT TERM
a_start_driver() {  # a_start_driver <log>: the pinned driver + adapter; returns once the container runs
  mkdir -p "$A_SOCK_DIR"
  FORKLIFT_DOCKER_NAME="$A_NAME" tools/ros2_dev.sh scan-a2m12 "$DEV" "$A_SOCK_DIR" > "$1" 2>&1 &
  A_CLIENT=$!
  for _ in $(seq 1 30); do
    [[ "$(docker inspect -f '{{.State.Running}}' "$A_NAME" 2>/dev/null)" == true ]] && return 0
    kill -0 "$A_CLIENT" 2>/dev/null || break
    sleep 1
  done
  echo "the driver container did not start (see $1)" >&2; tail -5 "$1" >&2; exit 3
}
a_first_scans() {  # a_first_scans <dir>: 10 s scan_check; fails at once when no scan arrives
  "$A_PY" tools/scan_check.py --socket "$A_SOCK_DIR/scan.sock" --seconds 10 --out "$1/first10s.json" > /dev/null 2>&1 || true
  local n; n=$("$A_PY" -c 'import json,sys; print(json.load(open(sys.argv[1])).get("scans", 0))' "$1/first10s.json" 2>/dev/null || echo 0)
  if (( n == 0 )); then echo "no scan in the first 10 s (driver log below)" >&2; tail -8 "$1/driver.log" >&2; exit 3; fi
  echo "first 10 s: $n scans"
}
