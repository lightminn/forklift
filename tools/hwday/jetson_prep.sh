#!/usr/bin/env bash
# 11-19 (the day before the main board arrives): bring the truck's Jetson to the current main and record its state.
#   HW_DAY=2026-11-19 bash tools/hwday/jetson_prep.sh       (about 15 min the first time: the scan image load is the long part)
# Needs the Jetson on and on the zetin Wi-Fi (jetson.local); JETSON_SSH_PASS / JETSON_SUDO_PASS from the session.
# Parts: the repo (this checkout's tracked files) and the venv; the arm64 scan image config/real/lidar.yaml scan_image
# names, loaded into docker if missing (data/hwday/<its tag>.tar.zst, built by tools/scan_image_build_arm64.sh -- no
# older image in its place); the power state as the Jetson booted --
# nvpmodel and jetson_clocks --show, to see whether jetson_clocks has to be run again after a reboot (not known since
# 11-17). Changes no power setting itself (jetson_power.sh does). Exits non-zero if a part failed; later parts still run.
source "$(dirname -- "${BASH_SOURCE[0]}")/jetson_lib.sh"
log="$HW_OUT/prep.txt"; : > "$log"
fails=()
say() { echo "$*" | tee -a "$log"; }
run() { local what="$1"; shift; if "$@" >> "$log" 2>&1; then say "OK    $what"; else say "FAIL  $what (see $log)"; fails+=("$what"); fi; }
run "repo -> ~/forklift ($(cd "$HW_REPO" && git rev-parse --short HEAD))" jsync
run "venv .[real,console] up to date (needs internet)" \
  jssh 'cd ~/forklift && .venv/bin/python -m pip install -q -e ".[real,console]" && .venv/bin/python -c "import forklift_core"'
run "D435i: pyrealsense2 from the source build still imports" \
  jssh 'cd ~/forklift && .venv/bin/python -c "import pyrealsense2; print(pyrealsense2.__file__)"'
img="$(python3 "$HW_REPO/tools/scan_image.py" configured --arch arm64)" || img="(lidar.yaml scan_image unreadable)"
tarball="$HW_REPO/data/hwday/$(python3 "$HW_REPO/tools/scan_image.py" tarball --arch arm64 2>/dev/null)"
load_image() {
  jssh "docker image inspect $img >/dev/null 2>&1" && { echo "already loaded"; return 0; }
  [[ -e "$tarball" ]] || { echo "no $tarball on this laptop"; return 1; }
  if jssh 'command -v zstd >/dev/null'; then jssh 'zstd -dc | docker load' < "$tarball"
  else zstd -dc "$tarball" | jssh 'docker load'; fi
  jssh "docker image inspect $img >/dev/null 2>&1"
}
run "scan image $img in docker" load_image
say "== power state as booted ($(date -Is))"
jsudo 'nvpmodel -q' 2>&1 | tee -a "$log"
jsudo 'jetson_clocks --show' 2>&1 | tee -a "$log"
jssh 'uptime -s' 2>&1 | sed 's/^/booted: /' | tee -a "$log"
say "read: MAXN_SUPER in nvpmodel -q, and in jetson_clocks --show whether the CPU MinFreq equals MaxFreq (clocks fixed)"
say "      -- if not fixed after this boot, jetson_clocks has to be run after every boot (jetson_power.sh does it)"
if (( ${#fails[@]} )); then say "NOT READY: ${fails[*]}"; exit 1; fi
say "READY"
