#!/usr/bin/env bash
# 11-17 (plan B6): the A2M12 on the Jetson (move its USB cable from the laptop to the Jetson first).
#   bash tools/hwday/jetson_a2m12.sh [minutes, default 3]
# Needs docker on the Jetson (jetson_info.sh says): loads the arm64 scan image built on the laptop
# (forklift/ros2-scan:jazzy-arm64, data/hwday/ros2-scan-jazzy-arm64.tar.zst) once, then runs tools/hwday/a2m12_run.sh
# there and copies the results back. Without docker it stops: installing docker is the user's call (lead 2026-11-16).
source "$(dirname -- "${BASH_SOURCE[0]}")/jetson_lib.sh"
minutes="${1:-3}"
img=forklift/ros2-scan:jazzy-arm64
tarball="$HW_REPO/data/hwday/ros2-scan-jazzy-arm64.tar.zst"
[[ -e "$tarball" ]] || tarball="/home/light/school/2-2/embedded-design/forklift/data/hwday/ros2-scan-jazzy-arm64.tar.zst"
jssh 'command -v docker >/dev/null && docker info >/dev/null 2>&1' \
  || { echo "docker missing on the Jetson, or this user cannot use it: skip (A2M12 measured on the laptop); ask the user before installing docker"; exit 3; }
if ! jssh "docker image inspect $img >/dev/null 2>&1"; then
  echo "loading $img onto the Jetson (about 560 MB over the LAN) ..."
  if jssh 'command -v zstd >/dev/null'; then jssh 'zstd -dc | docker load' < "$tarball"
  else zstd -dc "$tarball" | jssh 'docker load'; fi
fi
jsync
rc=0
jssh "cd ~/forklift && FORKLIFT_ROS2_SCAN_IMAGE=$img bash tools/hwday/a2m12_run.sh jetson $minutes" > "$HW_OUT/a2m12_run.txt" 2>&1 || rc=$?
mkdir -p "$HW_REPO/data/a2m12_check"
jssh 'cd ~/forklift/data/a2m12_check && tar -cf - 2026-11-17_jetson' | tar -xf - -C "$HW_REPO/data/a2m12_check" \
  || echo "copying the results back failed" >&2
tail -15 "$HW_OUT/a2m12_run.txt"
exit "$rc"
