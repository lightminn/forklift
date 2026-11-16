#!/usr/bin/env bash
# 11-17 (plan B2): record the Jetson as found (read only; nvpmodel and jetson_clocks are read with sudo).
#   bash tools/hwday/jetson_info.sh        -> data/jetson_check/2026-11-17/info.txt
source "$(dirname -- "${BASH_SOURCE[0]}")/jetson_lib.sh"
{
  echo "== host $JHOST $(date -Is)"
  jssh 'hostname; cat /etc/nv_tegra_release 2>/dev/null; dpkg-query -W -f="nvidia-jetpack \${Version}\n" nvidia-jetpack 2>/dev/null;
        lsb_release -ds 2>/dev/null; uname -r; nproc; free -h | sed -n 2p; df -h / | tail -1;
        echo "python3 $(python3 --version 2>&1)"; command -v cmake >/dev/null && cmake --version | head -1 || echo "cmake: none";
        gcc --version 2>/dev/null | head -1 || echo "gcc: none";
        command -v docker >/dev/null && { docker --version; docker info --format "docker server {{.ServerVersion}}" 2>&1 | tail -1; } || echo "docker: none";
        id; ls -l /dev/serial/by-id 2>/dev/null || echo "no /dev/serial/by-id"; ls -l /dev/forklift-mcu* 2>/dev/null || echo "no /dev/forklift-mcu";
        ls /etc/udev/rules.d/ | grep -i forklift || echo "no forklift udev rule"; ip -4 -br addr'
  echo "== nvpmodel -q"; jsudo 'nvpmodel -q' || echo "(nvpmodel failed)"
  echo "== jetson_clocks --show"; jsudo 'jetson_clocks --show' || echo "(jetson_clocks failed)"
} 2>&1 | tee "$HW_OUT/info.txt"
