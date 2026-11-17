#!/usr/bin/env bash
# 11-17 (plan B4): fix the Jetson's power mode before the stall measurement, and record before/after.
#   bash tools/hwday/jetson_power.sh [MODE_NAME]     default MAXN_SUPER (the procedure's power-check mode)
# First stores the clock settings (~/forklift-jetson_clocks.conf; `sudo jetson_clocks --restore <file>` undoes them)
# and changes nothing unless that file exists. Then sets the nvpmodel mode by NAME (its id differs between images)
# and runs jetson_clocks. If the new mode needs a reboot it says so: reboot the Jetson, wait 1 min, run this again.
source "$(dirname -- "${BASH_SOURCE[0]}")/jetson_lib.sh"
mode="${1:-MAXN_SUPER}"
log="$HW_OUT/power.txt"
exec > >(tee "$log") 2>&1
echo "== before $(date -Is)"; jsudo 'nvpmodel -q'
id=$(jssh "awk -v n=$mode '/POWER_MODEL/ {for(i=1;i<=NF;i++){if(\$i ~ /^ID=/) id=substr(\$i,4); if(\$i ~ /^NAME=/) nm=substr(\$i,6)} if(nm==n) print id}' /etc/nvpmodel.conf" | head -1)
[[ -n "$id" ]] || { echo "mode $mode not in /etc/nvpmodel.conf:"; jssh "grep POWER_MODEL /etc/nvpmodel.conf"; exit 2; }
echo "== store clocks (once)"
jsudo "test -e /home/${JETSON_USER:-zetin}/forklift-jetson_clocks.conf || jetson_clocks --store /home/${JETSON_USER:-zetin}/forklift-jetson_clocks.conf"
jssh 'test -s ~/forklift-jetson_clocks.conf' || { echo "the clock backup is missing: nothing changed"; exit 2; }
echo "== nvpmodel -m $id ($mode)"; jsudo "nvpmodel -m $id" || true
now=$(jsudo 'nvpmodel -q' | head -1)
echo "now: $now"
case "$now" in *"$mode"*) ;; *) echo "STILL NOT $mode: a reboot is needed (sudo reboot on the Jetson, then run this again)"; exit 3;; esac
echo "== jetson_clocks"; jsudo 'jetson_clocks'
echo "== after"; jsudo 'nvpmodel -q'; jsudo 'jetson_clocks --show'
