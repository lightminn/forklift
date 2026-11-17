#!/usr/bin/env bash
# 11-17 (plan B1): find the Jetson on the school LAN and check ssh. Plug the Jetson's Ethernet into the same LAN
# (or switch) as the laptop and power it on, then:  bash tools/hwday/jetson_find.sh [IP]
# 1) the IP you give, 2) mDNS jetson-orin.local, 3) the laptop's neighbour table after one ping sweep of its own
# /24 (only when 1-2 fail): entries with an NVIDIA MAC prefix are listed first. Writes data/jetson_check/2026-11-17/
# host.txt with the address that answered ssh.
set -euo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
out="$repo/data/jetson_check/2026-11-17"; mkdir -p "$out"
try_ssh() {
  SSHPASS="$JETSON_SSH_PASS" sshpass -e ssh -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new \
    -o ClearAllForwardings=yes "${JETSON_USER:-zetin}@$1" 'hostname; cat /etc/nv_tegra_release 2>/dev/null | head -1' 2>/dev/null
}
: "${JETSON_SSH_PASS:?JETSON_SSH_PASS is not in this session environment}"
cands=()
[[ -n "${1:-}" ]] && cands+=("$1")
ip_m=$(getent ahostsv4 jetson-orin.local 2>/dev/null | awk 'NR==1{print $1}' || true)
[[ -n "$ip_m" ]] && cands+=("$ip_m") && echo "mDNS: jetson-orin.local -> $ip_m"
for ip in "${cands[@]}"; do
  if who=$(try_ssh "$ip"); then echo "$ip" > "$out/host.txt"; echo "ssh OK: $ip ($who)"; exit 0; fi
  echo "ssh failed: $ip"
done
net=$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}')
[[ -z "$net" ]] && { echo "no IPv4 route: is the laptop on the LAN?" >&2; exit 2; }
base="${net%.*}"
echo "mDNS did not answer; one ping sweep of ${base}.0/24, then an ssh try only on hosts with port 22 open (about 15 s) ..."
for i in $(seq 1 254); do ping -c1 -W1 "${base}.$i" >/dev/null 2>&1 & done; wait
ip neigh show | awk '$5 ~ /:/ {print $1, $5}' | sort > "$out/neighbours.txt"
: > "$out/port22.txt"
while read -r ip mac; do
  (timeout 1 bash -c "echo > /dev/tcp/$ip/22" 2>/dev/null && echo "$ip $mac" >> "$out/port22.txt") &
done < "$out/neighbours.txt"; wait
# NVIDIA OUIs seen on Jetson dev kits (48:b0:2d, 00:04:4b, 3c:6d:66) first; at most 10 hosts are tried
mapfile -t tries < <({ grep -iE ' (48:b0:2d|00:04:4b|3c:6d:66)' "$out/port22.txt" || true; grep -viE ' (48:b0:2d|00:04:4b|3c:6d:66)' "$out/port22.txt" || true; } | head -10)
for entry in "${tries[@]}"; do
  ip="${entry%% *}"
  if who=$(try_ssh "$ip"); then echo "$ip" > "$out/host.txt"; echo "ssh OK: $entry ($who)"; exit 0; fi
done
echo "no host answered ssh as ${JETSON_USER:-zetin} (port 22 open: $(wc -l < "$out/port22.txt")). Read the IP off the Jetson's screen (ip -4 addr) and run: bash tools/hwday/jetson_find.sh <IP>"
exit 2
