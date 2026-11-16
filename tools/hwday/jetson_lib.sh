# 11-17 (plan B): sourced by the jetson_*.sh scripts. The Jetson's address comes from data/jetson_check/2026-11-17/
# host.txt (written by jetson_find.sh), else $JETSON_HOST, else the ssh alias "jetson" (zetin@jetson-orin.local).
# Passwords only from the environment: ssh through `sshpass -e` ($JETSON_SSH_PASS in SSHPASS), sudo with
# $JETSON_SUDO_PASS on ssh's stdin -- never on a command line, never in a file (global rules, wiki remote-hosts).
set -euo pipefail
HW_REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
HW_OUT="$HW_REPO/data/jetson_check/2026-11-17"
mkdir -p "$HW_OUT"
if [[ -s "$HW_OUT/host.txt" ]]; then JHOST="zetin@$(head -1 "$HW_OUT/host.txt")"
elif [[ -n "${JETSON_HOST:-}" ]]; then JHOST="$JETSON_HOST"
else JHOST=jetson; fi
: "${JETSON_SSH_PASS:?JETSON_SSH_PASS is not in this session environment (restart the session after adding it)}"
command -v sshpass >/dev/null || { echo "sshpass is not installed (sudo pacman -S sshpass)" >&2; exit 2; }
JOPTS=(-o ConnectTimeout=10 -o ServerAliveInterval=15 -o StrictHostKeyChecking=accept-new -o ClearAllForwardings=yes)
jssh() { SSHPASS="$JETSON_SSH_PASS" sshpass -e ssh "${JOPTS[@]}" "$JHOST" "$@"; }
# jsudo '<command>' : sudo takes the password line from ssh's stdin; the command itself runs with stdin from
# /dev/null (exec </dev/null inside the sudo'd shell), so a prompt such as nvpmodel's reboot question never sees the
# password, also when sudo had it cached and did not read it (Codex P1). No single quotes inside <command>.
jsudo() {
  : "${JETSON_SUDO_PASS:?JETSON_SUDO_PASS is not in this session environment}"
  case "$1" in *"'"*) echo "jsudo: no single quotes in the command" >&2; return 2;; esac
  printf '%s\n' "$JETSON_SUDO_PASS" | SSHPASS="$JETSON_SSH_PASS" sshpass -e ssh "${JOPTS[@]}" "$JHOST" \
    "sudo -S -p '' sh -c 'exec </dev/null; $1'"
}
# the repo's tracked files to ~/forklift on the Jetson (uncommitted edits included, like the ws1 pytest wrapper)
jsync() {
  (cd "$HW_REPO" && git ls-files -z | tar --null -T - -cf -) | jssh 'mkdir -p ~/forklift && tar -xf - -C ~/forklift'
}
