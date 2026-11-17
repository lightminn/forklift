#!/usr/bin/env bash
# 11-17 (plan B3): put the repo on the Jetson, install the MCU udev rule, make the Python venv and build the MCU model.
#   bash tools/hwday/jetson_setup.sh       (about 5 min with a network; each part says what it did)
# Does not install system packages: a missing cmake / python3-venv / docker is reported for the user to decide.
# Exits non-zero if any part failed (the later parts still run, so one look shows everything).
source "$(dirname -- "${BASH_SOURCE[0]}")/jetson_lib.sh"
log="$HW_OUT/setup.txt"; : > "$log"
fails=()
say() { echo "$*" | tee -a "$log"; }
run() { local what="$1"; shift; if "$@" >> "$log" 2>&1; then say "OK    $what"; else say "FAIL  $what (see $log)"; fails+=("$what"); fi; }
run "repo -> ~/forklift ($(cd "$HW_REPO" && git rev-parse --short HEAD))" jsync
jssh 'cat > /tmp/99-forklift-mcu.rules' < "$HW_REPO/deploy/udev/99-forklift-mcu.rules"
run "udev rule installed" jsudo 'install -m 644 /tmp/99-forklift-mcu.rules /etc/udev/rules.d/99-forklift-mcu.rules'
run "udev reload" jsudo 'udevadm control --reload'
run "udev trigger" jsudo 'udevadm trigger --subsystem-match=tty --subsystem-match=usb'
run "udev rule matches the repo" jssh 'cmp /tmp/99-forklift-mcu.rules /etc/udev/rules.d/99-forklift-mcu.rules'
run "${JETSON_USER:-zetin} in dialout (the rule gives 0660)" jssh 'id -nG | tr " " "\n" | grep -qx dialout'
run "venv ~/forklift/.venv with .[real,console] (needs internet)" \
  jssh 'cd ~/forklift && { [ -x .venv/bin/python ] || python3 -m venv .venv; } && .venv/bin/python -m pip install -q -e ".[real,console]"'
run "venv imports (numpy, yaml, websockets, forklift_core)" \
  jssh 'cd ~/forklift && .venv/bin/python -c "import numpy, yaml, websockets, forklift_core; print(numpy.__version__)"'
run "cmake present (else ask the user: sudo apt-get install -y cmake)" jssh 'command -v cmake'
run "MCU model built" jssh 'cd ~/forklift && cmake -S firmware -B build/rehearsal_mcu_model -DFW_MCU_MODEL=ON -DFW_PYTHON=$HOME/forklift/.venv/bin/python && cmake --build build/rehearsal_mcu_model --target mcu_model -j4 && test -x build/rehearsal_mcu_model/mcu_model'
if (( ${#fails[@]} )); then say "NOT READY: ${fails[*]}"; exit 1; fi
say "READY"
