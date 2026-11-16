#!/usr/bin/env bash
# 11-17 (plan B3): put the repo on the Jetson, install the MCU udev rule, make the Python venv and build the MCU model.
#   bash tools/hwday/jetson_setup.sh       (about 5 min with a network; each part says what it did)
# Does not install system packages: a missing cmake / python3-venv / docker is reported for the user to decide.
source "$(dirname -- "${BASH_SOURCE[0]}")/jetson_lib.sh"
log="$HW_OUT/setup.txt"; : > "$log"
say() { echo "$*" | tee -a "$log"; }
say "== repo -> ~/forklift"; jsync && say "copied $(cd "$HW_REPO" && git rev-parse --short HEAD)"
say "== udev (deploy/udev/99-forklift-mcu.rules)"
jssh 'cat > /tmp/99-forklift-mcu.rules' < "$HW_REPO/deploy/udev/99-forklift-mcu.rules"
jsudo 'install -m 644 /tmp/99-forklift-mcu.rules /etc/udev/rules.d/99-forklift-mcu.rules' \
  && jsudo 'udevadm control --reload' && jsudo 'udevadm trigger --subsystem-match=tty --subsystem-match=usb' \
  && say "udev rule installed and triggered" || say "udev install FAILED"
jssh 'cmp /tmp/99-forklift-mcu.rules /etc/udev/rules.d/99-forklift-mcu.rules && echo "rule matches the repo"' | tee -a "$log"
say "== venv ~/forklift/.venv (pip install -e .)"
jssh 'cd ~/forklift && { [ -x .venv/bin/python ] || python3 -m venv .venv; } && .venv/bin/python -m pip install -q -e . 2>&1 | tail -3 && .venv/bin/python -c "import numpy, yaml, forklift_core; print(\"venv ok\", numpy.__version__)"' 2>&1 | tee -a "$log" \
  || say "venv FAILED (python3-venv missing or no network: ask the user before installing packages)"
say "== MCU model build (cmake, for the stall measurement)"
jssh 'cd ~/forklift && command -v cmake >/dev/null || { echo "cmake: none -- ask the user (sudo apt-get install -y cmake)"; exit 3; }
      cmake -S firmware -B build/rehearsal_mcu_model -DFW_MCU_MODEL=ON -DFW_PYTHON=$PWD/.venv/bin/python >/dev/null &&
      cmake --build build/rehearsal_mcu_model --target mcu_model -j4 2>&1 | tail -1 && ls -l build/rehearsal_mcu_model/mcu_model' 2>&1 | tee -a "$log" \
  || say "MCU model build FAILED"
