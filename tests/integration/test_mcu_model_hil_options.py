"""PR 6a-2 (docs/design/2026-11-20-real-hil-6a.md 1, 6): the MCU model's HIL options -- ``--hil-plant`` (the
HIL_PLANT and OUTPUTS_LOCKED build bits together), ``--state-silent-at-ms``/``--state-silent-ms`` (STATE stops, commands
still taken), ``--accept-log`` (each frame the core handled: its prior mode and active expiry, its verdict) and
``--axis-log`` (each MCU tick's wall gap, mode, lift, steering and wheel speeds)."""

import os
import shutil
import signal
import subprocess
import sys
import time
import tty
from pathlib import Path

import pytest

from forklift_core.mcu_link.messages import Disarm, LinkDecoder, McuState, StateFlag, encode

REPO = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def mcu_model(tmp_path_factory) -> Path:
    if shutil.which("cmake") is None or (shutil.which("cc") is None and shutil.which("gcc") is None):
        pytest.skip("cmake or a C compiler missing")
    build = tmp_path_factory.mktemp("mcu_model_build")
    env = {**os.environ, "PYTHONPATH": str(REPO / "src")}
    for cmd in (["cmake", "-S", str(REPO / "firmware"), "-B", str(build), "-DFW_MCU_MODEL=ON",
                 f"-DFW_PYTHON={sys.executable}"], ["cmake", "--build", str(build), "--target", "mcu_model", "-j2"]):
        r = subprocess.run(cmd, capture_output=True, text=True, env=env)
        assert r.returncode == 0, r.stdout + r.stderr
    return build / "mcu_model"


class Model:
    def __init__(self, exe, *args):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        self.proc = subprocess.Popen([str(exe), "--link", os.ttyname(self.slave), *map(str, args)])
        self.dec = LinkDecoder()

    def states(self, seconds):
        out, end = [], time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                data = os.read(self.master, 4096)
            except BlockingIOError:
                time.sleep(0.002)
                continue
            out += [r.message for r in self.dec.feed(data, time.monotonic_ns()) if isinstance(r.message, McuState)]
        return out

    def send(self, msg, seq):
        os.write(self.master, encode(msg, seq))

    def close(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGTERM)
            self.proc.wait(timeout=5)
        os.close(self.master)
        os.close(self.slave)


def test_hil_plant_sets_both_build_bits(mcu_model):
    m = Model(mcu_model, "--hil-plant", 1)
    try:
        sts = m.states(0.3)
        assert sts
        assert all(s.flags & StateFlag.HIL_PLANT and s.flags & StateFlag.OUTPUTS_LOCKED for s in sts)
        assert not any(s.flags & StateFlag.COMMISSION_BUILD for s in sts)
    finally:
        m.close()


def test_hil_plant_with_a_commission_build_is_refused(mcu_model):
    m = Model(mcu_model, "--hil-plant", 1, "--commission-build", 1)
    try:
        assert m.proc.wait(timeout=5) != 0
    finally:
        m.close()


def test_state_silence_keeps_taking_commands(mcu_model):
    m = Model(mcu_model, "--state-silent-at-ms", 300, "--state-silent-ms", 300)
    try:
        seen = []
        while not seen or seen[-1].mcu_time_us < 290_000:
            seen += m.states(0.01)
        time.sleep(0.1)  # inside the window (300-600 ms of the model's clock)
        m.send(Disarm(), 4321)
        seen += m.states(0.6)
        quiet = [s for s in seen if 301_000 <= s.mcu_time_us < 599_000]
        assert not quiet
        after = [s for s in seen if s.mcu_time_us >= 600_000]
        assert after and after[0].last_rx_seq == 4321  # the DISARM sent in the silence was taken
        assert all(s.last_rx_seq != 4321 for s in seen if s.mcu_time_us < 300_000)
    finally:
        m.close()


def test_accept_log_names_each_handled_frame_with_its_prior_mode(mcu_model, tmp_path):
    log = tmp_path / "accept.log"
    m = Model(mcu_model, "--accept-log", log)
    try:
        m.states(0.2)
        m.send(Disarm(), 77)
        os.write(m.master, bytes([1]) * 96 + b"\x00")  # the poison: no frame
        m.send(Disarm(), 78)
        m.states(0.2)
    finally:
        m.close()
    rows = [line.split() for line in log.read_text().splitlines()]
    seqs = [(r[1], r[2], r[3]) for r in rows]
    assert ("77", "DISARM", "accepted") in seqs and ("78", "DISARM", "accepted") in seqs
    for r in rows:
        int(r[0]), int(r[4]), int(r[5])  # MCU time, prior mode, prior active expiry (-1: none)
    assert len(rows) == 2


def test_axis_log_has_one_row_per_mcu_tick(mcu_model, tmp_path):
    log = tmp_path / "axis.log"
    m = Model(mcu_model, "--axis-log", log)
    try:
        m.states(0.3)
    finally:
        m.close()
    rows = [line.split() for line in log.read_text().splitlines()]
    assert len(rows) > 150  # about 1 kHz over 0.3 s, a loaded host allowing
    t = [int(r[0]) for r in rows]
    assert t == sorted(t)
    for r in rows:
        assert len(r) == 7  # MCU us, wall gap ns, mode, lift m, steering rad, wheel L, R rad/s
        int(r[1]), int(r[2]), float(r[3]), float(r[4]), float(r[5]), float(r[6])
