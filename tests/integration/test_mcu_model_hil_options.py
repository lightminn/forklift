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

from forklift_core.mcu_link.messages import Arm, ClearFaults, Disarm, LinkDecoder, McuState, StateFlag, encode

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
        self.raw = b""

    def states(self, seconds):
        out, end = [], time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                data = os.read(self.master, 4096)
            except BlockingIOError:
                time.sleep(0.002)
                continue
            self.raw += data
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


def test_state_silence_keeps_taking_commands(mcu_model, tmp_path):
    log = tmp_path / "accept.log"
    m = Model(mcu_model, "--state-silent-at-ms", 300, "--state-silent-ms", 300, "--accept-log", log)
    try:
        seen, end = [], time.monotonic() + 5.0
        while (not seen or seen[-1].mcu_time_us < 290_000) and time.monotonic() < end and m.proc.poll() is None:
            seen += m.states(0.01)
        assert seen and seen[-1].mcu_time_us >= 290_000
        time.sleep(0.1)  # inside the window (300-600 ms of the model's clock)
        m.send(Disarm(), 4321)
        seen += m.states(0.6)
        quiet = [s for s in seen if 301_000 <= s.mcu_time_us < 599_000]
        assert not quiet  # none measured in it, none queued before it sent late in it
        after = [s for s in seen if s.mcu_time_us >= 600_000]
        assert after and after[0].last_rx_seq == 4321
    finally:
        m.close()
    rows = [r.split() for r in log.read_text().splitlines()]
    took = [r for r in rows if r[1] == "4321"]
    assert took and 300_000 <= int(took[0][0]) < 600_000  # handled inside the silence


def test_accept_log_names_each_handled_frame_with_its_prior_mode(mcu_model, tmp_path):
    log = tmp_path / "accept.log"
    m = Model(mcu_model, "--accept-log", log)
    try:
        m.states(0.2)
        t_sent = time.monotonic_ns()
        m.send(Disarm(), 77)
        os.write(m.master, bytes([1]) * 96 + b"\x00")  # the poison: no frame
        m.send(ClearFaults(0x1234), 77)  # the same seq again, ignored while DISARMED
        m.send(Arm(0x1234, 1), 78)  # a wrong boot: refused
        m.send(Disarm(), 78)  # the same seq after a refusal: taken
        m.states(0.2)
    finally:
        m.close()
    rows = [line.split() for line in log.read_text().splitlines()]
    got = [(r[1], r[2], r[3]) for r in rows]
    assert got[0] == ("77", "DISARM", "accepted")
    assert got[1] == ("77", "CLEAR_FAULTS", "handled")
    assert got[2][:2] == ("78", "ARM") and got[2][2].startswith("rejected:")
    assert got[3] == ("78", "DISARM", "accepted")
    assert len(rows) == 4
    for r in rows:
        int(r[0]), int(r[4])  # MCU time, prior mode
        assert int(r[5]) == -1  # no active command while DISARMED (round 2 P3)
        assert len(r) == 8 and int(r[6]) != 0 and int(r[7]) >= t_sent  # boot id, this host's monotonic ns (10.6)


def test_a_frame_no_branch_handles_is_logged_ignored(mcu_model, tmp_path):
    from forklift_core.mcu_link.messages import McuState, Mode, RejectReason

    log = tmp_path / "accept.log"
    m = Model(mcu_model, "--accept-log", log)
    try:
        m.states(0.1)
        st = McuState(1, 2, 0, 0, Mode.DISARMED, RejectReason.NONE, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, (0,) * 4,
                      (0,) * 4, (0,) * 4, 0, 0, StateFlag(0))
        m.send(st, 55)
        m.states(0.1)
    finally:
        m.close()
    rows = [line.split() for line in log.read_text().splitlines()]
    assert rows and rows[-1][1] == "55" and rows[-1][3] == "ignored"


def test_a_frame_cut_mid_way_at_a_state_cut_is_dropped_and_the_poison_resyncs(mcu_model, tmp_path):
    """Round 3 P2: a STATE half written just before the cut (the --short-write-at-ms injection) is not finished in
    the cut; after it the poison closes the head in the host's decoder and the first new STATE decodes; a DISARM sent
    in the cut is taken."""
    log = tmp_path / "accept.log"
    m = Model(mcu_model, "--short-write-at-ms", 280, "--state-silent-at-ms", 300, "--state-silent-ms", 300,
              "--accept-log", log)
    try:
        got, end = [], time.monotonic() + 5.0
        while (not got or got[-1][1].mcu_time_us < 250_000) and time.monotonic() < end:
            got += [(time.monotonic_ns(), s) for s in m.states(0.01)]
        time.sleep(0.15)
        m.send(Disarm(), 4321)
        mid = m.states(0.2)  # inside the cut
        after = m.states(0.4)
        sts = mid + after
        # nothing measured from the injection to the end of the cut is completed (the half STATE's rest dropped)
        assert not [s for s in sts if 280_000 <= s.mcu_time_us < 600_000]
        later = [s for s in sts if s.mcu_time_us >= 600_000]
        assert later and later[0].last_rx_seq == 4321
        assert m.dec.errors == 1  # the poison closed the half frame: one overflow, no spoiled STATE
        poison = bytes([1]) * 96 + b"\x00"
        p = m.raw.find(poison)
        assert p >= 0  # the poison went out
        tail = [r.message for r in LinkDecoder().feed(m.raw[p + len(poison):], 0) if isinstance(r.message, McuState)]
        assert tail and tail[0].mcu_time_us >= 600_000 and tail[0] is not None
        assert tail[0].mcu_time_us == later[0].mcu_time_us  # the first STATE after the cut follows the poison at once
    finally:
        m.close()
    rows = [r.split() for r in log.read_text().splitlines() if r.split()[1] == "4321"]
    assert rows and 300_000 <= int(rows[0][0]) < 600_000


def test_the_prior_expiry_is_none_after_an_estop_ended_the_armed_command(mcu_model, tmp_path):
    """Round 3-4 P3: the e-stop comes while the ARM's command is still live; the DISARM sent once ESTOP shows is handled
    before that command's own expiry, with prior mode ESTOP and no active expiry."""
    from forklift_core.mcu_link.messages import Mode

    log = tmp_path / "accept.log"
    m = Model(mcu_model, "--estop-at-ms", 250, "--accept-log", log)
    try:
        sts, end = [], time.monotonic() + 5.0
        while (not sts or sts[-1].mcu_time_us < 180_000) and time.monotonic() < end:
            sts += m.states(0.005)
        echo = sts[-1].mcu_time_us
        m.send(Arm(sts[-1].boot_id, echo), 10)
        seen, end = [], time.monotonic() + 2.0
        while not any(s.mode == Mode.ESTOP for s in seen) and time.monotonic() < end:
            seen += m.states(0.002)
        m.send(Disarm(), 11)
        m.states(0.1)
    finally:
        m.close()
    rows = [r.split() for r in log.read_text().splitlines()]
    arm = [r for r in rows if r[1] == "10"]
    dis = [r for r in rows if r[1] == "11"]
    assert arm and arm[0][3] == "accepted"
    assert dis and int(dis[0][4]) == int(Mode.ESTOP) and int(dis[0][5]) == -1
    assert int(dis[0][0]) < echo + 200_000  # before the ARM's own command would have run out


def test_the_accept_log_keeps_the_cores_read_budget(mcu_model, tmp_path):
    """Several frames of one tick are handled in that tick, as without the log (round 1 P2-1)."""
    sts = {}
    for name, extra in (("off", ()), ("on", ("--accept-log", tmp_path / "a.log"))):
        m = Model(mcu_model, *extra)
        try:
            for i in range(40):
                m.send(Disarm(), 100 + i)
            sts[name] = m.states(0.15)
        finally:
            m.close()
    assert sts["on"][0].last_rx_seq == sts["off"][0].last_rx_seq


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
        assert len(r) == 9  # MCU us, wall gap ns, mode, lift, steering, wheel L, R, host monotonic ns, pushed
        int(r[1]), int(r[2]), float(r[3]), float(r[4]), float(r[5]), float(r[6]), int(r[7]), int(r[8])


@pytest.mark.parametrize("option", ["--accept-log", "--axis-log"])
def test_a_log_that_cannot_be_written_ends_the_model_with_exit_4(mcu_model, option):
    """6a-2b (design 10.8): exit 0 only when every log row was written and the log closed; /dev/full refuses them."""
    m = Model(mcu_model, option, "/dev/full")
    try:
        m.states(0.1)
        m.send(Disarm(), 5)  # a handled frame: one accept-log row
        m.states(0.1)
        m.proc.send_signal(signal.SIGTERM)
        assert m.proc.wait(timeout=5) == 4
    finally:
        m.close()


def test_a_written_log_ends_the_model_with_exit_0(mcu_model, tmp_path):
    m = Model(mcu_model, "--accept-log", tmp_path / "a.log", "--axis-log", tmp_path / "x.log")
    try:
        m.states(0.1)
        m.send(Disarm(), 5)
        m.states(0.1)
        m.proc.send_signal(signal.SIGTERM)
        assert m.proc.wait(timeout=5) == 0
    finally:
        m.close()


def test_sigusr1_pushes_the_plants_axes_once(mcu_model, tmp_path):
    """6a-2b (design 2 (가)): SIGUSR1 moves the plant's lift and steering by --axis-push-lift-m / -steer-rad once,
    as an outside push; nothing else changes."""
    log = tmp_path / "axis.log"
    m = Model(mcu_model, "--axis-log", log, "--axis-push-lift-m", "0.006", "--axis-push-steer-rad", "0.025")
    try:
        m.states(0.2)
        m.proc.send_signal(signal.SIGUSR1)
        m.states(0.2)
    finally:
        m.close()
    rows = [line.split() for line in log.read_text().splitlines()]
    lift = [float(r[3]) for r in rows]
    steer = [float(r[4]) for r in rows]
    jumps = [i for i in range(1, len(lift)) if lift[i] - lift[i - 1] > 0.005]
    assert len(jumps) == 1 and abs(steer[jumps[0]] - steer[jumps[0] - 1] - 0.025) < 0.002
    assert [i for i, r in enumerate(rows) if r[8] == "1"] == jumps  # the marked tick is the jump's
