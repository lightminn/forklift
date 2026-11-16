"""mcu_link host core + tty adapter against the firmware core on a fake board over a pty (sim-to-real plan
P0-7). Real time: waits are generous wall-clock bounds for the protocol flows; exact deadline arithmetic
is pinned in tests/unit/plant/test_mcu_host.py and the C core tests. Skipped without cmake or a C compiler."""

import math
import os
import shutil
import signal
import subprocess
import sys
import time
import tty
from pathlib import Path

import numpy as np
import pytest

from forklift_core.mcu_link.messages import (
    Arm,
    ClearFaults,
    Disarm,
    FaultFlag,
    LinkDecoder,
    McuState,
    Mode,
    RejectReason,
    StateFlag,
    encode,
)
from forklift_core.plant.mcu_host import HostConfig, McuLinkHost
from forklift_core.plant.real_vehicle import (
    RealOdometry,
    load_real_vehicle,
    rear_drive_command,
)

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
from real_mcu_link import ReconnectingLink, TtyLink, service_fd  # noqa: E402

CFG = HostConfig(state_max_age_s=0.1, cap_ms=150, arm_timeout_s=1.0, zero_valid_ms=100, outbox_max=16)
VEHICLE = load_real_vehicle(REPO / "config" / "real" / "vehicle.yaml")


@pytest.fixture(scope="module")
def mcu_model(tmp_path_factory) -> Path:
    if shutil.which("cmake") is None:
        pytest.skip("cmake not installed")
    if shutil.which("cc") is None and shutil.which("gcc") is None:
        pytest.skip("no C compiler")
    build = tmp_path_factory.mktemp("mcu_model_build")
    env = {**os.environ, "PYTHONPATH": str(REPO / "src")}
    r = subprocess.run(["cmake", "-S", str(REPO / "firmware"), "-B", str(build), "-DFW_MCU_MODEL=ON",
                        f"-DFW_PYTHON={sys.executable}"], capture_output=True, text=True, env=env)
    assert r.returncode == 0, r.stdout + r.stderr
    r = subprocess.run(["cmake", "--build", str(build), "--target", "mcu_model", "-j2"], capture_output=True,
                       text=True, env=env)
    assert r.returncode == 0, r.stdout + r.stderr
    return build / "mcu_model"


class Bench:
    def __init__(self, exe: Path, *args: str):
        self.exe = exe
        self.master, self.slave = os.openpty()  # our slave fd stays open: the master never sees EIO between runs
        self.slave_path = os.ttyname(self.slave)
        tty.setraw(self.slave)  # no line discipline on either side
        os.set_blocking(self.master, False)
        self.link = TtyLink(self.master)  # the master as is (reopening /dev/ptmx would make a new pty)
        self.log = []
        self.host = McuLinkHost(CFG, log=lambda *a: self.log.append(a))
        self.events = []
        self.proc = None
        self.start(*args)

    def start(self, *args: str):
        self.proc = subprocess.Popen([str(self.exe), "--link", self.slave_path, *args])

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGTERM)
            self.proc.wait(timeout=5)

    def close(self):
        self.stop()
        self.link.close()
        os.close(self.slave)

    def run_until(self, pred, timeout_s=3.0, every=None):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            now = time.monotonic_ns()
            self.events += service_fd(self.link, self.host, now)
            if every:
                every(now)
            if pred():
                return True
            time.sleep(0.002)
        return False

    def state(self):
        return self.host.state

    def written(self):
        dec, out = LinkDecoder(), []
        for stage, _t, data, _d in self.log:
            if stage == "written":
                out += [r.message for r in dec.feed(data, 0)]
        return out


@pytest.fixture
def bench_factory(mcu_model):
    made = []

    def make(*args):
        b = Bench(mcu_model, *args)
        made.append(b)
        return b

    yield make
    for b in made:
        b.close()


def arm(b: Bench):
    assert b.run_until(lambda: b.state() is not None and b.state().mode == Mode.DISARMED), "no DISARMED STATE"
    assert b.host.request_arm(time.monotonic_ns()) is None
    assert b.run_until(lambda: b.host.ready), [e.kind for e in b.events]


def test_the_model_speaks_mcu_link_2_as_a_normal_wired_build(bench_factory):
    """mcu-link/2: the model is a normal build with its e-stop input wired -- no build bit, a power-on reset."""
    from forklift_core.mcu_link.build_policy import Client, connect_refusal, drive_refusal
    from forklift_core.mcu_link.messages import ResetCause

    b = bench_factory()
    assert b.run_until(lambda: b.state() is not None and b.state().mode == Mode.DISARMED), "no DISARMED STATE"
    s = b.state()
    build_bits = StateFlag.COMMISSION_BUILD | StateFlag.HIL_PLANT | StateFlag.OUTPUTS_LOCKED | StateFlag.ESTOP_UNWIRED
    assert not s.flags & build_bits and s.reset_cause == ResetCause.POWER
    assert connect_refusal(s.flags, Client.REAL) is None and drive_refusal(s.flags, Client.REAL) is None
    assert connect_refusal(s.flags, Client.HIL) == "not_hil_board"


def test_operator_arm_drive_and_stop_on_expiry(bench_factory):
    b = bench_factory()
    arm(b)
    cmd = rear_drive_command(0.3, 0.2, VEHICLE)
    odo = RealOdometry(VEHICLE, max_gap_s=0.1)
    poses = []

    def send(now):
        if b.host.ready:
            b.host.drive(cmd, 0.0, True, now + 60_000_000, now)
        s = b.state()
        poses.append(odo.update(s.mcu_time_us, s.boot_id, s.wheel_count_left, s.wheel_count_right,
                                s.steering_rad).pose)

    start_counts = b.state().wheel_count_left
    b.run_until(lambda: False, timeout_s=1.0, every=send)
    s = b.state()
    assert s.mode == Mode.ARMED and s.reject_reason.name == "NONE"  # repeated DRIVEs were accepted
    rolled_m = (s.wheel_count_left - start_counts) / VEHICLE.counts_per_wheel_rev[0] * 2 * math.pi * 0.125
    assert rolled_m > 0.1  # about 0.3 m/s for most of a second, after the wheel's lag
    assert abs(s.steering_rad - cmd.steering_rad) < 0.02  # the steering axis followed the mechanism angle
    assert poses[-1][0] > 0.1 and poses[-1][2] > 0.0  # forward and turning left on the host odometry
    # Stop sending: the MCU stops by itself (STOPPING -> DISARMED) and keeps the timeout bit.
    assert b.run_until(lambda: b.state().mode == Mode.DISARMED, timeout_s=2.0)
    assert any(e.kind == "mode_changed" and e.detail["to"] == Mode.STOPPING for e in b.events)
    assert b.state().fault_flags & FaultFlag.COMMAND_TIMEOUT
    assert not b.host.ready


def test_restarted_mcu_is_a_new_boot_and_is_not_rearmed(bench_factory):
    b = bench_factory("--boot-id", "0x100")
    arm(b)
    b.stop()
    b.start("--boot-id", "0x200")
    assert b.run_until(lambda: b.state() is not None and b.state().boot_id == 0x200
                       and b.state().mode == Mode.DISARMED, timeout_s=3.0)
    kinds = [e.kind for e in b.events]
    assert "boot_changed" in kinds and "authorization_revoked" in kinds
    b.run_until(lambda: False, timeout_s=0.3)
    assert [type(m) for m in b.written()].count(Arm) == 1  # only the operator's
    assert b.state().mode == Mode.DISARMED and not b.host.ready


def keep_driving(b: Bench, cmd):
    """Renew a short DRIVE every cycle while authorized, so an injection meets a driving MCU."""
    def send(now):
        if b.host.ready:
            b.host.drive(cmd, 0.0, True, now + 60_000_000, now)
    return send


def revoked_for(b: Bench, reason: str) -> bool:
    return any(e.kind == "authorization_revoked" and e.detail.get("reason") == reason for e in b.events)


def test_driver_fault_while_driving_latches_and_only_the_operator_clears_it(bench_factory):
    b = bench_factory("--fault-at-ms", "900")
    arm(b)
    send = keep_driving(b, rear_drive_command(0.3, 0.0, VEHICLE))
    assert b.run_until(lambda: b.state().mode == Mode.FAULT, timeout_s=3.0, every=send)
    assert revoked_for(b, "mode_fault"), [(e.kind, e.detail) for e in b.events]  # driving until the fault
    assert not b.host.ready
    b.run_until(lambda: False, timeout_s=0.3)
    assert not any(isinstance(m, ClearFaults) for m in b.written())
    assert b.host.request_clear_faults(time.monotonic_ns()) is None
    assert b.run_until(lambda: b.state().mode == Mode.DISARMED, timeout_s=3.0)
    assert b.host.request_arm(time.monotonic_ns()) is None  # a separate operator ARM after the clear
    assert b.run_until(lambda: b.host.ready, timeout_s=3.0)


def test_estop_while_driving_latches(bench_factory):
    b = bench_factory("--estop-at-ms", "900", "--estop-release-ms", "1200")
    arm(b)
    send = keep_driving(b, rear_drive_command(0.3, 0.0, VEHICLE))
    assert b.run_until(lambda: b.state().mode == Mode.ESTOP, timeout_s=3.0, every=send)
    assert revoked_for(b, "mode_estop") and not b.host.ready
    b.run_until(lambda: False, timeout_s=0.5)  # released, still latched
    assert b.state().mode == Mode.ESTOP
    assert b.host.request_clear_faults(time.monotonic_ns()) is None
    assert b.run_until(lambda: b.state().mode == Mode.DISARMED, timeout_s=3.0)


def test_silent_mcu_goes_stale_and_that_revokes(bench_factory):
    b = bench_factory("--silent-at-ms", "900")
    arm(b)
    send = keep_driving(b, rear_drive_command(0.3, 0.0, VEHICLE))
    assert b.run_until(lambda: any(e.kind == "stale" for e in b.events), timeout_s=3.0, every=send)
    assert revoked_for(b, "stale") and not b.host.ready


def test_reconnecting_link_on_a_pty_slave_reads_nothing_without_hanging_up():
    """open_raw on a live tty with no data reads empty -- not a hangup; closing the far end is one
    (Codex P0-7 round 2 P1). The test writes STATE bytes into the master as the MCU."""
    master, slave = os.openpty()
    path = os.ttyname(slave)
    tty.setraw(slave)
    os.close(slave)
    host = McuLinkHost(CFG)
    link = ReconnectingLink(path, retry_s=0.05)
    now = time.monotonic_ns()
    events = link.service(host, now)
    assert [e.kind for e in events] == ["connected"]
    for _ in range(5):  # empty reads
        events += link.service(host, time.monotonic_ns())
    st = McuState(1000, 7, 0, 0, Mode.DISARMED, RejectReason.NONE, FaultFlag(0), 0, 0, 0.0, 0.0, 0.0, 0.0,
                  (0,) * 4, (0,) * 4, (0xC0,) * 4, 0, 0, StateFlag(0))
    os.write(master, encode(st, 1))
    time.sleep(0.02)
    events += link.service(host, time.monotonic_ns())
    kinds = [e.kind for e in events]
    assert "first_state" in kinds and "disconnected" not in kinds
    os.close(master)
    time.sleep(0.02)
    events = link.service(host, time.monotonic_ns())
    assert "disconnected" in [e.kind for e in events]
    link.service(host, time.monotonic_ns() + 100_000_000)  # reopen attempt on a vanished path: no crash
    assert link.link is None


def test_a_frame_refused_before_its_first_byte_can_still_be_cancelled():
    """EAGAIN on the first byte: the DRIVE goes back to the host, and a DISARM drops it (Codex P0-7 round 2 P1)."""
    host = McuLinkHost(CFG)
    now = time.monotonic_ns()
    st = lambda t, mode, gen, flags: encode(McuState(t, 7, gen, 0, mode, RejectReason.NONE, FaultFlag(0), 0, 0,  # noqa: E731
                                                     0.0, 0.0, 0.0, 0.0, (0,) * 4, (0,) * 4, (0xC0,) * 4, 0, 0,
                                                     flags), 1)
    host.on_bytes(st(1000, Mode.DISARMED, 0, StateFlag(0)), now)
    host.request_arm(now)
    host.next_frame(now)
    host.on_bytes(st(2000, Mode.ARMED, 1, StateFlag(0)), now + 1)
    host.next_frame(now + 1)
    host.on_bytes(st(3000, Mode.ARMED, 1, StateFlag.ZERO_SEEN), now + 2)
    assert host.ready
    rfd, wfd = os.pipe()
    os.set_blocking(wfd, False)
    try:
        while True:
            os.write(wfd, b"x" * 65536)
    except BlockingIOError:
        pass
    link = TtyLink(wfd)
    host.drive(rear_drive_command(0.2, 0.0, VEHICLE), 0.0, True, now + 50_000_000, now + 3)
    link.pump_writes(host, now + 4)  # EAGAIN before the first byte
    host.request_disarm(now + 5)
    os.set_blocking(rfd, False)
    while True:
        try:
            if not os.read(rfd, 65536):
                break
        except BlockingIOError:
            break
    link.pump_writes(host, now + 6)
    os.close(wfd)
    data = b""
    while chunk := os.read(rfd, 65536):
        data += chunk
    os.close(rfd)
    msgs = [r.message for r in LinkDecoder().feed(data, 0)]
    assert msgs == [Disarm()]


def test_a_drop_while_a_frame_is_half_written_finishes_it_on_the_wire(monkeypatch):
    """Real bytes through TtyLink, 3 at a time: the DRIVE being written when the generation changes goes out
    whole (decodable, never cut), stays outstanding, and the next frame decodes after it (4a' design D4)."""
    import real_mcu_link

    from forklift_core.plant.kinematics import RealKinematics
    from forklift_core.plant.port import RealPlantPort

    host = McuLinkHost(CFG)
    port = RealPlantPort(host, RealKinematics(VEHICLE))
    now = time.monotonic_ns()
    st = lambda t, mode, gen, flags: encode(McuState(t, 7, gen, 0, mode, RejectReason.NONE, FaultFlag(0), 0, 0,  # noqa: E731
                                                     0.0, 0.0, 0.0, 0.0, (0,) * 4, (0,) * 4, (0xC0,) * 4, 0, 0,
                                                     flags), 1)
    host.on_bytes(st(1000, Mode.DISARMED, 0, StateFlag(0)), now)
    host.request_arm(now)
    host.frame_written(host.next_frame(now), now)
    host.on_bytes(st(2000, Mode.ARMED, 1, StateFlag(0)), now + 1)
    host.frame_written(host.next_frame(now + 1), now + 1)
    host.on_bytes(st(3000, Mode.ARMED, 1, StateFlag.ZERO_SEEN), now + 2)
    rfd, wfd = os.pipe()
    os.set_blocking(rfd, False)
    real_write = os.write
    monkeypatch.setattr(real_mcu_link.os, "write", lambda fd, data: real_write(fd, bytes(data[:3])))
    link = TtyLink(wfd)
    d = RealKinematics(VEHICLE).command(0.3, 0.1)
    port.command(port.make_command(now, now + 200_000_000, d, np.asarray(d.steering_rad), 0.0, "keep"), now + 3)
    port.command(port.make_command(now, now + 200_000_000, d, np.asarray(d.steering_rad), 0.0, "keep"), now + 3)
    first = host.next_frame(now + 4)
    host.unsent(first, now + 4)
    calls = {"n": 0}

    def three_then_drop(fd, data):
        calls["n"] += 1
        if calls["n"] == 2:
            port.new_control_generation()  # mid-frame: drops the second DRIVE, not the one on the wire
        return real_write(fd, bytes(data[:3]))

    monkeypatch.setattr(real_mcu_link.os, "write", three_then_drop)
    for _ in range(40):
        link.pump_writes(host, now + 5)
    host.request_disarm(now + 6)
    for _ in range(40):
        link.pump_writes(host, now + 6)
    data = os.read(rfd, 65536)
    os.close(rfd)
    os.close(wfd)
    msgs = [r.message for r in LinkDecoder().feed(data, 0)]
    assert [type(m).__name__ for m in msgs] == ["Drive", "Disarm"]  # the first DRIVE whole, the second dropped
    assert msgs[0].left_wheel_rad_s != 0.0
    assert port.outstanding_until(now + 6) == now + 200_000_000


def test_receive_never_writes_and_transmit_writes_what_is_still_queued():
    """The teleop bridge's order (docs/design/2026-11-12-teleop-bridge.md 5): what a cycle drops between receive and
    transmit never goes out."""
    import select

    from forklift_core.plant.mcu_host import HostConfig, McuLinkHost

    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    try:
        link = ReconnectingLink(os.ttyname(slave))
        host = McuLinkHost(HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100,
                                      outbox_max=8))
        now = time.monotonic_ns()
        host.request_disarm(now)
        link.receive(host, now)
        assert not select.select([master], [], [], 0.05)[0]  # nothing written by receive
        link.transmit(host, now)
        assert select.select([master], [], [], 0.5)[0] and os.read(master, 64)  # the DISARM went out
    finally:
        os.close(master)
        os.close(slave)


def test_the_model_skips_late_cycles_so_kth_counts_stay_within_one_per_ms(bench_factory):
    """mcu-link/3 3.3: at most one KTH count per axis per control cycle and a cycle of at least 1 ms. A model
    stalled (SIGSTOP) and resumed must not run the missed cycles back to back -- the counters would then
    outrun the MCU clock (Codex, mcu-link/3 C review P2)."""
    b = bench_factory()
    assert b.run_until(lambda: b.state() is not None and b.state().mode == Mode.DISARMED), "no DISARMED STATE"
    seen = []

    def keep(_now):
        s = b.state()
        if s is not None and (not seen or s.mcu_time_us != seen[-1].mcu_time_us):
            seen.append(s)

    b.run_until(lambda: False, timeout_s=0.2, every=keep)
    b.proc.send_signal(signal.SIGSTOP)
    time.sleep(0.3)
    b.proc.send_signal(signal.SIGCONT)
    b.run_until(lambda: False, timeout_s=0.5, every=keep)
    assert len(seen) > 20
    for a, c in zip(seen, seen[1:]):
        dt_ms = (c.mcu_time_us - a.mcu_time_us) / 1000.0
        for axis in range(2):
            frames = (c.kth_frames[axis] - a.kth_frames[axis]) % 0x10000
            assert frames <= math.floor(dt_ms) + 1, (axis, frames, dt_ms)
