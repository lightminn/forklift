"""PR 6a-2 (docs/design/2026-11-20-real-hil-6a.md 1-3, 8): the loop TTY transport on a pty pair -- the poison first
on every opening, a refused continuation abandoned on the same fd then the poison then the DISARM, the poison resumed
from its offset after EAGAIN, 0 bytes out back to the queue, only whole frames reported, the 4096-byte read cap with the
actual read time, the HIL check over every STATE of a boot, a lost device's reason, and the runner's pty (lock,
drain, refusals)."""

import errno
import os
import sys
import time
import tty
from collections import deque
from dataclasses import replace
from pathlib import Path

import pytest

from forklift_core.mcu_link.messages import Disarm, LinkDecoder, McuState, Mode, RejectReason, StateFlag, encode
from forklift_core.plant.mcu_host import Outgoing

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import real_loop_transport as rlt  # noqa: E402


class Host:
    def __init__(self):
        self.out, self.events, self.rx = deque(), [], []

    def queue(self, kind, seq, data=None):
        self.out.append(Outgoing(kind, seq, data or encode(Disarm(), seq), None, 0))

    def next_frame(self, now_ns):
        return self.out.popleft() if self.out else None

    def unsent(self, f, now_ns):
        self.out.appendleft(f)
        self.events.append(("unsent", f.seq))

    def frame_written(self, f, now_ns):
        self.events.append(("written", f.seq))

    def frame_abandoned(self, f, n, now_ns):
        self.events.append(("abandoned", f.seq, n))

    def drop_drives(self, why):
        self.events.append(("drop", why))

    def on_bytes(self, data, now_ns):
        self.rx.append((len(data), now_ns))


def pair():
    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    os.set_blocking(slave, False)
    return master, slave


def drain(fd):
    out = b""
    while True:
        try:
            chunk = os.read(fd, 65536)
        except (BlockingIOError, OSError):
            break
        if not chunk:
            break
        out += chunk
    return out


def gate_all(frame, write):
    write(judged_ns=123)
    return True


def state(**kw):
    s = McuState(1000, 7, 0, 0, Mode.DISARMED, RejectReason.NONE, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, (0,) * 4, (0,) * 4,
                 (0,) * 4, 0, 0, StateFlag.HIL_PLANT | StateFlag.OUTPUTS_LOCKED)
    return replace(s, **kw)


def test_the_first_bytes_are_the_poison_then_the_frame():
    m, s = pair()
    t = rlt.LoopTtyTransport(s)
    h = Host()
    h.queue("DISARM", 5)
    out = t.transmit(h, 0, gate_all)
    data = drain(m)
    assert data.startswith(rlt.POISON) and data[len(rlt.POISON):] == encode(Disarm(), 5)
    assert out == [("DISARM", 5, t.written_ns[5])] and t.judged[5] == 123


class Chunky:
    """A writer that takes ``k`` bytes per call, then EAGAIN on every other call."""

    def __init__(self, k):
        self.k, self.calls, self.sent = k, 0, b""

    def __call__(self, fd, data):
        self.calls += 1
        if self.calls % 2 == 0:
            raise BlockingIOError(errno.EAGAIN, "again")
        n = min(self.k, len(data))
        self.sent += bytes(data[:n])
        return n


def test_the_poison_resumes_from_its_offset_after_eagain():
    w = Chunky(32)
    t = rlt.LoopTtyTransport(-1, writer=w)
    h = Host()
    h.queue("DISARM", 9)
    for _ in range(20):
        t.transmit(h, 0, gate_all)
    assert w.sent == rlt.POISON + encode(Disarm(), 9)  # never restarted: 32-byte pieces add up to it once


def test_a_refused_continuation_is_abandoned_on_the_same_fd_then_poison_then_disarm():
    w = Chunky(4)
    t = rlt.LoopTtyTransport(-1, writer=w)
    t._poison_at = None  # noqa: SLF001 -- past the opening's poison
    h = Host()
    h.queue("DRIVE", 1, encode(Disarm(), 1) + b"")
    t.transmit(h, 0, gate_all)
    assert t.partial_frame is not None
    w.k = 1000
    h.queue("DISARM", 2)
    calls = {"n": 0}

    def gate(frame, write):
        calls["n"] += 1
        if frame.seq == 1:
            return False  # the continuation refused
        write(judged_ns=None)
        return True

    while not any(e[0] == "written" and e[1] == 2 for e in h.events):
        t.transmit(h, 0, gate)
    assert ("abandoned", 1, 4) in h.events and t.fd == -1 and t.resyncs == 1
    tail = w.sent[4:]
    assert tail == rlt.POISON + encode(Disarm(), 2)


def test_zero_bytes_out_goes_back_to_the_queue_head():
    def never(fd, data):
        raise BlockingIOError(errno.EAGAIN, "again")

    t = rlt.LoopTtyTransport(-1, writer=never)
    t._poison_at = None  # noqa: SLF001
    h = Host()
    h.queue("DISARM", 3)
    assert t.transmit(h, 0, gate_all) == []
    assert ("unsent", 3) in h.events and t.partial_frame is None and h.out[0].seq == 3


def test_a_read_takes_at_most_4096_bytes_with_the_actual_read_time():
    m, s = pair()
    os.write(m, encode(state(), 1) * 120)
    clock = iter([5_000_000_000, 5_000_000_001])
    t = rlt.LoopTtyTransport(s, clock=lambda: next(clock))
    h = Host()
    t.receive(h, 4_990_000_000)
    assert h.rx[0][0] <= rlt.READ_CAP and h.rx[0][1] == 5_000_000_000


def test_the_hil_check_needs_both_bits_on_every_state_of_the_boot():
    m, s = pair()
    t = rlt.LoopTtyTransport(s)
    h = Host()
    os.write(m, encode(state(), 1))
    t.receive(h, 0)
    assert t.hil_plant is True
    os.write(m, encode(state(mcu_time_us=2000, flags=StateFlag.HIL_PLANT), 2))  # OUTPUTS_LOCKED missing
    t.receive(h, 0)
    assert t.hil_plant is False
    os.write(m, encode(state(mcu_time_us=3000), 3))  # same boot, bits back: still refused
    t.receive(h, 0)
    assert t.hil_plant is False
    os.write(m, encode(state(boot_id=8, mcu_time_us=10), 4))  # a new boot starts over
    t.receive(h, 0)
    assert t.hil_plant is True
    os.write(m, encode(state(boot_id=9, flags=StateFlag.HIL_PLANT | StateFlag.OUTPUTS_LOCKED
                             | StateFlag.COMMISSION_BUILD), 5))
    t.receive(h, 0)
    assert t.hil_plant is False


def test_a_lost_device_names_its_first_path_and_a_left_half_frame():
    w = Chunky(4)
    t = rlt.LoopTtyTransport(-1, writer=w)
    t._poison_at = None  # noqa: SLF001
    h = Host()
    h.queue("DRIVE", 1)
    t.transmit(h, 0, gate_all)

    def dead(fd, data):
        raise OSError(errno.EIO, "gone")

    t._write = dead  # noqa: SLF001
    with pytest.raises(rlt.DeviceLost):
        t.transmit(h, 0, gate_all)
    assert t.lost["first"] == "write_eio" and t.lost["stream_tainted"] is True and t.hil_plant is False
    with pytest.raises(rlt.DeviceLost):
        t.receive(h, 0)


# --- the runner's pty ---------------------------------------------------------------------------------------------
def fake_model(tmp_path):
    p = tmp_path / "fake_model.py"
    p.write_text("import os, sys, time\nfd = int(sys.argv[sys.argv.index('--link-fd') + 1])\n"
                 "os.set_blocking(fd, False)\ntime.sleep(0.2)\n"
                 "try:\n    seen = os.read(fd, 4096)\nexcept OSError:\n    seen = b''\n"
                 "open(sys.argv[0] + '.seen', 'wb').write(seen)\n"
                 "open(sys.argv[0] + '.args', 'w').write(' '.join(sys.argv[1:]))\n"
                 "time.sleep(30)\n")
    exe = tmp_path / "fake_model"
    exe.write_text(f"#!{sys.executable}\nexec(open({str(p)!r}).read())\n")
    exe.chmod(0o755)
    return exe


def test_the_runner_pty_locks_drains_and_owns_the_model(tmp_path):
    exe = fake_model(tmp_path)
    hp = rlt.HilPty(exe, 3.0, before_spawn=lambda master, slave: os.write(slave, b"early"))
    try:
        path = os.ttyname(hp.slave)
        with pytest.raises(OSError) as e:
            os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        assert e.value.errno == errno.EBUSY or os.geteuid() == 0
        for _ in range(100):
            if Path(str(exe) + ".args").exists():
                break
            time.sleep(0.02)
        args = Path(str(exe) + ".args").read_text().split()
        assert args[:2] == ["--link-fd", args[1]] and "--hil-plant" in args
        assert args[args.index("--duration-ms") + 1] == "14000"  # 3 s + HIL_MODEL_EXTRA_S (11 s, 6a-2b)
        assert b"early" not in Path(str(exe) + ".seen").read_bytes()  # written before the lock: drained
    finally:
        rc = hp.close()
    assert rc is not None  # reaped


def test_the_runner_pty_refuses_reserved_model_arguments_and_a_shared_slave(tmp_path):
    exe = fake_model(tmp_path)
    with pytest.raises(rlt.HilRefused, match="hil_model_arg_reserved"):
        rlt.HilPty(exe, 3.0, ["--duration-ms"])
    with pytest.raises(rlt.HilRefused, match="duration_s"):
        rlt.HilPty(exe, 0)
    held = {}

    def opener(master, slave):
        held["fd"] = os.open(os.ttyname(slave), os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

    try:
        with pytest.raises(rlt.HilRefused, match="device_shared"):
            rlt.HilPty(exe, 3.0, before_spawn=opener)
    finally:
        if "fd" in held:
            os.close(held["fd"])


# --- Codex 6a-2a impl round 1 ------------------------------------------------------------------------------------
def raw_pair():
    master, slave = os.openpty()
    rlt._raw(slave)  # noqa: SLF001 -- the runner's own settings (VMIN 0, VTIME 0)
    os.set_blocking(master, False)
    os.set_blocking(slave, False)
    return master, slave


def test_an_empty_read_on_the_runners_pty_is_nothing_yet_and_a_hangup_is_lost():
    m, s = raw_pair()
    t = rlt.LoopTtyTransport(s)
    h = Host()
    t.receive(h, 0)  # nothing yet: not lost
    assert t.lost is None
    os.write(m, encode(state(), 1))
    t.receive(h, 0)
    assert t.lost is None and h.rx and t.hil_plant
    os.close(m)
    with pytest.raises(rlt.DeviceLost):
        t.receive(h, 0)
    assert t.lost["first"] in ("read_hup", "read_eio")


class Pieces:
    def __init__(self, k):
        self.k = k

    def __call__(self, fd, data):
        return min(self.k, len(data))


def test_each_piece_of_a_frame_passes_the_gate_again():
    w = Pieces(3)
    t = rlt.LoopTtyTransport(-1, writer=w)
    t._poison_at = None  # noqa: SLF001
    h = Host()
    h.queue("DISARM", 4)
    gates = []

    def gate(frame, write):
        gates.append(frame.seq)
        write(judged_ns=len(gates))
        return True

    for _ in range(40):
        t.transmit(h, 0, gate)
        if 4 in t.written_ns:
            break
    pieces = -(-len(encode(Disarm(), 4)) // 3)
    assert len(gates) == pieces  # one os.write per gate pass
    assert t.judged[4] == pieces and t.last_write_ns[4] <= t.written_ns[4]


def test_a_boot_id_zero_state_neither_opens_nor_resets_the_hil_check():
    m, s = pair()
    t = rlt.LoopTtyTransport(s)
    h = Host()
    os.write(m, encode(state(boot_id=0), 1))
    t.receive(h, 0)
    assert t.hil_plant is False
    os.write(m, encode(state(flags=StateFlag.HIL_PLANT), 2))  # boot 7, a bit missing
    t.receive(h, 0)
    os.write(m, encode(state(boot_id=0, mcu_time_us=5000), 3))
    os.write(m, encode(state(mcu_time_us=6000), 4))  # boot 7 again: still refused
    t.receive(h, 0)
    assert t.hil_plant is False


class LossHost(Host):
    def __init__(self):
        super().__init__()
        self.disconnects = 0

    def disconnected(self, now_ns):
        self.disconnects += 1


def test_a_loss_tells_the_host_once_and_returns_the_frames_finished_before_it():
    calls = {"n": 0}

    def writer(fd, data):
        calls["n"] += 1
        if calls["n"] >= 2:
            raise OSError(errno.EIO, "gone")
        return len(data)

    t = rlt.LoopTtyTransport(-1, writer=writer)
    t._poison_at = None  # noqa: SLF001
    h = LossHost()
    h.queue("DISARM", 1)
    h.queue("DISARM", 2)
    out = t.transmit(h, 0, gate_all)
    assert [o[1] for o in out] == [1] and h.disconnects == 1
    with pytest.raises(rlt.DeviceLost):
        t.transmit(h, 0, gate_all)
    with pytest.raises(rlt.DeviceLost):
        t.receive(h, 0)
    assert h.disconnects == 1


def test_closing_the_runner_pty_twice_never_closes_a_reused_number(tmp_path):
    exe = fake_model(tmp_path)
    hp = rlt.HilPty(exe, 3.0)
    n = hp.slave
    hp.close()
    reused = os.open(os.devnull, os.O_RDONLY)
    try:
        hp.close()
        os.fstat(reused)  # still open
    finally:
        os.close(reused)
    assert n >= 0 and hp.slave == -1
