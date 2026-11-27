"""The HIL and REAL transports never write a byte of a bench command (bench console plan A1a 1, contract 8): before
and after the REAL motion permission, by its kind or by the type byte of its own bytes. The hosts of those executors
cannot even queue one (tests/unit/plant/test_mcu_host_bench.py); this is the second wall."""

import os
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from forklift_core.mcu_link.messages import Arm, BenchArm, BenchSet, encode

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from test_real_tty_transport import feed, make, state  # noqa: E402

SET = BenchSet(7, 1000, 9, 1, 0, 0, 1, 0, (10000, 10000, 1500, 1500), (2147483, 2147483, 2000, 2000), 0)


def frame(kind, msg, seq=5):
    return SimpleNamespace(kind=kind, seq=seq, data=encode(msg, seq))


def hold_bytes(seq=5):
    """A frame of the reserved BENCH_HOLD type 0x07 (A1b) built from an ARM's bytes: only its type byte differs."""
    from forklift_core.mcu_link.framing import cobs_decode, cobs_encode, crc16_ccitt_false

    body = bytearray(cobs_decode(encode(Arm(7, 1000), seq)[:-1])[:-2])
    body[1] = 0x07
    body += crc16_ccitt_false(bytes(body)).to_bytes(2, "little")
    return cobs_encode(bytes(body)) + b"\x00"


@pytest.mark.parametrize("granted", [False, True])
@pytest.mark.parametrize("f", [frame("BENCH_SET", SET), frame("BENCH_ARM", BenchArm(7, 1000, 9)),
                               frame("ARM", BenchArm(7, 1000, 9)),  # mislabelled: the type byte still tells
                               frame("BENCH_SET", Arm(7, 1000)),    # labelled: the kind tells
                               SimpleNamespace(kind="BENCH_HOLD", seq=5, data=encode(Arm(7, 1000), 5)),
                               SimpleNamespace(kind="ARM", seq=5, data=hold_bytes()),  # the reserved 0x07
                               SimpleNamespace(kind="ARM", seq=5,  # a buffer holding a bench frame second
                                               data=encode(Arm(7, 1000), 5) + encode(BenchArm(7, 1000, 9), 6)),
                               SimpleNamespace(kind="ARM", seq=5, data=b"\x05\x01\x02\x00"),  # not a frame
                               SimpleNamespace(kind="ARM", seq=5, data=encode(Arm(7, 1000), 5)[:-1]),  # no delimiter
                               SimpleNamespace(kind="ARM", seq=5,  # a CRC error
                                               data=encode(Arm(7, 1000), 5)[:-3] + b"\x07\x07\x00")])
def test_a_bench_frame_never_leaves_the_real_transport(f, granted):
    m, t = make()
    feed(m, t, state())
    if granted:
        assert t.grant([], 7) is True and t.real_operating
    assert t._write_first(f, 0) == "refused"
    assert t._write_rest(f, 0) is False
    with pytest.raises(BlockingIOError):
        os.read(m, 256)  # not one byte reached the device


def test_an_ordinary_frame_still_goes_out_after_the_grant():
    m, t = make()
    feed(m, t, state())
    assert t.grant([], 7) is True
    assert t._write_first(frame("ARM", Arm(7, 1000)), 0) == "done"
    assert len(os.read(m, 256)) > 0


class _Host:
    """The minimum of the host the transmit loop drives: a queue and what happened to each frame."""

    def __init__(self, frames):
        self.q, self.written, self.refused, self.unsent = list(frames), [], [], []

    def next_frame(self, now_ns):
        return self.q.pop(0) if self.q else None

    def frame_written(self, f, ns):
        self.written.append(f.kind)

    def frame_refused(self, f, ns, why):
        self.refused.append((f.kind, why))

    def unsent(self, f, ns):
        self.unsent.append(f.kind)
        self.q.insert(0, f)

    def drop_drives(self, why):
        pass


def test_a_refused_bench_frame_is_dropped_and_the_disarm_behind_it_goes_out():
    """A bench frame (even labelled DISARM) never blocks the queue: dropped, and the real DISARM after it is sent."""
    from forklift_core.mcu_link.messages import Disarm

    m, t = make()
    feed(m, t, state())
    host = _Host([SimpleNamespace(kind="DISARM", seq=5, data=encode(BenchArm(7, 1000, 9), 5)),
                  SimpleNamespace(kind="DISARM", seq=6, data=encode(Disarm(), 6))])
    t.transmit(host, 0, lambda f, write: (write(), True)[1])
    assert host.refused == [("DISARM", "bench_frame_on_this_transport")]
    assert host.written == ["DISARM"] and host.unsent == []


def test_a_bench_frame_is_dropped_before_the_operating_gate_so_a_disarm_still_goes():
    """The operating gate refuses a motion frame without the permission before any write: a bench frame must not reach
    it and come back to the queue head, or the DISARM behind it would wait forever."""
    from forklift_core.mcu_link.messages import Disarm

    m, t = make()
    feed(m, t, state())
    host = _Host([SimpleNamespace(kind="BENCH_ARM", seq=5, data=encode(BenchArm(7, 1000, 9), 5)),
                  SimpleNamespace(kind="DISARM", seq=6, data=encode(Disarm(), 6))])

    def gate(f, write):  # the operating gate's shape: only a DISARM passes without the permission
        if f.kind != "DISARM":
            return False
        write()
        return True

    t.transmit(host, 0, gate)
    assert host.refused == [("BENCH_ARM", "bench_frame_on_this_transport")] and host.written == ["DISARM"]
