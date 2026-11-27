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
                               SimpleNamespace(kind="ARM", seq=5, data=b"\x05\x01\x02\x00")])  # not a frame
def test_a_bench_frame_never_leaves_the_real_transport(f, granted):
    m, t = make()
    feed(m, t, state())
    if granted:
        assert t.grant([], 7) is True and t.real_operating
    assert t._write_first(f, 0) == "eagain"
    assert t._write_rest(f, 0) is False
    with pytest.raises(BlockingIOError):
        os.read(m, 256)  # not one byte reached the device


def test_an_ordinary_frame_still_goes_out_after_the_grant():
    m, t = make()
    feed(m, t, state())
    assert t.grant([], 7) is True
    assert t._write_first(frame("ARM", Arm(7, 1000)), 0) == "done"
    assert len(os.read(m, 256)) > 0
