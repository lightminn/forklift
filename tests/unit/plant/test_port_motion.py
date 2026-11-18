"""Real control tick contract 2 (docs/design/2026-11-18-real-control-tick.md) for 4c: the port's result names its seq;
with ``lift_motion_counts`` a zero-wheel lift move and ``keep`` count as commands that may still act; the control stop
is one call (a new generation, then a stop frame at the measured position, not through the cached intent); and the
outstanding evidence the AUTO permit reads (settled since / pending / unknown). Off, the port is as before."""

import numpy as np
from test_mcu_host import MS, state  # noqa: E402
from test_port import CFG, KIN, READY, Bench  # noqa: E402

from forklift_core.mcu_link.messages import Drive, LinkDecoder, Mode
from forklift_core.plant.mcu_host import McuLinkHost
from forklift_core.plant.port import PortResult, RealPlantPort


class MotionBench(Bench):
    def __init__(self, counts=True):
        super().__init__()
        self.port = RealPlantPort(self.host, KIN, lift_motion_counts=counts)

    def written(self):
        return self.host.drive_traffic()["written"]


def armed(counts=True):
    b = MotionBench(counts)
    b.arm()
    return b


def test_a_queued_result_names_its_seq_and_still_equals_the_plain_queued():
    b = armed()
    r = b.port.command(b.cmd(), b.now)
    assert r == PortResult("queued") and r.seq is not None
    (f,) = [b.host.next_frame(b.now)]
    assert f.seq == r.seq


def test_a_zero_wheel_lift_move_and_keep_count_as_motion_only_when_asked():
    b = armed()
    b.feed(mode=Mode.ARMED, gen=b.gen, flags=READY)  # measured lift 0 (the state() default)
    b.port.command(b.cmd(speed=0.0, kappa=0.0, lift_m=0.10), b.now)  # wheels 0, the lift going up
    b.drain()
    b.port.command(b.cmd(speed=0.0, kappa=0.0, lift_mode="keep"), b.now)
    b.drain()
    b.port.command(b.cmd(speed=0.0, kappa=0.0, lift_m=0.0), b.now)  # wheels 0, the lift where it is: a wait frame
    b.drain()
    assert [w["nonzero"] for w in b.written()] == [False, True, True, False]  # the arm's zero, then the three
    off = armed(counts=False)
    off.port.command(off.cmd(speed=0.0, kappa=0.0, lift_m=0.10), off.now)
    off.drain()
    assert [w["nonzero"] for w in off.written()] == [False, False]  # as before: the wheels only


def test_the_control_stop_is_one_call_a_new_generation_then_the_stop_frame_at_the_measured_lift():
    b = armed()
    b.port.command(b.cmd(speed=0.3, kappa=0.0, lift_m=0.10), b.now)  # a moving frame queued, not yet written
    gen = b.port.control_gen
    r = b.port.control_stop(steering_rad=0.02, lift_m=0.07, now_ns=b.now, expires_ns=b.now + 100 * MS)
    assert r.kind == "queued" and r.seq is not None and b.port.control_gen == gen + 1
    msgs = b.drain()
    assert len(msgs) == 1 and isinstance(msgs[0], Drive)  # the old frame dropped, only the stop frame goes
    m = msgs[0]
    assert (m.left_wheel_rad_s, m.right_wheel_rad_s) == (0.0, 0.0) and not m.lift_hold
    assert abs(m.steering_rad - 0.02) < 1e-9 and abs(m.lift_m - 0.07) < 1e-9
    assert b.written()[-1]["nonzero"] is False  # the stop frame itself is no motion


def test_outstanding_is_unknown_without_state_pending_while_motion_may_act_then_settled_since():
    b = MotionBench()
    assert b.port.outstanding(b.now) == ("unknown", None)
    b.arm()
    kind, _ = b.port.outstanding(b.now)
    assert kind == "settled"
    b.port.command(b.cmd(speed=0.3, kappa=0.0), b.now)
    assert b.port.outstanding(b.now) == ("pending", None)  # queued
    b.drain()
    assert b.port.outstanding(b.now) == ("pending", None)  # written, not yet past its expiry
    b.now += 200 * MS  # past the 100 ms expiry
    b.feed(mode=Mode.ARMED, gen=b.gen, flags=READY)
    kind, since = b.port.outstanding(b.now)
    assert kind == "settled" and since == b.now  # settled from the first look that saw nothing left


def test_a_partly_written_frame_counts_until_its_expiry():
    b = armed()
    b.port.command(b.cmd(speed=0.3, kappa=0.0), b.now)
    f = b.host.next_frame(b.now)  # handed to the adapter: in flight, not finished
    assert f is not None and b.port.outstanding(b.now)[0] == "pending"
