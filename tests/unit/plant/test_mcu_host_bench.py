"""The bench image on the mcu_link host (bench console plan A1a 1, contract 8): only a commission-mode host, on the
opened tty whose product string is exactly the bench7 image, whose STATE says an open commissioning build, sends
BENCH_SET and BENCH_ARM; a plain ARM is refused there; a BENCH_STATE counts only right after its STATE."""

import pytest

from forklift_core.mcu_link.build_policy import Client
from forklift_core.mcu_link.messages import (BenchArm, BenchHold, BenchSet, BenchState, LinkDecoder, Mode,
                                            StateFlag, encode)
from forklift_core.plant import bench_limits as bl
from forklift_core.plant.mcu_host import McuLinkHost

from test_mcu_host_commission import CB, CFG, MS, Rig, state

BENCH5 = "forklift-mcu commission OPEN v0-1-gabcdefabcdef rec=1f4207f4 lim=lr1 bench7"
ASK = bl.BenchAsk(0, (), None, None)


def bench_rig(product=BENCH5, **kw):
    r = Rig(**kw)
    r.host.usb_product = product
    return r


def test_only_a_commission_host_on_the_bench6_image_sends_a_bench_set():
    for client in (Client.REAL, Client.HIL, Client.BRINGUP):
        h = McuLinkHost(CFG, client=client)
        h.usb_product = BENCH5
        assert h.bench_set(1, ASK, 0).reason == "bench_refused: not_commission_host"
        assert h.request_bench_arm(1, 0).reason == "bench_refused: not_commission_host"
    for product in (None, "forklift-mcu commission OPEN v rec=1 lim=lr1",
                    "forklift-mcu commission OPEN v rec=1 lim=lr1 bench4"):
        r = bench_rig(product)
        r.feed()
        assert r.host.bench_set(1, ASK, r.now).reason == "bench_refused: not_bench6_image"
    r = bench_rig()
    r.feed(flags=StateFlag.DRIVERS_READY)  # not a commissioning build
    assert r.host.bench_set(1, ASK, r.now).reason == "bench_refused: not_commission_build"
    for bad in (StateFlag.OUTPUTS_LOCKED, StateFlag.HIL_PLANT):
        r = bench_rig(allow_locked=True)
        r.feed(flags=CB | bad)
        assert r.host.bench_set(1, ASK, r.now).reason == "bench_refused: outputs_locked_or_hil"
    assert r.sent() == []


def test_a_bench_set_echoes_the_state_and_a_bench_arm_runs_the_arming_exchange():
    r = bench_rig()
    r.feed()
    assert r.host.bench_set(0xABCD, bl.BenchAsk(2, ("steer_range",), 0.25, 5.0), r.now) is None
    (bs,) = r.sent()
    assert isinstance(bs, BenchSet) and bs.run_id == 0xABCD and bs.echo_mcu_time_us == r.t_us
    assert bs.axis == 2 and bs.bypass == 1 and bs.duty_max_e4[2] == 2500 and bs.time_ms[2] == 5000
    assert bs.mode == 1 and bs.source == 0 and bs.stop_counts == 0 and bs.count_sign == 1
    assert r.host.request_arm(r.now).reason == "bench_image_arms_with_bench_arm"
    r.feed()
    assert r.host.request_bench_arm(0xABCD, r.now) is None
    (arm,) = r.sent()
    assert isinstance(arm, BenchArm) and arm.run_id == 0xABCD and arm.echo_mcu_time_us == r.t_us
    r.feed(mode=Mode.ARMED, gen=1)
    (zero,) = r.sent()
    assert zero.axis == 0 and zero.duty_e4 == 0  # the arming's L(0): preparation on the bench image
    r.feed(mode=Mode.ARMED, gen=1, flags=CB | StateFlag.ZERO_SEEN)
    assert r.host.ready


@pytest.mark.parametrize("count_sign", [1, -1])
@pytest.mark.parametrize("stop", [-4096, 40])
def test_a_distance_bench_set_carries_its_source_stop_and_count_sign(count_sign, stop):
    """bench6 (plan 4.2 A1b-1 decision 8): the KTH source, the signed stop and count_sign reach the SET bytes."""
    r = bench_rig()
    r.feed()
    assert r.host.bench_set(7, bl.BenchAsk(2, (), 0.2, 3.0), r.now, count_sign=count_sign, source=1,
                            stop_counts=stop) is None
    (bs,) = r.sent()
    assert bs.source == 1 and bs.stop_counts == stop and bs.count_sign == count_sign


def test_a_bench_set_needs_disarmed_and_no_arming_in_progress():
    r = bench_rig()
    r.feed(mode=Mode.ARMED, gen=1)
    assert r.host.bench_set(1, ASK, r.now).reason == "not_disarmed"


def _bench_state(pair_seq, run_id=5):
    return BenchState(pair_seq, run_id, run_id, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0xFFFF, 0xFFFF, 0, 0x13, 0, 0,
                      0, 0)


def test_a_bench_state_counts_only_right_after_its_state():
    r = bench_rig()
    r.t_us += 10_000
    r.now += 10 * MS
    good = state(r.t_us) + encode(_bench_state(1), 2)  # the STATE helper encodes with seq 1
    r.host.on_bytes(good, r.now)
    assert r.host.bench_state is not None and r.host.bench_state.result_run_id == 5
    r.host.on_bytes(encode(_bench_state(1, run_id=6), 3), r.now)  # no STATE right before it: not a pair
    assert r.host.bench_state.result_run_id == 5
    r.t_us += 10_000
    r.host.on_bytes(state(r.t_us) + encode(_bench_state(0x0FFF, run_id=7), 4), r.now)  # names another seq
    assert r.host.bench_state.result_run_id == 5
    r.host.disconnected(r.now)
    assert r.host.bench_state is None


def test_the_first_state_of_the_connection_decides_the_image():
    r = bench_rig(allow_locked=True)
    r.feed(flags=CB | StateFlag.HIL_PLANT)  # the first STATE says a HIL plant
    r.feed()  # a later one looks fine
    assert r.host.bench_set(1, ASK, r.now).reason == "bench_refused: outputs_locked_or_hil"
    r.host.disconnected(r.now)  # a new connection judges again
    r.host.usb_product = BENCH5
    r.feed()
    assert r.host.bench_set(1, ASK, r.now) is None


def test_a_queued_bench_set_never_reaches_another_boot():
    from test_mcu_host_commission import BOOT

    r = bench_rig()
    r.feed()
    assert r.host.bench_set(1, ASK, r.now) is None
    r.t_us += 10_000
    r.now += 10 * MS
    from forklift_core.mcu_link.messages import FaultFlag, McuState, RejectReason

    other = McuState(r.t_us, BOOT + 1, 0, 0, Mode.DISARMED, RejectReason.NONE, FaultFlag(0), 0, 0, 0.0, 0.0,
                     float("nan"), 0.0, (0, 0, 0, 0), (0, 0, 0, 0), (0xC0,) * 4, 0, 0, CB)
    r.host.on_bytes(encode(other, 1), r.now)
    assert r.sent() == []


def test_a_pair_split_across_two_reads_still_counts():
    r = bench_rig()
    r.t_us += 10_000
    r.now += 10 * MS
    whole = state(r.t_us) + encode(_bench_state(1, run_id=8), 2)
    cut = len(state(r.t_us)) + 5  # inside the BENCH_STATE
    r.host.on_bytes(whole[:cut], r.now)
    assert r.host.bench_state is None
    r.host.on_bytes(whole[cut:], r.now)
    assert r.host.bench_state is not None and r.host.bench_state.result_run_id == 8


def test_a_pair_followed_by_the_next_state_in_one_read_still_counts():
    """S1 | B1+S2 | B2+S3: each read ends a pair and starts the next STATE; every completed pair is kept."""
    r = bench_rig()
    seq = 1
    chunks, run = [], 20
    for _ in range(4):
        r.t_us += 10_000
        chunks.append((encode_state(r.t_us, seq), encode(_bench_state(seq, run_id=run), (seq + 1) & 0xFFFF)))
        seq += 2
        run += 1
    reads = [chunks[0][0]] + [chunks[i][1] + chunks[i + 1][0] for i in range(3)]
    for data in reads:
        r.now += 10 * MS
        r.host.on_bytes(data, r.now)
    assert r.host.bench_state is not None and r.host.bench_state.result_run_id == 22
    assert r.host.bench_state_of is not None


def test_a_pair_whose_header_seq_does_not_follow_the_state_is_not_one():
    r = bench_rig()
    r.t_us += 10_000
    r.now += 10 * MS
    r.host.on_bytes(state(r.t_us) + encode(_bench_state(1, run_id=9), 8), r.now)  # pair_seq right, header seq not
    assert r.host.bench_state is None


def encode_state(t_us, seq):
    from forklift_core.mcu_link.messages import FaultFlag, McuState, RejectReason
    from test_mcu_host_commission import BOOT

    return encode(McuState(t_us, BOOT, 0, 0, Mode.DISARMED, RejectReason.NONE, FaultFlag(0), 0, 0, 0.0, 0.0,
                           float("nan"), 0.0, (0, 0, 0, 0), (0, 0, 0, 0), (0xC0,) * 4, 0, 0, CB), seq)



def test_drop_frames_removes_only_the_named_kinds():
    """The bench console's cancelled run (plan 2026-11-27 3.1): its queued BENCH_SET goes, a DISARM queued after it
    stays, and nothing of the dropped frame is written later."""
    r = bench_rig()
    r.feed()
    assert r.host.bench_set(9, ASK, r.now) is None
    r.host._enqueue_disarm(r.now)  # a DISARM behind it, as the stop rule's would be
    r.host.drop_frames(("BENCH_SET", "BENCH_ARM"), "bench_cancelled")
    assert [type(m).__name__ for m in r.sent()] == ["Disarm"]


def test_drop_frames_can_keep_the_arming_zero():
    """A bench stop right after the ARM was answered keeps the host's own arming zero (plan 3.1 round 3 P1-2)."""
    r = bench_rig()
    r.feed()
    assert r.host.request_bench_arm(5, r.now) is None
    r.sent()
    r.feed(mode=Mode.ARMED, gen=1)  # the arming zero is queued now
    r.host.drop_frames(("COMMISSION",), "bench_cancelled", keep_arming_zero=True)
    (zero,) = r.sent()
    assert zero.duty_e4 == 0 and zero.axis == 0


def test_the_accepted_hooks_see_only_what_the_host_accepted():
    """bench6 (Codex A1b-1b round 4): on_accepted and on_pair report the STATEs and pairs the host itself took, in
    order -- an out-of-order STATE it dropped never reaches them."""
    r = bench_rig()
    got_states, got_pairs = [], []
    r.host.on_accepted = lambda st, rx: got_states.append(st.mcu_time_us)
    r.host.on_pair = lambda st, bs, rx: got_pairs.append(bs.result_run_id)
    r.t_us += 10_000
    r.now += 10 * MS
    t_good = r.t_us
    r.host.on_bytes(state(r.t_us) + encode(_bench_state(1), 2), r.now)
    r.host.on_bytes(state(r.t_us - 5_000), r.now)  # older than the accepted one: dropped as out of order
    assert got_states == [t_good] and got_pairs == [5]


# ---- bench7 hold (plan 4.3 A1b-2 decision 10) ----------------------------------------------------------------------
HOLD = bl.HoldAsk(3, -40, kd=0.02)
HOLD_ASK = bl.BenchAsk(2, (), None, None, source=1, mode=2, hold=HOLD)


def _bench_ready(r, zero_axis=2):
    r.host.commission_zero_axis = zero_axis  # the hold run's readiness zero on its own axis (independent round 2 P1-1)
    r.feed()
    assert r.host.bench_set(9, HOLD_ASK, r.now, source=HOLD_ASK.source) is None
    (bs,) = r.sent()
    assert bs.mode == 2 and bs.source == 1 and bs.axis == 2
    r.feed()
    assert r.host.request_bench_arm(9, r.now) is None
    r.sent()
    r.feed(mode=Mode.ARMED, gen=1)
    (zero,) = r.sent()
    r.feed(mode=Mode.ARMED, gen=1, flags=CB | StateFlag.ZERO_SEEN)
    assert r.host.ready
    return zero


def test_a_hold_run_arms_with_its_zero_on_the_run_axis_and_sends_bench_hold():
    r = bench_rig()
    zero = _bench_ready(r)
    assert zero.axis == 2 and zero.duty_e4 == 0
    assert r.host.request_bench_hold(9, 2, HOLD, -1, r.now + 100 * MS, r.now) is None
    (h,) = r.sent()
    assert isinstance(h, BenchHold)
    assert (h.run_id, h.axis, h.source, h.target_kind, h.target_value, h.cpr) == (9, 2, 1, 3, -40, 4096)
    assert (h.kp, h.ki) == (0.5, 0.0) and abs(h.kd - 0.02) < 1e-7
    assert (h.deadband_counts, h.duty_max_e4, h.min_duty_e4, h.sign) == (12, 1500, 1200, -1)
    assert h.arm_gen == 1 and h.echo_mcu_time_us == r.t_us and 1 <= h.valid_after_echo_ms <= 100


def test_a_bench_hold_follows_the_motion_rules():
    r = bench_rig()
    assert r.host.request_bench_hold(9, 2, HOLD, 1, r.now + 100 * MS, r.now).reason == "bench_refused: no_state"
    r = bench_rig()
    r.feed()
    assert r.host.request_bench_hold(9, 2, HOLD, 1, r.now + 100 * MS, r.now).reason == "not_authorized"
    r = bench_rig("forklift-mcu commission OPEN v rec=1 lim=lr1 bench6")
    r.feed()
    assert r.host.request_bench_hold(9, 2, HOLD, 1, r.now + 100 * MS, r.now).reason.startswith("bench_refused")
    # queued, then past its expiry before the write: dropped, never written
    r = bench_rig()
    _bench_ready(r)
    assert r.host.request_bench_hold(9, 2, HOLD, 1, r.now + 5 * MS, r.now) is None
    r.now += 6 * MS
    assert r.sent() == []
    # queued, then the authorization ends (a DISARM): dropped
    assert r.host.request_bench_hold(9, 2, HOLD, 1, r.now + 100 * MS, r.now) is None
    r.host.request_disarm(r.now)
    assert [type(m).__name__ for m in r.sent()] == ["Disarm"]
