"""The first one-shot frame's answer and the bench's expected status bits (bench console plan 4.4 B2, decisions 3-4):
``bench_session.first_answer`` -- the evidence ordered by MCU time with a STATE before its own pair, the two
deadlines, the stop zero that ends STATE attribution -- and ``bench_session.unexpected_faults``."""

from dataclasses import dataclass

from forklift_core.mcu_link.messages import FaultFlag, Mode, RejectReason
from forklift_core.plant import bench_session as bs_

RUN = 0xBEEF
SEQ = 0x1234
BOOT, ARM = 7, 3


@dataclass
class St:
    mcu_time_us: int
    last_rx_seq: int = SEQ - 1
    reject_reason: RejectReason = RejectReason.NONE
    boot_id: int = BOOT
    arm_gen: int = ARM
    mode: Mode = Mode.ARMED


@dataclass
class Bs:
    result_run_id: int = 0


@dataclass
class W:
    done_ns: int = 1_000
    boot_id: int = BOOT
    arm_gen: int = ARM
    echo_mcu_time_us: int = 100


W0 = W()


def ans(evidence, *, now=2_000, w=W0, lim_mcu=10_000, lim_host=5_000, stop_zero_done_ns=None):
    return bs_.first_answer(evidence, seq16=SEQ, run_id=RUN, write=w, lim_mcu_us=lim_mcu, lim_host_ns=lim_host,
                            now_ns=now, stop_zero_done_ns=stop_zero_done_ns)


def test_accept_state_then_its_result_pair_in_one_read_is_accepted():
    st = St(500, last_rx_seq=SEQ)
    # the host calls every STATE hook of a read before its pair hooks: the pair is still ordered after its STATE
    assert ans([("state", st, None, 1_500), ("pair", st, Bs(RUN), 1_500)]) == "accepted"


def test_result_pair_without_an_accepting_state_confirms():
    st = St(500)
    assert ans([("pair", st, Bs(RUN), 1_500)]) == "confirmed_by_result"


def test_an_unrelated_reject_does_not_mask_a_result_of_this_run():
    # a STATE of the same MCU time carries a reject reason but its last_rx_seq is not before the first frame
    st = St(500, last_rx_seq=SEQ + 1, reject_reason=RejectReason.STALE_ECHO)
    assert ans([("state", st, None, 1_500), ("pair", st, Bs(RUN), 1_500)]) == "confirmed_by_result"


def test_a_reject_before_the_first_frame_is_its_refusal():
    st = St(500, reject_reason=RejectReason.SENSOR_INVALID)
    assert ans([("state", st, None, 1_500)]) == "refused:sensor_invalid"


def test_evidence_ordered_by_mcu_time_not_by_list_order():
    late = St(800, reject_reason=RejectReason.STALE_ECHO)
    early = St(600, last_rx_seq=SEQ)
    assert ans([("state", late, None, 1_500), ("state", early, None, 1_400)]) == "accepted"


def test_evidence_at_the_deadline_counts_and_after_it_closes():
    assert ans([("state", St(10_000, last_rx_seq=SEQ), None, 1_500)]) == "accepted"
    assert ans([("state", St(10_001, last_rx_seq=SEQ), None, 1_500)]) == "unconfirmed"
    assert ans([("state", St(500, last_rx_seq=SEQ), None, 5_000)]) == "accepted"
    assert ans([("state", St(500, last_rx_seq=SEQ), None, 5_001)]) == "unconfirmed"


def test_evidence_received_in_time_judged_on_a_later_cycle_still_counts():
    assert ans([("state", St(500, last_rx_seq=SEQ), None, 4_000)], now=9_000) == "accepted"


def test_pending_until_the_host_deadline_then_unconfirmed():
    assert ans([], now=4_999) is None
    assert ans([], now=5_001) == "unconfirmed"


def test_a_frame_never_written_closes_at_the_host_deadline():
    assert ans([("state", St(500, last_rx_seq=SEQ), None, 1_500)], w=None, now=4_000) is None
    assert ans([], w=None, now=5_001) == "unconfirmed"


def test_samples_of_another_arming_or_from_before_the_write_are_not_evidence():
    assert ans([("state", St(500, last_rx_seq=SEQ, arm_gen=ARM + 1), None, 1_500)], now=5_001) == "unconfirmed"
    assert ans([("state", St(500, last_rx_seq=SEQ), None, 900)], now=5_001) == "unconfirmed"
    assert ans([("state", St(100, last_rx_seq=SEQ), None, 1_500)], now=5_001) == "unconfirmed"


def test_after_the_stop_zero_only_a_result_pair_attributes_the_first_frame():
    # seq wrapped: the first frame 0xFFFF's accepting STATE was lost, the stop zero 0x0000 was accepted
    st = St(700, last_rx_seq=0x0000)
    ev = [("state", st, None, 3_000)]
    v = bs_.first_answer(ev, seq16=0xFFFF, run_id=RUN, write=W(), lim_mcu_us=10_000, lim_host_ns=5_000,
                         now_ns=5_001, stop_zero_done_ns=2_500)
    assert v == "unconfirmed"
    v = bs_.first_answer(ev + [("pair", st, Bs(RUN), 3_000)], seq16=0xFFFF, run_id=RUN, write=W(),
                         lim_mcu_us=10_000, lim_host_ns=5_000, now_ns=5_001, stop_zero_done_ns=2_500)
    assert v == "confirmed_by_result"
    # a refusal after the stop zero is the zero's, never the first frame's
    rej = St(700, reject_reason=RejectReason.STALE_ECHO)
    assert ans([("state", rej, None, 3_000)], now=5_001, stop_zero_done_ns=2_500) == "unconfirmed"


def test_unexpected_faults_leave_out_other_axes_and_a_bypassed_range():
    m_range_off = FaultFlag.STEER_OUT_OF_RANGE | FaultFlag.LIFT_SENSOR_INVALID | FaultFlag.COMMAND_TIMEOUT
    assert bs_.unexpected_faults(m_range_off, 2, ("steer_range",)) == ()
    # without the bypass the run axis's own range is unexpected
    assert bs_.unexpected_faults(m_range_off, 2, ()) == ("STEER_OUT_OF_RANGE",)
    # the run axis's driver and sensor faults always count
    assert bs_.unexpected_faults(m_range_off | FaultFlag.DRIVER_FAULT_M, 2, ("steer_range",)) == ("DRIVER_FAULT_M",)
    assert bs_.unexpected_faults(FaultFlag.STEER_SENSOR_INVALID, 2, ("steer_range",)) == ("STEER_SENSOR_INVALID",)
    # a wheel run: the steering and lift status bits are the bench's
    assert bs_.unexpected_faults(FaultFlag.STEER_OUT_OF_RANGE | FaultFlag.STEER_SENSOR_INVALID
                                 | FaultFlag.LIFT_OUT_OF_RANGE, 0, ()) == ()
    assert bs_.unexpected_faults(FaultFlag.DRIVER_FAULT_L | FaultFlag.ENCODER_STUCK_L, 0, ()) == (
        "DRIVER_FAULT_L", "ENCODER_STUCK_L")
    # a lift run: steering's bits are the bench's, lift's own range only with its bypass
    assert bs_.unexpected_faults(FaultFlag.STEER_OUT_OF_RANGE | FaultFlag.LIFT_OUT_OF_RANGE, 3, ("lift_range",)) == ()
    assert bs_.unexpected_faults(FaultFlag.LIFT_OUT_OF_RANGE, 3, ()) == ("LIFT_OUT_OF_RANGE",)
