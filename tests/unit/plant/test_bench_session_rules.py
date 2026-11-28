"""The bench console's pure rules (bench console plan 3, 3.1 B1): the one stop rule by drive state, the reopen rule
(a sample after the boundary, then by mode), the clear-faults binding, the result's first verdict and the window
stop with invalid KTH samples."""

import math

import pytest

from forklift_core.mcu_link.messages import BenchDriveState as D
from forklift_core.mcu_link.messages import FaultFlag, Mode
from forklift_core.plant import bench_session as bs


def pair(mode=Mode.ARMED, drive=D.PREP, *, boot=7, conn=1, arm_gen=3, last_rx=100, run_id=5, result_run_id=0,
         stop_reason=0, faults=FaultFlag(0), duty=(0, 0, 0, 0), t=1000):
    return bs.Pair(boot_id=boot, conn_gen=conn, arm_gen=arm_gen, mode=mode, last_rx_seq=last_rx, drive_state=drive,
                   run_id=run_id, result_run_id=result_run_id, stop_reason=stop_reason, fault_flags=faults,
                   duty_e4=duty, mcu_time_us=t)


# --- the 16-bit seq comparison (plan 3, 10th round P1) -------------------------------------------------------------
@pytest.mark.parametrize("last,seq,ok", [(100, 100, True), (101, 100, True), (99, 100, False),
                                         (0x0000, 0xFFFF, True), (0xFFFE, 0x0000, False), (0x7FFF, 0, True)])
def test_seq_reached_is_modular(last, seq, ok):
    assert bs.seq_reached(last, seq) is ok


# --- the stop rule ----------------------------------------------------------------------------------------------------
LINK = (7, 1, 3)  # boot, connection generation, arming generation of the zero's link


def test_before_the_bench_arm_a_stop_only_cancels():
    assert bs.stop_action(arm_sent=False, zero_seq=None, zero_link=None, p=pair()) == "cancel"


def test_after_the_arm_without_a_zero_yet_a_zero_goes_first():
    assert bs.stop_action(arm_sent=True, zero_seq=None, zero_link=None, p=pair(drive=D.DRIVING)) == "send_zero"
    assert bs.stop_action(arm_sent=True, zero_seq=None, zero_link=None, p=None) == "send_zero"


def test_while_the_arming_exchange_is_open_a_stop_waits_and_a_late_disarmed_pair_never_closes():
    """ARM written -> STOP -> the earlier DISARMED pair arrives late -> the new ARMED comes (Codex B1 design P1-3)."""
    late = pair(Mode.DISARMED, D.NONE)
    assert bs.stop_action(arm_sent=True, zero_seq=None, zero_link=None, p=late, arming=True, after_arm=False) == "wait"
    assert bs.stop_action(arm_sent=True, zero_seq=None, zero_link=None, p=late, arming=False, after_arm=False) \
        == "send_zero"  # no evidence since the ARM: the zero still goes (the host refuses it until ready)
    armed = pair(Mode.ARMED, D.PREP)
    assert bs.stop_action(arm_sent=True, zero_seq=None, zero_link=None, p=armed, arming=False) == "send_zero"


def test_an_arming_refused_closes_on_a_disarmed_sample_after_the_arm():
    p = pair(Mode.DISARMED, D.NONE)
    assert bs.stop_action(arm_sent=True, zero_seq=None, zero_link=None, p=p, arming=False, after_arm=True) == "closed"


def test_a_fault_during_the_arming_exchange_closes_latched():
    p = pair(Mode.FAULT, D.NONE)
    assert bs.stop_action(arm_sent=True, zero_seq=None, zero_link=None, p=p, arming=True) == "closed_latched"


def test_driving_after_the_zero_waits_for_the_boards_own_stop_never_a_disarm():
    """Same fd: STOP while driving -> the zero taken -> the STOPPING STATE late: no DISARM in the brake grace."""
    for p in (pair(drive=D.DRIVING, last_rx=200), pair(Mode.STOPPING, D.ENDED, last_rx=200),
              pair(drive=D.ENDED, last_rx=200)):
        assert bs.stop_action(arm_sent=True, zero_seq=200, zero_link=LINK, p=p) == "wait"


def test_prep_after_the_zero_was_taken_disarms():
    assert bs.stop_action(arm_sent=True, zero_seq=200, zero_link=LINK, p=pair(drive=D.PREP, last_rx=200)) \
        == "send_disarm"


def test_a_late_prep_sample_from_before_the_zero_never_disarms():
    """A 'prep' sample made before the zero was taken (its last_rx before the zero's seq) is no evidence."""
    assert bs.stop_action(arm_sent=True, zero_seq=200, zero_link=LINK, p=pair(drive=D.PREP, last_rx=199)) \
        == "resend_zero"


def test_the_same_order_across_the_seq_wrap_sends_no_disarm():
    """Late sample 0xFFFE, first non-zero 0xFFFF, the STOP's zero 0x0000: the late 'prep' sample does not pass."""
    late = pair(drive=D.PREP, last_rx=0xFFFE)
    assert bs.stop_action(arm_sent=True, zero_seq=0x0000, zero_link=LINK, p=late) == "resend_zero"
    after = pair(drive=D.DRIVING, last_rx=0x0000)
    assert bs.stop_action(arm_sent=True, zero_seq=0x0000, zero_link=LINK, p=after) == "wait"


def test_a_sample_of_another_link_or_arming_is_never_the_zeros_evidence():
    for p in (pair(drive=D.PREP, last_rx=200, boot=8), pair(drive=D.PREP, last_rx=200, conn=2),
              pair(drive=D.PREP, last_rx=200, arm_gen=4)):
        assert bs.stop_action(arm_sent=True, zero_seq=200, zero_link=LINK, p=p) == "wait"


def test_an_old_prep_pair_kept_by_the_host_is_no_evidence_for_the_disarm():
    """Past PREP last_rx 100, first stop zero 40100 (more than half the seq ring apart): only a pair made after the
    zero counts (Codex B1 design round 2 P1-3)."""
    old = pair(drive=D.PREP, last_rx=100)
    assert bs.stop_action(arm_sent=True, zero_seq=40100, zero_link=LINK, p=old, after_zero=False) == "wait"


def test_stopped_and_latched_modes_close():
    assert bs.stop_action(arm_sent=True, zero_seq=200, zero_link=LINK, p=pair(Mode.DISARMED, D.ENDED)) == "closed"
    for m in (Mode.FAULT, Mode.ESTOP):
        assert bs.stop_action(arm_sent=True, zero_seq=None, zero_link=None, p=pair(m, D.ENDED)) == "closed_latched"


def test_stopping_before_the_zero_is_reflected_waits():
    assert bs.stop_action(arm_sent=True, zero_seq=200, zero_link=LINK, p=pair(Mode.STOPPING, D.ENDED, last_rx=150)) \
        == "wait"


# --- the reopen rule (plan 3, 7th/8th round) ----------------------------------------------------------------------------
def test_the_first_pair_of_a_new_host_is_after_the_boundary():
    assert bs.after_boundary(None, pair())


def test_a_new_boot_or_connection_generation_is_after_the_boundary_and_the_same_is_not():
    assert bs.after_boundary((7, 1), pair(boot=8))
    assert bs.after_boundary((7, 1), pair(conn=2))
    assert bs.after_boundary((7, 1), pair(boot=8, conn=1))  # a new boot with the same generation value
    assert not bs.after_boundary((7, 1), pair())


@pytest.mark.parametrize("mode,action", [(Mode.BOOT, "boot_wait"), (Mode.DISARMED, "go"),
                                         (Mode.ARMED, "wait_brake"), (Mode.STOPPING, "wait_brake"),
                                         (Mode.FAULT, "latched"), (Mode.ESTOP, "latched")])
def test_after_a_reopen_the_mode_decides_and_no_disarm_is_sent(mode, action):
    assert bs.reopen_action(pair(mode)) == action


# --- the clear-faults binding (plan 3, 9th/10th round P2) ---------------------------------------------------------------
def test_a_clear_is_bound_to_the_link_boot_and_fault_bits_it_was_asked_on():
    asked = bs.ClearBinding(conn=1, conn_gen=4, boot_id=7, fault_flags=FaultFlag.OVERCURRENT_M)
    assert asked.holds(1, 4, 7, FaultFlag.OVERCURRENT_M)
    assert not asked.holds(2, 4, 7, FaultFlag.OVERCURRENT_M)  # the host reopened
    assert not asked.holds(1, 5, 7, FaultFlag.OVERCURRENT_M)  # the board's connection generation moved (same fd)
    assert not asked.holds(1, 4, 8, FaultFlag.OVERCURRENT_M)  # rebooted
    assert not asked.holds(1, 4, 7, FaultFlag.OVERCURRENT_M | FaultFlag.DRIVER_FAULT_M)  # a new bit


# --- the first verdict (plan 3: run and cleanup) --------------------------------------------------------------------------
def ok_kwargs(**kw):
    base = dict(closed="ended", stop_reason="end_zero", aborted=None, cleanup_confirmed=True,
                last=pair(Mode.DISARMED, D.ENDED), record_errors=0, unexpected_faults=())
    base.update(kw)
    return base


def test_a_clean_run_passes_the_first_verdict():
    v = bs.run_verdict(**ok_kwargs())
    assert v["ok"] and v["why"] == []


@pytest.mark.parametrize("kw,why", [
    (dict(stop_reason="command_timeout"), "stop_reason:command_timeout"),
    (dict(aborted="cancelled:stop"), "aborted:cancelled:stop"),
    (dict(cleanup_confirmed=False), "cleanup_unconfirmed"),
    (dict(last=pair(Mode.FAULT, D.ENDED)), "last_mode:FAULT"),
    (dict(last=pair(Mode.DISARMED, D.ENDED, duty=(0, 0, 5, 0))), "duty_not_zero"),
    (dict(record_errors=1), "record_errors:1"),
    (dict(unexpected_faults=("OVERCURRENT_M",)), "faults:OVERCURRENT_M"),
    (dict(closed="no_drive", stop_reason=None), "no_drive"),
])
def test_each_failure_names_itself(kw, why):
    v = bs.run_verdict(**ok_kwargs(**kw))
    assert not v["ok"] and why in v["why"]


# --- the window stop with the two bypass bits (plan 3, the table) ---------------------------------------------------------
@pytest.mark.parametrize("value,kth_invalid_bypass,range_off,stop", [
    (0.5, False, False, None), (1.5, False, False, "window"),  # range on: outside stops
    (math.nan, False, False, "window"),  # range on, no kth bypass: an invalid sample is outside (the board faults too)
    (1.5, False, True, None),  # range off: no window check
    (math.nan, True, False, None),  # kth bypass: an invalid sample is skipped, not a window stop
    (1.5, True, False, "window"),  # kth bypass, a valid sample outside still stops
    (1.5, True, True, None),
])
def test_window_stop_by_the_two_bypass_bits(value, kth_invalid_bypass, range_off, stop):
    assert bs.window_stop(value, (0.0, 1.0), kth_invalid_bypass=kth_invalid_bypass, range_off=range_off) == stop


# --- B2: the board's own end and a STOPPING latest STATE (plan 4.4 decision 3) ---------------------------------------
def test_the_boards_own_end_or_a_stopping_state_sends_nothing_until_an_end_pair():
    link = (7, 1, 3)
    ready = pair(drive=D.PREP, last_rx=101)  # the zero 101 taken while preparing: normally a DISARM
    assert bs.stop_action(arm_sent=True, zero_seq=101, zero_link=link, p=ready) == "send_disarm"
    assert bs.stop_action(arm_sent=True, zero_seq=101, zero_link=link, p=ready, latest_stopping=True) == "wait"
    assert bs.stop_action(arm_sent=True, zero_seq=None, zero_link=None, p=None, board_end=True) == "wait"
    lagging = pair(drive=D.DRIVING, last_rx=99)  # the zero not taken yet: normally a resend
    assert bs.stop_action(arm_sent=True, zero_seq=101, zero_link=link, p=lagging, board_end=True) == "wait"
    # an end pair still closes first
    assert bs.stop_action(arm_sent=True, zero_seq=101, zero_link=link, p=pair(Mode.DISARMED),
                          board_end=True) == "closed"
    assert bs.stop_action(arm_sent=True, zero_seq=101, zero_link=link, p=pair(Mode.FAULT),
                          latest_stopping=True) == "closed_latched"
