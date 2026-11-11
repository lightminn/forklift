"""P0-7 mcu_link host core (docs/plans/2026-11-11-sim-to-real-bringup.md 2.2, P0-7): operator authorization,
freshness, deadline-preserving DRIVE, frame outbox. No I/O here."""

import math

import pytest

from forklift_core.mcu_link.messages import (
    Arm,
    ClearFaults,
    Disarm,
    Drive,
    FaultFlag,
    LinkDecoder,
    McuState,
    Mode,
    RejectReason,
    StateFlag,
    encode,
)
from forklift_core.plant.mcu_host import Blocked, HostConfig, McuLinkHost
from forklift_core.plant.real_vehicle import RearDrive

MS = 1_000_000
BOOT = 0xB007
CFG = HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100, outbox_max=8)


def state(t_us, *, boot=BOOT, mode=Mode.DISARMED, gen=0, flags=StateFlag.DRIVERS_READY, steering=0.01,
          faults=None, reject=RejectReason.NONE, last_rx=0, counts=(0, 0)):
    faults = FaultFlag(0) if faults is None else faults
    return encode(McuState(t_us, boot, gen, last_rx, mode, reject, faults, counts[0], counts[1], 0.0, 0.0,
                           steering, 0.0, (0, 0, 0, 0), (0, 0, 0, 0), (0xC0,) * 4, 0, 0, flags), 1)


def drive_cmd(rate=1.0):
    return RearDrive(0.1, 0.0, rate, rate, 0.0, False)


class Rig:
    """Host + a log of everything, + helpers to drain the outbox as decoded messages."""

    def __init__(self, cfg=CFG):
        self.log = []
        self.host = McuLinkHost(cfg, log=lambda *a: self.log.append(a))
        self.t_us = 1_000_000  # MCU clock
        self.now = 10_000 * MS  # host monotonic ns

    def feed(self, dt_ms=10, **kw):
        self.t_us += dt_ms * 1000
        self.now += dt_ms * MS
        return self.host.on_bytes(state(self.t_us, **kw), self.now)

    def sent(self):
        dec, out = LinkDecoder(), []
        while (f := self.host.next_frame(self.now)) is not None:
            self.host.frame_written(f, self.now)
            out += [r.message for r in dec.feed(f.data, self.now)]
        return out

    def ready(self):
        """Operator arm -> ARMED -> zero DRIVE -> ZERO_SEEN."""
        self.feed()
        assert self.host.request_arm(self.now) is None
        assert [type(m) for m in self.sent()] == [Arm]
        self.feed(mode=Mode.ARMED, gen=1)
        zero = self.sent()
        assert len(zero) == 1 and isinstance(zero[0], Drive)
        self.feed(mode=Mode.ARMED, gen=1, flags=StateFlag.DRIVERS_READY | StateFlag.ZERO_SEEN)
        assert self.host.ready


def kinds(events):
    return [e.kind for e in events]


def test_first_state_and_freshness():
    r = Rig()
    assert not r.host.health(r.now).fresh
    assert "first_state" in kinds(r.feed())
    h = r.host.health(r.now)
    assert h.fresh and h.mode == Mode.DISARMED and h.boot_id == BOOT
    assert not r.host.health(r.now + 51 * MS).fresh


def test_operator_arm_sends_arm_then_one_zero_drive_holding_steering_and_lift():
    r = Rig()
    r.feed(steering=0.042)
    assert r.host.request_arm(r.now) is None
    (arm,) = r.sent()
    assert arm == Arm(BOOT, r.t_us)
    ev = r.feed(mode=Mode.ARMED, gen=1, steering=0.042)
    assert "zero_sent" in kinds(ev)
    (zero,) = r.sent()
    assert (zero.left_wheel_rad_s, zero.right_wheel_rad_s) == (0.0, 0.0)
    assert zero.steering_rad == pytest.approx(0.042) and zero.lift_hold and zero.arm_gen == 1
    assert not r.host.ready  # sending is not readiness
    assert r.host.drive(drive_cmd(), 0.0, True, r.now + 100 * MS, r.now) == Blocked("zero_pending")
    ev = r.feed(mode=Mode.ARMED, gen=1, flags=StateFlag.DRIVERS_READY | StateFlag.ZERO_SEEN)
    assert "armed_ready" in kinds(ev) and r.host.ready


def test_drive_validity_counts_from_the_echoed_state_receipt_and_never_passes_the_deadline():
    r = Rig()
    r.ready()
    rx = r.now
    echo = r.t_us
    expires = rx + 120 * MS
    r.now += 30 * MS  # sent 30 ms after the STATE arrived: still counted from its receipt
    assert r.host.drive(drive_cmd(), 0.0, True, expires, r.now) is None
    (d,) = r.sent()
    assert d.echo_mcu_time_us == echo and d.valid_after_echo_ms == 120
    assert rx + d.valid_after_echo_ms * MS <= expires
    # The cap bounds it even when the decision lives longer.
    assert r.host.drive(drive_cmd(), 0.0, True, rx + 900 * MS, r.now) is None
    assert r.sent()[0].valid_after_echo_ms == 150
    assert r.host.drive(drive_cmd(), 0.0, True, rx + MS // 2, r.now) == Blocked("expired")


@pytest.mark.parametrize("setup,reason", [
    (lambda r: None, "no_state"),
    (lambda r: r.feed(), "not_authorized"),
])
def test_drive_is_blocked_without_authorization(setup, reason):
    r = Rig()
    setup(r)
    assert r.host.drive(drive_cmd(), 0.0, True, r.now + 100 * MS, r.now) == Blocked(reason)
    assert r.sent() == []


def test_stale_link_revokes_authorization_and_recovery_does_not_restore_it():
    """The MCU may still be ARMED when STATEs come back (Codex P0-7 P1): no DRIVE without a new operator arm."""
    r = Rig()
    r.ready()
    r.now += 60 * MS
    ev = r.host.tick(r.now)
    assert kinds(ev).count("stale") == 1 and "authorization_revoked" in kinds(ev)
    assert r.host.tick(r.now + MS) == []  # once
    ev = r.feed(mode=Mode.ARMED, gen=1, flags=StateFlag.DRIVERS_READY | StateFlag.ZERO_SEEN)
    assert "recovered" in kinds(ev) and not r.host.ready
    assert r.host.drive(drive_cmd(), 0.0, True, r.now + 100 * MS, r.now) == Blocked("not_authorized")
    assert r.host.request_arm(r.now) == Blocked("disarm_first")  # MCU still ARMED: DISARM, then a new ARM
    assert r.sent() == []


def test_a_restarted_host_seeing_an_armed_mcu_is_not_authorized():
    r = Rig()
    r.feed(mode=Mode.ARMED, gen=7, flags=StateFlag.DRIVERS_READY | StateFlag.ZERO_SEEN)
    assert not r.host.ready
    assert r.host.drive(drive_cmd(), 0.0, True, r.now + 100 * MS, r.now) == Blocked("not_authorized")


@pytest.mark.parametrize("event", ["boot", "gen", "mode", "disconnect"])
def test_authorization_boundaries(event):
    r = Rig()
    r.ready()
    if event == "boot":
        ev = r.feed(boot=BOOT + 1)
        assert "boot_changed" in kinds(ev)
    elif event == "gen":
        ev = r.feed(mode=Mode.ARMED, gen=2, flags=StateFlag.DRIVERS_READY | StateFlag.ZERO_SEEN)
    elif event == "mode":
        ev = r.feed(mode=Mode.STOPPING, gen=1)
    else:
        ev = r.host.disconnected(r.now)
    assert "authorization_revoked" in kinds(ev) and not r.host.ready


def test_no_event_sequence_makes_the_host_send_arm_or_clear_faults():
    r = Rig()
    r.ready()
    for kw in (dict(mode=Mode.STOPPING, gen=1), dict(mode=Mode.DISARMED, gen=1),
               dict(mode=Mode.FAULT, gen=1, faults=FaultFlag.DRIVER_FAULT_L),
               dict(mode=Mode.ESTOP, gen=1, faults=FaultFlag.ESTOP_INPUT),
               dict(boot=BOOT + 1, mode=Mode.BOOT), dict(boot=BOOT + 1, mode=Mode.DISARMED)):
        r.feed(**kw)
        r.host.tick(r.now)
    r.now += 500 * MS
    r.host.tick(r.now)
    r.feed(boot=BOOT + 1)
    assert not any(isinstance(m, (Arm, ClearFaults)) for m in r.sent())


def test_arm_request_is_cancelled_by_disarm_and_never_answers_a_late_armed():
    r = Rig()
    r.feed()
    r.host.request_arm(r.now)
    r.host.request_disarm(r.now)
    assert [type(m) for m in r.sent()] == [Disarm]  # the queued ARM was dropped
    r.feed(mode=Mode.ARMED, gen=1)
    assert r.sent() == [] and not r.host.ready


def test_arm_request_times_out():
    r = Rig()
    r.feed()
    r.host.request_arm(r.now)
    r.sent()
    r.now += 310 * MS
    ev = r.host.tick(r.now)
    assert "arm_failed" in kinds(ev)
    r.feed(mode=Mode.ARMED, gen=1)  # too late: ignored
    assert r.sent() == [] and not r.host.ready


def test_arm_rejected_by_the_mcu_fails():
    r = Rig()
    r.feed()
    r.host.request_arm(r.now)
    r.sent()
    # last_rx_seq counts accepted messages only: a refused ARM shows as a new reject reason.
    ev = r.feed(reject=RejectReason.SENSOR_INVALID)
    assert "arm_failed" in kinds(ev) and not r.host.ready


def test_arm_needs_a_finite_steering_reading_for_the_zero_drive():
    r = Rig()
    r.feed()
    r.host.request_arm(r.now)
    r.sent()
    ev = r.feed(mode=Mode.ARMED, gen=1, steering=math.nan)
    # No zero DRIVE without a steering to hold; the MCU is left disarmed rather than armed and unattended.
    assert "arm_failed" in kinds(ev) and r.sent() == [Disarm()]


@pytest.mark.parametrize("mode,reason", [(Mode.ARMED, "disarm_first"), (Mode.FAULT, "latched"),
                                         (Mode.BOOT, "not_disarmed")])
def test_arm_request_needs_a_fresh_disarmed_mcu(mode, reason):
    r = Rig()
    r.feed(mode=mode, gen=3)
    assert r.host.request_arm(r.now) == Blocked(reason)
    r2 = Rig()
    assert r2.host.request_arm(r2.now) == Blocked("no_state")


def test_clear_faults_only_on_request_and_only_when_latched():
    r = Rig()
    r.feed()
    assert r.host.request_clear_faults(r.now) == Blocked("not_latched")
    r.feed(mode=Mode.ESTOP, faults=FaultFlag.ESTOP_INPUT)
    assert r.host.request_clear_faults(r.now) is None
    assert r.sent() == [ClearFaults(BOOT)]


def test_disarm_needs_no_state_and_drops_queued_drive_and_arm():
    r = Rig()
    r.host.request_disarm(r.now)
    assert r.sent() == [Disarm()]
    r.ready()
    r.host.drive(drive_cmd(), 0.0, True, r.now + 100 * MS, r.now)
    r.host.request_disarm(r.now)
    assert r.sent() == [Disarm()] and not r.host.ready


def test_queued_drive_past_its_deadline_is_dropped_not_written():
    r = Rig()
    r.ready()
    r.host.drive(drive_cmd(), 0.0, True, r.now + 20 * MS, r.now)
    r.now += 25 * MS
    assert r.sent() == []
    assert any(entry[0] == "dropped" for entry in r.log)


def test_disconnect_drops_everything_queued():
    r = Rig()
    r.ready()
    r.host.drive(drive_cmd(), 0.0, True, r.now + 100 * MS, r.now)
    r.host.disconnected(r.now)
    assert r.sent() == []
    assert not r.host.health(r.now).fresh


def test_outbox_is_bounded_but_disarm_always_goes():
    r = Rig(HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100, outbox_max=2))
    r.ready()
    assert r.host.drive(drive_cmd(), 0.0, True, r.now + 100 * MS, r.now) is None
    assert r.host.drive(drive_cmd(), 0.0, True, r.now + 100 * MS, r.now) is None
    assert r.host.drive(drive_cmd(), 0.0, True, r.now + 100 * MS, r.now) == Blocked("outbox_full")
    r.host.request_disarm(r.now)
    assert r.sent() == [Disarm()]


def test_out_of_order_and_invalid_states_do_not_refresh_the_link():
    r = Rig()
    r.feed()
    t = r.t_us
    ev = r.host.on_bytes(state(t), r.now + 40 * MS)  # same MCU time again
    assert "state_out_of_order" in kinds(ev)
    assert not r.host.health(r.now + 55 * MS).fresh  # freshness still from the first one
    ev = r.host.on_bytes(state(t + 10_000, boot=0), r.now + 41 * MS)
    assert "state_invalid" in kinds(ev)


def test_split_and_joined_bytes_decode():
    r = Rig()
    a, b = state(1_000_000), state(1_010_000)
    joined = a + b
    ev = r.host.on_bytes(joined[:5], r.now) + r.host.on_bytes(joined[5:], r.now + MS)
    assert kinds(ev).count("first_state") == 1
    assert r.host.health(r.now + MS).mcu_time_us == 1_010_000


def test_mode_fault_and_reject_changes_are_reported_once():
    r = Rig()
    r.feed()
    ev = r.feed(mode=Mode.FAULT, faults=FaultFlag.OVERCURRENT_L, reject=RejectReason.NOT_ARMED)
    assert {"mode_changed", "faults_changed", "reject"} <= set(kinds(ev))
    assert r.feed(mode=Mode.FAULT, faults=FaultFlag.OVERCURRENT_L, reject=RejectReason.NOT_ARMED) == []


def test_seq_wraps_and_every_frame_is_logged_by_stage():
    r = Rig()
    r.host._seq = 0xFFFE  # noqa: SLF001 -- start near the wrap
    dec, seqs = LinkDecoder(), []
    for _ in range(3):
        r.host.request_disarm(r.now)
        f = r.host.next_frame(r.now)
        r.host.frame_written(f, r.now)
        seqs += [x.seq for x in dec.feed(f.data, r.now)]
    assert seqs == [0xFFFE, 0xFFFF, 0]
    stages = [e[0] for e in r.log]
    assert {"queued", "written"} <= set(stages)
    r.host.on_bytes(state(5_000_000), r.now)
    assert "received" in [e[0] for e in r.log]


def test_a_state_arriving_after_a_stale_gap_still_revokes():
    """on_bytes before tick (the adapter's order): the gap is seen on receipt (Codex P0-7 round 2 P1)."""
    r = Rig()
    r.ready()
    ev = r.feed(dt_ms=60, mode=Mode.ARMED, gen=1, flags=StateFlag.DRIVERS_READY | StateFlag.ZERO_SEEN)
    assert "stale" in kinds(ev) and "authorization_revoked" in kinds(ev) and "recovered" in kinds(ev)
    assert r.host.tick(r.now) == [] and not r.host.ready
    assert r.host.drive(drive_cmd(), 0.0, True, r.now + 100 * MS, r.now) == Blocked("not_authorized")


@pytest.mark.parametrize("late_ms,ok", [(290, True), (300, True), (301, False)])
def test_arm_deadline_is_checked_before_zero_seen_makes_it_ready(late_ms, ok):
    r = Rig(HostConfig(state_max_age_s=1.0, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100, outbox_max=8))
    r.feed()
    t0 = r.now
    r.host.request_arm(r.now)
    r.sent()
    r.feed(mode=Mode.ARMED, gen=1)
    r.sent()
    r.now = t0 + late_ms * MS - 10 * MS  # feed adds 10 ms
    ev = r.feed(mode=Mode.ARMED, gen=1, flags=StateFlag.DRIVERS_READY | StateFlag.ZERO_SEEN)
    assert r.host.ready is ok
    assert ("armed_ready" in kinds(ev)) is ok


def test_disarm_requests_coalesce_within_the_outbox_bound():
    r = Rig(HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100, outbox_max=2))
    for _ in range(5):
        r.host.request_disarm(r.now)
    assert r.sent() == [Disarm()]


def test_a_frame_handed_back_unsent_is_still_subject_to_cancellation():
    """The adapter got EAGAIN before the first byte: the frame goes back and a DISARM still drops it."""
    r = Rig()
    r.ready()
    r.host.drive(drive_cmd(), 0.0, True, r.now + 100 * MS, r.now)
    f = r.host.next_frame(r.now)
    assert f.kind == "DRIVE"
    r.host.unsent(f, r.now)
    r.host.request_disarm(r.now)
    assert r.sent() == [Disarm()]
