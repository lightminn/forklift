"""P0-4 4a' port contract against the P0-7 host (docs/design/2026-11-11-p04a-kinematics-and-command.md D4)."""

import math

import numpy as np
import pytest
from test_mcu_host import BOOT, MS, state  # noqa: E402
from test_real_vehicle import CHASSIS, mapping  # noqa: E402

from forklift_core.mcu_link.messages import Drive, LinkDecoder, Mode, StateFlag
from forklift_core.plant.kinematics import RealKinematics
from forklift_core.plant.mcu_host import HostConfig, McuLinkHost
from forklift_core.plant.port import PortResult, RealPlantPort
from forklift_core.plant.real_vehicle import real_vehicle_from_mapping

CFG = HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100, outbox_max=4)
KIN = RealKinematics(real_vehicle_from_mapping(mapping(), CHASSIS))
READY = StateFlag.DRIVERS_READY | StateFlag.ZERO_SEEN


class Bench:
    def __init__(self, cfg=CFG):
        self.host = McuLinkHost(cfg)
        self.port = RealPlantPort(self.host, KIN)
        self.t_us, self.now = 1_000_000, 10_000 * MS
        self.gen = 0

    def feed(self, dt_ms=10, **kw):
        self.t_us += dt_ms * 1000
        self.now += dt_ms * MS
        self.host.on_bytes(state(self.t_us, **kw), self.now)

    def drain(self, n=None):
        dec, out = LinkDecoder(), []
        while (n is None or len(out) < n) and (f := self.host.next_frame(self.now)) is not None:
            self.host.frame_written(f, self.now)
            out += [r.message for r in dec.feed(f.data, self.now)]
        return out

    def arm(self):
        self.feed()
        self.host.request_arm(self.now)
        self.drain()
        self.gen += 1
        self.feed(mode=Mode.ARMED, gen=self.gen)
        self.drain()
        self.feed(mode=Mode.ARMED, gen=self.gen, flags=READY)
        assert self.host.ready

    def cmd(self, speed=0.3, kappa=0.1, *, ttl_ms=100, lift_m=0.05, lift_mode="target"):
        d = KIN.command(speed, kappa)
        steer = np.asarray(d.steering_rad) if hasattr(d, "steering_rad") else np.array([0.0])
        return self.port.make_command(self.now, self.now + ttl_ms * MS, d, steer, lift_m, lift_mode)


def test_a_queued_command_is_the_drive_frame_mcu_link_expects():
    b = Bench()
    b.arm()
    c = b.cmd()
    assert b.port.command(c, b.now) == PortResult("queued")
    (msg,) = b.drain()
    assert isinstance(msg, Drive)
    assert (msg.left_wheel_rad_s, msg.right_wheel_rad_s) == pytest.approx(c.drive.wheel_rates_rad_s, rel=1e-6)
    assert msg.steering_rad == pytest.approx(c.drive.steering_rad[0], rel=1e-6)
    assert msg.lift_m == pytest.approx(0.05) and not msg.lift_hold and msg.valid_after_echo_ms == 100


def test_lift_keep_sets_the_firmware_hold_flag():
    b = Bench()
    b.arm()
    b.port.command(b.cmd(lift_mode="keep"), b.now)
    assert b.drain()[0].lift_hold


def test_refusal_and_blocked_are_told_apart():
    b = Bench()
    b.arm()
    assert b.port.command(b.cmd(kappa=0.5), b.now).kind == "refused"
    b2 = Bench()
    b2.feed()  # not armed
    assert b2.port.command(b2.cmd(), b2.now) == PortResult("blocked", "not_authorized")


def test_a_command_made_before_a_rearm_is_blocked_even_with_the_same_control_generation():
    """Same lease, mission and control_gen; a new MCU authorization (Codex 4a' document P1-1)."""
    b = Bench()
    b.arm()
    old = b.cmd(ttl_ms=5000)
    b.host.request_disarm(b.now)
    b.drain()
    b.feed(mode=Mode.DISARMED, gen=b.gen)
    b.arm()
    assert b.port.command(old, b.now) == PortResult("blocked", "stale_authorization")
    assert b.port.command(b.cmd(), b.now) == PortResult("queued")


def test_a_new_control_generation_drops_queued_drives_and_blocks_old_commands():
    b = Bench()
    b.arm()
    old = b.cmd()
    b.port.command(old, b.now)
    b.port.new_control_generation()
    assert b.drain() == []
    assert b.port.command(old, b.now) == PortResult("blocked", "stale_generation")


def test_blocked_is_not_zero_the_older_command_may_still_act():
    """A full queue of non-zero DRIVEs, then a zero that cannot be queued (Codex 4a' P1-1)."""
    b = Bench(HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100, outbox_max=2))
    b.arm()
    assert b.port.command(b.cmd(), b.now).kind == "queued"
    assert b.port.command(b.cmd(), b.now).kind == "queued"
    assert b.port.zero_wheels(b.now, b.now + 100 * MS) == PortResult("blocked", "outbox_full")
    assert b.port.older_nonzero_may_act(b.now)


def test_release_condition_a_needs_queue_inflight_and_written_commands_settled():
    b = Bench()
    b.arm()
    b.port.command(b.cmd(ttl_ms=100), b.now)
    assert b.port.older_nonzero_may_act(b.now)  # queued
    f = b.host.next_frame(b.now)
    assert b.port.older_nonzero_may_act(b.now)  # being written
    b.host.frame_written(f, b.now)
    assert b.port.older_nonzero_may_act(b.now)  # written, not expired, MCU still fresh
    assert b.port.outstanding_until(b.now) == b.now + 100 * MS
    b.now += 101 * MS
    assert not b.port.older_nonzero_may_act(b.now)  # expired


def test_release_condition_a_also_ends_when_the_mcu_reports_the_command_ended():
    """A STATE received after the write, that accepted the frame and holds no fresh command; or one that is
    no longer ARMED (a later ARM has a new arm_gen the frame cannot match)."""
    b = Bench()
    b.arm()
    b.port.command(b.cmd(ttl_ms=1000), b.now)
    b.drain()
    seq = (b.host._seq - 1) & 0xFFFF  # noqa: SLF001 -- the frame just written
    b.feed(mode=Mode.ARMED, gen=b.gen, flags=READY | StateFlag.COMMAND_FRESH, last_rx=seq)
    assert b.port.older_nonzero_may_act(b.now)  # accepted and still fresh
    b.feed(mode=Mode.ARMED, gen=b.gen, flags=READY, last_rx=seq)  # accepted, deadline passed
    assert not b.port.older_nonzero_may_act(b.now)
    b2 = Bench()
    b2.arm()
    b2.port.command(b2.cmd(ttl_ms=1000), b2.now)
    b2.drain()
    b2.feed(mode=Mode.STOPPING, gen=b2.gen)
    assert not b2.port.older_nonzero_may_act(b2.now)


def test_a_state_from_before_the_write_proves_nothing():
    b = Bench()
    b.arm()  # its last STATE has no fresh command
    b.port.command(b.cmd(ttl_ms=1000), b.now)
    b.drain()
    assert b.port.older_nonzero_may_act(b.now)


@pytest.mark.parametrize("written", range(0, 8))
def test_a_frame_cut_at_any_point_by_a_drop_is_finished_or_dropped_never_lost(written):
    """0 bytes out: back to the queue and dropped. 1+ bytes out: finished and outstanding (design D4)."""
    b = Bench()
    b.arm()
    b.port.command(b.cmd(ttl_ms=200), b.now)
    f = b.host.next_frame(b.now)
    if written == 0:
        b.host.unsent(f, b.now)
        b.port.new_control_generation()
        assert b.host.next_frame(b.now) is None
        assert not b.port.older_nonzero_may_act(b.now)
    else:
        b.port.new_control_generation()  # drops the queue; the frame in flight is finished by the adapter
        b.host.frame_written(f, b.now)
        assert b.port.older_nonzero_may_act(b.now) and b.port.outstanding_until(b.now) == b.now + 200 * MS


def test_zero_and_lift_compose_from_the_latest_intent():
    b = Bench()
    b.arm()
    c = b.cmd(lift_m=0.07)
    b.port.command(c, b.now)
    b.drain()
    assert b.port.zero_wheels(b.now, b.now + 50 * MS) == PortResult("queued")
    (z,) = b.drain()
    assert (z.left_wheel_rad_s, z.right_wheel_rad_s) == (0.0, 0.0)
    assert z.steering_rad == pytest.approx(c.drive.steering_rad[0], rel=1e-6) and z.lift_m == pytest.approx(0.07)
    assert b.port.hold_lift(0.12, b.now, b.now + 50 * MS) == PortResult("queued")
    (h,) = b.drain()
    assert h.lift_m == pytest.approx(0.12) and not h.lift_hold


def test_zero_without_an_intent_or_after_a_rearm_is_blocked():
    b = Bench()
    b.arm()
    assert b.port.zero_wheels(b.now, b.now + 50 * MS) == PortResult("blocked", "no_intent")
    b.port.command(b.cmd(), b.now)
    b.host.request_disarm(b.now)
    b.drain()
    b.feed(mode=Mode.DISARMED, gen=b.gen)
    b.arm()
    assert b.port.zero_wheels(b.now, b.now + 50 * MS) == PortResult("blocked", "no_intent")


def test_expiry_is_never_extended_by_a_composed_frame():
    b = Bench()
    b.arm()
    c = b.cmd(ttl_ms=40)
    b.port.command(c, b.now)
    b.drain()
    assert b.port.zero_wheels(b.now, b.now + 500 * MS) == PortResult("queued")
    (z,) = b.drain()
    assert z.valid_after_echo_ms <= 40 and z.boot_id == BOOT  # the intent's 40 ms, not the 500 asked


def test_an_earlier_command_still_fresh_keeps_the_release_closed():
    """A (100 ms) accepted, B (10 ms) written but not yet accepted: after B expires, A still acts (Codex 4a' impl P1-1)."""
    b = Bench()
    b.arm()
    b.port.command(b.cmd(ttl_ms=100), b.now)
    b.drain()
    seq_a = (b.host._seq - 1) & 0xFFFF  # noqa: SLF001
    b.port.command(b.cmd(ttl_ms=10), b.now)
    b.drain()
    b.feed(dt_ms=11, mode=Mode.ARMED, gen=b.gen, flags=READY | StateFlag.COMMAND_FRESH, last_rx=seq_a)
    assert b.port.older_nonzero_may_act(b.now)
    assert b.port.outstanding_until(b.now) == b.now - 11 * MS + 100 * MS


def test_a_later_accepted_drive_settles_the_earlier_ones():
    b = Bench()
    b.arm()
    b.port.command(b.cmd(ttl_ms=100), b.now)
    b.drain()
    b.port.zero_wheels(b.now, b.now + 100 * MS)
    b.drain()
    seq_zero = (b.host._seq - 1) & 0xFFFF  # noqa: SLF001
    b.feed(mode=Mode.ARMED, gen=b.gen, flags=READY | StateFlag.COMMAND_FRESH, last_rx=seq_zero)
    assert not b.port.older_nonzero_may_act(b.now)


def test_an_unknown_steering_stops_on_the_last_intent_or_disarms():
    from forklift_core.mcu_link.messages import Disarm
    from forklift_core.plant.kinematics import RealDrive

    b = Bench()
    b.arm()
    nan_drive = RealDrive(0.0, 0.0, (float("nan"),), (0.0, 0.0), False)
    cmd = b.port.make_command(b.now, b.now + 50 * MS, nan_drive, np.array([0.0]), 0.0, "keep")
    assert b.port.command(cmd, b.now) == PortResult("blocked", "steering_unknown_disarmed")
    assert b.drain() == [Disarm()]
    b2 = Bench()
    b2.arm()
    b2.port.command(b2.cmd(), b2.now)
    b2.drain()
    cmd2 = b2.port.make_command(b2.now, b2.now + 50 * MS, nan_drive, np.array([0.0]), 0.0, "keep")
    assert b2.port.command(cmd2, b2.now) == PortResult("queued")
    (z,) = b2.drain()
    assert (z.left_wheel_rad_s, z.right_wheel_rad_s) == (0.0, 0.0) and math.isfinite(z.steering_rad)


def test_the_real_sample_reads_the_latest_state_without_consuming_it():
    """Absolute counts, MCU time and host receipt kept; repeated reads see the same values (Codex 4b P2-2)."""
    b = Bench()
    assert b.port.sample() is None
    b.arm()
    s1, s2 = b.port.sample(), b.port.sample()
    assert s1 == s2
    assert s1.steering == (pytest.approx(0.01),) and s1.valid["steering"] and s1.wheel_counts == (0, 0)
    assert s1.boot_id == BOOT and s1.received_host_ns == b.now and s1.mcu_time_us == b.t_us
    assert s1.raw_joints is None  # an Isaac-only field


@pytest.mark.parametrize("steer,lift,ok_s,ok_l", [(0.5, 0.0, False, True), (float("nan"), float("nan"), False, False)])
def test_the_real_sample_marks_invalid_readings_without_zeroing_them(steer, lift, ok_s, ok_l):
    """An out-of-map or NaN steering and a NaN lift are flagged, never replaced by 0 (Codex 4b P2-3)."""
    from forklift_core.mcu_link.messages import FaultFlag, McuState, RejectReason, encode

    b = Bench()
    b.host.on_bytes(encode(McuState(1000, BOOT, 0, 0, Mode.DISARMED, RejectReason.NONE, FaultFlag(0), 5, 6, 0.0, 0.0,
                                    steer, lift, (0,) * 4, (0,) * 4, (0xC0,) * 4, 0, 0, StateFlag(0)), 1), b.now)
    s = b.port.sample()
    assert s.valid["steering"] is ok_s and s.valid["lift"] is ok_l
    assert s.lift_m is None if not ok_l else s.lift_m == 0.0
    assert (math.isnan(s.steering[0]) if math.isnan(steer) else s.steering == (steer,))


def test_the_real_health_tells_authorized_from_ready():
    b = Bench()
    h = b.port.health(b.now)
    assert not h.fresh and "no_state" in h.reasons
    b.feed()
    b.host.request_arm(b.now)
    b.drain()
    b.gen += 1
    b.feed(mode=Mode.ARMED, gen=b.gen)
    h = b.port.health(b.now)
    assert h.authorized and not h.ready and "zero_pending" in h.reasons
    b.drain()
    b.feed(mode=Mode.ARMED, gen=b.gen, flags=READY)
    h = b.port.health(b.now)
    assert h.ready and h.reasons == () and h.mode == Mode.ARMED
    assert "stale" in b.port.health(b.now + 60 * MS).reasons
