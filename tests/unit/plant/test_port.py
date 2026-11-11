"""P0-4 4a' port contract against the P0-7 host (docs/design/2026-11-11-p04a-kinematics-and-command.md D4)."""

import numpy as np
import pytest

from forklift_core.mcu_link.messages import Drive, LinkDecoder, Mode, StateFlag
from forklift_core.plant.kinematics import RealKinematics
from forklift_core.plant.mcu_host import HostConfig, McuLinkHost
from forklift_core.plant.port import PortResult, RealPlantPort
from forklift_core.plant.real_vehicle import real_vehicle_from_mapping

from test_mcu_host import BOOT, MS, state  # noqa: E402
from test_real_vehicle import CHASSIS, mapping  # noqa: E402

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
    assert b.port.outstanding_until_ns == b.now + 100 * MS
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
        assert b.port.older_nonzero_may_act(b.now) and b.port.outstanding_until_ns == b.now + 200 * MS


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
    assert z.valid_after_echo_ms <= 150 and z.boot_id == BOOT
