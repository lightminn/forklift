"""The commissioning mode of the mcu_link host core (docs/design/2026-11-12-bringup-commission-tool.md section 5):
COMMISSION only to a commissioning build, a zero COMMISSION after ARM, the outbox rules shared with DRIVE, and the
executor paths unchanged."""

import pytest

from forklift_core.mcu_link.build_policy import Client
from forklift_core.mcu_link.messages import (
    Arm, Commission, Disarm, Drive, FaultFlag, LinkDecoder, McuState, Mode, RejectReason, StateFlag, encode,
)
from forklift_core.plant.mcu_host import Blocked, HostConfig, McuLinkHost
from forklift_core.plant.real_vehicle import RearDrive

MS = 1_000_000
BOOT = 0xC0FF
CFG = HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100, outbox_max=8)
CB = StateFlag.COMMISSION_BUILD | StateFlag.DRIVERS_READY


def state(t_us, *, mode=Mode.DISARMED, gen=0, flags=CB, last_rx=0, steering=float("nan")):
    return encode(McuState(t_us, BOOT, gen, last_rx, mode, RejectReason.NONE, FaultFlag(0), 0, 0, 0.0, 0.0,
                           steering, 0.0, (0, 0, 0, 0), (0, 0, 0, 0), (0xC0,) * 4, 0, 0, flags), 1)


class Rig:
    def __init__(self, **kw):
        kw.setdefault("client", Client.BRINGUP)
        kw.setdefault("commission", True)
        self.host = McuLinkHost(CFG, **kw)
        self.t_us, self.now = 1_000_000, 10_000 * MS

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

    def ready(self, flags=CB):
        self.feed(flags=flags)
        assert self.host.request_arm(self.now) is None
        assert [type(m) for m in self.sent()] == [Arm]
        self.feed(mode=Mode.ARMED, gen=1, flags=flags)
        (zero,) = self.sent()
        self.feed(mode=Mode.ARMED, gen=1, flags=flags | StateFlag.ZERO_SEEN)
        assert self.host.ready
        return zero


def test_commission_mode_is_for_the_bringup_client_only():
    for client in (Client.REAL, Client.HIL):
        with pytest.raises(ValueError):
            McuLinkHost(CFG, client=client, commission=True)


def test_the_zero_command_after_arm_is_a_zero_commission_on_axis_0_even_with_steering_unmeasured():
    r = Rig()
    zero = r.ready()
    assert isinstance(zero, Commission)
    assert (zero.axis, zero.duty_e4, zero.arm_gen, zero.boot_id) == (0, 0, 1, BOOT)
    assert zero.valid_after_echo_ms <= CFG.zero_valid_ms


@pytest.mark.parametrize("flags,kw,refused", [
    (StateFlag.DRIVERS_READY, {}, "not_commission_build"),  # a normal build never gets a COMMISSION
    (StateFlag.HIL_PLANT | StateFlag.OUTPUTS_LOCKED | StateFlag.ESTOP_UNWIRED, {}, "not_commission_build"),
    (CB | StateFlag.OUTPUTS_LOCKED, {}, "outputs_locked"),
])
def test_the_commission_host_arms_only_a_commissioning_build(flags, kw, refused):
    r = Rig(**kw)
    r.feed(flags=flags)
    assert r.host.request_arm(r.now) == Blocked(f"build_refused: {refused}")
    assert r.sent() == []


def test_locked_outputs_are_allowed_only_when_asked():
    r = Rig(allow_locked=True)
    r.ready(flags=CB | StateFlag.OUTPUTS_LOCKED)


def test_the_plain_bringup_host_still_refuses_a_commissioning_build():
    r = Rig(commission=False)
    r.feed()
    assert r.host.request_arm(r.now) == Blocked("build_refused: commission_build")


def test_commission_and_drive_refuse_each_other():
    r = Rig()
    r.ready()
    cmd = RearDrive(0.0, 0.0, 0.0, 0.0, 0.0, False)
    assert r.host.drive(cmd, 0.0, True, r.now + 100 * MS, r.now) == Blocked("commission_host")
    plain = Rig(commission=False)
    plain.feed(flags=StateFlag.DRIVERS_READY)
    assert plain.host.commission(0, 1000, plain.now + 100 * MS, plain.now) == Blocked("not_commission_host")


def test_commission_carries_the_echo_and_a_validity_never_past_the_expiry():
    r = Rig()
    r.ready()
    rx, echo = r.now, r.t_us
    r.now += 30 * MS
    assert r.host.commission(2, -1500, rx + 120 * MS, r.now) is None
    (c,) = r.sent()
    assert (c.axis, c.duty_e4, c.echo_mcu_time_us, c.arm_gen) == (2, -1500, echo, 1)
    assert c.valid_after_echo_ms == 120  # from the echoed STATE's receipt to the expiry
    assert r.host.commission(0, 1000, rx + 900 * MS, r.now) is None
    (c,) = r.sent()
    assert c.valid_after_echo_ms == CFG.cap_ms


@pytest.mark.parametrize("axis,duty", [(4, 0), (-1, 0), (0, 10001)])
def test_commission_refuses_a_bad_value_without_queueing(axis, duty):
    r = Rig()
    r.ready()
    assert r.host.commission(axis, duty, r.now + 100 * MS, r.now) == Blocked("bad_value")
    assert r.sent() == []


def test_commission_needs_ready_freshness_and_a_future_expiry():
    r = Rig()
    r.feed()
    assert r.host.commission(0, 0, r.now + 100 * MS, r.now) == Blocked("not_authorized")
    r.ready()
    assert r.host.commission(0, 1000, r.now, r.now) == Blocked("expired")
    assert r.host.commission(0, 1000, r.now + 900 * MS, r.now + 60 * MS) == Blocked("stale")


def test_queued_commissions_follow_the_drive_outbox_rules():
    r = Rig()
    r.ready()
    # past its expiry when the adapter asks: dropped, not written
    assert r.host.commission(0, 1000, r.now + 5 * MS, r.now) is None
    r.now += 6 * MS
    assert r.sent() == []
    # drop_drives drops them too
    assert r.host.commission(0, 1000, r.now + 100 * MS, r.now) is None
    r.host.drop_drives("cleanup")
    assert r.sent() == []
    # a DISARM request drops them and goes out alone
    assert r.host.commission(0, 1000, r.now + 100 * MS, r.now) is None
    r.host.request_disarm(r.now)
    assert [type(m) for m in r.sent()] == [Disarm]


def test_a_revoked_authorization_drops_the_queued_commission():
    r = Rig()
    r.ready()
    assert r.host.commission(1, 1000, r.now + 100 * MS, r.now) is None
    r.feed(mode=Mode.STOPPING, gen=1, flags=CB | StateFlag.ZERO_SEEN)
    assert not r.host.ready
    assert [m for m in r.sent() if isinstance(m, Commission)] == []


def test_written_commissions_are_tracked_like_drives():
    r = Rig()
    r.ready()
    assert r.host.commission(0, 1200, r.now + 100 * MS, r.now) is None
    r.sent()
    written = r.host.drive_traffic()["written"]
    assert written[-1]["nonzero"] is True


def test_a_normal_build_appearing_while_armed_revokes_and_disarms():
    r = Rig()
    r.ready()
    events = r.feed(mode=Mode.ARMED, gen=1, flags=StateFlag.DRIVERS_READY | StateFlag.ZERO_SEEN)
    assert not r.host.ready and any("build_refused" in str(e.detail) for e in events)
    assert [type(m) for m in r.sent()] == [Disarm]


def test_the_executor_paths_are_unchanged():
    r = Rig(client=Client.REAL, commission=False)
    r.feed(flags=StateFlag.DRIVERS_READY)
    assert r.host.request_arm(r.now) is None
    r.sent()
    r.feed(mode=Mode.ARMED, gen=1, flags=StateFlag.DRIVERS_READY, steering=0.0)
    (zero,) = r.sent()
    assert isinstance(zero, Drive)
