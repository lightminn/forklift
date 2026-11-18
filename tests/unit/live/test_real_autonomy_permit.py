"""Design 2026-11-19-real-assembly 7: the real AUTO permit on the real host and port, the MCU faked (FakeMcu): the ARM
asked once after the adoption, ready only through the host's own zero DRIVE, every latch's order, the S10 control
stop, and the rearm conditions."""

import itertools

import pytest

from forklift_core.live import real_autonomy as ra
from forklift_core.mcu_link.build_policy import Client
from forklift_core.mcu_link.messages import Arm, Disarm, Drive, LinkDecoder, Mode, encode
from forklift_core.plant.kinematics import RealKinematics
from forklift_core.plant.mcu_host import HostConfig, McuLinkHost
from forklift_core.plant.port import RealPlantPort

from test_teleop_bridge_cycle import MS, VEH, FakeMcu


class Bench:
    def __init__(self):
        self.now = 50_000 * MS
        self.host = McuLinkHost(HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100,
                                           outbox_max=16), client=Client.REAL)
        self.port = RealPlantPort(self.host, RealKinematics(VEH))
        self.mcu = FakeMcu()
        self.dec = LinkDecoder()
        self.sent, self.order = [], []
        self.still = True
        self.permit = ra.AutoPermit(self.host, self.port, stop_confirmed=lambda now: self.still)
        self.cycle(3)

    def transmit(self):
        self.order.append("transmit")
        while (f := self.host.next_frame(self.now)) is not None:
            self.host.frame_written(f, self.now)
            for r in self.dec.feed(f.data, self.now):
                self.sent.append(r.message)
                self.mcu.receive(r.message, f.seq)

    def cycle(self, n=1, **hooks):
        for _ in range(n):
            for _ in range(2):
                self.now += 10 * MS
                self.mcu.advance(10_000)
                self.host.on_bytes(encode(self.mcu.state(), 1), self.now)
            self.host.tick(self.now)
            self.permit.tick(self.now, **hooks)
            self.transmit()

    def latch_hooks(self, collect_raises=False):
        def collect(reason):
            self.order.append(("collect", reason))
            if collect_raises:
                raise RuntimeError("slam stale")

        return dict(transmit=self.transmit, arbiter_stop=lambda reason: self.order.append(("arbiter", reason)),
                    collect=collect)

    def armed_ready(self):
        assert self.permit.adopted(self.now, pre_arm_refusal=None)
        self.cycle(5)
        assert self.permit.state == ra.READY


def kinds(msgs):
    return [type(m).__name__ for m in msgs]


def test_the_arm_is_asked_once_after_the_adoption_and_ready_comes_through_the_hosts_zero_drive():
    b = Bench()
    assert b.permit.state == ra.OFF and not any(isinstance(m, Arm) for m in b.sent)  # nothing before the adoption
    assert b.permit.adopted(b.now, pre_arm_refusal=None) and b.permit.state == ra.ARMING
    assert not b.permit.drive_allowed()
    b.cycle(5)
    assert b.permit.state == ra.READY and b.permit.drive_allowed() and b.permit.armed
    arms = [m for m in b.sent if isinstance(m, Arm)]
    zeros = [m for m in b.sent if isinstance(m, Drive)]
    assert len(arms) == 1 and zeros and zeros[0].left_wheel_rad_s == 0.0 == zeros[0].right_wheel_rad_s
    b.permit.drove(b.now)
    assert b.permit.state == ra.DRIVING


def test_a_pre_arm_refusal_or_a_blocked_arm_latches_without_an_arm():
    b = Bench()
    assert not b.permit.adopted(b.now, pre_arm_refusal="stop_unconfirmed")
    assert b.permit.state == ra.LATCHED and b.permit.reason == "pre_arm:stop_unconfirmed"
    b.transmit()
    assert not any(isinstance(m, Arm) for m in b.sent)


def test_an_arm_that_never_answers_latches():
    b = Bench()
    b.mcu.deaf = True
    b.permit.adopted(b.now, pre_arm_refusal=None)
    b.cycle(40, **b.latch_hooks())  # past the host's 0.3 s arm timeout
    assert b.permit.state == ra.LATCHED and b.permit.reason == "arm_failed"


def test_a_latch_drops_the_candidates_disarms_and_transmits_before_the_arbiter_and_the_collection():
    b = Bench()
    b.armed_ready()
    b.order.clear()
    with pytest.raises(RuntimeError):
        b.permit.latch(b.now, "slam_expired", **b.latch_hooks(collect_raises=True))
    assert b.order == ["transmit", ("arbiter", "slam_expired"), ("collect", "slam_expired")]
    assert isinstance(b.sent[-1], Disarm) and b.permit.state == ra.LATCHED
    b.cycle(2)
    assert b.mcu.mode == Mode.DISARMED and not b.permit.armed


def test_the_watch_stays_on_until_the_latchs_disarm_is_seen():
    b = Bench()
    b.armed_ready()
    b.permit.latch(b.now, "overspeed")  # no transmit yet: the MCU is still armed
    assert b.permit.state == ra.LATCHED and b.permit.armed
    b.transmit()
    b.cycle(1)
    assert not b.permit.armed


def test_a_lost_authorization_latches():
    b = Bench()
    b.armed_ready()
    b.mcu.mode = Mode.DISARMED  # the MCU dropped its arming on its own
    b.cycle(2, **b.latch_hooks())
    assert b.permit.state == ra.LATCHED and b.permit.reason == "authorization_revoked"


def queue_stop_frame(b):
    """What 4c's control stop does: a new generation, then the stop frame at the measured lift (seq from the host)."""
    b.port.new_control_generation()
    seq = b.host.skip_seq(0)
    b.host.drive(_stop_drive(), lift_m=0.0, lift_hold=True, expires_ns=b.now + 100 * MS, now_ns=b.now)
    return seq


def _stop_drive():
    from forklift_core.plant.real_vehicle import RearDrive

    return RearDrive(0.0, 0.0, 0.0, 0.0, 0.0, False)


def test_s10s_control_stop_is_confirmed_by_its_seq_then_disarmed_then_latched():
    b = Bench()
    b.armed_ready()
    seq = queue_stop_frame(b)
    b.permit.control_stop(b.now, seq)
    assert b.permit.state == ra.STOP_PENDING and b.permit.armed
    b.transmit()
    b.cycle(1)
    assert b.permit.state == ra.DISARMING and isinstance(b.sent[-1], Disarm)
    assert b.permit.events[-2].event == "control_stop_confirmed"
    b.cycle(2)
    assert b.permit.state == ra.LATCHED and b.permit.reason == "s10_stopped" and not b.permit.armed


def test_an_unconfirmed_control_stop_disarms_after_its_limit():
    b = Bench()
    b.armed_ready()
    b.mcu.ack_frozen = True  # applied, never echoed
    seq = queue_stop_frame(b)
    b.permit.control_stop(b.now, seq)
    b.cycle(6)  # 120 ms
    assert b.permit.reason == "control_stop_unconfirmed" and b.permit.state in (ra.DISARMING, ra.LATCHED)


def latched(b):
    b.armed_ready()
    b.permit.latch(b.now, "stop", transmit=b.transmit)
    b.cycle(3)
    assert b.permit.state == ra.LATCHED and b.mcu.mode == Mode.DISARMED


def test_a_rearm_needs_a_fresh_disarmed_no_command_left_and_a_stop_seen():
    b = Bench()
    latched(b)
    b.still = False
    assert b.permit.rearm_refusal(b.now) == "stop_unconfirmed"
    b.still = True
    b.cycle(10)  # every written DRIVE past its expiry
    assert b.permit.rearm_refusal(b.now) is None
    b.permit.rearmed(b.now, 5)
    assert b.permit.state == ra.OFF


def test_a_written_command_not_yet_expired_refuses_the_rearm():
    b = Bench()
    b.armed_ready()
    b.permit.latch(b.now, "stop", transmit=b.transmit)
    b.cycle(1)
    assert b.permit.rearm_refusal(b.now) in ("outstanding_unknown", "old_command_may_act", "mcu_armed")


def test_a_stale_state_or_a_fault_refuses_the_rearm():
    b = Bench()
    latched(b)
    b.cycle(10)
    b.now += 200 * MS  # no STATE since
    assert b.permit.rearm_refusal(b.now) == "state_stale"


def test_only_the_arbiters_rearm_transition_releases_the_latch():
    b = Bench()
    latched(b)
    b.permit.tick(b.now)
    assert b.permit.state == ra.LATCHED  # the inputs back alone do nothing
    b.permit.rearmed(b.now, 3)
    assert b.permit.state == ra.OFF
