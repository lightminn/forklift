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
    def __init__(self, start=True):
        self.now = 50_000 * MS
        self.host = McuLinkHost(HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100,
                                           outbox_max=16), client=Client.REAL)
        self.port = RealPlantPort(self.host, RealKinematics(VEH))
        self.mcu = FakeMcu()
        self.dec = LinkDecoder()
        self.sent, self.order, self.written = [], [], []
        self.settled = ("settled", 0)
        self.still = (0, self.mcu.boot)
        self.permit = ra.AutoPermit(self.host, self.port, outstanding=lambda now: self.settled,
                                    stop_since=lambda now: self.still)
        self.collect_raises = False
        self.permit.bind(transmit=self.transmit, arbiter_stop=lambda reason: self.order.append(("arbiter", reason)),
                         collect=self.collect)
        self.cycle(3)
        if start:
            self.permit.start_session(self.now)
            self.cycle(2)
            assert self.permit.session_started

    def collect(self, reason):
        self.order.append(("collect", reason))
        if self.collect_raises:
            raise RuntimeError("slam stale")

    def transmit(self):
        self.order.append("transmit")
        while (f := self.host.next_frame(self.now)) is not None:
            self.host.frame_written(f, self.now)
            self.written.append((f.seq, self.now))
            self.permit.stop_written(f.seq, self.now)
            for r in self.dec.feed(f.data, self.now):
                self.sent.append(r.message)
                self.mcu.receive(r.message, f.seq)

    def cycle(self, n=1, transmit=True):
        for _ in range(n):
            for _ in range(2):
                self.now += 10 * MS
                self.mcu.advance(10_000)
                self.host.on_bytes(encode(self.mcu.state(), 1), self.now)
            self.host.tick(self.now)
            self.permit.tick(self.now)
            if transmit:
                self.transmit()

    def armed_ready(self):
        assert self.permit.adopted(self.now, pre_arm_refusal=None)
        self.cycle(5)
        assert self.permit.state == ra.READY


def test_the_session_starts_with_a_disarm_and_no_arm_before_it_is_answered():
    b = Bench(start=False)
    assert not b.permit.session_started
    assert not b.permit.adopted(b.now, pre_arm_refusal=None)  # before the session's DISARM was answered
    assert b.permit.reason == "pre_arm:session_not_started" and not any(isinstance(m, Arm) for m in b.sent)
    b = Bench(start=False)
    b.permit.start_session(b.now)
    assert isinstance(b.sent[-1], Disarm)
    b.cycle(2)
    assert b.permit.session_started


def test_the_arm_is_asked_once_after_the_adoption_and_ready_comes_through_the_hosts_zero_drive():
    b = Bench()
    assert b.permit.state == ra.OFF and not any(isinstance(m, Arm) for m in b.sent)
    assert b.permit.adopted(b.now, pre_arm_refusal=None) and b.permit.state == ra.ARMING
    assert not b.permit.drive_allowed()
    b.cycle(5)
    assert b.permit.state == ra.READY and b.permit.drive_allowed() and b.permit.armed
    arms = [m for m in b.sent if isinstance(m, Arm)]
    zeros = [m for m in b.sent if isinstance(m, Drive)]
    assert len(arms) == 1 and zeros and zeros[0].left_wheel_rad_s == 0.0 == zeros[0].right_wheel_rad_s
    b.permit.drove(b.now)
    assert b.permit.state == ra.DRIVING


def test_a_pre_arm_refusal_runs_the_full_latch_and_collects_the_mission():
    b = Bench()
    b.order.clear()
    assert not b.permit.adopted(b.now, pre_arm_refusal="stop_unconfirmed")
    assert b.permit.state == ra.LATCHED and b.permit.reason == "pre_arm:stop_unconfirmed"
    assert b.order == ["transmit", ("arbiter", "pre_arm:stop_unconfirmed"), ("collect", "pre_arm:stop_unconfirmed")]
    assert not any(isinstance(m, Arm) for m in b.sent)


def test_an_arm_that_never_answers_latches():
    b = Bench()
    b.mcu.deaf = True
    b.permit.adopted(b.now, pre_arm_refusal=None)
    b.cycle(40)  # past the host's 0.3 s arm timeout
    assert b.permit.state == ra.LATCHED and b.permit.reason == "arm_failed"


def test_a_latch_runs_every_step_even_when_one_raises_and_collects_once():
    b = Bench()
    b.armed_ready()
    b.order.clear()
    b.collect_raises = True
    with pytest.raises(RuntimeError):
        b.permit.latch(b.now, "slam_expired")
    assert b.order == ["transmit", ("arbiter", "slam_expired"), ("collect", "slam_expired")]
    assert isinstance(b.sent[-1], Disarm) and b.permit.state == ra.LATCHED
    b.order.clear()
    b.permit.latch(b.now, "again")  # a latch while latched: the DISARM again, no second stop or collection
    assert b.order == ["transmit"] and b.permit.reason == "slam_expired"
    b.cycle(2)
    assert b.mcu.mode == Mode.DISARMED and not b.permit.armed


def test_a_transmit_failure_still_stops_the_arbiter_and_collects():
    b = Bench()
    b.armed_ready()
    b.order.clear()

    def broken():
        b.order.append("transmit")
        raise OSError("device gone")

    b.permit.bind(transmit=broken, arbiter_stop=lambda r: b.order.append(("arbiter", r)), collect=b.collect)
    with pytest.raises(OSError):
        b.permit.latch(b.now, "link_lost")
    assert b.order == ["transmit", ("arbiter", "link_lost"), ("collect", "link_lost")]
    assert b.permit.state == ra.LATCHED


def test_any_mission_end_leaves_the_authority_through_the_latch():
    b = Bench()
    b.armed_ready()
    b.order.clear()
    b.permit.mission_ended(b.now, "pocket_first_frame")
    assert b.permit.state == ra.LATCHED and b.permit.reason == "mission_end:pocket_first_frame"
    assert ("collect", "mission_end:pocket_first_frame") in b.order


def test_the_watch_stays_on_until_a_disarmed_is_seen_and_with_no_state_at_all():
    b = Bench()
    b.armed_ready()
    b.permit.bind(transmit=None, arbiter_stop=None, collect=None)
    b.permit.latch(b.now, "overspeed")  # not transmitted yet: the MCU is still armed
    assert b.permit.state == ra.LATCHED and b.permit.armed
    b.host._state = None  # a lost link: unknown is not disarmed  # noqa: SLF001
    assert b.permit.armed


def test_a_lost_authorization_latches():
    b = Bench()
    b.armed_ready()
    b.mcu.mode = Mode.DISARMED  # the MCU dropped its arming on its own
    b.cycle(2)
    assert b.permit.state == ra.LATCHED and b.permit.reason == "authorization_revoked"


def queue_stop_frame(b):
    """What 4c's control stop does: a new generation, then the stop frame at the measured lift; its seq."""
    from forklift_core.plant.real_vehicle import RearDrive

    b.port.new_control_generation()
    seq = b.host.skip_seq(0)
    b.host.drive(RearDrive(0.0, 0.0, 0.0, 0.0, 0.0, False), lift_m=0.0, lift_hold=True,
                 expires_ns=b.now + 100 * MS, now_ns=b.now)
    return seq


def test_s10s_control_stop_is_confirmed_by_its_exact_seq_after_its_write_then_disarmed_then_latched():
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


def test_a_stop_frame_never_written_is_not_confirmed_by_a_later_seq():
    b = Bench()
    b.armed_ready()
    seq = queue_stop_frame(b)
    b.permit.control_stop(b.now, seq)
    b.host.drop_drives("test")  # the frame never goes out
    b.mcu.rx = (seq + 5) & 0xFFFF  # an echo naming a later seq
    b.cycle(6, transmit=False)
    assert b.permit.reason == "control_stop_unconfirmed"


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
    b.permit.latch(b.now, "stop")
    b.cycle(3)
    assert b.permit.state == ra.LATCHED and b.mcu.mode == Mode.DISARMED


def test_a_rearm_needs_a_fresh_disarmed_everything_settled_and_a_stop_seen_after():
    b = Bench()
    latched(b)
    b.cycle(10)
    b.settled = ("unknown", None)
    assert b.permit.rearm_refusal(b.now) == "outstanding_unknown"
    b.settled = ("pending", None)
    assert b.permit.rearm_refusal(b.now) == "outstanding_pending"
    b.settled = ("settled", b.now - 50 * MS)
    b.still = None
    assert b.permit.rearm_refusal(b.now) == "stop_unconfirmed"
    b.still = (b.now - 80 * MS, b.mcu.boot)  # standing since before the commands settled
    assert b.permit.rearm_refusal(b.now) == "stop_before_settled"
    b.still = (b.now - 20 * MS, b.mcu.boot + 1)  # another boot's
    assert b.permit.rearm_refusal(b.now) == "stop_before_settled"
    b.still = (b.now - 20 * MS, b.mcu.boot)
    assert b.permit.rearm_refusal(b.now) is None
    b.permit.rearmed(b.now, 5)
    assert b.permit.state == ra.OFF and b.permit.reason is None
    assert b.permit.events[-1].reason == "stop"  # the old reason kept in the events only


def test_a_stale_state_refuses_the_rearm():
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
