"""The teleop bridge's cycle (docs/design/2026-11-12-teleop-bridge.md 4-9) against a real McuLinkHost and a small
stand-in MCU (frames decoded, ARM/DRIVE/DISARM applied, counts integrated, expiry -> STOPPING -> DISARMED). The
console speaks through the real Channels/Link. The C firmware core is the end-to-end test's."""

import itertools
import math
from pathlib import Path

import pytest

from forklift_core.live import real_teleop as rt
from forklift_core.live.link import Channels, Link
from forklift_core.mcu_link.build_policy import Client
from forklift_core.mcu_link.messages import (
    Arm, Disarm, Drive, FaultFlag, LinkDecoder, McuState, Mode, RejectReason, StateFlag, encode,
)
from forklift_core.plant.bringup import Sample
from forklift_core.plant.kinematics import RealKinematics, teleop_limit_overrides
from forklift_core.plant.mcu_host import HostConfig, McuLinkHost
from forklift_core.plant.port import RealPlantPort
from forklift_core.plant.real_vehicle import load_real_vehicle
from forklift_core.safety.arbiter import Arbiter, ArbiterConfig
from forklift_core.safety.teleop_drive import TeleopLimits

REPO = Path(__file__).resolve().parents[3]
VEH = load_real_vehicle(REPO / "config" / "real" / "vehicle.yaml")
PARAMS = rt.load_bridge_params(REPO / "config" / "real" / "teleop_bridge.yaml", VEH)
CAP = rt.command_cap_mps(VEH.teleop_speed_limit_mps, PARAMS)
EPOCH = 5
ALL = frozenset(rt.CONFIRMATIONS)
MS = 1_000_000


class FakeMcu:
    """Just enough of the firmware's link behaviour for the bridge's cycle."""

    def __init__(self, flags=StateFlag.DRIVERS_READY):
        self.t_us, self.boot, self.gen, self.rx = 1_000_000, 0x5151, 0, 0
        self.mode, self.flags0 = Mode.DISARMED, flags
        self.zero_seen = False
        self.cmd = (0.0, 0.0)
        self.deadline_us = None
        self.stop_until = None
        self.counts = [0.0, 0.0]
        self.rate = [0.0, 0.0]
        self.q = 0.0
        self.faults = FaultFlag(0)
        self.extra_rate = 0.0  # a wheel faster than asked (overspeed injection)
        self.deaf = False  # frames are not received (a TX-only cut seen from the MCU)
        self.disarms = 0
        self.arms = 0
        self.arm_delay = 0  # ARM answered this many 10 ms steps late
        self.ack_frozen = False  # frames applied but last_rx_seq no longer updated (no round-trip anchor)
        self._arm_due = None

    def receive(self, msg, seq):
        if self.deaf:
            return
        if isinstance(msg, Arm) and self.arm_delay and self._arm_due is None:
            self._arm_due = (self.arm_delay, seq)
            return
        if isinstance(msg, Arm):
            if self.mode == Mode.DISARMED:
                self.gen += 1
                self.mode, self.zero_seen, self.cmd = Mode.ARMED, False, (0.0, 0.0)
                self.deadline_us = self.t_us + 200_000
                self.arms += 1
                self.rx = seq
        elif isinstance(msg, Disarm):
            self.disarms += 1
            if self.mode in (Mode.ARMED, Mode.STOPPING):
                self.mode, self.cmd = Mode.DISARMED, (0.0, 0.0)
            self.rx = seq
        elif isinstance(msg, Drive) and self.mode == Mode.ARMED and msg.arm_gen == self.gen:
            nonzero = msg.left_wheel_rad_s != 0.0 or msg.right_wheel_rad_s != 0.0
            if nonzero and not self.zero_seen:
                return
            self.zero_seen = True
            self.cmd = (msg.left_wheel_rad_s, msg.right_wheel_rad_s)
            self.q = msg.steering_rad
            self.deadline_us = msg.echo_mcu_time_us + msg.valid_after_echo_ms * 1000
            if not self.ack_frozen:
                self.rx = seq

    def advance(self, dt_us):
        self.t_us += dt_us
        if self._arm_due is not None:
            n, seq = self._arm_due
            if n <= 1:
                self._arm_due, delay, self.arm_delay = None, self.arm_delay, 0
                self.receive(Arm(self.boot, self.t_us), seq)
                self.arm_delay = delay
            else:
                self._arm_due = (n - 1, seq)
        if self.mode == Mode.ARMED and self.deadline_us is not None and self.t_us > self.deadline_us:
            self.mode, self.cmd, self.stop_until = Mode.STOPPING, (0.0, 0.0), self.t_us + 100_000
        if self.mode == Mode.STOPPING and self.t_us >= self.stop_until:
            self.mode = Mode.DISARMED
        target = self.cmd if self.mode == Mode.ARMED else (0.0, 0.0)
        for i in (0, 1):
            self.rate[i] = target[i] + (self.extra_rate if target[i] else 0.0)
            self.counts[i] += self.rate[i] * dt_us / 1e6 * VEH.counts_per_wheel_rev[i] / (2 * math.pi)

    def state(self):
        flags = self.flags0 | (StateFlag.ZERO_SEEN if self.zero_seen and self.mode == Mode.ARMED else StateFlag(0))
        return McuState(self.t_us, self.boot, self.gen, self.rx, self.mode, RejectReason.NONE, self.faults,
                        _i32(self.counts[0]), _i32(self.counts[1]),
                        self.rate[0], self.rate[1], self.q, 0.0, (0,) * 4, (0,) * 4, (0xC0,) * 4, 0, 0, flags)


def _i32(c):
    return (int(round(c)) + 2**31) % 2**32 - 2**31


def open_floor(curvatures, direction, check):
    return 1.0, "ok"


class Rig:
    def __init__(self, *, confirmed=ALL, safety=None, floor=open_floor, mcu_flags=StateFlag.DRIVERS_READY):
        self.now = 50_000 * MS
        tokens = itertools.count(1)
        limits = rt.real_teleop_limits(TeleopLimits(**teleop_limit_overrides(RealKinematics(VEH))), CAP)
        self.arbiter = Arbiter(EPOCH, ArbiterConfig(tick_s=0.02, teleop_limits=limits), lambda: f"t{next(tokens)}")
        self.channels = Channels()
        self.link = Link(self.arbiter, self.channels, tick_period_s=0.05, state_period_s=0.1)
        self.tap = LinkDecoder()
        self.bridge = None
        self.host = McuLinkHost(HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100,
                                           outbox_max=16), client=Client.REAL,
                                on_state=lambda s, ns: self.bridge.add_sample(Sample(ns, s)),
                                log=self._log)
        self.kin = RealKinematics(VEH)
        self.port = RealPlantPort(self.host, self.kin)
        provider = (lambda now: safety) if safety is not None else (lambda now: rt.bench_safety(confirmed))
        self.bridge = rt.TeleopBridge(link=self.link, host=self.host, port=self.port, kinematics=self.kin,
                                      params=PARAMS, limit_mps=VEH.teleop_speed_limit_mps, floor_permit=floor,
                                      safety_provider=provider)
        self.mcu = FakeMcu(mcu_flags)
        self.dec = LinkDecoder()
        self.sent = []  # (now_ns, message) written to the MCU
        self.seq = itertools.count(1)
        self.tick = None
        self.lease = None
        self.channels.connect("c1", "op", control=True)
        self.recs = []
        self.replies = []

    def _log(self, what, now_ns, data, info):
        if what == "written" and info is not None:
            self.bridge.frame_written(info["seq"], now_ns)

    def say(self, kind, **fields):
        msg = {"t": kind, "request_id": f"r{next(self.seq)}", "session_epoch": EPOCH, **fields}
        if self.lease is not None:
            msg.setdefault("lease_id", self.lease)
        msg.setdefault("control_gen", self.arbiter.control_gen)
        self.channels.message("c1", "op", msg)

    def _pump(self):
        for conn, msg in self.channels.take_out():
            if msg.get("t") == "tick":
                self.tick = msg
            elif "request_id" in msg or msg.get("t") in ("ack", "nack"):
                self.replies.append(msg)

    def _replies(self):
        self._pump()
        out, self.replies = self.replies, []
        return out

    def echo(self):
        if self.tick is not None and self.lease is not None:
            self.channels.message("c1", "op", {"t": "echo", "session_epoch": EPOCH, "lease_id": self.lease,
                                               "echo": {"state_seq": self.tick["state_seq"],
                                                        "token": self.tick["token"]}})

    def teleop(self, speed_ratio=1.0, curvature_ratio=0.0, deadman=True):
        a = self.arbiter
        self.channels.message("c1", "op", {
            "t": "teleop", "session_epoch": EPOCH, "lease_id": self.lease, "control_gen": a.control_gen,
            "arm_gen": a.arm_gen, "seq": next(self.seq), "mode": "direct", "lift_dir": 0, "deadman": deadman,
            "speed_ratio": speed_ratio, "curvature_ratio": curvature_ratio,
            "echo": {"state_seq": self.tick["state_seq"], "token": self.tick["token"]}})

    def cycle(self, *, echo=True, transmit=True, teleop=None, before=None):
        """20 ms: two MCU STATEs received, the bridge's cycle, then transmit (design 5's order)."""
        for _ in range(2):
            self.now += 10 * MS
            self.mcu.advance(10_000)
            self.host.on_bytes(encode(self.mcu.state(), 1), self.now)
        self.host.tick(self.now)
        self._pump()  # the console reads the newest tick before it answers (its echo and its teleop name it)
        if echo:
            self.echo()
        if teleop is not None:
            self.teleop(**teleop)
        if before is not None:
            before(self)
        rec = self.bridge.cycle(self.now)
        self.recs.append(rec)
        if transmit:
            while (f := self.host.next_frame(self.now)) is not None:
                self.host.frame_written(f, self.now)
                for r in self.dec.feed(f.data, self.now):
                    self.sent.append((self.now, r.message))
                    self.mcu.receive(r.message, f.seq)
        return rec

    def run(self, n, **kw):
        for _ in range(n):
            rec = self.cycle(**kw)
        return rec

    def grant(self):
        self.run(3, echo=False)
        self.say("lease_request")
        self.cycle(echo=False)
        rep = [m for m in self._replies() if m.get("lease_id")]
        self.lease = rep[-1]["lease_id"]
        self.run(30)
        self._replies()

    def to_teleop(self):
        self.grant()
        self.say("mode_request", to="TELEOP")
        self.run(3)
        assert self.arbiter.mode == "TELEOP", (self.arbiter.mode, self._replies())

    def arm(self):
        self.say("arm")
        self.run(15)

    def nonzero_sent(self, since=0):
        return [m for t, m in self.sent[since:] if isinstance(m, Drive)
                and (m.left_wheel_rad_s != 0.0 or m.right_wheel_rad_s != 0.0)]


def armed_rig(**kw):
    r = Rig(**kw)
    r.to_teleop()
    r.arm()
    assert r.bridge.phase == rt.READY and r.host.ready, (r.bridge.phase, r.recs[-1])
    return r


# --- arming ------------------------------------------------------------------------------------------------------

def test_an_operator_arm_arms_the_mcu_once_and_reaches_ready():
    r = armed_rig()
    assert r.mcu.arms == 1 and [type(m) for _t, m in r.sent].count(Arm) == 1
    r.run(20)
    assert r.mcu.arms == 1  # never again by itself


def test_slam_off_teleop_drives_under_the_cap_and_the_bound():
    """Lead decision: without SLAM the teleop opens (autonomy stays closed); the bench picture has slam False."""
    r = armed_rig()
    assert r.bridge.safety_provider(0.0)["frame"]["slam"] is False
    for _ in range(40):
        r.cycle(teleop={"speed_ratio": 1.0})
    drives = r.nonzero_sent()
    assert drives, r.recs[-1]
    rad = max(abs(m.left_wheel_rad_s) for m in drives)
    assert rad * VEH.wheel_radius_m[0] <= CAP + 1e-6
    assert r.recs[-1]["speed"]["bound"] <= VEH.teleop_speed_limit_mps
    assert r.recs[-1]["speed"]["signed"] > 0.05


def test_auto_is_invalid_and_missions_unsupported():
    r = Rig()
    r.grant()
    r.say("mode_request", to="AUTO")
    r.run(2)
    reps = r._replies()
    assert any(m.get("t") == "reject" and m.get("reason") == "invalid" for m in reps), reps
    assert r.arbiter.mode != "AUTO"


@pytest.mark.parametrize("change", [
    {"recovery_required": True},
    {"load": {"loaded": True}},
    {"fork_in_pocket": {"value": True, "relation": "carrying"}},
    {"recovery_required": True, "fork_in_pocket": {"value": True, "relation": "constrained"},
     "recovery": {"stage": "in_pocket", "grounded": True, "evidence": True, "aligned": True}},
])
def test_the_real_truck_refusals_send_no_nonzero_drive(change):
    r = Rig(safety={**rt.bench_safety(ALL), **change})
    r.grant()
    r.say("mode_request", to="TELEOP")
    r.run(3)
    r.say("arm")
    for _ in range(30):
        r.cycle(teleop={"speed_ratio": 1.0} if r.arbiter.mode == "TELEOP" and r.arbiter.armed else None)
    assert r.nonzero_sent() == []


@pytest.mark.parametrize("missing", rt.CONFIRMATIONS)
def test_a_missing_start_confirmation_sends_no_nonzero_drive(missing):
    r = Rig(confirmed=ALL - {missing})
    r.grant()
    r.say("mode_request", to="TELEOP")
    r.say("arm")
    r.run(30)
    assert r.arbiter.mode != "TELEOP" and r.nonzero_sent() == [] and r.mcu.arms == 0


def test_without_a_scan_the_floor_permit_is_zero_and_nothing_moves():
    r = armed_rig(floor=lambda c, d, v: (0.0, "no_scan"))
    for _ in range(30):
        r.cycle(teleop={"speed_ratio": 1.0})
    assert r.nonzero_sent() == []


# --- stopping -------------------------------------------------------------------------------------------------------

def test_an_operator_stop_disarms_at_once_and_no_older_drive_goes_out_after_it():
    r = armed_rig()
    r.run(10, teleop={"speed_ratio": 1.0})
    n = len(r.sent)
    r.channels.message("c1", "op", {"t": "stop", "session_epoch": EPOCH, "request_id": "s1"})
    r.cycle(teleop=None)
    after = r.sent[n:]
    assert any(isinstance(m, Disarm) for _t, m in after)
    assert r.nonzero_sent(n) == []  # the queued nonzero DRIVE was dropped before the transmit
    assert r.arbiter.mode == "STOPPED"


def test_an_overspeed_bound_stops_and_disarms():
    r = armed_rig()
    r.mcu.extra_rate = 1.0  # +0.125 m/s on top of what was asked
    for _ in range(40):
        rec = r.cycle(teleop={"speed_ratio": 1.0})
        if rec.get("stop"):
            break
    assert r.arbiter.mode == "STOPPED" and any(isinstance(m, Disarm) for _t, m in r.sent)
    over = [x for x in r.recs if x.get("stop") == "overspeed"]
    assert over and over[0]["speed"]["bound"] > VEH.teleop_speed_limit_mps


def test_an_invalid_speed_while_driving_is_a_stop_not_only_a_latch():
    r = armed_rig()
    r.run(10, teleop={"speed_ratio": 1.0})
    r.mcu.faults = FaultFlag.ENCODER_GAP
    r.cycle(teleop={"speed_ratio": 1.0})
    assert r.arbiter.mode == "STOPPED"
    assert any(isinstance(m, Disarm) for _t, m in r.sent)


def test_an_mcu_stopping_is_let_run_out_before_the_disarm():
    r = armed_rig()
    r.run(5, teleop={"speed_ratio": 1.0})
    r.mcu.deaf = True  # the MCU hears nothing more: its deadline expires
    disarms_before = r.mcu.disarms
    seen_stopping = False
    for _ in range(30):
        rec = r.cycle(teleop={"speed_ratio": 1.0})
        if r.mcu.mode == Mode.STOPPING:
            seen_stopping = True
            assert not any(isinstance(m, Disarm) for _t, m in r.sent[-3:]), "a DISARM cut the STOPPING"
    assert seen_stopping and r.arbiter.mode == "STOPPED"
    r.mcu.deaf = False
    r.run(5)
    assert r.bridge.phase in (rt.IDLE, rt.DISARMING) and r.mcu.disarms >= disarms_before


def test_an_operator_stop_during_the_mcu_stopping_disarms_at_once():
    r = armed_rig()
    r.run(5, teleop={"speed_ratio": 1.0})
    r.mcu.deaf = True
    for _ in range(30):
        r.cycle()
        if r.mcu.mode == Mode.STOPPING:
            break
    assert r.bridge.mcu_wait is not None
    r.mcu.deaf = False
    n = len(r.sent)
    r.channels.message("c1", "op", {"t": "stop", "session_epoch": EPOCH, "request_id": "s2"})
    r.cycle()
    assert any(isinstance(m, Disarm) for _t, m in r.sent[n:])


def test_a_reconnect_does_not_rearm():
    r = armed_rig()
    r.host.disconnected(r.now)
    r.run(3)
    assert r.arbiter.mode == "STOPPED"
    arms = r.mcu.arms
    r.run(30)
    assert r.mcu.arms == arms


# --- arming conditions ----------------------------------------------------------------------------------------------

def test_disarm_and_arm_in_one_cycle_disarm_the_mcu_then_arm_it_again():
    r = armed_rig()
    gen0 = r.mcu.gen
    r.say("disarm")
    r.say("arm")
    r.run(30)
    assert r.mcu.disarms >= 1 and r.mcu.arms == 2 and r.mcu.gen == gen0 + 1
    assert r.bridge.phase == rt.READY


def test_a_new_arm_while_rolling_waits_for_a_stand():
    r = armed_rig()
    r.run(15, teleop={"speed_ratio": 1.0})
    arms_before = r.mcu.arms
    r.say("disarm")
    r.say("arm")
    recs = [r.cycle() for _ in range(60)]
    armed_at = [k for k, x in enumerate(recs) if x.get("arm") is not None]
    assert armed_at and r.mcu.arms == arms_before + 1
    k = armed_at[0]
    assert recs[k]["arm_still"] is True  # the ARM went out on a measured stand
    assert any(x.get("arm_wait") == "not_standing" for x in recs[:k])  # and not while still rolling


def test_a_late_arm_answer_is_waited_for_not_refused():
    """Codex impl P2-4: the host's ARM pending is not a revoke."""
    r = Rig()
    r.mcu.arm_delay = 6  # 60 ms
    r.to_teleop()
    r.arm()
    assert r.bridge.phase == rt.READY and r.mcu.arms == 1
    assert not any(str(x.get("stop", "")).startswith("mcu_arm_refused") for x in r.recs)


def test_a_locked_board_is_never_armed():
    r = Rig(mcu_flags=StateFlag.DRIVERS_READY | StateFlag.OUTPUTS_LOCKED)
    r.to_teleop()
    r.arm()
    assert r.mcu.arms == 0 and r.arbiter.mode == "STOPPED"
    assert any(str(x.get("stop", "")).startswith("mcu_arm_refused") for x in r.recs)


# --- the output gate ------------------------------------------------------------------------------------------------

def test_a_drive_never_lives_past_the_teleop_deadline_or_the_lease():
    r = armed_rig()
    r.run(5, teleop={"speed_ratio": 1.0})
    a = r.arbiter
    written = r.host.drive_traffic()["written"]
    last = written[-1]
    assert last["nonzero"]
    assert last["expires_ns"] <= int(a.teleop["deadline"] * 1e9)
    assert last["expires_ns"] <= int((a.lease.renewed_mono + a.config.lease_timeout_s) * 1e9)


def test_released_deadman_holds_the_measured_steering():
    r = armed_rig()
    r.run(10, teleop={"speed_ratio": 0.0, "curvature_ratio": 1.0})  # presteer
    q_turned = r.mcu.q
    assert abs(q_turned) > 1e-3
    r.run(5, teleop={"speed_ratio": 0.0, "curvature_ratio": -1.0, "deadman": False})
    last = [m for _t, m in r.sent if isinstance(m, Drive)][-1]
    assert last.steering_rad == pytest.approx(r.bridge.reading.steering_rad, abs=1e-6)
    assert last.left_wheel_rad_s == 0.0


def test_a_port_refusal_disarms_and_stops():
    r = armed_rig()
    from forklift_core.plant.port import PortResult

    r.port.command = lambda cmd, now_ns: PortResult("blocked", "outbox_full")
    r.cycle(teleop={"speed_ratio": 1.0})
    assert r.arbiter.mode == "STOPPED"
    r.port.command = RealPlantPort.command.__get__(r.port)
    r.cycle()
    assert any(isinstance(m, Disarm) for _t, m in r.sent)


def test_an_unasked_backward_motion_cuts_a_forward_request():
    """Codex P1-1 / the core's rule: the encoder interval at a stand spans both directions at the bound; a rear that
    does not allow that bound stops a forward request (a hold), whatever the forward permission."""
    r = armed_rig(floor=lambda c, d, v: (1.0, "ok") if d > 0 else (0.0, "blocked_behind"))
    n = len(r.sent)
    recs = [r.cycle(teleop={"speed_ratio": 1.0}) for _ in range(5)]
    assert all(x.get("direction_cut") == "blocked_behind" for x in recs if x["target_mps"] != 0.0)
    assert any(x["target_mps"] != 0.0 for x in recs) and r.nonzero_sent(n) == []


def test_a_rear_that_allows_the_bound_lets_forward_go():
    r = armed_rig(floor=lambda c, d, v: (1.0, "ok") if d > 0 else (0.05, "near_behind"))
    n = len(r.sent)
    for _ in range(10):
        r.cycle(teleop={"speed_ratio": 1.0})
    assert r.nonzero_sent(n)


def test_no_truth_is_read(monkeypatch):
    from forklift_core.safety import truth_access

    def boom(*a, **k):
        raise AssertionError("truth read")

    monkeypatch.setattr(truth_access.SafetySpeed, "exact", classmethod(lambda cls, v: boom()))
    r = armed_rig()
    r.run(20, teleop={"speed_ratio": 1.0})
    assert r.nonzero_sent()


def test_an_operator_stop_in_the_cycle_the_mcu_stopping_is_first_seen_disarms_at_once():
    """Codex impl P1-3."""
    r = armed_rig()
    r.run(5, teleop={"speed_ratio": 1.0})
    r.mcu.deaf = True
    sent = {"done": False}

    def stop_when_stopping(rig):
        if not sent["done"] and rig.mcu.mode == Mode.STOPPING:
            rig.channels.message("c1", "op", {"t": "stop", "session_epoch": EPOCH, "request_id": "s3"})
            sent["done"] = True

    for _ in range(30):
        n = len(r.sent)
        rec = r.cycle(before=stop_when_stopping)
        if sent["done"]:
            break
    assert sent["done"] and r.bridge.mcu_wait is None and rec.get("disarm")
    assert any(isinstance(m, Disarm) for _t, m in r.sent[n:])


def test_a_steering_only_command_never_outlives_the_teleop_deadline():
    """Codex impl P2-5: a presteer at zero wheel speed is still an active teleop command."""
    r = armed_rig()
    r.run(5, teleop={"speed_ratio": 0.0, "curvature_ratio": 1.0})
    last = [m for _t, m in r.sent if isinstance(m, Drive)][-1]
    assert last.left_wheel_rad_s == 0.0 and abs(last.steering_rad) > 0.0
    deadline_ns = int(r.arbiter.teleop["deadline"] * 1e9)
    expiry = r.host.drive_traffic()["written"][-1]["expires_ns"]
    assert expiry <= deadline_ns


def test_the_direction_check_covers_the_curvature_the_steering_holds():
    """Codex impl P1-2: the rear is closed only on near-straight arcs; the target curvature is large but the
    steering still holds a small one -- the check must see it."""
    def floor(curvs, d, v):
        if d < 0 and any(abs(k) < 0.1 for k in curvs):
            return 0.0, "blocked_behind_straight"
        return 1.0, "ok"

    r = armed_rig(floor=floor)
    n = len(r.sent)
    recs = [r.cycle(teleop={"speed_ratio": 1.0, "curvature_ratio": 1.0}) for _ in range(5)]
    assert r.nonzero_sent(n) == [] and any(x.get("direction_cut") for x in recs)


def test_states_the_host_rejects_never_reach_the_speed():
    """Codex impl P2-7."""
    from dataclasses import replace

    r = armed_rig()
    last = r.bridge.samples[-1]
    bad = r.mcu.state()
    r.host.on_bytes(encode(replace(bad, boot_id=0), 1), r.now + 1)
    r.host.on_bytes(encode(replace(bad, mcu_time_us=last.state.mcu_time_us - 50_000), 1), r.now + 2)
    assert r.bridge.samples[-1] is last
    good = replace(bad, mcu_time_us=last.state.mcu_time_us + 10_000)
    r.host.on_bytes(encode(good, 1), r.now + 3)
    assert r.bridge.samples[-1].state == good  # an accepted one does arrive


@pytest.mark.parametrize("offset_ns,case", [(-1, "preempt"), (0, "hold"), (1, "hold"), (999_999, "hold"),
                                            (5_000_000, "follow")])
def test_a_teleop_command_steers_only_before_its_deadline(offset_ns, case):
    """Codex re-review P2: past the deadline the arbiter preempts; at it, or with less life than the link's 1 ms
    validity step, the old steering target is not renewed -- the steering is held where it is measured; with life
    left, it is followed and the frame's expiry stays within the deadline."""
    r = armed_rig()
    r.run(5, teleop={"speed_ratio": 0.0, "curvature_ratio": 1.0})
    a = r.arbiter
    nxt = r.now + 20 * MS  # the next cycle's time
    a.teleop["deadline"] = (nxt + offset_ns) / 1e9
    n = len(r.sent)
    rec = r.cycle(echo=True, teleop=None)
    drives = [m for _t, m in r.sent[n:] if isinstance(m, Drive)]
    if case == "preempt":
        # the arbiter preempts the missed deadline (its latch goes again at once on the confirmed stand): disarmed,
        # the MCU disarmed, no frame renews the old target
        assert not rec["armed"] and any(e.get("event") == "disarm" or e.get("reason") for e in a.events[-4:])
        assert any(isinstance(m, Disarm) for _t, m in r.sent[n:]) and drives == []
    elif case == "hold":
        assert r.bridge.holding and drives and rec.get("stop") is None
        assert drives[-1].steering_rad == pytest.approx(r.bridge.reading.steering_rad, abs=1e-9)
    else:
        assert not r.bridge.holding and drives and rec.get("stop") is None
        assert r.host.drive_traffic()["written"][-1]["expires_ns"] <= nxt + offset_ns


def test_the_final_output_after_a_stop_is_a_preempt_zero():
    r = armed_rig()
    r.mcu.extra_rate = 1.0
    for _ in range(40):
        rec = r.cycle(teleop={"speed_ratio": 1.0})
        if rec.get("stop") == "overspeed":
            break
    out = rec["_out"]
    assert (out.mode, out.stop_kind, out.source, out.speed_mps, out.armed) == ("STOPPED", "preempt", "zero", 0.0,
                                                                                False)
    assert rec["mode"] == "STOPPED" and rec["armed"] is False


# --- the DISARM ping and the round-trip anchors (lead 2026-11-12) ----------------------------------------------------

def _frames_after(r, n):
    return [type(m).__name__ for _t, m in r.sent[n:]]


def test_no_ping_disarm_goes_out_after_the_arm_and_the_arm_is_not_revoked():
    """A ping queued but not yet written when the operator arms: it goes out ahead of the ARM (harmless on a disarmed
    MCU), never after it; no ping is made while the ARM is pending or armed; the ARM is not revoked."""
    r = Rig()
    r.to_teleop()
    for _ in range(40):  # wait until a ping is due, then let it queue without writing it
        if r.bridge.last_ping_ns is not None and r.now + 20 * MS - r.bridge.last_ping_ns >= r.bridge.PING_NS:
            break
        r.cycle()
    n = len(r.sent)
    rec = r.cycle(transmit=False)
    assert rec.get("ping") and [o.kind for o in r.host._outbox] == ["DISARM"]  # queued, unwritten
    r.say("arm")
    recs = [r.cycle() for _ in range(30)]
    frames = _frames_after(r, n)
    assert "Arm" in frames
    first_arm = frames.index("Arm")
    assert "Disarm" in frames[:first_arm] and "Disarm" not in frames[first_arm:], frames
    assert not any(x.get("ping") for x in recs if x["phase"] != "idle")
    assert r.bridge.phase == rt.READY and r.host.ready and r.mcu.arms == 1


def test_a_ping_in_the_cycle_before_the_arm_does_not_revoke_it():
    r = Rig()
    r.to_teleop()
    for _ in range(40):
        rec = r.cycle()
        if rec.get("ping"):
            break
    assert rec.get("ping")
    r.say("arm")  # the next cycle arms right after the ping went out
    r.run(25)
    assert r.bridge.phase == rt.READY and r.host.ready
    assert not any(str(x.get("stop", "")).startswith("mcu_") for x in r.recs)


def test_while_armed_the_drives_keep_the_anchor_fresh_without_pings():
    r = armed_rig()
    recs = [r.cycle(teleop={"speed_ratio": 0.5}) for _ in range(150)]  # 3 s
    assert not any(x.get("ping") for x in recs)
    delays = [x["delay_s"] for x in recs]
    assert all(d is not None and d < 0.03 for d in delays), max(d or 1 for d in delays)


def test_no_anchor_for_two_seconds_invalidates_the_speed_and_stops():
    """The MCU applies the DRIVEs but stops acknowledging them: after 2 s no anchor is recent enough -- no delay
    bound, no safety speed, a STOP while driving."""
    r = armed_rig()
    r.run(5, teleop={"speed_ratio": 0.5})
    r.mcu.ack_frozen = True
    t0 = r.now
    stopped_at = None
    for _ in range(200):
        rec = r.cycle(teleop={"speed_ratio": 0.5})
        if rec.get("stop"):
            stopped_at = r.now
            break
    assert rec["stop"] == "speed_invalid" and rec["speed_invalid"] == "no_delay_bound"
    assert 2.0e9 <= stopped_at - t0 <= 2.2e9
    assert any(isinstance(m, Disarm) for _t, m in r.sent[-5:])
