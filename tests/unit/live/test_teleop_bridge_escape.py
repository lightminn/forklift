"""The S6 bridge's wall escape (docs/design/2026-11-12-standing-direction-rule.md 4-5): a standing truck whose
forward request is held only by the unasked rear motion is offered an escape; the operator's approval lets it drive
straight, in the requested direction only, at 0.05 m/s at most, until the measured end formula, 4 s or any of the
immediate stops. Against the cycle rig's real McuLinkHost, arbiter and link, with stand-in MCUs."""

import math

import pytest
from test_teleop_bridge_cycle import CAP, VEH, FakeMcu, Rig

from forklift_core.live import real_teleop as rt
from forklift_core.mcu_link.messages import Disarm, Drive, FaultFlag, Mode

WALL = lambda c, d, v: (1.0, "ok") if d > 0 else (0.0, "blocked_behind")  # noqa: E731
M_PER_COUNT = 2 * math.pi * VEH.wheel_radius_m[0] / VEH.counts_per_wheel_rev[0]
MS = 1_000_000


class WheelMcu(FakeMcu):
    """Per-wheel overrides (m/s, wheel forward positive) applied whatever the command -- a push, a slip, a PI
    reversing one wheel."""

    def __init__(self):
        super().__init__()
        self.override = [None, None]

    def advance(self, dt_us):
        super().advance(dt_us)
        for i in (0, 1):
            if self.override[i] is not None:
                rad = self.override[i] / VEH.wheel_radius_m[i]
                # undo the base integration of this step, then integrate the override
                self.counts[i] -= self.rate[i] * dt_us / 1e6 * VEH.counts_per_wheel_rev[i] / (2 * math.pi)
                self.rate[i] = rad
                self.counts[i] += rad * dt_us / 1e6 * VEH.counts_per_wheel_rev[i] / (2 * math.pi)


class InertialMcu(FakeMcu):
    """A plant with inertia (design 4.3's end formula, 5th ed.): each wheel approaches its target at ``accel``
    (premise A1's wheel acceleration), brakes at ``brake`` (b_min); ``runaway`` keeps accelerating at ``accel``
    past the command while a late stop is pending; ``late_s`` applies a zero DRIVE or a DISARM
    that late; ``drop_zero`` never receives a zero DRIVE (the expiry path)."""

    def __init__(self, *, accel, brake=rt.ESCAPE_BRAKE_MIN_MPS2, late_s=0.0, drop_zero=False, runaway=False):
        super().__init__()
        self.accel, self.brake, self.late_s = accel, brake, late_s
        self.drop_zero, self.runaway = drop_zero, runaway
        self.v = [0.0, 0.0]
        self.faulty = False  # late_s / drop_zero / runaway act only once set (from the escape's start)
        self.pending = []  # (apply at t_us, message, seq)

    def receive(self, msg, seq):
        stop = (isinstance(msg, Disarm) and self.mode in (Mode.ARMED, Mode.STOPPING)) or (isinstance(msg, Drive) and msg.left_wheel_rad_s == 0.0
                                           and msg.right_wheel_rad_s == 0.0 and self.mode == Mode.ARMED
                                           and self.zero_seen)
        if stop and self.faulty and self.drop_zero and isinstance(msg, Drive):
            return
        if stop and self.faulty and self.late_s > 0.0:
            self.pending.append((self.t_us + int(self.late_s * 1e6), msg, seq))
            return
        super().receive(msg, seq)

    def advance(self, dt_us):
        due = [p for p in self.pending if p[0] <= self.t_us + dt_us]
        self.pending = [p for p in self.pending if p[0] > self.t_us + dt_us]
        for _t, msg, seq in due:
            FakeMcu.receive(self, msg, seq)
        super().advance(dt_us)
        dt = dt_us / 1e6
        for i in (0, 1):
            target = self.rate[i] * VEH.wheel_radius_m[i]
            if self.runaway and self.pending and target != 0.0:
                self.v[i] += math.copysign(self.accel * dt, target)
            elif abs(target) > abs(self.v[i]) or (target != 0.0 and math.copysign(1, target) != math.copysign(1, self.v[i])):
                self.v[i] = min(target, self.v[i] + self.accel * dt) if target > self.v[i] else \
                    max(target, self.v[i] - self.accel * dt)
            else:
                step = self.brake * dt
                self.v[i] = target if abs(self.v[i] - target) <= step else self.v[i] - math.copysign(step, self.v[i] - target)
            # the base class integrated the target; replace it with the plant's speed
            self.counts[i] -= self.rate[i] * dt * VEH.counts_per_wheel_rev[i] / (2 * math.pi)
            self.rate[i] = self.v[i] / VEH.wheel_radius_m[i]
            self.counts[i] += self.rate[i] * dt * VEH.counts_per_wheel_rev[i] / (2 * math.pi)


def wall_rig(mcu=None, floor=WALL):
    r = Rig(floor=floor)
    if mcu is not None:
        r.mcu = mcu
    r.to_teleop()
    r.arm()
    assert r.bridge.phase == rt.READY and r.host.ready, (r.bridge.phase, r.recs[-1])
    return r


def held(r, n=20, **teleop):
    teleop.setdefault("speed_ratio", 1.0)
    for _ in range(n):
        r.cycle(teleop=teleop)
    return r.bridge.offer


def approve(r, offer, **teleop):
    teleop.setdefault("speed_ratio", 1.0)
    r.say("escape_approve", arm_gen=r.arbiter.arm_gen, offer_id=offer["offer_id"])
    rec = r.cycle(teleop=teleop)
    return rec, [m for m in r._replies() if m.get("of") == "escape_approve"]


def escaping(r, **teleop):
    offer = held(r)
    assert offer is not None and offer["direction"] == 1 and offer["blocked"] == -1, r.recs[-1]
    rec, [reply] = approve(r, offer, **teleop)
    assert reply["t"] == "ack", reply
    assert r.bridge.escape is not None, rec
    return offer


def counts(r):
    return [r.mcu.counts[0], r.mcu.counts[1]]


def stand_after(r, n=150, **teleop):
    teleop.setdefault("speed_ratio", 1.0)
    for _ in range(n):
        r.cycle(teleop=teleop)
    return counts(r)


def travel_m(c0, c1):
    return max(abs(c1[i] - c0[i]) for i in (0, 1)) * M_PER_COUNT


def stopped_by(r, why):
    stops = [x.get("stop") for x in r.recs if x.get("stop")]
    return any(s == f"escape:{why}" for s in stops), stops


# --- the offer (4.1) ----------------------------------------------------------------------------------------------

def test_a_wall_behind_holds_forward_and_offers_the_escape():
    r = wall_rig()
    n = len(r.sent)
    offer = held(r)
    assert r.nonzero_sent(n) == []
    assert offer is not None and offer["direction"] == 1 and offer["blocked"] == -1
    assert offer["reason"] == "blocked_behind" and isinstance(offer["offer_id"], int)
    assert offer["control_gen"] == r.arbiter.control_gen and offer["arm_gen"] == r.arbiter.arm_gen
    assert held(r, 5)["offer_id"] == offer["offer_id"]  # one hold, one offer


def test_no_offer_when_the_requested_direction_is_itself_blocked():
    r = wall_rig(floor=lambda c, d, v: (0.0, "blocked"))
    assert held(r) is None


def test_no_offer_without_a_request_or_the_deadman():
    r = wall_rig()
    assert held(r, speed_ratio=0.0) is None
    assert held(r, deadman=False) is None


def test_no_offer_while_the_truck_is_not_standing():
    mcu = WheelMcu()
    r = wall_rig(mcu)
    mcu.override = [0.002, 0.002]  # a slow creep: counts move, no fresh stand
    assert held(r) is None


def test_no_offer_while_a_nonzero_command_may_still_act():
    r = wall_rig()
    held(r)
    r.bridge.last_cmd = (0.03, r.now + 10**9)
    r.cycle(teleop={"speed_ratio": 1.0})
    assert r.bridge.offer is None


def test_no_offer_with_the_steering_turned():
    r = wall_rig()
    r.mcu.q = 0.05  # past straight_threshold_rad (0.01)
    r.bridge.q_cmd = 0.05
    for _ in range(20):
        r.cycle(teleop={"speed_ratio": 1.0})
        r.mcu.q = 0.05
    assert r.bridge.offer is None


def test_the_offer_goes_when_the_request_goes():
    r = wall_rig()
    assert held(r) is not None
    r.cycle(teleop={"speed_ratio": 0.0})
    assert r.bridge.offer is None


# --- the approval (4.2) -------------------------------------------------------------------------------------------

def test_an_unknown_offer_is_refused():
    r = wall_rig()
    offer = held(r)
    _rec, [reply] = approve(r, {**offer, "offer_id": offer["offer_id"] + 1})
    assert reply["t"] == "reject" and reply["reason"] == "escape_no_offer" and r.bridge.escape is None


def test_an_offer_from_an_older_generation_is_refused():
    r = wall_rig()
    offer = held(r)
    r.channels.message("c1", "op", {"t": "stop", "session_epoch": 5, "request_id": "s1"})
    r.run(30)
    r.say("rearm")
    r.run(30)
    r.say("mode_request", to="TELEOP")
    r.run(3)
    r.arm()
    _rec, [reply] = approve(r, offer)
    assert reply["t"] == "reject" and r.bridge.escape is None


def test_approval_while_escaping_is_refused_and_one_approval_is_one_escape():
    r = wall_rig()
    offer = escaping(r)
    _rec, [again] = approve(r, offer)
    assert again["t"] == "reject" and again["reason"] == "escape_active"
    while r.bridge.escape is not None:
        r.cycle(teleop={"speed_ratio": 1.0})
    _rec, [late] = approve(r, offer)
    assert late["t"] == "reject" and late["reason"] == "escape_no_offer"
    nxt = held(r, 40)
    assert nxt is not None and nxt["offer_id"] != offer["offer_id"]  # a new stand: a new offer, a new approval
    _rec, [ok] = approve(r, nxt)
    assert ok["t"] == "ack"


def test_a_request_reversed_at_the_approval_does_not_start():
    r = wall_rig()
    offer = held(r)
    n = len(r.sent)
    _rec, [reply] = approve(r, offer, speed_ratio=-1.0)
    r.cycle(teleop={"speed_ratio": -1.0})
    assert r.bridge.escape is None and r.nonzero_sent(n) == [] or all(
        m.left_wheel_rad_s < 0 for m in r.nonzero_sent(n))  # never forward on a reversed request


# --- while escaping (4.3) -----------------------------------------------------------------------------------------

def test_the_escape_drives_forward_straight_and_slow_then_ends_on_the_measured_formula():
    r = wall_rig()
    c0 = counts(r)
    n = len(r.sent)
    escaping(r, curvature_ratio=1.0)
    recs = []
    while r.bridge.escape is not None:
        recs.append(r.cycle(teleop={"speed_ratio": 1.0, "curvature_ratio": 1.0}))
        assert len(recs) < 300
    drives = r.nonzero_sent(n)
    assert drives and all(m.left_wheel_rad_s > 0 and m.right_wheel_rad_s > 0 for m in drives)
    assert max(max(m.left_wheel_rad_s, m.right_wheel_rad_s) for m in drives) * VEH.wheel_radius_m[0] \
        <= rt.ESCAPE_MPS + 1e-9
    assert {m.steering_rad for m in drives} == {r.bridge.escape_log[-1]["steering_rad"]}  # the curvature ignored
    end = r.bridge.escape_log[-1]
    assert end["event"] == "end" and end["reason"] == "distance"
    assert all(x["escape"]["expires_ns"] <= x["escape"]["deadline_ns"] for x in recs if x.get("escape", {}).get("expires_ns"))
    c1 = stand_after(r)
    assert 0.03 < travel_m(c0, c1) <= rt.ESCAPE_DISTANCE_M
    assert r.bridge.escape is None and r.arbiter.mode == "TELEOP"  # a normal end: no STOP


def test_the_normal_end_drops_queued_drives_sends_a_zero_and_returns_to_the_general_rule():
    r = wall_rig()
    escaping(r)
    while r.bridge.escape is not None:
        r.cycle(teleop={"speed_ratio": 1.0})
    end_at = len(r.sent)
    r.run(10, teleop={"speed_ratio": 1.0})
    after = [m for _t, m in r.sent[end_at:] if isinstance(m, Drive)]
    assert after and r.nonzero_sent(end_at) == []
    assert any(x.get("direction_cut") == "blocked_behind" for x in r.recs[-10:])


def test_a_slow_request_ends_on_time():
    r = wall_rig()
    escaping(r, speed_ratio=0.1)
    t0 = r.now
    while r.bridge.escape is not None:
        r.cycle(teleop={"speed_ratio": 0.1})
        assert r.now - t0 < 5_000 * MS
    assert r.bridge.escape_log[-1]["reason"] == "time"
    assert r.now - t0 <= rt.ESCAPE_TIME_S * 1e9 + 40 * MS


@pytest.mark.parametrize("change, why", [
    ({"deadman": False}, "deadman"),
    ({"speed_ratio": 0.0}, "request"),
    ({"speed_ratio": -1.0}, "request"),
])
def test_an_operator_change_is_an_immediate_stop(change, why):
    r = wall_rig()
    escaping(r)
    r.run(5, teleop={"speed_ratio": 1.0})
    n = len(r.sent)
    r.cycle(teleop={"speed_ratio": 1.0, **change})
    assert stopped_by(r, why)[0], stopped_by(r, why)[1]
    assert r.bridge.escape is None and r.nonzero_sent(n) == [] and r.arbiter.mode == "STOPPED"
    assert any(isinstance(m, Disarm) for _t, m in r.sent[n:])


@pytest.mark.parametrize("override, why", [
    ([0.03, -0.03], "wheel_reverse"),  # the two wheels opposite, the mean zero
    ([-0.005, -0.005], "wheel_reverse"),  # a reverse under any speed threshold (21 counts / 10 ms per wheel)
    ([0.035, 0.05], "yaw"),
    ([0.08, 0.08], "wheel_overspeed"),
])
def test_a_measured_motion_out_of_the_escape_is_an_immediate_stop(override, why):
    mcu = WheelMcu()
    r = wall_rig(mcu)
    escaping(r)
    r.run(3, teleop={"speed_ratio": 1.0})
    mcu.override = override
    for _ in range(5):
        r.cycle(teleop={"speed_ratio": 1.0})
        if r.bridge.escape is None:
            break
    assert stopped_by(r, why)[0], stopped_by(r, why)[1]
    assert r.arbiter.mode == "STOPPED"


def test_an_invalid_speed_is_an_immediate_stop():
    r = wall_rig()
    escaping(r)
    r.run(3, teleop={"speed_ratio": 1.0})
    r.mcu.faults = FaultFlag.ENCODER_GAP
    r.cycle(teleop={"speed_ratio": 1.0})
    assert r.bridge.escape is None and r.arbiter.mode == "STOPPED"


def test_the_steering_leaving_straight_is_an_immediate_stop():
    r = wall_rig()
    escaping(r)
    r.run(3, teleop={"speed_ratio": 1.0})
    for _ in range(3):
        r.mcu.q = 0.03
        r.cycle(teleop={"speed_ratio": 1.0})
        if r.bridge.escape is None:
            break
    assert stopped_by(r, "steering")[0], stopped_by(r, "steering")[1]


def test_an_operator_stop_ends_the_escape():
    r = wall_rig()
    escaping(r)
    r.run(3, teleop={"speed_ratio": 1.0})
    r.channels.message("c1", "op", {"t": "stop", "session_epoch": 5, "request_id": "s9"})
    n = len(r.sent)
    r.cycle(teleop={"speed_ratio": 1.0})
    assert r.bridge.escape is None and r.nonzero_sent(n) == []
    assert r.bridge.escape_log[-1]["event"] == "end" and r.bridge.escape_log[-1]["reason"] == "generation"


def test_the_requested_direction_permit_still_applies():
    state = {"front": 1.0}
    r = wall_rig(floor=lambda c, d, v: (state["front"], "ok") if d > 0 else (0.0, "blocked_behind"))
    escaping(r)
    r.run(3, teleop={"speed_ratio": 1.0})
    state["front"] = 0.0
    n = len(r.sent)
    r.cycle(teleop={"speed_ratio": 1.0})
    assert r.bridge.escape is None and r.nonzero_sent(n) == []
    assert r.bridge.escape_log[-1]["reason"] == "permit"


# --- the end formula (4.3, 5th ed.) --------------------------------------------------------------------------------

def _reading(signed_l, signed_r, e_l, e_r):
    from forklift_core.safety.truth_access import SafetySpeed

    v = (signed_l, signed_r)
    e = (e_l, e_r)
    bound = max(abs(v[0]) + e[0], abs(v[1]) + e[1])
    s = (v[0] + v[1]) / 2
    return rt.EncoderReading(SafetySpeed(s, bound, (1, -1) if abs(s) < max(e) else ((1,) if s > 0 else (-1,))),
                             0.0, None, dt_s=0.02, age_s=0.03, steering_rad=0.0, wheel_mps=v, wheel_err=e)


@pytest.mark.parametrize("v, e", [
    ((0.04, 0.04), (0.01, 0.01)),
    ((-0.04, -0.04), (0.01, 0.01)),  # a reverse escape: |v|, never the signed value
    ((0.03, 0.045), (0.02, 0.005)),  # unequal per-wheel errors: the max of |v_w| + e_w, not max|v| + max e
])
def test_the_end_formula_speed_is_the_max_over_wheels_of_abs_v_plus_e(v, e):
    reading = _reading(v[0], v[1], e[0], e[1])
    p = rt.load_bridge_params(VEH and __import__("pathlib").Path(__file__).resolve().parents[3]
                              / "config" / "real" / "teleop_bridge.yaml", VEH)
    want_v = max(abs(v[0]) + e[0], abs(v[1]) + e[1])
    T = 0.03 + 0.02 + rt.OUT_LIFE_NS / 1e9
    A = p.wheel_accel_mps2
    b = rt.ESCAPE_BRAKE_MIN_MPS2
    want = 0.1 + want_v * T + 0.5 * A * T * T + (want_v + A * T) ** 2 / (2 * b)
    assert rt.escape_stop_reach_m(0.1, reading, p, cycle_s=0.02) == pytest.approx(want)


@pytest.mark.parametrize("mcu_kw", [
    {},  # accelerating at A1 toward the command at every reading
    {"late_s": 0.15},  # every stop takes effect only after the whole T
    {"late_s": 0.15, "runaway": True},  # ... while the wheels keep accelerating past the command
    {"drop_zero": True},  # the end's zero DRIVE never arrives: the last command runs to its expiry
])
def test_the_final_travel_with_inertia_stays_within_the_budget(mcu_kw):
    p = rt.load_bridge_params(__import__("pathlib").Path(__file__).resolve().parents[3]
                              / "config" / "real" / "teleop_bridge.yaml", VEH)
    mcu = InertialMcu(accel=p.wheel_accel_mps2, **mcu_kw)
    r = wall_rig(mcu)
    c0 = counts(r)
    escaping(r)
    mcu.faulty = True
    for _ in range(300):
        r.cycle(teleop={"speed_ratio": 1.0})
        if r.bridge.escape is None:
            break
    assert r.bridge.escape_log[-1]["reason"] == "distance"
    # the operator lets go at the end: the escape's own stopping path (holding on hands over to the general rule,
    # next test)
    c1 = stand_after(r, 200, deadman=False)
    assert all(v == 0.0 for v in mcu.v), mcu.v
    assert travel_m(c0, c1) <= rt.ESCAPE_DISTANCE_M, (travel_m(c0, c1), r.bridge.escape_log)


def test_holding_on_past_the_end_is_the_general_rule():
    """Design 4.3: after a normal end the general rule decides. A truck still rolling forward reads forward only
    (|signed| > e): the unasked rear motion is no longer possible by the measurement, so the general rule lets the
    forward request drive under the normal cap; a truck that stood again is held by the rear again."""
    p = rt.load_bridge_params(__import__("pathlib").Path(__file__).resolve().parents[3]
                              / "config" / "real" / "teleop_bridge.yaml", VEH)
    mcu = InertialMcu(accel=p.wheel_accel_mps2)
    r = wall_rig(mcu)
    escaping(r)
    while r.bridge.escape is not None:
        r.cycle(teleop={"speed_ratio": 1.0})
    recs = [r.cycle(teleop={"speed_ratio": 1.0}) for _ in range(20)]
    assert all("escape" not in x for x in recs)
    assert all(x["speed"]["dirs"] == [1] for x in recs if x.get("applied_mps"))  # driving only while forward-only
    assert any(x.get("applied_mps", 0.0) > rt.ESCAPE_MPS for x in recs)


# --- the record and the console (4.3) -----------------------------------------------------------------------------

def test_every_step_is_logged_and_the_views_carry_it():
    r = wall_rig()
    offer = held(r)
    view = r.bridge.offer_view()
    assert view == {"offer_id": offer["offer_id"], "direction": 1, "blocked": -1, "reason": "blocked_behind"}
    _rec, [reply] = approve(r, {**offer, "offer_id": offer["offer_id"] + 5})
    _rec, [reply] = approve(r, offer)
    assert r.bridge.escape_view()["offer_id"] == offer["offer_id"]
    while r.bridge.escape is not None:
        r.cycle(teleop={"speed_ratio": 1.0})
    events = [e["event"] for e in r.bridge.escape_log]
    assert events[:4] == ["offer", "refused", "approved", "start"] and events[-1] == "end"
    end = r.bridge.escape_log[-1]
    assert set(end) >= {"reason", "travel_m", "elapsed_s", "offer_id"} and len(end["travel_m"]) == 2
    assert r.bridge.escape_view() is None
    assert any("escape" in x for x in r.recs)


def test_the_bridge_puts_the_offer_and_the_escape_on_the_state_message():
    import sys

    sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[3] / "tools"))
    import teleop_bridge as tb

    class _Silent:  # a device that never answers
        def receive(self, host, now_ns):
            return host.tick(now_ns)

        def transmit(self, host, now_ns):
            while (f := host.next_frame(now_ns)) is not None:
                host.frame_written(f, now_ns)
            return []

    b = tb.Bridge(_Silent(), confirmed=set())
    assert b.arbiter.escape is not None  # injected by the bridge
    t0 = 1_000_000_000_000
    states = []
    for k in range(10):
        b.cycle(t0 + k * 20_000_000)
        states += [m for _c, m in b.channels.take_out() if m.get("t") == "state"]
    assert states and all("escape_offer" in s and "escape" in s for s in states)
    assert states[-1]["escape_offer"] is None and states[-1]["escape"] is None
