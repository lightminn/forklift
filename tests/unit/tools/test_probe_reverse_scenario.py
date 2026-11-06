"""The probe's reverse scenario (docs/plans/2026-11-06-unconfirmed-validation.md 2.1) against a fake vehicle that runs
the stroke rule and the re-match: the pressing rules (full, half, chatter), the stand and re-match after each stroke,
the stop at a confirmation (on) or at the strokes' end (the shadow), the hold, the mission after the confirmation and
the session's end; a check that ended leaves the session to end by itself; the check off is the hold scenario."""

import asyncio
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import live_console_probe as lcp  # noqa: E402

STEP_M, STOP_M = 0.01, 0.069
REAL_SLEEP = asyncio.sleep


class FakeVehicle:
    """The vehicle as the probe sees it: each deadman teleop moves the truck STEP_M back; a released truck stands at
    the next advance; a stroke closes STOP_M before its end; a stand past 0.5 m starts a re-match lasting three
    advances, which confirms (on) when ``confirm_at`` re-matches have run."""

    def __init__(self, shadow=False, start="unconfirmed", confirm_at=1, off=False, expire=None):
        self.shadow, self.confirm_at = shadow, confirm_at
        self.expire, self.since_command, self.armed, self.disarmed = expire, 0, False, 0
        self.report, self.calls, self.closed_code, self.lease_id, self.arm_gen = {}, [], None, None, 0
        self.targets = {"targets_revision": 3}
        self.status, self.stroke, self.left, self.used, self.s = start, "ready", 1.0, 0.0, 0.0
        self.d, self.moving, self.rematch, self.rematches, self.stood_at = 0.0, False, 0, 0, None
        self.mission_at, self.off, self.close_at = None, off, None
        self.state = None
        self._publish()

    def _publish(self):
        st = self.status
        pc = {"status": f"shadow:{st}" if self.shadow else st,
              "stroke": {"state": self.stroke, "left_m": min(1.0, 3.0 - self.used) - self.s,
                         "total_left_m": 3.0 - self.used - self.s},
              "rematch": "scans" if self.shadow and self.rematch else None}
        self.state = {"pose": {"x_m": 0.0}, "rear_axle_speed_mps": -0.1 if self.moving else 0.0,
                      "last_result": self.state.get("last_result") if self.state else None,
                      **({} if self.off else {"pose_check": pc})}

    def advance(self):
        self.since_command += 1
        if self.expire is not None and self.armed and self.since_command > self.expire:
            self.armed = False  # the arbiter's command age: no command, disarmed
            self.disarmed += 1
        if self.close_at is not None:
            self.close_at -= 1
            if self.close_at <= 0:
                self.closed_code = 1000
        if self.mission_at is not None:
            self.mission_at -= 1
            if self.mission_at <= 0:
                self.state["last_result"] = {"seq": 1, "mission_id": "m1", "outcome": "done"}
                self.mission_at = None
        if self.rematch:
            self.rematch -= 1
            if not self.rematch:
                self.rematches += 1
                if not self.shadow:
                    self.status = "confirmed" if self.rematches >= self.confirm_at else "unconfirmed"
        if not self.moving and self.stood_at is None:
            self.stood_at = self.d
            if self.stroke == "closed":
                self.used += self.s
                self.s = 0.0
                self.stroke = "spent" if 3.0 - self.used <= STOP_M else "ready"
            if self.d >= 0.5 and self.status in ("unconfirmed", "confirmed"):
                self.rematch = 3
                if not self.shadow:
                    self.status = "checking"
        self._publish()

    def teleop_(self, speed, deadman):
        self.since_command = 0
        if deadman and self.expire is not None and not self.armed:
            return  # not_armed: nothing moves
        if deadman and speed < 0 and self.stroke in ("ready", "active"):
            self.stroke = "active"
            self.s += STEP_M
            self.d += STEP_M
            self.moving, self.stood_at = True, None
            if min(1.0, 3.0 - self.used) - self.s <= STOP_M:
                self.stroke = "closed"
        elif not deadman:
            if self.stroke == "active":
                self.stroke = "closed"
            self.moving = False
        self._publish()

    # --- the probe's interface ---
    async def open(self):
        self.calls.append("open")

    async def until(self, pred, timeout, hz=20, label=""):
        for _ in range(2000):
            if self.state is not None and pred(self.state):
                return True
            if self.closed_code is not None:
                return False
            self.advance()
            await REAL_SLEEP(0)  # the probe's own tasks run, as on the event loop
        return False

    async def request(self, kind, timeout=5.0, **fields):
        self.calls.append(kind if kind != "mode_request" else f"mode:{fields['to']}")
        if kind == "mode_request" and fields["to"] != "TELEOP":
            self.armed = False  # a mode change disarms; nothing to expire
        if kind == "lease_request":
            return {"t": "ack", "lease_id": "L"}
        if kind == "arm":
            self.armed, self.since_command = True, 0
            return {"t": "ack", "arm_gen": 1}
        if kind == "mission_submit":
            assert fields["targets_revision"] == 3 and fields["pallet_id"] == "P-1"
            self.mission_at = 5
            return {"t": "ack", "mission_id": "m1"}
        return {"t": "ack"}

    async def request_until_standing(self, kind, tries=20, **fields):
        self.calls.append(kind)
        self.closed_code = 1000
        return {"t": "ack"}

    async def teleop(self, speed, curvature, deadman=True, mode="direct", lift_dir=0):
        assert curvature == 0.0 and speed in (lcp.REVERSE_RATIO, 0.0)
        self.teleop_(speed, deadman)

    def standing(self, s):
        return s.get("rear_axle_speed_mps") == 0.0


def _run(v, kind="full", mission=False):
    args = SimpleNamespace(reverse_kind=kind, mission_after_confirm=mission, reverse_wait_s=30.0, stroke_wall_s=30.0,
                           hold_sim_s=0.05, live_factor=1.0, cycle="P-1:D-1,P-1:D-0", mission_timeout_s=30.0)
    orig = asyncio.sleep

    async def sleep(s):
        v.advance()
        await orig(0)

    lcp.asyncio.sleep = sleep
    try:
        asyncio.run(lcp.reverse_scenario(v, args))
    finally:
        lcp.asyncio.sleep = orig
    return v.report


def test_the_release_rules():
    active = {"state": "active", "left_m": 0.6}
    assert not lcp.release_due("full", 1.0, active)
    assert lcp.release_due("half", 1.0, {**active, "left_m": 0.5})
    assert not lcp.release_due("half", 1.0, {**active, "left_m": 0.51})
    assert lcp.release_due("chatter", 1.0, {**active, "left_m": 0.8})
    assert lcp.release_due("full", 1.0, {"state": "closed", "left_m": 0.07}) and lcp.release_due("full", 1.0, None)


def test_on_a_full_stroke_stands_rematches_confirms_then_holds_runs_the_mission_and_ends():
    v = FakeVehicle()
    rep = _run(v, "full", mission=True)["reverse"]
    [stroke] = rep["strokes"]
    assert rep["check"] == "on" and rep["start_status"] == "unconfirmed" and stroke["released"] == "closed"
    assert stroke["start_left_m"] - stroke["end_left_m"] == pytest.approx(1.0 - STOP_M, abs=0.011)
    assert stroke["rematch"]["started"] and stroke["rematch"]["status"] == "confirmed"
    assert rep["end_status"] == "confirmed" and rep["mission"]["result"]["outcome"] == "done"
    assert v.calls == ["open", "lease_request", "mode:TELEOP", "arm", "mode:PAUSED", "mission_submit", "lease_request",
                       "session_end"]


def test_the_shadow_presses_every_chatter_stroke_to_the_total_and_runs_its_mission():
    v = FakeVehicle(shadow=True)
    rep = _run(v, "chatter", mission=True)["reverse"]
    moved = [s["start_left_m"] - s["end_left_m"] for s in rep["strokes"]]
    assert rep["check"] == "shadow" and len(rep["strokes"]) == lcp.REVERSE_MAX_STROKES
    assert all(m == pytest.approx(0.2, abs=0.011) for m in moved[:-1])
    assert not rep["strokes"][0]["rematch"]["started"] and rep["strokes"][2]["rematch"]["started"]  # past 0.5 m
    assert rep["mission"]["submit"]["t"] == "ack" and v.calls[-1] == "session_end"


def test_on_a_confirmed_start_does_not_reverse():
    v = FakeVehicle(start="confirmed")
    rep = _run(v, "full", mission=True)["reverse"]
    assert rep["strokes"] == [] and "mode:TELEOP" not in v.calls and rep["mission"]["submit"]["t"] == "ack"


def test_an_ended_check_leaves_the_session_to_end_by_itself():
    v = FakeVehicle(start="ended:pose_twin_preferred")
    v.close_at = 10
    rep = _run(v, "full", mission=True)
    assert rep["reverse"]["end_status"].startswith("ended:") and rep["close_code"] == 1000
    assert "session_end" not in v.calls and "mission_submit" not in v.calls


def test_with_the_check_off_it_is_the_hold_scenario():
    v = FakeVehicle(off=True)
    rep = _run(v)
    assert rep["reverse"]["check"] == "off" and "hold" in rep and v.calls[-1] == "session_end"


def test_the_shadow_presses_on_past_a_provisional_end():
    # Codex 2d P2-2: the provisional verdict selects nothing
    v = FakeVehicle(shadow=True, start="ended:pose_slam_disagree")
    rep = _run(v, "full")["reverse"]
    assert len(rep["strokes"]) >= 3 and v.calls[-1] == "session_end"


def test_the_released_command_keeps_the_arm_while_it_stands_and_rematches():
    # Codex 2d P2-1: a command older than 0.30 s disarms: the release is sent on while waiting
    v = FakeVehicle(shadow=True, expire=4)
    rep = _run(v, "half")["reverse"]
    assert v.disarmed == 0 and len(rep["strokes"]) >= 5
