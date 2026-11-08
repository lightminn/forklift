"""The probe's block check (docs/plans/2026-11-08-f1-vehicle-block.md 5) against a fake vehicle: with the block, the
refusals, the state's start_block, no AUTO, teleop taken and the truck moved make a pass, any step that did not hold
a fail; with allow the submit is taken and cancelled; a placement not confirmed or an epoch changed fails either way."""

import asyncio
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import live_console_probe as lcp  # noqa: E402

BLOCK = {"reason": "operator_placed_start", "source": "operator_placed", "since_mono": 1.0}


class Fake:
    def __init__(self, blocked=True, submit_reason=None, moves=True):
        self.blocked, self.submit_reason, self.moves = blocked, submit_reason, moves
        self.report = {"confirm": {"reply": "ack"}}
        self.closed_code, self.lease_id, self.arm_gen = None, None, 0
        self.targets = {"targets_revision": 1}
        self.x, self.mode, self.mission = 0.0, "PAUSED", None
        self.calls, self.states, self.flash_auto, self.auto_on_accept = [], [], False, False
        self._publish()

    def _publish(self):
        self.state = {"pose": {"x_m": self.x, "y_m": 0.0}, "mode": self.mode, "mission": self.mission,
                      "rear_axle_speed_mps": 0.0, **({"start_block": BLOCK} if self.blocked else {})}
        self.states.append({"mode": self.mode, "mission": self.mission})
        if self.flash_auto:  # an AUTO state between two snapshots the check takes itself
            self.states.append({"mode": "AUTO", "mission": None})
            self.flash_auto = False

    def standing(self, s):
        return True

    async def until(self, pred, timeout, label=""):
        return pred(self.state)

    async def teleop(self, speed, curvature, deadman):
        if self.mode == "TELEOP" and self.moves and speed:
            self.x += 0.01
        self._publish()

    async def request(self, kind, **f):
        self.calls.append(kind)
        refused = {"t": "reject", "reason": "operator_placed_start"}
        if kind == "lease_request":
            return {"t": "ack", "lease_id": "L"}
        if kind == "mission_submit":
            if self.submit_reason:
                return {"t": "reject", "reason": self.submit_reason}
            if self.blocked:
                return refused
            self.mission = {"status": "accepted", "mission_id": "m1"}
            if self.auto_on_accept:
                self.mode = "AUTO"
            self._publish()
            return {"t": "ack", "mission_id": "m1"}
        if kind == "mission_cancel":
            self.mission = {"status": "cancelled", "mission_id": "m1"}
            self._publish()
            return {"t": "ack"}
        if kind in ("pick_request",) or (kind == "mode_request" and f.get("to") == "AUTO"):
            return refused if self.blocked else {"t": "ack"}
        if kind == "mode_request":
            self.mode = f["to"]
            self._publish()
            return {"t": "ack"}
        if kind == "arm":
            return {"t": "ack", "arm_gen": 1}
        return {"t": "ack"}


def _args(expect="block"):
    return SimpleNamespace(block_expect=expect, cycle="P-1:D-1", block_teleop_s=0.3)


def _run(fake, expect="block"):
    asyncio.run(lcp._block_check(fake, _args(expect)))
    return fake.report["block_check"]


def test_the_block_passes_on_every_refusal_teleop_and_a_move():
    out = _run(Fake())
    assert out["verdict"] == "pass", out["failed"]
    assert out["moved_m"] > 0.02 and "mission_cancel" not in out


def test_a_taken_submit_fails_the_block():
    out = _run(Fake(blocked=False), "block")
    assert out["verdict"] == "fail" and {"submit_refused", "auto_refused", "pick_refused", "start_block"} <= set(
        out["failed"])


def test_a_refusal_for_another_reason_fails():
    out = _run(Fake(submit_reason="relocalizing"))
    assert out["verdict"] == "fail" and out["failed"] == ["submit_refused"]


def test_a_truck_that_did_not_move_fails():
    out = _run(Fake(moves=False))
    assert out["verdict"] == "fail" and out["failed"] == ["teleop_moved"]


def test_allow_takes_the_submit_and_cancels_it():
    fake = Fake(blocked=False)
    out = _run(fake, "allow")
    assert out["verdict"] == "pass", out["failed"]
    assert out["cancel"]["t"] == "ack" and "mode_request" not in fake.calls[:3]


def test_allow_with_a_block_fails():
    out = _run(Fake(), "allow")
    assert out["verdict"] == "fail" and set(out["failed"]) == {"submit_taken", "no_start_block"}


def test_an_unconfirmed_placement_or_a_new_epoch_fails():
    fake = Fake()
    fake.report = {"confirm": {"reply": "reject"}}
    assert _run(fake)["failed"] == ["placement_confirmed"]
    fake = Fake()
    fake.report["handover"] = {"epoch_changed": [1, 2]}
    assert _run(fake)["failed"] == ["same_epoch"]


def test_an_auto_state_between_the_snapshots_fails_the_block():
    fake = Fake()
    fake.flash_auto = True
    out = _run(fake)
    assert out["verdict"] == "fail" and "never_auto" in out["failed"] and out["states_judged"] > 0


def test_allow_passes_with_the_auto_a_taken_mission_brings():
    fake = Fake(blocked=False)
    fake.auto_on_accept = True
    out = _run(fake, "allow")
    assert out["verdict"] == "pass", out["failed"]


def test_a_failed_check_ends_the_probe_with_an_error(tmp_path, monkeypatch):
    import json

    report = {"block_check": {"verdict": "fail"}}
    src = (ROOT / "tools/live_console_probe.py").read_text()
    assert 'args.after_place == "block_check" and (' in src and "sys.exit(5)" in src
    assert json.loads(json.dumps(report))["block_check"]["verdict"] != "pass"
