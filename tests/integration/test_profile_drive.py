"""G6 end to end (docs/design/2026-11-14-profile-drive-and-m-analysis.md 2.2): a profile drives the MCU model through
the same arbiter, TeleopDrive, floor permit and MCU as the operator's hand, and the record carries request, output and
measurement. The model's plant is synthetic: this proves the path and the gates, not real curvature or delay."""

import io
import json
import sys
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tests" / "integration"))
from test_bringup_tool import mcu_model  # noqa: E402,F401,I001 -- the module-scoped model build
from test_teleop_bridge import Run  # noqa: E402

PROFILE = {"schema": "drive_profile/1", "name": "straight", "load": "empty", "segments": [
    {"label": "presteer", "presteer_kappa_inv_m": 0.0},
    {"label": "still_pre", "hold_s": 0.4},
    {"label": "drive", "speed_mps": 0.10, "kappa_inv_m": 0.0, "distance_m": 0.15},
    {"label": "still_post", "hold_s": 0.3}]}


@pytest.fixture
def run(mcu_model):  # noqa: F811
    made = []

    def make(*extra, **kw):
        r = Run(mcu_model, *extra, **kw)
        r.b.record = io.StringIO()
        made.append(r)
        return r
    yield make
    for r in made:
        r.close()


def rows(r, kind):
    return [x for x in (json.loads(line) for line in r.b.record.sink.getvalue().splitlines()) if x.get("t") == kind]


def start(r):
    r.to_ready()
    r.cycles(60, teleop={"speed_ratio": 0.0, "deadman": False})  # standing, armed, the console sending
    return r.b.profile_start(time.monotonic_ns())


# A loaded host stretches this harness's wall-clock cycles past the live gates (STATE age 50 ms, command age,
# lease): the arbiter or the bridge then stops and disarms the truck for real, and the trial is void with that
# gate's reason (ws1 19203: void:disarmed under load, 5/5 alone). Those voids are the gates working, not the
# profile's logic, so a trial they end is run again from a standstill, at most ATTEMPTS times.
GATE_VOIDS = ("void:disarmed", "void:stopped")
ATTEMPTS = 3


def trial(r, *, teleop, until, expect, stall_once=False):
    """Start a trial and cycle with ``teleop`` until ``until(rec)`` or the end; a gate void (unless expected) is
    retried. Returns (result, the gate voids retried). ``stall_once`` stops cycling for 0.6 s in the first drive
    to make a gate fire (the recovery's own test)."""
    retried = []
    for _ in range(ATTEMPTS):
        if r.b.profile.state != "idle":
            r.cycles(50, teleop={"speed_ratio": 0.0, "deadman": False}, until=lambda rec: r.b.profile.state == "idle")
        if r.b.arbiter.mode == "STOPPED":
            # a gate's preempt locks STOPPED: the operator re-arms from a stand (STOPPED -> PAUSED), as on the truck
            r.cycles(60)
            r.c.request("rearm")
            r.cycles(10)
            assert r.b.arbiter.mode == "PAUSED", r.c.replies[-3:]
        assert start(r) is None
        if stall_once and not retried:
            r.cycles(400, teleop=teleop, until=lambda rec: r.b.profile.moving and r.b.profile.s_m > 0.02)
            time.sleep(0.6)  # nothing cycles: the STATE, the command and the lease age past their gates
        r.cycles(500, teleop=teleop, until=lambda rec: until(rec) or r.b.profile.state == "zeroing")
        result = r.b.profile.result
        if result is None or result.startswith(expect) or not result.startswith(GATE_VOIDS):
            return result, retried
        retried.append(result)
    return r.b.profile.result, retried


def test_a_profile_drives_the_model_through_the_operators_gates_and_records_each_stage(run):
    r = run(profile=PROFILE)
    # the operator holds the deadman with the joystick anywhere: the profile's ratios replace it
    result, retried = trial(r, teleop={"speed_ratio": -1.0, "curvature_ratio": 0.7, "deadman": True},
                            until=lambda rec: False, expect="done")
    assert result == "done", (retried, rows(r, "profile")[-3:])
    prof = rows(r, "profile")
    drive = [x for x in prof if x["label"] == "drive" and x["req_mps"] > 0]
    assert drive and max(x["drive_mps"] for x in drive) == pytest.approx(0.10, abs=0.002)
    assert all(x["req_kappa"] == 0.0 for x in drive) and max(x["s_m"] for x in prof) >= 0.15
    assert {"arb_mps", "arb_limit", "drive_q", "port", "meas_q"} <= set(drive[-1])
    t = r.b.profile.trial
    marks = [m["label"] for m in rows(r, "marker") if m["label"].startswith(f"profile:{t}:")]
    assert marks == [f"profile:{t}:presteer:start", f"profile:{t}:still_pre:start", f"profile:{t}:drive:start",
                     f"profile:{t}:still_post:start"]
    # one profile_end row per trial (G6 design 2; arrival-day rehearsal defect D1), and the G8 trials read the record
    ends = rows(r, "profile_end")
    assert [(e["trial"], e["result"]) for e in ends if e["trial"] == t] == [(t, "done")]
    from forklift_core.plant import m_analysis as ma
    assert [x.result for x in ma.trials(prof) if x.trial == t] == ["done"]
    assert r.nonzero(), "the model was never driven"
    # ended: zeros until the deadman is let go, then the joystick is the operator's again
    n = len(r.dev.sent)
    r.cycles(15, teleop={"speed_ratio": 1.0, "deadman": True})
    assert r.nonzero(n) == []
    r.cycles(5, teleop={"speed_ratio": 0.0, "deadman": False})
    assert r.b.profile.state == "idle"


def test_without_a_scan_the_floor_permit_holds_the_profile_and_the_trial_is_void(run):
    r = run(profile=PROFILE, floor=lambda c, d, v: (0.0, "no_scan"))
    result, retried = trial(r, teleop={"speed_ratio": 0.0, "deadman": True}, until=lambda rec: False,
                            expect="void:limited")
    assert result.startswith("void:limited"), (result, retried)
    assert r.nonzero() == []


def test_letting_go_while_driving_voids_and_stops(run):
    r = run(profile=PROFILE)
    result, retried = trial(r, teleop={"speed_ratio": 0.0, "deadman": True}, expect="void:deadman",
                            until=lambda rec: (r.b.profile.segment is not None
                                               and r.b.profile.segment.label == "drive"
                                               and r.b.profile.moving and r.b.profile.s_m > 0.02))
    assert result is None, (result, retried)  # driving now
    n = len(r.dev.sent)
    r.cycles(30, teleop={"speed_ratio": 0.0, "deadman": False})
    assert r.b.profile.result == "void:deadman"
    assert r.nonzero(n + 10) == []  # a DRIVE in flight may still go out, then nothing moves


def test_a_start_is_refused_while_moving_or_unarmed(run):
    r = run(profile=PROFILE)
    r.cycles(30)
    assert r.b.profile_start(time.monotonic_ns()) == "not_armed"


def test_the_end_drops_the_accepted_command_to_zero_at_once(run):
    """Codex G6 code review P1-2: the drive's end stops the truck in the next cycle, not at the console's next message."""
    r = run(profile=PROFILE)
    result, retried = trial(r, teleop={"speed_ratio": 0.0, "deadman": True}, expect="done",
                            until=lambda rec: (r.b.profile.segment is not None
                                               and r.b.profile.segment.label == "still_post"))
    assert result is None, (result, retried)
    assert r.b.arbiter.teleop["speed_ratio"] == 0.0  # the accepted command itself, not the next message
    k = len(r.recs)
    r.cycles(6)  # the console sends nothing more: the output slews down to zero on the accepted command alone
    targets = [x["target_mps"] for x in r.recs[k - 1:]]
    assert all(b <= a for a, b in zip(targets, targets[1:], strict=False)) and targets[-1] == 0.0, targets


def test_a_gate_that_fires_mid_drive_voids_with_its_reason_and_the_trial_is_run_again(run):
    """The harness's recovery, made deterministic: 0.6 s without a cycle while driving lets the live gates fire
    (STATE age, command age, lease), the trial is void with that gate named, and a fresh trial completes."""
    r = run(profile=PROFILE)
    result, retried = trial(r, teleop={"speed_ratio": 0.0, "deadman": True}, until=lambda rec: False,
                            expect="done", stall_once=True)
    assert result == "done" and retried, (result, retried)
    assert all(v.startswith(GATE_VOIDS) and v.count(":") >= 2 for v in retried), retried  # the gate is named

