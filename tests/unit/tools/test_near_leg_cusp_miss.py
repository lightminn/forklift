"""The near-leg cusp miss diagnosis's joins and classification (docs/plans/2026-11-03-near-leg-cusp-miss-diagnosis.md):
a stall joined to its replan by its own closed key_changed event (not later than the close, no leg switch, unique),
the excluded runs, the last scan's reason, and the robust choice with the on s2 veto."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import near_leg_cusp_miss as m  # noqa: E402

T = m.TICK_S


def stall(t, closed=None, why="key_changed", scans=None, disposition="observe_rule"):
    ev = [{"time_s": t, "event": "stop_confirmed"}, {"time_s": t + 5.0, "event": "observe_rule"}]
    if closed is not None:
        ev.append({"time_s": closed, "event": "closed", "why": why})
    return {"phase": "observe", "time_s": t, "disposition": disposition, "events": ev,
            "scans": scans if scans is not None else [scan()]}


def scan(bounds=True, converged=False, sweep=True, rear=(0.0, 0.0, 0.0)):
    return {"time_s": 0.0, "rear": list(rear), "position_error_m": 0.04, "yaw_error_rad": 0.01, "bounds_ok": bounds,
            "far": not bounds, "converged": converged, "sweep_ok": sweep, "prefix": {"failure": "endpoint_heading"}}


def run(stalls, replans, switches=(), stands=(), **extra):
    return {"cusp_stalls": stalls, "observe_replans": replans, "leg_switches": list(switches),
            "observe_end_stands": list(stands), "near_capture": {"waypoint": [0, 0, 0]}, "success": True, **extra}


def test_a_stall_after_the_k3_wait_joins_the_replan_its_close_marks():
    # K3 waits 5 s, the observe rule takes it, the replan installs and the key changes on that tick
    s = run([stall(10.0, closed=16.0 + T / 2)], [{"time_s": 16.0, "leg_serial": 1}])
    row = m.analyse(s, "r")["stalls"][0]
    assert row["join"] == "linked" and row["role"] == "near" and row["replan_trigger"] == "observe_miss"


def test_a_replan_one_tick_after_a_leg_switch_is_not_joined():
    # Codex plan review 4: the cusp accepted (leg switch, key changed), the next leg's goal fails a tick later
    s = run([stall(5.0, closed=6.016667)], [{"time_s": 6.025, "leg_serial": 1}],
            switches=[{"phase": "observe", "via": "arrival", "time_s": 6.016667}])
    assert m.analyse(s, "r")["stalls"][0]["join"] == "leg_switch"
    s = run([stall(5.0, closed=6.016667)], [{"time_s": 6.025, "leg_serial": 1}])  # later than the close: not its own
    assert m.analyse(s, "r")["stalls"][0]["join"] == "unlinked"


def test_two_candidates_are_ambiguous_and_a_non_key_close_has_no_replan():
    s = run([stall(5.0, closed=8.0)], [{"time_s": 8.0, "leg_serial": 1}, {"time_s": 8.0 - T / 2, "leg_serial": 1}])
    assert m.analyse(s, "r")["stalls"][0]["join"] == "ambiguous"
    s = run([stall(5.0, closed=8.0, why="ended")], [])
    assert m.analyse(s, "r")["stalls"][0]["join"] == "no_replan"


def test_a_realign_trace_makes_every_stall_unconfirmed():
    s = run([stall(10.0, closed=16.0)], [{"time_s": 16.0, "leg_serial": 1}], near_capture_refused=[{"x": 1}])
    row = m.analyse(s, "r")["stalls"][0]
    assert row["role"] == "unconfirmed" and row["basis"] == "realign_or_retry"


def test_a_serial_zero_replan_is_far_and_no_near_evidence_is_unconfirmed():
    s = run([stall(3.0, closed=4.0)], [{"time_s": 4.0, "leg_serial": 0}])
    assert m.analyse(s, "r")["stalls"][0]["role"] == "far"
    s = dict(run([stall(3.0)], []), near_capture={})
    assert m.analyse(s, "r")["stalls"][0]["role"] == "unconfirmed"


def test_the_last_scan_gives_the_reason_and_a_scanless_stall_is_k3_absent():
    assert m.last_scan_reason(stall(1.0, scans=[scan(bounds=False)])) == "bounds"
    assert m.last_scan_reason(stall(1.0, scans=[scan(bounds=True, converged=False)])) == "converge"
    assert m.last_scan_reason(stall(1.0, scans=[scan(converged=True, sweep=False)])) == "sweep"
    assert m.last_scan_reason(stall(1.0, scans=[scan(converged=True)])) == "sensor_or_other"
    assert m.last_scan_reason(stall(1.0, scans=[])) == "k3_absent"


def row(role, reason):
    return {"role": role, "reason": reason}


def test_the_choice_holds_only_with_every_unconfirmed_against_it():
    # Codex plan review 2 P2-5: 7 of 9 confirmed converge with 3 unconfirmed is 7/12, under 2/3
    pop = [row("near", "converge")] * 7 + [row("near", "bounds")] * 2 + [row("unconfirmed", "converge")] * 3
    assert m.choose(pop, row("near", "converge"))["decision"] == "report_only"
    pop = [row("near", "converge")] * 9 + [row("unconfirmed", "bounds")] * 1 + [row("far", "bounds")] * 5
    out = m.choose(pop, row("near", "converge"))
    assert out["decision"] == "C1" and out["choice"] == ["C1", "converge"] and out["n"] == 10


def test_on_s2_vetoes_when_unconfirmed_or_of_another_reason():
    pop = [row("near", "bounds")] * 9
    assert m.choose(pop, row("near", "bounds"))["decision"] == "cause_diagnosis"
    assert m.choose(pop, row("unconfirmed", "bounds"))["veto"] == "on_s2_unconfirmed"
    assert m.choose(pop, row("near", "converge"))["veto"] == "on_s2_reason_converge"
