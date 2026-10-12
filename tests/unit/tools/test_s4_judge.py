import copy
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from tools import s4_judge as J

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim" / "isaac"))
import insertion_geometry as IG  # noqa: E402

URDF = ROOT / "sim/models/dls08_measured/forklift.urdf"
PALLET = ROOT / "sim/models/epal6_pallet/pallet.urdf"
TICK = 1.0 / 120.0
PHASES = np.array(["other", "observe", "approach", "insert"])
CORNERS = ((0.604, 0.235), (0.604, -0.235))
BLADES = ((0.59, 0.95, 0.1175, 0.1725), (0.59, 0.95, -0.1725, -0.1175))
TERMS = {"delta_v_mps": 0.003, "tick_s": TICK, "delta_s_m": [0.001] * 121}


def yaw_quat(yaw):
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


def truth_from(xs, ys=None, yaws=None, phase="insert", speed=None, pallet_yaw=0.0):
    """Base poses per tick near an EPAL 6 pallet at the origin (entry face x = -0.3 when
    the pallet is not turned; the blade tips are 0.95 m ahead of base_link)."""
    n = len(xs)
    ys = np.zeros(n) if ys is None else np.asarray(ys)
    yaws = np.zeros(n) if yaws is None else np.asarray(yaws)
    return {
        "stamp_s": np.arange(n) * TICK + 10.0,
        "base_pose": np.array(
            [[xs[i], ys[i], 0.0, *yaw_quat(yaws[i])] for i in range(n)]
        ),
        "pallet_pose": np.tile([0.0, 0.0, 0.0, *yaw_quat(pallet_yaw)], (n, 1)),
        "lift_m": np.zeros(n),
        "phase": np.full(n, list(PHASES).index(phase)),
        "phases": PHASES,
        "applied_speed_mps": np.full(n, 0.05)
        if speed is None
        else np.asarray(speed, dtype=float),
    }


@pytest.fixture(scope="module")
def geometry():
    return IG.InsertionGeometry.from_urdfs(URDF, PALLET)


# (vi) -----------------------------------------------------------------------------------
def test_an_insertion_inside_the_depth_range_with_no_carriage_overlap_passes(geometry):
    out = J.check_vi(
        truth_from(np.linspace(-1.25 + 0.20, -1.25 + 0.32, 50)), geometry, 0.6, 49
    )
    assert out["passed"] and out["contact"]["passed"] and out["depth"]["passed"]
    assert out["depth"]["depths_m"]["left"] == pytest.approx(0.32)
    assert out["contact"]["min_carriage_face_gap_m"] == pytest.approx(
        0.346 - 0.32, abs=1e-6
    )


def test_a_carriage_that_reaches_the_face_fails_even_if_it_backs_off(geometry):
    xs = np.concatenate(
        (np.linspace(-1.25 + 0.30, -1.25 + 0.36, 30), np.full(10, -1.25 + 0.32))
    )
    out = J.check_vi(truth_from(xs), geometry, 0.6, 39)
    assert out["contact"]["carriage_overlap_ticks"] > 0 and not out["passed"]
    assert out["depth"]["passed"]  # the end depth alone would pass


def test_contact_is_judged_without_an_insertion_end_and_a_shallow_end_fails(geometry):
    xs = np.concatenate(
        (np.linspace(-1.25 + 0.30, -1.25 + 0.36, 30), np.full(10, -1.25 + 0.32))
    )
    aborted = J.check_vi(truth_from(xs), geometry, 0.6, None)
    assert (
        not aborted["depth"]["applicable"]
        and not aborted["contact"]["passed"]
        and not aborted["passed"]
    )
    shallow = J.check_vi(
        truth_from(np.linspace(-1.25 + 0.10, -1.25 + 0.29, 20)), geometry, 0.6, 19
    )
    assert (
        shallow["contact"]["passed"]
        and not shallow["depth"]["passed"]
        and not shallow["passed"]
    )


# (vii) ----------------------------------------------------------------------------------
def braking_truth(
    v=0.05,
    brake_s=0.2,
    curvature=0.0,
    n_before=24,
    n_after=36,
    pallet_yaw=0.0,
    after=None,
):
    """Constant v, then a linear speed ramp to zero over brake_s from the stop's start
    (tick n_before), then standing (or `after(i)` = (speed, yaw rate) for each tick past the
    ramp); the rear axle runs along its heading on a no-slip arc, base_link 0.34 m ahead.
    The last tick is phase "other" (the record goes one tick past the approach)."""
    speeds = [v] * n_before
    k = int(round(brake_s / TICK))
    speeds += [v * (1 - (i + 1) / k) for i in range(k)]
    tail = (
        [(0.0, 0.0)] * n_after if after is None else [after(i) for i in range(n_after)]
    )
    speeds += [s_ for s_, _ in tail]
    rates = [0.0] * (n_before + k) + [r for _, r in tail]
    rear, yaw = np.array([-3.34, 0.0]), 0.0
    xs, ys, yaws = [], [], []
    for i, s_ in enumerate(speeds):
        if i:
            yaw += (curvature * s_ + rates[i]) * TICK
            rear = rear + s_ * TICK * np.array([math.cos(yaw), math.sin(yaw)])
        xs.append(rear[0] + 0.34 * math.cos(yaw))
        ys.append(rear[1] + 0.34 * math.sin(yaw))
        yaws.append(yaw)
    applied = [v] * (n_before + 1) + [0.0] * (len(speeds) - n_before - 1)
    truth = truth_from(
        xs, ys=ys, yaws=yaws, phase="approach", speed=applied, pallet_yaw=pallet_yaw
    )
    truth["phase"][-1] = 0
    return truth, 10.0 + n_before * TICK, sum(speeds[n_before + 1 :]) * TICK


def near(truth, events, protect_index=None, ticks=None, last=None, terminal=False):
    """A near-field record as the runner writes it: a tick row for every tick of the
    section up to its last tick, flagged once the protection (when given) started."""
    t = truth["stamp_s"]
    last = len(t) - 2 if last is None else last
    rows = {
        round(float(t[i]), 9): {
            "t": float(t[i]),
            "hold": False,
            "reason": None,
            "terminal": None,
        }
        for i in range(1, last + 1)
    }
    for r in ticks or []:
        rows[round(float(r["t"]), 9)] = {**rows.get(round(float(r["t"]), 9), {}), **r}
    if protect_index is not None:
        for i in range(protect_index, last + 1):
            rows[round(float(t[i]), 9)]["protect"] = True
    events_ = (
        [{"event": "terminal_standing", "time_s": float(t[-1])}] if terminal else []
    )
    return {
        "stop_events": events,
        "ticks": list(rows.values()),
        "events": events_,
        "protect": None
        if protect_index is None
        else {"time_s": float(t[protect_index])},
    }


def event(start, sweep, ks=(1.0, 1.0), lateral=0.0005):
    return {
        "start_s": start,
        "standing_s": start + 0.3,
        "reason": "budget:budget",
        "sweep_m": sweep,
        "corner_sweep_k": None if ks is None else list(ks),
        "lateral_sweep_m": lateral,
    }


def judge(truth, events, protect=None, ticks=None, span=None, terminal=False):
    """protect: "before" (the protection began before the stop), "after", or None."""
    n = len(truth["stamp_s"])
    span = span or (1, n - 2)
    index = {"before": 1, "after": span[1] - 1, None: None}[protect]
    record = near(truth, events, index, ticks, last=span[1], terminal=terminal)
    return J.check_vii(truth, record, TERMS, span, -0.34, CORNERS, BLADES)


def test_a_stop_within_its_contract_and_sweeps_passes():
    truth, start, travel = braking_truth()
    out = judge(truth, [event(start, travel + 0.002)], protect="before")
    stop = out["stops"][0]
    assert (
        out["passed"]
        and stop["standing_at_end"]
        and stop["travel_ok"]
        and stop["corner_ok"]
        and stop["blade_ok"]
    )
    assert stop["travel_m"] == pytest.approx(travel, rel=1e-9)
    assert stop["corner_axial_m"][0] == pytest.approx(stop["travel_m"], rel=1e-6)
    assert out["zero_steps_in_section"] == 1 and stop["source"] == "near_stop"


def test_a_stop_past_its_contract_or_turning_past_its_sweep_fails():
    truth, start, travel = braking_truth()
    assert not judge(truth, [event(start, travel - 0.0005)], protect="before")["stops"][
        0
    ]["travel_ok"]
    turning, start, travel = braking_truth(curvature=0.36)
    stop = judge(
        turning, [event(start, travel + 0.001, lateral=0.00001)], protect="before"
    )["stops"][0]
    assert stop["travel_ok"] and not stop["blade_ok"] and not stop["passed"]
    assert stop["blade_lateral_m"] == pytest.approx(0.36 * 1.29 * travel, rel=0.05)


def test_the_sideways_travel_is_across_the_body_axis_not_the_pallets():
    truth, start, travel = braking_truth(pallet_yaw=0.02)
    stop = judge(truth, [event(start, travel + 0.001, lateral=0.0)], protect="before")[
        "stops"
    ][0]
    assert stop["blade_lateral_m"] == pytest.approx(0.0, abs=1e-12) and stop["blade_ok"]


def test_rolling_turning_or_moving_again_under_the_zero_command_fails():
    # Codex review: creeping at 0.011 m/s; turning on at 5 mrad/s after the brake; quiet
    # for a while, then moving again -- all under the zero command.
    for after in (
        lambda i: (0.011, 0.0),
        lambda i: (0.0, 0.005),
        lambda i: (0.0 if i < 20 else 0.004, 0.0),
    ):
        truth, start, travel = braking_truth(after=after)
        stop = judge(truth, [event(start, 0.05, lateral=0.05)], protect="before")[
            "stops"
        ][0]
        assert not stop["standing_at_end"] and not stop["passed"]
    # The turn counts in the sweep over the whole interval, not just to a first quiet tick.
    truth, start, travel = braking_truth(after=lambda i: (0.0, 0.005))
    stop = judge(truth, [event(start, 0.05, lateral=0.05)], protect="before")["stops"][
        0
    ]
    assert stop["blade_lateral_m"] > 0.9 * 0.005 * 36 * TICK * 1.29


def test_missing_records_are_unverified_never_passed():
    truth, start, travel = braking_truth()
    with pytest.raises(J.Unverified, match="sweeps"):
        judge(truth, [event(start, travel + 0.002, ks=None)], protect="before")
    assert judge(truth, [event(start, travel + 0.002, ks=None)], protect="after")[
        "passed"
    ]  # before it: not required
    with pytest.raises(J.Unverified, match="off the record"):
        judge(truth, [event(start + 0.4 * TICK, 0.01)], protect="before")
    with pytest.raises(J.Unverified, match="protection's start"):
        judge(truth, [], protect=None, ticks=[{"t": 10.0, "protect": True}])
    bad = near(truth, [], protect_index=5)
    bad["protect"]["time_s"] = 99.0  # a start off the record cannot skip the sweeps
    with pytest.raises(J.Unverified, match="protection's start"):
        J.check_vii(
            truth, bad, TERMS, (1, len(truth["stamp_s"]) - 2), -0.34, CORNERS, BLADES
        )
    with pytest.raises(J.Unverified, match="two near-field stops"):
        judge(truth, [event(start, 0.01), event(start, 0.02)], protect="before")


def test_an_unrecorded_stop_uses_the_tick_record_or_the_drive_terms_before_the_protection():
    truth, start, travel = braking_truth()
    stop = judge(truth, [])["stops"][0]
    assert stop["source"] == "zero_step" and stop["contract"] == "drive_terms"
    assert stop["sweep_m"] == pytest.approx(J.stop_distance_m(0.05 + 0.003) + 0.001)
    tick = {
        "t": start,
        "sweep_m": travel + 0.001,
        "protection": {"corner_sweep_k": [1.0, 1.0], "lateral_sweep_m": 0.00001},
    }
    turning, start, travel = braking_truth(curvature=0.36)
    tick["t"], tick["sweep_m"] = start, travel + 0.001
    stop = judge(turning, [], protect="before", ticks=[tick])["stops"][0]
    assert (
        stop["contract"] == "tick" and not stop["blade_ok"]
    )  # the sweep is not bypassed
    with pytest.raises(J.Unverified, match="no tick record"):
        judge(turning, [], protect="before", ticks=[])


def test_a_zero_command_given_at_the_last_section_tick_is_judged():
    truth, start, travel = braking_truth()
    i0 = int(round((start - 10.0) / TICK))
    out = judge(truth, [event(start, travel + 0.002)], protect="before", span=(1, i0))
    assert (
        out["zero_steps_in_section"] == 1 and out["stops"][0]["source"] == "near_stop"
    )


def test_a_stop_whose_command_does_not_drop_is_unverified():
    truth, start, travel = braking_truth()
    truth["applied_speed_mps"][:] = 0.05
    with pytest.raises(J.Unverified, match="no zero command"):
        judge(truth, [event(start, 0.05)], protect="before")


# Records --------------------------------------------------------------------------------
def test_record_problems_catch_gaps_interruptions_and_nonfinite_values():
    truth, _, _ = braking_truth()
    assert J.record_problems(truth) == []
    gap = {
        k: (np.delete(v, 30, axis=0) if k != "phases" else v) for k, v in truth.items()
    }
    assert any("contiguous" in p for p in J.record_problems(gap))
    other = {**truth, "phase": truth["phase"].copy()}
    other["phase"][30] = 0
    assert any("interrupts" in p for p in J.record_problems(other))
    bad = {**truth, "lift_m": truth["lift_m"].copy()}
    bad["lift_m"][20] = np.nan
    assert any("non-finite lift_m" in p for p in J.record_problems(bad))
    ends = {**truth, "phase": np.full_like(truth["phase"], 2)}
    assert J.record_problems(ends) == []  # a run may end on its last approach tick
    assert J.record_problems({**truth, "phase": np.zeros_like(truth["phase"])}) == [
        "no approach or insert tick in the truth record"
    ]


def test_the_insertion_end_must_be_the_last_insert_tick_at_the_transition():
    truth = truth_from(np.linspace(-1.0, -0.9, 30))
    truth["phase"][21:] = 0
    t20 = float(truth["stamp_s"][20])
    assert (
        J.insert_end_index(
            truth, {"transitions": [{"from": "insert", "to": "lift", "time_s": t20}]}
        )
        == 20
    )
    assert J.insert_end_index(truth, {"transitions": []}) is None
    for when in (100.0, float(truth["stamp_s"][15])):
        with pytest.raises(J.Unverified):
            J.insert_end_index(
                truth,
                {"transitions": [{"from": "insert", "to": "lift", "time_s": when}]},
            )


def test_the_section_needs_an_armed_tick_inside_the_approach():
    truth, _, _ = braking_truth()
    with pytest.raises(J.Unverified, match="never entered"):
        J.near_section(truth, {"events": []})
    with pytest.raises(J.Unverified, match="off the record"):
        J.near_section(truth, {"events": [{"event": "armed", "time_s": 999.0}]})
    assert J.near_section(
        truth, {"events": [{"event": "armed", "time_s": float(truth["stamp_s"][3])}]}
    ) == (3, len(truth["stamp_s"]) - 2)


def test_the_restart_tick_is_not_part_of_the_stop():
    # Codex review: a 0.0025 m/s restart after the stand added one tick of travel and failed
    # a stop that met its contract.
    truth, start, travel = braking_truth()
    n = len(truth["stamp_s"])
    rise = n - 6
    truth["applied_speed_mps"][rise:] = 0.0025
    xs = truth["base_pose"][:, 0].copy()
    xs[rise:] = xs[rise - 1] + 0.0025 * TICK * np.arange(1, n - rise + 1)
    truth["base_pose"][:, 0] = xs
    stop = judge(truth, [event(start, travel + 0.00001)], protect="before")["stops"][0]
    assert (
        stop["rose"]
        and stop["travel_m"] == pytest.approx(travel, rel=1e-9)
        and stop["passed"]
    )


def test_a_stop_begun_while_the_command_is_already_zero_is_judged_from_its_tick():
    # Codex review: a new budget stop on a standing truck (or right after a release) holds
    # a zero command with no step to it.
    truth, start, travel = braking_truth()
    i0 = int(round((start - 10.0) / TICK))
    later = float(truth["stamp_s"][i0 + 30])
    out = judge(
        truth, [event(start, travel + 0.002), event(later, 0.001)], protect="before"
    )
    held = [s_ for s_ in out["stops"] if s_["source"] == "near_stop_held"]
    assert len(held) == 1 and held[0]["passed"] and held[0]["travel_m"] < 1e-9


def test_a_truth_record_cut_short_is_unverified():
    # Codex review: dropping the end of the truth record must not turn a fail into a pass.
    truth, start, travel = braking_truth(after=lambda i: (0.0, 0.005))
    record = near(truth, [event(start, 0.05, lateral=0.05)], protect_index=1)
    cut = {k: (v[:-20] if k != "phases" else v) for k, v in truth.items()}
    span = (1, len(cut["stamp_s"]) - 1)
    with pytest.raises(
        J.Unverified, match="off the record's ticks|do not end at the same tick"
    ):
        J.check_vii(cut, record, TERMS, span, -0.34, CORNERS, BLADES)
    # The near record's ticks beyond the cut dropped too: the two still end apart.
    record["ticks"] = [
        r for r in record["ticks"] if r["t"] <= cut["stamp_s"][-1] + 1e-9
    ][:-5]
    with pytest.raises(J.Unverified, match="do not end at the same tick"):
        J.check_vii(cut, record, TERMS, span, -0.34, CORNERS, BLADES)


def test_a_terminal_stop_on_the_last_tick_of_a_standing_truck_is_judged():
    # Codex review: a new terminal reason on a standing truck ends the run on that tick.
    truth, start, travel = braking_truth()
    ended = {
        k: (v[:-1] if k != "phases" else v) for k, v in truth.items()
    }  # ends in the approach
    n = len(ended["stamp_s"])
    last_t = float(ended["stamp_s"][-1])
    out = judge(
        ended,
        [event(start, travel + 0.002), event(last_t, 0.001)],
        protect="before",
        span=(1, n - 1),
        terminal=True,
    )
    held = [s_ for s_ in out["stops"] if s_["source"] == "near_stop_held"]
    assert len(held) == 1 and held[0]["standing_at_end"] and held[0]["passed"]
    with pytest.raises(
        J.Unverified, match="no zero command"
    ):  # without the terminal event: unverified
        judge(
            ended,
            [event(start, travel + 0.002), event(last_t, 0.001)],
            protect="before",
            span=(1, n - 1),
        )


def test_nan_record_times_are_unverified():
    # Codex review: a NaN compares false; times are checked finite and on a tick first.
    truth, start, travel = braking_truth()
    n = len(truth["stamp_s"])
    record = near(truth, [event(start, travel + 0.002)], protect_index=1, terminal=True)
    record["events"][0]["time_s"] = float("nan")
    with pytest.raises(J.Unverified, match="no time"):
        J.check_vii(truth, record, TERMS, (1, n - 2), -0.34, CORNERS, BLADES)
    record = near(truth, [event(start, travel + 0.002)], protect_index=1)
    record["ticks"][0]["t"] = float("nan")
    with pytest.raises(J.Unverified, match="no time"):
        J.check_vii(truth, record, TERMS, (1, n - 2), -0.34, CORNERS, BLADES)


def follower_blip(rise_after=5):
    """The follower creeps to an arrival, gives a zero command for rise_after ticks and
    then a small positive one again (smoke2 at 70.858 s)."""
    truth, start, travel = braking_truth(v=0.0022, brake_s=0.03, n_after=40)
    i0 = int(round((start - 10.0) / TICK))
    truth["applied_speed_mps"][i0 + 1 + rise_after :] = 0.0025
    return truth, start, i0


def test_a_follower_zero_command_that_rises_again_is_interrupted_not_failed():
    truth, start, i0 = follower_blip()
    out = judge(
        truth,
        [],
        protect="before",
        ticks=[
            {
                "t": start,
                "sweep_m": 0.005,
                "protection": {"corner_sweep_k": [1.0, 1.0], "lateral_sweep_m": 0.001},
            }
        ],
    )
    stop = out["stops"][0]
    assert stop["follower"] and stop["rose"] and stop["interrupted"] and stop["passed"]
    assert out["interrupted"] == 1 and out["completed_stops"] == 0


def test_a_tick_holding_a_stop_without_its_record_is_unverified():
    # Codex review: a missing stop record must not pass as a follower's zero command.
    truth, start, i0 = follower_blip()
    with pytest.raises(
        J.Unverified, match="holds a near-field stop|with no stop record"
    ):
        judge(
            truth,
            [],
            protect="before",
            ticks=[
                {
                    "t": start,
                    "sweep_m": 0.005,
                    "hold": True,
                    "reason": "budget:budget",
                    "protection": {
                        "corner_sweep_k": [1.0, 1.0],
                        "lateral_sweep_m": 0.001,
                    },
                }
            ],
        )
    record = near(truth, [], protect_index=1)
    record["ticks"] = [r for r in record["ticks"] if abs(r["t"] - start) > 1e-9]
    with pytest.raises(J.Unverified, match="no tick record|no near-field tick record"):
        J.check_vii(
            truth, record, TERMS, (1, len(truth["stamp_s"]) - 2), -0.34, CORNERS, BLADES
        )


def test_a_near_stop_that_rises_before_standing_still_fails():
    truth, start, travel = braking_truth()
    i0 = int(round((start - 10.0) / TICK))
    truth["applied_speed_mps"][i0 + 4 :] = (
        0.0025  # released after 3 ticks, still braking
    )
    stop = judge(truth, [event(start, 0.05)], protect="before")["stops"][0]
    assert (
        stop["rose"]
        and not stop["interrupted"]
        and not stop["standing_at_end"]
        and not stop["passed"]
    )


def test_a_restart_at_the_section_boundary_is_not_an_interruption():
    # Codex review: the first positive record at last + 1 means the restart began on the
    # section's last tick -- the zero interval ends at the boundary and must stand.
    truth, start, i0 = follower_blip(rise_after=3)
    rise = i0 + 1 + 3
    tick = {
        "t": start,
        "sweep_m": 0.005,
        "protection": {"corner_sweep_k": [1.0, 1.0], "lateral_sweep_m": 0.001},
    }
    out = judge(truth, [], protect="before", ticks=[tick], span=(1, rise - 1))
    stop = out["stops"][0]
    assert (
        not stop["interrupted"] and not stop["standing_at_end"] and not stop["passed"]
    )
    assert out["unfinished_stops"] == 1 and out["completed_stops"] == 0


def test_a_near_stop_beginning_inside_a_follower_zero_interval_needs_its_record():
    # Codex review: a budget hold from two ticks into the follower's zero command, its record
    # removed, must not pass as an interruption; missing stop-state fields are unverified.
    truth, start, i0 = follower_blip(rise_after=8)
    tick0 = {
        "t": start,
        "sweep_m": 0.005,
        "protection": {"corner_sweep_k": [1.0, 1.0], "lateral_sweep_m": 0.001},
    }
    held = [
        {"t": float(truth["stamp_s"][i0 + k]), "hold": True, "reason": "budget:budget"}
        for k in (2, 3, 4)
    ]
    with pytest.raises(J.Unverified, match="begins at .* with no stop record"):
        judge(truth, [], protect="before", ticks=[tick0, *held])
    record = near(truth, [], protect_index=1, ticks=[tick0])
    for r in record["ticks"]:
        r.pop("hold", None)
    with pytest.raises(J.Unverified, match="lacks its stop state"):
        J.check_vii(
            truth, record, TERMS, (1, len(truth["stamp_s"]) - 2), -0.34, CORNERS, BLADES
        )


def test_a_new_stop_inside_a_near_stop_interval_needs_its_record():
    # Codex review: a near stop, released while the command stays zero, then a new budget stop
    # -- its record removed -- must not pass.
    truth, start, travel = braking_truth()
    i0 = int(round((start - 10.0) / TICK))
    t = truth["stamp_s"]
    states = [
        {"t": float(t[j]), "hold": True, "reason": "budget:budget"}
        for j in range(i0, i0 + 20)
    ]
    states += [
        {"t": float(t[j]), "hold": True, "reason": "budget:budget"}
        for j in range(i0 + 30, i0 + 40)
    ]
    second = float(t[i0 + 30])
    with pytest.raises(J.Unverified, match="begins at .* with no stop record"):
        judge(truth, [event(start, travel + 0.002)], protect="before", ticks=states)
    out = judge(
        truth,
        [event(start, travel + 0.002), event(second, 0.001)],
        protect="before",
        ticks=states,
    )
    assert {s_["source"] for s_ in out["stops"]} == {"near_stop", "near_stop_held"}


def test_missing_tick_rows_in_the_section_are_unverified():
    # Codex review: dropping the released ticks between two stops joined them into one.
    truth, start, travel = braking_truth()
    record = near(truth, [event(start, travel + 0.002)], protect_index=1)
    record["ticks"] = record["ticks"][:30] + record["ticks"][45:]
    with pytest.raises(J.Unverified, match="have no near-field tick record"):
        J.check_vii(
            truth, record, TERMS, (1, len(truth["stamp_s"]) - 2), -0.34, CORNERS, BLADES
        )


# (i)-(v) --------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def bounds():
    from forklift_core.perception.near_field_bounds import NearFieldBounds

    return NearFieldBounds.load(ROOT / "config/near_field_bounds_measured.json")


def test_v_a_still_pallet_passes_and_a_nudge_of_two_hundredths_of_a_millimetre_fails():
    truth = truth_from(np.linspace(-1.5, -1.2, 40))
    assert J.check_v(truth, float(truth["stamp_s"][2]), 39)["passed"]
    truth["pallet_pose"][20:, 0] += 2e-5
    out = J.check_v(truth, float(truth["stamp_s"][2]), 39)
    assert not out["passed"] and out["max_translation_m"] == pytest.approx(2e-5)
    turned = truth_from(np.linspace(-1.5, -1.2, 40))
    turned["pallet_pose"][20:, 3:] = yaw_quat(2e-5)
    assert not J.check_v(turned, float(turned["stamp_s"][2]), 39)["passed"]


def test_v_a_non_finite_pallet_or_an_empty_span_is_unverified():
    truth = truth_from(np.linspace(-1.5, -1.2, 40))
    with pytest.raises(J.Unverified, match="before the near capture"):
        J.check_v(truth, float(truth["stamp_s"][20]), 10)
    truth["pallet_pose"][25, 0] = np.nan  # max() would drop it
    with pytest.raises(J.Unverified, match="pallet pose"):
        J.check_v(truth, float(truth["stamp_s"][2]), 39)


def states_every(truth, period_ticks, delay_ticks=1, latency=0.0917):
    """A result per period: stamped at its tick, read delay_ticks later."""
    t = truth["stamp_s"]
    return [
        {
            "since_s": float(t[i + delay_ticks]),
            "stamp_s": float(t[i]),
            "source": "front",
        }
        for i in range(0, len(t) - delay_ticks, period_ticks)
    ]


def test_iv_the_travel_from_a_pixel_time_to_the_next_result_is_bounded(bounds):
    # 0.055 m/s with a result every 12 ticks: about (0.0989 + 0.1083) x 0.055 = 11.4 mm.
    xs = -3.0 + 0.055 * TICK * np.arange(240)
    truth = truth_from(xs, phase="approach")
    states = states_every(truth, 12)
    out = J.check_iv(truth, states, (20, 230), bounds, -0.34)
    assert out["passed"] and out["worst"]["travel_m"] == pytest.approx(
        0.055 * (bounds.max_latency_s + 12 * TICK + TICK), abs=0.0012
    )
    # Results stop coming: the travel from the last one runs past 0.03 m.
    out = J.check_iv(truth, states[:5], (20, 230), bounds, -0.34)
    assert not out["passed"]
    # Codex review: a NaN arrival after the section made the travel NaN, which the
    # maximum dropped -- the same starved window then passed.
    nan_end = [*states[:5], {**states[5], "since_s": float("nan")}]
    with pytest.raises(J.Unverified, match="window's ends"):
        J.check_iv(truth, nan_end, (20, 230), bounds, -0.34)


def start_and_frames(truth, read_s):
    """A near-capture start record and one accepted frame read at read_s."""
    t = truth["stamp_s"]
    pocket = {"width_m": 0.2275, "height_m": 0.09}
    observation = {
        "stamp_ns": 1,
        "clock_domain": "synthetic",
        "frame_id": "base_link",
        "source_provenance": "synthetic",
        "status": "valid",
        "left": {**pocket, "center_m": [1.0, 0.18625, 0.061]},
        "right": {**pocket, "center_m": [1.0, -0.18625, 0.061]},
        "insertion_yaw_rad": 0.0,
        "position_sigma_m": 0.001,
        "yaw_sigma_rad": 0.001,
        "reason": None,
    }
    start = {
        "built_s": float(t[2]),
        "aligned_s": float(t[1]),
        "pose": [0.0, 0.0, 0.0],
        "observation": observation,
    }
    frame = {
        "accepted": True,
        "read_s": read_s,
        "latest_stamp_s": float(t[10]),
        "latest_held": {
            "left": (1.0, 0.18625, 0.061),
            "right": (1.0, -0.18625, 0.061),
            "yaw": 0.0,
        },
        "latest_widths_m": [0.2275, 0.2275],
        "latest_bound": {"wall_m": 0.0005, "yaw_rad": 0.0003},
        "latest_source": "front",
    }
    return start, {"frames": [frame]}


def test_accepted_states_refuse_non_finite_or_incomplete_observations(bounds):
    truth = truth_from(np.linspace(-1.5, -1.2, 40))
    start, near_ = start_and_frames(truth, float(truth["stamp_s"][11]))
    states = J.accepted_states(near_, start, bounds)
    assert [st["source"] for st in states] == ["near_capture", "front"]
    bad = [
        ("read_s", float("nan"), "times"),
        ("latest_stamp_s", float("inf"), "times"),
        ("latest_widths_m", [0.2275, float("nan")], "widths"),
        ("latest_widths_m", [0.2275, 0.0], "not positive"),
        ("latest_bound", {"wall_m": float("nan"), "yaw_rad": 0.0003}, "bound"),
        ("latest_bound", {"yaw_rad": 0.0003}, "incomplete"),
        (
            "latest_held",
            {"left": (1.0, float("nan"), 0.061), "right": (1.0, -0.1, 0.06), "yaw": 0},
            "held pockets",
        ),
    ]
    for key, value, message in bad:
        _, broken = start_and_frames(truth, float(truth["stamp_s"][11]))
        broken["frames"][0][key] = value
        with pytest.raises(J.Unverified, match=message):
            J.accepted_states(broken, start, bounds)


def odometry_case(bounds, scale=1.0):
    from types import SimpleNamespace

    n = 200
    t = 10.0 + TICK * np.arange(n)
    x = -3.0 + 0.055 * TICK * np.arange(n)
    control = np.column_stack(
        (t, x * scale, np.zeros(n), np.zeros(n), x, np.zeros(n), np.zeros(n))
    )
    run = SimpleNamespace(control=control, truth=SimpleNamespace(t=t))
    # A result every tick whose pixel time is 0.2 s before it: age 0.2 s throughout.
    states = [
        {
            "since_s": float(t[i]),
            "stamp_s": float(t[i]) - 0.2 + bounds.max_latency_s,
            "source": "roof",
        }
        for i in range(n)
    ]
    ticks = [{"t": float(t[i]), "pixel_s": float(t[i]) - 0.2} for i in range(40, 190)]
    return run, states, ticks


def test_ii_exact_odometry_passes_and_a_drift_past_the_table_fails(bounds):
    run, states, ticks = odometry_case(bounds)
    out = J.check_ii(run, {"ticks": ticks}, states, bounds, (40, 189))
    assert out["passed"] and out["worst"]["e_ratio"] == pytest.approx(0.0, abs=1e-9)
    run, states, ticks = odometry_case(bounds, scale=1.5)  # the control reads 50 % long
    assert not J.check_ii(run, {"ticks": ticks}, states, bounds, (40, 189))["passed"]


def test_ii_a_pixel_time_that_is_not_the_latest_results_is_unverified(bounds):
    # Codex review: a forged pixel time (= now) would zero the age and pass.
    run, states, ticks = odometry_case(bounds, scale=1.5)
    forged = [{**row, "pixel_s": row["t"]} for row in ticks]
    with pytest.raises(J.Unverified, match="records pixel time"):
        J.check_ii(run, {"ticks": forged}, states, bounds, (40, 189))
    missing = [{**row, "pixel_s": float("nan")} for row in ticks]
    with pytest.raises(J.Unverified, match="pixel time"):
        J.check_ii(run, {"ticks": missing}, states, bounds, (40, 189))


def test_ii_a_non_finite_control_record_is_unverified(bounds):
    run, states, ticks = odometry_case(bounds, scale=1.5)
    run.control[60:, 1] = np.nan  # the drift would otherwise vanish in the ratio
    with pytest.raises(J.Unverified, match="poses over the age"):
        J.check_ii(run, {"ticks": ticks}, states, bounds, (40, 189))


def test_i_an_estimate_on_the_true_walls_passes_and_one_off_by_three_centimetres_fails(
    bounds,
):
    from types import SimpleNamespace

    from forklift_core.perception.pallet_geometry import load_pallet_geometry
    from tools import d8b_calibration as CAL

    geometry = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
    truth_d = truth_from(np.full(30, -1.10))  # tips 0.15 m before the face at x = -0.3
    truth = CAL.Truth(truth_d, None)
    run = SimpleNamespace(truth=truth)
    true_held = {
        "left": (-0.3, 0.18625, 0.061),
        "right": (-0.3, -0.18625, 0.061),
        "yaw": 0.0,
    }
    base = (-1.10, 0.0, 0.0)
    state = {
        "since_s": 9.0,
        "stamp_s": float(truth_d["stamp_s"][0]) + 0.1,
        "held": true_held,
        "widths": (geometry.opening_width_m, geometry.opening_width_m),
        "bound": {
            "wall_m": 0.0005,
            "yaw_rad": 0.0003,
            "along_m": 0.006,
            "lateral_m": 0.0005,
        },
        "source": "roof",
    }
    ticks = [
        {"t": float(truth_d["stamp_s"][i]), "protect": True, "base": list(base)}
        for i in range(5, 25)
    ]
    out = J.check_i(
        run, {"ticks": ticks}, [state], bounds, geometry, (5, 24), 0.346, -0.34
    )
    assert out["passed"] and out["worst"]["ratio"] == pytest.approx(0.0, abs=1e-9)
    off = {
        **state,
        "held": {
            **true_held,
            "left": (-0.3, 0.21625, 0.061),
            "right": (-0.3, -0.15625, 0.061),
        },
    }
    assert not J.check_i(
        run, {"ticks": ticks}, [off], bounds, geometry, (5, 24), 0.346, -0.34
    )["passed"]
    # a NaN in the record must not vanish in the ratio comparison
    nan_base = [dict(row) for row in ticks]
    nan_base[3]["base"] = [-1.10, float("nan"), 0.0]
    with pytest.raises(J.Unverified):
        J.check_i(
            run, {"ticks": nan_base}, [state], bounds, geometry, (5, 24), 0.346, -0.34
        )
    nan_bound = {**state, "bound": {**state["bound"], "wall_m": float("nan")}}
    with pytest.raises(J.Unverified):
        J.check_i(
            run, {"ticks": ticks}, [nan_bound], bounds, geometry, (5, 24), 0.346, -0.34
        )
    no_bound = {**state, "bound": {**state["bound"], "wall_m": -1.0}}
    with pytest.raises(J.Unverified):
        J.check_i(
            run, {"ticks": ticks}, [no_bound], bounds, geometry, (5, 24), 0.346, -0.34
        )


def test_iii_checks_the_read_cadence_the_run_delay_and_the_frames(monkeypatch):
    from types import SimpleNamespace

    from tools import d8b_calibration as CAL

    data = {
        "latency": {
            "read_delay_s": TICK,
            "render_period_s": 0.1,
            "band_s": [0.0844, 0.0989],
        }
    }
    calibration = {
        "dwells": {
            "1": [
                {
                    "bias_m": 1e-6,
                    "max_shift_m": 1e-5,
                    "distance_m": 2.5,
                    "distance_range_m": [2.5, 2.5],
                },
                {
                    "bias_m": 2e-6,
                    "max_shift_m": 1e-5,
                    "distance_m": 0.5,
                    "distance_range_m": [0.5, 0.5],
                },
            ]
        }
    }
    reads = [
        {"read_s": 10.0 + 0.1 * k + TICK, "stamp_s": 10.0 + 0.1 * k, "result": "ok"}
        for k in range(20)
    ]
    run = SimpleNamespace(reads=reads)
    good = {
        "L_run_s": 0.0917,
        "frames": 200,
        "eligible": 150,
        "single_piece": 140,
        "multi_piece": 10,
        "mismatch": 0,
        "out_of_range": 0,
        "edge": 0,
        "not_eligible": {},
        "band_s": [0.088, 0.096],
    }
    seen = {}
    monkeypatch.setattr(
        CAL,
        "analyse_frames",
        lambda run_, model, entries: seen.setdefault("entries", entries),
    )
    monkeypatch.setattr(CAL, "summarise_run", lambda rows: dict(good))
    out = J.check_iii(run, data, calibration, None)
    assert out["passed"] and out["cadence_ok"] and not out["frame_failures"]
    assert all(
        e[1] == pytest.approx(2e-6 + 1e-5) for e in seen["entries"]
    )  # the largest, everywhere
    late = [dict(r) for r in reads]
    late[7]["read_s"] += TICK
    assert not J.check_iii(SimpleNamespace(reads=late), data, calibration, None)[
        "passed"
    ]
    for change in (
        {"L_run_s": 0.12},
        {"out_of_range": 19},
        {"mismatch": 1},
        {"edge": 2},
        {"single_piece": 3},
    ):
        monkeypatch.setattr(CAL, "summarise_run", lambda rows, c=change: {**good, **c})
        assert not J.check_iii(run, data, calibration, None)["passed"], change


def test_iv_the_window_ends_are_interpolated_and_a_result_replaced_before_the_section_is_skipped(
    bounds,
):
    xs = -3.0 + 0.055 * TICK * np.arange(240)
    truth = truth_from(xs, phase="approach")
    t = truth["stamp_s"]
    stamp = float(t[20]) + 0.3 * TICK  # its pixel time falls between ticks
    states = [
        {"since_s": float(t[0]), "stamp_s": float(t[0]), "source": "old"},
        {"since_s": float(t[20]), "stamp_s": stamp, "source": "front"},
    ]
    out = J.check_iv(truth, states, (20, 30), bounds, -0.34)
    assert out["observations"] == 1  # the old one is never the latest at a section tick
    expected = 0.055 * (float(t[30]) - (stamp - bounds.max_latency_s))
    assert out["worst"]["travel_m"] == pytest.approx(expected, rel=1e-9)


# (viii) -- plan D8 S4b correction points ------------------------------------------------
def s4b_case(bounds):
    """A synthetic S4b record: the estimate passes 1.0 m at tick 10 and 0.6 m at tick 100
    (pallet at the origin, face x = -0.3, blade tips 0.95 m ahead of base_link); each point
    stands from t_s (ticks 20 and 110), is decided on the first tick a front observation
    taken after t_s is the latest, installs a revision there and releases two ticks later
    with zero commands."""
    sys.path.insert(0, str(ROOT / "sim" / "isaac"))
    import near_tracking as NT

    n = 220
    t = np.arange(n) * TICK + 10.0
    L = bounds.max_latency_s
    obs_stamps = (float(t[20]) + L + 0.01, float(t[110]) + L + 0.01)
    reads = tuple(stamp + 0.1 for stamp in obs_stamps)
    decisions = tuple(int(np.searchsorted(t, r - 1e-9)) for r in reads)
    xs = np.empty(n)
    xs[:10] = np.linspace(-2.262, -2.2505, 10)
    xs[10 : decisions[0] + 3] = -2.2495
    xs[decisions[0] + 3 : 100] = np.linspace(-2.2490, -1.851, 100 - decisions[0] - 3)
    xs[100 : decisions[1] + 3] = -1.8495
    xs[decisions[1] + 3 :] = np.linspace(-1.8490, -1.5, n - decisions[1] - 3)
    speed = np.full(n, 0.05)
    speed[11 : decisions[0] + 4] = 0.0
    speed[101 : decisions[1] + 4] = 0.0
    truth = truth_from(xs, phase="approach", speed=speed)
    held = {
        "left": (-0.3, 0.18625, 0.061),
        "right": (-0.3, -0.18625, 0.061),
        "yaw": 0.0,
    }
    widths = (0.2275, 0.2275)
    bound = {"wall_m": 0.0005, "yaw_rad": 0.0003, "along_m": 0.006, "lateral_m": 0.0005}
    states = [
        {
            "since_s": 9.0,
            "stamp_s": 9.0,
            "held": held,
            "widths": widths,
            "bound": bound,
            "source": "near_capture",
        }
    ]
    regular = [float(t[i]) for i in range(0, n - 20, 12)]
    # No other result arrives between an observation's read and its decision tick.
    regular = [
        s_
        for s_ in regular
        if not any(
            r - 0.1 < s_ + 0.1 <= t[d] + 1e-9
            for r, d in zip(reads, decisions, strict=True)
        )
    ]
    for stamp in sorted({*regular, *obs_stamps}):
        states.append(
            {
                "since_s": stamp + 0.1,
                "stamp_s": stamp,
                "held": held,
                "widths": widths,
                "bound": bound,
                "source": "front",
            }
        )
    standing = np.zeros(n, dtype=bool)
    standing[20 : decisions[0] + 3] = True
    standing[110 : decisions[1] + 3] = True

    def path(x0):
        poses = np.array([[x0, 0.0, 0.0], [x0 + 0.04, 0.0, 0.0], [x0 + 0.08, 0.0, 0.0]])
        return poses, np.ones(3), np.zeros(3)

    revisions, corrections = [], []
    for k, (point, trig, ts, dec) in enumerate(
        ((1.0, 10, 20, decisions[0]), (0.6, 100, 110, decisions[1]))
    ):
        paths_ = {}
        for j, name in enumerate(("approach", "insert", "extract")):
            poses, dirs, curv = path(-2.0 + 0.3 * j + 0.001 * k)
            paths_[name] = {
                "sha256": NT.path_digest(poses, dirs, curv),
                "poses": poses.tolist(),
                "directions": dirs.tolist(),
                "curvatures_inv_m": curv.tolist(),
            }
        revisions.append(
            {
                "revision": k + 1,
                "time_s": float(t[dec]),
                "point_m": point,
                "target": [0.002 * (k + 1), 0.003 * (k + 1), 0.001],
                "previous_target": [0.0, 0.0, 0.0],
                "paths": paths_,
                "trackers": {name: p["sha256"] for name, p in paths_.items()},
            }
        )
        corrections.append(
            {
                "point_m": point,
                "trigger_s": float(t[trig]),
                "face_distance_m": (-0.3 - xs[trig]) - 0.95,
                "standing_s": [float(t[ts])],
                "moved_s": [],
                "accepted": True,
                "decision_s": float(t[dec]),
                "observation": {
                    "stamp_s": obs_stamps[k],
                    "pixel_s": obs_stamps[k] - L,
                    "source": "front",
                    "frame_version": 1,
                },
                "revision": k + 1,
                "closed_s": float(t[dec + 1]),
                "release_s": float(t[dec + 2]),
            }
        )
    ticks = [
        {
            "t": float(t[i]),
            "base": [float(xs[i]), 0.0, 0.0],
            "standing": bool(standing[i]),
            "path_revision": sum(1 for d in decisions if d < i),
        }
        for i in range(1, n - 1)
    ]
    for d in decisions:  # the release, two ticks after the decision
        ticks[d + 2 - 1]["released"] = True
    events = [{"event": "release", "time_s": float(t[d + 2])} for d in decisions]
    record = {
        "ticks": ticks,
        "corrections": corrections,
        "path_revisions": revisions,
        "events": events,
    }
    final = {
        name: {
            key: revisions[-1]["paths"][name][key]
            for key in ("poses", "directions", "curvatures_inv_m")
        }
        for name in ("approach", "insert", "extract")
    }
    return truth, record, {"fork_tip_x_m": 0.95, "episode": 1}, final, states


def viii(case, bounds):
    truth, record, start, final, states = case
    return J.check_viii(truth, record, start, final, states, bounds, -0.34)


def test_viii_two_corrections_decided_standing_and_followed_pass(bounds):
    out = viii(s4b_case(bounds), bounds)
    assert out["passed"], out["failures"]
    assert [c["point_m"] for c in out["corrections"]] == [1.0, 0.6]
    assert out["corrections"][0]["observation_to_decision_travel_m"] == pytest.approx(
        0.0, abs=1e-12
    )
    assert out["corrections"][1]["target_change"]["lateral_m"] == pytest.approx(0.006)


@pytest.mark.parametrize(
    "breaks",
    [
        "one_point",
        "refused",
        "pixel_before_t_s",
        "roof",
        "other_hold",
        "not_latest",
        "moving_after_t_s",
        "stood_before_t_s",
        "install_command",
        "release_command",
        "revision_row",
        "follower_copy",
        "final_paths",
        "late_trigger",
        "insert_phase",
        "release_moved",
        "dip_before_trigger",
        "release_not_first",
    ],
)
def test_viii_fails_each_broken_condition(bounds, breaks):
    truth, record, start, final, states = s4b_case(bounds)
    c0, c1 = record["corrections"]
    t = truth["stamp_s"]
    if breaks == "one_point":
        record["corrections"] = [c0]
    elif breaks == "refused":
        c1["accepted"] = False
    elif breaks == "pixel_before_t_s":
        late = float(t[int(np.searchsorted(t, c0["observation"]["pixel_s"] + 0.02))])
        c0["standing_s"] = [late]
        for row in record["ticks"]:
            if float(t[20]) <= row["t"] < late - 1e-9:
                row["standing"] = False
    elif breaks == "roof":
        c0["observation"]["source"] = "roof"
    elif breaks == "other_hold":
        c1["observation"]["frame_version"] = 2
    elif breaks == "not_latest":
        states.append(
            {
                **states[-1],
                "since_s": c0["decision_s"] - 0.001,
                "stamp_s": c0["decision_s"] - 0.1,
            }
        )
        states.sort(key=lambda s_: s_["since_s"])
    elif breaks == "moving_after_t_s":
        record["ticks"][39]["standing"] = False  # tick 40
    elif breaks == "stood_before_t_s":
        record["ticks"][18]["standing"] = True  # tick 19, before t_s at 20
    elif breaks == "install_command":
        truth["applied_speed_mps"][
            int(np.searchsorted(t, c0["decision_s"] - 1e-9)) + 1
        ] = 0.01
    elif breaks == "release_command":
        truth["applied_speed_mps"][
            int(np.searchsorted(t, c1["release_s"] - 1e-9)) + 1
        ] = 0.01
    elif breaks == "revision_row":
        record["ticks"][70]["path_revision"] = 0
    elif breaks == "follower_copy":
        record["path_revisions"][1]["trackers"]["insert"] = "0" * 64
    elif breaks == "final_paths":
        final["insert"] = {
            k: record["path_revisions"][0]["paths"]["insert"][k]
            for k in ("poses", "directions", "curvatures_inv_m")
        }
    elif breaks == "late_trigger":
        # Opened one tick late: the tick before already had the estimate within 1.0 m.
        truth["base_pose"][9, 0] = -2.2496
        record["ticks"][8]["base"][0] = -2.2496
    elif breaks == "insert_phase":
        truth["phase"][100] = list(PHASES).index("insert")
    elif breaks == "release_moved":
        # Codex S4b review P2: release_s one tick early, the real release row and event kept.
        c0["release_s"] -= TICK
    elif breaks == "dip_before_trigger":
        # Codex S4b review P2: the estimate dips inside 1.0 m at tick 7, out again, and the
        # point opens only at tick 10.
        dips = (1.00433, 0.9995, 1.00178, 1.0005, 0.9995)
        for i, d in zip(range(6, 11), dips, strict=True):
            truth["base_pose"][i, 0] = -1.25 - d
            record["ticks"][i - 1]["base"][0] = -1.25 - d
    elif breaks == "release_not_first":
        i_dec = int(np.searchsorted(t, c1["decision_s"] - 1e-9))
        record["ticks"][i_dec]["released"] = True  # tick i_dec + 1 released already
    out = viii((truth, record, start, final, states), bounds)
    assert not out["passed"], breaks


def test_viii_incomplete_records_are_unverified(bounds):
    cases = (
        "missing_row",
        "distance",
        "no_release",
        "nan_distance",
        "nan_pixel",
        "nan_target",
        "no_directions",
        "revision_zero",
        "final_short",
        "wrapped_target",
        "wrapped_stamp",
        "revision_float",
        "revision_inf",
    )
    for breaks in cases:
        truth, record, start, final, states = s4b_case(bounds)
        c0, c1 = record["corrections"]
        if breaks == "missing_row":
            del record["ticks"][44]  # tick 45, between t_s and the decision
        elif breaks == "distance":
            c0["face_distance_m"] += 0.001
        elif breaks == "no_release":
            del c1["release_s"]
        elif breaks == "nan_distance":  # Codex S4b review P2: NaN passed
            c0["face_distance_m"] = float("nan")
        elif breaks == "nan_pixel":
            c1["observation"]["pixel_s"] = float("nan")
        elif breaks == "nan_target":
            record["path_revisions"][0]["target"][0] = float("nan")
        elif breaks == "no_directions":  # a KeyError, not unverified, before
            del record["path_revisions"][1]["paths"]["insert"]["directions"]
        elif breaks == "revision_zero":
            c0["revision"] = 0
        elif (
            breaks == "wrapped_target"
        ):  # Codex S4b review P2: [[x, y, yaw]] passed the count
            record["path_revisions"][0]["target"] = [
                record["path_revisions"][0]["target"]
            ]
            record["path_revisions"][0]["previous_target"] = [
                record["path_revisions"][0]["previous_target"]
            ]
        elif breaks == "wrapped_stamp":
            c0["observation"]["stamp_s"] = [c0["observation"]["stamp_s"]]
            c0["observation"]["pixel_s"] = [c0["observation"]["pixel_s"]]
        elif breaks == "revision_float":  # int(1.9) was revision 1
            c0["revision"] = 1.9
        elif breaks == "revision_inf":
            c0["revision"] = float("inf")
        else:
            final["approach"] = {
                "poses": final["approach"]["poses"][:1],
                "directions": [1.0],
                "curvatures_inv_m": [0.0],
            }
        with pytest.raises(J.Unverified):
            viii((truth, record, start, final, states), bounds)


# (ix) -- plan D8 S4c-1 injections -------------------------------------------------------
def pocket_observation(x=1.6, y=0.05, psi=0.03, spacing=0.3725):
    from forklift_core.perception.pocket_observation import Pocket, PocketObservation

    axis = np.array([math.cos(psi), math.sin(psi)])
    normal = np.array([-axis[1], axis[0]])
    mid = np.array([x, y])
    pockets = [
        Pocket((*(mid + sign * spacing / 2 * normal), 0.061), 0.2275, 0.09)
        for sign in (1, -1)
    ]
    return PocketObservation(
        5,
        "synthetic",
        "base_link",
        "synthetic",
        "valid",
        *pockets,
        psi,
        0.001,
        0.001,
        None,
    )


def injection_case(kind="correction", lateral=0.02, yaw=0.0, drop=None):
    """A result and near record of an injected run: a far and a near capture, each biased as
    the runner does it."""
    from dataclasses import asdict

    sys.path.insert(0, str(ROOT / "sim" / "isaac"))
    import near_tracking as NT

    captures = []
    for attempt, phase, base in (
        (1, "far", (-3.0, 1.0, 0.1)),
        (2, "near", (-1.2, 0.4, 0.02)),
    ):
        raw = pocket_observation(x=3.0 if phase == "far" else 1.6)
        obs = (
            NT.biased_observation(raw, 0.6, lateral, yaw) if kind == "mismatch" else raw
        )
        raw_target = J._site_from_observation(asdict(raw), base, 0.6)
        target = J._biased_site(raw_target, lateral, yaw) if kind else raw_target
        captures.append(
            {
                "attempt": attempt,
                "phase": phase,
                "time_s": 10.0 + attempt,
                "capture_base": list(base),
                "raw_target": raw_target.tolist(),
                "target": target.tolist(),
                "planned_target": target.tolist(),
                "raw_observation": asdict(raw),
                "observation": asdict(obs),
            }
        )
    result = {
        "arguments": {
            "near_inject": kind,
            "near_inject_lateral_m": lateral,
            "near_inject_yaw_rad": yaw,
            "near_drop_frames_from_m": drop,
        },
        "geometry": {"pallet_depth_m": 0.6},
        "near_capture": {"status": "done"},
        "near_tracking": {
            "injection": {
                "kind": kind,
                "lateral_m": lateral,
                "yaw_rad": yaw,
                "drop_from_m": drop,
                "captures": captures if kind else [],
            },
            "start": {"observation": copy.deepcopy(captures[-1]["observation"])},
            "outcome": None,
        },
    }
    # The first correction's front observation (held pockets) and the revision it made:
    # the raw target plus a natural few millimetres, the bias gone.
    seen = np.asarray(captures[-1]["raw_target"]) + [0.001, -0.002, 0.0005]
    axis = np.array([math.cos(seen[2]), math.sin(seen[2])])
    normal = np.array([-axis[1], axis[0]])
    mid = seen[:2] - 0.3 * axis
    held = {
        "left": [*(mid + 0.18625 * normal), 0.061],
        "right": [*(mid - 0.18625 * normal), 0.061],
        "yaw": float(seen[2]),
    }
    near_ = {
        "events": [],
        "frames": [
            {
                "read_s": 20.1,
                "accepted": True,
                "latest_source": "front",
                "latest_stamp_s": 20.0,
                "latest_held": held,
            }
        ],
        "ticks": [],
        "corrections": [
            {
                "accepted": True,
                "revision": 1,
                "decision_s": 20.2,
                "observation": {"stamp_s": 20.0},
            },
            {
                "accepted": True,
                "revision": 2,
                "decision_s": 21.2,
                "observation": {"stamp_s": 21.0},
            },
        ],
        "path_revisions": [{"target": seen.tolist()}, {"target": seen.tolist()}],
    }
    return result, near_


def ix(result, near_, n=200):
    return J.check_ix(result, near_, truth_from(np.linspace(-2.0, -1.5, n)))


def test_ix_a_correction_run_with_the_bias_taken_off_passes():
    for lateral, yaw in ((0.02, 0.0), (0.0, -0.015)):
        out = ix(*injection_case("correction", lateral, yaw))
        assert out["status"] == "pass", out["failures"]
    assert J.check_ix({"near_tracking": {}}, {}, {})["applicable"] is False


@pytest.mark.parametrize(
    "breaks",
    [
        "arguments",
        "target_raw",
        "planner_raw",
        "start_biased",
        "observation_biased",
        "not_taken_off",
        "no_correction",
        "near_missing",
    ],
)
def test_ix_a_correction_run_fails_each_broken_condition(breaks):
    result, near_ = injection_case("correction", 0.0, 0.015)
    inj = result["near_tracking"]["injection"]
    caps = inj["captures"]
    if breaks == "arguments":
        result["arguments"]["near_inject_yaw_rad"] = -0.015
    elif breaks == "target_raw":
        caps[0]["target"] = caps[0]["raw_target"]
    elif breaks == "planner_raw":
        caps[1]["planned_target"] = caps[1]["raw_target"]
    elif breaks == "start_biased":
        from dataclasses import asdict

        sys.path.insert(0, str(ROOT / "sim" / "isaac"))
        import near_tracking as NT

        biased = asdict(NT.biased_observation(pocket_observation(), 0.6, 0.0, 0.015))
        result["near_tracking"]["start"]["observation"] = biased
    elif breaks == "observation_biased":
        caps[1]["observation"]["insertion_yaw_rad"] += 0.015
    elif breaks == "not_taken_off":  # the first correction kept the biased plan
        near_["path_revisions"][0]["target"] = caps[1]["target"]
    elif breaks == "no_correction":
        near_["corrections"] = []
    else:
        inj["captures"] = caps[:1]
    assert ix(result, near_)["status"] == "fail", breaks


def test_ix_a_refused_first_acceptance_is_not_run():
    result, near_ = injection_case("correction", 0.02, 0.0)
    result["near_capture"]["status"] = "pending"
    assert ix(result, near_)["status"] == "not_run"


def mismatch_events(frames, outcome="mismatch"):
    return [
        {
            "event": "mismatch",
            "stamp_s": frames[2]["stamp_s"],
            "time_s": frames[2]["read_s"],
        },
        {"event": "terminal_standing", "reason": outcome, "time_s": 10.9},
    ]


def mismatch_frames(
    reasons=("left_lateral", "left_lateral", "anchor_left_lateral"), counts=(1, 2, 3)
):
    frames = [
        {
            "read_s": 10.3 + 0.1 * i,
            "stamp_s": 10.29 + 0.1 * i,
            "rejected": True,
            "consecutive_rejects": c,
            "reason": r,
            "mismatch": i == 2,
        }
        for i, (r, c) in enumerate(zip(reasons, counts, strict=True))
    ]
    ticks = [
        {"t": 10.0 + TICK * i, "face_distance_m": 2.2 - 0.0005 * i}
        for i in range(1, 120)
    ]
    return frames, ticks


def test_ix_a_mismatch_run_with_three_rejections_and_the_terminal_passes():
    result, near_ = injection_case("mismatch", 0.06, 0.0)
    near_["frames"], near_["ticks"] = mismatch_frames()
    near_["events"] = mismatch_events(near_["frames"])
    result["near_tracking"]["outcome"] = "near_field_mismatch"
    out = ix(result, near_)
    assert out["status"] == "pass", out["failures"]


@pytest.mark.parametrize(
    "breaks",
    [
        "swapped_bias",
        "counts",
        "model_reason",
        "after_entry",
        "ended_lost",
        "no_event",
        "start_invalid",
    ],
)
def test_ix_a_mismatch_run_fails_each_broken_condition(breaks):
    if breaks == "swapped_bias":
        # Codex S4c-1 review P2: a yaw bias went in where a lateral one was asked for.
        result, near_ = injection_case("mismatch", 0.0, 0.05)
        inj = result["near_tracking"]["injection"]
        inj["lateral_m"], inj["yaw_rad"] = 0.06, 0.0
        (
            result["arguments"]["near_inject_lateral_m"],
            result["arguments"]["near_inject_yaw_rad"],
        ) = 0.06, 0.0
    else:
        result, near_ = injection_case("mismatch", 0.06, 0.0)
    near_["frames"], near_["ticks"] = mismatch_frames()
    near_["events"] = mismatch_events(near_["frames"])
    result["near_tracking"]["outcome"] = "near_field_mismatch"
    if breaks == "counts":
        near_["frames"], _ = mismatch_frames(counts=(1, 1, 2))
    elif breaks == "model_reason":
        near_["frames"], _ = mismatch_frames(
            reasons=("left_width", "left_lateral", "yaw")
        )
    elif breaks == "after_entry":
        for row in near_["ticks"]:
            row["face_distance_m"] = -0.01
    elif breaks == "ended_lost":
        result["near_tracking"]["outcome"] = "near_field_lost"
        near_["events"][-1]["reason"] = "lost"
    elif breaks == "no_event":  # Codex S4c-1 review P2: the event was not required
        near_["events"] = near_["events"][1:]
    elif breaks == "start_invalid":  # ... nor the observation's non-geometric fields
        result["near_tracking"]["start"]["observation"]["position_sigma_m"] = 1000.0
    assert ix(result, near_)["status"] == "fail", breaks


def lost_case(latch_i=60, drop=0.3):
    result, near_ = injection_case(None, 0.0, 0.0, drop)
    t = truth_from(np.linspace(-2.0, -1.5, 200))["stamp_s"]
    near_["ticks"] = [
        {
            "t": float(t[i]),
            "face_distance_m": 0.3 + 0.0005 * (latch_i - i) - 1e-6,
            "waits": [],
        }
        for i in range(1, 199)
    ]
    near_["ticks"][latch_i + 40 - 1]["waits"] = ["lost_wait"]
    near_["events"] = [
        {"event": "drop_latch", "time_s": float(t[latch_i])},
        {"event": "terminal_standing", "reason": "lost", "time_s": float(t[190])},
    ]
    near_["frames"] = [
        {"read_s": float(t[i]), "accepted": True} for i in range(5, latch_i, 12)
    ] + [
        {"read_s": float(t[i]), "skipped": "dropped"}
        for i in range(latch_i + 2, 190, 12)
    ]
    result["near_tracking"]["outcome"] = "near_field_lost"
    return result, near_


def test_ix_a_lost_run_latched_on_the_first_tick_within_d_passes():
    out = ix(*lost_case())
    assert out["status"] == "pass", out["failures"]
    assert out["dropped"] > 0


@pytest.mark.parametrize(
    "breaks",
    [
        "late_latch",
        "fed_after",
        "no_wait",
        "ended_mismatch",
        "not_reached",
        "earlier_dip",
    ],
)
def test_ix_a_lost_run_fails_each_broken_condition(breaks):
    result, near_ = lost_case()
    if breaks == "late_latch":
        near_["ticks"][59 - 1]["face_distance_m"] = (
            0.29  # the tick before was within D already
        )
    elif breaks == "fed_after":
        near_["frames"].append(
            {"read_s": near_["events"][0]["time_s"] + 0.2, "accepted": True}
        )
    elif breaks == "no_wait":
        for row in near_["ticks"]:
            row["waits"] = []
    elif breaks == "ended_mismatch":
        result["near_tracking"]["outcome"] = "near_field_mismatch"
    elif breaks == "earlier_dip":  # Codex S4c-1 review P2: within D earlier, out again
        near_["ticks"][20]["face_distance_m"] = 0.29
    else:
        near_["events"] = near_["events"][1:]
        assert ix(result, near_)["status"] == "not_run"
        return
    assert ix(result, near_)["status"] == "fail", breaks


def test_ix_an_incomplete_injection_record_is_unverified():
    result, near_ = injection_case("correction", 0.02, 0.0)
    del result["near_tracking"]["injection"]["captures"][0]["raw_observation"]
    with pytest.raises(J.Unverified):
        ix(result, near_)


def test_ix_a_frame_read_at_the_latch_time_was_fed_before_it():
    # Codex S4c-1 review P2: frames are read after the step and the next tick latches.
    result, near_ = lost_case()
    near_["frames"].append({"read_s": near_["events"][0]["time_s"], "accepted": True})
    assert ix(result, near_)["status"] == "pass"


def test_ix_an_asked_injection_must_be_recorded_with_all_its_arguments():
    result, near_ = injection_case("correction", 0.02, 0.0)
    del result["near_tracking"]["injection"]
    with pytest.raises(J.Unverified, match="did not record"):
        ix(result, near_)
    result, near_ = injection_case("correction", 0.02, 0.0)
    del result["arguments"]["near_inject_yaw_rad"]
    with pytest.raises(J.Unverified, match="lack"):
        ix(result, near_)


def test_s4c_lost_and_correction_need_the_correction_points_judged():
    contact_ok = {"contact": {"passed": True}}
    checks = {"ix": {"applicable": True, "kind": None, "status": "pass"}}
    assert J.s4c_outcome("pass", checks, {}, contact_ok, [])["status"] == "fail"
    checks["viii"] = {"passed": True}
    assert J.s4c_outcome("pass", checks, {}, contact_ok, [])["status"] == "pass"
    assert (
        J.s4c_outcome("pass", checks, {}, contact_ok, [{"pair": 1}])["status"] == "fail"
    )


def test_s4c_a_mismatch_terminal_needs_a_judged_stop_that_ends_standing():
    # Codex S4c-1 2nd review P2: with no stop at all (vii) passed vacuously.
    contact_ok = {"contact": {"passed": True}}
    mismatch = {
        "ix": {
            "applicable": True,
            "kind": "mismatch",
            "status": "pass",
            "terminal_s": 12.0,
        }
    }
    stop = {
        "source": "near_stop",
        "command_t": 11.0,
        "end_t": 12.5,
        "standing_at_end": True,
        "passed": True,
    }
    assert (
        J.s4c_outcome("pass", mismatch, {}, contact_ok, [], {"stops": [stop]})["status"]
        == "pass"
    )
    assert (
        J.s4c_outcome("pass", mismatch, {}, contact_ok, [], {"stops": []})["status"]
        == "fail"
    )
    for bad in (
        {"standing_at_end": False},
        {"passed": False},
        {"end_t": 11.5},
        {"source": "zero_step"},
    ):
        out = J.s4c_outcome(
            "pass", mismatch, {}, contact_ok, [], {"stops": [{**stop, **bad}]}
        )
        assert out["status"] == "fail", bad


def test_a_mismatch_section_must_start_with_the_approach_standing():
    truth = truth_from(
        np.linspace(-2.0, -1.9, 60), phase="approach", speed=np.zeros(60)
    )
    truth["base_pose"][:, 0] = -2.0  # standing through the warm-up
    t = truth["stamp_s"]
    near_ = {"ticks": [{"t": float(t[i])} for i in range(12, 59)]}
    assert J.mismatch_section(truth, near_, -0.34) == (12, 59)
    moved = {k: np.array(v, copy=True) for k, v in truth.items() if k != "phases"} | {
        "phases": truth["phases"]
    }
    moved["applied_speed_mps"][5] = 0.02  # a command before the first row
    with pytest.raises(J.Unverified, match="does not start"):
        J.mismatch_section(moved, near_, -0.34)
    # Codex S4c-1 2nd review P2: zero commands, yet the truck rolled 40 mm before the first row.
    rolled = {k: np.array(v, copy=True) for k, v in truth.items() if k != "phases"} | {
        "phases": truth["phases"]
    }
    rolled["base_pose"][:12, 0] = np.linspace(-2.04, -2.0, 12)
    with pytest.raises(J.Unverified, match="moved"):
        J.mismatch_section(rolled, near_, -0.34)


def test_ix_an_injection_record_missing_a_key_or_a_stamp_is_unverified():
    result, near_ = injection_case("correction", 0.02, 0.0)
    del result["near_tracking"]["injection"]["yaw_rad"]
    with pytest.raises(J.Unverified, match="lacks"):
        ix(result, near_)
    result, near_ = injection_case("mismatch", 0.06, 0.0)
    near_["frames"], near_["ticks"] = mismatch_frames()
    near_["events"] = mismatch_events(near_["frames"])
    result["near_tracking"]["outcome"] = "near_field_mismatch"
    del near_["frames"][2]["stamp_s"]
    del near_["events"][0]["stamp_s"]
    with pytest.raises(J.Unverified, match="stamp"):
        ix(result, near_)


def test_ix_the_correction_effect_uses_the_observation_the_decision_had():
    # Codex S4c-1 review P2: a frame read after the decision with the same stamp and pockets
    # fitted to the biased plan made a correction that kept the bias pass.
    result, near_ = injection_case("correction", 0.02, 0.0)
    caps = result["near_tracking"]["injection"]["captures"]
    near_["path_revisions"][0]["target"] = caps[1]["target"]
    biased = np.asarray(caps[1]["target"])
    axis = np.array([math.cos(biased[2]), math.sin(biased[2])])
    normal = np.array([-axis[1], axis[0]])
    mid = biased[:2] - 0.3 * axis
    fitted = {
        "left": [*(mid + 0.18625 * normal), 0.061],
        "right": [*(mid - 0.18625 * normal), 0.061],
        "yaw": float(biased[2]),
    }
    near_["frames"].append(
        {
            "read_s": 22.8,
            "accepted": True,
            "latest_source": "front",
            "latest_stamp_s": 20.0,
            "latest_held": fitted,
        }
    )
    assert ix(result, near_)["status"] == "fail"


def test_ix_broken_held_pockets_of_the_first_correction_are_unverified():
    result, near_ = injection_case("correction", 0.02, 0.0)
    del near_["frames"][0]["latest_held"]["left"]
    with pytest.raises(J.Unverified, match="held"):
        ix(result, near_)
    result, near_ = injection_case("correction", 0.02, 0.0)
    near_["frames"][0]["read_s"] = (
        20.5  # read after the decision: not the decision's observation
    )
    with pytest.raises(J.Unverified, match="latest accepted front frame"):
        ix(result, near_)


def test_ix_a_skipped_row_cannot_stand_in_for_the_decisions_observation():
    # Codex S4c-1 review P2: an accepted-and-skipped row read before the decision.
    result, near_ = injection_case("correction", 0.02, 0.0)
    near_["frames"].append(
        {**near_["frames"][0], "read_s": 20.15, "skipped": "uncovered"}
    )
    with pytest.raises(J.Unverified, match="both accepted and skipped"):
        ix(result, near_)


# (x) -- plan D8 S4c-2 the pickup retry ---------------------------------------------------
def retried_record(n=400, retry_i=150, next_i=260):
    """A near record of a retried pickup: episode 2 from tick 20 to the retry at tick
    retry_i, episode 3 from next_i on; distances 2.2 m falling in episode 2."""
    t = truth_from(np.linspace(-3.0, -1.5, n))["stamp_s"]
    ticks = [
        {"t": float(t[i]), "episode": 2, "face_distance_m": 2.2 - 0.0005 * (i - 20)}
        for i in range(20, retry_i + 1)
    ]
    ticks += [
        {"t": float(t[i]), "episode": 3, "face_distance_m": 2.2 - 0.001 * (i - next_i)}
        for i in range(next_i, n - 1)
    ]
    near_ = {
        "ticks": ticks,
        "frames": [
            {"read_s": float(t[i]), "episode": 2 if i <= retry_i else 3}
            for i in [*range(19, retry_i + 1, 12), *range(next_i - 1, n - 1, 12)]
        ],
        "events": [
            {"event": "tracker_built", "time_s": float(t[19])},
            {
                "event": "retry_start",
                "time_s": float(t[retry_i]),
                "episode": 2,
                "reason": "mismatch",
            },
            {"event": "tracker_built", "time_s": float(t[next_i - 1])},
            {"event": "protect", "time_s": float(t[n - 30])},
        ],
        "stop_events": [
            {"start_s": float(t[100]), "episode": 2},
            {"start_s": float(t[300]), "episode": 3},
        ],
        "corrections": [{"episode": 3, "point_m": 1.0}],
        "path_revisions": [{"episode": 3, "revision": 1}],
    }
    nt = {
        "starts": [
            {"episode": 2, "built_s": float(t[19])},
            {"episode": 3, "built_s": float(t[next_i - 1])},
        ]
    }
    return t, near_, nt


def test_x_a_retried_record_splits_into_its_episodes():
    t, near_, nt = retried_record()
    views = J.split_episodes(near_, nt)
    assert [v["episode"] for v in views] == [2, 3]
    assert views[0]["retry"]["reason"] == "mismatch" and views[1]["retry"] is None
    assert len(views[0]["near"]["stop_events"]) == 1 and views[1]["near"]["corrections"]
    assert (
        views[0]["near"]["protect"] is None and views[1]["near"]["protect"] is not None
    )
    assert max(f["read_s"] for f in views[0]["near"]["frames"]) <= float(t[150])
    for starts in ([], [*nt["starts"], {"episode": 4, "built_s": float(t[390])}]):
        with pytest.raises(J.Unverified, match="starts"):
            J.split_episodes(near_, {"starts": starts})
    # Every record placed, by its own hold (Codex S4c-2 implementation review P2).
    broken = {
        "a tick row after its retry": lambda nr: nr["ticks"].append(
            {"t": float(t[200]), "episode": 2, "face_distance_m": 1.0}
        ),
        "a frame read between the episodes": lambda nr: nr["frames"].append(
            {"read_s": float(t[200]), "episode": 2}
        ),
        "a frame of no episode": lambda nr: nr["frames"].append(
            {"read_s": float(t[300]), "episode": 7}
        ),
        "an accepted frame holding the other episode's observation": lambda nr: nr[
            "frames"
        ][1].update(accepted=True, latest_frame_version=3),
        "an event between the episodes": lambda nr: nr["events"].append(
            {"event": "stop", "time_s": float(t[200])}
        ),
    }
    for name, breaking in broken.items():
        _, near2, nt2 = retried_record()
        breaking(near2)
        with pytest.raises(J.Unverified):
            J.split_episodes(near2, nt2)
        assert name


def test_x_record_problems_allow_other_phases_only_inside_a_retry():
    truth = truth_from(np.linspace(-3.0, -1.5, 400), phase="approach")
    truth["phase"][160:250] = list(PHASES).index("observe")
    assert any("interrupts" in p for p in J.record_problems(truth))
    assert J.record_problems(truth, [(150, 260)]) == []
    assert any("interrupts" in p for p in J.record_problems(truth, [(170, 260)]))
    truth["stamp_s"][200] += 0.004  # a tick missing inside the retry is still caught
    assert any("contiguous" in p for p in J.record_problems(truth, [(150, 260)]))


def test_x_the_read_cadence_is_judged_within_the_episodes(monkeypatch):
    from types import SimpleNamespace

    from tools import d8b_calibration as CAL

    data = {
        "latency": {
            "read_delay_s": TICK,
            "render_period_s": 0.1,
            "band_s": [0.0844, 0.0989],
        }
    }
    calibration = {
        "dwells": {
            "1": [
                {
                    "bias_m": 1e-6,
                    "max_shift_m": 1e-5,
                    "distance_m": 2.5,
                    "distance_range_m": [2.5, 2.5],
                },
                {
                    "bias_m": 2e-6,
                    "max_shift_m": 1e-5,
                    "distance_m": 0.5,
                    "distance_range_m": [0.5, 0.5],
                },
            ]
        }
    }
    reads = [
        {"read_s": 10.0 + 0.1 * k + TICK, "stamp_s": 10.0 + 0.1 * k, "result": "ok"}
        for k in range(10)
    ]
    reads += [
        {"read_s": 20.0 + 0.1 * k + TICK, "stamp_s": 20.0 + 0.1 * k, "result": "ok"}
        for k in range(10)
    ]
    good = {
        "L_run_s": 0.0917,
        "frames": 200,
        "eligible": 150,
        "single_piece": 140,
        "multi_piece": 10,
        "mismatch": 0,
        "out_of_range": 0,
        "edge": 0,
        "not_eligible": {},
        "band_s": [0.088, 0.096],
    }
    monkeypatch.setattr(CAL, "analyse_frames", lambda run_, model, entries: [])
    monkeypatch.setattr(CAL, "summarise_run", lambda rows: dict(good))
    run = SimpleNamespace(reads=reads)
    assert not J.check_iii(run, data, calibration, None)[
        "cadence_ok"
    ]  # the retry's gap
    assert J.check_iii(
        run, data, calibration, None, spans=[(10.0, 11.0), (20.0, 21.0)]
    )["passed"]
    with pytest.raises(J.Unverified, match="outside every episode"):
        J.check_iii(run, data, calibration, None, spans=[(10.0, 11.0)])


def test_viii_an_episode_ended_by_a_refusal_judges_its_points_up_to_it(bounds):
    truth, record, start, final, states = s4b_case(bounds)
    t = truth["stamp_s"]
    c1 = record["corrections"][1]
    record["corrections"][1] = {
        k: c1[k] for k in ("point_m", "trigger_s", "face_distance_m")
    } | {
        "accepted": False,
        "decision_s": c1["decision_s"],
        "refused": "box",
        "standing_s": c1["standing_s"],
        "moved_s": [],
    }
    record["path_revisions"] = record["path_revisions"][:1]
    for row in record["ticks"]:
        row["path_revision"] = min(row["path_revision"], 1)
    record["events"] = record["events"][:1]
    for row in record["ticks"]:
        row["released"] = row.get("released") and row["t"] < c1["decision_s"]
    out = J.check_viii(
        truth, record, start, final, states, bounds, -0.34, ended="replan_rejected"
    )
    assert out["passed"], out["failures"]
    # Ended by a refusal yet the last point accepted: not the refusal the episode ended on.
    truth2, record2, start2, final2, states2 = s4b_case(bounds)
    assert not J.check_viii(
        truth2, record2, start2, final2, states2, bounds, -0.34, ended="replan_rejected"
    )["passed"]
    # A mismatch before any point: no corrections to judge.
    record3 = {**record2, "corrections": [], "path_revisions": []}
    for row in record3["ticks"]:
        row["path_revision"] = 0
    assert J.check_viii(
        truth2, record3, start2, final2, states2, bounds, -0.34, ended="mismatch"
    )["passed"]
    assert not J.check_viii(truth2, record3, start2, final2, states2, bounds, -0.34)[
        "passed"
    ]  # final: both needed
    del t


def test_ix_a_mismatch_episode_ended_in_a_retry_judges_the_retry_time():
    result, near_ = injection_case("mismatch", 0.06, 0.0)
    near_["frames"], near_["ticks"] = mismatch_frames()
    near_["events"] = mismatch_events(near_["frames"])[
        :1
    ]  # no terminal: the episode retried
    out = J.check_ix(
        result,
        near_,
        truth_from(np.linspace(-2.0, -1.5, 200)),
        retry={"reason": "mismatch", "time_s": 10.9},
    )
    assert out["status"] == "pass" and out["terminal_s"] == 10.9
    out = J.check_ix(
        result,
        near_,
        truth_from(np.linspace(-2.0, -1.5, 200)),
        retry={"reason": "replan_rejected", "time_s": 10.9},
    )
    assert out["status"] == "fail"
    # A correction run refused at its first point: (x) judges the retry, not 'no correction'.
    result, near_ = injection_case("correction", 0.0, -0.03)
    near_["corrections"], near_["path_revisions"] = [{"accepted": False}], []
    assert ix(result, near_)["status"] == "fail"
    assert (
        J.check_ix(
            result,
            near_,
            truth_from(np.linspace(-2.0, -1.5, 200)),
            retry={"reason": "replan_rejected", "time_s": 10.9},
        )["status"]
        == "pass"
    )


def retry_case(geometry_, pitch=0.0):
    """A retried pickup for (x): the truck backs from the pallet's face 2.5 m away, nothing in
    reach, the release and its re-projection recorded, the re-observation valid at attempt 3
    and planned on as placed, the first two captures biased."""
    from dataclasses import asdict

    from forklift_core.localization.slam_pose import compose, invert

    t, near_, nt = retried_record()
    near_["events"] = [e for e in near_["events"] if e["event"] != "protect"]
    n = len(t)
    # The truck stands from the retry (tick 150) to tick 170 (the re-observation at 165),
    # then drives on.
    xs = np.linspace(-3.0, -2.9, n)
    xs[150:171] = xs[150]
    xs[171:] -= xs[171] - xs[150]
    speed = np.full(n, 0.05)
    speed[150:171] = 0.0
    truth = truth_from(xs, phase="approach", speed=speed)
    if pitch:
        # A body pitched about its y axis throughout (q = q_yaw * q_pitch).
        hc, hs = math.cos(pitch / 2), math.sin(pitch / 2)
        for row in truth["base_pose"]:
            w, x, y, z = row[3:]
            row[3:] = [
                w * hc - y * hs,
                x * hc + z * hs,
                y * hc + w * hs,
                z * hc - x * hs,
            ]
    before, after = (-3.34, 0.01, 0.002), (-3.33, 0.03, 0.003)
    change = compose(after, invert(before))
    applied_old = (0.05, -0.02, 0.004)
    jump = (0.0224, 0.001)
    obs = asdict(pocket_observation(x=1.6))
    # The capture accepted the truth pose at tick 165; control took the estimate there, the
    # truth composed with the SLAM error recorded at the same instant.
    pose7 = truth["base_pose"][165]
    accepted = [pose7[:3].tolist(), pose7[3:].tolist()]
    err = {"along_m": 0.012, "lateral_m": -0.006, "yaw_rad": 0.002}
    yaw = J.yaw_of(pose7)
    rear_t = pose7[:2] - 0.34 * np.array([math.cos(yaw), math.sin(yaw)])
    c_, s_ = math.cos(yaw), math.sin(yaw)
    rear_e = rear_t + [c_ * 0.012 + s_ * 0.006, s_ * 0.012 - c_ * 0.006]
    yaw_e = yaw + 0.002
    cbase = (
        rear_e[0] + 0.34 * math.cos(yaw_e),
        rear_e[1] + 0.34 * math.sin(yaw_e),
        yaw_e,
    )
    target = J._site_from_observation(obs, cbase, 0.6).tolist()
    result = {
        "arguments": {"rear_axle_offset_m": -0.34},
        "geometry": {"pallet_depth_m": 0.6},
        "near_tracking": {
            "retry": {
                "reason": "mismatch",
                "capture_attempt": 3,
                "capture_status": "valid",
                "jump_m": jump[0],
                "jump_rad": jump[1],
                "frame_change": list(change),
                "rear_before": list(before),
                "rear_after": list(after),
                "applied_before": list(applied_old),
                "reprojected": {
                    "version": 9,
                    "applied": list(compose(change, applied_old)),
                    "reprojections": 3,
                },
                "capture_base": list(cbase),
                "observation": obs,
                "target": target,
                "planned_target": list(target),
            },
            "injection": {"captures": [{"attempt": 1}, {"attempt": 2}]},
        },
        "slam_summary": {
            "holds": [
                {"event": "hold", "time_s": float(t[10])},
                {
                    "event": "release_pickup_retry",
                    "time_s": float(t[150]),
                    "jump_m": jump[0],
                    "jump_rad": jump[1],
                },
            ]
        },
        "observation_attempts": [
            {},
            {},
            {
                "pocket_observation": copy.deepcopy(obs),
                "accepted_pose": accepted,
                "slam_error": err,
                "t_before_capture_s": float(t[163]),
                "capture_diagnostics": {
                    "capture_start_time_s": 101.0,
                    "physics_time_after_s": 101.0 + float(t[165] - t[163]),
                },
                "capture_control": {
                    "time_s": float(t[165]),
                    "control_rear": [float(rear_e[0]), float(rear_e[1]), float(yaw_e)],
                    "truth_rear": [float(rear_t[0]), float(rear_t[1]), float(yaw)],
                },
            },
        ],
        "obstacle_layer": {
            "unpermitted": [],
            "unpermitted_entries": 0,
            "reprojections": 3,
        },
    }
    return truth, near_, nt, result


def control_from(truth, err=(0.012, -0.006, 0.002)):
    """A control record (t, estimated rear x y yaw, true rear x y yaw) whose estimate is the
    truth composed with one SLAM error -- the stand's, as at the re-observation."""
    rows = []
    for t_, pose in zip(truth["stamp_s"], truth["base_pose"], strict=True):
        yaw = J.yaw_of(pose)
        c, s_ = math.cos(yaw), math.sin(yaw)
        rx, ry = pose[0] - 0.34 * c, pose[1] - 0.34 * s_
        ex, ey = rx + c * err[0] - s_ * err[1], ry + s_ * err[0] + c * err[1]
        rows.append([t_, ex, ey, yaw + err[2], rx, ry, yaw])
    return np.array(rows)


def check_x_(result, near_, truth, views, geometry, control=None):
    control = control_from(truth) if control is None else control
    return J.check_x(result, near_, truth, views, geometry, control=control)


def moved_capture(rs, dx=0.0, dyaw=0.0):
    """Move the re-observation's capture pose and rebuild its target and plan with it."""
    rec = rs["near_tracking"]["retry"]
    c = rec["capture_base"]
    rec["capture_base"] = [
        c[0] + dx * math.cos(c[2]),
        c[1] + dx * math.sin(c[2]),
        c[2] + dyaw,
    ]
    rec["target"] = J._site_from_observation(
        rec["observation"], rec["capture_base"], 0.6
    ).tolist()
    rec["planned_target"] = list(rec["target"])


def turned_accepted(rs, angle):
    """Turn the accepted truth heading by angle and rebuild the capture pose from it."""
    att = rs["observation_attempts"][2]
    w, x, y, z = att["accepted_pose"][1]
    half = angle / 2
    cw, sz = math.cos(half), math.sin(half)
    att["accepted_pose"][1] = [
        w * cw - z * sz,
        x * cw - y * sz,
        y * cw + x * sz,
        z * cw + w * sz,
    ]
    moved_capture(rs, dyaw=angle)


def test_x_a_clean_retry_passes_and_each_broken_condition_fails(geometry):
    truth, near_, nt, result = retry_case(geometry)
    views = J.split_episodes(near_, nt)
    out = check_x_(result, near_, truth, views, geometry)
    assert out["passed"], out["failures"]
    assert out["reason"] == "mismatch" and out["span_s"] > 0
    bad = {
        "protected": lambda tr, nr, rs: nr["events"].append(
            {"event": "protect", "time_s": nr["events"][1]["time_s"] - 0.5}
        ),
        "too_close": lambda tr, nr, rs: [
            r.__setitem__("face_distance_m", 0.5)
            for r in nr["ticks"]
            if r["episode"] == 2 and r["t"] == nr["events"][1]["time_s"]
        ],
        "invalid_reobservation": lambda tr, nr, rs: rs["observation_attempts"][
            2
        ].__setitem__("pocket_observation", {"status": "no_pallet"}),
        "biased_after": lambda tr, nr, rs: rs["near_tracking"]["injection"][
            "captures"
        ].append({"attempt": 3}),
        "unpermitted": lambda tr, nr, rs: rs["obstacle_layer"].update(
            unpermitted=[{"time_s": float(tr["stamp_s"][200])}], unpermitted_entries=1
        ),
        "no_release": lambda tr, nr, rs: rs["slam_summary"]["holds"].pop(),
        "release_jump_differs": lambda tr, nr, rs: rs["slam_summary"]["holds"][
            -1
        ].update(jump_m=0.5),
        "frame_change_not_the_estimates": lambda tr, nr, rs: rs["near_tracking"][
            "retry"
        ].update(frame_change=[1000.0, -1000.0, 3.14]),
        "not_reprojected": lambda tr, nr, rs: rs["near_tracking"]["retry"].update(
            reprojected=None
        ),
        # Codex S4c-2 implementation review 2 P2: a re-projection record without one.
        "no_reprojection_counted": lambda tr, nr, rs: [
            rs["near_tracking"]["retry"]["reprojected"].update(reprojections=0),
            rs["obstacle_layer"].update(reprojections=0),
        ],
        "grid_not_on_the_released_correction": lambda tr, nr, rs: rs["near_tracking"][
            "retry"
        ]["reprojected"]["applied"].__setitem__(0, 0.0),
        "grid_version_zero": lambda tr, nr, rs: rs["near_tracking"]["retry"][
            "reprojected"
        ].update(version=0),
        "planned_elsewhere": lambda tr, nr, rs: rs["near_tracking"]["retry"][
            "planned_target"
        ].__setitem__(1, 0.05),
        "target_not_its_observation": lambda tr, nr, rs: rs["near_tracking"]["retry"][
            "target"
        ].__setitem__(0, 0.0),
        "another_observation": lambda tr, nr, rs: rs["observation_attempts"][2][
            "pocket_observation"
        ]["left"].__setitem__("width_m", 0.3),
        "touching": lambda tr, nr, rs: tr["base_pose"].__setitem__(
            (slice(180, 190), 0), -0.85
        ),  # carriage 0.4 m in
        # X_RULES v3: the capture pose moved 1 m with its target and plan recomputed, and a
        # stand that drove (Codex S4c-3 stage-2 review P2-3).
        "capture_pose_moved": lambda tr, nr, rs: (
            rs["near_tracking"]["retry"]["capture_base"].__setitem__(
                0, rs["near_tracking"]["retry"]["capture_base"][0] + 1.0
            ),
            rs["near_tracking"]["retry"].update(
                target=J._site_from_observation(
                    rs["near_tracking"]["retry"]["observation"],
                    rs["near_tracking"]["retry"]["capture_base"],
                    0.6,
                ).tolist()
            ),
            rs["near_tracking"]["retry"].update(
                planned_target=list(rs["near_tracking"]["retry"]["target"])
            ),
        ),
        "slam_error_not_the_pose": lambda tr, nr, rs: rs["observation_attempts"][2][
            "slam_error"
        ].update(along_m=0.05),
        # Codex S4c-3 stage-2 review 2: a command before the re-observation hid 8 mm of
        # driving; a SLAM error and capture pose moved together by 1 m; the accepted heading
        # turned 0.3 rad with the chain rebuilt.
        "commanded_before_the_reobservation": lambda tr, nr, rs: (
            tr["applied_speed_mps"].__setitem__(151, 0.05),
            tr["base_pose"].__setitem__(
                (slice(158, 164), 0), tr["base_pose"][158:164, 0] + 0.004
            ),
        ),
        "error_and_pose_moved_together": lambda tr, nr, rs: (
            rs["observation_attempts"][2]["slam_error"].update(along_m=1.012),
            moved_capture(rs, dx=1.0),
        ),
        "accepted_heading_turned": lambda tr, nr, rs: turned_accepted(rs, 0.3),
        "a sample dated inside the capture": lambda tr, nr, rs: rs[
            "observation_attempts"
        ][2]["capture_control"].update(time_s=float(tr["stamp_s"][164])),
        "commanded_during_the_capture": lambda tr, nr, rs: tr[
            "applied_speed_mps"
        ].__setitem__(164, 0.05),
        "drove_during_the_stand": lambda tr, nr, rs: tr["base_pose"].__setitem__(
            (slice(158, 171), 0), tr["base_pose"][158:171, 0] + 0.004
        ),
        "lost_reason": lambda tr, nr, rs: [
            nr["events"][1].__setitem__("reason", "lost"),
            rs["near_tracking"]["retry"].__setitem__("reason", "lost"),
        ],
    }
    for name, breaking in bad.items():
        truth, near_, nt, result = retry_case(geometry)
        breaking(truth, near_, result)
        views = J.split_episodes(near_, nt)
        assert not check_x_(result, near_, truth, views, geometry)["passed"], name
    # A same-instant control row decides: the sample moved 20 mm with its chain rebuilt, a
    # 40 mm correction on the next row widening the bracket (Codex x-v3 review 3 P2-2).
    truth, near_, nt, result = retry_case(geometry)
    control = control_from(truth)
    control[166:, 1] += 0.04
    rec = result["observation_attempts"][2]
    rec["capture_control"]["control_rear"][0] += 0.02
    cr = rec["capture_control"]["control_rear"]
    tr_ = rec["capture_control"]["truth_rear"]
    c, s_ = math.cos(tr_[2]), math.sin(tr_[2])
    dx, dy = cr[0] - tr_[0], cr[1] - tr_[1]
    rec["slam_error"] = {
        "along_m": c * dx + s_ * dy,
        "lateral_m": -s_ * dx + c * dy,
        "yaw_rad": cr[2] - tr_[2],
    }
    result["near_tracking"]["retry"]["capture_base"] = [
        cr[0] + 0.34 * math.cos(cr[2]),
        cr[1] + 0.34 * math.sin(cr[2]),
        cr[2],
    ]
    moved_capture(result)
    assert not check_x_(
        result, near_, truth, J.split_episodes(near_, nt), geometry, control=control
    )["passed"]
    # A pitched body is not a false failure: the rear is the base less the offset along the heading.
    truth, near_, nt, result = retry_case(geometry, pitch=0.01)
    out = check_x_(result, near_, truth, J.split_episodes(near_, nt), geometry)
    assert out["passed"], out["failures"]
    # The control record moved 40 mm / 50 mrad at tick 160 and the capture kept the old estimate.
    truth, near_, nt, result = retry_case(geometry)
    control = control_from(truth)
    control[160:, 1] += 0.04
    control[160:, 3] += 0.05
    assert not check_x_(
        result, near_, truth, J.split_episodes(near_, nt), geometry, control=control
    )["passed"]
    incomplete = {
        "no re-observation attempt": lambda rs: rs["near_tracking"]["retry"].pop(
            "capture_attempt"
        ),
        "no permission record": lambda rs: rs.pop("obstacle_layer"),
        "no entry count": lambda rs: rs["obstacle_layer"].pop("unpermitted_entries"),
        # 50 kept before the retry, the 51st (cut) could lie in it.
        "a cut permission record": lambda rs: rs["obstacle_layer"].update(
            unpermitted=[{"time_s": 10.0}] * 50, unpermitted_entries=51
        ),
        "no SLAM record": lambda rs: rs.pop("slam_summary"),
        "no estimates": lambda rs: rs["near_tracking"]["retry"].pop("rear_before"),
        "no planned target": lambda rs: rs["near_tracking"]["retry"].pop(
            "planned_target"
        ),
        "no capture control sample": lambda rs: rs["observation_attempts"][2].pop(
            "capture_control"
        ),
        "a capture ending before the retry": lambda rs: rs["observation_attempts"][2][
            "capture_control"
        ].update(time_s=rs["slam_summary"]["holds"][0]["time_s"]),
        "no correction before the release": lambda rs: rs["near_tracking"]["retry"].pop(
            "applied_before"
        ),
        "a NaN grid correction": lambda rs: rs["near_tracking"]["retry"][
            "reprojected"
        ].update(applied=[math.nan] * 3),
        # Codex S4c-2 implementation review 2 P2: an entry with no time is not 'none'.
        "an unpermitted entry with no time": lambda rs: rs["obstacle_layer"].update(
            unpermitted=[{"phase": "observe"}], unpermitted_entries=1
        ),
        "an unpermitted entry at NaN": lambda rs: rs["obstacle_layer"].update(
            unpermitted=[{"time_s": math.nan}], unpermitted_entries=1
        ),
    }
    for name, breaking in incomplete.items():
        truth, near_, nt, result = retry_case(geometry)
        breaking(result)
        with pytest.raises(J.Unverified):
            check_x_(result, near_, truth, J.split_episodes(near_, nt), geometry)
        assert name
    # A cut record whose last kept entry is after the retry is complete over it.
    truth, near_, nt, result = retry_case(geometry)
    result["obstacle_layer"].update(
        unpermitted=[{"time_s": float(truth["stamp_s"][-1])}] * 50,
        unpermitted_entries=60,
    )
    assert check_x_(result, near_, truth, J.split_episodes(near_, nt), geometry)[
        "passed"
    ]


def test_ix_judges_the_injection_against_the_first_episodes_tracker_start():
    """A retried run's record keeps the last episode's start (Codex S4c-2 implementation
    review P2): (ix) is given the first episode's."""
    for kind, lateral in (("mismatch", 0.06), ("correction", 0.02)):
        result, near_ = injection_case(kind, lateral, 0.0)
        first = copy.deepcopy(result["near_tracking"]["start"])
        last = copy.deepcopy(first)
        x, y, z = last["observation"]["left"]["center_m"]
        last["observation"]["left"]["center_m"] = (x + 0.001, y, z)
        result["near_tracking"]["start"] = last
        if kind == "mismatch":
            near_["frames"], near_["ticks"] = mismatch_frames()
            near_["events"] = mismatch_events(near_["frames"])[:1]
        truth = truth_from(np.linspace(-2.0, -1.5, 200))
        retry = {"reason": "mismatch", "time_s": 10.9}
        stale = J.check_ix(result, near_, truth, retry=retry)
        assert stale["status"] == "fail", kind
        right = J.check_ix(result, near_, truth, retry=retry, start=first)
        assert right["status"] == "pass", (kind, right["failures"])


def test_a_later_episode_ended_before_its_armed_section_starts_at_its_first_row():
    """mismatch_section from a later episode's first approach tick (plan v4)."""
    xs = np.full(300, -3.0)
    truth = truth_from(xs, phase="approach", speed=np.zeros(300))
    truth["phase"][100:200] = list(PHASES).index("observe")
    near_ = {"ticks": [{"t": float(truth["stamp_s"][i])} for i in range(205, 260)]}
    assert J.mismatch_section(truth, near_, -0.34, end_index=259, begin_index=200) == (
        205,
        259,
    )
    truth["applied_speed_mps"][202] = 0.05  # driven before its first row
    with pytest.raises(J.Unverified, match="does not start"):
        J.mismatch_section(truth, near_, -0.34, end_index=259, begin_index=200)


def test_x_v2_a_run_that_ends_inside_its_retry_fails_rather_than_unverified(geometry):
    """X_RULES v2 (after the first smoke): no episode after the retry -- the run ended in its
    retry span (there: near_capture_misaligned at the second near capture) -- is a judged
    failure; the retry span runs to the record's end."""
    truth, near_, nt, result = retry_case(geometry)
    near_["ticks"] = [r for r in near_["ticks"] if r["episode"] == 2]
    near_["frames"] = [f for f in near_["frames"] if f["episode"] == 2]
    near_["events"] = near_["events"][:2]
    nt["starts"] = nt["starts"][:1]
    result["failure_reason"] = "near_capture_misaligned"
    views = J.split_episodes(near_, nt)
    assert len(views) == 1 and views[0]["retry"] is not None
    out = check_x_(result, near_, truth, views, geometry)
    assert not out["passed"]
    assert out["failures"] == [
        "the retry never reached a new episode (the run ended near_capture_misaligned)"
    ]
    assert out["span_s"] == pytest.approx(
        float(truth["stamp_s"][-1] - truth["stamp_s"][150])
    )
    # Ended before the re-observation: a failure as well, not an incomplete record.
    for drop in ("capture_attempt", "capture_status", "observation"):
        result["near_tracking"]["retry"].pop(drop)
    out = check_x_(result, near_, truth, views, geometry)
    assert "the run ended before the re-observation" in out["failures"]
    # Contact after the retry still counts, to the record's end.
    truth["base_pose"][380:390, 0] = -0.85
    assert any(
        "touched" in f
        for f in check_x_(result, near_, truth, views, geometry)["failures"]
    )
    # A retry with no re-observation recorded while a new episode exists is incomplete.
    truth, near_, nt, result = retry_case(geometry)
    result["near_tracking"]["retry"].pop("capture_attempt")
    with pytest.raises(J.Unverified):
        check_x_(result, near_, truth, J.split_episodes(near_, nt), geometry)


def test_x_v2_an_open_retry_checks_the_record_to_its_end(geometry):
    """Codex S4c-2 results review P2: the record after a retry that reached no new episode is
    checked contiguous and finite to its end, and a NaN there is unverified, not an error."""
    truth = truth_from(np.linspace(-3.0, -2.9, 400), phase="approach")
    truth["phase"][151:] = list(PHASES).index("observe")
    assert J.record_problems(truth, [], open_retry=True) == []
    gap = {
        k: (v if k == "phases" else np.delete(v, 250, axis=0)) for k, v in truth.items()
    }
    assert J.record_problems(gap, []) == []  # v1's range stopped at the retry
    assert any("contiguous" in p for p in J.record_problems(gap, [], open_retry=True))
    truth["lift_m"][250] = math.nan
    assert any("lift_m" in p for p in J.record_problems(truth, [], open_retry=True))
    truth, near_, nt, result = retry_case(geometry)
    near_["ticks"] = [r for r in near_["ticks"] if r["episode"] == 2]
    near_["frames"] = [f for f in near_["frames"] if f["episode"] == 2]
    near_["events"] = near_["events"][:2]
    nt["starts"] = nt["starts"][:1]
    truth["lift_m"][250] = math.nan
    with pytest.raises(J.Unverified, match="non-finite"):
        check_x_(result, near_, truth, J.split_episodes(near_, nt), geometry)


# S4c-3 stage 1 -- the re-draw acceptance at the near capture ---------------------------
def redraw_case(injection=None):
    """A near capture its box refused (heading 0.08 rad), accepted by the S4b re-draw: its
    record, the capture and the tracker's start on it."""
    from dataclasses import asdict, replace

    sys.path.insert(0, str(ROOT / "sim" / "isaac"))
    import near_tracking as NT

    from forklift_core.control.path_tracking import TrackerConfig
    from forklift_core.control.rollout import bicycle_rollout
    from forklift_core.perception.near_field_tracking import held_pockets
    from forklift_core.planning.geometry import Pose2D
    from forklift_core.planning.pallet_mission import (
        PalletSite,
        SyntheticMissionGeometry,
        site_poses,
        straight_from_pose,
    )

    geo = SyntheticMissionGeometry(alignment_straight_m=2.1)
    record = {
        "unloaded_footprint": asdict(geo.unloaded_footprint),
        **{
            k: getattr(geo, k)
            for k in (
                "pallet_depth_m",
                "pallet_width_m",
                "axle_to_fork_tip_m",
                "approach_gap_m",
                "alignment_straight_m",
                "delivery_straight_m",
                "extraction_m",
                "withdrawal_m",
                "spawn_clearance_m",
                "carriage_limit_m",
                "insertion_reserve_m",
                "prealign_offset_m",
                "approach_offset_m",
                "inserted_offset_m",
            )
        },
    }
    # The target first, then the rear 0.03 m along, 0.02 m across and 0.08 rad off its line,
    # the capture base that rear's (0.34 m ahead), and the observation that base sees.
    raw = np.array([5.0, 1.2, 0.15])
    sites_raw = site_poses(PalletSite(*raw), geo)
    pre = sites_raw["prealign"]
    c, s_ = math.cos(pre.yaw_rad), math.sin(pre.yaw_rad)
    rear = (
        pre.x_m + 0.03 * c - 0.02 * s_,
        pre.y_m + 0.03 * s_ + 0.02 * c,
        pre.yaw_rad + 0.08,
    )
    base = (
        rear[0] + 0.34 * math.cos(rear[2]),
        rear[1] + 0.34 * math.sin(rear[2]),
        rear[2],
    )
    psi = raw[2] - base[2]
    cb, sb = math.cos(base[2]), math.sin(base[2])
    dx, dy = raw[0] - base[0], raw[1] - base[1]
    centre = np.array([cb * dx + sb * dy, -sb * dx + cb * dy])
    mid = centre - geo.pallet_depth_m / 2 * np.array([math.cos(psi), math.sin(psi)])
    observation = pocket_observation(x=float(mid[0]), y=float(mid[1]), psi=float(psi))
    obs = asdict(observation)
    raw = J._site_from_observation(obs, base, geo.pallet_depth_m)
    target = raw if injection is None else J._biased_site(raw, *injection)
    sites = site_poses(PalletSite(*(float(v) for v in target)), geo)
    pre = sites["prealign"]
    straight, offsets = straight_from_pose(
        Pose2D(*rear),
        pre,
        sites["approach"],
        max_lateral_m=0.05,
        max_yaw_rad=0.12,
        min_length_m=0.3,
    )
    held = held_pockets(observation, base)
    widths = (observation.left.width_m, observation.right.width_m)
    tip_x = 1.29 - 0.34
    # The run's approach follower at the baseline and the driven cruise, rolled out from the
    # rear on the straight: their ends and their correction-point crossings.
    baseline_cfg = TrackerConfig(
        cruise_speed_mps=0.6,
        max_curvature_inv_m=0.36,
        max_acceleration_mps2=0.3,
        lookahead_m=0.28,
        position_tolerance_m=0.008,
        yaw_tolerance_rad=0.02,
        stop_speed_mps=0.012,
        max_cross_track_error_m=0.35,
        cusp_position_tolerance_m=0.03,
        cusp_yaw_tolerance_rad=0.05,
        cusp_brake_window_m=0.008,
        overshoot_tolerance_m=0.03,
    )
    configs = [baseline_cfg, replace(baseline_cfg, cruise_speed_mps=0.055)]
    rollouts = [
        bicycle_rollout(
            straight.poses,
            straight.directions,
            straight.curvatures_inv_m,
            c,
            np.asarray(rear),
        )
        for c in configs
    ]
    end_ = sites["approach"]
    u_end = np.array([math.cos(end_.yaw_rad), math.sin(end_.yaw_rad)])
    dries = []
    for ro in rollouts:
        last = np.asarray(ro.trajectory[-1])
        rel = last[:2] - [end_.x_m, end_.y_m]
        dries.append(
            {
                "status": ro.status,
                "end_ahead_m": float(rel @ u_end),
                "end_lateral_m": float(rel @ [-u_end[1], u_end[0]]),
                "end_yaw_rad": math.atan2(
                    math.sin(last[2] - end_.yaw_rad), math.cos(last[2] - end_.yaw_rad)
                ),
                "swept_clear": True,
            }
        )
    points, inside = NT.correction_points_check(
        [ro.trajectory for ro in rollouts],
        pre,
        NT.rollout_face_distance(held, widths, tip_x, -0.34),
    )
    assert inside

    def two(a, b, direction):
        return {
            "poses": [[sites[k].x_m, sites[k].y_m, sites[k].yaw_rad] for k in (a, b)],
            "directions": [direction, direction],
            "curvatures_inv_m": [0.0, 0.0],
        }

    installed = {
        "approach": {
            "poses": np.asarray(straight.poses).tolist(),
            "directions": np.asarray(straight.directions).tolist(),
            "curvatures_inv_m": np.asarray(straight.curvatures_inv_m).tolist(),
        },
        "insert": two("approach", "inserted", 1),
        "extract": two("inserted", "extracted", -1),
    }
    for path in installed.values():
        path["sha256"] = path["tracker_sha256"] = NT.path_digest(
            path["poses"], path["directions"], path["curvatures_inv_m"]
        )
    entry = {
        "stage": "redraw",
        "attempt": 2,
        "time_s": 20.0,
        "capture_base": list(base),
        "rear": list(rear),
        "observation": obs,
        "raw_target": raw.tolist(),
        "target": np.asarray(target).tolist(),
        "line": [[p.x_m, p.y_m, p.yaw_rad] for p in (pre, sites["approach"])],
        "dry_run_cruise_mps": [0.6, 0.055],
        "fork_tip_x_m": tip_x,
        "dry_run_configs": [asdict(c) for c in configs],
        "record": {"offsets": offsets, "refused": None, "dry_runs": dries},
        "correction_points": points,
        "accepted": True,
        "installed": installed,
    }
    result = {
        "arguments": {
            "approach_straight_speed_mps": 0.055,
            "rear_axle_offset_m": -0.34,
        },
        "settings_synthetic": {"approach_speed_mps": 0.6},
        "tracker_configs": {"approach": asdict(configs[1])},
        "geometry": record,
        "near_capture": {
            "status": "done",
            "near_attempt": 2,
            "offsets_to_new_line": {
                "along_m": 0.03,
                "lateral_m": 0.02,
                "yaw_rad": 0.08,
                "length_left_m": 2.07,
            },
            "recoveries": [entry],
        },
    }
    nt = {
        "starts": [
            {
                "attempt": 2,
                "built_s": 20.0,  # the same tick as the re-draw
                "pose": list(base),
                "observation": copy.deepcopy(obs),
                "fork_tip_x_m": tip_x,
                "paths0": {k: v["tracker_sha256"] for k, v in installed.items()},
            }
        ],
        "injection": None
        if injection is None
        else {
            "kind": "correction",
            "lateral_m": injection[0],
            "yaw_rad": injection[1],
            "captures": [{"attempt": 1}, {"attempt": 2}],
        },
    }
    return result, nt


def test_redraw_a_stage_one_acceptance_with_its_chain_passes_and_each_broken_link_fails():
    sys.path.insert(0, str(ROOT / "sim" / "isaac"))
    import near_tracking as NT

    result, nt = redraw_case()
    out = J.check_near_redraw(result, nt)
    assert out["passed"], out["failures"]
    assert [
        pt["point_m"]
        for pt in result["near_capture"]["recoveries"][0]["correction_points"]
    ] == [1.0, 0.6] * 2
    # A correction injection on this attempt moves the target (same tick as the first build).
    result, nt = redraw_case(injection=(0.02, 0.0))
    assert J.check_near_redraw(result, nt)["passed"]
    nt["injection"]["captures"] = [
        {"attempt": 1}
    ]  # ... not on this attempt: not the raw target
    assert not J.check_near_redraw(result, nt)["passed"]

    def e(r):
        return r["near_capture"]["recoveries"][0]

    def redigest(path):
        path["sha256"] = path["tracker_sha256"] = NT.path_digest(
            path["poses"], path["directions"], path["curvatures_inv_m"]
        )
        return path["sha256"]

    def shift_approach(r, n):
        path = e(r)["installed"]["approach"]
        path["poses"][3][1] += (
            0.01  # consistent digests, but not the straight from the rear
        )
        n["starts"][0]["paths0"]["approach"] = redigest(path)

    def detour_insert(r, n):
        path = e(r)["installed"]["insert"]
        a, b = np.asarray(path["poses"][0]), np.asarray(path["poses"][1])
        path["poses"] = [
            a.tolist(),
            ((a + b) / 2 + [0.0, 1.0, 0.0]).tolist(),
            b.tolist(),
        ]
        path["directions"], path["curvatures_inv_m"] = [1, 1, 1], [0.0, 0.2, 0.0]
        n["starts"][0]["paths0"]["insert"] = redigest(path)

    def shift_points(r, n):
        # Every crossing pose and the pose before it moved 0.1 m across the line, the recorded
        # offsets kept (Codex S4c-3 implementation review 2 P2).
        line_yaw = e(r)["line"][0][2]
        normal = np.array([-math.sin(line_yaw), math.cos(line_yaw)]) * 0.1
        for pt in e(r)["correction_points"]:
            for key in ("pose", "previous_pose"):
                pt[key] = [pt[key][0] + normal[0], pt[key][1] + normal[1], pt[key][2]]

    def points_from_the_plan(r, n):
        # Crossings taken from the planned straight instead of the roll-outs, each consistent
        # in itself (Codex S4c-3 implementation review 3 P2).
        from forklift_core.perception.near_field_tracking import held_pockets
        from forklift_core.planning.geometry import Pose2D
        from tools.d8c_replay import observation_from_dict

        ent = e(r)
        held = held_pockets(
            observation_from_dict(ent["observation"]), tuple(ent["capture_base"])
        )
        obs = ent["observation"]
        widths = (obs["left"]["width_m"], obs["right"]["width_m"])
        poses = ent["installed"]["approach"]["poses"]
        ent["correction_points"], _ = NT.correction_points_check(
            [poses, poses],
            Pose2D(*ent["line"][0]),
            NT.rollout_face_distance(held, widths, ent["fork_tip_x_m"], -0.34),
        )

    def not_the_crossing(r, n):
        pt = e(r)["correction_points"][0]
        pt["previous_pose"], pt["previous_face_distance_m"] = (
            pt["pose"],
            pt["face_distance_m"],
        )

    broken = {
        "outside the box": lambda r, n: e(r)["record"]["offsets"].update(
            lateral_m=0.06
        ),
        "a refusal recorded": lambda r, n: e(r)["record"].update(refused="dry_run"),
        "not the baseline and driven cruise": lambda r, n: e(r).update(
            dry_run_cruise_mps=[0.6, 0.3]
        ),
        "a dry run not clear": lambda r, n: e(r)["record"]["dry_runs"][1].update(
            swept_clear=False
        ),
        "a dry run past the goal": lambda r, n: e(r)["record"]["dry_runs"][0].update(
            end_ahead_m=0.01
        ),
        "one dry run": lambda r, n: e(r)["record"]["dry_runs"].pop(),
        "a correction point outside": lambda r, n: e(r)["correction_points"][2][
            "offsets"
        ].update(lateral_m=0.06),
        "a correction point missing": lambda r, n: e(r)["correction_points"].pop(),
        "a correction point not the crossing": not_the_crossing,
        "a correction distance not recomputed": lambda r, n: e(r)["correction_points"][
            3
        ].update(face_distance_m=0.5),
        "a correction point offset not its pose's": shift_points,
        "the rear not the capture base's": lambda r, n: e(r)["rear"].__setitem__(
            2, e(r)["rear"][2] + 0.12
        ),
        "the correction points from the plan, not the roll-outs": points_from_the_plan,
        "a dry run's end not its roll-out's": lambda r, n: e(r)["record"]["dry_runs"][
            1
        ].update(end_lateral_m=0.001),
        "a dry run's status not its roll-out's": lambda r, n: e(r)["record"][
            "dry_runs"
        ][0].update(status="stalled"),
        "both dry-run settings not the run's follower": lambda r, n: [
            c.update(max_curvature_inv_m=0.40) for c in e(r)["dry_run_configs"]
        ],
        "the dry-run settings of two followers": lambda r, n: e(r)["dry_run_configs"][
            1
        ].update(lookahead_m=0.3),
        "the raw target not the observation's": lambda r, n: e(r)[
            "raw_target"
        ].__setitem__(0, 9.0),
        "the target not the raw target": lambda r, n: e(r)["target"].__setitem__(
            1, 0.0
        ),
        "the line not the target's": lambda r, n: e(r)["line"][0].__setitem__(0, 0.0),
        "the approach not the straight": shift_approach,
        "an insert detour with consistent digests": detour_insert,
        "the insert not the target's": lambda r, n: e(r)["installed"]["insert"][
            "poses"
        ][1].__setitem__(0, 0.0),
        "a follower copy differing": lambda r, n: e(r)["installed"]["extract"].update(
            tracker_sha256="0" * 64
        ),
        "the tracker not started on it": lambda r, n: n["starts"][0]["paths0"].update(
            insert="0" * 64
        ),
        "no tracker for the attempt": lambda r, n: n["starts"][0].update(attempt=3),
        "the start's observation another": lambda r, n: n["starts"][0]["observation"][
            "left"
        ].update(center_m=[2.6, 0.24, 0.061]),
        "the start's pose another": lambda r, n: n["starts"][0]["pose"].__setitem__(
            1, 0.52
        ),
        "the entry deleted": lambda r, n: r["near_capture"].update(recoveries=[]),
        "relabelled refused with paths installed": lambda r, n: e(r).update(
            accepted=False
        ),
        "a re-draw for a capture its box accepted": lambda r, n: (
            r["near_capture"]["offsets_to_new_line"].update(yaw_rad=0.01)
            or r["near_capture"].update(
                dry_run={
                    "status": "arrived",
                    "position_error_m": 0.001,
                    "yaw_error_rad": 0.001,
                }
            )
        ),
    }
    for name, breaking in broken.items():
        result, nt = redraw_case()
        breaking(result, nt)
        assert not J.check_near_redraw(result, nt)["passed"], name
    # A refused stage 1 (the run then ended: no done capture, no installation, no tracker) passes.
    result, nt = redraw_case()
    e(result).update(accepted=False)
    e(result).pop("installed")
    e(result)["record"].update(refused="box")
    result["near_capture"]["status"] = "pending"
    nt["starts"] = []
    assert J.check_near_redraw(result, nt)["passed"]
    # No refusal and no re-draw: not applicable.
    result, nt = redraw_case()
    result["near_capture"]["recoveries"] = []
    result["near_capture"]["offsets_to_new_line"]["yaw_rad"] = 0.01
    result["near_capture"]["dry_run"] = {
        "status": "arrived",
        "position_error_m": 0.001,
        "yaw_error_rad": 0.001,
    }
    assert J.check_near_redraw(result, nt) == {"applicable": False}
    # A smoke's narrowed heading box (0.01 rad) sends a capture beyond it to the re-draw and
    # only such a capture; the capture records the box, which must be the run's.
    result, nt = redraw_case()
    result["arguments"]["near_capture_box_yaw_rad"] = 0.01
    result["near_capture"]["box"] = {
        "lateral_m": 0.05,
        "yaw_rad": 0.01,
        "length_m": 0.3,
    }
    assert J.check_near_redraw(result, nt)["passed"]
    result["near_capture"]["offsets_to_new_line"]["yaw_rad"] = (
        0.008  # inside: no re-draw due
    )
    result["near_capture"]["dry_run"] = {
        "status": "arrived",
        "position_error_m": 0.001,
        "yaw_error_rad": 0.001,
    }
    assert not J.check_near_redraw(result, nt)["passed"]
    result, nt = redraw_case()
    result["arguments"]["near_capture_box_yaw_rad"] = 0.01
    result["near_capture"]["box"] = {
        "lateral_m": 0.05,
        "yaw_rad": 0.02,
        "length_m": 0.3,
    }
    with pytest.raises(J.Unverified, match="box"):
        J.check_near_redraw(result, nt)
    result["arguments"]["near_capture_box_yaw_rad"] = 0.2
    with pytest.raises(J.Unverified, match="heading box"):
        J.check_near_redraw(result, nt)
    for drop in ("installed", "rear", "observation", "fork_tip_x_m"):
        result, nt = redraw_case()
        e(result).pop(drop)
        with pytest.raises(J.Unverified):
            J.check_near_redraw(result, nt)
    # A NaN in a correction point is unverified, not a pass (Codex S4c-3 implementation P2).
    result, nt = redraw_case()
    e(result)["correction_points"][1]["offsets"].update(yaw_rad=math.nan)
    with pytest.raises(J.Unverified, match="correction-point offsets"):
        J.check_near_redraw(result, nt)


# (xi) -- plan D8 S4c-3 stage 2, the realignment ----------------------------------------
def recapture_on_line(along=0.05, lateral=0.01, yaw=0.01):
    """A mission geometry record (alignment straight 2.1 m), and a capture whose rear axle is
    `along` past the prealign of a target, `lateral` across and `yaw` off its line: the base,
    the observation that base sees of the target, and the straight's offsets from that rear."""
    from dataclasses import asdict

    from forklift_core.planning.geometry import Pose2D
    from forklift_core.planning.pallet_mission import (
        PalletSite,
        SyntheticMissionGeometry,
        site_poses,
        straight_from_pose,
    )

    geo = SyntheticMissionGeometry(alignment_straight_m=2.1)
    record = {
        "unloaded_footprint": asdict(geo.unloaded_footprint),
        **{
            k: getattr(geo, k)
            for k in (
                "pallet_depth_m",
                "pallet_width_m",
                "axle_to_fork_tip_m",
                "approach_gap_m",
                "alignment_straight_m",
                "delivery_straight_m",
                "extraction_m",
                "withdrawal_m",
                "spawn_clearance_m",
                "carriage_limit_m",
                "insertion_reserve_m",
                "prealign_offset_m",
                "approach_offset_m",
                "inserted_offset_m",
            )
        },
    }
    raw = np.array([2.0, 0.3, 0.02])
    pre = site_poses(PalletSite(*raw), geo)["prealign"]
    c, s_ = math.cos(pre.yaw_rad), math.sin(pre.yaw_rad)
    rear = (
        pre.x_m + along * c - lateral * s_,
        pre.y_m + along * s_ + lateral * c,
        pre.yaw_rad + yaw,
    )
    base = (
        rear[0] + 0.34 * math.cos(rear[2]),
        rear[1] + 0.34 * math.sin(rear[2]),
        rear[2],
    )
    psi = raw[2] - base[2]
    cb, sb = math.cos(base[2]), math.sin(base[2])
    dx, dy = raw[0] - base[0], raw[1] - base[1]
    centre = np.array([cb * dx + sb * dy, -sb * dx + cb * dy])
    mid = centre - geo.pallet_depth_m / 2 * np.array([math.cos(psi), math.sin(psi)])
    obs = asdict(pocket_observation(x=float(mid[0]), y=float(mid[1]), psi=float(psi)))
    target = J._site_from_observation(obs, base, geo.pallet_depth_m)
    sites = site_poses(PalletSite(*(float(v) for v in target)), geo)
    _, offsets = straight_from_pose(
        Pose2D(*rear),
        sites["prealign"],
        sites["approach"],
        max_lateral_m=0.05,
        max_yaw_rad=0.05,
        min_length_m=0.3,
    )
    return record, list(base), obs, offsets


def realign_case(geometry_):
    """A near capture (attempt 2, tick 150) refused by its box and by stage 1, realigned once:
    the release at its tick, the stand and in-place re-observation (attempt 3, tick 165),
    the near leg installed at 171, the re-capture (attempt 4) accepted at tick 300 with the
    straight its pose and observation give."""
    sys.path.insert(0, str(ROOT / "sim" / "isaac"))
    import near_tracking as NT

    truth, _near, _nt, result = retry_case(geometry_)
    record, base4, obs4, offsets4 = recapture_on_line()
    result["geometry"] = record
    t = truth["stamp_s"]
    rec = result["near_tracking"].pop("retry")
    rec.pop("reason")

    def install(n, reason, poses, at):
        poses = np.asarray(poses, dtype=float)
        dirs, curvs = np.ones(len(poses), dtype=np.int8), np.zeros(len(poses))
        digest = NT.path_digest(poses, dirs, curvs)
        return {
            "n": n,
            "time_s": float(t[at]),
            "reason": reason,
            "slam_mode": "tracking",
            "length_m": float(np.sum(np.hypot(*np.diff(poses[:, :2], axis=0).T))),
            "poses": poses.tolist(),
            "directions": dirs.tolist(),
            "curvatures_inv_m": curvs.tolist(),
            "sha256": digest,
            "tracker_sha256": digest,
        }

    leg = [[-3.3 + 0.1 * k, 0.0, 0.0] for k in range(31)]  # 3.0 m
    rec.update(
        time_s=float(t[150]),
        refused_attempt=2,
        installs=[
            install(0, "stand", [[-3.39, 0.03, 0.003], [-3.34, 0.03, 0.003]], 150),
            install(1, "near_leg", leg, 171),
        ],
        end={
            "reason": "recaptured",
            "attempt": 4,
            "time_s": float(t[300]),
            "length_left_m": offsets4["length_left_m"],
        },
    )
    rec["end"]["observe_tracker_sha256"] = rec["installs"][-1]["tracker_sha256"]
    # The refused capture's attempt at the realignment's tick; the re-capture: attempt 4 at
    # tick 300, accepted by its box with the straight from its rear (~2.05 m), a tracker
    # started on it then.
    result["observation_attempts"][1]["t_before_capture_s"] = float(t[150])
    result["observation_attempts"].append(
        {"t_before_capture_s": float(t[300]), "pocket_observation": {"status": "valid"}}
    )
    result["near_tracking"] = {
        "realign": rec,
        "realigns": 1,
        "starts": [
            {
                "attempt": 4,
                "built_s": float(t[300]),
                "aligned_s": float(t[300]),
                "pose": base4,
                "observation": obs4,
            }
        ],
    }
    result["slam_summary"]["holds"][-1]["event"] = "release_near_realign"
    result["near_capture_refused"] = [
        {
            "status": "pending",
            "refused_attempt": 2,
            "offsets_to_new_line": {
                "along_m": 0.02,
                "lateral_m": 0.02,
                "yaw_rad": 0.13,
                "length_left_m": 2.08,
            },
            "recoveries": [
                {
                    "stage": "redraw",
                    "attempt": 2,
                    "accepted": False,
                    "record": {"refused": "box"},
                }
            ],
        }
    ]
    result["near_capture"] = {
        "status": "done",
        "near_attempt": 4,
        "offsets_to_new_line": dict(offsets4),
    }
    result["arguments"]["near_capture_redraw"] = "on"
    return truth, result["near_tracking"], result


def check_xi_(result, nt, truth, geometry, control=None):
    control = control_from(truth) if control is None else control
    return J.check_xi(result, nt, truth, geometry, control)


def test_xi_a_clean_realignment_passes_and_each_broken_condition_fails(geometry):
    truth, nt, result = realign_case(geometry)
    out = check_xi_(result, nt, truth, geometry)
    assert out["passed"], out["failures"]
    assert out["installs"] == ["stand", "near_leg"] and out[
        "longest_install_m"
    ] == pytest.approx(3.0)

    def rec(rs):
        return rs["near_tracking"]["realign"]

    def long_install(tr, rs):
        sys.path.insert(0, str(ROOT / "sim" / "isaac"))
        import near_tracking as NT

        poses = np.array([[-3.3 + 0.1 * k, 0.0, 0.0] for k in range(71)])  # 7.0 m
        dirs, curvs = np.ones(71, dtype=np.int8), np.zeros(71)
        digest = NT.path_digest(poses, dirs, curvs)
        rec(rs)["installs"].append(
            {
                **rec(rs)["installs"][1],
                "n": 2,
                "poses": poses.tolist(),
                "directions": dirs.tolist(),
                "curvatures_inv_m": curvs.tolist(),
                "length_m": 7.0,
                "sha256": digest,
                "tracker_sha256": digest,
            }
        )

    def short_recapture(tr, rs):
        _, base, obs, offsets = recapture_on_line(along=1.7)
        assert offsets["length_left_m"] == pytest.approx(0.4, abs=1e-6)
        rs["near_tracking"]["starts"][0].update(pose=base, observation=obs)

    bad = {
        "two realignments": lambda tr, rs: rs["near_tracking"].update(realigns=2),
        "refused capture missing": lambda tr, rs: rs.update(near_capture_refused=[]),
        "refused capture accepted by its box": lambda tr, rs: (
            rs["near_capture_refused"][0]["offsets_to_new_line"].update(yaw_rad=0.01)
            or rs["near_capture_refused"][0].update(
                dry_run={
                    "status": "arrived",
                    "position_error_m": 0.001,
                    "yaw_error_rad": 0.001,
                }
            )
        ),
        "no stage-1 refusal": lambda tr, rs: rs["near_capture_refused"][0].update(
            recoveries=[]
        ),
        "stage 1 accepted": lambda tr, rs: rs["near_capture_refused"][0]["recoveries"][
            0
        ].update(accepted=True),
        "stage 1 in a redraw-off run": lambda tr, rs: rs["arguments"].update(
            near_capture_redraw="off"
        ),
        "never re-captured": lambda tr, rs: rec(rs).update(end=None),
        "re-capture short": lambda tr, rs: rec(rs)["end"].update(length_left_m=1.4),
        "re-capture not accepted": lambda tr, rs: rs["near_capture"].update(
            status="pending"
        ),
        "install too long": long_install,
        "install held": lambda tr, rs: rec(rs)["installs"][1].update(
            slam_mode="holding"
        ),
        "install not its content": lambda tr, rs: rec(rs)["installs"][1].update(
            tracker_sha256="0" * 64
        ),
        "installs without the stand": lambda tr, rs: rec(rs).update(
            installs=rec(rs)["installs"][1:]
        ),
        "held recovery": lambda tr, rs: rs.update(
            failure_reason="near_capture_realign_held_recovery"
        ),
        "release missing": lambda tr, rs: rs["slam_summary"]["holds"].pop(),
        "re-observation invalid": lambda tr, rs: rec(rs).update(
            capture_status="no_pallet"
        ),
        # Codex S4c-3 stage-2 implementation review: an install left out, the re-capture's time
        # moved before the re-observation, its straight 0.4 m by its acceptance, an observe
        # replan recorded without its installation.
        "the near leg left out": lambda tr, rs: rec(rs).update(
            installs=rec(rs)["installs"][:1]
        ),
        "the re-capture's time moved": lambda tr, rs: rec(rs)["end"].update(
            time_s=float(tr["stamp_s"][156])
        ),
        "the re-capture's straight short by its acceptance": lambda tr, rs: rs[
            "near_capture"
        ]["offsets_to_new_line"].update(length_left_m=0.4),
        "an observe replan without its installation": lambda tr, rs: rs.update(
            observe_replans=[{"time_s": float(tr["stamp_s"][200]), "status": "success"}]
        ),
        "no tracker on the re-capture": lambda tr, rs: rs["near_tracking"].update(
            starts=[]
        ),
        "contact during the realignment": lambda tr, rs: tr["base_pose"].__setitem__(
            (slice(200, 210), 0), -0.85
        ),
        "the re-capture came by another follower": lambda tr, rs: rec(rs)["end"].update(
            observe_tracker_sha256="1" * 64
        ),
        # Codex S4c-3 stage-2 implementation review 2: the end and its attempt moved before
        # the tracker started (a violation at 200 left outside), the start moved after the
        # refused capture, and a re-capture whose pose and observation give 0.4 m recorded as
        # its acceptance's 2.05 m.
        "the re-capture moved before its tracker": lambda tr, rs: (
            rec(rs)["end"].update(time_s=float(tr["stamp_s"][171]))
            or rs["observation_attempts"][3].update(
                t_before_capture_s=float(tr["stamp_s"][171])
            )
        ),
        "the realignment's start moved after its refused capture": lambda tr, rs: rs[
            "observation_attempts"
        ][1].update(t_before_capture_s=float(tr["stamp_s"][140])),
        "the re-capture's pose gives a short straight": short_recapture,
    }
    for name, breaking in bad.items():
        truth, nt, result = realign_case(geometry)
        breaking(truth, result)
        assert not check_xi_(result, result["near_tracking"], truth, geometry)[
            "passed"
        ], name
    # A re-capture that does not record the follower it came by cannot be bound to the last
    # installation (the 2026-10-12 smoke ran a runner from before the field).
    truth, nt, result = realign_case(geometry)
    del result["near_tracking"]["realign"]["end"]["observe_tracker_sha256"]
    with pytest.raises(J.Unverified):
        check_xi_(result, result["near_tracking"], truth, geometry)
    # A replan in the record without a usable time is unverified, not outside the span.
    for replan in (
        {"status": "success"},
        {"status": "success", "time_s": math.nan},
        {"status": "success", "time_s": None},
    ):
        truth, nt, result = realign_case(geometry)
        result["observe_replans"] = [replan]
        with pytest.raises(J.Unverified):
            check_xi_(result, result["near_tracking"], truth, geometry)
    # A redraw-off run needs no stage-1 refusal on its refused capture.
    truth, nt, result = realign_case(geometry)
    result["arguments"]["near_capture_redraw"] = "off"
    result["near_capture_refused"][0]["recoveries"] = []
    assert check_xi_(result, nt, truth, geometry)["passed"]
    # No realignment: not applicable; a refused capture without one is unverified.
    truth, nt, result = realign_case(geometry)
    result["near_tracking"] = {}
    result.pop("near_capture_refused")
    assert check_xi_(result, result["near_tracking"], truth, geometry) == {
        "applicable": False
    }
    result["near_capture_refused"] = [{}]
    with pytest.raises(J.Unverified):
        check_xi_(result, result["near_tracking"], truth, geometry)


def test_xi_the_realignment_span_is_checked_in_the_truth_record():
    truth = truth_from(np.linspace(-3.0, -2.9, 400), phase="observe")
    # A run that ended in its realignment: no approach tick, the span checked to the end.
    assert J.record_problems(truth, realign_span=(150, 399)) == []
    assert any("approach or insert" in p for p in J.record_problems(truth))
    gap = {
        k: (v if k == "phases" else np.delete(v, 250, axis=0)) for k, v in truth.items()
    }
    assert any(
        "realignment" in p for p in J.record_problems(gap, realign_span=(150, 398))
    )
    truth["lift_m"][260] = math.nan
    assert any(
        "lift_m over the realignment" in p
        for p in J.record_problems(truth, realign_span=(150, 399))
    )


def test_ix_after_a_realignment_the_captures_include_the_refused_near_capture():
    """Plan D8 S4c-3 stage 2: far captures, the refused near capture, the re-observation (far)
    and the accepted re-capture (near), the tracker on the last; an open realignment has no
    tracker to compare and is judged, not not_run."""
    from dataclasses import asdict

    result, near_ = injection_case("mismatch", 0.06, 0.0)
    caps = result["near_tracking"]["injection"]["captures"]
    refused = copy.deepcopy(caps[1])
    reobs = {**copy.deepcopy(caps[0]), "attempt": 3}
    recapture = {**copy.deepcopy(caps[1]), "attempt": 4}
    del asdict
    result["near_tracking"]["injection"]["captures"] = [
        caps[0],
        refused,
        reobs,
        recapture,
    ]
    result["near_capture_refused"] = [{"status": "pending", "refused_attempt": 2}]
    result["near_tracking"]["realign"] = {"time_s": 10.5}
    near_["frames"], near_["ticks"] = mismatch_frames()
    near_["events"] = mismatch_events(near_["frames"])
    out = ix(result, near_)
    assert "not far captures" not in " ".join(out["failures"]), out["failures"]
    result[
        "near_capture_refused"
    ] = []  # the same captures without a refused one recorded
    assert any("not far captures" in f for f in ix(result, near_)["failures"])
    # Open: no tracker started, the captures end at the re-observation.
    result, near_ = injection_case("correction", 0.02, 0.0)
    caps = result["near_tracking"]["injection"]["captures"]
    result["near_tracking"]["injection"]["captures"] = [
        caps[0],
        caps[1],
        {**copy.deepcopy(caps[0]), "attempt": 3},
    ]
    result["near_capture_refused"] = [{"status": "pending", "refused_attempt": 2}]
    result["near_tracking"]["realign"] = {"time_s": 10.5}
    result["near_tracking"].pop("start")
    result["near_capture"] = {"status": "pending"}
    out = J.check_ix(result, near_, truth_from(np.linspace(-2.0, -1.5, 200)))
    assert out[
        "status"
    ] != "not_run" and "not the realignment's captures" not in " ".join(out["failures"])


def test_xi_a_mismatch_injected_re_observation_is_judged_through_its_bias(geometry):
    """Codex S4c-3 stage-2 implementation review P2-1: before the first tracker the
    re-observation is biased like every capture -- its record holds the biased observation
    and the target from it, the attempt keeps the raw one."""
    from dataclasses import asdict

    sys.path.insert(0, str(ROOT / "sim" / "isaac"))
    import near_tracking as NT

    truth, nt, result = realign_case(geometry)
    rec = nt["realign"]
    raw = pocket_observation(x=1.6)
    biased = NT.biased_observation(raw, 0.6, 0.0, -0.05)
    rec["observation"] = asdict(biased)
    rec["target"] = J._site_from_observation(
        rec["observation"], rec["capture_base"], 0.6
    ).tolist()
    rec["planned_target"] = list(rec["target"])
    nt["injection"] = {
        "kind": "mismatch",
        "lateral_m": 0.0,
        "yaw_rad": -0.05,
        "drop_from_m": None,
        "captures": [{"attempt": k} for k in (1, 2, 3, 4)],
    }
    out = check_xi_(result, nt, truth, geometry)
    assert out["passed"], out["failures"]
    nt["injection"]["captures"] = [
        {"attempt": k} for k in (1, 2, 4)
    ]  # attempt 3 not biased
    assert not check_xi_(result, nt, truth, geometry)["passed"]
