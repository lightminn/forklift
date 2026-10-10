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
