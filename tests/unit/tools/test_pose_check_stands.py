"""The D batch's two monitor candidates read from the records (tools/pose_check_stands.py, plan
docs/plans/2026-11-06-unconfirmed-validation.md section 6 step 4): the correction's change over a sliding window of path,
and the offline re-match of the mission stands' scan sets (a snapshot approximation, plan 1.5)."""

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "sim/isaac"))
sys.path.insert(0, str(ROOT / "tests/unit/localization"))

import pose_check_stands as S  # noqa: E402
import test_global_match as TG  # noqa: E402

from forklift_core.localization import global_match as G  # noqa: E402
from forklift_core.localization import start_reloc as R  # noqa: E402
from forklift_core.localization.slam_pose import compose, invert  # noqa: E402

CFG = G.MatchConfig(yaw_step_rad=math.radians(2.0))
I = (0.0, 0.0, 0.0)
RELOC = R.RelocConfig(e_min=0.8, r_max=0.9, r_struct=0.8)
ANGLES = (0.0, 2 * math.pi / 360)


def _samples(n=300, step=0.1, c_per_m=0.01, jump_at=None, jump=0.0):
    """A straight drive; the correction moves c_per_m per metre sideways, and steps by ``jump`` at sample ``jump_at``."""
    out = []
    for k in range(n):
        d = k * step
        side = c_per_m * d + (jump if jump_at is not None and k >= jump_at else 0.0)
        out.append({"t": 0.1 * k, "o": [d, 0.0, 0.0], "applied": [d, side, 0.0], "truth": [d, 0.0, 0.0]})
    return out


def _window(samples, window_m, start_t=0.0):
    ser = S.series(samples, renewals=[(start_t, (0.0, 0.0, 0.0))])
    return S._gaps(*S._pairs(ser, S.Monitor("W", window_m=window_m))[:2])[0]


def test_the_window_change_follows_the_correction_over_that_path():
    assert _window(_samples(), 5.0).max() == pytest.approx(0.05, abs=0.002)  # 1 % over 5 m


def test_the_window_runs_from_the_first_sample_a_jump_on_a_short_path_shows():
    # Codex revision review P2-2: under L of path there was no row at all (a 6 m jump over 1 m of path, no result)
    assert _window(_samples(n=10, c_per_m=0.0, jump_at=5, jump=6.0), 5.0).max() > 5.9


def test_the_window_starts_at_its_reference():
    # a change before the confirmation is not the monitor's
    assert _window(_samples(jump_at=5, jump=6.0), 5.0, start_t=1.0).max() < 0.1


def test_a_jump_shows_in_the_window_whatever_the_path_before_it():
    assert _window(_samples(jump_at=200, jump=6.0), 5.0).max() > 6.0


def test_a_standing_correction_step_counts_in_the_window():
    samples = _samples(n=100) + [{"t": 10 + 0.1 * k, "o": [9.9, 0.0, 0.0], "applied": [9.9, 0.3 + 0.099, 0.0],
                                  "truth": [9.9, 0.0, 0.0]} for k in range(50)]
    assert _window(samples, 5.0).max() > 0.3


def _record(o, C=(0.0, 0.0, 0.0), truth=None, later=None):
    sample = {"t": 5.0, "o": list(o), "applied": list(compose(later or C, o)), "truth": list(truth or o)}
    early = {"t": 4.9, "o": list(o), "applied": list(compose(C, o)), "truth": list(truth or o)}
    return {"mode": "shadow", "start": {"o_scan": list(o)}, "hyps": [], "samples": [early, sample],
            "stand_sets": [{"t0": 4.0, "t1": 6.0, "delta": [0.15, 0.01], "ref": "int",
                            "sets": [{"stamps": [4.6, 4.7, 4.8, 4.9, 5.0], "o": list(o)}]}]}


def _rematch_ctx(grid):
    import pose_runtime as PR

    return PR.JobContext(grid, CFG, RELOC, PR.SHADOW_BOUNDS)


def _scans(grid, base, tmp_path, name="s.npz"):
    ranges = np.asarray(TG._scan(grid, R.laser_from_base(base), beams=360)[0], float)
    path = tmp_path / name
    np.savez_compressed(path, angles=np.asarray(ANGLES), ranges_m=np.asarray([0.2, 12.0, 8.0]),
                        s0_0=np.stack([ranges] * 5).astype(np.float32))
    return path


def test_a_stand_the_correction_agrees_with_would_renew_the_reference(tmp_path):
    grid = TG._grid(*TG.ROOM)
    o = (2.42, 1.7, 0.6)
    out = S.rematch_stands(_record(o), np.load(_scans(grid, o, tmp_path)), _rematch_ctx(grid), b=o, o0=o,
                           e_slam=(0.15, 0.05))
    [row] = out
    assert row["outcome"] == "renew" and row["truth_class"] == "success" and row["sets_used"] == 1


def test_a_stand_the_correction_disagrees_with_is_evidence_of_a_revocation(tmp_path):
    grid = TG._grid(*TG.ROOM)
    o = (2.42, 1.7, 0.6)
    out = S.rematch_stands(_record(o, C=(0.5, 0.0, 0.0)), np.load(_scans(grid, o, tmp_path)), _rematch_ctx(grid), b=o,
                           o0=o, e_slam=(0.15, 0.05))
    assert out[0]["outcome"] == "revoke" and out[0]["agree_m"] == pytest.approx(0.5, abs=0.06)


def test_the_comparison_is_with_the_last_correction_of_the_stand(tmp_path):
    # Codex revision review P2-3: plan 1.5's snapshot compares with the C before the truck moves, not the set's
    grid = TG._grid(*TG.ROOM)
    o = (2.42, 1.7, 0.6)
    rec = _record(o, later=(0.5, 0.0, 0.0))
    out = S.rematch_stands(rec, np.load(_scans(grid, o, tmp_path)), _rematch_ctx(grid), b=o, o0=o, e_slam=(0.15, 0.05))
    assert out[0]["outcome"] == "revoke"


def test_the_conservative_check_shares_the_generations_deadline(tmp_path, monkeypatch):
    # Codex revision review P2-4: a new 30 s gave renewals the operating path cannot have
    import pose_runtime as PR

    grid = TG._grid(*TG.ROOM)
    o = (2.42, 1.7, 0.6)
    seen, real = [], PR.run_job

    def job(kind, payload, ctx):
        seen.append((kind, payload["deadline_at"]))
        return real(kind, payload, ctx)

    monkeypatch.setattr(PR, "run_job", job)
    S.rematch_stands(_record(o), np.load(_scans(grid, o, tmp_path)), _rematch_ctx(grid), b=o, o0=o, e_slam=(0.15, 0.05))
    assert [k for k, _ in seen] == ["relocalize", "rematch"] and seen[0][1] == seen[1][1]


def test_a_stand_with_no_sample_after_its_scans_is_not_estimable(tmp_path):
    grid = TG._grid(*TG.ROOM)
    o = (2.42, 1.7, 0.6)
    rec = _record(o)
    rec["samples"] = []
    out = S.rematch_stands(rec, np.load(_scans(grid, o, tmp_path)), _rematch_ctx(grid), b=o, o0=o, e_slam=(0.15, 0.05))
    assert out[0]["outcome"] == "not_estimable"


def test_a_stand_with_no_scan_set_is_not_estimable_not_no_evidence(tmp_path):
    grid = TG._grid(*TG.ROOM)
    o = (2.42, 1.7, 0.6)
    rec = _record(o)
    rec["stand_sets"][0]["sets"] = []
    out = S.rematch_stands(rec, np.load(_scans(grid, o, tmp_path)), _rematch_ctx(grid), b=o, o0=o, e_slam=(0.15, 0.05))
    assert out[0]["outcome"] == "not_estimable"


def test_the_summary_counts_outcomes_and_the_path_between_renewals():
    rows = [{"t": 10.0, "outcome": "renew"}, {"t": 20.0, "outcome": "no_evidence"}, {"t": 30.0, "outcome": "renew"}]
    out = S.summary({"a": rows}, skipped=2)
    assert out["outcomes"] == {"renew": 2, "no_evidence": 1, "not_estimable": 2} and out["stands"] == 5
    assert json.dumps(out)


# --- the offline injection (the revision plan's positive control B) ---
def _series(n=400, step=0.1, c_per_m=0.0, yaw=0.0):
    rows = _samples(n=n, step=step, c_per_m=c_per_m)
    return S.series(rows, renewals=[(0.0, (0.0, 0.0, 0.0))])


def test_a_fixed_translation_step_is_detected_by_a_window_and_measured_on_the_true_error():
    ser = _series()
    case = S.Injection("step_xy", 0.35, direction=(0.0, 1.0))
    out = S.run_case(ser, 100, case, S.Monitor("W", window_m=5.0, limit=(0.20, 0.05)))
    assert out["detected"] and out["delay_m"] == pytest.approx(0.0, abs=0.11) and out["wrong_reached"]
    assert out["exposure_m"] == pytest.approx(0.0, abs=0.11)


def test_the_true_error_is_computed_not_assumed():
    # Codex revision review P2-6: an error of -0.10 m with +0.35 m injected is 0.25 m, not wrong
    rows = _samples(n=400, c_per_m=0.0)
    for r in rows:
        r["applied"][1] = -0.10
    ser = S.series(rows, renewals=[(0.0, (0.0, -0.10, 0.0))])
    out = S.run_case(ser, 100, S.Injection("step_xy", 0.35, direction=(0.0, 1.0)),
                     S.Monitor("W", window_m=5.0, limit=(0.20, 0.05)))
    assert out["wrong_reached"] is False


def test_a_heading_step_is_a_fixed_correction_about_the_truck():
    # Codex revision review P2-6: no position change where it enters, none growing once the window has passed it
    ser = _series()
    case = S.Injection("step_yaw", 0.12)
    out = S.run_case(ser, 100, case, S.Monitor("W", window_m=2.0, limit=(10.0, 0.05)))
    assert out["detected"] and out["wrong_reached"]
    applied = S.injected_applied(ser, 100, case)
    assert applied[100][:2] == pytest.approx(ser.applied[100][:2], abs=1e-9)
    assert applied[399][2] - ser.applied[399][2] == pytest.approx(0.12)


def test_a_slow_creep_under_the_natural_rate_is_a_miss_and_counted_as_one():
    ser = _series()
    out = S.run_case(ser, 50, S.Injection("ramp_xy", 0.02, direction=(0.0, 1.0)),
                     S.Monitor("W", window_m=5.0, limit=(0.20, 0.05)))
    assert out["detected"] is False and out["wrong_reached"] is True and out["exposure_m"] > 10.0


def test_a_late_onset_that_never_reaches_wrong_is_apart_from_a_miss():
    ser = _series(n=120)
    out = S.run_case(ser, 110, S.Injection("ramp_xy", 0.02, direction=(0.0, 1.0)),
                     S.Monitor("W", window_m=5.0, limit=(0.20, 0.05)))
    assert out["wrong_reached"] is False and out["detected"] is False


def test_the_cumulative_monitors_read_the_reference_and_the_proportional_one_the_path():
    ser = _series(c_per_m=0.05)
    cum = S.first_alarm(ser, S.Monitor("M0", limit=(0.20, 0.05)))
    assert ser.path[cum] == pytest.approx(4.0, abs=0.11)
    assert S.first_alarm(ser, S.Monitor("P", limit=(0.20, 0.05), per_m=(0.06, 0.01))) is None


def test_a_threshold_is_the_clean_sessions_largest_times_1_5_and_leave_one_out_counts_the_held_out():
    quiet, loud = _series(c_per_m=0.01), _series(c_per_m=0.02)
    mon = S.Monitor("W", window_m=5.0)
    limit = S.fit_limit({"a": quiet, "b": loud}, mon)
    assert limit[0] == pytest.approx(1.5 * 0.10, abs=0.006)
    assert S.leave_one_out({"a": quiet, "b": loud}, mon) == {"b": True, "a": False}


# --- Codex revision re-review ---
def test_a_shadow_generation_renews_the_reference_only_where_the_operating_rule_would():
    # the shadow matches at every stand; an on run only past 0.10 m of Delta_int (design 1.5)
    rows = _samples(n=300, c_per_m=0.05)  # Delta_int 0.044 m at 0.9 m of path, 0.11 m at 2.2 m
    near, far = (compose((0.0, 0.05 * 0.9, 0.0), (0.0, 0.0, 0.0))), (0.0, 0.05 * 2.2, 0.0)
    ser = S.series(rows, renewals=[(0.0, I), (0.9, near), (2.2, far)])
    assert ser.renewals_taken == [2.2]
    at = int(np.searchsorted(ser.t, 2.35))
    assert ser.ref[at] == pytest.approx(far) and ser.since[at] == pytest.approx(0.2, abs=0.02)


def test_the_available_renewals_are_the_generations_that_passed_not_the_provisional_events():
    # after the provisional check ended no agreement is written; the raw generations go on
    rec = {"start": {"o_scan": [1.0, 0.0, 0.0]}, "events": [],
           "raw": {"start": {"confirmed": True, "r": [1.0, 0.0, 0.0], "agree_m": 0.01, "agree_rad": 0.0, "t": 1.0,
                             "C": [0.0, 0.0, 0.0]},
                   "rematches": [{"t": 9.0, "g": [2.0, 0.0, 0.0], "o_scan": [2.0, 0.0, 0.0], "residual_confirmed": True,
                                  "applicable": True, "agree_m": 0.05, "agree_rad": 0.01, "C": [0.0, 0.0, 0.0]},
                                 {"t": 12.0, "g": [3.0, 0.0, 0.0], "o_scan": [3.0, 0.0, 0.0],
                                  "residual_confirmed": False, "applicable": True, "agree_m": 0.05, "agree_rad": 0.01, "C": [0.0, 0.0, 0.0]},
                                 {"t": 13.0, "g": [3.0, 0.0, 0.0], "o_scan": [3.0, 0.0, 0.0],
                                  "residual_confirmed": True, "applicable": True, "agree_m": 0.5, "agree_rad": 0.01,
                                  "C": [0.0, 0.0, 0.0]}]}}
    got = S.renewals_of(rec, e_slam=(0.109, 0.0137))
    assert [r[0] for r in got] == [1.0, 9.0]


def test_every_direction_and_sign_keeps_its_own_key():
    keys = {S.case_key(c) for c in S.kinds()}
    assert len(keys) == len(S.kinds()) == 50
    assert "step_xy:0.35:+y" in keys and "step_yaw:0.12:-" in keys


def _tally(rate, n=20):
    return {"cases": n, "wrong_reached": n, "detected": int(rate * n), "rate": rate, "exposure_m_max": 1.0}


def _candidate(rates, loo=(), positive=True, name="W5", window=5.0):
    by = {f"step_xy:{s:g}:{d}": _tally(rates.get(s, 1.0)) for s in (0.35, 0.5, 1.0, 3.0, 6.0, 12.0)
          for d in ("+x", "-x", "+y", "-y")}
    by.update({f"step_yaw:{s:g}:{d}": _tally(rates.get(s, 1.0)) for s in (0.12, 0.3, 3.14159) for d in ("+", "-")})
    return {"window_m": window, "loo_false_in_missions": list(loo), "by_key": by,
            "positive_a": {"val3_055": {"observed": True, "alarm_t": None if not positive else 5.0},
                           "val3_016": {"observed": False}}}


def test_the_choice_eliminates_by_kind_and_compares_the_small_kinds_minimum():
    missions = [f"m{k}" for k in range(10)]
    one_axis = _candidate({})
    one_axis["by_key"]["step_xy:1:+x"] = _tally(0.80)  # the sum over four axes would pass at 0.95
    out = S.choose({"a": one_axis, "b": _candidate({}, loo=["m1", "m2"]), "c": _candidate({}, positive=False),
                    "d": _candidate({0.35: 0.5}), "e": _candidate({0.35: 0.9}, window=10.0)}, missions)
    assert set(out["eliminated"]) == {"a", "b", "c"} and out["ranked"] == ["e", "d"]


def test_no_candidate_left_means_no_winner():
    out = S.choose({"a": _candidate({}, positive=False)}, [f"m{k}" for k in range(10)])
    assert out["ranked"] == [] and out["winner"] is None


def test_the_onsets_lie_after_the_last_renewal_and_a_turn_through_pi_is_no_turn():
    rows = _samples(n=200)
    for k, r in enumerate(rows):
        r["applied"][2] = r["o"][2] = 3.13 if k < 150 else (-3.13 if k < 170 else -2.9)
    ser = S.series(rows, renewals=[(0.0, I)])
    got = S.onsets(ser, "a", 100)
    assert min(i for _, i in got) >= 100 and dict(got)["first"] == 100
    assert dict(got)["turn"] == 169  # the real turn, not the wrap at 149


# --- Codex revision review, draft 3 ---
def test_a_suffix_without_a_turn_keeps_its_onsets_inside_and_no_suffix_gives_none():
    ser = S.series(_samples(n=200), renewals=[(0.0, I)])
    assert min(i for _, i in S.onsets(ser, "a", 100)) >= 100
    assert S.onsets(ser, "a", 199) == []


def test_a_renewals_eligibility_reads_the_correction_of_its_own_tick_not_the_next_sample():
    # a Delta of 0.09 m at the result, 0.21 m at the next sample: not eligible
    rows = _samples(n=100, c_per_m=0.0)
    for r in rows[51:]:
        r["applied"][1] = 0.21
    at = (0.0, 0.09, 0.0)
    ser = S.series(rows, renewals=[(0.0, I), (5.05, (0.0, 0.05, 0.0), at, (5.0, 0.0, 0.0))])
    assert ser.renewals_taken == []


def test_without_a_confirmed_start_the_first_reference_is_a_confirming_generation():
    def gen(t, outcome):
        return {"t": t, "phase": "unconfirmed", "outcome": outcome, "g": [t, 0.0, 0.0], "o_scan": [t, 0.0, 0.0],
                "residual_confirmed": True, "applicable": True, "agree_m": 0.01, "agree_rad": 0.0, "C": [0.0, 0.0, 0.0]}

    rec = {"start": {"o_scan": [0.0, 0.0, 0.0]},
           "raw": {"start": {"confirmed": False}, "rematches": [gen(1.0, "unconfirmed"), gen(2.0, "confirm"),
                                                                gen(3.0, "confirm")]}}
    assert [r[0] for r in S.renewals_of(rec, e_slam=(0.1, 0.1))] == [2.0, 3.0]
    # an operating run ends at out_of_set or twin_preferred: what the shadow went on to record is not its confirmation
    rec["raw"]["rematches"] = [gen(1.0, "out_of_set"), gen(2.0, "confirm")]
    assert S.renewals_of(rec, e_slam=(0.1, 0.1)) == []
    rec["raw"]["rematches"] = [gen(1.0, "twin_preferred"), gen(2.0, "confirm")]
    assert S.renewals_of(rec, e_slam=(0.1, 0.1)) == []
    far = gen(2.0, "confirm")
    far["agree_m"] = 0.5  # a confirming generation the correction disagrees with: pose_slam_disagree ends it
    rec["raw"]["rematches"] = [far, gen(3.0, "confirm")]
    assert S.renewals_of(rec, e_slam=(0.1, 0.1)) == []


# --- Codex revision review, draft 4 ---
def test_a_renewal_on_a_samples_own_tick_applies_to_that_sample():
    # the runtime handles the result before the tick's sample: the same t is already under the new reference
    rows = _samples(n=100, c_per_m=0.0)
    for r in rows[50:]:
        r["applied"][1] = 0.25
    new = (0.0, 0.25, 0.0)
    ser = S.series(rows, renewals=[(0.0, I), (rows[50]["t"], new, new, (5.0, 0.0, 0.0))])
    assert ser.renewals_taken == [rows[50]["t"]] and ser.ref[50] == pytest.approx(new) and ser.ref[49] == pytest.approx(I)


def test_a_kind_with_no_case_is_not_evaluated_and_no_winner_comes_of_nothing():
    cand = _candidate({})
    cand["by_key"] = {}
    out = S.choose({"a": cand}, [f"m{k}" for k in range(10)])
    assert out["winner"] is None and any("no cases" in w for w in out["eliminated"]["a"])


def test_the_sessions_without_a_suffix_are_counted():
    ser = S.series(_samples(n=100), renewals=[(0.0, I), (rows_end := 9.85, (0.0, 0.5, 0.0))])
    out = S.evaluate({"a": ser}, ["a"], {})
    assert out["sessions_with_cases"] == 0 and out["sessions_without_suffix"] == ["a"]
    assert out["choice"]["winner"] is None and rows_end


# --- after the evaluation: the window's time cap, the stand's rebasing at a release (revision plan 9.7) ---
def _standing_drift(seconds=120.0, rate=0.002):
    """A truck standing, the correction turning at ``rate`` against a gyro's residual: the applied pose stays true."""
    return [{"t": 0.1 * k, "o": [1.0, 2.0, -rate * 0.1 * k], "applied": [1.0, 2.0, 0.0], "truth": [1.0, 2.0, 0.0]}
            for k in range(int(seconds * 10))]


def test_a_path_window_never_ends_on_a_stand_and_a_time_cap_does():
    ser = S.series(_standing_drift(), renewals=[(0.0, I)])
    assert S.first_alarm(ser, S.Monitor("W", window_m=2.0, limit=(10.0, 0.103))) is not None
    assert S.first_alarm(ser, S.Monitor("W", window_m=2.0, window_s=10.0, limit=(10.0, 0.103))) is None


def test_leave_one_out_keeps_the_time_cap():
    # Codex: fitted with the 10 s cap and held out on an open window, a slow drift alarms for nothing
    t = np.arange(0.0, 20.0, 0.1)

    def session():
        samples = [{"t": float(x), "o": [0.1 * x, 0.0, 0.0], "truth": [0.1 * x, 0.0, 0.0],
                    "applied": list(S.compose((0.0, 0.0, 0.002 * float(x)), (0.1 * x, 0.0, 0.0)))} for x in t]
        return S.series(samples, [(0.0, (0.0, 0.0, 0.0))])

    streams = {"a": session(), "b": session()}
    assert not any(S.leave_one_out(streams, S.Monitor("W", window_m=1000.0, window_s=10.0)).values())


def _stand_samples(c_of_t, o_of_t=lambda t: (0.0, 0.0, 0.0)):
    out = []
    for k in range(100):
        t, o = 0.1 * k, o_of_t(0.1 * k)
        out.append({"t": t, "o": list(o), "applied": list(S.compose(c_of_t(t), o))})
    return out


def test_the_stand_change_is_rebased_at_a_hold_release():
    samples = _stand_samples(lambda t: (0.0, 0.0, 0.0 if t < 5.0 else 0.04))  # the release's step at 5 s, nothing else
    assert S.stand_change(samples, t0=0.0, t1=9.9, releases=[])[1] == pytest.approx(0.04)
    assert S.stand_change(samples, t0=0.0, t1=9.9, releases=[(5.0, (0.0, 0.0, 0.04))]) == pytest.approx((0.0, 0.0))


def test_the_rebased_reference_is_what_the_release_applied():
    # Codex: +0.07 m released at 2.45 s and gone at 2.49 s, no sample between: the first sample after it is no reference
    samples = _stand_samples(lambda t: (0.0, 0.0, 0.0))
    assert S.stand_change(samples, t0=0.0, t1=9.9, releases=[(2.49, (0.07, 0.0, 0.0))])[0] == pytest.approx(0.07)


def test_the_stand_change_is_the_correction_at_this_odometry_as_the_check_reads_it():
    # Codex: O +0.04 m with C -0.04 m leaves the applied pose where it was, and the correction moved
    samples = _stand_samples(lambda t: (-0.04 if t >= 5.0 else 0.0, 0.0, 0.0),
                             lambda t: (0.04 if t >= 5.0 else 0.0, 0.0, 0.0))
    assert S.stand_change(samples, t0=0.0, t1=9.9, releases=[])[0] == pytest.approx(0.04)


def test_a_release_is_read_from_the_replies_either_side_of_it():
    # Codex: +0.07 m at the release and back at the next reply: the replies' applied values alone would say 0
    rows = [{"mode": "tracking", "hold": 0, "applied_after": [0.0, 0, 0], "received_after": [0.0, 0, 0],
             "applied_tick_s": 1.0, "odom_base": [0.0, 0, 0]},
            {"mode": "holding", "hold": 1, "applied_after": [0.0, 0, 0], "received_after": [0.07, 0, 0],
             "applied_tick_s": 2.0, "odom_base": [0.0, 0, 0]},
            {"mode": "tracking", "hold": 1, "applied_after": [0.0, 0, 0], "received_after": [0.0, 0, 0],
             "applied_tick_s": 3.0, "odom_base": [0.0, 0, 0]}]
    out = S.hold_releases(rows)
    assert len(out) == 1 and out[0]["m"] == pytest.approx(0.07) and (out[0]["t0"], out[0]["t1"]) == (2.0, 3.0)
    assert out[0]["applied"] == [0.07, 0, 0]


def test_the_release_mark_is_the_first_sample_on_the_applied_correction():
    held, after = (0.0, 0.0, 0.0), (0.0, 0.0, 0.03)
    release = {"t0": 2.0, "t1": 3.0, "held": list(held), "applied": list(after)}
    assert S.release_mark(_stand_samples(lambda t: held if t < 2.45 else after), release) == pytest.approx(2.5)
    # no sample shows it: the span's end
    assert S.release_mark(_stand_samples(lambda t: held), release) == 3.0
