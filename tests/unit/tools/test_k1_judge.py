"""K1 comparison judgement (docs/validation/2026-11-14-k1-comparison/k1_judge.py) on synthetic results."""

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
_spec = importlib.util.spec_from_file_location(
    "k1_judge", ROOT / "docs/validation/2026-11-14-k1-comparison/k1_judge.py")
kj = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(kj)


def result(success=True, cycle=200.0, lateral=0.03, events=(), failure=None, phase="complete", watch=(), stands=()):
    trans = [{"to": "approach", "time_s": 50.0}, {"to": "insert", "time_s": 57.0}]
    if success:
        trans.append({"to": "home_settle", "time_s": cycle})
    r = {"success": success, "phase": phase, "transitions": trans,
         "deterministic": {"enabled": True, "broken": []},
         "near_capture": {"status": "done", "offsets_to_new_line": {"lateral_m": lateral, "yaw_rad": 0.0}},
         "near_loose_refusals": list(events), "near_loose_watch": list(watch), "observe_end_stands": list(stands)}
    if failure:
        r["failure_reason"] = failure
    return r


def event(inside=True, capture="done"):
    return {"time_s": 40.0, "kinds": ["blocked_arrival", "extra_c2"], "c2_record_index": 0,
            "arrival": {"inside_narrow": inside, "bounds_source": "fallback"}, "capture": {"status": capture}}


WATCH = {"start_s": 40.0, "end_s": 50.0, "steps": 100, "max_move_m": 1e-6, "contact_steps": 0, "first_contact": None,
         "near_capture_status": "done"}
STAND = {"kind": "near", "outcome": "installed"}


def rows_for(k1_override=None, b0_override=None):
    rows = []
    for scene, seed in kj.CELLS:
        for arm in kj.ARMS:
            override = (k1_override if arm == "K1" else b0_override) or {}
            r = override.get((scene, seed)) or result()
            rows.append(kj.summarize({"name": f"k1c_{arm}_{scene}_s{seed}", "arm": arm, "scene": scene, "seed": seed,
                                      "result": r}))
    return rows


def intervened(n, inside=True, delta=9.0):
    return {("random_1", s): result(cycle=200.0 + delta, events=[event(inside)], watch=[WATCH], stands=[STAND])
            for s in range(n)}


def test_a_missing_run_is_not_judged():
    rows = rows_for()[:-1]
    out = kj.judge(rows)
    assert out["verdict"].startswith("not_judged") and out["checks"]["missing_or_duplicate"]


def test_no_intervention_is_underexposed_not_a_difference():
    out = kj.judge(rows_for())
    assert out["verdict"] == "effect_unconfirmed_promotion_held"
    assert out["criteria"]["4_recovery"]["underexposed"] and out["criteria"]["4_recovery"]["pass"]


def test_recovered_events_with_enough_exposure_make_k1_better():
    big = {("random_1", s): result(lateral=0.09) for s in range(4)}
    k1 = intervened(6)
    out = kj.judge(rows_for(k1_override=k1, b0_override=big))
    c = out["criteria"]
    assert c["4_recovery"]["success_rate"] == 1.0 and c["3_near_refusals"]["lateral_over_0_08"] == {"B0": 4, "K1": 0}
    assert c["5_time"]["intervened_increase_median_s"] == 9.0
    assert out["verdict"] == "k1_better", out["criteria"]
    assert out["records"]["cycle_delta_s_intervened"] == [9.0] * 6


def test_arrivals_outside_the_narrow_bounds_are_not_successes():
    big = {("random_1", s): result(lateral=0.09) for s in range(4)}
    k1 = intervened(6, inside=False)
    out = kj.judge(rows_for(k1_override=k1, b0_override=big))
    assert out["criteria"]["4_recovery"]["verdicts"] == {"outside_narrow": 6}
    assert out["verdict"] == "no_difference_confirmed"


def test_a_contact_in_the_watch_makes_k1_worse():
    k1 = intervened(6)
    k1[("random_1", 0)]["near_loose_watch"] = [{**WATCH, "contact_steps": 2}]
    out = kj.judge(rows_for(k1_override=k1))
    assert out["verdict"] == "worse" and not out["criteria"]["1_safety"]["pass"]


def test_a_different_outcome_on_line_2_is_a_regression():
    k1 = {("line_2", 3): result(success=False, failure="Tracking timeout in transport", phase="transport")}
    out = kj.judge(rows_for(k1_override=k1))
    assert out["criteria"]["6_non_regression"]["different_outcome"] == ["k1c_K1_line_2_s3"]
    assert out["verdict"] == "worse"


def test_a_new_observe_timeout_fails_the_time_cost():
    big = {("random_1", s): result(lateral=0.09) for s in range(4)}
    k1 = intervened(6)
    k1[("random_1", 10)] = result(success=False, failure="Tracking timeout in observe", phase="observe")
    out = kj.judge(rows_for(k1_override=k1, b0_override=big))
    assert out["criteria"]["5_time"]["new_observe_timeouts"] == ["k1c_K1_random_1_s10"]
    assert out["verdict"] == "worse"  # also one completion short on random_1 (criterion 2)


def test_outcome_classes():
    assert kj.outcome_of(result()) == "complete"
    assert kj.outcome_of(result(success=False, failure="near_capture_misaligned:x")) == "insert"  # reached insert
    r = result(success=False, failure="near_capture_misaligned:x")
    r["transitions"] = [{"to": "approach", "time_s": 50.0}]
    assert kj.outcome_of(r) == "refused"
    r["failure_reason"] = "Tracking timeout in approach"
    assert kj.outcome_of(r) == "accepted"
    r["near_capture"]["status"] = "pending"
    assert kj.outcome_of(r) == "pre_capture"


def test_the_output_is_strict_json():
    out = kj.judge(rows_for(k1_override=intervened(6)))
    json.dumps(out, allow_nan=False)


# ---- Codex result review 1 ---------------------------------------------------------------------------------------


def test_a_scene_damage_failure_in_an_intervened_run_makes_k1_worse():
    """The class is mission.failures' SCENE, not a word list (Pallet dropped during transport)."""
    k1 = intervened(6)
    k1[("random_1", 0)] = result(success=False, failure="Pallet dropped during transport: z=0.01", phase="transport",
                                 events=[event()], watch=[WATCH], stands=[STAND])
    out = kj.judge(rows_for(k1_override=k1))
    assert out["criteria"]["1_safety"]["damage_failures"] == ["k1c_K1_random_1_s0"]
    assert out["verdict"] == "worse"


def test_missing_watch_records_stop_the_judgement():
    for bad in ([], [{**WATCH, "end_s": None}], [{**WATCH, "steps": 0}], [{**WATCH, "max_move_m": float("nan")}]):
        k1 = intervened(6)
        k1[("random_1", 0)]["near_loose_watch"] = bad
        out = kj.judge(rows_for(k1_override=k1))
        assert out["verdict"] == "not_judged: missing records", bad


def test_a_joint_completion_without_its_cycle_time_stops_the_judgement():
    r = result()
    r["transitions"] = [t for t in r["transitions"] if t["to"] != "home_settle"]
    out = kj.judge(rows_for(k1_override={("random_1", 3): r}))
    assert out["verdict"] == "not_judged: missing records"


def test_the_extension_is_judged_on_its_own_cells_and_runs_outside_them_stop_it():
    rows = rows_for()
    extra = [kj.row(f"k1c_{a}_random_1_s{s}", result()) for s in range(30, 60) for a in kj.ARMS]
    out = kj.judge(rows + extra)  # the registered cells: the extension's runs are outside them
    assert out["verdict"].startswith("not_judged") and len(out["checks"]["outside_cells"]) == 60
    out = kj.judge(rows + extra, kj.cells_for(60))
    assert out["checks"]["cells"] == 70 and not out["verdict"].startswith("not_judged")


def test_a_failed_intervention_on_line_2_is_a_regression():
    k1 = intervened(6)
    k1[("line_2", 1)] = result(events=[{**event(), "c2_record_index": None}], watch=[WATCH])
    out = kj.judge(rows_for(k1_override=k1))
    assert out["criteria"]["6_non_regression"]["k1_events_not_successful"] == [
        {"name": "k1c_K1_line_2_s1", "verdict": "no_c2_install"}]
    assert out["verdict"] == "worse"


def test_underexposure_holds_promotion_even_when_another_term_fails():
    """Plan 5: an underexposed second term holds promotion whatever 3-5 otherwise say."""
    k1 = intervened(6, inside=False)  # 4 fails (0/6) while 3 is underexposed (B0 has no offset over 0.08)
    out = kj.judge(rows_for(k1_override=k1))
    assert out["criteria"]["3_near_refusals"]["underexposed"] and not out["criteria"]["4_recovery"]["pass"]
    assert out["verdict"] == "effect_unconfirmed_promotion_held"


def test_the_raw_fields_file_judges_the_same(tmp_path):
    import gzip
    fields = {f"k1c_{a}_{c}_s{s}": result() for c, s in kj.CELLS for a in kj.ARMS}
    path = tmp_path / "f.json.gz"
    with gzip.open(path, "wt") as f:
        json.dump(fields, f)
    assert kj.judge(kj.load_fields(path)) == kj.judge(rows_for())
