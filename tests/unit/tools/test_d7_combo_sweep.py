"""The D7 adoption rule (tools/d7_combo_sweep.py judge) on synthetic records."""

import json

import pytest

from tools import d7_combo_sweep as D7

RUN = "run-hash"


def row(goal, length, status="success", cycles=0):
    return {"goal": goal, "status": status, "length_to_goal_m": length if status == "success" else None,
            "gear_changes": 1, "ladder_s": [0.1], "cycles_in_path": cycles, "root_shot": "taken"}


def record(combo, seed, lengths, mission="success", cycles=0, run=RUN, scene="s", timeouts=0):
    return {"combo": combo, "settings": D7.COMBOS[combo], "seed": seed, "input_sha256": run,
            "scenario_sha256": f"{scene}{seed}",
            "mission": {"outcome": mission, "timeouts": timeouts, "gear_changes": 0,
                        "observation": {"candidate_index": 0, "extended": False} if mission == "success" else None},
            "R": [row("transport" if i % 2 == 0 else "return", v, cycles=cycles) for i, v in enumerate(lengths)]}


def write(tmp_path, records, seeds=(1, 2), bm_timeouts=None, bm_extra=None):
    bm = tmp_path / "bm" / "part0"
    bm.mkdir(parents=True, exist_ok=True)
    for s in seeds:
        trace = [{"stage": "observe_candidate", "status": "timeout"}] * (bm_timeouts or {}).get(s, 0)
        (bm / f"seed_{s}.json").write_text(json.dumps({"seed": s, "outcome": "success", "trace": trace,
                                                        "scenario_sha256": f"s{s}", **(bm_extra or {}).get(s, {})}))
    (tmp_path / "inputs.json").write_text(json.dumps({"run_sha256": RUN, "bm_successes": list(seeds),
                                                      "bm_dir": str(tmp_path / "bm"),
                                                      "bm_records_sha256": D7.bm_records_sha256(tmp_path / "bm")}))
    for r in records:
        d = tmp_path / f"combo_{r['combo']:02d}"
        d.mkdir(exist_ok=True)
        (d / f"seed_{r['seed']}.json").write_text(json.dumps(r))
    return type("A", (), {"output": tmp_path})()


LOOPY = [20.0, 1.0, 20.0, 1.0]


def test_the_bm_successes_are_the_reference_not_combination_zeros_rerun(tmp_path):
    # Codex D7 5th P1-1: combination 0 and the candidate both fail seed 2 -- still a loss.
    recs = [record(0, 1, LOOPY), record(0, 2, LOOPY, mission="planning_failure"),
            record(1, 1, [1.0] * 4), record(1, 2, [1.0] * 4, mission="planning_failure")]
    out = D7.judge(write(tmp_path, recs))
    assert out["combos"][1]["mission_lost"] == [2] and not out["combos"][1]["passes"]
    assert out["baseline_rerun_mission_lost"] == [2] and out["adopted"] is None


def test_records_from_other_inputs_are_refused(tmp_path):
    recs = [record(0, 1, LOOPY), record(0, 2, LOOPY), record(1, 1, [1.0] * 4, run="old"), record(1, 2, [1.0] * 4)]
    with pytest.raises(SystemExit):
        D7.judge(write(tmp_path, recs))


def test_the_length_ranking_pools_both_goals(tmp_path):
    # Codex D7 5th P2-3: a median of the two goal medians is not the pooled median.
    base = [record(0, s, LOOPY) for s in (1, 2)]
    a = [record(1, 1, [3.0, 1.0, 3.0, 1.0]), record(1, 2, [3.0, 1.0, 3.0, 1.0])]   # pooled median 2.0
    b = [record(2, 1, [1.9, 2.0, 1.9, 2.0]), record(2, 2, [1.9, 2.0, 1.9, 2.0])]   # pooled median 1.95
    out = D7.judge(write(tmp_path, base + a + b))
    assert out["adopted"] == 2


def test_d7b_without_an_effect_gives_way_before_the_ranking(tmp_path):
    # Codex D7 5th P2-4: on (no cycle used) is dropped for its passing off twin, and the
    # ranking then runs over the rest -- not a swap after the winner is chosen.
    on = D7.COMBOS.index({**D7.COMBOS[0], "turn_cycles": True, "shot_cap_m": 4.0})
    off = D7.COMBOS.index({**D7.COMBOS[0], "shot_cap_m": 4.0})
    other = D7.COMBOS.index({**D7.COMBOS[0], "goal_connection": "reeds_shepp", "shot_cap_m": 4.0})
    recs = [record(0, s, LOOPY) for s in (1, 2)]
    recs += [record(on, s, [0.5] * 4) for s in (1, 2)]
    recs += [record(off, s, [3.0] * 4) for s in (1, 2)]
    recs += [record(other, s, [1.0] * 4) for s in (1, 2)]
    out = D7.judge(write(tmp_path, recs))
    assert on in out["passing"] and on not in out["ranked_candidates"]
    assert out["adopted"] == other


def test_mission_timeouts_count_against_the_original_bm(tmp_path):
    # Codex D7 6th P2-1: B_M had none; combination 0's rerun and the candidate one each.
    recs = [record(0, 1, LOOPY, timeouts=1), record(0, 2, LOOPY), record(1, 1, [1.0] * 4, timeouts=1),
            record(1, 2, [1.0] * 4)]
    out = D7.judge(write(tmp_path, recs))
    assert out["combos"][1]["new_mission_timeouts"] == 1 and not out["combos"][1]["passes"]


def test_a_record_of_another_combination_in_a_folder_is_refused(tmp_path):
    recs = [record(0, 1, LOOPY), record(0, 2, LOOPY), record(1, 1, [1.0] * 4)]
    args = write(tmp_path, recs)
    stray = record(3, 2, [1.0] * 4)
    (tmp_path / "combo_01" / "seed_2.json").write_text(json.dumps(stray))
    with pytest.raises(SystemExit):
        D7.judge(args)


def test_another_bm_with_the_same_successes_is_refused(tmp_path):
    # Codex D7 7th P2: the same success list with other timeouts must not pass as the reference.
    recs = [record(0, 1, LOOPY), record(0, 2, LOOPY), record(1, 1, [1.0] * 4, timeouts=1), record(1, 2, [1.0] * 4)]
    args = write(tmp_path, recs)
    other = tmp_path / "other" / "part0"
    other.mkdir(parents=True)
    for s in (1, 2):
        trace = [{"status": "timeout"}] if s == 1 else []
        (other / f"seed_{s}.json").write_text(json.dumps({"seed": s, "outcome": "success", "trace": trace,
                                                          "scenario_sha256": f"s{s}"}))
    args.bm_dir = tmp_path / "other"
    with pytest.raises(SystemExit):
        D7.judge(args)


def test_a_run_without_the_hash_needs_a_frozen_manifest(tmp_path):
    recs = [record(0, s, LOOPY) for s in (1, 2)] + [record(1, s, [1.0] * 4) for s in (1, 2)]
    args = write(tmp_path, recs)
    inputs = json.loads((tmp_path / "inputs.json").read_text())
    inputs.pop("bm_records_sha256")
    (tmp_path / "inputs.json").write_text(json.dumps(inputs))
    with pytest.raises(SystemExit):
        D7.judge(args)
    D7.freeze_bm(args)
    assert D7.judge(args)["adopted"] == 1
    with pytest.raises(SystemExit):  # never rewritten
        D7.freeze_bm(args)


def test_a_scene_other_than_bms_is_refused(tmp_path):
    recs = [record(0, 1, LOOPY), record(0, 2, LOOPY), record(1, 1, [1.0] * 4, scene="x"), record(1, 2, [1.0] * 4, scene="x")]
    recs[0]["scenario_sha256"] = "x1"
    with pytest.raises(SystemExit):
        D7.judge(write(tmp_path, recs))


# Plan D7 측정 복구: the registered re-run pairs replace the run's, under a control.
TRACE = [{"stage": "observe_candidate", "candidate_index": 0, "extended": False, "status": "success",
          "expansions": 50},
         {"stage": "transport_search", "status": "success", "expansions": 900, "gear_changes": 0}]
BM_LADDER = {s: {"ladder_wall_s": [0.5, 12.0], "trace": TRACE} for s in (1, 2)}
PAIRS = [[0, 2], [1, 2]]


def overlay(tmp_path, records, pairs=PAIRS, ladder=(0.5, 12.5), skip=(), run=RUN, sidecar=None):
    o = tmp_path / "rerun"
    o.mkdir()
    (o / "pairs.json").write_text(json.dumps({"pairs": pairs, "run_sha256": run}))
    for r in records:
        if (r["combo"], r["seed"]) in skip:
            continue
        d = o / f"combo_{r['combo']:02d}"
        d.mkdir(exist_ok=True)
        r = {**r, "mission": {**r["mission"], "wall_s": round(sum(ladder) + 0.2, 3)}}
        (d / f"seed_{r['seed']}.json").write_text(json.dumps(r))
        (d / f"seed_{r['seed']}.instrument.json").write_text(json.dumps(
            {"combo": r["combo"], "seed": r["seed"], "mission_ladder_s": list(ladder), "trace": TRACE,
             **(sidecar or {})}))
    return o


def timed_out_run(tmp_path):
    # Combination 0 and 1 both lose seed 2 to a timeout in the loaded run.
    recs = [record(0, 1, LOOPY), record(0, 2, LOOPY, mission="planning_failure", timeouts=1),
            record(1, 1, [1.0] * 4), record(1, 2, [1.0] * 4, mission="planning_failure", timeouts=1)]
    return write(tmp_path, recs, bm_extra=BM_LADDER)


RERUN = [record(0, 2, LOOPY), record(1, 2, [1.0] * 4)]


def test_the_overlay_replaces_the_timed_out_pairs_under_a_passing_control(tmp_path):
    args = timed_out_run(tmp_path)
    assert D7.judge(args)["adopted"] is None
    args.overlay = overlay(tmp_path, RERUN)
    out = D7.judge(args)
    assert out["control"]["passed"] and out["control"]["seeds"] == [2]
    assert out["control"]["ladder_ratio_median"] == pytest.approx(12.5 / 12.0)
    assert out["control"]["stage_expansions_differ"] == []
    assert out["adopted"] == 1 and out["overlay"]["pairs"] == [(0, 2), (1, 2)]


def test_a_failed_control_adopts_nothing(tmp_path):
    args = timed_out_run(tmp_path)
    args.overlay = overlay(tmp_path, RERUN, ladder=(0.5, 13.5))
    out = D7.judge(args)
    assert out["combos"][1]["passes"] and not out["control"]["passed"] and out["adopted"] is None


def test_a_rerun_that_fails_again_is_used_as_it_is(tmp_path):
    args = timed_out_run(tmp_path)
    args.overlay = overlay(tmp_path, [record(0, 2, LOOPY),
                                      record(1, 2, [1.0] * 4, mission="planning_failure", timeouts=1)])
    out = D7.judge(args)
    assert out["combos"][1]["mission_lost"] == [2] and out["adopted"] is None


@pytest.mark.parametrize("pairs,skip,run", [
    (PAIRS, {(1, 2)}, RUN),  # a registered pair without its record
    ([[1, 2]], set(), RUN),  # the control seed dropped from the list (with its record kept)
    ([[1, 2]], {(0, 2)}, RUN),  # ... or dropped from both (Codex overlay review P2-1)
    ([[0, 2], [1, 2], [0, 1]], set(), RUN),  # a pair nobody timed out
    (PAIRS + [[0, 2]], set(), RUN),  # a pair listed twice
    (PAIRS, set(), "old"),  # a list registered for another run
])
def test_the_overlay_must_hold_exactly_the_registered_selection(tmp_path, pairs, skip, run):
    args = timed_out_run(tmp_path)
    args.overlay = overlay(tmp_path, RERUN, pairs=pairs, skip=skip, run=run)
    with pytest.raises(SystemExit):
        D7.judge(args)


def test_a_second_record_under_another_name_is_refused(tmp_path):
    # Codex overlay review P2-2: combo_0/seed_2.json beside combo_00/seed_2.json.
    args = timed_out_run(tmp_path)
    args.overlay = overlay(tmp_path, RERUN)
    (args.overlay / "combo_0").mkdir()
    (args.overlay / "combo_0" / "seed_2.json").write_text(json.dumps(record(0, 2, LOOPY, run="foreign")))
    with pytest.raises(SystemExit):
        D7.judge(args)


@pytest.mark.parametrize("sidecar", [{"combo": 47, "seed": 999},  # another pair's file
                                     {"mission_ladder_s": [0.5, 11.0]},  # another execution's times (2nd P2)
                                     {"record_sha256": "0" * 64},  # a hash of another record
                                     {"trace": [{"stage": "observe_candidate", "candidate_index": 3,
                                                 "extended": True, "status": "success"}]}])  # another run's trace
def test_a_control_sidecar_must_belong_to_its_record(tmp_path, sidecar):
    args = timed_out_run(tmp_path)
    args.overlay = overlay(tmp_path, RERUN, sidecar=sidecar)
    with pytest.raises(SystemExit):
        D7.judge(args)


def test_a_sidecar_that_names_its_records_hash_passes_on_the_hash(tmp_path):
    import hashlib
    args = timed_out_run(tmp_path)
    args.overlay = overlay(tmp_path, RERUN)
    path = args.overlay / "combo_00" / "seed_2.instrument.json"
    inst = json.loads(path.read_text())
    record_bytes = (args.overlay / "combo_00" / "seed_2.json").read_bytes()
    path.write_text(json.dumps({**inst, "record_sha256": hashlib.sha256(record_bytes).hexdigest()}))
    assert D7.judge(args)["control"]["passed"]


def test_the_runners_d7_options_are_the_adopted_combination():
    # Plan D7 CPU judgement (judge_rerun.json): combination 3, as changes from combination 0.
    from forklift_core.planning.pallet_mission import D7_PLANNER_OPTIONS
    adopted = {k: v for k, v in D7.COMBOS[3].items() if v != D7.COMBOS[0][k]}
    assert D7_PLANNER_OPTIONS == adopted
