"""Post-fix baseline live comparison (docs/validation/2026-11-15-post-fix-baseline/live_compare.py, plan
docs/plans/2026-11-15-post-fix-baseline.md 2.3 and 4.1 column L) on synthetic missions.csv files and the pre-fix
status survey's own missions.csv."""

import csv
import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
_spec = importlib.util.spec_from_file_location(
    "live_compare", ROOT / "docs/validation/2026-11-15-post-fix-baseline/live_compare.py")
lc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(lc)

SURVEY = ROOT / "docs/validation/2026-11-06-ct-status-survey/missions.csv"
# ct_survey.py writes more columns than the comparison reads; keep a few extra ones to show they are not needed
HEADER = ["arm", "scene", "session", "order", "start", "start_evidence", "outcome", "reason", "stage", "substage",
          "class", "forbidden_contacts", "delivery_error", "started_s", "ended_s", "indicators"]


def m(arm, scene, seed, order, start="confirmed", outcome="done", reason="", stage="", cls="", de="0.01",
      t0=None, t1=None, fc="0", prefix=None):
    t0 = (order - 1) * 150.0 + 1.0 if t0 is None else t0
    t1 = t0 + 100.0 + seed if t1 is None else t1
    return {"arm": arm, "scene": scene, "session": f"{prefix or arm}_{scene}_s{seed}", "order": order, "start": start,
            "start_evidence": "probe", "outcome": outcome, "reason": reason, "stage": stage, "substage": "",
            "class": cls, "forbidden_contacts": fc, "delivery_error": de, "started_s": t0, "ended_s": t1,
            "indicators": "{}"}


def not_submitted(arm, scene, seed, order):
    return m(arm, scene, seed, order, start="not_submitted", outcome="", de="", t0="", t1="", fc="")


def failed(arm, scene, seed, order, reason="near_capture_failed:perception_no_pallet:no_expected_candidate",
           stage="observe", cls="perception"):
    return m(arm, scene, seed, order, outcome="failed", reason=reason, stage=stage, cls=cls, de="")


def write(path: Path, rows) -> Path:
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=HEADER)
        w.writeheader()
        w.writerows(rows)
    return path


def full(arm, scene, seed, **kw):
    return [m(arm, scene, seed, k, **kw) for k in (1, 2, 3)]


def group(rows, arm="LD", scene="line_m", side="new", arm_map=None):
    new = [r for r in rows if side == "new"]
    pre = [r for r in rows if side == "prefix"]
    return lc.compare(new, pre, arm_map or {"LD": "D"})["arms"][arm]["scenes"][scene][side]


def load(tmp_path, rows, name="x.csv"):
    return lc.read_missions(write(tmp_path / name, rows))


def test_counts_complete_failed_and_unsubmitted_sessions(tmp_path):
    rows = (full("LD", "line_m", 0) + full("LD", "line_m", 1)
            + [m("LD", "line_m", 2, 1), failed("LD", "line_m", 2, 2), not_submitted("LD", "line_m", 2, 3)])
    g = group(load(tmp_path, rows))
    assert g["sessions"] == 3
    assert g["sessions_all_done"] == 2
    assert g["missions"] == {"scheduled": 9, "listed": 9, "confirmed": 8, "done": 7, "failed": 1,
                             "not_submitted": 1, "start_other": {}}
    assert g["failed_missions"] == [{"session": "LD_line_m_s2", "order": 2, "outcome": "failed",
                                     "reason": "near_capture_failed:perception_no_pallet:no_expected_candidate"}]
    assert g["forbidden_contacts"] == {"total": 0, "missing": 0}


def test_other_start_states_are_counted_apart(tmp_path):
    rows = [m("LD", "line_m", 0, 1), m("LD", "line_m", 0, 2, start="unknown", outcome="no_record", de=""),
            m("LD", "line_m", 0, 3, start="rejected", outcome="", de="", t0="", t1="", fc="")]
    g = group(load(tmp_path, rows))
    assert g["missions"]["confirmed"] == 1
    assert g["missions"]["failed"] == 0  # failure is a confirmed start that did not finish
    assert g["missions"]["not_submitted"] == 0
    assert g["missions"]["start_other"] == {"unknown": 1, "rejected": 1}
    assert g["sessions_all_done"] == 0


def test_classes_stages_and_statistics_with_missing_cells(tmp_path):
    rows = [m("LD", "random_m", 0, 1, de="0.01", t0=0.0, t1=100.0),
            m("LD", "random_m", 0, 2, de="0.03", t0=100.0, t1=300.0),
            m("LD", "random_m", 0, 3, de="", t0=300.0, t1=""),         # both missing
            m("LD", "random_m", 1, 1, de="abc", t0="x", t1=50.0),       # not numbers
            m("LD", "random_m", 1, 2, de="0.02", t0=0.0, t1=400.0, fc="2"),
            m("LD", "random_m", 1, 3, de="nan", t0=0.0, t1=150.0, fc=""),  # non-finite; contacts blank
            failed("LD", "random_m", 2, 1, reason="Tracking failed in transport", stage="transport", cls="tracking"),
            failed("LD", "random_m", 3, 1, reason="Tracking failed in transport", stage="transport", cls="tracking"),
            failed("LD", "random_m", 4, 1),
            m("LD", "random_m", 5, 1, outcome="no_record", de="", stage="", cls="")]
    g = group(load(tmp_path, rows), scene="random_m")
    assert g["failed_classes"] == {"tracking": 2, "perception": 1, "(empty)": 1}
    assert g["failed_stages"] == {"transport": 2, "observe": 1, "(empty)": 1}
    de = g["delivery_error_m"]
    assert de["n"] == 3 and de["missing"] == 3
    assert de["median"] == pytest.approx(0.02)
    assert de["p90"] == pytest.approx(0.028)  # numpy linear: 0.02 + 0.8 * 0.01
    assert de["max"] == pytest.approx(0.03)
    t = g["mission_time_s"]
    assert t["n"] == 4 and t["missing"] == 2
    assert t["median"] == pytest.approx(175.0)
    assert t["p90"] == pytest.approx(340.0)  # sorted 100, 150, 200, 400: index 2.7
    assert t["max"] == pytest.approx(400.0)
    assert g["forbidden_contacts"] == {"total": 2, "missing": 1}
    # a failed mission's delivery error is not a done mission's statistic
    assert "failed" not in de


def test_empty_group_has_no_statistics(tmp_path):
    g = group(load(tmp_path, full("LD", "line_m", 0)), scene="random_m")
    assert g["sessions"] == 0 and g["missions"]["scheduled"] == 0
    assert g["delivery_error_m"] == {"n": 0, "median": None, "p90": None, "max": None, "missing": 0}


def test_arm_map_pairs_and_ignores_arms(tmp_path):
    new = load(tmp_path, full("LD", "line_m", 0) + full("LO", "line_m", 0) + full("ZZ", "line_m", 0), "new.csv")
    pre = load(tmp_path, full("D", "line_m", 0) + full("O6", "line_m", 0, de="0.05")
               + full("Dp", "line_m", 0) + full("O0", "line_m", 0) + full("Dp", "line_m", 0, prefix="Dp2"),
               "pre.csv")  # Dp's seed 0 twice: an ignored arm is not keyed
    out = lc.compare(new, pre, lc.parse_arm_map("LD=D, LO=O6"))
    assert set(out["arms"]) == {"LD", "LO"}
    assert out["arms"]["LD"]["prefix_arm"] == "D" and out["arms"]["LO"]["prefix_arm"] == "O6"
    assert out["ignored_arms"] == {"new": ["ZZ"], "prefix": ["Dp", "O0"]}
    lo = out["arms"]["LO"]["scenes"]["line_m"]
    assert lo["prefix"]["delivery_error_m"]["median"] == pytest.approx(0.05)
    assert lo["prefix"]["sessions"] == 1 and lo["new"]["sessions"] == 1
    assert lo["pairs"] == {"common_seeds": [0], "only_new_seeds": [], "only_prefix_seeds": [], "changed": []}
    assert set(out["arms"]["LD"]["scenes"]) == {"line_m", "random_m"}


def test_pairs_list_outcome_changes_by_seed_and_order(tmp_path):
    new = load(tmp_path, [m("LD", "random_m", 3, 1), failed("LD", "random_m", 3, 2),
                          not_submitted("LD", "random_m", 3, 3)] + full("LD", "random_m", 4)
               + full("LD", "random_m", 6), "new.csv")
    pre = load(tmp_path, full("D", "random_m", 3, t1=999.0) + full("D", "random_m", 4, de="0.2")
               + full("D", "random_m", 5), "pre.csv")
    pairs = lc.compare(new, pre, {"LD": "D"})["arms"]["LD"]["scenes"]["random_m"]["pairs"]
    assert pairs["common_seeds"] == [3, 4]
    assert pairs["only_new_seeds"] == [6] and pairs["only_prefix_seeds"] == [5]
    assert pairs["changed"] == [
        {"seed": 3, "order": 2,
         "new": {"session": "LD_random_m_s3", "outcome": "failed", "listed": True,
                 "reason": "near_capture_failed:perception_no_pallet:no_expected_candidate", "stage": "observe"},
         "prefix": {"session": "D_random_m_s3", "outcome": "done", "reason": None, "stage": None, "listed": True}},
        {"seed": 3, "order": 3,
         "new": {"session": "LD_random_m_s3", "outcome": None, "reason": None, "stage": None, "listed": True},
         "prefix": {"session": "D_random_m_s3", "outcome": "done", "reason": None, "stage": None, "listed": True}}]


def test_pairs_count_a_mission_listed_on_one_side(tmp_path):
    new = load(tmp_path, full("LD", "line_m", 0) + [m("LD", "line_m", 0, 4)], "new.csv")
    pre = load(tmp_path, full("D", "line_m", 0), "pre.csv")
    changed = lc.compare(new, pre, {"LD": "D"})["arms"]["LD"]["scenes"]["line_m"]["pairs"]["changed"]
    assert [(c["seed"], c["order"], c["prefix"]["listed"]) for c in changed] == [(0, 4, False)]


def test_duplicate_key_is_an_error(tmp_path):
    rows = full("LD", "line_m", 0) + [m("LD", "line_m", 0, 2, prefix="retry_LD")]
    with pytest.raises(ValueError, match="appears twice"):
        lc.compare(load(tmp_path, rows), [], {"LD": "D"})
    with pytest.raises(ValueError, match="appears twice"):
        lc.compare([], load(tmp_path, full("D", "line_m", 1) + full("D", "line_m", 1)), {"LD": "D"})


def test_malformed_rows_and_maps_are_errors(tmp_path):
    with pytest.raises(ValueError, match="_s<seed>"):
        lc.compare(load(tmp_path, [dict(m("LD", "line_m", 0, 1), session="LD_line_m")]), [], {"LD": "D"})
    with pytest.raises(ValueError, match="not an integer"):
        lc.compare(load(tmp_path, [dict(m("LD", "line_m", 0, 1), order="first")]), [], {"LD": "D"})
    bad = tmp_path / "bad.csv"
    bad.write_text("arm,scene,session,order\nLD,line_m,LD_line_m_s0,1\n")
    with pytest.raises(ValueError, match="missing columns"):
        lc.read_missions(bad)
    for text in ("", "LD", "LD=", "LD=D,LD=O6"):
        with pytest.raises(ValueError):
            lc.parse_arm_map(text)


def test_main_writes_schema(tmp_path):
    new = write(tmp_path / "new.csv", full("LD", "line_m", 0))
    pre = write(tmp_path / "pre.csv", full("D", "line_m", 0) + full("Dp", "line_m", 0))
    out = tmp_path / "out.json"
    assert lc.main(["--new", str(new), "--prefix", str(pre), "--arm-map", "LD=D,LO=O6", "--out", str(out)]) == 0
    res = json.loads(out.read_text())
    assert res["schema"] == "post_fix_live_compare/1"
    assert res["arm_map"] == {"LD": "D", "LO": "O6"} and res["scheduled_per_session"] == 3
    assert res["ignored_arms"] == {"new": [], "prefix": ["Dp"]}
    assert res["arms"]["LD"]["scenes"]["line_m"]["new"]["missions"]["done"] == 3
    assert res["arms"]["LO"]["scenes"]["line_m"]["new"]["sessions"] == 0


def test_prefix_survey_counts_match_its_record():
    """The pre-fix status survey's own missions.csv reads as its summary item 2 table: D 16 sessions all complete with
    48 of 48 done; O6 4 sessions, 3 complete, 11 started, 10 done, 1 not submitted."""
    pre = lc.read_missions(SURVEY)
    out = lc.compare([], pre, {"LD": "D", "LO": "O6"})
    assert out["ignored_arms"]["prefix"] == ["Dp", "O0"]

    def total(arm, path):
        cells = [out["arms"][arm]["scenes"][s]["prefix"] for s in ("line_m", "random_m")]
        vals = []
        for c in cells:
            for k in path:
                c = c[k]
            vals.append(c)
        return sum(vals)

    assert total("LD", ["sessions"]) == 16
    assert total("LD", ["sessions_all_done"]) == 16
    for k, v in {"scheduled": 48, "listed": 48, "confirmed": 48, "done": 48, "failed": 0, "not_submitted": 0}.items():
        assert total("LD", ["missions", k]) == v, k
    assert total("LO", ["sessions"]) == 4
    assert total("LO", ["sessions_all_done"]) == 3
    for k, v in {"scheduled": 12, "listed": 12, "confirmed": 11, "done": 10, "failed": 1, "not_submitted": 1}.items():
        assert total("LO", ["missions", k]) == v, k
    lo = out["arms"]["LO"]["scenes"]["random_m"]["prefix"]
    assert lo["failed_missions"] == [{"session": "O6_random_m_s0", "order": 2, "outcome": "failed",
                                      "reason": "near_capture_failed:perception_no_pallet:no_expected_candidate"}]
    assert lo["failed_classes"] == {"perception": 1} and lo["failed_stages"] == {"observe": 1}
    for arm in ("LD", "LO"):
        assert total(arm, ["forbidden_contacts", "total"]) == 0
        assert total(arm, ["forbidden_contacts", "missing"]) == 0
        assert total(arm, ["delivery_error_m", "missing"]) == 0
        assert total(arm, ["mission_time_s", "missing"]) == 0
    assert total("LD", ["delivery_error_m", "n"]) == 48 and total("LO", ["mission_time_s", "n"]) == 10


def test_confirmed_without_outcome_is_a_failure_shown_as_empty(tmp_path):
    """Codex review 1 P2: a confirmed start with no outcome counts as a failure and says so in its outcome."""
    rows = load(tmp_path, [m("LD", "line_m", 0, 1), m("LD", "line_m", 0, 2, outcome="", de=""),
                           m("LD", "line_m", 0, 3, outcome="no_record", de="")])
    g = group(rows)
    assert g["missions"]["failed"] == 2
    assert g["failed_outcomes"] == {lc.EMPTY: 1, "no_record": 1}
    assert [f["outcome"] for f in g["failed_missions"]] == [lc.EMPTY, "no_record"]


def test_done_with_blank_or_text_numbers_is_missing(tmp_path):
    rows = load(tmp_path, [m("LD", "line_m", 0, 1, de=""), m("LD", "line_m", 0, 2, de="abc", t1=""),
                           m("LD", "line_m", 0, 3)])
    g = group(rows)
    assert g["delivery_error_m"]["n"] == 1 and g["delivery_error_m"]["missing"] == 2
    assert g["mission_time_s"]["n"] == 2 and g["mission_time_s"]["missing"] == 1


@pytest.mark.parametrize("bad", [0, -1, True, 2.5])
def test_scheduled_must_be_a_positive_whole_number(tmp_path, bad):
    rows = load(tmp_path, full("LD", "line_m", 0))
    with pytest.raises(ValueError, match="scheduled"):
        lc.compare(rows, [], {"LD": "D"}, scheduled=bad)
