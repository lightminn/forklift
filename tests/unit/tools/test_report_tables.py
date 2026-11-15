"""Post-fix baseline report tables (docs/validation/2026-11-15-post-fix-baseline/report_tables.py, plan
docs/plans/2026-11-15-post-fix-baseline.md 3 and 4) on outputs the three tools themselves write from synthetic runs."""

import copy
import csv
import importlib.util
import json
from pathlib import Path

import pytest

from tools import deterministic_compare as dc

ROOT = Path(__file__).resolve().parents[3]
FOLDER = ROOT / "docs/validation/2026-11-15-post-fix-baseline"


def _load(name):
    spec = importlib.util.spec_from_file_location(name, FOLDER / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


rt = _load("report_tables")
bs = _load("baseline_summary")
lc = _load("live_compare")

# --- synthetic riib runs (the record shape test_baseline_summary.py fixes) --------------------------------------------

TRANSITIONS = [{"from": "observe", "to": "approach", "time_s": 40.0},
               {"from": "extract", "to": "transport", "time_s": 70.0},
               {"from": "transport", "to": "lower", "time_s": 150.0},
               {"from": "withdraw", "to": "return_home", "time_s": 170.0},
               {"from": "return_home", "to": "complete", "time_s": 250.0}]
ARGS = {"deterministic": True, "path_display": "overlay"}
PRE_ARGS = {"deterministic": True}


def result(**over):
    r = {"success": True, "phase": "complete", "failure_reason": None, "arguments": dict(ARGS),
         "observation_attempts": [{"recognition": {"status": "selected"}, "pocket_observation": {"status": "valid"}},
                                  {"recognition": None, "pocket_observation": {"status": "valid"}}],
         "observe_replans": [], "near_capture": {"status": "done", "far_attempt": 1, "near_attempt": 2},
         "transitions": copy.deepcopy(TRANSITIONS), "tracking_failure": None,
         "cusp_replans": [{"time_s": 100.0}, {"time_s": 200.0}],
         "delivery_error_m": 0.0123, "simulated_time_s": 250.04, "forbidden_pocket_contacts": [],
         "deterministic": {"enabled": True, "broken": []}}
    r.update(over)
    return r


FAIL = dict(success=False, phase="transport",
            failure_reason="Tracking failed in transport: pos=2.99,yaw=1.01 | " + "x" * 80,
            tracking_failure={"phase": "transport", "tracking": {"failure": "cross_track"}},
            delivery_error_m=None, simulated_time_s=None)


def write_run(root: Path, name: str, res: dict, header="overlay"):
    d = root / name / "run"
    d.mkdir(parents=True)
    (d / "result.json").write_text(json.dumps(res))
    if header is not None:
        (d / "path_display.jsonl").write_text(json.dumps({"t": "header", "mode": header}) + "\n")


def riib_summary(tmp_path, match=False, compare=True) -> dict:
    root, comp = tmp_path / "R", tmp_path / "B0"
    write_run(root, "pfb_R_random_1_s0", result())
    write_run(root, "pfb_R_random_1_s1", result(delivery_error_m=0.0201, simulated_time_s=260.0))
    write_run(root, "pfb_R_random_1_s2", result())
    write_run(root, "pfb_R_line_2_s0", result(**FAIL))
    pre = PRE_ARGS
    write_run(comp, "lcr_B0_random_1_s0", result(arguments=pre), header=None)
    write_run(comp, "lcr_B0_random_1_s1", result(arguments=pre, **FAIL), header=None)
    no_near = result(arguments=pre)
    no_near.pop("near_capture")
    write_run(comp, "lcr_B0_line_2_s0", no_near, header=None)
    out = tmp_path / "s.json"
    argv = [str(root), "--mode", "overlay", "--out", str(out)]
    if compare:
        argv += ["--compare", str(comp), "--compare-mode", "prefix"] + (["--match-compare"] if match else [])
    assert bs.main(argv) == 0
    return json.loads(out.read_text())


def rows_of(md: str, heading: str) -> dict:
    """{first cell: [cells]} of the first table after ``heading``."""
    lines = md.splitlines()
    i = next(k for k, line in enumerate(lines) if line.startswith(heading))
    while not lines[i].startswith("|"):
        i += 1
    header = [c.strip() for c in lines[i].strip("|").split(" | ")]
    out = {"__header__": header}
    for line in lines[i + 2:]:
        if not line.startswith("|"):
            break
        cells = [c.strip() for c in line[2:-2].split(" | ")]
        out.setdefault(cells[0], cells) if cells[0] not in out else out.setdefault(f"{cells[0]}#{len(out)}", cells)
    return out


def col(rows: dict, row: str, name: str) -> str:
    return rows[row][rows["__header__"].index(name)]


def test_riib_single_set(tmp_path):
    md = rt.render([("R 단독", riib_summary(tmp_path, compare=False))], None, [])
    rows = rows_of(md, "### riib: R 단독")
    assert list(k for k in rows if k != "__header__") == ["random_1", "line_2", "합계"]
    assert col(rows, "random_1", "n") == "3"
    assert col(rows, "random_1", "완주 k/n") == "3/3"
    assert col(rows, "합계", "완주 k/n") == "3/4"
    assert col(rows, "random_1", "근접 최종 상태") == "done 3"
    assert col(rows, "random_1", "근접 시도 valid/n") == "3/3"
    # two cusp replans per run, one in transport (70-150 s) and one on the way home
    assert col(rows, "random_1", "운반 cusp 재계획 total(실행 수)") == "3(3)"
    assert col(rows, "line_2", "운반 이탈(추종 실패)") == "cross_track 1·endpoint_heading 0"
    assert col(rows, "line_2", "실패 분류") == "tracking 1"
    assert col(rows, "random_1", "실패 분류") == "—"
    # m -> mm, one decimal: median 12.3, p90 of (12.3, 12.3, 20.1) 18.5, max 20.1
    assert col(rows, "random_1", "하역 오차 중앙값·p90·최대 mm") == "12.3·18.5·20.1"
    assert col(rows, "random_1", "시뮬 시간 중앙값·p90 s") == "250.0·258.0"
    # no finished run: no statistic
    assert col(rows, "line_2", "하역 오차 중앙값·p90·최대 mm") == "비교 불가"
    assert " / " not in col(rows, "random_1", "완주 k/n")


def test_riib_compare_cells_are_a_slash_b_and_not_comparable(tmp_path):
    md = rt.render([("R vs 곡률 B0", riib_summary(tmp_path))], None, [])
    rows = rows_of(md, "### riib: R vs 곡률 B0")
    assert "칸은 a / b" in md
    assert col(rows, "random_1", "n") == "3 / 2"
    assert col(rows, "random_1", "완주 k/n") == "3/3 / 1/2"
    assert col(rows, "합계", "완주 k/n") == "3/4 / 2/3"
    # B0's line_2 run has no near_capture: the metric reads as not comparable and names the unread field
    assert col(rows, "line_2", "근접 최종 상태") == "done 1 / 비교 불가 (결측 near_capture 1)"
    assert col(rows, "line_2", "근접 시도 valid/n") == "1/1 / 비교 불가 (결측 near_capture 1)"
    assert col(rows, "line_2", "완주 k/n") == "0/1 / 1/1"
    assert col(rows, "random_1", "하역 오차 중앙값·p90·최대 mm") == "12.3·18.5·20.1 / 12.3·12.3·12.3"


def test_riib_pair_table(tmp_path):
    md = rt.render([("R vs 곡률 B0", riib_summary(tmp_path))], None, [])
    assert "- 공통 3 · 바뀐 짝 2 · only_a 1 · only_b 0" in md
    rows = rows_of(md, "#### 짝 차이")
    assert rows["__header__"][:3] == ["장면", "seed", "a 완주"]
    changed = {(v[0], v[1]): v for k, v in rows.items() if k != "__header__"}
    assert set(changed) == {("random_1", "1"), ("line_2", "0")}
    r1 = changed[("random_1", "1")]
    assert r1[2:5] == ["완주", "complete", "—"]
    assert r1[5:7] == ["실패", "transport"]
    # the reason: first 60 characters, the pipe escaped
    assert r1[7] == rt.esc(FAIL["failure_reason"][:60])
    assert "\\|" in r1[7]


def test_riib_matched_aggregates_are_used(tmp_path):
    data = riib_summary(tmp_path, match=True)
    assert data["compare"]["matched"]["n"] == 3
    md = rt.render([("R vs K1(40)", data)], None, [])
    rows = rows_of(md, "### riib: R vs K1(40)")
    assert "공통 (장면, seed) 3 실행으로 맞춘 집계" in md
    # random_1 s2 has no B0 pair: the matched a-side holds two random_1 runs, not three
    assert col(rows, "random_1", "n") == "2 / 2"
    assert col(rows, "합계", "n") == "3 / 3"


def test_riib_scene_absent_on_one_side(tmp_path):
    data = riib_summary(tmp_path)
    data["compare"]["aggregate"]["scenes"].pop("line_2")
    rows = rows_of(rt.render([("x", data)], None, []), "### riib: x")
    assert col(rows, "line_2", "n") == "1 / 0"
    assert col(rows, "line_2", "완주 k/n") == "0/1 / 비교 불가"


def test_riib_schema_is_checked(tmp_path):
    with pytest.raises(ValueError, match="schema"):
        rt.render([("x", {"aggregate": {}})], None, [])


# --- live -------------------------------------------------------------------------------------------------------------

HEADER = ["arm", "scene", "session", "order", "start", "outcome", "reason", "stage", "class", "delivery_error",
          "started_s", "ended_s", "forbidden_contacts"]


def m(arm, scene, seed, order, start="confirmed", outcome="done", reason="", stage="", cls="", de="0.01", t0=None,
      t1=None):
    t0 = (order - 1) * 150.0 if t0 is None else t0
    t1 = t0 + 100.0 if t1 is None else t1
    return {"arm": arm, "scene": scene, "session": f"{arm}_{scene}_s{seed}", "order": order, "start": start,
            "outcome": outcome, "reason": reason, "stage": stage, "class": cls, "delivery_error": de,
            "started_s": t0, "ended_s": t1, "forbidden_contacts": "0"}


def write_csv(path, rows):
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=HEADER)
        w.writeheader()
        w.writerows(rows)
    return path


def live_json(tmp_path) -> dict:
    fail = dict(outcome="failed", reason="near_capture_failed:perception_no_pallet", stage="observe",
                cls="perception", de="")
    new = ([m("LD", "line_m", 0, k) for k in (1, 2, 3)]
           + [m("LD", "line_m", 1, 1), m("LD", "line_m", 1, 2, **fail),
              m("LD", "line_m", 1, 3, start="not_submitted", outcome="", de="", t0="", t1="")]
           + [m("LO", "random_m", 0, 1, de=""), m("LO", "random_m", 0, 2, **fail),
              m("LO", "random_m", 0, 3, start="unknown", outcome="no_record", de="")])
    pre = ([m("D", "line_m", s, k, de="0.02") for s in (0, 1) for k in (1, 2, 3)]
           + [m("O6", "random_m", 0, k) for k in (1, 2, 3)])
    out = tmp_path / "live.json"
    assert lc.main(["--new", str(write_csv(tmp_path / "new.csv", new)), "--prefix",
                    str(write_csv(tmp_path / "pre.csv", pre)), "--arm-map", "LD=D,LO=O6", "--out", str(out)]) == 0
    return json.loads(out.read_text())


def test_live_table(tmp_path):
    md = rt.render([], live_json(tmp_path), [])
    assert "칸은 새 / 수정 전" in md
    rows = rows_of(md, "### 라이브")
    body = {(v[0], v[1]): v for k, v in rows.items() if k != "__header__"}
    hdr = rows["__header__"]
    ld = body[("LD(수정 전 D)", "line_m")]
    assert ld[hdr.index("세션")] == "2 / 2"
    assert ld[hdr.index("세 임무 완주 세션")] == "1/2 / 2/2"
    assert ld[hdr.index("예정·시작 확인·done·실패·미제출")] == "6·5·4·1·1 / 6·6·6·0·0"
    assert ld[hdr.index("실패 class")] == "perception 1 / —"
    assert ld[hdr.index("하역 오차 중앙값·p90·최대 mm")] == "10.0·10.0·10.0 / 20.0·20.0·20.0"
    assert ld[hdr.index("임무 시간 중앙값·p90 s")] == "100.0·100.0 / 100.0·100.0"
    lo = body[("LO(수정 전 O6)", "random_m")]
    assert lo[hdr.index("그 밖의 시작 상태")] == "unknown 1 / —"
    # the one done mission has no delivery error: no statistic, the unread value counted
    assert lo[hdr.index("하역 오차 중앙값·p90·최대 mm")] == "비교 불가 (결측 1) / 10.0·10.0·10.0"
    # an arm and scene with no session on either side
    empty = body[("LO(수정 전 O6)", "line_m")]
    assert empty[hdr.index("세션")] == "0 / 0"
    assert empty[hdr.index("하역 오차 중앙값·p90·최대 mm")] == "비교 불가 / 비교 불가"


def test_live_pairs_only_for_listed_arms(tmp_path):
    data = live_json(tmp_path)
    md = rt.render([], data, [])
    rows = rows_of(md, "#### seed·order 짝에서 outcome 이 다른 임무")
    body = [v for k, v in rows.items() if k != "__header__"]
    assert [(r[0], r[1], r[2], r[3], r[4], r[7]) for r in body] == [
        ("LD", "line_m", "1", "2", "failed", "done"), ("LD", "line_m", "1", "3", "(빈 값)", "done")]
    assert "- LD line_m: 공통 seed 2" in md
    assert "LO random_m:" not in md  # plan 7: L-O is not paired by seed
    md_all = rt.render([], data, [], pair_arms=("LD", "LO"))
    assert "- LO random_m: 공통 seed 1" in md_all


# --- determinism ------------------------------------------------------------------------------------------------------

def det_run(path: Path, value=1, deterministic=True, broken=()):
    path.mkdir(parents=True)
    det = {"enabled": True, "broken": list(broken)} if deterministic else {}
    (path / "result.json").write_text(json.dumps({"value": value, "deterministic": det}))
    return path


def det_json(tmp_path, name, a_kw, b_kw) -> Path:
    a = det_run(tmp_path / name / "a", **a_kw)
    b = det_run(tmp_path / name / "b", **b_kw)
    out = tmp_path / f"{name}.json"
    dc.main([str(a), str(b), "--json", str(out)])
    return out


def test_determinism_table(tmp_path, capsys):
    same = det_json(tmp_path, "rep_random_1_s0", {}, {})
    diff = det_json(tmp_path, "rep_line_2_s0", {}, {"value": 2})
    bad = det_json(tmp_path, "rep_aisle_rack_s0", {}, {"deterministic": False})
    brk = det_json(tmp_path, "rep_random_1_s1", {"broken": ["clock"]}, {"broken": ["clock"]})
    out = tmp_path / "t.md"
    assert rt.main(["--determinism", str(same), str(diff), str(bad), str(brk), "--out", str(out)]) == 0
    rows = rows_of(out.read_text(), "### 결정성 반복")
    assert rows["rep_random_1_s0"][1:3] == ["같음", "—"]
    assert rows["rep_line_2_s0"][1] == "다름"
    assert rows["rep_line_2_s0"][2] == json.loads(diff.read_text())["differences"][0]
    assert rows["rep_line_2_s0"][3] == "1"
    assert rows["rep_aisle_rack_s0"][1] == "비교 불가"
    assert "no deterministic block" in rows["rep_aisle_rack_s0"][2]
    # the same broken reason on both runs: different, no file difference
    assert rows["rep_random_1_s1"][1:3] == ["다름", "broken A: clock"]


def test_determinism_named_and_checked(tmp_path, capsys):
    same = det_json(tmp_path, "p", {}, {})
    md = rt.render([], None, [("R random_1 s0", json.loads(same.read_text()))])
    assert "| R random_1 s0 | 같음 |" in md
    with pytest.raises(ValueError, match="missing keys"):
        rt.render([], None, [("x", {"equal": True})])
    assert rt._det_arg(f"반복 짝={same}") == ("반복 짝", same)


# --- CLI --------------------------------------------------------------------------------------------------------------

def test_main_writes_all_sections(tmp_path, capsys):
    s = tmp_path / "riib.json"
    s.write_text(json.dumps(riib_summary(tmp_path / "r", match=True)))
    live = tmp_path / "live.json"
    live.write_text(json.dumps(live_json(tmp_path)))
    rep = det_json(tmp_path, "rep_x", {}, {})
    out = tmp_path / "tables.md"
    assert rt.main(["--riib", f"R vs S(40)={s}", "--live", str(live), "--determinism", str(rep),
                    "--out", str(out)]) == 0
    md = out.read_text()
    for heading in ("### riib: R vs S(40)", "#### 짝 차이", "### 라이브 연속 이송", "#### seed·order 짝",
                    "### 결정성 반복"):
        assert heading in md
    assert "None" not in md and "not_comparable" not in md
