"""Post-fix Isaac baseline: the result document's tables from the aggregation outputs (plan
docs/plans/2026-11-15-post-fix-baseline.md 3 and 4, RUNBOOK step 6). Tables only -- no verdict, threshold or reading.

Inputs (each optional; a table is written for what is given):

- ``--riib NAME=PATH`` (repeatable): a ``baseline_summary.py --out`` JSON. With ``compare`` each cell is ``a / b``
  (a = the summary's own root, b = ``--compare``); with ``compare.matched`` (``--match-compare``) the matched aggregates
  are used. Then (a) a per-scene table and, with ``compare``, (b) the changed pairs.
- ``--live PATH``: a ``live_compare.py`` JSON -> (c) arm x scene, cells ``new / pre-fix``, and the seed-order pairs whose
  outcome differs for the arms in ``--live-pair-arms`` (default ``LD``: plan 7 does not pair L-O by seed).
- ``--determinism [NAME=]PATH ...``: ``tools/deterministic_compare.py --json`` outputs, one per repeat pair (name =
  the file stem unless given) -> (d) same / different / not comparable and the tool's first difference.

Numbers are written as recorded; only the delivery error (m -> mm) and times (s) are rounded, to one decimal. A
``not_comparable`` or ``None`` value is written ``비교 불가``; a statistic with unread values carries ``(결측 k)``.

  python report_tables.py --riib 'R vs 곡률 B0=R_vs_lcrB0.json' --live L_vs_cts.json \\
      --determinism rep_*.json --out tables.md
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

NA = "비교 불가"
EMPTY_COUNT = "없음"  # a counted field with no entry (e.g. no failure)
NONE = "—"
SCENE_ORDER = ("random_1", "line_2", "aisle_rack")
REASON_CHARS = 60
RIIB_SCHEMA = "post_fix_baseline_summary/1"
LIVE_SCHEMA = "post_fix_live_compare/1"
DET_KEYS = ("equal", "invalid", "differences", "broken")


def esc(text) -> str:
    """A markdown table cell: pipes escaped, line breaks flattened."""
    return str(text).replace("|", "\\|").replace("\r", " ").replace("\n", " ")


def missing_note(k) -> str:
    return f" (결측 {k})" if isinstance(k, int) and k > 0 else ""


def done_missing_note(k) -> str:
    """Completed runs whose value the statistic could not read (its own denominator, not a field audit)."""
    return f" (완주 중 값 없음 {k})" if isinstance(k, int) and k > 0 else ""


def counts(d) -> str:
    """{"done": 59, "pending": 1} -> 'done 59·pending 1' (insertion order of the record); {} -> '없음' (counted,
    none found); None or not_comparable -> '비교 불가' (no run could read it)."""
    if d is None or d == "not_comparable":
        return NA
    if not isinstance(d, dict):
        return esc(d)
    return "·".join(f"{esc(k)} {v}" for k, v in d.items()) if d else EMPTY_COUNT


def ratio(r) -> str:
    if isinstance(r, list) and len(r) == 2:
        return f"{r[0]}/{r[1]}"
    return NA


def stat(v, scale: float = 1.0) -> str:
    return NA if v is None else f"{v * scale:.1f}"


def stats(s: dict | None, keys: tuple, scale: float = 1.0, missing=None) -> str:
    """'median·p90·max' of a statistic, '비교 불가' when it has no value."""
    if not isinstance(s, dict) or s.get("n", 0) == 0 or s.get(keys[0]) is None:
        return NA + done_missing_note(missing)
    return "·".join(stat(s.get(k), scale) for k in keys) + done_missing_note(missing)


def pair(a: str, b: str | None) -> str:
    return a if b is None else f"{a} / {b}"


def table(header: list, rows: list) -> list:
    out = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    out += ["| " + " | ".join(r) + " |" for r in rows]
    return out


def name_path(text: str) -> tuple[str, Path]:
    name, sep, path = text.partition("=")
    if not sep or not name.strip() or not path.strip():
        raise ValueError(f"{text!r} is not NAME=PATH")
    return name.strip(), Path(path.strip())


def load(path: Path) -> dict:
    data = json.loads(Path(path).read_text())
    if not isinstance(data, dict):
        raise ValueError(f"{path}: not a JSON object")
    return data


# --- (a) and (b): riib ------------------------------------------------------------------------------------------------

RIIB_HEADER = ["장면", "n", "완주 k/n", "원거리 첫 시도 채택 k/n", "근접 최종 상태", "근접 시도 valid/n",
               "운반 이탈(추종 실패)", "운반 cusp 재계획 total(실행 수)", "하역 오차 중앙값·p90·최대 mm",
               "시뮬 시간 중앙값·p90 s", "실패 분류", "금지 접촉 / 결정성 깨짐 실행"]


def _miss(g: dict, *keys) -> str:
    """The unread-field counts of a group for the fields a cell reads: one field '(결측 k)', several named."""
    mf = g.get("missing_fields") or {}
    found = [(k, mf[k]) for k in keys if isinstance(mf.get(k), int) and mf[k] > 0]
    if not found:
        return ""
    if len(keys) == 1:
        return missing_note(found[0][1])
    return " (결측 " + "·".join(f"{k} {v}" for k, v in found) + ")"


def riib_cells(g: dict | None) -> list:
    """The cells of one aggregate group (``baseline_summary._group``), in RIIB_HEADER order after the scene."""
    if g is None:
        return ["0"] + [NA] * (len(RIIB_HEADER) - 2)
    succ = g.get("success")
    done = succ[0] if isinstance(succ, list) else None
    de, st = g.get("delivery_error_m") or {}, g.get("simulated_time_s") or {}
    de_miss = done - de.get("n", 0) if done is not None else None
    st_miss = done - st.get("n", 0) if done is not None else None
    na = g.get("near_attempts")
    near_tries = f"{na['valid']}/{na['n']}" if isinstance(na, dict) else NA
    tf = dict(g.get("transport_tracking_failures") or {})
    track = "·".join([f"{k} {tf.pop(k, 0)}" for k in ("cross_track", "endpoint_heading")]
                     + [f"{esc(k)} {v}" for k, v in tf.items()])
    tcr = g.get("transport_cusp_replans")
    cusp = f"{tcr['total']}({tcr['runs_with']})" if isinstance(tcr, dict) else NA
    return [str(g.get("n")),
            ratio(succ) + _miss(g, "success"),
            ratio(g.get("far_selected")) + _miss(g, "observation_attempts"),
            counts(g.get("near_status")) + _miss(g, "near_capture", "near_capture.status"),
            near_tries + _miss(g, "observation_attempts", "near_capture"),
            track,
            cusp + _miss(g, "transitions", "cusp_replans"),
            stats(de, ("median", "p90", "max"), 1000.0, de_miss),
            stats(st, ("median", "p90"), 1.0, st_miss),
            counts(g.get("failure_classes")),
            f"{g.get('forbidden_contacts', NA)} / {g.get('deterministic_broken_runs', NA)}"]  # plan 4.1 safety


def _scenes(*aggs) -> list:
    present = set()
    for agg in aggs:
        if isinstance(agg, dict):
            present |= set(agg.get("scenes") or {})
    return [s for s in SCENE_ORDER if s in present] + sorted(present - set(SCENE_ORDER))


def riib_tables(name: str, data: dict) -> list:
    if data.get("schema") != RIIB_SCHEMA:
        raise ValueError(f"riib {name!r}: schema {data.get('schema')!r}, not {RIIB_SCHEMA!r}")
    comp = data.get("compare")
    matched = comp.get("matched") if isinstance(comp, dict) else None
    if matched is not None:
        a, b = matched["a"], matched["b"]
    else:
        a, b = data["aggregate"], (comp or {}).get("aggregate")
    lines = [f"### riib: {esc(name)}", ""]
    src = f"a: `{esc(data.get('root'))}`(mode {esc(data.get('mode'))})"
    if comp is not None:
        src += f" · b: `{esc(comp.get('root'))}`(mode {esc(comp.get('mode'))}) · 칸은 a / b (a = 이번 기준선)"
        if matched is not None:
            pr = comp.get("pairs") or {}
            src += (f" · 맞댄 짝 {matched.get('n')}(두 쪽 모두 거부되지 않은 공통 장면·seed; 공통 이름 {pr.get('common')},"
                    f" a 에만 {len(pr.get('only_a') or [])}, b 에만 {len(pr.get('only_b') or [])})")
    lines += [f"- {src}",
              f"- 거부 실행: a {len(a.get('refused') or [])}"
              + (f" · b {len(b.get('refused') or [])}" if b is not None else ""), ""]
    rows = []
    for scene, label in [(s, s) for s in _scenes(a, b)] + [("all", "합계")]:
        ga = a.get("all") if scene == "all" else (a.get("scenes") or {}).get(scene)
        ca = riib_cells(ga)
        if b is None:
            rows.append([label] + ca)
        else:
            gb = b.get("all") if scene == "all" else (b.get("scenes") or {}).get(scene)
            rows.append([label] + [pair(x, y) for x, y in zip(ca, riib_cells(gb))])
    lines += table(RIIB_HEADER, rows) + [""]
    if comp is not None:
        lines += pair_table(comp.get("pairs") or {})
    return lines


def _done(v) -> str:
    return NA if v is None else ("완주" if v else "실패")


def _short(text) -> str:
    return NONE if text is None else esc(str(text)[:REASON_CHARS])


def pair_table(pairs: dict) -> list:
    changed = pairs.get("changed") or []
    lines = ["#### 짝 차이(완주·끝 단계·실패 분류가 다른 짝)", "",
             f"- 공통 {pairs.get('common')} · 바뀐 짝 {len(changed)} · only_a {len(pairs.get('only_a') or [])}"
             f" · only_b {len(pairs.get('only_b') or [])}", ""]
    if not changed:
        return lines + ["(바뀐 짝 없음)", ""]
    rows = [[esc(c["scene"]), str(c["seed"]),
             _done(c["a"].get("success")), esc(c["a"].get("phase") or NA), _short(c["a"].get("failure_reason")),
             _done(c["b"].get("success")), esc(c["b"].get("phase") or NA), _short(c["b"].get("failure_reason"))]
            for c in changed]
    return lines + table(["장면", "seed", "a 완주", "a 끝 단계", "a 사유(앞 60 자)",
                          "b 완주", "b 끝 단계", "b 사유(앞 60 자)"], rows) + [""]


# --- (c): live --------------------------------------------------------------------------------------------------------

LIVE_HEADER = ["팔", "장면", "세션", "세 임무 완주 세션", "예정·시작 확인·done·실패·미제출", "그 밖의 시작 상태",
               "실패 class", "하역 오차 중앙값·p90·최대 mm", "임무 시간 중앙값·p90 s"]


def live_cells(g: dict | None) -> list:
    if g is None:
        return [NA] * (len(LIVE_HEADER) - 2)
    m = g.get("missions") or {}
    de, mt = g.get("delivery_error_m") or {}, g.get("mission_time_s") or {}
    return [str(g.get("sessions")),
            f"{g.get('sessions_all_done')}/{g.get('sessions')}",
            "·".join(str(m.get(k, NA)) for k in ("scheduled", "confirmed", "done", "failed", "not_submitted")),
            counts(m.get("start_other")),
            counts(g.get("failed_classes")),
            stats(de, ("median", "p90", "max"), 1000.0, de.get("missing")),
            stats(mt, ("median", "p90"), 1.0, mt.get("missing"))]


def _side(s: dict) -> str:
    if not s.get("listed"):
        return "(행 없음)"
    return esc(s.get("outcome")) if s.get("outcome") is not None else "(빈 값)"


def live_tables(data: dict, pair_arms: tuple) -> list:
    if data.get("schema") != LIVE_SCHEMA:
        raise ValueError(f"live: schema {data.get('schema')!r}, not {LIVE_SCHEMA!r}")
    lines = ["### 라이브 연속 이송", "",
             f"- 새: `{esc(data.get('new'))}` · 수정 전: `{esc(data.get('prefix'))}` · 칸은 새 / 수정 전"
             f" · 세션당 예정 임무 {data.get('scheduled_per_session')}", ""]
    rows, pair_rows, pair_heads = [], [], []
    for arm, a in (data.get("arms") or {}).items():
        for scene, s in (a.get("scenes") or {}).items():
            cells = [pair(x, y) for x, y in zip(live_cells(s.get("new")), live_cells(s.get("prefix")))]
            rows.append([f"{esc(arm)}(수정 전 {esc(a.get('prefix_arm'))})", esc(scene)] + cells)
            if arm not in pair_arms:
                continue
            p = s.get("pairs") or {}
            pair_heads.append(f"- {esc(arm)} {esc(scene)}: 공통 seed {len(p.get('common_seeds') or [])}"
                              f" · 새에만 {p.get('only_new_seeds') or []} · 수정 전에만 {p.get('only_prefix_seeds') or []}"
                              f" · outcome 다른 임무 {len(p.get('changed') or [])}")
            for c in p.get("changed") or []:
                n, q = c.get("new") or {}, c.get("prefix") or {}
                pair_rows.append([esc(arm), esc(scene), str(c.get("seed")), str(c.get("order")),
                                  _side(n), esc(n.get("stage") or "—"), _short(n.get("reason")),
                                  _side(q), esc(q.get("stage") or "—"), _short(q.get("reason"))])
    lines += table(LIVE_HEADER, rows) + [""]
    if pair_arms:
        lines += [f"#### seed·order 짝에서 outcome 이 다른 임무({', '.join(pair_arms)})", ""] + pair_heads + [""]
        if pair_rows:
            lines += table(["팔", "장면", "seed", "order", "새 outcome", "새 단계", "새 사유(앞 60 자)",
                            "수정 전 outcome", "수정 전 단계", "수정 전 사유(앞 60 자)"], pair_rows) + [""]
        else:
            lines += ["(outcome 다른 임무 없음)", ""]
    return lines


# --- (d): determinism repeats -----------------------------------------------------------------------------------------

def determinism_row(name: str, data: dict) -> list:
    absent = [k for k in DET_KEYS if k not in data]
    if absent:
        raise ValueError(f"determinism {name!r}: missing keys {absent} (not a deterministic_compare --json output)")
    invalid = [f"{side}: {p}" for side in ("A", "B") for p in (data["invalid"] or {}).get(side) or []]
    diffs = data["differences"] or []
    broken = [f"{side}: {b}" for side in ("A", "B") for b in (data["broken"] or {}).get(side) or []]
    if invalid:
        verdict, first = NA, invalid[0]
    elif data["equal"]:
        verdict, first = "같음", NONE
    else:
        verdict = "다름"
        first = diffs[0] if diffs else (f"broken {broken[0]}" if broken else NONE)
    return [esc(name), verdict, esc(first), str(len(diffs)), str(len(broken)), str(len(invalid))]


def determinism_tables(items: list) -> list:
    rows = [determinism_row(name, data) for name, data in items]
    return (["### 결정성 반복", ""]
            + table(["짝", "결과", "첫 차이(또는 기록 불완전 사유)", "차이 수", "broken 수", "기록 불완전 수"], rows)
            + [""])


# --- main -------------------------------------------------------------------------------------------------------------

def _det_arg(text: str) -> tuple[str, Path]:
    if "=" in text and not Path(text).exists():
        return name_path(text)
    return Path(text).stem, Path(text)


def render(riib: list, live: dict | None, determinism: list, pair_arms: tuple = ("LD",)) -> str:
    lines: list = []
    for name, data in riib:
        lines += riib_tables(name, data)
    if live is not None:
        lines += live_tables(live, pair_arms)
    if determinism:
        lines += determinism_tables(determinism)
    return "\n".join(lines).rstrip() + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--riib", action="append", default=[], metavar="NAME=PATH",
                    help="baseline_summary.py --out JSON, titled NAME (repeatable)")
    ap.add_argument("--live", type=Path, default=None, help="live_compare.py --out JSON")
    ap.add_argument("--live-pair-arms", default="LD",
                    help="comma-separated new arms whose seed-order pairs are listed (default LD; '' for none)")
    ap.add_argument("--determinism", nargs="+", action="extend", default=[], metavar="[NAME=]PATH",
                    help="deterministic_compare.py --json outputs, one per repeat pair")
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    riib = [(n, load(p)) for n, p in (name_path(x) for x in a.riib)]
    live = load(a.live) if a.live is not None else None
    det = [(n, load(p)) for n, p in (_det_arg(x) for x in a.determinism)]
    pair_arms = tuple(x.strip() for x in a.live_pair_arms.split(",") if x.strip())
    a.out.write_text(render(riib, live, det, pair_arms))
    return 0


if __name__ == "__main__":
    sys.exit(main())
