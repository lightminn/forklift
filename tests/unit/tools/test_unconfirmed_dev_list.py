"""The D sessions' start list (docs/plans/2026-11-06-unconfirmed-validation.md 3.1): fixed before any result, drawn from
seed 4's starts alone, 10 of each pressing rule, half with the mission."""

import importlib.util
import json
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ASSETS = ROOT / "docs/validation/assets/unconfirmed"


def _module():
    spec = importlib.util.spec_from_file_location("make_dev_list", ASSETS / "make_dev_list.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_the_committed_list_is_the_drawn_one_and_balanced():
    mod = _module()
    listed = [json.loads(line) for line in (ASSETS / "dev_starts.jsonl").read_text().splitlines()]
    assert listed == mod.dev_list(mod.rows())
    assert len({r["id"] for r in listed}) == 30
    assert Counter((r["kind"], r["mission"]) for r in listed) == {
        (k, m): 5 for k in ("full", "half", "chatter") for m in (True, False)}
    seed5 = {json.loads(line)["id"] for line in (ASSETS / "s5_starts_rows.jsonl").read_text().splitlines()}
    assert not {r["id"] for r in listed} & seed5  # seed 5 stays closed until the freeze


def test_the_revision_lists_are_the_drawn_ones_outside_the_d_list():
    spec = importlib.util.spec_from_file_location("make_rev_lists", ASSETS / "make_rev_lists.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    out = mod.lists(mod.rows())
    d = {json.loads(line)["id"] for line in (ASSETS / "dev_starts.jsonl").read_text().splitlines()}
    for name, rows in out.items():
        assert [json.loads(line) for line in (ASSETS / name).read_text().splitlines()] == rows
        assert not {r["id"] for r in rows} & d
    assert len(out["tail_starts.jsonl"]) == 20 and len(out["inject_starts.jsonl"]) == 24
    assert Counter(r["inject"] for r in out["inject_starts.jsonl"]) == {k: 3 for k in mod.WRONG}
    assert Counter(r["jump"] for r in out["loop_starts.jsonl"]) == {k: 3 for k in mod.JUMPS}


def test_the_two_small_seed_4_lists_are_the_named_ones():
    # the lead's decision of 2026-11-07: A, the 11 recorded wrong confirmations once each; B, the naturally unconfirmed three
    spec = importlib.util.spec_from_file_location("make_ab_lists", ASSETS / "make_ab_lists.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    a, b = mod.a_list(), mod.b_list()
    assert [json.loads(line) for line in (ASSETS / "a_starts.jsonl").read_text().splitlines()] == a
    assert [json.loads(line) for line in (ASSETS / "b_starts.jsonl").read_text().splitlines()] == b
    assert len(a) == 11 and len({r["id"] for r in a}) == 9
    assert {"yaw180", "bay+y", "bay-y"} == {r["click_kind"] for r in a} and {"full", "half"} == {r["reverse_kind"] for r in a}
    assert "val3_019" in {r["id"] for r in a}
    assert sorted((r["id"], r["reverse_kind"]) for r in b) == [(i, k) for i in ("val3_032", "val3_052", "val3_072")
                                                               for k in ("full", "half")]
