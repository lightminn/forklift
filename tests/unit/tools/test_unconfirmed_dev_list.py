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
