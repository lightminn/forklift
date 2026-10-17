"""Ordered prefix comparison of two boundary records (M-a1 per-slice evidence; plan M-a1 row).

Two records of the same configuration -- before and after a slice moved -- are compared in execution
order: each tick's head, then its calls in sequence (port, method and request first, then the response,
the exception and the changes the call returned), then its outputs and branches. Wall-clock fields
(``*_wall_s``, ``wall_time_s``, ``elapsed_s``, ``wall_s_*``) are left out everywhere, and the value a
wall-clock read returns (``Clock.monotonic``/``time``) is not compared -- its order and request are; nothing
else is left out.

- ``equal``: every tick matches.
- ``prefix_equal``: they match until a known external difference -- the response of a call whose result
  depends on the renderer or on the SLAM map's asynchronous arrival (a capture, a grid preparation) with
  the same request -- after which the runs are not comparable. The cut tick's branches are not counted.
- ``differ``: a request, an order, an unknown response, an output or the record length differs first.
- ``incomplete``: a record is unfinished, or stopped at its size cap.

``branches_covered`` lists the branch ids of the fully compared ticks. A prefix match is a regression
check for moving on to the next slice, not the slice's equivalence (that is the final replay).

    python tools/boundary_diff.py RECORD_A RECORD_B [--json OUT]
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

WALL_KEY = re.compile(r"(_wall_s$|^wall_time_s$|^elapsed_s$|^wall_s_)")
# Calls whose response may legitimately differ between runs of one configuration (same request):
# render readiness of a capture, the SLAM map version a grid preparation reads.
KNOWN_EXTERNAL = {("Capture", "capture"), ("Planner", "grid_kwargs")}
# Wall-clock reads (the Clock proxy of the runner's `time` module): the value differs on every run, so only
# the call (its order and request) is compared. Simulation time (Clock.current_time) is compared in full.
WALL_READS = {("Clock", "monotonic"), ("Clock", "time"), ("Clock", "perf_counter")}


def _scrub(value):
    if isinstance(value, dict):
        return {k: _scrub(v) for k, v in value.items() if not WALL_KEY.search(str(k))}
    if isinstance(value, list):
        return [_scrub(v) for v in value]
    return value


def _same(a, b) -> bool:
    return json.dumps(_scrub(a), sort_keys=True) == json.dumps(_scrub(b), sort_keys=True)


def _ticks(directory: Path):
    for path in sorted(Path(directory).glob("boundary_*.jsonl")):
        with open(path) as f:
            for line in f:
                if line.strip():
                    yield json.loads(line)


def _complete(directory: Path) -> bool:
    meta = Path(directory) / "boundary_meta.json"
    if not meta.exists():
        return False
    return json.loads(meta.read_text()).get("stopped_at_tick") is None


def compare(record_a: Path, record_b: Path) -> dict:
    out = {"record_a": str(record_a), "record_b": str(record_b), "ticks_compared": 0, "branches_covered": []}
    if not (_complete(record_a) and _complete(record_b)):
        out["status"] = "incomplete"
        return out
    covered: set[str] = set()
    a_iter, b_iter = _ticks(record_a), _ticks(record_b)
    while True:
        ta, tb = next(a_iter, None), next(b_iter, None)
        if ta is None and tb is None:
            out["status"] = "equal"
            break
        if ta is None or tb is None:
            out.update(status="differ", first_difference={"tick": (ta or tb)["tick"], "what": "record length"})
            break
        head = ("tick", "tick_t", "work_t", "commands", "stop_kind")
        if not _same({k: ta.get(k) for k in head}, {k: tb.get(k) for k in head}):
            out.update(status="differ", first_difference={"tick": ta["tick"], "what": "tick head"})
            break
        cut = None
        for ca, cb in zip(ta["calls"], tb["calls"]):
            where = {"tick": ta["tick"], "seq": ca.get("seq"), "port": ca.get("port"), "method": ca.get("method")}
            if (ca.get("seq"), ca.get("port"), ca.get("method")) != (cb.get("seq"), cb.get("port"), cb.get("method")):
                cut = ("differ", {**where, "what": "call order"})
                break
            if not _same(ca.get("request"), cb.get("request")):
                cut = ("differ", {**where, "what": "request"})
                break
            for part in ("response", "exception", "changes"):
                if part == "response" and (ca.get("port"), ca.get("method")) in WALL_READS:
                    continue
                if not _same(ca.get(part), cb.get(part)):
                    known = (ca.get("port"), ca.get("method")) in KNOWN_EXTERNAL and part in ("response", "changes")
                    cut = ("prefix_equal" if known else "differ", {**where, "what": part})
                    break
            if cut:
                break
        if cut is None and len(ta["calls"]) != len(tb["calls"]):
            cut = ("differ", {"tick": ta["tick"], "what": f"call count {len(ta['calls'])} vs {len(tb['calls'])}"})
        if cut is None and not _same(ta.get("outputs"), tb.get("outputs")):
            cut = ("differ", {"tick": ta["tick"], "what": "outputs"})
        if cut is None and ta.get("branches") != tb.get("branches"):
            cut = ("differ", {"tick": ta["tick"], "what": "branches"})
        if cut is not None:
            status, where = cut
            out["status"] = status
            out["cutoff" if status == "prefix_equal" else "first_difference"] = where
            break
        out["ticks_compared"] += 1
        covered.update(ta.get("branches") or [])
    out["branches_covered"] = sorted(covered)
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("record_a", type=Path)
    parser.add_argument("record_b", type=Path)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args(argv)
    rep = compare(args.record_a, args.record_b)
    if args.json:
        args.json.write_text(json.dumps(rep, indent=1) + "\n")
    where = rep.get("cutoff") or rep.get("first_difference")
    print(f"{rep['status'].upper()}: {rep['ticks_compared']} ticks compared, {len(rep['branches_covered'])} branches"
          + (f"; at {where}" if where else ""))
    return 0 if rep["status"] in ("equal", "prefix_equal") else 1


if __name__ == "__main__":
    sys.exit(main())
