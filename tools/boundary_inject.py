"""Inject a fault into a copy of a format-2 boundary record and check the replay refuses it (M-a1 replay plan R4).

A record the code no longer matches is the same as code that no longer matches its record: dropping a recorded
call is the code making an extra call, duplicating one is a call left unmade, and so on. Each mutation is made on
a copy (the original is never written) and the copy is replayed with tools/boundary_replay.py; the injection
passes when the replay fails, and for the exact mutations when it fails at the mutated call.

    python tools/boundary_inject.py RUN_DIR --mutation drop [--tick N] [--json OUT]
"""

from __future__ import annotations

import argparse
import copy
import importlib.util
import json
import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# name -> (what it stands for, whether the replay must fail at the mutated tick)
MUTATIONS = {
    "drop": ("the code makes a call the record does not have", True),
    "extra": ("the code leaves a recorded call unmade", True),
    "swap": ("the code makes two calls in the other order", True),
    "truncate": ("the run ended (failed) earlier in the same tick", True),
    "request": ("the code sends a different request", True),
    "response": ("a service answered differently (the code must then diverge somewhere)", False),
    "sink": ("the code writes a different payload", True),
    "callback": ("a callback runs with different settings", True),
    "ending": ("the run ended another way", False),
    "final_state": ("the run's end state differs", False),
}


def _load_harness():
    spec = importlib.util.spec_from_file_location("boundary_replay_inject", ROOT / "tools/boundary_replay.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _chunks(boundary: Path):
    return sorted(boundary.glob("boundary_*.jsonl"))


def _pick(ticks: list[dict], mutation: str, after: int):
    """(tick index in the chunk, call index) of the first place at or after tick ``after`` that suits the mutation."""
    for i, tick in enumerate(ticks):
        if tick["tick"] < after:
            continue
        calls = tick["calls"]
        for k, call in enumerate(calls):
            pm = (call["port"], call["method"])
            if mutation in ("drop", "extra", "truncate") and pm[0] not in ("Trace", "Sink"):
                return i, k
            if mutation == "swap" and k + 1 < len(calls) and pm != (calls[k + 1]["port"], calls[k + 1]["method"]):
                return i, k
            if mutation == "request" and pm[0] == "Planner":
                return i, k
            if mutation == "response" and pm == ("Clock", "monotonic") and isinstance(call.get("response"), float):
                return i, k
            if mutation == "sink" and pm[0] == "Sink":
                return i, k
            if mutation == "callback" and pm == ("Trace", "callback_spec"):
                return i, k
    return None


def _bump_number(node):
    """The first float in a request node, changed by 1e-6 (returns the new node, or None when it has no float)."""
    if isinstance(node, float):
        return node + 1e-6
    if isinstance(node, dict):
        for key in sorted(node):
            if key.startswith("$") and key not in ("$map", "$list", "$tuple"):
                continue
            new = _bump_number(node[key])
            if new is not None:
                out = dict(node)
                out[key] = new
                return out
    if isinstance(node, list):
        for i, item in enumerate(node):
            new = _bump_number(item)
            if new is not None:
                out = list(node)
                out[i] = new
                return out
    return None


def make_copy(run_dir: Path, work: Path, mutation: str, after: int) -> dict:
    """The mutated copy at ``work`` / run; returns where the mutation is."""
    src = Path(run_dir) / "boundary"
    dst = work / "run" / "boundary"
    dst.mkdir(parents=True)
    for item in src.iterdir():  # arrays and setup files are linked, the call chunks copied
        if item.suffix == ".jsonl" or item.name in ("boundary_meta.json", "final_state.json"):
            shutil.copy2(item, dst / item.name)
        else:
            (dst / item.name).symlink_to(item.resolve())
    where = {"mutation": mutation, "stands_for": MUTATIONS[mutation][0]}
    if mutation == "ending":
        meta = json.loads((dst / "boundary_meta.json").read_text())
        ending = meta.get("ending") or {}
        meta["ending"] = ({"kind": "exception", "type": "builtins:RuntimeError", "message": "injected"}
                          if ending.get("kind") == "returned" else {"kind": "returned"})
        (dst / "boundary_meta.json").write_text(json.dumps(meta, indent=1) + "\n")
        return where
    if mutation == "final_state":
        path = dst / "final_state.json"
        if not path.exists():
            raise SystemExit("the record has no final state to change")
        node = json.loads(path.read_text())
        text = json.dumps(node, sort_keys=True)
        digest = next(part for part in text.split('"') if len(part) == 64 and all(c in "0123456789abcdef" for c in part))
        path.write_text(text.replace(digest, ("0" if digest[0] != "0" else "1") + digest[1:], 1) + "\n")
        where["changed_digest"] = digest[:12]
        return where
    for chunk in _chunks(dst):
        ticks = [json.loads(line) for line in chunk.read_text().splitlines() if line.strip()]
        found = _pick(ticks, mutation, after)
        if found is None:
            continue
        i, k = found
        tick = ticks[i]
        calls = tick["calls"]
        where.update(tick=tick["tick"], call=k, port=calls[k]["port"], method=calls[k]["method"])
        if mutation == "drop":
            del calls[k]
        elif mutation == "extra":
            calls.insert(k + 1, copy.deepcopy(calls[k]))
        elif mutation == "swap":
            calls[k], calls[k + 1] = calls[k + 1], calls[k]
        elif mutation == "truncate":
            del calls[k:]
        elif mutation in ("request", "sink", "callback"):
            node = calls[k]["request"]
            if mutation == "sink":
                text = json.dumps(node)
                old = next(p for p in text.split('"') if len(p) == 64)
                node = json.loads(text.replace(old, ("0" if old[0] != "0" else "1") + old[1:]))
            else:
                node = _bump_number(node)
                if node is None:
                    raise SystemExit(f"tick {tick['tick']} call {k}: no number to change in the request")
            calls[k]["request"] = node
        elif mutation == "response":
            calls[k]["response"] = calls[k]["response"] + 1e-3
        for j, call in enumerate(calls):
            call["seq"] = j
        chunk.write_text("".join(json.dumps(t) + "\n" for t in ticks))
        return where
    raise SystemExit(f"no place for {mutation} at or after tick {after}")


def inject(run_dir: Path, mutation: str, after: int = 0) -> dict:
    harness = _load_harness()
    with tempfile.TemporaryDirectory(prefix="boundary_inject_") as tmp:
        where = make_copy(Path(run_dir), Path(tmp), mutation, after)
        outcome = harness.replay_run(Path(tmp) / "run", Path(tmp) / "out")
    failed = outcome.get("failure") is not None or outcome.get("finish") != "ok"
    at_tick = outcome.get("tick_open")
    exact = MUTATIONS[mutation][1]
    located = (not exact) or (at_tick == where.get("tick")) or (
        mutation == "extra" and str(where.get("tick")) in str(outcome.get("failure")))
    return {**where, "replay_failed": failed, "failure": outcome.get("failure"), "finish": outcome.get("finish"),
            "failed_in_tick": at_tick, "located": located, "passed": failed and located}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--mutation", choices=sorted(MUTATIONS), required=True)
    parser.add_argument("--tick", type=int, default=0, help="mutate the first suitable call at or after this tick")
    parser.add_argument("--json", type=Path)
    args = parser.parse_args(argv)
    out = inject(args.run_dir, args.mutation, args.tick)
    text = json.dumps(out, indent=1)
    if args.json:
        args.json.write_text(text + "\n")
    print(text)
    return 0 if out["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
