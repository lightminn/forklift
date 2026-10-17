"""What a boundary record holds, for the M-a1 replay (plan M-a1 slice 7.0).

A replay hands every recorded call its recorded answer. That works only for what the record carries by
value; this audit lists, per ``port.method``, how often it was called and what in it a replay cannot
restore or must apply besides the answer:

- ``opaque``: values recorded by type name only (``{"$object": T}`` with nothing else) -- in the request,
  the response, an exception's attributes or the changes; a replay cannot hand these back by value.
- ``dataclasses``: dataclass types in responses (``{"$dataclass": T, ...}``): restorable by value, but a
  replay must rebuild the object.
- ``exceptions``: exception types raised through the call (a replay re-raises them, attributes included).
- ``request_after``: calls that changed their own arguments (a replay applies the change to the caller's).
- ``watched``: calls that changed the core's watched dictionaries (keys per dictionary).

It reads the JSON lines only (array payloads are referenced by hash and not loaded), so it runs on a
large record in little memory.

    python tools/boundary_audit.py RECORD_DIR [--json OUT]
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path


def _walk(value, path=""):
    """(path, node) for every dict node in value."""
    if isinstance(value, dict):
        yield path, value
        for k, v in value.items():
            yield from _walk(v, f"{path}.{k}" if path else k)
    elif isinstance(value, list):
        for i, v in enumerate(value):
            yield from _walk(v, f"{path}[{i}]")


def _opaque_types(value) -> list[str]:
    return [node["$object"] for _, node in _walk(value) if "$object" in node and len(node) == 1]


def _dataclass_types(value) -> list[str]:
    return [node["$dataclass"] for _, node in _walk(value) if "$dataclass" in node]


def audit(directory: Path) -> dict:
    directory = Path(directory)
    meta_path = directory / "boundary_meta.json"
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else None
    calls = Counter()
    opaque = defaultdict(Counter)
    dataclasses_ = defaultdict(Counter)
    exceptions = defaultdict(Counter)
    request_after = Counter()
    watched = defaultdict(Counter)
    ticks, first_tick, last_tick, setup_ticks = 0, None, None, 0
    output_keys = Counter()
    for path in sorted(directory.glob("boundary_*.jsonl")):
        with path.open() as fh:
            for line in fh:
                if not line.strip():
                    continue
                tick = json.loads(line)
                ticks += 1
                first_tick = tick["tick"] if first_tick is None else first_tick
                last_tick = tick["tick"]
                setup_ticks += tick["tick"] < 0
                output_keys.update(list(tick["outputs"]) if isinstance(tick.get("outputs"), dict) else [])
                for call in tick["calls"]:
                    key = f"{call['port']}.{call['method']}"
                    calls[key] += 1
                    for where in ("request", "response"):
                        for t in _opaque_types(call.get(where)):
                            opaque[key][f"{where}:{t}"] += 1
                    for t in _dataclass_types(call.get("response")):
                        dataclasses_[key][t] += 1
                    if "exception" in call:
                        exceptions[key][call["exception"]["type"]] += 1
                        for t in _opaque_types(call["exception"].get("attributes")):
                            opaque[key][f"exception:{t}"] += 1
                    changes = call.get("changes") or {}
                    if "request_after" in changes:
                        request_after[key] += 1
                        for t in _opaque_types(changes["request_after"]):
                            opaque[key][f"request_after:{t}"] += 1
                    for name, diff in (changes.get("watched") or {}).items():
                        for k in diff:
                            watched[key][f"{name}.{k}"] += 1
                        for t in _opaque_types(diff):
                            opaque[key][f"watched:{t}"] += 1
    return {
        "directory": str(directory),
        "meta": meta,
        "ticks": ticks,
        "first_tick": first_tick,
        "last_tick": last_tick,
        "setup_ticks": setup_ticks,
        "output_keys": dict(output_keys),
        "calls": dict(calls.most_common()),
        "opaque": {k: dict(v) for k, v in sorted(opaque.items())},
        "dataclasses": {k: dict(v) for k, v in sorted(dataclasses_.items())},
        "exceptions": {k: dict(v) for k, v in sorted(exceptions.items())},
        "request_after": dict(request_after),
        "watched": {k: dict(v) for k, v in sorted(watched.items())},
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("record", type=Path)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args(argv)
    result = audit(args.record)
    text = json.dumps(result, indent=1, sort_keys=False)
    if args.json is not None:
        args.json.write_text(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
