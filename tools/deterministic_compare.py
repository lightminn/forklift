"""Strict comparison of two runs of one deterministic configuration (docs/plans/2026-10-25-deterministic-sim-mode.md
D8). ``tools/mission_ab_compare.py`` stays as it is; this one is the deterministic mode's verdict.

Every file under the two run folders is compared:
- arrays (``.npz``/``.npy``) bit for bit -- dtype, shape and bytes of every member (the sensor and proposal frames
  included, so a one-bit change in a frame fails even when every summary agrees);
- JSON by value after dropping an explicit list of wall-clock, identity and diagnostic fields (``WALL_KEY``,
  ``IDENTITY_KEYS``, ``deterministic.diagnostics``) and normalising the run's own path and snapshot names; the fields
  the M-a0 comparison reports as nondeterministic (SLAM map version, memory revision, camera readiness) are compared
  like any other and fail here;
- JSONL line by line the same way; the live log keeps its events and their ticks but drops the console traffic
  (``where`` cycle/gate) and the wall stamps;
- spool directories (``spool/<stream>/*.jsonl``) as one logical stream each, so where the writer cut its files does
  not matter;
- other files byte for byte; images, videos, scene exports and text logs are skipped.

The comparison fails when any run has a ``deterministic_broken`` reason (``result.json`` ``deterministic.broken``),
even the same reason on both runs. Output: per file or stream hashes for each run, the first difference of each
differing one, the broken reasons.

    python tools/deterministic_compare.py RUN_A RUN_B [--json OUT]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

import numpy as np

SKIP_SUFFIXES = {".usd", ".usda", ".usdc", ".png", ".jpg", ".mp4", ".log", ".txt"}
# Plan D1b: RTX RGB differs between processes even on the same timeline (probes v7-v9) and a deterministic run keeps it
# off the decision path (the oracle masks replace the learned proposer), so array members holding RGB are left out.
RGB_MEMBERS = {"rgb", "rgba"}
WALL_KEY = re.compile(r"(^|_)(wall|mono)(_s)?$|_wall_s$|^wall_s_|^wall_time_s$|^elapsed_s$|^model_ms$|_mono$")
# wall stamps only as numbers: "decided" is also the recognition's candidate record and the cusp hold's motion state
STAMP_KEYS = {"submitted", "decided", "started"}
IDENTITY_KEYS = {"pid", "session_epoch", "token", "request_token", "port",
                 # the Kit render frame number (the frame sync takes a varying number of renders, dv3 V1) and the
                 # scene-replan tracker label id(tracker) % 10**6
                 "sensor_frame_id", "tracker_serial"}
# key / cancelled_by = [phase, id(tracker), leg]; the watch's tracker = id(tracker)
HOLD_LISTS = {"cusp_heading_holds", "cusp_heading_hold", "cusp_heading_watch"}
SNAPSHOT = re.compile(r"/snapshots/[^/]+/")
CONSOLE_TRAFFIC = {"cycle", "gate"}


def _norm_str(value: str, roots: tuple) -> str:
    for root in roots:
        value = value.replace(str(root), "<RUN>")
    return SNAPSHOT.sub("/snapshots/<SNAP>/", value)


def _tracker_ordinals(state: dict) -> dict:
    """A container holding the cusp heading records with id(tracker) replaced by its first-seen ordinal -- one table
    for the closed holds, the open hold and the watch, so "the same tracker" still compares across them (as
    ``run_transport._identity_free`` does for boundary records)."""
    ordinals: dict = {}

    def ordinal(tid):
        return f"<tracker {ordinals.setdefault(tid, len(ordinals))}>"

    def hold_(hold):
        if not isinstance(hold, dict):
            return hold
        hold = dict(hold)
        for field in ("key", "cancelled_by"):
            v = hold.get(field)
            if isinstance(v, list) and len(v) == 3:
                hold[field] = [v[0], ordinal(v[1]), v[2]]
        return hold

    state = dict(state)
    if isinstance(state.get("cusp_heading_holds"), list):
        state["cusp_heading_holds"] = [hold_(h) for h in state["cusp_heading_holds"]]
    if isinstance(state.get("cusp_heading_hold"), dict):
        state["cusp_heading_hold"] = hold_(state["cusp_heading_hold"])
    watch = state.get("cusp_heading_watch")
    if isinstance(watch, dict) and "tracker" in watch:
        state["cusp_heading_watch"] = {**watch, "tracker": ordinal(watch["tracker"])}
    return state


def normalise(value, roots: tuple, path: str = ""):
    """The comparable form of a JSON value: excluded keys dropped, run paths and snapshot names replaced."""
    if isinstance(value, dict):
        if HOLD_LISTS & set(value):
            value = _tracker_ordinals(value)
        out = {}
        for key, sub in value.items():
            k = str(key)
            here = f"{path}.{k}" if path else k
            stamp = k in STAMP_KEYS and isinstance(sub, (int, float)) and not isinstance(sub, bool)
            if WALL_KEY.search(k) or k in IDENTITY_KEYS or stamp or here == "deterministic.diagnostics":
                continue
            out[k] = normalise(sub, roots, here)
        return out
    if isinstance(value, list):
        return [normalise(v, roots, path) for v in value]
    if isinstance(value, str):
        return _norm_str(value, roots)
    return value


def _live_log_line(entry):
    if not isinstance(entry, dict) or entry.get("where") in CONSOLE_TRAFFIC:
        return None
    entry = dict(entry)
    if isinstance(entry.get("t"), (int, float)):
        entry.pop("t")  # an event's wall stamp (a console entry's t is its message type, dropped above)
    return entry


def _canon(value) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def _arrays(path: Path) -> dict:
    if path.suffix == ".npy":
        return {"": np.load(path, allow_pickle=False)}
    with np.load(path, allow_pickle=False) as z:
        return {k: z[k] for k in z.files if k.lower() not in RGB_MEMBERS}


def _array_key(a: np.ndarray) -> bytes:
    return f"{a.dtype.str}{a.shape}".encode() + np.ascontiguousarray(a).tobytes()


def _items(run: Path) -> dict:
    """{relative name: ("json", lines) | ("arrays", {member: array}) | ("bytes", data)}"""
    roots = (run, run.resolve())
    if run.name == "run":  # a runner slot <slot>/run: the sock/ and bridge/ the arguments name sit beside it
        roots += (run.parent, run.resolve().parent)
    items: dict = {}
    spools: dict = {}
    for path in sorted(p for p in run.rglob("*") if p.is_file()):
        rel = path.relative_to(run).as_posix()
        if path.suffix in SKIP_SUFFIXES:
            continue
        parts = rel.split("/")
        if path.name == "manifest.json" and len(parts) >= 2 and parts[-2] == "spool":
            continue  # chunk counts follow the wall-clock cut interval: checked by _invalid, not compared
        if "spool" in parts and path.suffix == ".jsonl" and parts.index("spool") == len(parts) - 3:
            stream = "/".join(parts[:-1])
            spools.setdefault(stream, []).append(path)
            continue
        if path.suffix in (".npz", ".npy"):
            items[rel] = ("arrays", _arrays(path))
        elif path.suffix == ".json":
            items[rel] = ("json", [normalise(json.loads(path.read_text()), roots)])
        elif path.suffix == ".jsonl":
            lines = []
            for line in path.read_text().splitlines():
                if not line.strip():
                    continue
                entry = json.loads(line)
                if path.name == "live_log.jsonl":
                    entry = _live_log_line(entry)
                    if entry is None:
                        continue
                lines.append(normalise(entry, roots))
            items[rel] = ("json", lines)
        else:
            items[rel] = ("bytes", path.read_bytes())
    for stream, files in spools.items():
        lines = []
        for path in sorted(files):
            lines += [normalise(json.loads(x), roots) for x in path.read_text().splitlines() if x.strip()]
        items[stream] = ("json", lines)
    return items


def _hash(item) -> str:
    kind, value = item
    h = hashlib.sha256(kind.encode())
    if kind == "arrays":
        for k in sorted(value):
            h.update(k.encode())
            h.update(_array_key(value[k]))
    elif kind == "json":
        for line in value:
            h.update(_canon(line).encode())
            h.update(b"\n")
    else:
        h.update(value)
    return h.hexdigest()[:16]


def _first_json_difference(a, b, path="") -> str | None:
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b)):
            if k not in a or k not in b:
                return f"{path}.{k} only in {'A' if k in a else 'B'}"
            d = _first_json_difference(a[k], b[k], f"{path}.{k}")
            if d:
                return d
        return None
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            for i, (x, y) in enumerate(zip(a, b)):
                d = _first_json_difference(x, y, f"{path}[{i}]")
                if d:
                    return d
            return f"{path} length {len(a)} vs {len(b)}"
        for i, (x, y) in enumerate(zip(a, b)):
            d = _first_json_difference(x, y, f"{path}[{i}]")
            if d:
                return d
        return None
    return None if a == b else f"{path}: {str(a)[:80]} vs {str(b)[:80]}"


def _difference(name: str, a, b) -> str:
    if a[0] != b[0]:
        return f"{name}: kind {a[0]} vs {b[0]}"
    if a[0] == "arrays":
        for k in sorted(set(a[1]) | set(b[1])):
            if k not in a[1] or k not in b[1]:
                return f"{name}: member {k} only in {'A' if k in a[1] else 'B'}"
            if _array_key(a[1][k]) != _array_key(b[1][k]):
                return f"{name}: member {k or '<array>'} differs"
        return f"{name}: differs"
    if a[0] == "json":
        d = _first_json_difference(a[1], b[1])
        return f"{name}: {d}"
    return f"{name}: bytes differ"


def _broken(items: dict) -> list:
    out = []
    for name, (kind, value) in items.items():
        if kind == "json" and name.endswith("result.json") and value and isinstance(value[0], dict):
            out += list((value[0].get("deterministic") or {}).get("broken") or [])
    return out


def _record_errors(value, path=""):
    if isinstance(value, dict):
        for k, sub in value.items():
            if k == "record_errors" and sub:
                yield f"{path}.{k}"
            else:
                yield from _record_errors(sub, f"{path}.{k}")
    elif isinstance(value, list):
        for i, sub in enumerate(value):
            yield from _record_errors(sub, f"{path}[{i}]")


def _invalid(run: Path) -> list:
    """Why ``run`` is not a whole deterministic record: a missing folder or result, a result without the deterministic
    block, a spool without a complete manifest, recorded write errors. An ordinary mission failure is valid."""
    if not run.is_dir():
        return [f"{run}: no such folder"]
    results = sorted(run.rglob("result.json"))
    problems = [] if results else [f"{run}: no result.json"]
    for path in results:
        result = json.loads(path.read_text())
        if not (isinstance(result, dict) and (result.get("deterministic") or {}).get("enabled") is True):
            problems.append(f"{path.relative_to(run)}: no deterministic block (not a deterministic run)")
        problems += [f"{path.relative_to(run)}: {where} not empty" for where in _record_errors(result)]
    for spool in sorted(p for p in run.rglob("spool") if p.is_dir()):
        manifest = spool / "manifest.json"
        if not manifest.exists() or json.loads(manifest.read_text()).get("complete") is not True:
            problems.append(f"{spool.relative_to(run)}: the spooled record is incomplete (manifest)")
    return problems


def compare(run_a: Path, run_b: Path) -> dict:
    invalid = {"A": _invalid(Path(run_a)), "B": _invalid(Path(run_b))}
    if invalid["A"] or invalid["B"]:
        return {"equal": False, "invalid": invalid, "differences": [], "broken": {"A": [], "B": []},
                "hashes": {"A": {}, "B": {}}}
    ia, ib = _items(Path(run_a)), _items(Path(run_b))
    differences = []
    for name in sorted(set(ia) | set(ib)):
        if name not in ia or name not in ib:
            differences.append(f"{name}: only in {'A' if name in ia else 'B'}")
        elif _hash(ia[name]) != _hash(ib[name]):
            differences.append(_difference(name, ia[name], ib[name]))
    broken = {"A": _broken(ia), "B": _broken(ib)}
    return {"equal": not differences and not broken["A"] and not broken["B"], "invalid": invalid,
            "differences": differences,
            "broken": broken, "hashes": {"A": {k: _hash(v) for k, v in ia.items()},
                                         "B": {k: _hash(v) for k, v in ib.items()}}}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run_a", type=Path)
    ap.add_argument("run_b", type=Path)
    ap.add_argument("--json", type=Path, default=None)
    args = ap.parse_args(argv)
    out = compare(args.run_a, args.run_b)
    if args.json is not None:
        args.json.write_text(json.dumps(out, indent=1) + "\n")
    print("EQUAL" if out["equal"] else "DIFFERENT", f"{len(out['differences'])} differences", "broken", out["broken"])
    for side in ("A", "B"):
        for problem in out["invalid"][side]:
            print(f"  invalid {side}:", problem)
    for d in out["differences"][:40]:
        print(" ", d)
    return 0 if out["equal"] else 1


if __name__ == "__main__":
    sys.exit(main())
