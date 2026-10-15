"""Boundary record for the M-a replay evidence (docs/plans/2026-10-15-m-a05-boundary.md, section 5).

One record per control tick: its head (tick, loop time, work time, commands, stop kind), every port
call in order (port, method, request, response or exception, changes returned to the core) and its
tail (core outputs, branch names). Values are encoded when the call is recorded, so a service that
changes its arrays afterwards does not change the record. Arrays are stored once by content
(sha256 of dtype, shape and bytes) in ``arrays_NNNNN.npz`` beside ``boundary_NNNNN.jsonl``; ticks are
flushed every ``chunk_ticks`` so the record does not accumulate in memory. Past ``max_bytes`` the
recorder stops writing (the run goes on) and says so in ``boundary_meta.json``.
"""

from __future__ import annotations

import contextlib
import dataclasses
import functools
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterator

import numpy as np

__all__ = ["BoundaryRecorder", "DictWatch", "RecordingGate", "RecordingProxy", "read_record", "recorded_function"]


class BoundaryRecorder:
    def __init__(self, directory: Path, *, chunk_ticks: int = 1200, max_bytes: int = 2_000_000_000) -> None:
        if chunk_ticks < 1 or max_bytes < 1:
            raise ValueError("chunk_ticks and max_bytes must be positive")
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.chunk_ticks = int(chunk_ticks)
        self.max_bytes = int(max_bytes)
        self._chunk = 0
        self._lines: list[str] = []
        self._chunk_arrays: dict[str, np.ndarray] = {}
        self._written: set[str] = set()
        self._tick: dict | None = None
        self._pending_arrays: dict[str, np.ndarray] = {}
        self._bytes = 0
        self._ticks = 0
        self.stopped_at_tick: int | None = None
        self.stopped_reason: str | None = None
        # Objects without a value encoding (trackers, planner internals) are recorded by type name
        # only and counted: such an entry cannot be replayed by value.
        self.opaque_ok = True
        self.opaque_values = 0
        # Classes recorded by value through their instance attributes (e.g. Isaac's ArticulationAction).
        self.value_types: set[str] = set()
        self._closed = False

    @property
    def in_tick(self) -> bool:
        return self._tick is not None

    # -- recording ---------------------------------------------------------------------------------
    def tick_begin(self, tick: int, *, tick_t: float, work_t: float, commands: list, stop_kind: str) -> None:
        if self._tick is not None:
            raise RuntimeError("tick_begin before tick_end")
        self._pending_arrays = {}
        self._tick = {"tick": int(tick), "tick_t": self._encode(tick_t), "work_t": self._encode(work_t),
                      "commands": self._encode(commands), "stop_kind": stop_kind, "calls": []}

    def encode(self, value: Any) -> Any:
        """``value`` encoded now (arrays copied): for recording a request before the call runs."""
        return self._encode(value)

    def call(self, port: str, method: str, *, request: Any, response: Any = None, exception: BaseException | None = None,
             changes: Any = None, encoded_request: Any = None) -> None:
        """``encoded_request`` (from :meth:`encode` before the call) replaces ``request``."""
        if self._tick is None:
            raise RuntimeError("boundary call recorded outside a tick")
        entry = {"seq": len(self._tick["calls"]), "port": port, "method": method,
                 "request": encoded_request if encoded_request is not None else self._encode(request)}
        if exception is not None:
            entry["exception"] = {"type": type(exception).__name__, "message": str(exception)}
            attributes = {k: v for k, v in vars(exception).items() if not k.startswith("__")} if hasattr(exception, "__dict__") else {}
            if attributes:  # what a caller reads from the exception (e.g. a capture failure's diagnostics)
                entry["exception"]["attributes"] = self._encode(attributes)
        else:
            entry["response"] = self._encode(response)
        if changes is not None:
            entry["changes"] = self._encode(changes)
        self._tick["calls"].append(entry)

    def tick_end(self, *, outputs: Any, branches: list[str]) -> None:
        if self._tick is None:
            raise RuntimeError("tick_end without tick_begin")
        tick, self._tick = self._tick, None
        tick["outputs"] = self._encode(outputs)
        tick["branches"] = list(branches)
        if self.stopped_at_tick is not None:
            return
        line = json.dumps(tick, separators=(",", ":"), allow_nan=True)
        size = len(line) + 1 + sum(a.nbytes for a in self._pending_arrays.values())
        if self._bytes + size > self.max_bytes:
            self.stopped_at_tick, self.stopped_reason = tick["tick"], "max_bytes"
            return
        self._bytes += size
        self._lines.append(line)
        self._chunk_arrays.update(self._pending_arrays)
        self._written.update(self._pending_arrays)
        self._ticks += 1
        if len(self._lines) >= self.chunk_ticks:
            self._flush()

    def close(self) -> None:
        if self._closed:
            return
        if self._tick is not None:
            raise RuntimeError("close inside a tick")
        self._closed = True
        self._flush()
        (self.directory / "boundary_meta.json").write_text(json.dumps({
            "format": "forklift_boundary_record/1", "ticks": self._ticks, "chunks": self._chunk,
            "bytes": self._bytes, "stopped_at_tick": self.stopped_at_tick, "stopped_reason": self.stopped_reason,
            "opaque_values": self.opaque_values,
        }, indent=1) + "\n")

    # -- internals ---------------------------------------------------------------------------------
    def _flush(self) -> None:
        if not self._lines:
            return
        name = f"{self._chunk:05d}"
        (self.directory / f"boundary_{name}.jsonl").write_text("\n".join(self._lines) + "\n")
        if self._chunk_arrays:
            np.savez_compressed(self.directory / f"arrays_{name}.npz", **self._chunk_arrays)
        self._lines, self._chunk_arrays = [], {}
        self._chunk += 1

    def _array_ref(self, value: np.ndarray) -> dict:
        array = np.ascontiguousarray(value).copy()
        digest = hashlib.sha256()
        digest.update(str(array.dtype).encode())
        digest.update(repr(array.shape).encode())
        digest.update(array.tobytes())
        key = digest.hexdigest()
        if key not in self._written and key not in self._pending_arrays:
            self._pending_arrays[key] = array
        return {"$array": key, "dtype": str(array.dtype), "shape": list(array.shape)}

    def _encode(self, value: Any) -> Any:
        if isinstance(value, np.ndarray):
            return self._array_ref(value)
        if isinstance(value, (np.floating,)):
            return float(value)
        if isinstance(value, (np.integer,)):
            return int(value)
        if isinstance(value, (np.bool_,)):
            return bool(value)
        if isinstance(value, float) and not math.isfinite(value):
            return {"$float": repr(value)}
        if dataclasses.is_dataclass(value) and not isinstance(value, type):
            return {"$dataclass": type(value).__name__,
                    **{f.name: self._encode(getattr(value, f.name)) for f in dataclasses.fields(value)}}
        if isinstance(value, dict):
            return {str(k): self._encode(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [self._encode(v) for v in value]
        if value is None or isinstance(value, (bool, int, float, str)):
            return value
        if type(value).__name__ in self.value_types and hasattr(value, "__dict__"):
            return {"$object": type(value).__name__,
                    **{k: self._encode(v) for k, v in vars(value).items() if not k.startswith("_")}}
        if self.opaque_ok:
            self.opaque_values += 1
            return {"$object": type(value).__name__}
        raise TypeError(f"cannot record a {type(value).__name__}")


def read_record(directory: Path) -> Iterator[dict]:
    """The recorded ticks in order, arrays restored (non-finite floats as floats)."""
    directory = Path(directory)
    arrays: dict[str, np.ndarray] = {}
    for path in sorted(directory.glob("arrays_*.npz")):
        with np.load(path, allow_pickle=False) as z:
            arrays.update({k: z[k] for k in z.files})

    def decode(value: Any) -> Any:
        if isinstance(value, dict):
            if "$array" in value:
                return arrays[value["$array"]]
            if "$float" in value:
                return float(value["$float"])
            return {k: decode(v) for k, v in value.items()}
        if isinstance(value, list):
            return [decode(v) for v in value]
        return value

    for path in sorted(directory.glob("boundary_*.jsonl")):
        for line in path.read_text().splitlines():
            if line:
                yield decode(json.loads(line))


def _hash_only(value: Any) -> Any:
    """``value`` with every array replaced by its content hash, dtype and shape (not stored)."""
    if isinstance(value, np.ndarray):
        array = np.ascontiguousarray(value)
        digest = hashlib.sha256(str(array.dtype).encode() + repr(array.shape).encode() + array.tobytes())
        return {"$sha256": digest.hexdigest(), "dtype": str(array.dtype), "shape": list(array.shape)}
    if dataclasses.is_dataclass(value) and not isinstance(value, type):  # e.g. a SceneInput holding frames
        return {"$dataclass": type(value).__name__,
                **{f.name: _hash_only(getattr(value, f.name)) for f in dataclasses.fields(value)}}
    if isinstance(value, dict):
        return {k: _hash_only(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_hash_only(v) for v in value]
    return value


class RecordingGate:
    """Whether a boundary call is recorded now: a tick is open and no recorded call (or suspended
    service work, such as physics stepping) is running -- the calls a recorded call makes inside
    itself belong to that call, not to the core."""

    def __init__(self, recorder: BoundaryRecorder) -> None:
        self.recorder = recorder
        self.depth = 0

    @property
    def active(self) -> bool:
        return self.depth == 0 and self.recorder.in_tick

    @contextlib.contextmanager
    def suspended(self):
        self.depth += 1
        try:
            yield
        finally:
            self.depth -= 1

    def run(self, port: str, method: str, fn, args: tuple, kwargs: dict, *, hash_only: bool = False,
            watch: "DictWatch | None" = None):
        """Run ``fn`` and record it: the request as it was before the call, the response or the exception,
        the arguments again when the call changed them (``request_after``), and what the call changed in
        the watched core dictionaries (``watched``)."""
        if not self.active:
            return fn(*args, **kwargs)
        raw = {"args": list(args), "kwargs": dict(kwargs)}
        before = self.recorder.encode(_hash_only(raw) if hash_only else raw)
        marks = watch.mark() if watch is not None else None
        self.depth += 1
        try:
            result = fn(*args, **kwargs)
        except BaseException as exc:
            self.depth -= 1
            self.recorder.call(port, method, request=None, encoded_request=before, exception=exc,
                               changes=self._changes(raw, before, hash_only, watch, marks))
            raise
        self.depth -= 1
        self.recorder.call(port, method, request=None, encoded_request=before, response=result,
                           changes=self._changes(raw, before, hash_only, watch, marks))
        return result

    def _changes(self, raw, before, hash_only, watch, marks):
        out = {}
        after = self.recorder.encode(_hash_only(raw) if hash_only else raw)
        if json.dumps(after, sort_keys=True) != json.dumps(before, sort_keys=True):
            out["request_after"] = after
        if watch is not None:
            changed = watch.changes(marks)
            if changed:
                out["watched"] = changed
        return out or None


def recorded_function(fn, port: str, method: str, gate: RecordingGate, *, hash_only: bool = False,
                      watch: "DictWatch | None" = None):
    """``fn`` with each call recorded through ``gate`` (the result and exceptions pass through);
    ``watch`` records what the call changed in core dictionaries."""

    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        return gate.run(port, method, fn, args, kwargs, hash_only=hash_only, watch=watch)

    return wrapper


class DictWatch:
    """What a service call changes in the core's dictionaries (e.g. the physics step updating
    ``slam["stop_now"]``): per watched dictionary, every key whose encoded value changed, and for
    append-only record lists only the items appended. ``skip`` keys are not compared at all."""

    def __init__(self, recorder: BoundaryRecorder, dicts: dict, *, append_only=(), skip=(), keys=None) -> None:
        self.recorder = recorder
        self.dicts = dicts
        self.append_only = set(append_only)
        self.skip = set(skip)
        # Per dictionary, the only keys compared (None: all but ``skip``) -- keeps a per-physics-tick watch cheap.
        self.keys = {name: set(k) for name, k in (keys or {}).items()}

    def _snapshot(self, d: dict, name: str = "") -> dict:
        snap = {}
        only = self.keys.get(name)
        for k, v in list(d.items()):
            if k in self.skip or (only is not None and k not in only):
                continue
            if k in self.append_only and isinstance(v, list):
                snap[k] = ("$len", len(v))
            else:
                snap[k] = json.dumps(self.recorder.encode(v), sort_keys=True)
        return snap

    def mark(self) -> dict:
        return {name: self._snapshot(d, name) for name, d in self.dicts.items() if d is not None}

    def changes(self, marks: dict) -> dict:
        out = {}
        for name, d in self.dicts.items():
            if d is None or name not in marks:
                continue
            before, now = marks[name], self._snapshot(d, name)
            diff = {}
            for k in set(before) | set(now):
                if before.get(k) == now.get(k):
                    continue
                if k not in d:
                    diff[k] = {"$deleted": True}
                elif isinstance(now.get(k), tuple) and now[k][0] == "$len":
                    start = before[k][1] if isinstance(before.get(k), tuple) else 0
                    diff[k] = {"$append": d[k][start:]}
                else:
                    diff[k] = d[k]
            if diff:
                out[name] = diff
        return out


class RecordingProxy:
    """A service object whose method calls, attribute reads and attribute writes are recorded through
    ``gate``; every value returned is the real object's. ``hash_only_requests`` names methods whose
    request arrays are recorded by hash only (large frames). ``child_ports`` maps attribute names that
    hold sub-services (e.g. the obstacle layer's ``permission``) to the port their calls are recorded
    under: reading such an attribute returns a proxy of it instead of recording an opaque object."""

    def __init__(self, target, port: str, gate: RecordingGate, *, hash_only_requests=(), child_ports=None,
                 watches=None) -> None:
        object.__setattr__(self, "_target", target)
        object.__setattr__(self, "_port", port)
        object.__setattr__(self, "_gate", gate)
        object.__setattr__(self, "_hash_only", frozenset(hash_only_requests))
        object.__setattr__(self, "_child_ports", dict(child_ports or {}))
        object.__setattr__(self, "_children", {})
        # Per method, a DictWatch for what the call changes in the core's dictionaries (e.g. a capture that
        # steps physics inside itself, where the nested step's own watch is not recorded).
        object.__setattr__(self, "_watches", dict(watches or {}))

    def __getattr__(self, name: str):
        target, gate = object.__getattribute__(self, "_target"), object.__getattribute__(self, "_gate")
        port = object.__getattribute__(self, "_port")
        value = getattr(target, name)
        child_port = object.__getattribute__(self, "_child_ports").get(name)
        if child_port is not None and value is not None:
            children = object.__getattribute__(self, "_children")
            cached = children.get(name)
            if cached is None or object.__getattribute__(cached, "_target") is not value:
                cached = RecordingProxy(value, child_port, gate)
                children[name] = cached
            return cached
        if callable(value):
            hash_only = name in object.__getattribute__(self, "_hash_only")
            watch = object.__getattribute__(self, "_watches").get(name)

            def method(*args, **kwargs):
                return gate.run(port, name, value, args, kwargs, hash_only=hash_only, watch=watch)

            return method
        if gate.active:
            gate.recorder.call(port, name, request={"get": True}, response=value)
        return value

    def __setattr__(self, name: str, value) -> None:
        target, gate = object.__getattribute__(self, "_target"), object.__getattribute__(self, "_gate")
        setattr(target, name, value)
        if gate.active:
            gate.recorder.call(object.__getattribute__(self, "_port"), name, request={"set": value}, response=None)

    @property
    def unwrapped(self):
        return object.__getattribute__(self, "_target")
