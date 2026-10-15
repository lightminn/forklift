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

__all__ = ["BoundaryRecorder", "DictWatch", "measurement_refusal", "RecordingGate", "RecordingProxy", "read_record", "recorded_function"]


class BoundaryRecorder:
    def __init__(self, directory: Path, *, chunk_ticks: int = 1200, max_bytes: int = 2_000_000_000,
                 value_format: int = 1) -> None:
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
        # Format 2 (the M-a1 replay, docs/plans/2026-10-16-m-a1-replay.md): values through replay_codec, so a
        # replay can hand them back (tuples, qualified dataclasses, aliases); format 1 keeps the comparison form.
        if value_format not in (1, 2):
            raise ValueError("value_format is 1 or 2")
        self.value_format = int(value_format)

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

    def encode_request(self, raw: Any, hash_only: bool = False) -> Any:
        """A call's arguments encoded now; ``hash_only`` keeps arrays by content hash only (format 2 keeps
        every other type and the alias graph; format 1 flattens as before)."""
        if self.value_format == 1:
            return self._encode(_hash_only(raw) if hash_only else raw)
        return self._encode(raw, hash_arrays=hash_only)

    def encode_changes(self, request_after: Any, watched: dict | None, replaced_paths=(), problem=None) -> dict | None:
        """Format 2: a call's changes as plain JSON whose leaves are value nodes -- ``request_after`` (an encoded
        request) with the argument paths that now hold another (fresh) object, and per watched dictionary and
        key one of ``{"value": node, "replaced_paths": [...]}``, ``{"append": node}`` or ``{"deleted": true}``.
        ``unsupported`` names a change outside the replay's change model (``check_change``); a replay refuses
        that call."""
        out: dict = {}
        problems = [problem] if problem else []
        if request_after is not None:
            out["request_after"] = request_after
            if replaced_paths:
                out["replaced_paths"] = [list(q) for q in replaced_paths]
        if watched:
            enc: dict = {}
            for name, diff in watched.items():
                enc[name] = {}
                for key, change in diff.items():
                    if isinstance(change, dict) and change.get("$deleted") is True and len(change) == 1:
                        enc[name][key] = {"deleted": True}
                    elif isinstance(change, dict) and set(change) == {"$append"}:
                        enc[name][key] = {"append": self._encode(change["$append"])}
                    else:
                        node = {"value": self._encode(change["$value"])}
                        if change.get("$replaced_paths"):
                            node["replaced_paths"] = [list(q) for q in change["$replaced_paths"]]
                        if change.get("$problem"):
                            problems.append(f"{name}.{key}: {change['$problem']}")
                        enc[name][key] = node
            out["watched"] = enc
        if problems:
            out["unsupported"] = "; ".join(problems)
        return out or None

    def call(self, port: str, method: str, *, request: Any, response: Any = None, exception: BaseException | None = None,
             changes: Any = None, encoded_request: Any = None) -> None:
        """``encoded_request`` (from :meth:`encode` before the call) replaces ``request``."""
        if self._tick is None:
            raise RuntimeError("boundary call recorded outside a tick")
        entry = {"seq": len(self._tick["calls"]), "port": port, "method": method,
                 "request": encoded_request if encoded_request is not None else self._encode(request)}
        if exception is not None:
            type_name = (type(exception).__name__ if self.value_format == 1
                         else f"{type(exception).__module__}:{type(exception).__qualname__}")
            entry["exception"] = {"type": type_name, "message": str(exception)}
            if self.value_format == 2:  # what rebuilding it needs: its args (errno, filename ... ride in them)
                entry["exception"]["args"] = self._encode(tuple(exception.args))
                if isinstance(exception, OSError) and (exception.filename is not None or exception.filename2 is not None):
                    # Native state outside args (OSError keeps the file names apart).
                    entry["exception"]["os_error"] = self._encode(
                        (exception.errno, exception.strerror, exception.filename, None, exception.filename2))
            attributes = {k: v for k, v in vars(exception).items() if not k.startswith("__")} if hasattr(exception, "__dict__") else {}
            if attributes:  # what a caller reads from the exception (e.g. a capture failure's diagnostics)
                entry["exception"]["attributes"] = self._encode(attributes)
        else:
            entry["response"] = self._encode(response)
        if changes is not None:
            entry["changes"] = changes if self.value_format == 2 else self._encode(changes)
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

    def close(self, final_state: Any = None) -> None:
        """``final_state`` (format 2): the core's containers as the run ended -- written beside the record so a
        replay can compare its own end state (changes a call made that its record entry does not carry, e.g.
        to an already-logged item, show up there)."""
        if self._closed:
            return
        self._final_state = False
        if final_state is not None and self.value_format == 2:
            from forklift_core.mission.replay_codec import Encoder

            # Comparison only: arrays by content hash and read-only state.
            node = Encoder(opaque_ok=True, value_types=self.value_types, hash_arrays=True,
                           identity=False).encode(final_state)
            (self.directory / "final_state.json").write_text(json.dumps(node, sort_keys=True, allow_nan=True) + "\n")
            self._final_state = True
        if self._tick is not None:
            raise RuntimeError("close inside a tick")
        self._closed = True
        self._flush()
        (self.directory / "boundary_meta.json").write_text(json.dumps({
            "format": "forklift_boundary_record/1", "ticks": self._ticks, "chunks": self._chunk,
            "bytes": self._bytes, "stopped_at_tick": self.stopped_at_tick, "stopped_reason": self.stopped_reason,
            "opaque_values": self.opaque_values,
        } | ({"format": "forklift_boundary_record/2", "value_types": sorted(self.value_types),
              "final_state": self._final_state}
             if self.value_format == 2 else {}), indent=1) + "\n")

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

    def _encode(self, value: Any, *, hash_arrays: bool = False) -> Any:
        if self.value_format == 2:
            from forklift_core.mission.replay_codec import Encoder

            encoder = Encoder(opaque_ok=self.opaque_ok, value_types=self.value_types, hash_arrays=hash_arrays)
            node = encoder.encode(value)
            for key, array in encoder.arrays.items():
                if key not in self._written and key not in self._pending_arrays:
                    self._pending_arrays[key] = array
            self.opaque_values += json.dumps(node).count('"$opaque"')
            return node
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


def measurement_refusal(result: dict) -> str | None:
    """Why a run may not be measurement evidence (a bounds, error-table or gate-bound input), or None: a
    boundary-recorded run (``--record-boundary``) applies fixed bounds and may be judged against them, but
    never produces new ones -- the recorder can shift wall-clock timing (docs/plans/2026-10-15-m-a05-
    boundary.md section 5). A result without the argument predates it and was not recorded."""
    if ((result or {}).get("arguments") or {}).get("record_boundary"):
        return "a boundary-recorded run (--record-boundary) is never measurement evidence"
    return None


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
        the watched core dictionaries (``watched``). Format 2 also checks the call's effect against the
        replay's change model (:func:`check_state`) and records where fresh objects replaced old ones."""
        if not self.active:
            return fn(*args, **kwargs)
        raw = {"args": list(args), "kwargs": dict(kwargs)}
        before = self.recorder.encode_request(raw, hash_only)
        marks = watch.mark() if watch is not None else None
        state = _State(self.recorder, raw, watch) if self.recorder.value_format == 2 else None
        self.depth += 1
        try:
            result = fn(*args, **kwargs)
        except BaseException as exc:
            self.depth -= 1
            self.recorder.call(port, method, request=None, encoded_request=before, exception=exc,
                               changes=self._changes(raw, before, hash_only, watch, marks, state))
            raise
        self.depth -= 1
        changes = self._changes(raw, before, hash_only, watch, marks, state)
        if self.recorder.value_format == 2 and _shares_objects(result, raw):
            # A response that hands back an argument's own object: a replay cannot restore that identity.
            changes = dict(changes or {}, response_aliases_request=True)
        self.recorder.call(port, method, request=None, encoded_request=before, response=result, changes=changes)
        return result

    def _changes(self, raw, before, hash_only, watch, marks, state=None):
        after = self.recorder.encode_request(raw, hash_only)
        changed_request = json.dumps(after, sort_keys=True) != json.dumps(before, sort_keys=True)
        if self.recorder.value_format == 2:
            replaced, problem = check_state(state, _State(self.recorder, raw, watch, texts=False))
            arg_paths = [q[1:] for q in replaced if q[0] == "A"]
            watched_paths: dict = {}
            for q in replaced:
                if q[0] == "W":
                    watched_paths.setdefault((q[1], q[2]), []).append(q[3:])
            watched = watch.changes(marks, force=set(watched_paths)) if watch is not None else None
            for (name, key), rel in watched_paths.items():
                change = (watched or {}).get(name, {}).get(key)
                if isinstance(change, dict) and "$value" in change:
                    change["$replaced_paths"] = rel
            if arg_paths or problem:
                changed_request = True
            return self.recorder.encode_changes(after if changed_request else None, watched, arg_paths, problem)
        watched = watch.changes(marks) if watch is not None else None
        out = {}
        if changed_request:
            out["request_after"] = after
        if watched:
            out["watched"] = watched
        return out or None


def _mutable(value: Any) -> bool:
    return isinstance(value, (list, dict, np.ndarray)) or (
        dataclasses.is_dataclass(value) and not isinstance(value, type)) or (
        hasattr(value, "__dict__") and not isinstance(value, type) and not callable(value))


def _simple_key(key: Any) -> bool:
    """Keys the replay rebuilds exactly by value (no floats: NaN keys and -0.0 do not round-trip by value)."""
    return key is None or isinstance(key, (str, int, bool)) or (
        isinstance(key, tuple) and all(_simple_key(k) for k in key))


def _walk(value: Any, path: tuple, out: dict, problems: list, stack=()) -> None:
    """path -> every mutable object reachable from ``value`` (dict keys by repr, list and tuple indices,
    ``.attribute``; an object reached by two paths appears under both). Dictionary keys must be plain
    immutable values (the replay rebuilds them by value)."""
    from forklift_core.mission.replay_codec import plain_value

    if not (_mutable(value) or plain_value(value) or isinstance(value, tuple) or callable(value)):
        problems.append(f"{path} holds a {type(value).__name__}, which has no value encoding")
        return
    if _mutable(value):
        out[path] = value
        if id(value) in stack:
            problems.append(f"{path} closes a reference cycle")
            return
        stack = stack + (id(value),)
        if (not isinstance(value, (list, dict, np.ndarray)) and not dataclasses.is_dataclass(value)
                and type(value).__name__ not in getattr(_walk, "value_types", ())):
            # Not encoded by value (recorded as opaque): its changes cannot be seen or replayed.
            problems.append(f"{path} holds a {type(value).__name__} with no value encoding")
            return
    if isinstance(value, dict):
        for k, v in value.items():
            if not _simple_key(k):
                problems.append(f"{path} has a {type(k).__name__} key")
            _walk(v, path + (repr(k),), out, problems, stack)
    elif isinstance(value, (list, tuple)):
        for i, v in enumerate(value):
            _walk(v, path + (i,), out, problems, stack)
    elif dataclasses.is_dataclass(value) and not isinstance(value, type):
        for f in dataclasses.fields(value):
            _walk(getattr(value, f.name), path + ("." + f.name,), out, problems, stack)
    elif hasattr(value, "__dict__") and not isinstance(value, type) and not callable(value):
        for k, v in vars(value).items():
            if not k.startswith("_"):
                _walk(v, path + ("." + k,), out, problems, stack)


def _compare_text(recorder, value: Any) -> str:
    """A comparison encoding: arrays by hash, nothing stored, not counted as recorded values."""
    from forklift_core.mission.replay_codec import Encoder

    return json.dumps(Encoder(opaque_ok=True, value_types=recorder.value_types, hash_arrays=True).encode(value),
                      sort_keys=True)


class _State:
    """One call's view of what it may change: the arguments (paths under ``("A",)``), each watched key's
    value (``("W", dict, key)``) and each append-only watched list (its object and its items). ``texts``:
    each object's encoding, to tell later whether an object that left its path was changed."""

    def __init__(self, recorder, raw: dict, watch, *, texts: bool = True) -> None:
        self.paths: dict = {}
        self.problems: list = []
        _walk.value_types = recorder.value_types
        _walk(raw["args"], ("A", "'args'"), self.paths, self.problems)
        _walk(raw["kwargs"], ("A", "'kwargs'"), self.paths, self.problems)
        self.paths.pop(("A", "'args'"), None)  # the call's own argument containers
        self.paths.pop(("A", "'kwargs'"), None)
        self.appends: dict = {}
        for name, d in ((watch.dicts if watch is not None else {}) or {}).items():
            if d is None:
                continue
            for k, v in list(d.items()):
                if not watch.in_scope(name, k):
                    continue
                if not isinstance(k, str):
                    self.problems.append(f"watched {name} has a {type(k).__name__} key")
                if k in watch.append_only and isinstance(v, list):
                    self.appends[(name, k)] = (v, list(v))  # the list and its items (kept alive)
                    # The last logged item by content (an edit to it is checked per call); every item's objects
                    # by identity (cached per list, items are never re-walked once seen).
                    ids = watch.logged_ids(name, k, v)
                    self.log_ids = getattr(self, "log_ids", set()) | ids
                    self.problems += [f"log {name}.{k} item {q}" for q in watch.logged_problems(name, k)]
                    if v and texts:
                        self.log_last = getattr(self, "log_last", {})
                        self.log_last[(name, k)] = (v[-1], _compare_text(recorder, v[-1]))
                else:
                    _walk(v, ("W", name, k), self.paths, self.problems)
        # Comparison only: arrays by hash (nothing stored in the record).
        self.texts = {q: _compare_text(recorder, o) for q, o in self.paths.items()} if texts else None
        self.recorder = recorder


def check_state(before: "_State", after: "_State") -> tuple[list, str | None]:
    """Whether a call's effect fits the replay's change model, and where fresh objects replaced old ones.

    The model: an object that was there stays at its path (its content may change); a path that holds
    another object holds a fresh one, and so does every path under it; a fresh object (at a new path, a
    replaced one or appended to an append-only list) contains no object that existed before anywhere in the
    call's view and is reached by one path only; an object that left the view is unchanged; append-only lists
    keep their object and their items and only grow; dictionary keys are plain values. Returns (the minimal
    replaced paths, None) or ([], why not)."""
    if after.problems or before.problems:
        return [], "; ".join(sorted(set(before.problems + after.problems)))
    before_ids = {id(o) for o in before.paths.values()} | getattr(before, "log_ids", set())
    for lst, items in before.appends.values():
        before_ids.add(id(lst))
        before_ids.update(id(x) for x in items)
    for key, (item, text) in getattr(before, "log_last", {}).items():
        if _compare_text(before.recorder, item) != text:
            return [], f"append-only {key}: its last logged item was changed"
    seen: dict[int, tuple] = {}
    for key in after.appends:
        if key not in before.appends:  # a new log: a fresh list of its own, its items like any appended item
            lst = after.appends[key][0]
            if id(lst) in before_ids or id(lst) in seen:
                return [], f"append-only {key} is new and holds an object that existed or is held elsewhere"
            seen[id(lst)] = ("new log", key)
            before.appends[key] = (lst, [])
    differ = [q for q, o in after.paths.items() if q in before.paths and before.paths[q] is not o]
    differ.sort(key=lambda q: (len(q), repr(q)))
    replaced = [q for q in differ if not any(q[:len(r)] == r for r in differ if len(r) < len(q))]

    def under(q):
        return any(q[:len(r)] == r for r in replaced)

    def stays(q, o):  # an old object still at its own path (before and after), under no replaced path
        return (q in before.paths and before.paths[q] is o and q in after.paths and after.paths[q] is o
                and not under(q))

    for q, o in after.paths.items():
        if stays(q, o):
            continue
        if id(o) in before_ids:
            return [], f"{q} now holds an object that existed before the call"
        if id(o) in seen:
            return [], f"{q} and {seen[id(o)]} now hold one new object"
        seen[id(o)] = q
    view_ids = {id(o) for o in before.paths.values()}
    log_lists = {id(lst) for lst, _ in before.appends.values()}
    for key, (lst, items) in before.appends.items():
        now = after.appends.get(key)
        if now is None or now[0] is not lst:
            return [], f"append-only {key} was replaced or removed"
        cur = now[1]
        if len(cur) < len(items) or any(a is not b for a, b in zip(cur, items)):
            return [], f"append-only {key} changed its existing items"
        for x in cur[len(items):]:
            inner: dict = {}
            problems: list = []
            _walk(x, (), inner, problems)
            if problems:
                return [], f"append-only {key} got an item outside the model: {'; '.join(problems)}"
            # Logs are value records: an appended item may share objects with logged items (a cached plan
            # bundle, run_transport grid_kwargs) -- the replay gives each its own copy, nothing mutates a logged
            # object afterwards -- but never with the core's state (arguments, watched values) or the list itself.
            for o in {id(o): o for o in inner.values()}.values():
                if id(o) in view_ids or id(o) in log_lists or seen.get(id(o), ("",))[0] not in ("", "logged"):
                    return [], f"append-only {key} got an object held by the core's state"
                seen.setdefault(id(o), ("logged", key))
    after_ids = {id(o) for o in after.paths.values()}
    for q, o in before.paths.items():
        if stays(q, o):
            continue
        if id(o) in after_ids:
            # Still in the view: legitimate only where it also stays at an old path of its own.
            if not any(stays(r, o) for r, x in after.paths.items() if x is o):
                return [], f"the object at {q} moved"
            continue
        if _compare_text(before.recorder, o) != before.texts[q]:
            return [], f"the object that left {q} was changed"
    return replaced, None


def _identities(value: Any, out: set, depth: int = 0) -> set:
    """The ids of the mutable objects (containers, arrays, dataclass and plain objects) inside ``value``."""
    if depth > 50 or value is None or isinstance(value, (bool, int, float, str, bytes)):
        return out
    if id(value) in out:
        return out
    if isinstance(value, (list, dict, np.ndarray)) or dataclasses.is_dataclass(value) or hasattr(value, "__dict__"):
        out.add(id(value))
    if isinstance(value, dict):
        for v in value.values():
            _identities(v, out, depth + 1)
    elif isinstance(value, (list, tuple)):
        for v in value:
            _identities(v, out, depth + 1)
    elif dataclasses.is_dataclass(value) and not isinstance(value, type):
        for f in dataclasses.fields(value):
            _identities(getattr(value, f.name), out, depth + 1)
    elif hasattr(value, "__dict__") and not isinstance(value, type):
        for k, v in vars(value).items():
            if not k.startswith("_"):
                _identities(v, out, depth + 1)
    return out


def _shares_objects(response: Any, raw: dict) -> bool:
    if response is None:
        return False
    inner = _identities(raw, set()) - {id(raw), id(raw["args"]), id(raw["kwargs"])}
    return bool(inner & _identities(response, set()))


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

    def logged_ids(self, name: str, key, lst: list) -> set:
        """The ids of every mutable object inside an append-only list's items (format 2's change check),
        cached per list: an item is walked once, when first seen."""
        cache = self.__dict__.setdefault("_logged", {})
        entry = cache.get((name, key))
        if entry is None or entry[0] is not lst or entry[1] > len(lst):
            entry = [lst, 0, set(), []]  # the list, items seen, ids, the items (kept alive)
            cache[(name, key)] = entry
        problems = entry[4] if len(entry) > 4 else []
        if len(entry) == 4:
            entry.append(problems)
        for item in lst[entry[1]:]:
            inner: dict = {}
            _walk(item, (len(entry[3]),), inner, problems)
            entry[2].update(id(o) for o in inner.values())
            entry[3].append(item)
        entry[1] = len(lst)
        return set(entry[2])

    def logged_problems(self, name: str, key) -> list:
        entry = self.__dict__.get("_logged", {}).get((name, key))
        return list(entry[4]) if entry is not None and len(entry) > 4 else []

    def in_scope(self, name: str, key) -> bool:
        only = self.keys.get(name)
        return key not in self.skip and (only is None or key in only)

    def _snapshot(self, d: dict, name: str = "") -> dict:
        snap = {}
        v2 = self.recorder.value_format == 2
        for k, v in list(d.items()):
            if not self.in_scope(name, k):
                continue
            if k in self.append_only and isinstance(v, list):
                snap[k] = ("$len", len(v), v) if v2 else ("$len", len(v))
            else:
                snap[k] = json.dumps(self.recorder.encode(v), sort_keys=True)
        return snap

    def mark(self) -> dict:
        return {name: self._snapshot(d, name) for name, d in self.dicts.items() if d is not None}

    def changes(self, marks: dict, force=()) -> dict:
        """``force``: (dict, key) pairs reported as value changes even when their encoding is the same (format
        2: a fresh object put in place of an equal one)."""
        out = {}
        for name, d in self.dicts.items():
            if d is None or name not in marks:
                continue
            before, now = marks[name], self._snapshot(d, name)
            diff = {}
            for k in set(before) | set(now):
                if before.get(k) == now.get(k) and (name, k) not in force:
                    continue
                if k not in d:
                    diff[k] = {"$deleted": True}
                elif isinstance(now.get(k), tuple) and now[k][0] == "$len" and (
                        len(now[k]) == 2 or (isinstance(before.get(k), tuple) and before[k][2] is now[k][2])):
                    start = before[k][1] if isinstance(before.get(k), tuple) else 0
                    if len(now[k]) == 3 and start == now[k][1]:
                        continue  # same list, same length: nothing appended (format 2 compares the list object)
                    diff[k] = {"$append": d[k][start:]}
                elif self.recorder.value_format == 2:
                    diff[k] = {"$value": d[k]}
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
