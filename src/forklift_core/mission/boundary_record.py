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

import collections
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

    def tick_time(self, *, tick_t: float, work_t: float, commands: list | None = None,
                  stop_kind: str | None = None) -> None:
        """Format 2's two-step tick head: the times, read inside the open tick, set after it began -- and, with
        mission commands (M-a3), the commands this tick applied and the stop kind it ran under."""
        if self._tick is None:
            raise RuntimeError("tick_time outside a tick")
        self._tick["tick_t"], self._tick["work_t"] = self._encode(tick_t), self._encode(work_t)
        if commands is not None:
            self._tick["commands"] = self._encode(commands)
        if stop_kind is not None:
            self._tick["stop_kind"] = stop_kind

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

    def close(self, final_state: Any = None, ending: dict | None = None) -> None:
        """``final_state`` (format 2): the core's containers as the run ended -- written beside the record so a
        replay can compare its own end state (changes a call made that its record entry does not carry, e.g.
        to an already-logged item, show up there). ``ending`` (format 2, :func:`boundary_ending`): how the
        recorded code left -- returned, or the exception it raised -- which a replay must end with too."""
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
        log_changes = [p for w in self.__dict__.get("_watches", []) for p in w.final_problems()]
        self._closed = True
        self._flush()
        (self.directory / "boundary_meta.json").write_text(json.dumps({
            "format": "forklift_boundary_record/1", "ticks": self._ticks, "chunks": self._chunk,
            "bytes": self._bytes, "stopped_at_tick": self.stopped_at_tick, "stopped_reason": self.stopped_reason,
            "opaque_values": self.opaque_values,
        } | ({"format": "forklift_boundary_record/2", "value_types": sorted(self.value_types),
              "final_state": self._final_state, "ending": ending, "log_changes": log_changes}
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
            from forklift_core.mission.replay_codec import recorded_fields

            return {"$dataclass": type(value).__name__,
                    **{f.name: self._encode(getattr(value, f.name)) for f in recorded_fields(value)}}
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
        from forklift_core.mission.replay_codec import recorded_fields

        return {"$dataclass": type(value).__name__,
                **{f.name: _hash_only(getattr(value, f.name)) for f in recorded_fields(value)}}
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

    @property
    def value_format(self) -> int:
        return self.recorder.value_format

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
            # A response that hands back an argument's own object: where, so a replay puts its own argument's object
            # there (e.g. dataclasses.replace keeping a dict field); ``True`` when one has no argument path.
            changes = dict(changes or {}, response_aliases_request=_alias_paths(result, raw) or True)
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


def _immutable(value: Any) -> bool:
    from forklift_core.mission.replay_codec import immutable

    return immutable(value)


def _mutable(value: Any) -> bool:
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return not _immutable(value)
    return isinstance(value, (list, dict, np.ndarray, collections.deque)) or (
        hasattr(value, "__dict__") and not isinstance(value, type) and not callable(value))


def _simple_key(key: Any) -> bool:
    """Keys the replay rebuilds exactly by value (no floats: NaN keys and -0.0 do not round-trip by value)."""
    return key is None or isinstance(key, (str, int, bool)) or (
        isinstance(key, tuple) and all(_simple_key(k) for k in key))


def _walk(value: Any, path: tuple, out: dict, problems: list, stack=()) -> None:
    """path -> every mutable object reachable from ``value`` (dict keys by repr, list and tuple indices,
    ``.attribute``; an object reached by two paths appears under both). Dictionary keys must be plain
    immutable values (the replay rebuilds them by value)."""
    if _immutable(value):
        return  # nothing in it can change or be shared harmfully
    if not (_mutable(value) or isinstance(value, tuple) or callable(value)):
        problems.append(f"{path} holds a {type(value).__name__}, which has no value encoding")
        return
    if _mutable(value):
        out[path] = value
        if id(value) in stack:
            problems.append(f"{path} closes a reference cycle")
            return
        stack = stack + (id(value),)
        if (not isinstance(value, (list, dict, np.ndarray, collections.deque)) and not dataclasses.is_dataclass(value)
                and type(value).__name__ not in getattr(_walk, "value_types", ())):
            # Not encoded by value (recorded as opaque): its changes cannot be seen or replayed.
            problems.append(f"{path} holds a {type(value).__name__} with no value encoding")
            return
    if isinstance(value, dict):
        for k, v in value.items():
            if not _simple_key(k):
                problems.append(f"{path} has a {type(k).__name__} key")
            _walk(v, path + (repr(k),), out, problems, stack)
    elif isinstance(value, (list, tuple, collections.deque)):
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
                    # The list, its length and its last item (nothing copied: a long log made every call O(n),
                    # Codex P2); every logged item's objects by identity and position, cached per list and walked
                    # once (watch.logged_index); an older item replaced in place is caught when the record closes
                    # (DictWatch.final_problems), the last item per call by content (log_last).
                    self.appends[(name, k)] = (v, len(v), v[-1] if v else _NO_ITEM)
                    self.log_index = getattr(self, "log_index", {})
                    self.log_index[(name, k)] = watch.logged_index(name, k, v)
                    self.problems += [f"log {name}.{k} item {q}" for q in watch.logged_problems(name, k)]
                    if v and texts:
                        self.log_last = getattr(self, "log_last", {})
                        self.log_last[(name, k)] = (v[-1], _compare_text(recorder, v[-1]))
                else:
                    _walk(v, ("W", name, k), self.paths, self.problems)
        # Comparison only: arrays by hash (nothing stored in the record).
        self.texts = {q: _compare_text(recorder, o) for q, o in self.paths.items()} if texts else None
        self.recorder = recorder


_NO_ITEM = object()  # an empty log's "last item"


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
    before_small = {id(o) for o in before.paths.values()} | {id(lst) for lst, _, _ in before.appends.values()}
    log_index = getattr(before, "log_index", {})

    class _Existed:  # `id in before_ids`: the view's objects, the logs, and every logged item's objects so far
        def __contains__(self, i):
            return i in before_small or any(d.get(i, n) < n for d, n in log_index.values())

    before_ids = _Existed()
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
            before.appends[key] = (lst, 0, _NO_ITEM)
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
    log_lists = {id(lst) for lst, _, _ in before.appends.values()}
    for key, (lst, count, last) in before.appends.items():
        now = after.appends.get(key)
        if now is None or now[0] is not lst:
            return [], f"append-only {key} was replaced or removed"
        if now[1] < count or (count and lst[count - 1] is not last):
            return [], f"append-only {key} changed its existing items"
        for x in lst[count:now[1]]:
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
    if depth > 50 or _immutable(value):
        return out  # a value no one can change: sharing it is no alias
    if id(value) in out:
        return out
    if isinstance(value, (list, dict, np.ndarray, collections.deque)) or dataclasses.is_dataclass(value) \
            or hasattr(value, "__dict__"):
        out.add(id(value))
    if isinstance(value, dict):
        for v in value.values():
            _identities(v, out, depth + 1)
    elif isinstance(value, (list, tuple, collections.deque)):
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


def _alias_paths(response: Any, raw: dict) -> list | None:
    """Where ``response`` holds an argument's own object: ``[response path, argument path]`` pairs, the outermost
    shared objects only, response paths under ``("R",)`` and argument paths under ``("A", "'args'" | "'kwargs'")``
    in :func:`_walk`'s steps. None when a shared object has no argument path."""
    shared = _identities(raw, set()) - {id(raw), id(raw["args"]), id(raw["kwargs"])}
    paths: dict = {}
    _walk(raw["args"], ("A", "'args'"), paths, [])
    _walk(raw["kwargs"], ("A", "'kwargs'"), paths, [])
    where: dict = {}
    for q, o in paths.items():
        where.setdefault(id(o), q)
    pairs: list = []
    walked: set = set()  # the response's own containers: each walked once (a cycle, or one held twice -- the
    # decoder restores that sharing, so the argument put at the first path is at the others too). Not tuples: the
    # codec writes a tuple at each place, so each place needs its own pairs (Codex review 2 P2).

    def visit(value, path, depth) -> bool:
        if depth > 50 or _immutable(value):
            return True
        if id(value) in shared:  # every place, not only the first: each gets the argument's object
            q = where.get(id(value))
            if q is None or q in (("A", "'args'"), ("A", "'kwargs'")):
                return False
            pairs.append([list(path), list(q)])
            return True
        if not isinstance(value, tuple):
            if id(value) in walked:
                return True
            walked.add(id(value))
        if isinstance(value, dict):
            return all(visit(v, path + (repr(k),), depth + 1) for k, v in value.items())
        if isinstance(value, (list, tuple, collections.deque)):
            return all(visit(v, path + (i,), depth + 1) for i, v in enumerate(value))
        if dataclasses.is_dataclass(value) and not isinstance(value, type):
            return all(visit(getattr(value, f.name), path + ("." + f.name,), depth + 1)
                       for f in dataclasses.fields(value))
        if hasattr(value, "__dict__") and not isinstance(value, type) and not callable(value):
            return all(visit(v, path + ("." + k,), depth + 1) for k, v in vars(value).items() if not k.startswith("_"))
        return True

    return pairs if visit(response, ("R",), 0) and pairs else None


def boundary_ending(exc: BaseException | None) -> dict:
    """How the recorded code ended, for the record's meta: returned, or the exception's type and message."""
    if exc is None:
        return {"kind": "returned"}
    return {"kind": "exception", "type": f"{type(exc).__module__}:{type(exc).__qualname__}", "message": str(exc)}


def payload_digest(payload) -> str:
    """The sha256 of what a sink writes, by value: text as UTF-8, anything else through the codec with arrays by
    content (a file's own bytes would carry the archive's timestamps, not the payload)."""
    import hashlib

    if isinstance(payload, str):
        data = payload.encode("utf-8")
    else:
        from forklift_core.mission.replay_codec import Encoder

        data = json.dumps(Encoder(opaque_ok=False, hash_arrays=True, identity=False).encode(payload), sort_keys=True,
                          allow_nan=True).encode("utf-8")
    return hashlib.sha256(data).hexdigest()


def trace_entry(gate, method: str, descriptor) -> None:
    """Format 2: a ``Trace.<method>`` entry -- what the code computed at a point no port records (e.g. the new
    obstacle schedule's inputs and actions, plan section 6) -- compared on replay as a request."""
    if gate is None or not gate.active or getattr(gate, "value_format", 1) != 2:
        return
    gate.run("Trace", method, lambda d: None, (descriptor,), {})


def sink_effect(gate, method: str, path, payload, write=None):
    """Format 2 (plan 3.5): an output effect -- a file the run writes, a line it prints -- as a ``Sink.<method>``
    entry with its logical path (relative to the run's output) and the payload's digest, recorded where it happens.
    ``write`` makes the effect inside the recorded call, so its exception (a full disk) is recorded with it and a
    replay raises the same one at the same place (Codex P2); it makes no port calls of its own. A replay makes the
    write too (``ReplayGate`` runs a Sink's write) and compares only what was recorded. Returns what ``write``
    returned."""
    if gate is None or not gate.active or getattr(gate, "value_format", 1) != 2:
        return None if write is None else write()
    return gate.run("Sink", method, lambda effect: None if write is None else write(),
                    ({"path": str(path), "sha256": payload_digest(payload)},), {})


def _callback_description(value):
    """A callback's request descriptor: what it declares as ``callback_spec`` (the settings its body runs with),
    else its type -- a callback without one is still seen, never the settings inside it."""
    if isinstance(value, (list, tuple)):
        return [_callback_description(v) for v in value]
    spec = getattr(value, "callback_spec", None)
    if spec is not None:
        return spec
    return {"unspecified": f"{getattr(value, '__module__', None)}:{getattr(value, '__qualname__', type(value).__qualname__)}"}


def _is_callback(value) -> bool:
    if isinstance(value, (list, tuple)):
        return bool(value) and all(_is_callback(v) for v in value)
    return callable(value) and not isinstance(value, type)


def callback_trace(gate, port: str, method: str, args: tuple, kwargs: dict, callback_spec=None) -> None:
    """Format 2 (plan 3.4): before a call that hands callbacks to a port -- whose request records them by type only
    -- a ``Trace.callback_spec`` entry with each callback's descriptor, whether the port runs it or not.
    ``callback_spec(args, kwargs)`` builds the descriptors of a call whose callbacks the port makes itself
    (redraw_docking_traced: its dry runs from ``dry_configs``)."""
    if gate is None or not gate.active or getattr(gate, "value_format", 1) != 2:
        return
    if callback_spec is not None:
        callbacks = callback_spec(args, kwargs)
    else:
        callbacks = {k: _callback_description(v) for k, v in sorted(kwargs.items()) if _is_callback(v)}
        # Positional callbacks by position ("#i"): a replay accepts a callable by type only where it is described.
        callbacks.update({f"#{i}": _callback_description(v) for i, v in enumerate(args) if _is_callback(v)})
    if not callbacks:
        return
    gate.run("Trace", "callback_spec", lambda descriptor: None,
             ({"call": f"{port}.{method}", "callbacks": callbacks},), {})


def recorded_function(fn, port: str, method: str, gate: RecordingGate, *, hash_only: bool = False,
                      watch: "DictWatch | None" = None, callback_spec=None):
    """``fn`` with each call recorded through ``gate`` (the result and exceptions pass through);
    ``watch`` records what the call changed in core dictionaries. Format 2 puts a ``Trace.callback_spec``
    entry before a call that passes callbacks (:func:`callback_trace`)."""

    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        callback_trace(gate, port, method, args, kwargs, callback_spec)
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
        if recorder is not None:  # its logs are checked once more when the record closes
            recorder.__dict__.setdefault("_watches", []).append(self)
        self.skip = set(skip)
        # Per dictionary, the only keys compared (None: all but ``skip``) -- keeps a per-physics-tick watch cheap.
        self.keys = {name: set(k) for name, k in (keys or {}).items()}

    def logged_index(self, name: str, key, lst: list) -> tuple[dict, int]:
        """(id -> first position, item count) for the objects inside an append-only list's items and the items
        themselves (format 2's change check), cached per list: an item is walked once, when first seen. The
        dict is shared and only grows: a caller holding an older count reads only the positions below it."""
        cache = self.__dict__.setdefault("_logged", {})
        entry = cache.get((name, key))
        if entry is None or entry[0] is not lst or entry[1] > len(lst):
            entry = [lst, 0, {}, [], []]  # the list, items seen, id -> position, the items (kept alive), problems
            cache[(name, key)] = entry
        for item in lst[entry[1]:]:
            position = len(entry[3])
            inner: dict = {}
            _walk(item, (position,), inner, entry[4])
            entry[2].setdefault(id(item), position)
            for o in inner.values():
                entry[2].setdefault(id(o), position)
            entry[3].append(item)
        entry[1] = len(lst)
        return entry[2], entry[1]

    def logged_ids(self, name: str, key, lst: list) -> set:
        index, _ = self.logged_index(name, key, lst)
        return set(index)

    def final_problems(self) -> list:
        """Logged items replaced in place since they were first seen (checked once, when the record closes)."""
        out = []
        for (name, key), entry in self.__dict__.get("_logged", {}).items():
            lst, items = entry[0], entry[3]
            if len(lst) < len(items) or any(a is not b for a, b in zip(lst, items)):
                out.append(f"append-only {name}.{key} changed an item already logged")
        return out

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
                if _same_snapshot(before.get(k), now.get(k)) and (name, k) not in force:
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


def _same_snapshot(a, b) -> bool:
    """Two snapshot entries alike: an append-only list by its length and object, never by its items (comparing a
    long log item by item every physics step made recording quadratic in the run length)."""
    if isinstance(a, tuple) and isinstance(b, tuple) and a and b and a[0] == b[0] == "$len":
        return a[1] == b[1] and len(a) == len(b) and (len(a) == 2 or a[2] is b[2])
    return a == b


@dataclasses.dataclass(frozen=True)
class ServiceRef:
    """What a recorded call answered when it handed back a service object (a ``returns`` method): the port
    its own calls are recorded under and its methods (a replay builds a proxy of it without its class)."""

    port: str
    methods: tuple


def _method_names(obj) -> tuple:
    names, seen = set(), set()
    for klass in type(obj).__mro__:
        for key, value in vars(klass).items():
            if key in seen:
                continue
            seen.add(key)
            if not isinstance(value, property) and (callable(value) or isinstance(value, (staticmethod, classmethod))):
                names.add(key)
    return tuple(sorted(names))


class RecordingProxy:
    """A service object whose method calls, attribute reads and attribute writes are recorded through
    ``gate``; every value returned is the real object's. ``hash_only_requests`` names methods whose
    request arrays are recorded by hash only (large frames). ``child_ports`` maps attribute names that
    hold sub-services (e.g. the obstacle layer's ``permission``) to the port their calls are recorded
    under: reading such an attribute returns a proxy of it instead of recording an opaque object."""

    def __init__(self, target, port: str, gate: RecordingGate, *, hash_only_requests=(), child_ports=None,
                 watches=None, returns=None) -> None:
        object.__setattr__(self, "_target", target)
        # Methods that hand back a service object: method name -> the port that object's calls go under; the call
        # is recorded as answering a ServiceRef and the caller gets a proxy of the object.
        object.__setattr__(self, "_returns", dict(returns or {}))
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
            returned_port = object.__getattribute__(self, "_returns").get(name)
            if returned_port is not None:
                def method(*args, **kwargs):
                    real = []

                    def call(*a, **k):
                        obj = value(*a, **k)
                        real.append(obj)
                        return None if obj is None else ServiceRef(returned_port, _method_names(obj))

                    gate.run(port, name, call, args, kwargs, hash_only=hash_only, watch=watch)
                    return None if real[0] is None else RecordingProxy(real[0], returned_port, gate)

                return method

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
