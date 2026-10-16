"""Replaying a format-2 boundary record (docs/plans/2026-10-16-m-a1-replay.md, steps R1/R4).

The replayed code runs with :class:`ReplayGate` where the recorded run had a ``RecordingGate`` and with
:class:`ReplayProxy` where it had ``RecordingProxy`` objects. Every boundary call is matched, in order,
against the record: port, method and request must equal the recorded ones (the encoding, alias graph
included; opaque values only where the session allows them, hash-only requests by array hash), nothing is
skipped. The recorded answer is then handed back as if the call had run: the arguments the call changed are
changed the same way, in place (``request_after``), the watched core dictionaries get the recorded changes
(``watched``) -- also when the call raised -- and the response is returned or the exception raised again.

A mismatch raises :class:`ReplayMismatch` at the call and is kept by the session: :meth:`ReplaySession.finish`
fails on it even when the replayed code caught the exception. A record that stopped at its size cap, has a
different tick count from its meta, or whose array payload does not match its hash is refused.
"""

from __future__ import annotations

import collections
import dataclasses
import importlib
import json
from pathlib import Path
from typing import Any, Iterator

import numpy as np

from forklift_core.mission.replay_codec import Encoder, decode


class ReplayMismatch(AssertionError):
    """The replayed run asked something the recorded run did not, at this point."""


class Unsupported(ReplayMismatch):
    """A recorded change the replay cannot apply faithfully."""


def check_format(directory: Path) -> dict:
    meta = json.loads((Path(directory) / "boundary_meta.json").read_text())
    if meta.get("format") != "forklift_boundary_record/2":
        raise ValueError(f"not a format-2 boundary record: {meta.get('format')}")
    if meta.get("stopped_at_tick") is not None:
        raise ValueError(f"the record stopped at tick {meta['stopped_at_tick']} ({meta.get('stopped_reason')})")
    return meta


def read_ticks(directory: Path) -> Iterator[dict]:
    """The ticks of a format-2 record in order, as written."""
    directory = Path(directory)
    check_format(directory)
    for path in sorted(directory.glob("boundary_*.jsonl")):
        with path.open() as fh:
            for line in fh:
                if line.strip():
                    yield json.loads(line)


def load_arrays(directory: Path) -> dict[str, np.ndarray]:
    """The array payloads, each checked against its content hash."""
    arrays: dict[str, np.ndarray] = {}
    for path in sorted(Path(directory).glob("arrays_*.npz")):
        with np.load(path, allow_pickle=False) as z:
            for key in z.files:
                array = z[key]
                if Encoder.digest(array) != key:
                    raise ValueError(f"{path.name}: array {key[:12]} does not match its hash")
                arrays[key] = array
    return arrays


def _canonical(node: Any) -> str:
    return json.dumps(node, sort_keys=True, allow_nan=True)


def _has_opaque(node: Any) -> bool:
    if isinstance(node, dict):
        return "$opaque" in node or any(_has_opaque(v) for v in node.values())
    if isinstance(node, list):
        return any(_has_opaque(v) for v in node)
    return False


def _resolve(qualname: str):
    module, _, name = qualname.partition(":")
    obj: Any = importlib.import_module(module)
    for part in name.split("."):
        obj = getattr(obj, part)
    return obj


def apply_inplace(actual: Any, after: Any, where: str = "argument", *, replaced=frozenset(), path: tuple = (),
                  value_types=frozenset()) -> Any:
    """Make ``actual`` equal ``after`` keeping every mutable object the caller may hold elsewhere: lists and
    dictionaries are updated in place element by element (their elements recursively), arrays of the same
    shape and dtype in place, mutable dataclasses and registered value objects attribute by attribute,
    tuples through their mutable elements. At a path in ``replaced`` (the recorded call put another object
    there) the new object is used instead. Returns the object to use; a change that needs a held object
    replaced without the record saying so is refused."""
    if path in replaced:
        return after
    from forklift_core.mission.boundary_record import _immutable

    if _immutable(actual):
        # Nothing held can change in place: an equal value keeps the caller's object, another one replaces it.
        same = _canonical(Encoder(opaque_ok=True, value_types=value_types).encode(actual)) == _canonical(
            Encoder(opaque_ok=True, value_types=value_types).encode(after))
        return actual if same else after

    def sub(a, b, step):
        return apply_inplace(a, b, where, replaced=replaced, path=path + (step,), value_types=value_types)

    if isinstance(actual, list) and isinstance(after, list):
        new = [sub(actual[i], b, i) if i < len(actual) else b for i, b in enumerate(after)]
        actual[:] = new
        return actual
    if isinstance(actual, collections.deque) and isinstance(after, collections.deque):
        if actual.maxlen != after.maxlen:
            raise Unsupported(f"{where}: a deque's maxlen changed")
        new = [sub(actual[i], b, i) if i < len(actual) else b for i, b in enumerate(after)]
        actual.clear()
        actual.extend(new)
        return actual
    if isinstance(actual, dict) and isinstance(after, dict):
        for key in list(actual):
            if key not in after:
                del actual[key]
        for key, value in after.items():
            actual[key] = sub(actual[key], value, repr(key)) if key in actual else value
        return actual
    if isinstance(actual, np.ndarray) and isinstance(after, np.ndarray):
        if actual.shape != after.shape or actual.dtype != after.dtype:
            raise Unsupported(f"{where}: an array changed shape or dtype in place")
        if actual.tobytes() != after.tobytes():  # the exact bytes (-0.0, NaN payloads); unchanged ones untouched
            if not actual.flags.writeable:
                try:
                    actual.setflags(write=True)
                except ValueError:
                    raise Unsupported(f"{where}: a read-only array that does not own its data changed") from None
            actual[...] = after
        if actual.flags.writeable != after.flags.writeable:
            try:
                actual.setflags(write=after.flags.writeable)
            except ValueError:
                raise Unsupported(f"{where}: an array's read-only state cannot be restored") from None
        return actual
    if isinstance(actual, tuple) and isinstance(after, tuple):
        # A tuple cannot change in place: its mutable elements are updated where they are, and if any element
        # (or the length) differs the holder gets a new tuple made of them.
        elements = [sub(actual[i], b, i) if i < len(actual) else b for i, b in enumerate(after)]
        if len(elements) == len(actual) and all(e is a for e, a in zip(elements, actual)):
            return actual
        return tuple(elements)
    if dataclasses.is_dataclass(actual) and not isinstance(actual, type) and type(actual) is type(after):
        for f in dataclasses.fields(actual):
            a, b = getattr(actual, f.name), getattr(after, f.name)
            new = sub(a, b, "." + f.name)
            if new is not a:
                try:
                    setattr(actual, f.name, new)
                except dataclasses.FrozenInstanceError:
                    raise Unsupported(f"{where}: a frozen dataclass field would have to change") from None
        return actual
    if type(actual) is type(after) and type(actual).__name__ in value_types and hasattr(actual, "__dict__"):
        for key in list(vars(actual)):
            if not key.startswith("_") and key not in vars(after):
                delattr(actual, key)
        for key, b in vars(after).items():
            a = getattr(actual, key, None)
            new = sub(a, b, "." + key) if hasattr(actual, key) else b
            if new is not a:
                setattr(actual, key, new)
        return actual
    enc = Encoder(opaque_ok=True, value_types=value_types)
    if _canonical(enc.encode(actual)) == _canonical(Encoder(opaque_ok=True, value_types=value_types).encode(after)):
        return actual  # unchanged
    # Another value of another kind: the recording's change check let it through only if the old object left
    # unchanged (or stays elsewhere), so the holder simply gets the new value.
    return after


class ReplaySession:
    """The recorded ticks, consumed in order by the replayed run. ``allow_opaque``: the (port, method) pairs
    whose recorded request may hold opaque values (callbacks whose request descriptors are checked elsewhere,
    plan section 3.4)."""

    def __init__(self, directory: Path, *, allow_opaque=()) -> None:
        self.directory = Path(directory)
        self.meta = check_format(self.directory)
        self.value_types = set(self.meta.get("value_types") or ())
        self.allow_opaque = set(allow_opaque)
        self.arrays = load_arrays(self.directory)
        self._ticks = read_ticks(self.directory)
        self.tick: dict | None = None
        self._next = 0
        self.ticks_done = 0
        self.failure: str | None = None

    def decode(self, node: Any) -> Any:
        return decode(node, self.arrays)

    def encode(self, value: Any, *, hash_arrays: bool = False, opaque_ok: bool = True) -> Any:
        return Encoder(opaque_ok=opaque_ok, value_types=self.value_types, hash_arrays=hash_arrays).encode(value)

    def fail(self, message: str) -> ReplayMismatch:
        if self.failure is None:
            self.failure = message
        return ReplayMismatch(message)

    # -- ticks ---------------------------------------------------------------------------------------
    @property
    def in_tick(self) -> bool:
        return self.tick is not None

    def tick_begin(self, tick: int, **head) -> None:
        if self.tick is not None:
            raise self.fail("tick_begin inside a tick")
        recorded = next(self._ticks, None)
        if recorded is None:
            raise self.fail(f"tick {tick}: the record has no more ticks")
        if recorded["tick"] != tick:
            raise self.fail(f"tick {tick}: the record's next tick is {recorded['tick']}")
        self._time_pending = False
        for key in ("tick_t", "work_t", "commands", "stop_kind"):
            if key in head and head[key] is None and key in ("tick_t", "work_t"):
                self._time_pending = True  # set later in the tick (tick_time), required before it ends
                continue
            if key in head and _canonical(self.encode(head[key])) != _canonical(recorded[key]):
                raise self.fail(f"tick {tick} {key}: {head[key]!r} differs from the recorded one")
        self.tick, self._next = recorded, 0

    def tick_time(self, *, tick_t: float, work_t: float) -> None:
        """The head's times computed inside the tick, checked against the recorded head."""
        tick = self.tick
        if tick is None:
            raise self.fail("tick_time outside a tick")
        for key, value in (("tick_t", tick_t), ("work_t", work_t)):
            if _canonical(self.encode(value)) != _canonical(tick[key]):
                raise self.fail(f"tick {tick['tick']} {key}: {value!r} differs from the recorded head")
        self._time_pending = False

    def tick_end(self, *, outputs: Any, branches: list[str]) -> None:
        tick = self.tick
        if tick is None:
            raise self.fail("tick_end without tick_begin")
        if getattr(self, "_time_pending", False) and (tick["tick_t"] is not None or tick["work_t"] is not None):
            # Never set here but set in the record. (A tick the recorded run left before its times were known --
            # an exception between the clock read and tick_time -- keeps them unset in the record as well.)
            raise self.fail(f"tick {tick['tick']}: its head times were never set (tick_time)")
        if self._next != len(tick["calls"]):
            left = tick["calls"][self._next]
            raise self.fail(f"tick {tick['tick']}: {len(tick['calls']) - self._next} recorded calls not made, "
                            f"next {left['port']}.{left['method']}")
        try:
            got = self.encode(outputs, opaque_ok=False)
        except TypeError as exc:
            raise self.fail(f"tick {tick['tick']} outputs: {exc}") from None
        if _canonical(got) != _canonical(tick["outputs"]):
            raise self.fail(f"tick {tick['tick']} outputs differ")
        if list(branches) != tick["branches"]:
            raise self.fail(f"tick {tick['tick']} branches {branches} differ from {tick['branches']}")
        self.tick = None
        self.ticks_done += 1

    def finish(self, final_state: Any = None) -> None:
        """``final_state``: the replayed run's core containers at its end, compared with the recorded ones when
        the record has them (``final_state.json``)."""
        recorded = self.directory / "final_state.json"
        if self.failure is None and self.meta.get("final_state"):
            if not recorded.exists():
                raise self.fail("the record says it has a final state but final_state.json is missing")
            if final_state is None:
                raise self.fail("the record has a final state and the replay gave none")
            # Arrays by content hash (the node carries it): exact bytes, NaN included.
            node = Encoder(opaque_ok=True, value_types=self.value_types, hash_arrays=True,
                           identity=False).encode(final_state)
            if _canonical(node) != recorded.read_text().strip() and _canonical(node) != _canonical(
                    json.loads(recorded.read_text())):
                raise self.fail("the replay's final state differs from the recorded one")
        if self.failure is not None:
            raise ReplayMismatch(f"the replay failed earlier: {self.failure}")
        if self.tick is not None:
            raise self.fail(f"the replay ended inside tick {self.tick['tick']}")
        left = next(self._ticks, None)
        if left is not None:
            raise self.fail(f"the record has ticks left, from tick {left['tick']}")
        if self.ticks_done != self.meta.get("ticks"):
            raise self.fail(f"{self.ticks_done} ticks replayed, the meta says {self.meta.get('ticks')}")

    # -- calls ---------------------------------------------------------------------------------------
    def peek(self) -> dict | None:
        if self.tick is None or self._next >= len(self.tick["calls"]):
            return None
        return self.tick["calls"][self._next]

    def _described(self, port: str, method: str) -> bool:
        """The call just before this one, already compared, is a ``Trace.callback_spec`` describing every
        callback of this call: only then may its request hold callbacks by type (plan 3.4)."""
        if self.tick is None or self._next == 0:
            return False
        prev = self.tick["calls"][self._next - 1]
        if (prev["port"], prev["method"]) != ("Trace", "callback_spec"):
            return False
        try:
            descriptor = decode(prev["request"], self.arrays)["args"][0]
        except (KeyError, IndexError, TypeError, ValueError):
            return False
        if not isinstance(descriptor, dict) or descriptor.get("call") != f"{port}.{method}":
            return False
        described = {k for k, v in (descriptor.get("callbacks") or {}).items() if '"unspecified"' not in json.dumps(v)}
        # Every argument holding a value by type only must be one of the described callbacks (Codex R3/R4 P2: one
        # descriptor must not let an undescribed callback through).
        request = self.peek()["request"]
        try:
            args_ = request["$map"]["args"]["$list"]
            kwargs_ = request["$map"]["kwargs"]["$map"]
        except (KeyError, TypeError):
            return False
        opaque_at = {f"#{i}" for i, node in enumerate(args_) if _has_opaque(node)}
        opaque_at |= {k for k, node in kwargs_.items() if _has_opaque(node)}
        return opaque_at <= described

    def peek_at(self, offset: int) -> dict | None:
        if self.tick is None or self._next + offset >= len(self.tick["calls"]):
            return None
        return self.tick["calls"][self._next + offset]

    def take(self, port: str, method: str, request_node: Any, *, drop_recorded_kwargs=()) -> dict:
        """``drop_recorded_kwargs``: recorded keyword arguments left out of the comparison (only for the one
        allowed source mapping, :meth:`ReplayGate._baseline_redraw`)."""
        entry = self.peek()
        where = f"tick {self.tick['tick'] if self.tick else None} call {self._next}"
        if entry is None:
            raise self.fail(f"{where}: {port}.{method} was not recorded here (no more calls)")
        if (entry["port"], entry["method"]) != (port, method):
            raise self.fail(f"{where}: {port}.{method} where {entry['port']}.{entry['method']} was recorded")
        recorded_request = entry["request"]
        if drop_recorded_kwargs:
            recorded_request = json.loads(json.dumps(recorded_request))
            kwargs_ = recorded_request["$map"]["kwargs"]["$map"]
            for key in drop_recorded_kwargs:
                kwargs_.pop(key, None)
        if _has_opaque(recorded_request) and (port, method) not in self.allow_opaque \
                and not self._described(port, method):
            raise self.fail(f"{where}: {port}.{method} was recorded with an opaque request value")
        got, want = _canonical(request_node), _canonical(recorded_request)
        if got != want:
            at = next((i for i, (a, b) in enumerate(zip(got, want)) if a != b), min(len(got), len(want)))
            raise self.fail(f"{where}: {port}.{method} request differs from the recorded one "
                            f"(at {at}: got ...{got[max(0, at - 80):at + 120]}... recorded ...{want[max(0, at - 80):at + 120]}...)")
        if _has_opaque(entry.get("response")) or _has_opaque(entry.get("exception")):
            raise self.fail(f"{where}: {port}.{method} answered with an opaque value (cannot be handed back)")
        self._next += 1
        return entry


class ReplayGate:
    """In place of ``RecordingGate``: calls inside an open tick are answered from the record; outside a
    tick (or nested inside an answered call, which never runs) nothing is replayed -- the rule the recording
    gate applied."""

    def __init__(self, session: ReplaySession) -> None:
        self.session = session
        self.depth = 0

    @property
    def active(self) -> bool:
        return self.depth == 0 and self.session.in_tick

    value_format = 2  # a replay reads format-2 records only

    def run(self, port: str, method: str, fn, args: tuple, kwargs: dict, *, hash_only: bool = False,
            watch=None):
        if not self.active:
            return fn(*args, **kwargs)
        if self._baseline_bundle_ahead(port, method, args):
            if method == "callback_spec":  # the descriptor names the call the baseline made
                descriptor = dict(args[0], call="Planner.redraw_docking")
                return self.answer(self.session.take(port, method, self.session.encode(
                    {"args": [descriptor], "kwargs": dict(kwargs)})), (descriptor,), kwargs, watch)
            return self._baseline_redraw(args, kwargs, hash_only, watch)
        raw = {"args": list(args), "kwargs": dict(kwargs)}
        entry = self.session.take(port, method, self.session.encode(raw, hash_arrays=hash_only))
        return self.answer(entry, args, kwargs, watch)

    # -- the one source mapping (plan section 4): the baseline's stage-1 redraw bundle ---------------------------
    def _baseline_bundle_ahead(self, port: str, method: str, args: tuple) -> bool:
        """The moved code's ``redraw_docking_traced`` (and its descriptor) where the baseline recorded
        ``Trace.callback_spec`` + ``Planner.redraw_docking`` + ``Trace.redraw_rollouts``."""
        session = self.session
        if (port, method) == ("Trace", "callback_spec"):
            if not (args and isinstance(args[0], dict) and args[0].get("call") == "Planner.redraw_docking_traced"):
                return False
            ahead = [session.peek_at(i) for i in range(3)]
        elif (port, method) == ("Planner", "redraw_docking_traced"):
            ahead = [None, session.peek_at(0), session.peek_at(1)]
        else:
            return False
        names = [None if e is None else (e["port"], e["method"]) for e in ahead]
        return names[1:] == [("Planner", "redraw_docking"), ("Trace", "redraw_rollouts")] and (
            method != "callback_spec" or names[0] == ("Trace", "callback_spec"))

    def _baseline_redraw(self, args: tuple, kwargs: dict, hash_only: bool, watch):
        """redraw_docking_traced answered from the baseline bundle: the request without the dry-run inputs (their
        descriptor was compared just before) against redraw_docking's without its callbacks; the answer is
        (plans, record) with the rollouts trace's trajectories; a recorded exception is raised after the trace is
        consumed (the effects before it applied)."""
        session = self.session
        call_kwargs = {k: v for k, v in kwargs.items() if k not in ("start_rear", "dry_configs")}
        request = session.encode({"args": list(args), "kwargs": call_kwargs}, hash_arrays=hash_only)
        entry = session.take("Planner", "redraw_docking", request, drop_recorded_kwargs=("dry_runs",))
        try:
            answer = self.answer(entry, args, call_kwargs, watch)
        finally:
            rollouts = session.peek()
            if rollouts is not None and (rollouts["port"], rollouts["method"]) == ("Trace", "redraw_rollouts"):
                session._next += 1
            elif session.failure is None:
                raise session.fail("the baseline redraw bundle has no Trace.redraw_rollouts after its Planner call")
        trajectories = session.decode(rollouts["request"])["args"][0]
        plans, record = answer
        return plans, record, list(trajectories)

    def answer(self, entry: dict, args: tuple = (), kwargs: dict | None = None, watch=None):
        session = self.session
        where = f"{entry['port']}.{entry['method']}"
        changes = entry.get("changes") or {}
        if changes.get("response_aliases_request"):
            raise session.fail(f"{where}: the response handed back an argument's own object (not replayable)")
        if changes.get("unsupported"):
            raise session.fail(f"{where}: a change outside the replay's model was recorded: {changes['unsupported']}")
        try:
            if "request_after" in changes:
                after = session.decode(changes["request_after"])
                actual = {"args": list(args), "kwargs": dict(kwargs or {})}
                replaced = frozenset(tuple(p) for p in changes.get("replaced_paths") or ())
                apply_inplace(actual, after, f"{where} arguments", replaced=replaced,
                              value_types=frozenset(session.value_types))
            for name, diff in (changes.get("watched") or {}).items():
                target = None if watch is None else watch.dicts.get(name)
                if target is None:
                    raise session.fail(f"{where}: recorded changes to {name} but no such watched dictionary here")
                for key, change in diff.items():
                    if change.get("deleted"):
                        target.pop(key, None)
                    elif "append" in change:
                        target.setdefault(key, []).extend(session.decode(change["append"]))
                    elif key not in target:
                        target[key] = session.decode(change["value"])
                    else:
                        paths = frozenset(tuple(q) for q in change.get("replaced_paths") or ())
                        target[key] = apply_inplace(target[key], session.decode(change["value"]),
                                                    f"{where} {name}.{key}", replaced=paths,
                                                    value_types=frozenset(session.value_types))
        except ReplayMismatch as exc:
            raise session.fail(str(exc)) from None
        except Exception as exc:  # restoring the record failed: never a service exception the caller may catch
            raise session.fail(f"{where}: restoring the recorded changes failed: {type(exc).__name__}: {exc}") from None
        if "exception" in entry:
            try:
                exc = self._exception(entry["exception"])
            except ReplayMismatch:
                raise
            except Exception as err:
                raise session.fail(f"{where}: restoring the recorded exception failed: {type(err).__name__}: {err}") from None
            raise exc
        try:
            return session.decode(entry["response"])
        except Exception as exc:
            raise session.fail(f"{where}: restoring the recorded response failed: {type(exc).__name__}: {exc}") from None

    def _exception(self, record: dict) -> BaseException:
        if ":" not in record["type"]:
            raise self.session.fail(f"exception type {record['type']} is not qualified (format 1?)")
        cls = _resolve(record["type"])
        args = tuple(self.session.decode(record["args"])) if "args" in record else (record["message"],)
        if "os_error" in record:  # errno, strerror, filename, winerror, filename2
            args = tuple(self.session.decode(record["os_error"]))
        try:
            exc = cls(*args)
        except Exception:  # a constructor with another signature: rebuild without it
            exc = cls.__new__(cls)
            exc.args = args
        if "attributes" in record:
            for key, value in self.session.decode(record["attributes"]).items():
                try:
                    setattr(exc, key, value)
                except AttributeError:
                    pass  # read-only native attributes come back through args
        return exc


class ReplayProxy:
    """In place of a ``RecordingProxy``. ``spec`` is the service's class: a name that is a method on it is a
    call, anything else an attribute read -- an attribute read that is not the next recorded entry fails at
    once. ``child_ports`` as for the recording proxy; a child recorded as ``None`` (read with ``{"get":
    True}``) comes back as ``None``."""

    def __init__(self, port: str, gate: ReplayGate, *, spec: type, child_ports=None, child_specs=None,
                 hash_only_requests=(), watches=None, returns=None) -> None:
        object.__setattr__(self, "_returns", dict(returns or {}))
        object.__setattr__(self, "_port", port)
        object.__setattr__(self, "_gate", gate)
        object.__setattr__(self, "_spec", spec)
        object.__setattr__(self, "_child_ports", dict(child_ports or {}))
        object.__setattr__(self, "_child_specs", dict(child_specs or {}))
        object.__setattr__(self, "_children", {})
        object.__setattr__(self, "_hash_only", frozenset(hash_only_requests))
        object.__setattr__(self, "_watches", dict(watches or {}))

    def _attribute_entry(self, name: str) -> dict | None:
        gate = object.__getattribute__(self, "_gate")
        entry = gate.session.peek() if gate.active else None
        if entry is not None and (entry["port"], entry["method"]) == (object.__getattribute__(self, "_port"), name) \
                and _canonical(entry["request"]) == _canonical(gate.session.encode({"get": True})):
            return entry
        return None

    def __getattr__(self, name: str):
        gate = object.__getattribute__(self, "_gate")
        port = object.__getattribute__(self, "_port")
        session = gate.session
        child_port = object.__getattribute__(self, "_child_ports").get(name)
        if child_port is not None:
            entry = self._attribute_entry(name)
            if entry is not None:  # the child was None when recorded
                session.take(port, name, entry["request"])
                return gate.answer(entry)
            children = object.__getattribute__(self, "_children")
            if name not in children:
                spec = object.__getattribute__(self, "_child_specs").get(name, object)
                children[name] = ReplayProxy(child_port, gate, spec=spec)
            return children[name]
        spec = object.__getattribute__(self, "_spec")
        if isinstance(spec, dict):  # a recorded spec (a class the replay cannot import): method names
            is_method = name in spec.get("methods", ())
        else:
            static = None
            for klass in getattr(spec, "__mro__", ()):
                if name in vars(klass):
                    static = vars(klass)[name]
                    break
            is_method = isinstance(static, (staticmethod, classmethod)) or (
                callable(static) and not isinstance(static, property))
        if not is_method:
            if not gate.active:
                raise session.fail(f"{port}.{name} read outside a recorded tick (no service to read)")
            entry = self._attribute_entry(name)
            if entry is None:
                nxt = session.peek() or {}
                raise session.fail(f"{port}.{name} read where {nxt.get('port')}.{nxt.get('method')} was recorded")
            session.take(port, name, entry["request"])
            return gate.answer(entry)
        hash_only = name in object.__getattribute__(self, "_hash_only")
        watch = object.__getattribute__(self, "_watches").get(name)

        returned_port = object.__getattribute__(self, "_returns").get(name)

        def method(*args, **kwargs):
            def unreachable(*a, **k):
                raise session.fail(f"{port}.{name} called outside a recorded tick (no service to run)")
            answer = gate.run(port, name, unreachable, args, kwargs, hash_only=hash_only, watch=watch)
            if returned_port is None:
                return answer
            from forklift_core.mission.boundary_record import ServiceRef

            if answer is None:
                return None
            if not isinstance(answer, ServiceRef) or answer.port != returned_port:
                raise session.fail(f"{port}.{name} was to hand back a {returned_port} service, recorded {answer!r}")
            return ReplayProxy(answer.port, gate, spec={"methods": list(answer.methods)})

        return method

    def __setattr__(self, name: str, value) -> None:
        gate = object.__getattribute__(self, "_gate")
        if gate.active:
            gate.session.take(object.__getattribute__(self, "_port"), name, gate.session.encode({"set": value}))
