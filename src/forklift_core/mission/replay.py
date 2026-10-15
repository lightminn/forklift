"""Replaying a format-2 boundary record (docs/plans/2026-10-16-m-a1-replay.md, steps R1/R4).

The replayed code runs with :class:`ReplayGate` where the recorded run had a ``RecordingGate`` and with
:class:`ReplayProxy` where it had ``RecordingProxy`` objects. Every boundary call is matched, in order,
against the record: port, method and request must equal the recorded ones (opaque values by their type,
hash-only requests by hash), nothing is skipped. The recorded answer is then handed back as if the call had
run: the arguments the call changed are changed the same way (``request_after``), the watched core
dictionaries get the recorded changes (``watched``) -- also when the call raised -- and the response is
returned or the exception raised again. A mismatch raises :class:`ReplayMismatch` at the call.
"""

from __future__ import annotations

import importlib
import json
from pathlib import Path
from typing import Any, Iterator

import numpy as np

from forklift_core.mission.replay_codec import Encoder, decode


class ReplayMismatch(AssertionError):
    """The replayed run asked something the recorded run did not, at this point."""


def check_format(directory: Path) -> dict:
    meta = json.loads((Path(directory) / "boundary_meta.json").read_text())
    if meta.get("format") != "forklift_boundary_record/2":
        raise ValueError(f"not a format-2 boundary record: {meta.get('format')}")
    return meta


def read_ticks(directory: Path) -> Iterator[dict]:
    """The ticks of a format-2 record in order, as written (decode values with :meth:`ReplaySession.decode`)."""
    directory = Path(directory)
    check_format(directory)
    for path in sorted(directory.glob("boundary_*.jsonl")):
        with path.open() as fh:
            for line in fh:
                if line.strip():
                    yield json.loads(line)


def load_arrays(directory: Path) -> dict[str, np.ndarray]:
    arrays: dict[str, np.ndarray] = {}
    for path in sorted(Path(directory).glob("arrays_*.npz")):
        with np.load(path, allow_pickle=False) as z:
            arrays.update({k: z[k] for k in z.files})
    return arrays


def _canonical(node: Any) -> str:
    """A request's comparison form: the encoding, with alias ids dropped (a fresh encoding numbers them
    the same way only if the same objects alias the same way, which the request compare checks anyway
    through the structure)."""
    def strip(n):
        if isinstance(n, dict):
            return {k: strip(v) for k, v in n.items() if k != "$id"}
        if isinstance(n, list):
            return [strip(v) for v in n]
        return n
    return json.dumps(strip(node), sort_keys=True, allow_nan=True)


def _resolve(qualname: str):
    module, _, name = qualname.partition(":")
    obj: Any = importlib.import_module(module)
    for part in name.split("."):
        obj = getattr(obj, part)
    return obj


def _apply_inplace(actual: Any, after: Any) -> Any:
    """Make ``actual`` (a caller's argument) look like ``after`` in place where it is mutable; returns
    the value to use."""
    if isinstance(actual, list) and isinstance(after, list):
        actual[:] = after
        return actual
    if isinstance(actual, dict) and isinstance(after, dict):
        actual.clear()
        actual.update(after)
        return actual
    return after


class ReplaySession:
    """The recorded ticks, consumed in order by the replayed run."""

    def __init__(self, directory: Path) -> None:
        self.directory = Path(directory)
        self.meta = check_format(self.directory)
        self.arrays = load_arrays(self.directory)
        self._ticks = read_ticks(self.directory)
        self.tick: dict | None = None
        self._next = 0
        self.ticks_done = 0

    def decode(self, node: Any) -> Any:
        return decode(node, self.arrays)

    # -- ticks ---------------------------------------------------------------------------------------
    @property
    def in_tick(self) -> bool:
        return self.tick is not None

    def tick_begin(self, tick: int, **head) -> None:
        if self.tick is not None:
            raise ReplayMismatch("tick_begin inside a tick")
        recorded = next(self._ticks, None)
        if recorded is None:
            raise ReplayMismatch(f"tick {tick}: the record has no more ticks")
        if recorded["tick"] != tick:
            raise ReplayMismatch(f"tick {tick}: the record's next tick is {recorded['tick']}")
        for key in ("tick_t", "work_t", "stop_kind"):
            if key in head:
                got = _canonical(Encoder(opaque_ok=True).encode(head[key]))
                if got != _canonical(recorded[key]):
                    raise ReplayMismatch(f"tick {tick} {key}: {head[key]!r} != recorded {self.decode(recorded[key])!r}")
        self.tick, self._next = recorded, 0

    def tick_end(self, *, outputs: Any, branches: list[str]) -> None:
        tick = self.tick
        if tick is None:
            raise ReplayMismatch("tick_end without tick_begin")
        if self._next != len(tick["calls"]):
            left = tick["calls"][self._next]
            raise ReplayMismatch(f"tick {tick['tick']}: {len(tick['calls']) - self._next} recorded calls not made, "
                                 f"next {left['port']}.{left['method']}")
        got = _canonical(Encoder(opaque_ok=True).encode(outputs))
        if got != _canonical(tick["outputs"]):
            raise ReplayMismatch(f"tick {tick['tick']} outputs differ")
        if list(branches) != tick["branches"]:
            raise ReplayMismatch(f"tick {tick['tick']} branches {branches} != recorded {tick['branches']}")
        self.tick = None
        self.ticks_done += 1

    def finish(self) -> None:
        if self.tick is not None:
            raise ReplayMismatch(f"the replay ended inside tick {self.tick['tick']}")
        left = next(self._ticks, None)
        if left is not None:
            raise ReplayMismatch(f"the record has ticks left, from tick {left['tick']}")

    # -- calls ---------------------------------------------------------------------------------------
    def peek(self) -> dict | None:
        if self.tick is None or self._next >= len(self.tick["calls"]):
            return None
        return self.tick["calls"][self._next]

    def take(self, port: str, method: str, request_node: Any) -> dict:
        entry = self.peek()
        where = f"tick {self.tick['tick'] if self.tick else None} call {self._next}"
        if entry is None:
            raise ReplayMismatch(f"{where}: {port}.{method} was not recorded here (no more calls)")
        if (entry["port"], entry["method"]) != (port, method):
            raise ReplayMismatch(f"{where}: {port}.{method} != recorded {entry['port']}.{entry['method']}")
        if _canonical(request_node) != _canonical(entry["request"]):
            raise ReplayMismatch(f"{where}: {port}.{method} request differs from the recorded one")
        self._next += 1
        return entry


class ReplayGate:
    """In place of ``RecordingGate``: calls inside an open tick are answered from the record; outside a
    tick (or nested inside an answered call, which never runs) nothing is recorded and nothing replayed --
    the same rule the recording gate applied."""

    def __init__(self, session: ReplaySession) -> None:
        self.session = session
        self.depth = 0

    @property
    def active(self) -> bool:
        return self.depth == 0 and self.session.in_tick

    def run(self, port: str, method: str, fn, args: tuple, kwargs: dict, *, hash_only: bool = False,
            watch=None):
        if not self.active:
            return fn(*args, **kwargs)
        from forklift_core.mission.boundary_record import _hash_only

        raw = {"args": list(args), "kwargs": dict(kwargs)}
        entry = self.session.take(port, method, Encoder(opaque_ok=True).encode(_hash_only(raw) if hash_only else raw))
        return self.answer(entry, args, kwargs, watch)

    def answer(self, entry: dict, args: tuple = (), kwargs: dict | None = None, watch=None):
        changes = self.session.decode(entry["changes"]) if entry.get("changes") is not None else {}
        if "request_after" in changes:
            after = changes["request_after"]
            for i, value in enumerate(after.get("args", [])):
                if i < len(args):
                    _apply_inplace(args[i], value)
            for key, value in (after.get("kwargs") or {}).items():
                if kwargs is not None and key in kwargs:
                    _apply_inplace(kwargs[key], value)
        if "watched" in changes:
            if watch is None:
                raise ReplayMismatch(f"{entry['port']}.{entry['method']}: recorded watched changes but no watch here")
            for name, diff in changes["watched"].items():
                target = watch.dicts.get(name)
                if target is None:
                    raise ReplayMismatch(f"{entry['port']}.{entry['method']}: no watched dictionary {name}")
                for key, value in diff.items():
                    if isinstance(value, dict) and value.get("$deleted") is True and len(value) == 1:
                        target.pop(key, None)
                    elif isinstance(value, dict) and set(value) == {"$append"}:
                        target.setdefault(key, []).extend(value["$append"])
                    else:
                        target[key] = value
        if "exception" in entry:
            raise self._exception(entry["exception"])
        return self.session.decode(entry["response"])

    def _exception(self, record: dict) -> BaseException:
        cls = _resolve(record["type"]) if ":" in record["type"] else RuntimeError
        exc = cls.__new__(cls)
        BaseException.__init__(exc, record["message"])
        for key, value in self.session.decode(record.get("attributes") or {}).items():
            setattr(exc, key, value)
        return exc


class ReplayProxy:
    """In place of a ``RecordingProxy``: attribute reads and method calls answered from the record.
    Whether a name is an attribute or a method is what the record says (an attribute read is recorded with
    the request ``{"get": True}``)."""

    def __init__(self, port: str, gate: ReplayGate, *, child_ports=None, hash_only_requests=(), watches=None) -> None:
        object.__setattr__(self, "_port", port)
        object.__setattr__(self, "_gate", gate)
        object.__setattr__(self, "_child_ports", dict(child_ports or {}))
        object.__setattr__(self, "_children", {})
        object.__setattr__(self, "_hash_only", frozenset(hash_only_requests))
        object.__setattr__(self, "_watches", dict(watches or {}))

    def __getattr__(self, name: str):
        gate = object.__getattribute__(self, "_gate")
        port = object.__getattribute__(self, "_port")
        child_port = object.__getattribute__(self, "_child_ports").get(name)
        if child_port is not None:
            children = object.__getattribute__(self, "_children")
            if name not in children:
                children[name] = ReplayProxy(child_port, gate)
            return children[name]
        entry = gate.session.peek() if gate.active else None
        if entry is not None and (entry["port"], entry["method"]) == (port, name) \
                and _canonical(entry["request"]) == _canonical(Encoder().encode({"get": True})):
            gate.session.take(port, name, entry["request"])
            return gate.answer(entry)
        hash_only = name in object.__getattribute__(self, "_hash_only")
        watch = object.__getattribute__(self, "_watches").get(name)

        def method(*args, **kwargs):
            def unreachable(*a, **k):
                raise ReplayMismatch(f"{port}.{name} called outside a recorded tick (no service to run)")
            return gate.run(port, name, unreachable, args, kwargs, hash_only=hash_only, watch=watch)

        return method

    def __setattr__(self, name: str, value) -> None:
        gate = object.__getattribute__(self, "_gate")
        if gate.active:
            gate.session.take(object.__getattribute__(self, "_port"), name,
                              Encoder(opaque_ok=True).encode({"set": value}))
