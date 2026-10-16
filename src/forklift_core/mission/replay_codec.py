"""The value codec of the M-a1 replay (docs/plans/2026-10-16-m-a1-replay.md, step R1).

Boundary records of format 1 (``boundary_record.py``) keep values well enough to compare them, not to hand
them back: tuples become lists, a dataclass keeps its bare class name, dictionary keys become strings and
two references to one list become two lists. A replay hands recorded values back to code that may unpack
a tuple, test ``isinstance`` or append to a list another holder also sees, so the setup snapshot and the
format-2 records use this codec instead:

- arrays by content hash into a side table (``{"$array": key, "dtype", "shape"}``);
- ``{"$tuple": [...]}``; ``{"$dataclass": "module:Qualname", "fields": {...}}``; non-finite floats as
  ``{"$float": "nan"}``; dictionaries whose keys are not all strings as ``{"$dict": [[k, v], ...]}``;
- every list, dictionary, array, dataclass and registered value object (``value_types``) met twice in one
  encoding is written once with ``"$id"`` and referred to by ``{"$ref": id}`` afterwards (the alias graph,
  by object identity -- content-equal arrays stay distinct objects);
- registered value objects as ``{"$object": "module:Qualname", "attrs": {...}}``; with ``hash_arrays`` an
  array is only its hash (``{"$sha256": ...}``, comparison-only) and every other type is kept;
- an object with no value encoding raises -- the snapshot must not keep opaque values (a replay could not
  hand them back); callers pass ``opaque_ok=True`` only for comparison-only payloads, where it is written as
  ``{"$opaque": "module:Qualname"}``.

Decoding rebuilds tuples, dataclasses (imported by their qualified name; a missing class raises), the
alias graph and non-string keys.
"""

from __future__ import annotations

import base64
import collections
import dataclasses
import pathlib
import hashlib
import importlib
import json
import math
from typing import Any

import numpy as np

FORMAT = "forklift_replay_value/1"


def _qualname(cls: type) -> str:
    return f"{cls.__module__}:{cls.__qualname__}"


def _resolve(qualname: str) -> type:
    module, _, name = qualname.partition(":")
    obj: Any = importlib.import_module(module)
    for part in name.split("."):
        obj = getattr(obj, part)
    return obj


_PLAIN_TYPES = (type(None), bool, int, float, str, bytes)


def plain_value(value: Any) -> bool:
    """A scalar this codec encodes by value (shared with the recorder's change check): exactly a built-in
    scalar, a numpy scalar of an encoded kind or a pathlib path -- not a subclass that may carry more state."""
    t = type(value)
    if t in _PLAIN_TYPES:
        return True
    if isinstance(value, np.generic) and t.__module__ == "numpy":
        return value.dtype.kind in "biufcmMSU"
    return t.__module__ == "pathlib" and isinstance(value, pathlib.PurePath)


def immutable(value: Any, depth: int = 0) -> bool:
    """A value no one can change in place: plain scalars, and tuples / frozen dataclasses made only of such
    values. Its identity carries no meaning, so neither the change model nor the alias graph tracks it."""
    if depth > 20:
        return False
    if plain_value(value):
        return True
    if type(value) is tuple:
        return all(immutable(v, depth + 1) for v in value)
    if dataclasses.is_dataclass(value) and not isinstance(value, type) and value.__dataclass_params__.frozen \
            and not getattr(value, "__dict__", {}).keys() - {f.name for f in dataclasses.fields(value)}:
        return all(immutable(getattr(value, f.name), depth + 1) for f in dataclasses.fields(value))
    return False


class Encoder:
    """One encoding scope: the array table and the alias graph are shared by everything it encodes.

    ``value_types``: class names (bare, as the recorder registers them) of plain objects kept by their instance
    attributes (``{"$object": qualname, "attrs": {...}}``). ``hash_arrays``: arrays as their content hash only
    (``{"$sha256": ...}``, nothing stored) -- for large requests compared by hash; every other type and the
    alias graph are kept as usual."""

    def __init__(self, *, opaque_ok: bool = False, value_types=(), hash_arrays: bool = False,
                 identity: bool = True, services=None, full_types=()) -> None:
        self.arrays: dict[str, np.ndarray] = {}
        self.opaque_ok = opaque_ok
        self.value_types = set(value_types)
        self.hash_arrays = hash_arrays
        # identity=False: a pure value encoding (no $id/$ref; an object met twice is written twice) -- for value
        # comparisons such as a run's final state, where object identity was already checked call by call.
        self.identity = identity
        # Live service objects by identity -> their logical id, written as {"$service": id} (a replay supplies its
        # own object for that id); checked before any other kind.
        self.services = dict(services or {})
        # Value types kept with their private attributes too (a setup snapshot rebuilding a helper object whole).
        self.full_types = set(full_types)
        self._stack: set[int] = set()
        self._seen: dict[int, list] = {}  # id(obj) -> [obj, encoded node, assigned id or None]
        self._next_id = 0

    @staticmethod
    def digest(array: np.ndarray) -> str:
        array = np.ascontiguousarray(array)
        digest = hashlib.sha256()
        digest.update(str(array.dtype).encode())
        digest.update(repr(array.shape).encode())
        digest.update(array.tobytes())
        return digest.hexdigest()

    def array_key(self, value: np.ndarray) -> dict:
        array = np.ascontiguousarray(value).copy()
        key = self.digest(array)
        if self.hash_arrays:
            node = {"$sha256": key, "dtype": str(array.dtype), "shape": list(array.shape)}
            if not value.flags.writeable:
                node["readonly"] = True
            return node
        self.arrays.setdefault(key, array)
        node = {"$array": key, "dtype": str(array.dtype), "shape": list(array.shape)}
        if not value.flags.writeable:
            node["readonly"] = True
        return node

    def _shared(self, value: Any, build) -> dict:
        """A node for an object whose identity matters (mutable containers, arrays, dataclasses, value
        objects): written once, referred to by ``{"$ref": id}`` when met again."""
        if not self.identity:
            if id(value) in self._stack:
                return {"$cycle": True}
            self._stack.add(id(value))
            try:
                return build()
            finally:
                self._stack.discard(id(value))
        seen = self._seen.get(id(value))
        if seen is not None:
            if seen[2] is None:  # second reference: name the first one
                seen[2] = self._next_id
                self._next_id += 1
                seen[1]["$id"] = seen[2]
            return {"$ref": seen[2]}
        node: dict = {}
        self._seen[id(value)] = [value, node, None]  # keeps value alive: ids stay unique in the scope
        node.update(build())
        return node

    def encode(self, value: Any) -> Any:
        if self.services and id(value) in self.services:
            return {"$service": self.services[id(value)]}
        if isinstance(value, pathlib.PurePath):
            return {"$path": str(value), "kind": type(value).__name__}
        if isinstance(value, np.str_):  # a str subclass: its own text, NULs and emptiness included
            return {"$np_str": str.__str__(value)}  # str(np.str_) drops trailing NULs
        if isinstance(value, np.bytes_):
            return {"$np_bytes": base64.b64encode(bytes(value)).decode("ascii")}
        if isinstance(value, np.generic) and value.dtype.kind in "biufcmM":  # numpy scalars: dtype + exact bytes
            return {"$np": value.dtype.str, "b": base64.b64encode(value.tobytes()).decode("ascii")}
        if isinstance(value, np.ndarray):
            return self._shared(value, lambda: self.array_key(value))
        if isinstance(value, float) and not math.isfinite(value):
            return {"$float": repr(value)}
        if type(value) in (type(None), bool, int, float, str):
            return value
        if type(value) is bytes:
            return {"$bytes": base64.b64encode(value).decode("ascii")}
        if isinstance(value, tuple):
            return {"$tuple": [self.encode(v) for v in value]}
        if dataclasses.is_dataclass(value) and not isinstance(value, type) and immutable(value):
            return {"$dataclass": _qualname(type(value)),  # a value: no identity (sharing it is harmless)
                    "fields": {f.name: self.encode(getattr(value, f.name)) for f in dataclasses.fields(value)}}
        if dataclasses.is_dataclass(value) and not isinstance(value, type):
            return self._shared(value, lambda: {
                "$dataclass": _qualname(type(value)),
                "fields": {f.name: self.encode(getattr(value, f.name)) for f in dataclasses.fields(value)}})
        if isinstance(value, list):
            return self._shared(value, lambda: {"$list": [self.encode(v) for v in value]})
        if isinstance(value, collections.deque):
            return self._shared(value, lambda: {"$deque": [self.encode(v) for v in value], "maxlen": value.maxlen})
        if type(value) in (set, frozenset):  # members are hashable values: in a fixed order (by their encoding)
            members = sorted((self.encode(v) for v in value), key=lambda n: json.dumps(n, sort_keys=True))
            if type(value) is frozenset:
                return {"$frozenset": members}
            return self._shared(value, lambda: {"$set": members})
        if isinstance(value, dict):
            if all(isinstance(k, str) for k in value):
                return self._shared(value, lambda: {"$map": {k: self.encode(v) for k, v in value.items()}})
            return self._shared(value, lambda: {"$dict": [[self.encode(k), self.encode(v)] for k, v in value.items()]})
        if type(value).__name__ in (self.value_types | self.full_types) and hasattr(value, "__dict__"):
            full = type(value).__name__ in self.full_types
            return self._shared(value, lambda: {
                "$object": _qualname(type(value)),
                "attrs": {k: self.encode(v) for k, v in vars(value).items() if full or not k.startswith("_")}})
        if self.opaque_ok:
            return {"$opaque": _qualname(type(value))}
        raise TypeError(f"no value encoding for a {_qualname(type(value))}")


def encode(value: Any, *, opaque_ok: bool = False, value_types=()) -> tuple[Any, dict[str, np.ndarray]]:
    """``value`` encoded in a scope of its own: (encoded, arrays)."""
    encoder = Encoder(opaque_ok=opaque_ok, value_types=value_types)
    return encoder.encode(value), encoder.arrays


def decode(node: Any, arrays: dict[str, np.ndarray], services=None, *, by_id: dict | None = None) -> Any:
    """The inverse of :func:`encode`; comparison-only payloads (``$opaque``, ``$sha256``) come back as their
    marker dictionaries. A reference to an object that is still being built (a cycle through a dataclass or a
    value object) is refused."""
    # ``by_id``: share it across several decodes of nodes one Encoder wrote in sequence (their alias ids are
    # one numbering), decoding them in the order they were encoded.
    by_id = {} if by_id is None else by_id
    building = object()

    def walk(n: Any) -> Any:
        if isinstance(n, list):  # only inside our own containers (never a top-level encoding)
            return [walk(v) for v in n]
        if not isinstance(n, dict):
            return n
        if "$service" in n:
            if services is None or n["$service"] not in services:
                raise ValueError(f"no service supplied for {n['$service']}")
            return services[n["$service"]]
        if "$ref" in n:
            obj = by_id[n["$ref"]]
            if obj is building:
                raise ValueError("a reference cycle through an immutable-built object cannot be restored")
            return obj
        ident = n.get("$id")
        if "$list" in n:
            out: list = []
            if ident is not None:
                by_id[ident] = out
            out.extend(walk(v) for v in n["$list"])
            return out
        if "$frozenset" in n:
            return frozenset(walk(v) for v in n["$frozenset"])
        if "$set" in n:
            out_s: set = set()
            if ident is not None:
                by_id[ident] = out_s
            out_s.update(walk(v) for v in n["$set"])
            return out_s
        if "$deque" in n:
            out_q: collections.deque = collections.deque(maxlen=n.get("maxlen"))
            if ident is not None:
                by_id[ident] = out_q
            out_q.extend(walk(v) for v in n["$deque"])
            return out_q
        if "$map" in n or "$dict" in n:
            out_d: dict = {}
            if ident is not None:
                by_id[ident] = out_d
            items = n["$map"].items() if "$map" in n else ((walk(k), v) for k, v in n["$dict"])
            for k, v in items:
                out_d[k] = walk(v)
            return out_d
        if ident is not None:
            by_id[ident] = building
        obj = build(n)
        if ident is not None:
            by_id[ident] = obj
        return obj

    def build(n: dict) -> Any:
        if "$array" in n:
            array = arrays[n["$array"]].copy()
            if n.get("readonly"):
                array.setflags(write=False)
            return array
        if "$float" in n:
            return float(n["$float"])
        if "$path" in n:
            return getattr(pathlib, n.get("kind", "Path"))(n["$path"])
        if "$np_str" in n:
            return np.str_(n["$np_str"])
        if "$np_bytes" in n:
            return np.bytes_(base64.b64decode(n["$np_bytes"]))
        if "$np" in n:
            return np.frombuffer(base64.b64decode(n["b"]), dtype=np.dtype(n["$np"]))[0].copy()
        if "$bytes" in n:
            return base64.b64decode(n["$bytes"])
        if "$tuple" in n:
            return tuple(walk(v) for v in n["$tuple"])
        if "$dataclass" in n:
            # Built through __init__ (so __post_init__ derives whatever it derives), then every field set back to
            # the restored object itself: __post_init__ may copy or normalise a field (an array copied) and break
            # the alias graph.
            cls = _resolve(n["$dataclass"])
            fields = {k: walk(v) for k, v in n["fields"].items()}
            try:
                obj = cls(**{f.name: fields[f.name] for f in dataclasses.fields(cls) if f.init and f.name in fields})
            except Exception:
                obj = cls.__new__(cls)
            for k, v in fields.items():
                object.__setattr__(obj, k, v)
            return obj
        if "$object" in n:
            cls = _resolve(n["$object"])
            obj = cls.__new__(cls)
            for k, v in n["attrs"].items():
                object.__setattr__(obj, k, walk(v))
            return obj
        if "$opaque" in n or "$sha256" in n:
            return {k: v for k, v in n.items() if k != "$id"}
        raise ValueError(f"unknown node {sorted(n)}")

    return walk(node)
