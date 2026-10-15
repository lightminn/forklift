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

import dataclasses
import hashlib
import importlib
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


class Encoder:
    """One encoding scope: the array table and the alias graph are shared by everything it encodes.

    ``value_types``: class names (bare, as the recorder registers them) of plain objects kept by their instance
    attributes (``{"$object": qualname, "attrs": {...}}``). ``hash_arrays``: arrays as their content hash only
    (``{"$sha256": ...}``, nothing stored) -- for large requests compared by hash; every other type and the
    alias graph are kept as usual."""

    def __init__(self, *, opaque_ok: bool = False, value_types=(), hash_arrays: bool = False) -> None:
        self.arrays: dict[str, np.ndarray] = {}
        self.opaque_ok = opaque_ok
        self.value_types = set(value_types)
        self.hash_arrays = hash_arrays
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
        if isinstance(value, np.ndarray):
            return self._shared(value, lambda: self.array_key(value))
        if isinstance(value, np.bool_):
            return bool(value)
        if isinstance(value, np.integer):
            return int(value)
        if isinstance(value, np.floating):
            value = float(value)
        if isinstance(value, float) and not math.isfinite(value):
            return {"$float": repr(value)}
        if value is None or isinstance(value, (bool, int, float, str)):
            return value
        if isinstance(value, tuple):
            return {"$tuple": [self.encode(v) for v in value]}
        if dataclasses.is_dataclass(value) and not isinstance(value, type):
            return self._shared(value, lambda: {
                "$dataclass": _qualname(type(value)),
                "fields": {f.name: self.encode(getattr(value, f.name)) for f in dataclasses.fields(value)}})
        if isinstance(value, list):
            return self._shared(value, lambda: {"$list": [self.encode(v) for v in value]})
        if isinstance(value, dict):
            if all(isinstance(k, str) for k in value):
                return self._shared(value, lambda: {"$map": {k: self.encode(v) for k, v in value.items()}})
            return self._shared(value, lambda: {"$dict": [[self.encode(k), self.encode(v)] for k, v in value.items()]})
        if type(value).__name__ in self.value_types and hasattr(value, "__dict__"):
            return self._shared(value, lambda: {
                "$object": _qualname(type(value)),
                "attrs": {k: self.encode(v) for k, v in vars(value).items() if not k.startswith("_")}})
        if self.opaque_ok:
            return {"$opaque": _qualname(type(value))}
        raise TypeError(f"no value encoding for a {_qualname(type(value))}")


def encode(value: Any, *, opaque_ok: bool = False, value_types=()) -> tuple[Any, dict[str, np.ndarray]]:
    """``value`` encoded in a scope of its own: (encoded, arrays)."""
    encoder = Encoder(opaque_ok=opaque_ok, value_types=value_types)
    return encoder.encode(value), encoder.arrays


def decode(node: Any, arrays: dict[str, np.ndarray]) -> Any:
    """The inverse of :func:`encode`; comparison-only payloads (``$opaque``, ``$sha256``) come back as their
    marker dictionaries. A reference to an object that is still being built (a cycle through a dataclass or a
    value object) is refused."""
    by_id: dict[int, Any] = {}
    building = object()

    def walk(n: Any) -> Any:
        if isinstance(n, list):  # only inside our own containers (never a top-level encoding)
            return [walk(v) for v in n]
        if not isinstance(n, dict):
            return n
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
