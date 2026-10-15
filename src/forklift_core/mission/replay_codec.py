"""The value codec of the M-a1 replay (docs/plans/2026-10-16-m-a1-replay.md, step R1).

Boundary records of format 1 (``boundary_record.py``) keep values well enough to compare them, not to hand
them back: tuples become lists, a dataclass keeps its bare class name, dictionary keys become strings and
two references to one list become two lists. A replay hands recorded values back to code that may unpack
a tuple, test ``isinstance`` or append to a list another holder also sees, so the setup snapshot and the
format-2 records use this codec instead:

- arrays by content hash into a side table (``{"$array": key, "dtype", "shape"}``);
- ``{"$tuple": [...]}``; ``{"$dataclass": "module:Qualname", "fields": {...}}``; non-finite floats as
  ``{"$float": "nan"}``; dictionaries whose keys are not all strings as ``{"$dict": [[k, v], ...]}``;
- every list and dictionary met twice in one encoding is written once with ``"$id"`` and referred to by
  ``{"$ref": id}`` afterwards (the alias graph, by object identity);
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
    """One encoding scope: the array table and the alias graph are shared by everything it encodes."""

    def __init__(self, *, opaque_ok: bool = False) -> None:
        self.arrays: dict[str, np.ndarray] = {}
        self.opaque_ok = opaque_ok
        self._seen: dict[int, list] = {}  # id(obj) -> [obj, encoded node, assigned id or None]
        self._next_id = 0

    def array_key(self, value: np.ndarray) -> dict:
        array = np.ascontiguousarray(value).copy()
        digest = hashlib.sha256()
        digest.update(str(array.dtype).encode())
        digest.update(repr(array.shape).encode())
        digest.update(array.tobytes())
        key = digest.hexdigest()
        self.arrays.setdefault(key, array)
        return {"$array": key, "dtype": str(array.dtype), "shape": list(array.shape)}

    def encode(self, value: Any) -> Any:
        if isinstance(value, np.ndarray):
            return self.array_key(value)
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
            return {"$dataclass": _qualname(type(value)),
                    "fields": {f.name: self.encode(getattr(value, f.name)) for f in dataclasses.fields(value)}}
        if isinstance(value, (list, dict)):
            seen = self._seen.get(id(value))
            if seen is not None:
                if seen[2] is None:  # second reference: name the first one
                    seen[2] = self._next_id
                    self._next_id += 1
                    seen[1]["$id"] = seen[2]
                return {"$ref": seen[2]}
            node: dict = {}
            self._seen[id(value)] = [value, node, None]  # keeps value alive: ids stay unique in the scope
            if isinstance(value, list):
                node["$list"] = [self.encode(v) for v in value]
            elif all(isinstance(k, str) for k in value):
                node["$map"] = {k: self.encode(v) for k, v in value.items()}
            else:
                node["$dict"] = [[self.encode(k), self.encode(v)] for k, v in value.items()]
            return node
        if self.opaque_ok:
            return {"$opaque": _qualname(type(value))}
        raise TypeError(f"no value encoding for a {_qualname(type(value))}")


def encode(value: Any, *, opaque_ok: bool = False) -> tuple[Any, dict[str, np.ndarray]]:
    """``value`` encoded in a scope of its own: (encoded, arrays)."""
    encoder = Encoder(opaque_ok=opaque_ok)
    return encoder.encode(value), encoder.arrays


def decode(node: Any, arrays: dict[str, np.ndarray]) -> Any:
    """The inverse of :func:`encode` (opaque values come back as the marker dictionary)."""
    by_id: dict[int, Any] = {}

    def walk(n: Any) -> Any:
        if isinstance(n, list):  # only inside our own containers (never a top-level encoding)
            return [walk(v) for v in n]
        if not isinstance(n, dict):
            return n
        if "$array" in n:
            return arrays[n["$array"]].copy()
        if "$float" in n:
            return float(n["$float"])
        if "$tuple" in n:
            return tuple(walk(v) for v in n["$tuple"])
        if "$dataclass" in n:
            cls = _resolve(n["$dataclass"])
            fields = {k: walk(v) for k, v in n["fields"].items()}
            init = {f.name: fields[f.name] for f in dataclasses.fields(cls) if f.init and f.name in fields}
            obj = cls(**init)
            for f in dataclasses.fields(cls):
                if not f.init and f.name in fields:
                    object.__setattr__(obj, f.name, fields[f.name])
            return obj
        if "$ref" in n:
            return by_id[n["$ref"]]
        if "$list" in n:
            out: list = []
            if "$id" in n:
                by_id[n["$id"]] = out
            out.extend(walk(v) for v in n["$list"])
            return out
        if "$map" in n or "$dict" in n:
            out_d: dict = {}
            if "$id" in n:
                by_id[n["$id"]] = out_d
            items = n["$map"].items() if "$map" in n else ((walk(k), v) for k, v in n["$dict"])
            for k, v in items:
                out_d[k] = walk(v)
            return out_d
        if "$opaque" in n or "$sha256" in n:  # comparison-only payloads come back as their markers
            return dict(n)
        raise ValueError(f"unknown node {sorted(n)}")

    return walk(node)
