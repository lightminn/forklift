"""The shadow's same-process audit (docs/plans/2026-11-06-unconfirmed-validation.md 0, lead 2026-11-06): with
``--pose-shadow-audit``, the control side's shared objects -- the arbiter, the obstacle layer with its permit, the
mission commands, the cargo and the SLAM tracker -- are fingerprinted whole (every level, arrays by content) before and
after a place the shadow computes in a tick, and every difference is counted with the attributes that changed. A whole
walk is too dear for every call, so it runs every PERIOD-th call and at each rear check; the record says how long one
took. Supporting evidence for what the source tests fix (the shadow's writes listed), in place of the deterministic
shadow-vs-off byte comparison a live session cannot have. The arbiter runs on the control thread only; a change seen
on an object another thread also writes is read from the attribute it names. Isaac-free.
"""

from __future__ import annotations

import hashlib
import time
import types

import numpy as np

DEPTH = 256  # the object graph's guard: in practice the whole of it
PERIOD = 121  # calls between audits (three a tick, so it walks through the sites)
KEEP = 20  # the first differences kept in full
NAMED_ONLY = (types.ModuleType, types.FunctionType, types.BuiltinFunctionType, types.MethodType, type)


def _walk(obj, depth: int, seen: set, out: list) -> None:
    if obj is None or isinstance(obj, (bool, int, float, str, bytes)):
        out.append(repr(obj))
        return
    if isinstance(obj, np.ndarray):
        out.append(f"nd{obj.dtype}{obj.shape}")
        out.append(hashlib.blake2b(obj.tobytes(), digest_size=16).hexdigest())
        return
    if isinstance(obj, np.generic):
        out.append(repr(obj.item()))
        return
    if isinstance(obj, NAMED_ONLY) or type(obj).__name__ in ("lock", "RLock"):
        out.append(f"<{type(obj).__name__}>")
        return
    if id(obj) in seen or depth <= 0:
        out.append(f"<{type(obj).__name__}:{id(obj)}>")
        return
    seen.add(id(obj))
    if isinstance(obj, dict):
        out.append("{")
        for k in sorted(obj, key=repr):
            out.append(repr(k))
            _walk(obj[k], depth - 1, seen, out)
        out.append("}")
    elif isinstance(obj, (list, tuple, set, frozenset)) or type(obj).__name__ == "deque":
        items = sorted(obj, key=repr) if isinstance(obj, (set, frozenset)) else obj
        out.append(f"{type(obj).__name__}[")
        for v in items:
            _walk(v, depth - 1, seen, out)
        out.append("]")
    elif hasattr(obj, "__dict__"):
        out.append(type(obj).__name__ + "(")
        for k in sorted(vars(obj)):
            out.append(k)
            _walk(vars(obj)[k], depth - 1, seen, out)
        out.append(")")
    else:
        out.append(f"<{type(obj).__name__}:{repr(obj)[:80]}>")


def fingerprint(obj, depth: int = DEPTH) -> dict:
    """{attribute: sha1} of an object's attributes (a dict's keys), each walked down; what two attributes share is
    walked once, under the first."""
    items = vars(obj) if hasattr(obj, "__dict__") and not isinstance(obj, dict) else obj
    out, seen = {}, {id(obj)}
    for k in sorted(items, key=repr):
        parts: list = []
        _walk(items[k], depth, seen, parts)
        out[str(k)] = hashlib.sha1("\x1f".join(parts).encode()).hexdigest()
    return out


class ShadowAudit:
    def __init__(self, targets: dict, period: int = PERIOD, clock=time.monotonic):
        """``targets``: {name: a callable giving the object now}."""
        self.targets, self.period, self.clock = targets, period, clock
        self.calls = 0
        self.wall_max_s = 0.0
        self.checks = 0
        self.changed = 0
        self.first: list = []
        self.by_site: dict = {}
        self._before = None

    def due(self, force: bool = False) -> bool:
        self.calls += 1
        return bool(force) or self.calls % self.period == 0

    def before(self) -> None:
        t0 = self.clock()
        self._before = {name: fingerprint(get()) for name, get in self.targets.items()}
        self.wall_max_s = max(self.wall_max_s, self.clock() - t0)

    def after(self, site: str, t: float) -> None:
        if self._before is None:
            return
        before, self._before = self._before, None
        diff = []
        for name, get in self.targets.items():
            now = fingerprint(get())
            diff += [f"{name}.{k}" for k in sorted(set(now) | set(before[name])) if now.get(k) != before[name].get(k)]
        self.checks += 1
        row = self.by_site.setdefault(site, {"checks": 0, "changed": 0})
        row["checks"] += 1
        if diff:
            self.changed += 1
            row["changed"] += 1
            if len(self.first) < KEEP:
                self.first.append({"t": float(t), "site": site, "attributes": diff})

    def record(self) -> dict:
        return {"targets": sorted(self.targets), "depth": DEPTH, "period": self.period, "calls": self.calls,
                "checks": self.checks, "changed": self.changed, "by_site": self.by_site, "first": self.first,
                "wall_max_s": self.wall_max_s}


__all__ = ["ShadowAudit", "fingerprint"]
