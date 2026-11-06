"""The shadow's same-process audit (docs/plans/2026-11-06-unconfirmed-validation.md 1.1, lead 2026-11-06): with
``--pose-shadow-audit``, the control side's shared objects -- the arbiter, the permit, the mission commands and the
cargo -- are fingerprinted before and after each place the shadow computes in a tick, and every difference is counted
with the attributes that changed. It stands in for the deterministic shadow-vs-off byte comparison, which a live
session cannot have (the deterministic mode has no live part). The arbiter and the core run on the control thread
only, so nothing else moves them between the two fingerprints. Isaac-free.
"""

from __future__ import annotations

import hashlib

import numpy as np

DEPTH = 3
ARRAY_HASH_MAX = 1 << 20  # larger arrays by shape and identity only (their content is not hashed every tick)
KEEP = 20  # the first differences kept in full


def _walk(obj, depth: int, seen: set, out: list) -> None:
    if obj is None or isinstance(obj, (bool, int, float, str, bytes)):
        out.append(repr(obj))
        return
    if isinstance(obj, np.ndarray):
        out.append(f"nd{obj.dtype}{obj.shape}")
        out.append(hashlib.sha1(obj.tobytes()).hexdigest() if obj.nbytes <= ARRAY_HASH_MAX else f"id{id(obj)}")
        return
    if isinstance(obj, np.generic):
        out.append(repr(obj.item()))
        return
    if id(obj) in seen or depth <= 0 or callable(obj) and not hasattr(obj, "__dict__"):
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
    """{attribute: sha1} of an object's attributes (a dict's keys), each walked ``depth`` levels down."""
    items = vars(obj) if hasattr(obj, "__dict__") and not isinstance(obj, dict) else obj
    out = {}
    for k in sorted(items, key=repr):
        parts: list = []
        _walk(items[k], depth, set(), parts)
        out[str(k)] = hashlib.sha1("\x1f".join(parts).encode()).hexdigest()
    return out


class ShadowAudit:
    def __init__(self, targets: dict):
        """``targets``: {name: a callable giving the object now}."""
        self.targets = targets
        self.checks = 0
        self.changed = 0
        self.first: list = []
        self.by_site: dict = {}
        self._before = None

    def before(self) -> None:
        self._before = {name: fingerprint(get()) for name, get in self.targets.items()}

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
        return {"targets": sorted(self.targets), "depth": DEPTH, "checks": self.checks, "changed": self.changed,
                "by_site": self.by_site, "first": self.first}


__all__ = ["ShadowAudit", "fingerprint"]
