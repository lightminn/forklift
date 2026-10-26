"""Restart confirmation after an occupied stop (docs/plans/2026-10-25-restart-confirmation.md R1).

The obstacle grid lets a new scan's FREE evidence clear a cell it painted a moment before, by design. A cell of the
painted halo around a thin prop can then vanish for one scan and come back on the next, and the permission, which reads
each snapshot as it is, lets a stopped truck creep a few millimetres each time (CT-1 line_m P-3: 11 pulses, 6 cm, and
the P6 recovery came too late). Once the permission's own limit has stopped the truck for an occupied cell, its output
stays at zero until every clearing sensor has given N new scans in a row on which the raw limit is positive; the raw
limit is never raised (the output is the lesser of the two)."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Gated:
    allowed_mps: float
    why: str
    gate_only: bool  # the raw limit was positive and the confirmation alone held it at zero


class RestartConfirm:
    """One leg's confirmation state (a new ``key`` -- phase and tracker -- starts without it)."""

    def __init__(self, scans: int):
        if int(scans) < 1:
            raise ValueError("scans must be at least 1")
        self.n = int(scans)
        self.key = None
        self.armed = False
        self.baseline: dict = {}  # sensor -> its newest scan when the raw limit was last zero
        self.seen: dict = {}  # sensor -> new scan stamps since, on positive raw ticks
        self.arms = 0
        self.releases = 0
        self.gate_ticks = 0

    def _clear(self, scans: dict) -> None:
        self.baseline = {k: float(v) for k, v in scans.items()}
        self.seen = {k: set() for k in scans}

    def observation_gap(self, scans: dict) -> None:
        """A tick that read the permission another way (a plan wait's watch): start the count over from now."""
        if self.armed:
            self._clear(scans)

    def step(self, key, raw_allowed: float, raw_why: str, scans: dict) -> Gated:
        """This tick's output for the raw limit ``raw_allowed`` (reason ``raw_why``) with ``scans`` the newest scan stamp
        of each clearing sensor."""
        if key != self.key:
            self.key, self.armed = key, False
            self._clear(scans)
        raw_allowed = float(raw_allowed)
        if raw_allowed <= 0.0:
            self._clear(scans)  # a zero of any reason starts the count over and keeps its own reason
            if raw_why == "occupied" and not self.armed:
                self.armed = True
                self.arms += 1
            return Gated(raw_allowed, raw_why, False)
        if not self.armed:
            return Gated(raw_allowed, raw_why, False)
        for name, stamp in scans.items():
            if float(stamp) > self.baseline.get(name, -math.inf):
                self.seen.setdefault(name, set()).add(float(stamp))
        if scans and all(len(self.seen.get(name, ())) >= self.n for name in scans):
            self.armed = False
            self.releases += 1
            return Gated(raw_allowed, raw_why, False)
        self.gate_ticks += 1
        return Gated(0.0, "occupied", True)

    def record(self) -> dict:
        return {"scans": self.n, "arms": self.arms, "releases": self.releases, "gate_ticks": self.gate_ticks}
