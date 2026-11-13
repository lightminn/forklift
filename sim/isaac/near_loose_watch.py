"""K1's record-only safety watch (docs/plans/2026-11-13-k1-near-loose-arrival.md 2.2): from each intervention the
mission core opens (a new entry in ``state["near_loose_refusals"]``) to the end of the near capture that follows (the
near capture's status leaving ``pending``), every physics step's pallet move since the intervention began and the
fork/body-to-pallet geometric contacts, accumulated into ``state["near_loose_watch"]``.

It never stops anything. The runner builds it only with ``--near-observe-arrival``: without the option there is no
object and ``step_world`` only tests ``is None``.
"""

from __future__ import annotations

import math


class NearLooseWatch:
    def __init__(self) -> None:
        self.seen = 0  # interventions already watched
        self.window: dict | None = None  # the open window (also the last entry of state["near_loose_watch"])
        self._start_xy: tuple[float, float] | None = None

    @property
    def active(self) -> bool:
        return self.window is not None

    def tick(self, t: float, state: dict, pallet_xy) -> None:
        """Each control tick, after the core's: open a window at a new intervention, close it once the near capture is
        no longer pending (the capture's own steps ran before this tick)."""
        refusals = state.get("near_loose_refusals") or ()
        if self.window is None and len(refusals) > self.seen:
            self.seen = len(refusals)
            self._start_xy = (float(pallet_xy[0]), float(pallet_xy[1]))
            self.window = {"start_s": float(t), "refusal_index": len(refusals) - 1,
                           "start_pallet_xy": list(self._start_xy), "steps": 0, "max_move_m": 0.0,
                           "contact_steps": 0, "first_contact": None, "end_s": None}
            state.setdefault("near_loose_watch", []).append(self.window)
            return
        if self.window is not None and (state.get("near_capture") or {}).get("status") != "pending":
            self.window["end_s"] = float(t)
            self.window["near_capture_status"] = (state.get("near_capture") or {}).get("status")
            self.window = None
            self.seen = len(refusals)  # an intervention inside the closed window is part of it

    def step(self, pallet_xy, contacts: list, time_s: float) -> None:
        """Each physics step while a window is open: the pallet's move since the intervention began, and the contacts
        the geometric checks found (any, even if gone the next step)."""
        if self.window is None:
            return
        w = self.window
        w["steps"] += 1
        move = math.hypot(float(pallet_xy[0]) - self._start_xy[0], float(pallet_xy[1]) - self._start_xy[1])
        w["max_move_m"] = max(w["max_move_m"], move)
        if contacts:
            w["contact_steps"] += 1
            if w["first_contact"] is None:
                w["first_contact"] = {"time_s": float(time_s), "contacts": list(contacts)}
