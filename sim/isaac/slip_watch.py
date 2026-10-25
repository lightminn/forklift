"""The carried load's slip watch: a truth check of the simulation (docs/plans/2026-10-25-slip-check-relative.md).

The earlier check put the pallet origin within 0.04 m of the truck axis while carried. That absolute lateral sums the
forks' place in the pockets and the fork-pallet yaw over a ~0.93 m lever, so a load that never moved could fail it
(three recorded "slips", all seated so at the lift; the joint-angle promotion diagnosis 2.3). The watch judges instead:

1. the forks in the pockets -- each blade with a pocket wall on both sides, both gaps above POCKET_MARGIN_M
   (insertion_geometry.pocket_side_gaps; the seat and any later slide together, against the pallet's real blocks);
2. the motion since the load was taken -- the pallet outline's corners in the truck frame against the insert-to-lift
   instant, at most MOTION_MAX_M (translation and rotation; the yaw difference wrapped);
3. the absolute along check, unchanged (the origin within ALONG_MAX_M of the inserted offset).

The baseline follows the load: taken at the insert-to-lift transition's tick (the truth read before that tick's
physics step), kept through extract, transport and a loaded resubmit, cleared when the next insert starts. Without one
(a load the watch never saw lifted) condition 2 is skipped. The old absolute lateral is recorded, not judged, so a run
shows where the earlier check would have failed.
"""
from __future__ import annotations

import math

POCKET_MARGIN_M = 0.002   # the forbidden-contact check's margin (run_transport pocket_clearance_margin_m)
MOTION_MAX_M = 0.04       # the earlier lateral bound's size, now on motion since the load was taken
ALONG_MAX_M = 0.08        # unchanged
OLD_LATERAL_MAX_M = 0.04  # the earlier check, recorded only


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def relative(base, pallet) -> tuple:
    """The pallet origin and yaw in the truck's base frame: (along, lateral, yaw) from world (x, y, yaw) poses."""
    bx, by, byaw = (float(v) for v in base)
    px, py, pyaw = (float(v) for v in pallet)
    c, s = math.cos(byaw), math.sin(byaw)
    dx, dy = px - bx, py - by
    return (c * dx + s * dy, -s * dx + c * dy, _wrap(pyaw - byaw))


class SlipWatch:
    def __init__(self, *, inserted_along_m: float, outline_half_m: tuple):
        self.inserted_along_m = float(inserted_along_m)
        self.outline_half_m = (float(outline_half_m[0]), float(outline_half_m[1]))
        self.baseline: dict | None = None
        self._phase: str | None = None

    def _corners(self, rel) -> list:
        x, y, a = rel
        c, s = math.cos(a), math.sin(a)
        hx, hy = self.outline_half_m
        return [(x + c * u - s * v, y + s * u + c * v) for u in (-hx, hx) for v in (-hy, hy)]

    def end_tick(self, phase: str, base, pallet, *, t: float) -> None:
        """The tick's end, with the phase it ended in and the truth read at its start (as TraceEvidence): the baseline's
        lifecycle. Any other phase -- extract, transport, a loaded resubmit, ``idle`` between missions -- keeps it."""
        if phase == "insert":
            self.baseline = None
        elif phase == "lift" and self._phase == "insert":
            self.baseline = {"seat": list(relative(base, pallet)), "t": float(t)}
        self._phase = phase

    def check(self, summary: dict, *, t: float, phase: str, base, pallet, gaps: dict) -> str | None:
        """One watched tick: None, or the failing condition (pocket_clearance / outline_motion / along). ``summary``
        is the mission's record (state["slip_watch"]), updated -- the failing tick included -- before returning."""
        rel = relative(base, pallet)
        summary.setdefault("checks", 0)
        summary["checks"] += 1
        summary["baseline"] = None if self.baseline is None else dict(self.baseline)
        # the earlier check, recorded only
        old = abs(rel[1])
        summary["old_lateral_max_m"] = max(summary.get("old_lateral_max_m", 0.0), old)
        summary.setdefault("old_would_fail_s", None)
        if old >= OLD_LATERAL_MAX_M and summary["old_would_fail_s"] is None:
            summary["old_would_fail_s"] = float(t)
        # 1. the forks in the pockets
        gap_min = {}
        in_pockets = True
        for side in ("left", "right"):
            pair = (gaps or {}).get(side) or (None, None)
            values = [v for v in pair if v is not None]
            gap_min[side] = min(values) if values else None
            if len(values) < 2 or min(values) <= POCKET_MARGIN_M:
                in_pockets = False
        mins = summary.setdefault("pocket_gap_min_m", {"left": None, "right": None})
        for side, v in gap_min.items():
            if v is not None and (mins[side] is None or v < mins[side]):
                mins[side] = v
        # 2. the motion since the load was taken
        motion = None
        if self.baseline is not None:
            now, then = self._corners(rel), self._corners(self.baseline["seat"])
            motion = max(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(now, then))
            summary["outline_motion_max_m"] = max(summary.get("outline_motion_max_m", 0.0), motion)
        # 3. the absolute along check
        along = abs(rel[0] - self.inserted_along_m)
        summary["along_error_max_m"] = max(summary.get("along_error_max_m", 0.0), along)
        reason = None
        if not in_pockets:
            reason = "pocket_clearance"
        elif motion is not None and motion > MOTION_MAX_M:
            reason = "outline_motion"
        elif along >= ALONG_MAX_M:
            reason = "along"
        if reason is not None:
            summary["failure"] = {"reason": reason, "t": float(t), "phase": phase, "rel": list(rel),
                                  "pocket_gaps": {k: list(v) if v is not None else None for k, v in (gaps or {}).items()},
                                  "outline_motion_m": motion, "along_error_m": along}
        return reason
