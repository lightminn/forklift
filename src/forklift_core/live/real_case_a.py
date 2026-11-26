"""The operating REAL run's case A and run time (docs/plans/2026-11-26-real-operating-assembly.md 2.4 and 2.2; unlock
U3b-1). Pure rules the loop and the assembly apply on the operating run only -- no new case classifier: case A is the
approved placement and path.

- ``placement_refusal``: the submitted pallet's registered pose within ``PLACE_TOL_M`` / ``PLACE_TOL_RAD`` of S10-A's
  ``check_pallet_pose``.
- ``corridor`` / ``path_refusal``: the adopted plan's ``approach`` and ``insert`` (the stages S10 drives; ``extract``
  is always planned in reverse and never driven) go forward only and stay within ``CORRIDOR_HALF_M`` of the approved
  corridor -- the segment from S10-A's taped start (rear axle) to the insert end fixed by the approved placement and
  the geometry's ``inserted_offset_m`` (never taken back from the adopted plan's own end).
- ``remaining_s``: the run's time left on its single deadline.

The start check (the rear axle within the loop's start tolerance) is the loop's own ``_off_start``, which the operating
run applies whatever the ``start_pose`` bypass says."""

from __future__ import annotations

import math

PLACE_TOL_M = 0.10
PLACE_TOL_RAD = math.radians(5.0)
CORRIDOR_HALF_M = 0.10
# provisional run-time margins (plan 2.2; autonomy.yaml may set them): a submit needs this much time left, and the
# insert's entry and every resume inside it this much
MISSION_TIME_BUDGET_S = 300.0
INSERT_TIME_MIN_S = 120.0
DRIVEN_STAGES = ("approach", "insert")


def _gap(a, b) -> tuple:
    d = math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))
    yaw = abs(math.remainder(float(a[2]) - float(b[2]), math.tau))
    return d, yaw


def placement_refusal(pallet_pose, check_pose) -> str | None:
    """``case_a:placement`` when the registered pallet is off the approved placement (or either is missing)."""
    if pallet_pose is None or check_pose is None:
        return "case_a:placement_unset"
    d, yaw = _gap(pallet_pose, check_pose)
    if d > PLACE_TOL_M or yaw > PLACE_TOL_RAD:
        return "case_a:placement"
    return None


def corridor(start_rear, check_pallet_pose, inserted_offset_m: float) -> tuple:
    """((x0, y0), (x1, y1)): the start's rear axle to the insert end's rear axle on the approved placement
    (``pallet_mission.site_poses``: the pallet centre less the offset along its yaw)."""
    px, py, pyaw = (float(v) for v in check_pallet_pose)
    end = (px - float(inserted_offset_m) * math.cos(pyaw), py - float(inserted_offset_m) * math.sin(pyaw))
    return (float(start_rear[0]), float(start_rear[1])), end


def _off_segment(p, a, b) -> float:
    ax, ay = a
    bx, by = b
    vx, vy = bx - ax, by - ay
    n2 = vx * vx + vy * vy
    s = 0.0 if n2 == 0.0 else max(0.0, min(1.0, ((p[0] - ax) * vx + (p[1] - ay) * vy) / n2))
    return math.hypot(p[0] - (ax + s * vx), p[1] - (ay + s * vy))


def path_refusal(plan, segment) -> str | None:
    """The first case-A breach of the adopted plan's driven stages, or None. A stage missing is a breach too."""
    for name in DRIVEN_STAGES:
        stage = getattr(plan, name, None)
        poses = getattr(stage, "poses", None)
        if stage is None or poses is None or len(poses) == 0:
            return f"case_a:{name}_missing"
        dirs = getattr(stage, "directions", None)
        if dirs is not None and any(int(d) < 0 for d in dirs):
            return f"case_a:{name}_reverse"
        for p in poses:
            if _off_segment((float(p[0]), float(p[1])), *segment) > CORRIDOR_HALF_M + 1e-9:
                return f"case_a:{name}_outside_corridor"
    return None


def remaining_s(deadline_ns, now_ns: int) -> float:
    """Seconds left before the run's deadline (infinite with none)."""
    return math.inf if deadline_ns is None else (int(deadline_ns) - int(now_ns)) / 1e9


__all__ = ["CORRIDOR_HALF_M", "DRIVEN_STAGES", "INSERT_TIME_MIN_S", "MISSION_TIME_BUDGET_S", "PLACE_TOL_M",
           "PLACE_TOL_RAD", "corridor", "path_refusal", "placement_refusal", "remaining_s"]
