"""Brief cases A-D at chassis scale: placements fixed by a turning radius.

Plan R2 of docs/plans/2026-10-01-measured-chassis-revalidation.md. The
placements are defined before any result from a reference radius R -- the
provisional planner's 2.000 m or the measured condition's 3.448 m -- and
each is planned with both planners, unloaded approach only: Hybrid A* with
the mission's 1.29 / 0.17 / 0.36 m footprint, 0.10 m clearance, 0.25 m
primitives and 30,000 expansions inside the original bay.

The rows record what the planner did. Where it differs from what the brief
expects (A straight in, B a curve without stopping, C back up first, D a
limited retreat or relocation) that is a finding about the planner, which
has no contract for the gear it ends or starts in, not a test failure.

    python tools/chassis_ad_cases.py [--json out.json]
"""

from __future__ import annotations

import argparse
import json
import math

import numpy as np

from forklift_core.planning import (
    Bounds,
    Footprint,
    Pose2D,
    Rectangle,
    plan_hybrid_astar,
)
from forklift_core.planning.pallet_mission import make_transport_planner_config

BAY = Bounds(-3.0, 4.7, -1.75, 3.05)
FOOTPRINT = Footprint(1.29, 0.17, 0.36)
START = Pose2D(-2.34, 0.0, 0.0)
LATERAL_M = 0.72
FINAL_STRAIGHT_M = 0.8
REFERENCE_RADII_M = {"P": 2.000, "M": 1 / 0.29}
PLANNER_CURVATURE = {"P": 0.50, "M": 0.29}
# D: a 0.15 x 0.8 m block whose front face is 0.30 m behind the truck's rear end.
REAR_GAP_M = 0.30
REAR_BLOCK = (0.15, 0.8)
# Defined after the first results (see placements); never pooled with A-D.
POST_HOC = frozenset({"B0", "C0"})


def reference_length_m(radius_m: float) -> float:
    """Longitudinal run of two opposite R arcs shifting LATERAL_M, plus the straight."""
    return math.sqrt(4 * radius_m * LATERAL_M - LATERAL_M**2) + FINAL_STRAIGHT_M


def rear_obstacle() -> Rectangle:
    rear_end = START.x_m - FOOTPRINT.rear_m
    front = rear_end - REAR_GAP_M
    return Rectangle(front - REAR_BLOCK[0] / 2, 0.0, *REAR_BLOCK)


def placements(radius_m: float) -> dict[str, tuple[Pose2D, list[Rectangle]]]:
    """A-D as fixed by the plan, then B0/C0 added after the first results.

    B and C put the final 0.8 m straight into the goal distance, but this
    search ends at the goal without having to arrive straight, so it may
    spend that 0.8 m turning; the P-radius B then fits the M planner as
    well. B0/C0 drop the straight. They were defined after seeing that
    result and are reported as such, beside the pre-registered rows.
    """
    length = reference_length_m(radius_m)
    arcs = length - FINAL_STRAIGHT_M
    near = Pose2D(START.x_m + length / 2, LATERAL_M, 0.0)
    return {
        "A": (Pose2D(1.5, 0.0, 0.0), []),
        "B": (Pose2D(START.x_m + length, LATERAL_M, 0.0), []),
        "C": (near, []),
        "D": (near, [rear_obstacle()]),
        "B0": (Pose2D(START.x_m + arcs, LATERAL_M, 0.0), []),
        "C0": (Pose2D(START.x_m + arcs / 2, LATERAL_M, 0.0), []),
    }


def _segment_distance(p, q, a, b) -> float:
    """Distance between segments pq and ab (2-D), zero if they cross."""

    def cross(o, u, v):
        return (u[0] - o[0]) * (v[1] - o[1]) - (u[1] - o[1]) * (v[0] - o[0])

    def point_segment(x, u, v):
        ux, uy = v[0] - u[0], v[1] - u[1]
        t = max(
            0.0,
            min(1.0, ((x[0] - u[0]) * ux + (x[1] - u[1]) * uy) / (ux * ux + uy * uy)),
        )
        return math.hypot(x[0] - u[0] - t * ux, x[1] - u[1] - t * uy)

    d1, d2 = cross(a, b, p), cross(a, b, q)
    d3, d4 = cross(p, q, a), cross(p, q, b)
    if d1 * d2 < 0 and d3 * d4 < 0:
        return 0.0
    return min(
        point_segment(p, a, b),
        point_segment(q, a, b),
        point_segment(a, p, q),
        point_segment(b, p, q),
    )


def _footprint_polygon(x: float, y: float, yaw: float) -> list[tuple[float, float]]:
    c, s = math.cos(yaw), math.sin(yaw)
    local = [
        (FOOTPRINT.front_m, FOOTPRINT.half_width_m),
        (-FOOTPRINT.rear_m, FOOTPRINT.half_width_m),
        (-FOOTPRINT.rear_m, -FOOTPRINT.half_width_m),
        (FOOTPRINT.front_m, -FOOTPRINT.half_width_m),
    ]
    return [(x + c * u - s * v, y + s * u + c * v) for u, v in local]


def polygon_gap_m(first, second) -> float:
    """Edge-to-edge distance of two convex polygons; 0 when they meet."""

    def edges(poly):
        return [(poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))]

    return min(
        _segment_distance(p, q, a, b) for p, q in edges(first) for a, b in edges(second)
    )


def _min_gap_m(poses: np.ndarray, block: Rectangle) -> float:
    """Smallest footprint-to-block gap over the path's samples.

    This is the minimum at the planner's pose samples, not a continuous
    swept-volume bound between them.
    """
    x0, x1 = block.x_m - block.length_m / 2, block.x_m + block.length_m / 2
    y0, y1 = block.y_m - block.width_m / 2, block.y_m + block.width_m / 2
    rect = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return min(polygon_gap_m(_footprint_polygon(*pose), rect) for pose in poses)


def run_cases() -> list[dict]:
    rows = []
    for basis, radius in REFERENCE_RADII_M.items():
        for case, (goal, obstacles) in placements(radius).items():
            for planner, curvature in PLANNER_CURVATURE.items():
                config = make_transport_planner_config(
                    curvature_limit_inv_m=curvature,
                    clearance_m=0.10,
                    max_expansions=30000,
                )
                result = plan_hybrid_astar(
                    START, goal, obstacles, FOOTPRINT, BAY, config
                )
                row = {
                    "definition": "post_hoc" if case in POST_HOC else "predefined",
                    "basis": basis,
                    "case": case,
                    "planner": planner,
                    "goal": [goal.x_m, goal.y_m, goal.yaw_rad],
                    "status": result.status,
                    "length_m": round(float(result.length_m), 3),
                    "expansions": int(result.expanded_nodes),
                }
                if result.success:
                    d = np.asarray(result.directions)
                    row.update(
                        first_gear=int(d[1] if d.size > 1 else d[0]),
                        last_gear=int(d[-1]),
                        gear_changes=int(np.count_nonzero(np.diff(d))),
                        max_abs_yaw_rad=round(
                            float(np.max(np.abs(result.poses[:, 2]))), 3
                        ),
                    )
                    if obstacles:
                        row["min_sampled_gap_to_rear_block_m"] = round(
                            _min_gap_m(result.poses, obstacles[0]), 3
                        )
                rows.append(row)
    return rows


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", default=None)
    args = parser.parse_args(argv)
    rows = run_cases()
    print(
        f"{'def':>10} {'basis':>5} {'case':>4} {'plan':>4} {'goal x':>7} {'status':>16} "
        f"{'first':>5} {'last':>4} {'shift':>5} {'max|yaw|':>8} {'len':>6} {'rear gap':>8}"
    )
    for r in rows:
        print(
            f"{r['definition']:>10} {r['basis']:>5} {r['case']:>4} {r['planner']:>4} {r['goal'][0]:7.3f}"
            f" {r['status']:>16} {r.get('first_gear', ''):>5} {r.get('last_gear', ''):>4}"
            f" {r.get('gear_changes', ''):>5} {r.get('max_abs_yaw_rad', ''):>8}"
            f" {r['length_m']:6.2f} {r.get('min_sampled_gap_to_rear_block_m', ''):>8}"
        )
    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(rows, handle, indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
