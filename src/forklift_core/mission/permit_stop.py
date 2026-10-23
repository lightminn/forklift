"""The permission-stop trigger of the scene-mode replan device (docs/plans/2026-10-23-planner-permission-clearance.md
3 P6, user decision 2026-10-23).

In a scene layout a travel leg the permission holds at zero for an occupied cell stood until its tracking timeout:
the planner keeps its clearance from the props at their size while the obstacle layer marks them wider, so the
same path came back from any replan. The trigger hands such a stop to the device (docs/plans/2026-10-23-scene-
replan.md) with what the permission saw: the blocked cells, grown over the occupied region they belong to, as
obstacles, and a clearance ``PERMIT_STOP_CLEARANCE_ADD_M`` wider than the leg's.
"""

from __future__ import annotations

from collections import deque

import numpy as np

from forklift_core.perception.obstacle_grid import OCCUPIED
from forklift_core.planning import Rectangle

PERMIT_STOP_S = 3.0  # the zero limit for an occupied cell, held this long (work time), raises the request
PERMIT_STOP_CLEARANCE_ADD_M = 0.10  # the replan's clearance over the leg's (the lower end of the measured need)
PERMIT_STOP_GROW_M = 0.5  # a blocked cell stands for the occupied cells connected to it within this distance


def blocked_rectangles(snapshot, seeds, *, grow_m: float = PERMIT_STOP_GROW_M) -> list:
    """The blocked cells ``seeds`` ((i, j) of ``snapshot``) and the OCCUPIED cells 8-connected to them within
    ``grow_m`` of a seed, as axis-aligned rectangles: each row's runs of cells, runs with the same span in
    neighbouring rows merged. A seed the snapshot no longer marks occupied stands for itself (the permission
    blocked on it)."""
    seeds = [(int(i), int(j)) for i, j in seeds]
    if not seeds:
        return []
    state = np.asarray(snapshot.state)
    nx, ny = state.shape
    res = float(snapshot.resolution_m)
    reach2 = (grow_m / res) ** 2
    seed_arr = np.asarray(seeds, dtype=float)
    keep = {s for s in seeds if 0 <= s[0] < nx and 0 <= s[1] < ny}
    queue = deque(s for s in keep if state[s] == OCCUPIED)
    seen = set(queue)
    while queue:
        i, j = queue.popleft()
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                c = (i + di, j + dj)
                if c in seen or not (0 <= c[0] < nx and 0 <= c[1] < ny) or state[c] != OCCUPIED:
                    continue
                if float(np.min(((seed_arr - c) ** 2).sum(axis=1))) > reach2 + 1e-9:
                    continue
                seen.add(c)
                queue.append(c)
    cells = keep | seen
    # runs along i for each j, then the same run in consecutive j merged
    runs = {}
    for j in sorted({c[1] for c in cells}):
        row = sorted(c[0] for c in cells if c[1] == j)
        start = prev = row[0]
        for i in row[1:] + [None]:
            if i is not None and i == prev + 1:
                prev = i
                continue
            runs.setdefault((start, prev), []).append(j)
            if i is not None:
                start = prev = i
    rects = []
    for (i0, i1), js in runs.items():
        js = sorted(js)
        lo = hi = js[0]
        for j in js[1:] + [None]:
            if j is not None and j == hi + 1:
                hi = j
                continue
            rects.append(Rectangle(
                float(snapshot.origin_x_m + (i0 + i1 + 1) * res / 2), float(snapshot.origin_y_m + (lo + hi + 1) * res / 2),
                float((i1 - i0 + 1) * res), float((hi - lo + 1) * res), 0.0))
            if j is not None:
                lo = hi = j
    return rects


def permission_blocked_rectangles(permission, *, grow_m: float = PERMIT_STOP_GROW_M) -> list:
    """The rectangles of what ``permission`` (a DrivePermission) blocks on now: its path check's blocked cells, else
    its emergency-stop arc's, on its current snapshot; none without a snapshot."""
    snap = getattr(permission, "snapshot", None)
    if snap is None:
        return []
    cells = tuple(getattr(getattr(permission, "path_check", None), "blocked_cells", ()) or ())
    if not cells:
        cells = tuple(getattr(getattr(permission, "last_estop", None), "blocked_cells", ()) or ())
    return blocked_rectangles(snap, cells, grow_m=grow_m)


__all__ = ["PERMIT_STOP_CLEARANCE_ADD_M", "PERMIT_STOP_GROW_M", "PERMIT_STOP_S", "blocked_rectangles",
           "permission_blocked_rectangles"]
