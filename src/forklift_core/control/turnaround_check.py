"""The return turnaround's arc check (docs/plans/2026-10-12-return-forward-turnaround.md) as a pure function, and a
frozen, picklable checker for the planning worker (persistent console plan D11): the permission and the snapshot
copied at the request, so the worker judges the arcs on the same grid the plan's occupancy was fixed from."""

from __future__ import annotations

import copy

import numpy as np


def turnaround_arc_reason(permission, snapshot, footprint, own, poses, *, current_pose) -> str | None:
    """None when the unloaded reverse arc ``poses`` (rear-axle samples from ``current_pose``) meets the spatial
    conditions for a turnaround candidate to be compared, else why not: the permission's own walk ahead verified as
    for any path, and no cell known occupied anywhere on the arc (a walk past unknown cells). ``footprint``/``own``
    are the reversing unloaded outlines the layer checks with."""
    poses = np.asarray(poses, dtype=float)
    arc_m = float(np.hypot(*np.diff(poses[:, :2], axis=0).T).sum())
    near = permission.check_path(snapshot, poses, footprint, own, current_pose=current_pose, direction=-1)
    if near.blocked is not None:
        return near.blocked
    if not near.reached_end and near.verified_m < min(arc_m, permission.config.lookahead_m) - 1e-6:
        return "short"
    whole = permission.check_path(snapshot, poses, footprint, own, current_pose=current_pose, direction=-1,
                                  unknown_passes=True, lookahead_m=arc_m + 0.05)
    return "occupied_ahead" if whole.blocked == "occupied" else None


class TurnaroundArcCheck:
    """The arc check frozen at a request: copies of the permission and its snapshot (about 1 MB, 5 ms), the two
    outlines, and the replay's callback_spec. Called with an arc's poses, from its first pose."""

    def __init__(self, permission, snapshot, footprint, own, callback_spec: dict):
        self.permission = copy.deepcopy(permission)
        self.snapshot = copy.deepcopy(snapshot)
        self.footprint, self.own = footprint, own
        self.callback_spec = dict(callback_spec)

    def __call__(self, poses) -> str | None:
        if self.snapshot is None:
            return "no_snapshot"
        poses = np.asarray(poses, dtype=float)
        return turnaround_arc_reason(self.permission, self.snapshot, self.footprint, self.own, poses,
                                     current_pose=tuple(float(v) for v in poses[0]))
