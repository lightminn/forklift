"""Limited memory of the truck's own shadow band (priority-5 plan D4 delta, 2026-10-05).

A low planar LiDAR cannot certify the strip right in front of the truck's
own faces: every beam that reaches it ends on the face a few centimetres
later, and a return that close to the face is indistinguishable from the
face within the range bound. From the corner mounts the strip is up to
0.77 a deep, a being how far the sensor sits ahead of the face -- 43 mm for
the front-left LiDAR in front of the carriage. A stop that starts flush with
the face therefore always sweeps unverifiable cells, so the strict rule
(every swept cell freshly FREE) can never let the truck start or creep.

The user approved (2026-10-05) a static-scope memory confined to that band:

* the band B is every cell overlapping the own outline grown by band_m that
  is not wholly inside the outline;
* a FREE cell is remembered with its own observation time, never a later
  one; a cell wholly under the truck is remembered at each snapshot it is
  covered (the truck itself is there); a remembered cell that leaves B and
  the outline, or turns OCCUPIED, is forgotten;
* an UNKNOWN cell of B that is remembered is RETAINED while its age is in the
  odometry error table (10 s) and every cell within its placement error
  e(age) + 2 rho sin(psi / 2) + res / 2 is freshly FREE, wholly inside the
  outline, or itself RETAINED -- band cells support each other only inside
  B, whose outer boundary must therefore be freshly observed;
* at start-up the band's non-OCCUPIED cells are registered at the start time
  under the operating premise "nothing inside the band at start-up";
* a SLAM correction change moves the memory once, cell by cell, keeping a new
  cell only if all its corners and its centre came from remembered cells.

Out of scope (recorded in the plan): objects moving into the band, or
appearing inside it, after they were last seen outside it.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from math import cos, hypot, sin

import numpy as np

from forklift_core.control.drive_permission import RETAINED, footprint_cells
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, UNKNOWN, AgeErrorTable, GridSnapshot
from forklift_core.planning.geometry import Footprint



def _compose(a, b):
    x, y, t = a
    return (x + b[0] * cos(t) - b[1] * sin(t), y + b[0] * sin(t) + b[1] * cos(t), t + b[2])


def _invert(a):
    x, y, t = a
    c, s = cos(t), sin(t)
    return (-x * c - y * s, x * s - y * c, -t)


@dataclass
class ShadowMemory:
    band_m: float
    error: AgeErrorTable
    evidence_max_age_s: float = 0.2  # a RETAINED cell is judged for this long ahead (Codex P1)
    rho_margin_m: float = 0.15  # truck travel between a FREE scan and the snapshot recording it
    memory: dict = field(default_factory=dict)  # absolute cell -> (observation time, source, rho bound)
    correction: tuple | None = None
    started: bool = False
    stats: dict = field(default_factory=lambda: {"retained_max": 0, "moves": 0, "startup_cells": 0,
                                                 "startup_reach_m": None})

    def _move(self, correction, res: float) -> None:
        """Carry the memory from the old correction's map frame into the new one.

        A new cell is kept only if every old cell its transformed square can
        touch (the bounding box of its corners) was remembered (Codex P1: five
        points do not cover the square); it takes the oldest of them.
        """
        old_from_new = _compose(self.correction, _invert(correction))  # map_old <- odom <- map_new
        new_from_old = _invert(old_from_new)
        candidates = set()
        for i, j in self.memory:
            px, py, _ = _compose(new_from_old, ((i + 0.5) * res, (j + 0.5) * res, 0.0))
            ci, cj = int(np.floor(px / res)), int(np.floor(py / res))
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    candidates.add((ci + di, cj + dj))
        moved = {}
        for i, j in candidates:
            corners = [_compose(old_from_new, ((i + dx) * res, (j + dy) * res, 0.0)) for dx, dy in
                       ((0, 0), (1, 0), (0, 1), (1, 1))]
            xs = [p[0] for p in corners]
            ys = [p[1] for p in corners]
            i0, i1 = int(np.floor(min(xs) / res + 1e-9)), int(np.floor(max(xs) / res - 1e-9))
            j0, j1 = int(np.floor(min(ys) / res + 1e-9)), int(np.floor(max(ys) / res - 1e-9))
            values = []
            for a in range(i0, i1 + 1):
                for b in range(j0, j1 + 1):
                    if (a, b) not in self.memory:
                        break
                    values.append(self.memory[(a, b)])
                else:
                    continue
                break
            else:
                moved[(i, j)] = (min(v[0] for v in values), "moved", max(v[2] for v in values))
        self.memory = moved
        self.stats["moves"] += 1

    def apply(self, snapshot: GridSnapshot, own_pose, own_footprint: Footprint,
              extra_support: dict | None = None) -> GridSnapshot:
        """The snapshot with the band's supported remembered cells set to RETAINED.

        A RETAINED cell carries, as its FREE stamp, the oldest fresh FREE
        evidence its support relied on, so the permission's freshness limit
        expires it with that support (Codex P1); its radius is sized for the
        age it may reach within that limit.
        """
        res = snapshot.resolution_m
        oi = int(round(snapshot.origin_x_m / res))
        oj = int(round(snapshot.origin_y_m / res))
        if abs(oi * res - snapshot.origin_x_m) > 1e-6 or abs(oj * res - snapshot.origin_y_m) > 1e-6:
            raise ValueError("snapshot origin must be a multiple of the resolution")
        correction = tuple(float(v) for v in snapshot.correction)
        if self.correction is not None and correction != self.correction and self.memory:
            self._move(correction, res)
        self.correction = correction
        state = snapshot.state
        nx, ny = state.shape
        band = band_cells(snapshot, own_pose, own_footprint, self.band_m)
        whole = whole_cells(snapshot, own_pose, own_footprint)
        band -= whole
        now = snapshot.stamp_s

        def rho_of(a, b):
            return hypot(snapshot.origin_x_m + (a + 0.5) * res - own_pose[0],
                         snapshot.origin_y_m + (b + 0.5) * res - own_pose[1])

        if not self.started:
            # Operating premise: nothing inside these whole cells at start-up --
            # the outline grown by band_m and then by every cell it touches.
            self.started = True
            for a, b in band:
                if state[a, b] != OCCUPIED:
                    self.memory[(a + oi, b + oj)] = (now, "startup", rho_of(a, b))
            self.stats["startup_cells"] = len(self.memory)
            self.stats["startup_reach_m"] = self.band_m + res * 2 ** 0.5
        # Forget what left the band and the outline or turned OCCUPIED; record
        # fresh FREE with its own time. A cell wholly under the truck is empty
        # because the truck is there (static scope): it is recorded at this
        # snapshot, so a cell the truck backs off from keeps that evidence when
        # it re-emerges in the band (P0b: reversing stopped on the strip the
        # carriage had just left).
        keep = {}
        for (ai, aj), value in self.memory.items():
            a, b = ai - oi, aj - oj
            if (a, b) in band and state[a, b] == UNKNOWN:
                keep[(ai, aj)] = value
        for a, b in whole:
            if 0 <= a < nx and 0 <= b < ny and state[a, b] != OCCUPIED:
                keep[(a + oi, b + oj)] = (now, "covered", rho_of(a, b))
        for a, b in band:
            if state[a, b] == FREE:
                keep[(a + oi, b + oj)] = (float(snapshot.free_stamp[a, b]), "free", rho_of(a, b) + self.rho_margin_m)
        self.memory = keep
        # Candidates: remembered UNKNOWN band cells whose error stays in the
        # table over the next evidence_max_age_s.
        cand = {}
        for (ai, aj), (t_obs, _, rho) in self.memory.items():
            a, b = ai - oi, aj - oj
            if state[a, b] != UNKNOWN:
                continue
            bound = self.error.at(now - t_obs + self.evidence_max_age_s)
            if bound is None:
                continue
            e, psi = bound
            cand[(a, b)] = e + 2 * rho * sin(psi / 2) + res / 2
        # Greatest supported subset: drop unsupported candidates until none is left to drop.
        # extra_support: cells observed free by another sensor (the depth pocket
        # check), with that observation's time -- evidence, never an exemption.
        fresh = state == FREE
        stamps_fresh = snapshot.free_stamp
        if extra_support:
            fresh = fresh.copy()
            stamps_fresh = stamps_fresh.copy()
            for (a, b), t_cert in extra_support.items():
                if 0 <= a < nx and 0 <= b < ny and state[a, b] != OCCUPIED:
                    fresh[a, b] = True
                    if not np.isfinite(stamps_fresh[a, b]) or t_cert < stamps_fresh[a, b]:
                        stamps_fresh[a, b] = t_cert
        changed = True
        while changed and cand:
            changed = False
            for (a, b), r in list(cand.items()):
                if self._support(a, b, r, res, fresh, whole, cand, nx, ny) is None:
                    del cand[(a, b)]
                    changed = True
        if not cand:
            return snapshot
        # Each RETAINED cell inherits the oldest fresh FREE stamp along its support.
        stamp = {}
        for (a, b), r in cand.items():
            fresh_n, _ = self._support(a, b, r, res, fresh, whole, cand, nx, ny)
            stamp[(a, b)] = min((float(stamps_fresh[p, q]) for p, q in fresh_n), default=np.inf)
        changed = True
        while changed:
            changed = False
            for (a, b), r in cand.items():
                _, ret_n = self._support(a, b, r, res, fresh, whole, cand, nx, ny)
                low = min([stamp[(a, b)]] + [stamp[n] for n in ret_n])
                if low < stamp[(a, b)]:
                    stamp[(a, b)] = low
                    changed = True
        out = state.copy()
        free_stamp = snapshot.free_stamp.copy()
        for (a, b), t in stamp.items():
            if not np.isfinite(t):
                continue  # no fresh evidence anywhere along the support: not retained
            out[a, b] = RETAINED
            free_stamp[a, b] = t
        self.stats["retained_max"] = max(self.stats["retained_max"], int((out == RETAINED).sum()))
        return replace(snapshot, state=out, free_stamp=free_stamp)

    @staticmethod
    def _support(a, b, r, res, fresh, whole, cand, nx, ny):
        """(fresh FREE, candidate) cells within reach r of cell (a, b), or None
        if any cell in reach is neither those nor wholly inside the outline.
        Reach is square to square, so it already holds the cell's extent."""
        fresh_n, ret_n = [], []
        k = int(np.ceil(r / res)) + 1
        for di in range(-k, k + 1):
            for dj in range(-k, k + 1):
                if di == 0 and dj == 0:
                    continue
                # Smallest distance between the two cell squares.
                gap = res * hypot(max(abs(di) - 1, 0), max(abs(dj) - 1, 0))
                if gap > r:
                    continue
                p, q = a + di, b + dj
                if not (0 <= p < nx and 0 <= q < ny):
                    return None
                if fresh[p, q]:
                    fresh_n.append((p, q))
                elif (p, q) in cand:
                    ret_n.append((p, q))
                elif (p, q) not in whole:
                    return None
        return fresh_n, ret_n


def band_cells(snapshot: GridSnapshot, pose, footprint: Footprint, band_m: float) -> set:
    """In-grid cells overlapping the outline grown by band_m."""
    cells, _ = footprint_cells(snapshot, pose, footprint, band_m)
    nx, ny = snapshot.state.shape
    return {(int(a), int(b)) for a, b in cells if 0 <= a < nx and 0 <= b < ny}


def whole_cells(snapshot: GridSnapshot, pose, footprint: Footprint) -> set:
    """Cells of the outline whose four corners all lie inside it."""
    cells, _ = footprint_cells(snapshot, pose, footprint, 0.0)
    x, y, yaw = pose
    c, s = cos(yaw), sin(yaw)
    res = snapshot.resolution_m
    out = set()
    for a, b in cells:
        x0 = snapshot.origin_x_m + a * res
        y0 = snapshot.origin_y_m + b * res
        for dx, dy in ((0, 0), (res, 0), (0, res), (res, res)):
            px, py = x0 + dx - x, y0 + dy - y
            u = px * c + py * s
            w = -px * s + py * c
            if not (-footprint.rear_m <= u <= footprint.front_m and abs(w) <= footprint.half_width_m):
                break
        else:
            out.add((int(a), int(b)))
    return out


__all__ = ["RETAINED", "ShadowMemory", "band_cells", "whole_cells"]
