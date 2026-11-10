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

import os
from dataclasses import dataclass, field, replace
from itertools import chain
from math import cos, hypot, sin

import numpy as np

from forklift_core import kernels
from forklift_core.control.drive_permission import RETAINED, footprint_cells
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, UNKNOWN, AgeErrorTable, GridSnapshot
from forklift_core.perf import changed, dual, dumped, shape_kind
from forklift_core.planning.geometry import Footprint

# Real-time plan 5.27 G3-1c: the shadow apply on arrays, bit for bit the original including the memory's key order and
# value types (FORKLIFT_SHADOW_KERNEL=1, read once at start; off by default). Kernel mode ``reference`` keeps the
# original path; ``numba`` judges RETAINED with control/shadow_numba.py, ``numpy`` with _retained_stamps_batched.
SHADOW_KERNEL = os.environ.get("FORKLIFT_SHADOW_KERNEL", "0") == "1"


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

    @dumped("shadow_apply", strata=lambda self_, a, k, r: (
        shape_kind(a[2]), bool(k.get("extra_support") or (a[3] if len(a) > 3 else None)),
        changed("shadow_apply.correction", tuple(float(v) for v in a[0].correction)),
        bool((r.state == RETAINED).any())))
    @dual("shadow_apply", carried=True)
    def apply(self, snapshot: GridSnapshot, own_pose, own_footprint: Footprint,
              extra_support: dict | None = None) -> GridSnapshot:
        """The snapshot with the band's supported remembered cells set to RETAINED.

        A RETAINED cell carries, as its FREE stamp, the oldest fresh FREE
        evidence its support relied on, so the permission's freshness limit
        expires it with that support (Codex P1); its radius is sized for the
        age it may reach within that limit.
        """
        if SHADOW_KERNEL and kernels.mode() != "reference" and self.started and self._exact_floats(snapshot, own_pose):
            return self._apply_arrays(snapshot, own_pose, own_footprint, extra_support)
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
        # A zero or NaN among the stamps the minima may pick: the reference, whose reduction order decides which
        # zero (+0.0 or -0.0) is kept (Codex real-time round 3 P1); a nonzero finite minimum has one bit pattern.
        zero_or_nan = bool(np.any(fresh & ((stamps_fresh == 0.0) | np.isnan(stamps_fresh))))
        retained_stamps = self._retained_stamps if kernels.mode() == "reference" or zero_or_nan else \
            _retained_stamps_batched
        stamp = retained_stamps(cand, res, fresh, stamps_fresh, whole, nx, ny)
        if not stamp:
            return snapshot
        out = state.copy()
        free_stamp = snapshot.free_stamp.copy()
        for (a, b), t in stamp.items():
            if not np.isfinite(t):
                continue  # no fresh evidence anywhere along the support: not retained
            out[a, b] = RETAINED
            free_stamp[a, b] = t
        self.stats["retained_max"] = max(self.stats["retained_max"], int((out == RETAINED).sum()))
        return replace(snapshot, state=out, free_stamp=free_stamp)

    def _exact_floats(self, snapshot: GridSnapshot, own_pose) -> bool:
        """Every scalar the array path computes with is a built-in float (plan 5.27, Codex round 2-3): then a float64
        array operation is the same binary64 operation as the original scalar one. Otherwise the original path."""
        e = self.error
        scalars = (snapshot.stamp_s, snapshot.origin_x_m, snapshot.origin_y_m, snapshot.resolution_m, own_pose[0],
                   own_pose[1], self.band_m, self.evidence_max_age_s, self.rho_margin_m, *e.ages_s, *e.position_m,
                   *e.yaw_rad)
        if any(type(v) is not float for v in scalars):
            return False
        if snapshot.free_stamp.dtype != np.float64:  # its values are stored and handed to the kernels as float64
            return False
        return all(type(v[0]) is float and type(v[2]) is float for v in self.memory.values())

    def _apply_arrays(self, snapshot: GridSnapshot, own_pose, own_footprint: Footprint,
                      extra_support: dict | None) -> GridSnapshot:
        """``apply`` after start-up on arrays (plan 5.27 method A): the same cells, values, value types and memory key
        order. The new memory is built by the original's three insertions in order (kept memory, wholly covered cells,
        fresh FREE band cells; a key already present keeps its place); each distance is ``math.hypot`` of arguments
        computed by the original expressions; the candidates' radius uses ``math.sin`` of the table rows used."""
        res = snapshot.resolution_m
        oi = int(round(snapshot.origin_x_m / res))
        oj = int(round(snapshot.origin_y_m / res))
        if abs(oi * res - snapshot.origin_x_m) > 1e-6 or abs(oj * res - snapshot.origin_y_m) > 1e-6:
            raise ValueError("snapshot origin must be a multiple of the resolution")
        correction = tuple(float(v) for v in snapshot.correction)
        if self.correction is not None and correction != self.correction and self.memory:
            self._move(correction, res)  # the original (plan 5.27: its share stayed under the 5 % rule)
        self.correction = correction
        state = snapshot.state
        nx, ny = state.shape
        cells, _ = footprint_cells(snapshot, own_pose, own_footprint, self.band_m)
        band = _cell_set(cells, nx, ny)
        whole = whole_cells(snapshot, own_pose, own_footprint)
        band -= whole
        now = snapshot.stamp_s
        ox, oy, px, py = snapshot.origin_x_m, snapshot.origin_y_m, own_pose[0], own_pose[1]

        def rho(a, b) -> list:
            return list(map(hypot, (ox + (a + 0.5) * res - px).tolist(), (oy + (b + 0.5) * res - py).tolist()))

        band_mask = np.zeros((nx, ny), dtype=bool)
        if band:
            ba = np.fromiter((c[0] for c in band), dtype=np.int64, count=len(band))
            bb = np.fromiter((c[1] for c in band), dtype=np.int64, count=len(band))
            band_mask[ba, bb] = True
        else:
            ba = bb = np.zeros(0, dtype=np.int64)
        # 1. remembered cells still in the band and UNKNOWN, in the memory's order (their original key and value)
        keys = list(self.memory)
        values = list(self.memory.values())
        if keys:
            k = np.fromiter(chain.from_iterable(keys), dtype=np.int64, count=2 * len(keys)).reshape(-1, 2)
            a, b = k[:, 0] - oi, k[:, 1] - oj
            inside = (a >= 0) & (a < nx) & (b >= 0) & (b < ny)  # no negative-index wrap (plan 5.27, agy 6)
            ac, bc = np.where(inside, a, 0), np.where(inside, b, 0)
            stay = np.flatnonzero(inside & band_mask[ac, bc] & (state[ac, bc] == UNKNOWN)).tolist()
            keep = {keys[i]: values[i] for i in stay}
        else:
            keep = {}
        # 2. cells wholly under the truck, not OCCUPIED, in the whole set's order
        if whole:
            w = np.fromiter(chain.from_iterable(whole), dtype=np.int64, count=2 * len(whole)).reshape(-1, 2)
            wa, wb = w[:, 0], w[:, 1]
            ok = (wa >= 0) & (wa < nx) & (wb >= 0) & (wb < ny)
            wa, wb = wa[ok], wb[ok]
            sel = state[wa, wb] != OCCUPIED
            wa, wb = wa[sel], wb[sel]
            keep.update(zip(zip((wa + oi).tolist(), (wb + oj).tolist()),
                            ((now, "covered", r) for r in rho(wa, wb))))
        # 3. fresh FREE band cells, in the band set's order (band -= whole: the reference's set, iterated alike)
        if band:
            sel = state[ba, bb] == FREE
            fa, fb = ba[sel], bb[sel]
            margin = self.rho_margin_m
            keep.update(zip(zip((fa + oi).tolist(), (fb + oj).tolist()),
                            ((float(t), "free", r + margin) for t, r in zip(snapshot.free_stamp[fa, fb].tolist(),
                                                                       rho(fa, fb)))))
        self.memory = keep
        # Candidates: remembered UNKNOWN cells whose error stays in the table over the next evidence_max_age_s.
        keys = list(keep)
        if keys:
            k = np.fromiter(chain.from_iterable(keys), dtype=np.int64, count=2 * len(keys)).reshape(-1, 2)
            ca, cb = k[:, 0] - oi, k[:, 1] - oj  # every remembered cell is in the grid now (band or whole)
            unk = state[ca, cb] == UNKNOWN
            vals = list(keep.values())
            t_obs = np.array([v[0] for v in vals])[unk]
            rho_c = np.array([v[2] for v in vals])[unk]
            ca, cb = ca[unk], cb[unk]
            age = now - t_obs + self.evidence_max_age_s
            age = np.where(age < 0, 0.0, age)
            table = np.array(self.error.ages_s)
            le = age[:, None] <= table[None, :]
            found = le.any(axis=1)
            row = le.argmax(axis=1)[found]
            ca, cb, rho_c = ca[found], cb[found], rho_c[found]
            e = np.array(self.error.position_m)[row]
            yaw = self.error.yaw_rad
            half_sin = {i: sin(yaw[i] / 2) for i in set(row.tolist())}  # only the rows used (Codex P3)
            s_row = np.array([half_sin[i] for i in row.tolist()]) if len(row) else np.zeros(0)
            radius = e + 2 * rho_c * s_row + res / 2
        else:
            ca = cb = np.zeros(0, dtype=np.int64)
            radius = np.zeros(0)
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
        zero_or_nan = bool(np.any(fresh & ((stamps_fresh == 0.0) | np.isnan(stamps_fresh))))
        if not len(ca):
            return snapshot
        # A non-finite radius: the batched code's max() over the candidates decides (Codex 1c-0 review P3).
        if kernels.mode() == "numba" and not zero_or_nan and bool(np.isfinite(radius).all()):
            from forklift_core.control import shadow_numba

            whole_mask = np.zeros((nx, ny), dtype=bool)
            if whole:
                whole_mask[_whole_index(whole, nx, ny)] = True
            di, dj, gap = _offsets(float(radius.max()), res)
            alive, stamp_arr = shadow_numba.retained(ca, cb, radius, np.ascontiguousarray(fresh),
                                                     np.ascontiguousarray(stamps_fresh), whole_mask, di, dj, gap)
            idx = np.flatnonzero(alive)
            sa, sb, st = ca[idx], cb[idx], stamp_arr[idx]
        else:
            cand = dict(zip(zip(ca.tolist(), cb.tolist()), radius.tolist()))
            retained_stamps = self._retained_stamps if zero_or_nan else _retained_stamps_batched
            stamp = retained_stamps(cand, res, fresh, stamps_fresh, whole, nx, ny)
            if not stamp:
                return snapshot
            sa = np.array([c[0] for c in stamp], dtype=np.int64)
            sb = np.array([c[1] for c in stamp], dtype=np.int64)
            st = np.array(list(stamp.values()))
        if not len(sa):
            return snapshot
        out = state.copy()
        free_stamp = snapshot.free_stamp.copy()
        finite = np.isfinite(st)  # no fresh evidence anywhere along the support: not retained
        out[sa[finite], sb[finite]] = RETAINED
        free_stamp[sa[finite], sb[finite]] = st[finite]
        self.stats["retained_max"] = max(self.stats["retained_max"], int((out == RETAINED).sum()))
        return replace(snapshot, state=out, free_stamp=free_stamp)

    @staticmethod
    def _retained_stamps(cand, res, fresh, stamps_fresh, whole, nx, ny) -> dict:
        """The greatest supported subset of the candidates and each one's stamp.

        The same rule as repeated ``_support`` calls -- every cell within reach
        is fresh FREE, a still-supported candidate or wholly inside the outline,
        and a candidate takes the oldest fresh stamp along its support chain --
        computed once per cell with arrays and a removal queue (the repeated
        Python scan was 27 % of a mission's wall time, L3c v50 profile).
        """
        if not cand:
            return {}
        keys = list(cand)
        index = {key: n for n, key in enumerate(keys)}
        cand_id = np.full((nx, ny), -1, dtype=np.int64)
        for n, (a, b) in enumerate(keys):
            cand_id[a, b] = n
        whole_mask = np.zeros((nx, ny), dtype=bool)
        if whole:
            w = np.array([c for c in whole if 0 <= c[0] < nx and 0 <= c[1] < ny], dtype=np.int64).reshape(-1, 2)
            whole_mask[w[:, 0], w[:, 1]] = True
        k = int(np.ceil(max(cand.values()) / res)) + 1
        offsets = [(i, j) for i in range(-k, k + 1) for j in range(-k, k + 1) if (i, j) != (0, 0)]
        di = np.array([o[0] for o in offsets])
        dj = np.array([o[1] for o in offsets])
        # The same float as _support (math.hypot), so a cell at the reach edge
        # is judged alike (Codex checkpoint 10 P2: np.hypot differed in the last bit).
        gap = np.array([res * hypot(max(abs(i) - 1, 0), max(abs(j) - 1, 0)) for i, j in offsets])
        alive = np.ones(len(keys), dtype=bool)
        deps = [None] * len(keys)  # candidate neighbours a cell takes its stamp from
        first = np.full(len(keys), np.inf)  # oldest fresh stamp in reach
        users = [[] for _ in keys]  # who loses its support when a candidate goes
        for n, (a, b) in enumerate(keys):
            m = gap <= cand[(a, b)]
            p, q = a + di[m], b + dj[m]
            if ((p < 0) | (p >= nx) | (q < 0) | (q >= ny)).any():
                alive[n] = False
                continue
            fr = fresh[p, q]
            cid = cand_id[p, q]
            ret = ~fr & (cid >= 0)
            if (~fr & ~ret & ~whole_mask[p, q]).any():
                alive[n] = False
                continue
            if fr.any():
                first[n] = float(np.min(stamps_fresh[p[fr], q[fr]]))
            deps[n] = cid[ret]
            # A candidate wholly inside the outline still supports when it is
            # dropped (_support accepts whole cells): only the others withdraw
            # support (Codex checkpoint 10 P1).
            for d in cid[ret & ~whole_mask[p, q]]:
                users[d].append(n)
        # Removing a candidate withdraws the support of every cell relying on it,
        # unless that candidate is also wholly inside the outline.
        queue = [n for n in range(len(keys)) if not alive[n]]
        while queue:
            gone = queue.pop()
            for u in users[gone]:
                if alive[u]:
                    alive[u] = False
                    queue.append(u)
        stamp = first.copy()
        changed = True
        while changed:
            changed = False
            for n in range(len(keys)):
                if not alive[n] or deps[n] is None or not len(deps[n]):
                    continue
                live = deps[n][alive[deps[n]]]  # a dropped whole candidate gives no stamp
                if not len(live):
                    continue
                low = min(stamp[n], float(np.min(stamp[live])))
                if low < stamp[n]:
                    stamp[n] = low
                    changed = True
        return {keys[n]: float(stamp[n]) for n in range(len(keys)) if alive[n]}

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


def _cell_set(cells, nx, ny) -> set:
    """band_cells' set from footprint_cells' cells: the same in-grid cells added one by one in the same order as its
    set comprehension, so the same set iterating alike (plan 5.27; checked on 588 recorded calls)."""
    c = np.asarray(cells, dtype=np.int64).reshape(-1, 2)
    m = (c[:, 0] >= 0) & (c[:, 0] < nx) & (c[:, 1] >= 0) & (c[:, 1] < ny)
    return set(zip(c[m, 0].tolist(), c[m, 1].tolist()))


def _whole_index(whole, nx, ny):
    w = np.array([c for c in whole if 0 <= c[0] < nx and 0 <= c[1] < ny], dtype=np.int64).reshape(-1, 2)
    return w[:, 0], w[:, 1]


_OFFSETS: dict = {}


def _offsets(r_max: float, res: float):
    """The batched code's offset list for a largest reach, with its gaps by ``math.hypot`` (cached per size)."""
    k = int(np.ceil(r_max / res)) + 1
    key = (k, res)
    if key not in _OFFSETS:
        offsets = [(i, j) for i in range(-k, k + 1) for j in range(-k, k + 1) if (i, j) != (0, 0)]
        _OFFSETS[key] = (np.array([o[0] for o in offsets], dtype=np.int64), np.array([o[1] for o in offsets],
                         dtype=np.int64), np.array([res * hypot(max(abs(i) - 1, 0), max(abs(j) - 1, 0))
                                                    for i, j in offsets]))
    return _OFFSETS[key]


def band_cells(snapshot: GridSnapshot, pose, footprint: Footprint, band_m: float) -> set:
    """In-grid cells overlapping the outline grown by band_m."""
    cells, _ = footprint_cells(snapshot, pose, footprint, band_m)
    nx, ny = snapshot.state.shape
    return {(int(a), int(b)) for a, b in cells if 0 <= a < nx and 0 <= b < ny}


def whole_cells(snapshot: GridSnapshot, pose, footprint: Footprint) -> set:
    """Cells of the outline whose four corners all lie inside it."""
    cells, _ = footprint_cells(snapshot, pose, footprint, 0.0)
    if kernels.mode() != "reference":
        return _whole_cells_batched(snapshot, pose, footprint, cells)
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


def _whole_cells_batched(snapshot: GridSnapshot, pose, footprint: Footprint, cells) -> set:
    """whole_cells with every corner judged in one pass by the same expressions (real-time plan S1); the set is
    filled in the reference's order, so it iterates alike."""
    x, y, yaw = pose
    c, s = cos(yaw), sin(yaw)
    res = snapshot.resolution_m
    x0 = snapshot.origin_x_m + cells[:, 0] * res
    y0 = snapshot.origin_y_m + cells[:, 1] * res
    inside = np.ones(len(cells), dtype=bool)
    for dx, dy in ((0, 0), (res, 0), (0, res), (res, res)):
        px, py = x0 + dx - x, y0 + dy - y
        u = px * c + py * s
        w = -px * s + py * c
        inside &= (-footprint.rear_m <= u) & (u <= footprint.front_m) & (np.abs(w) <= footprint.half_width_m)
    return set(map(tuple, cells[inside].tolist()))


def _retained_stamps_batched(cand, res, fresh, stamps_fresh, whole, nx, ny) -> dict:
    """ShadowMemory._retained_stamps on arrays (real-time plan S1): the same support test per candidate, then
    the removal and the stamps as fixpoints -- removal is the greatest set closed under losing a non-whole
    support, a stamp the least over the alive support chain, so the order of the updates does not change them."""
    if not cand:
        return {}
    keys = list(cand)
    a = np.array([k[0] for k in keys], dtype=np.int64)
    b = np.array([k[1] for k in keys], dtype=np.int64)
    r = np.array([cand[k] for k in keys], dtype=float)
    cand_id = np.full((nx, ny), -1, dtype=np.int64)
    cand_id[a, b] = np.arange(len(keys))
    whole_mask = np.zeros((nx, ny), dtype=bool)
    if whole:
        w = np.array([c for c in whole if 0 <= c[0] < nx and 0 <= c[1] < ny], dtype=np.int64).reshape(-1, 2)
        whole_mask[w[:, 0], w[:, 1]] = True
    k = int(np.ceil(max(cand.values()) / res)) + 1
    offsets = [(i, j) for i in range(-k, k + 1) for j in range(-k, k + 1) if (i, j) != (0, 0)]
    di = np.array([o[0] for o in offsets])
    dj = np.array([o[1] for o in offsets])
    gap = np.array([res * hypot(max(abs(i) - 1, 0), max(abs(j) - 1, 0)) for i, j in offsets])
    reach = gap[None, :] <= r[:, None]
    p, q = a[:, None] + di[None, :], b[:, None] + dj[None, :]
    off_grid = reach & ((p < 0) | (p >= nx) | (q < 0) | (q >= ny))
    pc, qc = np.clip(p, 0, nx - 1), np.clip(q, 0, ny - 1)
    fr = reach & fresh[pc, qc]
    cid = cand_id[pc, qc]
    ret = reach & ~fresh[pc, qc] & (cid >= 0)
    wm = whole_mask[pc, qc]
    unsupported = reach & ~fresh[pc, qc] & ~ret & ~wm
    alive = ~(off_grid.any(axis=1) | unsupported.any(axis=1))
    first = np.where(alive, np.min(np.where(fr, stamps_fresh[pc, qc], np.inf), axis=1), np.inf)
    ret &= alive[:, None]
    withdraw = ret & ~wm  # a dropped candidate wholly inside the outline still supports
    dep = np.where(ret, cid, 0)
    while True:
        lost = (withdraw & ~alive[dep]).any(axis=1)
        if not (alive & lost).any():
            break
        alive &= ~lost
    stamp = first.copy()
    while True:
        low = np.min(np.where(ret & alive[dep], stamp[dep], np.inf), axis=1)
        update = alive & (low < stamp)
        if not update.any():
            break
        stamp[update] = low[update]
    return {keys[n]: float(stamp[n]) for n in range(len(keys)) if alive[n]}


__all__ = ["RETAINED", "ShadowMemory", "band_cells", "whole_cells"]
