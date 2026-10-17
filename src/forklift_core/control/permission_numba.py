"""Compiled drive-permission walk (real-time plan S3, docs/plans/2026-10-17-realtime-sim.md): the reference walk of
``DrivePermission._walk_reference`` one check at a time, with every element's expression as the reference writes
it, so the result is the reference's bit for bit (plan D2; checked by tests/unit/control/test_permission_kernels.py
and tools/perf_bench.py on recorded calls).

The work it saves: no NumPy call per check, and a partly own cell is decided at its first sub-sample inside the
checked shape (the reference tests them all, then asks whether any is in). Imported only when the ``numba``
kernels are chosen (``forklift_core.kernels``); compiled on first use and cached on disk.
"""

from __future__ import annotations

import math

import numpy as np
from numba import njit

OCCUPIED, FREE, RETAINED = 2, 1, 3
EDGE, BLOCK_OCCUPIED, BLOCK_UNKNOWN, NEEDS_REFERENCE = 1, 2, 3, -1


@njit(cache=True, nogil=True)
def _cells(ox, oy, res, nx, ny, x, y, c, s, margin, direction, p_front, p_rear, p_hw, p_lat, p_lon):
    """shape_cells: (cells (n, 2) in its order, any outside, a cell beyond the union key's range)."""
    nparts = len(p_front)
    half = res / 2
    total = 0
    bounds = np.empty((nparts, 4), dtype=np.int64)
    scal = np.empty((nparts, 8))
    for p in range(nparts):
        front = p_front[p] + (margin if direction >= 0 else 0.0)
        rear = p_rear[p] + (margin if direction <= 0 else 0.0)
        offset = (front - rear) / 2 + p_lon[p]
        cx = x + offset * c - p_lat[p] * s
        cy = y + offset * s + p_lat[p] * c
        hl = (front + rear) / 2
        hw = p_hw[p] + margin
        ex = abs(c) * hl + abs(s) * hw
        ey = abs(s) * hl + abs(c) * hw
        i0 = int(math.floor((cx - ex - ox) / res)) - 1
        i1 = int(math.ceil((cx + ex - ox) / res)) + 1
        j0 = int(math.floor((cy - ey - oy) / res)) - 1
        j1 = int(math.ceil((cy + ey - oy) / res)) + 1
        proj = half * (abs(c) + abs(s))
        bounds[p, 0], bounds[p, 1], bounds[p, 2], bounds[p, 3] = i0, i1, j0, j1
        scal[p, 0], scal[p, 1], scal[p, 2], scal[p, 3] = cx, cy, ex + half, ey + half
        scal[p, 4], scal[p, 5] = hl + proj, hw + proj
        total += (i1 - i0) * (j1 - j0)
    out = np.empty((total, 2), dtype=np.int64)
    n = 0
    for p in range(nparts):
        cx, cy, exh, eyh, hlp, hwp = scal[p, 0], scal[p, 1], scal[p, 2], scal[p, 3], scal[p, 4], scal[p, 5]
        for ii in range(bounds[p, 0], bounds[p, 1]):
            sx = ox + (ii + 0.5) * res
            dx = sx - cx
            if not abs(dx) <= exh:
                continue
            for jj in range(bounds[p, 2], bounds[p, 3]):
                sy = oy + (jj + 0.5) * res
                dy = sy - cy
                if (abs(dy) <= eyh) and (abs(dx * c + dy * s) <= hlp) and (abs(-dx * s + dy * c) <= hwp):
                    out[n, 0] = ii
                    out[n, 1] = jj
                    n += 1
    cells = out[:n]
    outside = False
    big = False
    for k in range(n):
        if cells[k, 0] < 0 or cells[k, 0] >= nx or cells[k, 1] < 0 or cells[k, 1] >= ny:
            outside = True
        if abs(cells[k, 0]) >= (1 << 20) or abs(cells[k, 1]) >= (1 << 20):
            big = True
    if nparts > 1 and n:
        if big:
            return cells, outside, True
        off, span = 1 << 20, 1 << 21
        keys = np.empty(n, dtype=np.int64)
        for k in range(n):
            keys[k] = (cells[k, 0] + off) * span + (cells[k, 1] + off)
        keys = np.unique(keys)
        cells = np.empty((len(keys), 2), dtype=np.int64)
        for k in range(len(keys)):
            cells[k, 0] = keys[k] // span - off
            cells[k, 1] = keys[k] % span - off
    return cells, outside, False


@njit(cache=True, nogil=True)
def walk(state, free_stamp, ox, oy, res, xs, ys, cs, ss, svals, margins, direction, unknown_passes,
         p_front, p_rear, p_hw, p_lat, p_lon, has_own, oi0, oj0, whole_box, pid_box, pts, pstart, pcount,
         own_pose_given, band):
    """(verified, blocked code, oldest, first_unknown (nan: none), blocked cells (k, 2))."""
    nx, ny = state.shape
    verified, blocked, oldest = 0.0, 0, np.inf
    first_unknown = np.nan
    blocked_cells = np.empty((0, 2), dtype=np.int64)
    nparts = len(p_front)
    for k in range(len(xs)):
        x, y, c, s, sv, margin = xs[k], ys[k], cs[k], ss[k], svals[k], margins[k]
        cells, outside, big = _cells(ox, oy, res, nx, ny, x, y, c, s, margin, direction,
                                     p_front, p_rear, p_hw, p_lat, p_lon)
        if big:
            return verified, NEEDS_REFERENCE, oldest, first_unknown, blocked_cells
        if outside:
            if not unknown_passes:
                blocked = EDGE
                break
            if math.isnan(first_unknown):
                first_unknown = sv
        keep = np.zeros(len(cells), dtype=np.bool_)
        for q in range(len(cells)):
            i, j = cells[q, 0], cells[q, 1]
            if i < 0 or i >= nx or j < 0 or j >= ny:
                continue
            if has_own:
                bi, bj = i - oi0, j - oj0
                inbox = 0 <= bi < whole_box.shape[0] and 0 <= bj < whole_box.shape[1]
                if inbox and whole_box[bi, bj]:
                    continue
                pid = pid_box[bi, bj] if inbox else -1
                if pid >= 0 and own_pose_given:
                    reach = False
                    for m in range(pstart[pid], pstart[pid] + pcount[pid]):
                        dxp = pts[m, 0] - x
                        dyp = pts[m, 1] - y
                        u = dxp * c + dyp * s
                        v = -dxp * s + dyp * c
                        for p in range(nparts):
                            front = p_front[p] + (margin if direction >= 0 else 0.0)
                            rear = p_rear[p] + (margin if direction <= 0 else 0.0)
                            w = u - p_lon[p]
                            if (w >= -rear) and (w <= front) and (abs(v - p_lat[p]) <= p_hw[p] + margin):
                                reach = True
                                break
                        if reach:
                            break
                    if not reach:
                        continue
            keep[q] = True
        kept = cells[keep]
        occupied = 0
        for q in range(len(kept)):
            if state[kept[q, 0], kept[q, 1]] == OCCUPIED:
                occupied += 1
        if occupied:
            blocked = BLOCK_OCCUPIED
            blocked_cells = np.empty((min(occupied, 20), 2), dtype=np.int64)
            n = 0
            for q in range(len(kept)):
                if n < 20 and state[kept[q, 0], kept[q, 1]] == OCCUPIED:
                    blocked_cells[n, 0], blocked_cells[n, 1] = kept[q, 0], kept[q, 1]
                    n += 1
            break
        bad = 0
        lowest = np.nan
        any_seen = False
        for q in range(len(kept)):
            st = state[kept[q, 0], kept[q, 1]]
            if st == RETAINED and not band[kept[q, 0], kept[q, 1]]:
                st = 0
            if st != FREE and st != RETAINED:
                bad += 1
            else:
                value = free_stamp[kept[q, 0], kept[q, 1]]
                if value == 0.0 or math.isnan(value):
                    # Which zero, or NaN, np.min keeps depends on its reduction order: the reference decides.
                    return verified, NEEDS_REFERENCE, oldest, first_unknown, blocked_cells
                if not any_seen:
                    lowest = value
                    any_seen = True
                elif not math.isnan(lowest) and (math.isnan(value) or value < lowest):
                    lowest = value
        if bad:
            if not unknown_passes:
                blocked = BLOCK_UNKNOWN
                blocked_cells = np.empty((min(bad, 20), 2), dtype=np.int64)
                n = 0
                for q in range(len(kept)):
                    st = state[kept[q, 0], kept[q, 1]]
                    if st == RETAINED and not band[kept[q, 0], kept[q, 1]]:
                        st = 0
                    if n < 20 and st != FREE and st != RETAINED:
                        blocked_cells[n, 0], blocked_cells[n, 1] = kept[q, 0], kept[q, 1]
                        n += 1
                break
            if math.isnan(first_unknown):
                first_unknown = sv
        if any_seen and lowest < oldest:
            oldest = lowest
        if math.isnan(first_unknown):
            verified = sv
    return verified, blocked, oldest, first_unknown, blocked_cells


@njit(cache=True, nogil=True)
def own_lookup(ox, oy, res, nx, ny, x, y, c, s, o_front, o_rear, o_hw, o_lat, o_lon):
    """DrivePermission._own_cells and _own_lookup in one pass, as a box around the outline: (box origin i, j,
    wholly own (bi, bj), partial id (bi, bj) or -1, outside sub-samples (m, 2), start and count per partial cell,
    any own cell). The same cells, the same corner and sub-sample tests (in_shape's expressions)."""
    cells, _, _ = _cells(ox, oy, res, nx, ny, x, y, c, s, 0.0, 0, o_front, o_rear, o_hw, o_lat, o_lon)
    n = len(cells)
    if n == 0:
        return (0, 0, np.zeros((1, 1), dtype=np.bool_), np.full((1, 1), -1, dtype=np.int64), np.zeros((0, 2)),
                np.zeros(0, dtype=np.int64), np.zeros(0, dtype=np.int64), False)
    i0, i1, j0, j1 = cells[0, 0], cells[0, 0], cells[0, 1], cells[0, 1]
    for k in range(n):
        i0, i1 = min(i0, cells[k, 0]), max(i1, cells[k, 0])
        j0, j1 = min(j0, cells[k, 1]), max(j1, cells[k, 1])
    whole = np.zeros((i1 - i0 + 1, j1 - j0 + 1), dtype=np.bool_)
    pid = np.full((i1 - i0 + 1, j1 - j0 + 1), -1, dtype=np.int64)
    nparts = len(o_front)
    partial = np.empty((n, 2), dtype=np.int64)
    npartial = 0
    for k in range(n):
        a, b = cells[k, 0], cells[k, 1]
        x0 = ox + a * res
        y0 = oy + b * res
        inside = False
        for p in range(nparts):
            in_part = True
            for corner in range(4):
                px = x0 + (res if corner == 1 or corner == 3 else 0)
                py = y0 + (res if corner >= 2 else 0)
                u = (px - x) * c + (py - y) * s
                v = -(px - x) * s + (py - y) * c
                w = u - o_lon[p]
                if not ((w >= -o_rear[p]) and (w <= o_front[p]) and (abs(v - o_lat[p]) <= o_hw[p])):
                    in_part = False
                    break
            if in_part:
                inside = True
                break
        if not (0 <= a < nx and 0 <= b < ny):
            continue  # the lookups hold in-grid cells only
        if inside:
            whole[a - i0, b - j0] = True
        else:
            partial[npartial, 0], partial[npartial, 1] = a, b
            npartial += 1
    pts = np.empty((npartial * 121, 2))
    pstart = np.empty(npartial, dtype=np.int64)
    pcount = np.empty(npartial, dtype=np.int64)
    m = 0
    for q in range(npartial):
        a, b = partial[q, 0], partial[q, 1]
        pid[a - i0, b - j0] = q
        pstart[q] = m
        for gi in range(11):
            for gj in range(11):
                px = ox + a * res + (gi / 10) * res
                py = oy + b * res + (gj / 10) * res
                u = (px - x) * c + (py - y) * s
                v = -(px - x) * s + (py - y) * c
                covered = False
                for p in range(nparts):
                    w = u - o_lon[p]
                    if (w >= -o_rear[p]) and (w <= o_front[p]) and (abs(v - o_lat[p]) <= o_hw[p]):
                        covered = True
                        break
                if not covered:
                    pts[m, 0], pts[m, 1] = px, py
                    m += 1
        pcount[q] = m - pstart[q]
    return i0, j0, whole, pid, pts[:m], pstart, pcount, True
