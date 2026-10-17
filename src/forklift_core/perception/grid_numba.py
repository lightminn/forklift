"""Compiled obstacle-grid kernels (real-time plan S3, docs/plans/2026-10-17-realtime-sim.md): the beam mask, the
gap closing along rows, columns and diagonals, and the erosion and dilation by an offset list, each giving the
NumPy reference's result bit for bit (plan D2): the beam samples use the reference's own expressions, the rest is
integer and boolean work. Imported only when the ``numba`` kernels are chosen (``forklift_core.kernels``).
"""

from __future__ import annotations

import math

import numpy as np
from numba import njit


@njit(cache=True, nogil=True)
def ray_mask(nx, ny, x_min, y_min, res, lx, ly, cos_a, sin_a, lengths, s):
    """ObstacleGrid._ray_mask: cells crossed by each beam at the half-cell samples ``s`` up to its length."""
    mask = np.zeros((nx, ny), dtype=np.bool_)
    for b in range(len(lengths)):
        length, c, sn = lengths[b], cos_a[b], sin_a[b]
        for k in range(len(s)):
            if not s[k] <= length:
                break  # s rises: the rest of this beam's samples are past its length
            i = int(math.floor((lx + c * s[k] - x_min) / res))
            j = int(math.floor((ly + sn * s[k] - y_min) / res))
            if 0 <= i < nx and 0 <= j < ny:
                mask[i, j] = True
    return mask


@njit(cache=True, nogil=True)
def _close_line(occupied, out, ii, jj, max_gap):
    """Along one line of cells (ii, jj), fill every gap of at most max_gap cells between two occupied ones."""
    last = -1
    for k in range(len(ii)):
        if occupied[ii[k], jj[k]]:
            if last >= 0 and k - last - 1 <= max_gap:
                for m in range(last + 1, k):
                    out[ii[m], jj[m]] = True
            last = k


@njit(cache=True, nogil=True)
def line_close(occupied, max_gap, diag_gap):
    """_line_close: rows and columns with max_gap, both diagonals with diag_gap."""
    nx, ny = occupied.shape
    out = occupied.copy()
    for i in range(nx):
        _close_line(occupied, out, np.full(ny, i), np.arange(ny), max_gap)
    for j in range(ny):
        _close_line(occupied, out, np.arange(nx), np.full(nx, j), max_gap)
    for d in range(nx + ny - 1):  # i + j = d, by rising i
        i0, i1 = max(0, d - ny + 1), min(d, nx - 1)
        ii = np.arange(i0, i1 + 1)
        _close_line(occupied, out, ii, d - ii, diag_gap)
    for d in range(nx + ny - 1):  # i + (ny - 1 - j) = d, by rising i (the flipped grid's lines)
        i0, i1 = max(0, d - ny + 1), min(d, nx - 1)
        ii = np.arange(i0, i1 + 1)
        _close_line(occupied, out, ii, ny - 1 - (d - ii), diag_gap)
    return out


@njit(cache=True, nogil=True)
def erode(mask, offsets):
    """_erode with an offset list: mask cells all of whose listed neighbours are mask cells (outside: not)."""
    nx, ny = mask.shape
    out = np.zeros((nx, ny), dtype=np.bool_)
    for i in range(nx):
        for j in range(ny):
            if not mask[i, j]:
                continue
            ok = True
            for k in range(len(offsets)):
                p, q = i + offsets[k, 0], j + offsets[k, 1]
                if not (0 <= p < nx and 0 <= q < ny and mask[p, q]):
                    ok = False
                    break
            out[i, j] = ok
    return out


@njit(cache=True, nogil=True)
def dilate(mask, offsets):
    """_dilate with an offset list: mask cells and every cell a listed offset away from one."""
    nx, ny = mask.shape
    out = mask.copy()
    for i in range(nx):
        for j in range(ny):
            if not mask[i, j]:
                continue
            for k in range(len(offsets)):
                p, q = i - offsets[k, 0], j - offsets[k, 1]
                if 0 <= p < nx and 0 <= q < ny:
                    out[p, q] = True
    return out


def row_intervals(offsets):
    """The offsets as (di, lowest dj, highest dj) rows when every row is one contiguous run, else None."""
    rows = {}
    for di, dj in np.asarray(offsets, dtype=np.int64).tolist():
        rows.setdefault(di, []).append(dj)
    out = []
    for di in sorted(rows):
        djs = sorted(rows[di])
        if djs != list(range(djs[0], djs[-1] + 1)):
            return None
        out.append((di, djs[0], djs[-1]))
    return np.array(out, dtype=np.int64).reshape(-1, 3)


@njit(cache=True, nogil=True)
def erode_rows(mask, rows):
    """erode for offsets given as row runs: a mask cell stays when, for every run, the cells (i + di, j + lo ..
    j + hi) are all inside the grid and mask cells -- the same test, by the next non-mask cell of each row."""
    nx, ny = mask.shape
    nxt = np.empty((nx, ny + 1), dtype=np.int64)  # first non-mask column at or after j
    for i in range(nx):
        nxt[i, ny] = ny
        for j in range(ny - 1, -1, -1):
            nxt[i, j] = j if not mask[i, j] else nxt[i, j + 1]
    out = np.zeros((nx, ny), dtype=np.bool_)
    for i in range(nx):
        for j in range(ny):
            if not mask[i, j]:
                continue
            ok = True
            for r in range(len(rows)):
                p, lo, hi = i + rows[r, 0], j + rows[r, 1], j + rows[r, 2]
                if p < 0 or p >= nx or lo < 0 or hi >= ny or nxt[p, lo] <= hi:
                    ok = False
                    break
            out[i, j] = ok
    return out


@njit(cache=True, nogil=True)
def dilate_rows(mask, rows):
    """dilate for offsets given as row runs: out[i, j] is mask[i, j] or any mask cell (i + di, j + lo .. j + hi)
    inside the grid -- the same test, by row prefix counts."""
    nx, ny = mask.shape
    count = np.zeros((nx, ny + 1), dtype=np.int64)
    for i in range(nx):
        for j in range(ny):
            count[i, j + 1] = count[i, j] + (1 if mask[i, j] else 0)
    out = mask.copy()
    for i in range(nx):
        for j in range(ny):
            if out[i, j]:
                continue
            for r in range(len(rows)):
                p = i + rows[r, 0]
                if p < 0 or p >= nx:
                    continue
                lo, hi = max(j + rows[r, 1], 0), min(j + rows[r, 2], ny - 1)
                if lo <= hi and count[p, hi + 1] - count[p, lo] > 0:
                    out[i, j] = True
                    break
    return out
