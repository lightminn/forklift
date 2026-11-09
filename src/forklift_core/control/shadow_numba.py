"""Compiled RETAINED judgement of the shadow-band memory (real-time plan 5.27 G3-1c, docs/plans/2026-10-17-realtime-sim.md):
``_retained_stamps_batched`` on the candidates' arrays, its result bit for bit. Each candidate's support test walks
its in-reach offsets once; the removal (the greatest set closed under losing a non-whole support) and the stamps (the
least over the alive support chain) are fixpoints, so the order of the updates does not change them. The minima pick
among stamps with no zero or NaN (the caller keeps the reference otherwise), whose bit pattern is unique. Imported
only when the ``numba`` kernels are chosen (``forklift_core.kernels``).
"""

from __future__ import annotations

import numpy as np
from numba import njit


@njit(cache=True, nogil=True)
def retained(a, b, r, fresh, stamps, whole, di, dj, gap):
    """(alive, stamp) per candidate, as ``_retained_stamps_batched`` decides them (a dead candidate's stamp is inf)."""
    nx, ny = fresh.shape
    n = len(a)
    k = len(di)
    cand_id = np.full((nx, ny), -1, dtype=np.int64)
    for m in range(n):
        cand_id[a[m], b[m]] = m
    alive = np.ones(n, dtype=np.bool_)
    first = np.full(n, np.inf)
    dep_start = np.zeros(n + 1, dtype=np.int64)
    dep = np.empty(n * k, dtype=np.int64)  # candidate neighbours, in offset order
    dep_withdraw = np.empty(n * k, dtype=np.bool_)  # the neighbour withdraws its support when it goes
    used = 0
    for m in range(n):
        dep_start[m] = used
        low = np.inf
        start = used
        for o in range(k):
            if not gap[o] <= r[m]:
                continue
            p = a[m] + di[o]
            q = b[m] + dj[o]
            if p < 0 or p >= nx or q < 0 or q >= ny:
                alive[m] = False
                continue
            if fresh[p, q]:
                s = stamps[p, q]
                if s < low:
                    low = s
                continue
            c = cand_id[p, q]
            if c >= 0:
                dep[used] = c
                dep_withdraw[used] = not whole[p, q]
                used += 1
            elif not whole[p, q]:
                alive[m] = False
        if alive[m]:
            first[m] = low
        else:
            used = start  # the batched code keeps no support row for a candidate dead from the start
    dep_start[n] = used
    # users: who loses its support when a candidate goes (withdrawing neighbours only)
    count = np.zeros(n + 1, dtype=np.int64)
    for m in range(n):
        for e in range(dep_start[m], dep_start[m + 1]):
            if dep_withdraw[e]:
                count[dep[e] + 1] += 1
    for m in range(n):
        count[m + 1] += count[m]
    users = np.empty(count[n], dtype=np.int64)
    fill = count[:n].copy()
    for m in range(n):
        for e in range(dep_start[m], dep_start[m + 1]):
            if dep_withdraw[e]:
                users[fill[dep[e]]] = m
                fill[dep[e]] += 1
    queue = np.empty(n, dtype=np.int64)
    top = 0
    for m in range(n):
        if not alive[m]:
            queue[top] = m
            top += 1
    while top:
        top -= 1
        gone = queue[top]
        for e in range(count[gone], count[gone + 1]):
            u = users[e]
            if alive[u]:
                alive[u] = False
                queue[top] = u
                top += 1
    stamp = first.copy()
    changed = True
    while changed:
        changed = False
        for m in range(n):
            if not alive[m]:
                continue
            low = stamp[m]
            for e in range(dep_start[m], dep_start[m + 1]):
                d = dep[e]
                if alive[d] and stamp[d] < low:
                    low = stamp[d]
            if low < stamp[m]:
                stamp[m] = low
                changed = True
    return alive, stamp
