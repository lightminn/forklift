"""All open crossing orders at once: a mixed-integer solve of fleet_windows' timing.

Plan: docs/plans/2026-10-16-fleet-crossing-queues.md (3). The model is
fleet_windows.timed with every open order a choice:

- Each truck n has a delay d[n, j] from each of its checkpoints on (a
  crossing's first and last sample, on the route of the truck that may go
  second there), non-decreasing along the route and non-negative.
- If f goes first at crossing c, the second s reaches checkpoint `at` no
  earlier than f passes the matching release point:
  free_s(at) + d[s, at] >= free_f(release) + d[f, release] (+ park_s where f
  parks there), with the big-M switched off for the other order.
- Orders already fixed (a token out, a truck in) are constraints.
- Minimise the sum over trucks of weight x delay at the route's end (+ a
  little of every delay, so none is larger than it needs to be).

Stop costs (physics timing) are left out of the model; the caller keeps a
solution only if fleet_windows' own cost falls. scipy.optimize.milp (HiGHS)
is imported only here, so the package itself needs NumPy alone; a node limit,
not a wall-clock one, bounds the search so a run repeats exactly.
"""

from __future__ import annotations

import numpy as np

_TOL = 1e-9
_BIG_M = 1.0e4  # s; longer than any delay in a 1,800 s run


def _roles(windows, crossings, trucks, fixed):
    """(crossing id, second, first, at, release, free_s(at), free_f(release)) for
    each order still possible."""
    active = {n for n, t in trucks.items() if len(t.g) > 1}
    out = []
    for c in crossings.values():
        if c.a not in active or c.b not in active:
            continue
        for second in (c.a, c.b):
            first = c.other(second)
            if c.id in fixed and fixed[c.id] != first:
                continue
            lead, mine = trucks[first], trucks[second]
            lo, hi = c.interval(second)
            for k in sorted({0, hi - lo}):
                release = float(c.need(second)[k]) + windows.sample_m
                if lead.progress_m > release:
                    continue
                at = float(mine.g[lo + k])
                if release >= lead.g[-1] - _TOL:  # it parks there: gone park_s later
                    release = float(lead.g[-1])
                    base = windows._free(lead, release) + windows.park_s
                else:
                    base = windows._free(lead, release)
                out.append((c.id, second, first, at, release, windows._free(mine, at), base))
    return out


def solve_orders(windows, crossings, trucks, fixed: dict, weights: dict, node_limit: int):
    """Delays as fleet_windows.timed returns them ({truck: (checkpoint arc
    lengths, delay from each on)}) for the best orders found; None if the
    solve finds no solution."""
    from scipy.optimize import Bounds, LinearConstraint, milp

    roles = _roles(windows, crossings, trucks, fixed)
    if not roles:
        return {}
    points = {}
    for _, second, _, at, *_ in roles:
        points.setdefault(second, set()).add(at)
    points = {n: np.array(sorted(p)) for n, p in points.items()}
    column, n_cols = {}, 0
    for n, p in points.items():
        for j in range(len(p)):
            column[(n, j)] = n_cols
            n_cols += 1
    choices = sorted({cid for cid, *_ in roles if cid not in fixed})
    binary = {cid: n_cols + i for i, cid in enumerate(choices)}
    n_vars = n_cols + len(choices)

    def delay_column(name, g_value):
        """The delay variable that applies at g_value on name's route (None: none yet)."""
        p = points.get(name)
        if p is None:
            return None
        j = int(np.searchsorted(p, g_value + _TOL, side="right")) - 1
        return column[(name, j)] if j >= 0 else None

    rows, lower = [], []
    for cid, second, first, at, release, mine, base in roles:
        row = np.zeros(n_vars)
        row[delay_column(second, at)] += 1.0
        lead = delay_column(first, release)
        if lead is not None:
            row[lead] -= 1.0
        rhs = base - mine
        if cid in binary:  # y = 1: a first
            if first == crossings[cid].a:  # active when y = 1: ... - M (1 - y) >= rhs
                row[binary[cid]] -= _BIG_M
                rhs -= _BIG_M
            else:  # active when y = 0: ... - M y >= rhs
                row[binary[cid]] += _BIG_M
        rows.append(row)
        lower.append(rhs)
    for n, p in points.items():  # delays only grow along a route
        for j in range(1, len(p)):
            row = np.zeros(n_vars)
            row[column[(n, j)]] = 1.0
            row[column[(n, j - 1)]] = -1.0
            rows.append(row)
            lower.append(0.0)
    objective = np.zeros(n_vars)
    for n, p in points.items():
        for j in range(len(p)):
            objective[column[(n, j)]] = 1e-3
        objective[column[(n, len(p) - 1)]] += weights.get(n, 1.0)
    integrality = np.zeros(n_vars)
    integrality[n_cols:] = 1
    upper = np.full(n_vars, _BIG_M)
    upper[n_cols:] = 1.0
    result = milp(
        objective,
        constraints=LinearConstraint(np.array(rows), np.array(lower), np.inf),
        integrality=integrality,
        bounds=Bounds(np.zeros(n_vars), upper),
        options={"node_limit": node_limit, "presolve": True, "disp": False},
    )
    if result.x is None:
        return None
    x = result.x
    return {n: (p, np.array([x[column[(n, j)]] for j in range(len(p))])) for n, p in points.items()}
