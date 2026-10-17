"""Crossing orders from the literature, for comparison with fleet_windows / fleet_queues.

Plan: docs/plans/2026-10-17-fleet-literature-baselines.md. Each class keeps
fleet_windows' crossings, ``need`` rule, lock check (``_run``) and healing,
and the claims stay in fleet_traffic; only how the orders are chosen differs.
No tokens are issued and nothing expires.

- ``FixedOrders`` (``mode="adg"``): action-dependency-graph execution (Hönig et
  al., RA-L 2019). A crossing's order is the plan's -- first-come in free-flow
  time when the route is set -- and is kept to the end; it is turned over only
  where the orders would otherwise lock (``_heal``), as a re-plan would.
- ``Sections`` (``mode="sections"``): critical sections with a precedence
  heuristic (Pecora et al., ICAPS 2018; coordination_oru). Every ``period_s``
  each open crossing goes to the truck closer to it along its route; a truck
  whose grant is in (it can no longer stop short) keeps going first, and a
  change that would lock the orders is not made. No times are used.
- ``Reservations`` (``mode="mapf"``): prioritized space-time reservation on
  the fixed routes, re-solved in a rolling horizon and executed through the
  orders (RHCR, Li et al., AAAI 2021, with ADG execution as in RHCRE; the
  single-path timing as in SIPP). Every ``replan_s`` the trucks are scheduled
  in priority order (``key``: rank, then name); a truck goes first at a
  crossing with a higher-priority truck only if it clears it before that
  truck's grant would enter, otherwise it waits for it to pass and the wait
  delays the rest of its route. Crossings neither truck reaches within
  ``window_s`` keep their order.

Arc lengths in metres, times in seconds.
"""

from __future__ import annotations

import numpy as np

from .fleet_windows import _TOL, Windows


class FixedOrders(Windows):
    """ADG execution: the plan's orders, kept (see the module docstring)."""

    def _expire(self, now, trucks) -> None:
        return

    def _optimise(self, now, trucks) -> None:
        return

    def _issue(self, now, trucks) -> None:
        return


class Sections(FixedOrders):
    """Critical sections, closest truck first (see the module docstring)."""

    def _optimise(self, now, trucks) -> None:
        def ahead(c, name):
            t = trucks[name]
            return float(t.g[c.interval(name)[0]]) - t.progress_m

        open_ = [c for c in self.crossings.values() if not c.entered]
        open_.sort(key=lambda c: min(ahead(c, c.a), ahead(c, c.b)))
        for c in open_:
            closer = min((c.a, c.b), key=lambda n: (ahead(c, n), trucks[n].rank, n))
            if closer == c.first:
                continue
            c.first = closer
            if self._run(self.crossings, trucks)[1]:
                c.first = c.second
                self.counts["kept_for_lock"] += 1
            else:
                self.counts["turned"] += 1


class Reservations(FixedOrders):
    """Prioritized space-time reservation in a rolling horizon (see the module docstring)."""

    def __init__(self, *, window_s: float = 30.0, replan_s: float = 5.0, **timing) -> None:
        super().__init__(**timing)
        self.window_s = window_s
        self.replan_s = replan_s
        self._last_plan = -np.inf

    def schedule(self, now: float, trucks) -> dict[int, str]:
        """The order each crossing gets from the prioritized schedule."""
        active = sorted((t for t in trucks.values() if len(t.g) > 1), key=lambda t: t.key)
        priority = {t.name: k for k, t in enumerate(active)}
        mine: dict[str, list] = {t.name: [] for t in active}
        for c in self.crossings.values():
            if c.a in mine and c.b in mine:
                mine[c.a].append(c)
                mine[c.b].append(c)
        booked: dict[tuple[int, str], tuple[float, float]] = {}  # (crossing, truck) -> grant entry, release
        orders: dict[int, str] = {}
        for t in active:
            n = t.name
            wait = 0.0
            for c in sorted(mine[n], key=lambda c: c.interval(n)[0]):
                lo, hi = c.interval(n)
                gate = max(float(t.g[lo]) - self.lookahead_m, t.progress_m)
                out = min(float(t.g[hi]) + self.sample_m, float(t.g[-1]))
                enter = now + self._free(t, gate) + wait
                leave = now + self._free(t, out) + wait
                other = c.other(n)
                if priority[other] < priority[n]:
                    o_enter, o_leave = booked[(c.id, other)]
                    if c.entered:
                        first = c.first
                    elif enter - now > self.window_s and o_enter - now > self.window_s:
                        first = c.first
                    else:
                        first = n if leave <= o_enter + _TOL else other
                    orders[c.id] = first
                    if first == other and o_leave > enter:
                        late = o_leave - enter
                        wait += late
                        enter, leave = enter + late, leave + late
                booked[(c.id, n)] = (enter, leave)
        return orders

    def _optimise(self, now, trucks) -> None:
        if now - self._last_plan < self.replan_s - _TOL:
            return
        self._last_plan = now
        orders = self.schedule(now, trucks)
        before = {c.id: c.first for c in self.crossings.values()}
        changed = [c for c in self.crossings.values() if not c.entered and c.id in orders and orders[c.id] != c.first]
        for c in changed:
            c.first = orders[c.id]
        if changed and self._run(self.crossings, trucks)[1]:
            # The whole schedule would lock (the claims or a stale order): take
            # its changes one at a time, by the higher priority of the pair.
            for c in changed:
                c.first = before[c.id]
            changed.sort(key=lambda c: min(trucks[c.a].key, trucks[c.b].key))
            for c in changed:
                c.first = orders[c.id]
                if self._run(self.crossings, trucks)[1]:
                    c.first = before[c.id]
                    self.counts["kept_for_lock"] += 1
        self.counts["turned"] += sum(1 for c in self.crossings.values() if before.get(c.id, c.first) != c.first)
        self.counts["plans"] += 1
