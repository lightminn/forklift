"""Crossing queues: every truck through one place in line, each with its own window.

Plan: docs/plans/2026-10-16-fleet-crossing-queues.md. ``TrafficManager(mode=
"queues")`` keeps fleet_windows' crossings -- pairs of touching route
intervals with a ``need`` per sample -- as the rule a grant obeys, and decides
who goes first in them by queues:

- **Members and places.** Each truck's interval in each crossing is a member,
  so a crossing has two members, one in line ahead of the other by ticket
  (lower first): its first truck, except where a truck is already in (it goes
  first). A truck's members overlapping on its route are one pass through one
  place (a junction three routes meet at); a place's members are in line by
  ticket. Intervals are not merged: a long shared lane would tie the
  junctions along it into one member, whose order is fixed everywhere once
  the truck enters any of it (plan, fixes). Following traffic follows
  (``need``).
- **Tickets.** A member joins at its free-flow entry time, so a new member is
  first-come in free-flow time as in fleet_windows. Members keep their ticket
  while their crossing stands.
- **Windows.** Every member gets a token once its planned entry is within
  ``issue_s``: its window ends ``slack_s`` after that entry. A token not used
  by then expires and the member joins the line again at its new free-flow
  entry time -- behind the other member of its crossing if that one is due
  earlier -- where that keeps the orders free of locks (otherwise it keeps its
  place, counted). A member late only because the truck ahead of it has not
  let it in yet keeps its place and gets a new window. Once in, overstaying
  is only recorded.
- **Order.** ``optimiser="local"`` turns over open crossings (neither member
  holds a token or is in) as fleet_windows does, by swapping the two members'
  tickets; ``"global"`` (fleet_queue_opt) sets every open member's ticket from
  one mixed-integer solve of all open orders every ``opt_period_s``, kept only
  if the orders stay free of locks and the weighted delay falls; the local
  turns run as well. With ``opt_horizon_s`` only orders whose crossing a
  truck reaches within that many seconds (free flow) are choices; the ones
  further on keep their order this time, as fixed ones do (plan
  2026-10-17-fleet-large-random-chained, 「계산 줄이기」). With ``opt_time_s``
  the solve starts from the current orders and stops after that many seconds
  with the best found (fleet_queue_opt).
- **Pace.** ``pace`` gives the speed at which a truck reaches the end of the
  grant it will be held at just as the truck ahead lets it go on (``None``:
  no cap, below ``pace_min_mps`` or nothing to wait for). Safety stays with
  the grant end.

No-lock and held trucks are fleet_windows'. Arc lengths in metres, times in
seconds.
"""

from __future__ import annotations

import copy
import time
from dataclasses import dataclass

import numpy as np

from .fleet_windows import _TOL, Windows

_EPS = 1e-3  # s between a ticket put just ahead of or behind another


@dataclass
class Member:
    """One truck's interval in one crossing, and its token."""

    id: int
    truck: str
    lo: int  # first and last sample index on the truck's route
    hi: int
    ticket: float
    issued_s: float | None = None
    deadline_s: float | None = None
    close_s: float | None = None
    planned_s: float | None = None
    entered: bool = False
    overrun: bool = False
    crossing: int = -1

    @property
    def fixed(self) -> bool:
        """Its orders may no longer change: it holds a token or is in."""
        return self.entered or self.issued_s is not None


class Queues(Windows):
    """Members, their lines and tokens over fleet_windows' crossings."""

    def __init__(
        self,
        *,
        pace_min_mps: float = 0.1,
        optimiser: str = "local",
        opt_period_s: float = 5.0,
        node_limit: int = 2000,
        opt_horizon_s: float | None = None,
        opt_time_s: float | None = None,
        **kwargs,
    ) -> None:
        if optimiser not in ("local", "global"):
            raise ValueError("optimiser is local or global")
        super().__init__(**kwargs)
        self.pace_min_mps = pace_min_mps
        self.optimiser = optimiser
        self.opt_period_s = opt_period_s
        self.node_limit = node_limit
        self.opt_horizon_s = opt_horizon_s
        self.opt_time_s = opt_time_s
        self.members: dict[int, Member] = {}
        self._next_member = 0
        self._last_global = -np.inf
        self._delay: dict | None = None  # the last timed() of step, for pace

    # -- members and lines ---------------------------------------------------
    @staticmethod
    def _index(members) -> dict[int, dict[str, Member]]:
        """Crossing id -> {truck: its member there}."""
        out: dict[int, dict[str, Member]] = {}
        for m in members.values():
            out.setdefault(m.crossing, {})[m.truck] = m
        return out

    def _pair(self, c, index) -> tuple[Member, Member]:
        """c's member on a's route and on b's."""
        both = index[c.id]
        return both[c.a], both[c.b]

    def _members_now(self, crossings, members, trucks, now: float) -> dict[int, Member]:
        """A member for each truck in each crossing: the ones there were, and
        new ones joining at their free-flow entry time."""
        old = self._index(members)
        out = {}
        for c in crossings.values():
            for name in (c.a, c.b):
                m = old.get(c.id, {}).get(name)
                if m is None:
                    lo, hi = c.interval(name)
                    truck = trucks[name]
                    ticket = now + self._free(truck, float(truck.g[lo]))
                    m = Member(self._new_member(), name, lo, hi, ticket, crossing=c.id)
                out[m.id] = m
        return out

    def _new_member(self) -> int:
        self._next_member += 1
        return self._next_member - 1

    @staticmethod
    def _key(m: Member, trucks):
        return (m.ticket, trucks[m.truck].rank, m.truck)

    def _conform(self, crossings, members, trucks, only) -> None:
        """Move tickets so the lines agree with the orders of the crossings
        in only (the first truck's member just ahead where it is not)."""
        index = self._index(members)
        for _ in range(10):
            moved = False
            for c in crossings.values():
                if c.id not in only:
                    continue
                ma, mb = self._pair(c, index)
                first, other = (ma, mb) if c.first == c.a else (mb, ma)
                if self._key(first, trucks) > self._key(other, trucks):
                    first.ticket = other.ticket - _EPS
                    moved = True
            if not moved:
                return

    def _derive(self, crossings, members, trucks) -> None:
        """Each crossing's first truck from the lines: the one already in
        (its member moved ahead if need be), else the member ahead."""
        self._conform(crossings, members, trucks, {c.id for c in crossings.values() if c.entered})
        index = self._index(members)
        for c in crossings.values():
            if c.entered:
                continue
            ma, mb = self._pair(c, index)
            c.first = c.a if self._key(ma, trucks) < self._key(mb, trucks) else c.b
        self._seconds = None

    def places(self, crossings=None, members=None) -> dict[int, int]:
        """Member id -> its place (the lowest member id tied to it by a crossing
        or by overlapping it on its truck's route)."""
        crossings = self.crossings if crossings is None else crossings
        members = self.members if members is None else members
        parent = {mid: mid for mid in members}

        def root(k):
            while parent[k] != k:
                parent[k] = parent[parent[k]]
                k = parent[k]
            return k

        def tie(a, b):
            ra, rb = root(a), root(b)
            if ra != rb:
                parent[max(ra, rb)] = min(ra, rb)

        index = self._index(members)
        for c in crossings.values():
            ma, mb = self._pair(c, index)
            tie(ma.id, mb.id)
        by_truck: dict[str, list[Member]] = {}
        for m in members.values():
            by_truck.setdefault(m.truck, []).append(m)
        for ms in by_truck.values():
            ms.sort(key=lambda m: m.lo)
            reach = ms[0]
            for m in ms[1:]:
                if m.lo <= reach.hi:
                    tie(reach.id, m.id)
                if m.hi > reach.hi:
                    reach = m
        return {mid: root(mid) for mid in members}

    def line(self, member_id: int) -> list[Member]:
        """The members of member_id's place, in line."""
        place = self.places()
        return sorted((m for m in self.members.values() if place[m.id] == place[member_id]), key=lambda m: (m.ticket, m.truck))

    # -- building ------------------------------------------------------------
    def _replace(self, name: str, groups: dict, trucks: dict, now: float, dry: bool) -> tuple[bool, dict, bool]:
        """As fleet_windows, with name's members joining their lines."""
        crossings = copy.deepcopy(self.crossings) if dry else self.crossings
        members = copy.deepcopy(self.members) if dry else self.members
        fresh, clash = self._renew(crossings, name, groups, trucks)
        kept = {mid: m for mid, m in members.items() if m.truck != name}
        new = self._members_now(crossings, kept, trucks, now)
        members.clear()
        members.update(new)
        self._conform(crossings, members, trucks, set(crossings) - set(fresh))
        tickets = {mid: m.ticket for mid, m in members.items()}
        ok = self._settle(crossings, members, trucks, fresh) and not clash
        if not ok and not clash and not dry:
            ok = self._give_way_in_line(crossings, members, trucks, name, fresh, tickets)
        if not dry:
            self.last_feasible = ok
            self._seconds = None
        return ok, crossings, clash

    def _give_way_in_line(self, crossings, members, trucks, name, fresh, tickets) -> bool:
        """fleet_windows' _give_way in the lines: the tickets before the
        settling, name's members behind the other's at its open new crossings."""
        tried = {mid: m.ticket for mid, m in members.items()}
        for mid, ticket in tickets.items():
            members[mid].ticket = ticket
        index = self._index(members)
        for cid in fresh:
            c = crossings[cid]
            if not c.entered:
                ma, mb = self._pair(c, index)
                mine, other = (ma, mb) if c.a == name else (mb, ma)
                mine.ticket = max(mine.ticket, other.ticket + _EPS)
        self._derive(crossings, members, trucks)
        if not self._run(crossings, trucks)[1]:
            return True
        for mid, ticket in tried.items():
            members[mid].ticket = ticket
        self._derive(crossings, members, trucks)
        return False

    def drop(self, name: str) -> None:
        super().drop(name)
        kept = {mid: m for mid, m in self.members.items() if m.truck != name}
        self.members = self._members_now(self.crossings, kept, {}, 0.0)
        self._seconds = None

    def _settle(self, crossings, members, trucks, fresh) -> bool:
        """Derive the orders; where they lock, move the held member ahead of
        its leader -- new crossings first -- until they do not."""
        self._derive(crossings, members, trucks)
        tried = set()
        for _ in range(2 * len(fresh) + 4):
            stuck = self._run(crossings, trucks)[1]
            if not stuck:
                return True
            options = [cid for cid in stuck if not crossings[cid].entered and cid not in tried]
            options.sort(key=lambda cid: cid not in fresh)
            if not options:
                return False
            c = crossings[options[0]]
            tried.add(c.id)
            index = self._index(members)
            ma, mb = self._pair(c, index)
            held, leader = (mb, ma) if c.first == c.a else (ma, mb)
            held.ticket = leader.ticket - _EPS
            self._derive(crossings, members, trucks)
        return not self._run(crossings, trucks)[1]

    # -- per update ----------------------------------------------------------
    def step(self, now: float, trucks: dict) -> None:
        """Crossings passed, members in; every period_s orders healed, tokens
        expired, orders optimised and tokens issued."""
        self._seconds = None
        passed = False
        for c in list(self.crossings.values()):
            if self._passed(trucks[c.first], float(c.need(c.second)[-1])):
                del self.crossings[c.id]
                passed = True
                continue
            if not c.entered and any(self._in(c, n, trucks) for n in (c.a, c.b)):
                c.entered = True
        if passed:
            self.members = self._members_now(self.crossings, self.members, trucks, now)
            self._conform(self.crossings, self.members, trucks, set(self.crossings))
        for m in self.members.values():
            t = trucks[m.truck]
            if not m.entered and t.granted_m >= t.g[m.lo] - _TOL:
                m.entered = True
                self.counts["entered"] += 1
                if m.planned_s is not None:
                    self.eta_errors.append(now - m.planned_s)
            if m.entered and m.close_s is not None and not m.overrun and now > m.close_s:
                m.overrun = True
                self.counts["overrun"] += 1
        if now - self._last_step < self.period_s - _TOL:
            return
        self._last_step = now
        t0 = time.perf_counter()
        self._conform(self.crossings, self.members, trucks, set(self.crossings))  # after a drop
        self._heal(now, trucks)
        self._expire(now, trucks)
        self._optimise(now, trucks)
        self._issue(now, trucks)
        self._mirror()
        self.wall_s += time.perf_counter() - t0

    def _heal(self, now, trucks) -> None:
        stuck = self._run(self.crossings, trucks)[1]
        if not stuck:
            return
        before = {c.id: c.first for c in self.crossings.values()}
        index = self._index(self.members)

        def turn(c):
            """c turned over in the lines: the held member just ahead of its leader."""
            ma, mb = self._pair(c, index)
            held, leader = (mb, ma) if c.first == c.a else (ma, mb)
            ticket = held.ticket
            held.ticket = leader.ticket - _EPS
            self._derive(self.crossings, self.members, trucks)

            def back():
                held.ticket = ticket
                self._derive(self.crossings, self.members, trucks)

            return back

        ok = self._unlock(self.crossings, trucks, turn)
        turned = [c for c in self.crossings.values() if before.get(c.id, c.first) != c.first]
        self.counts["healed" if ok else "heal_failed"] += 1
        self.events.append({"t_s": round(now, 1), "kind": "orders_locked", "healed": ok,
                            "turned": [[c.first, c.second] for c in turned],
                            "held": sorted({self.crossings[cid].second for cid in stuck if cid in self.crossings})})

    def _held_in_line(self, m: Member, trucks) -> bool:
        """Is the truck ahead of m in its crossing still keeping it out?"""
        c = self.crossings.get(m.crossing)
        if c is None or c.second != m.truck:
            return False
        return trucks[c.first].progress_m - self.sample_m < float(c.need(m.truck)[0])

    def _expire(self, now, trucks) -> None:
        late = [m for m in self.members.values() if not m.entered and m.deadline_s is not None and now > m.deadline_s]
        for m in late:
            if self._held_in_line(m, trucks):
                self.counts["renewed"] += 1
            else:
                self._rejoin(m, now, trucks)
            m.issued_s = m.deadline_s = m.close_s = m.planned_s = None

    def _rejoin(self, m: Member, now, trucks) -> None:
        """m joins the line again at its new free-flow entry time: behind the
        other member of its crossing if that one is due earlier, where the
        orders do not lock."""
        c = self.crossings[m.crossing]
        ma, mb = self._pair(c, self._index(self.members))
        other = mb if m is ma else ma
        truck = trucks[m.truck]
        want = now + self._free(truck, float(truck.g[m.lo]))
        if self._key(other, trucks) < self._key(m, trucks) or want <= other.ticket:
            self.counts["expired_kept"] += 1  # already behind, or still due first
            return
        old = m.ticket
        m.ticket = max(want, other.ticket + _EPS)
        self._derive(self.crossings, self.members, trucks)
        if not self._run(self.crossings, trucks)[1]:
            self.counts["requeued"] += 1
            self.events.append({"t_s": round(now, 1), "kind": "window_expired", "truck": m.truck, "other": other.truck, "turned": True})
            return
        m.ticket = old
        self._derive(self.crossings, self.members, trucks)
        self.counts["expired_kept"] += 1

    def _open(self, c, index) -> bool:
        ma, mb = self._pair(c, index)
        return not c.entered and not ma.fixed and not mb.fixed

    def _optimise(self, now, trucks) -> None:
        delay = self.timed(self.crossings, trucks, now)
        self._delay = delay
        if delay is None:
            return
        if self.optimiser == "global" and now - self._last_global >= self.opt_period_s - _TOL:
            self._last_global = now
            self._optimise_global(now, trucks)
            delay = self.timed(self.crossings, trucks, now)
            if delay is None:
                return
        index = self._index(self.members)
        open_ = [c for c in self.crossings.values() if self._open(c, index)]
        open_.sort(key=lambda c: (getattr(trucks[c.second], "waiting_on", None) != c.first, self._entry(c, trucks, delay, now)[0]))
        best = self.cost(self.crossings, trucks, now)
        for c in open_[: self.evals]:
            ma, mb = self._pair(c, index)
            ma.ticket, mb.ticket = mb.ticket, ma.ticket
            self._derive(self.crossings, self.members, trucks)
            if not self._run(self.crossings, trucks)[1]:
                cost = self.cost(self.crossings, trucks, now)
                if cost < best - 1e-6:
                    best = cost
                    self.counts["turned"] += 1
                    continue
            ma.ticket, mb.ticket = mb.ticket, ma.ticket
            self._derive(self.crossings, self.members, trucks)
        self._delay = self.timed(self.crossings, trucks, now)

    def _optimise_global(self, now, trucks) -> None:
        from .fleet_queue_opt import solve_orders

        index = self._index(self.members)
        fixed = {c.id: c.first for c in self.crossings.values() if not self._open(c, index)}
        if self.opt_horizon_s is not None:  # orders further on than the horizon stay as they are
            for c in self.crossings.values():
                if c.id not in fixed and min(self._free(trucks[m.truck], float(trucks[m.truck].g[m.lo])) for m in self._pair(c, index)) > self.opt_horizon_s:
                    fixed[c.id] = c.first
        weights = {}
        for n, t in trucks.items():
            waited = 0.0 if t.waiting_since_s is None else now - t.waiting_since_s
            weights[n] = 1.0 + waited / self.age_s
        t0 = time.perf_counter()
        solved = solve_orders(self, self.crossings, trucks, fixed, weights, self.node_limit, self.opt_time_s)
        self.counts["global_wall_ms"] += int(round(1000 * (time.perf_counter() - t0)))
        if solved is None:
            self.counts["global_failed"] += 1
            return
        before = self.cost(self.crossings, trucks, now)
        old = {mid: m.ticket for mid, m in self.members.items()}
        firsts = {c.id: c.first for c in self.crossings.values()}
        for m in self.members.values():
            if m.fixed:
                continue
            gs, ds = solved.get(m.truck, (np.zeros(0), np.zeros(0)))
            g_value = float(trucks[m.truck].g[m.lo])
            k = int(np.searchsorted(gs, g_value, side="right"))
            m.ticket = now + self._free(trucks[m.truck], g_value) + (float(ds[k - 1]) if k else 0.0)
        for cid, first in fixed.items():  # an open member stays where a fixed order puts it
            c = self.crossings[cid]
            if c.entered:
                continue
            ma, mb = self._pair(c, index)
            lead, back = (ma, mb) if first == c.a else (mb, ma)
            if not back.fixed and self._key(back, trucks) <= self._key(lead, trucks):
                back.ticket = lead.ticket + _EPS
            elif not lead.fixed and self._key(lead, trucks) >= self._key(back, trucks):
                lead.ticket = back.ticket - _EPS
        self._derive(self.crossings, self.members, trucks)
        ok = not self._run(self.crossings, trucks)[1] and all(self.crossings[cid].first == f for cid, f in fixed.items())
        after = self.cost(self.crossings, trucks, now) if ok else np.inf
        if after < before - 1e-6:
            self.counts["global_turned"] += sum(firsts[cid] != c.first for cid, c in self.crossings.items())
            self.counts["global_kept"] += 1
            return
        for mid, ticket in old.items():
            self.members[mid].ticket = ticket
        self._derive(self.crossings, self.members, trucks)
        self.counts["global_worse" if ok else "global_locked"] += 1

    def _member_entry(self, m: Member, trucks, delay, now: float) -> tuple[float, float]:
        """Planned times m's truck's grant enters m and its body leaves it."""
        t = trucks[m.truck]
        gate = max(float(t.g[m.lo]) - self.lookahead_m, t.progress_m)
        out = float(t.g[m.hi]) + self.sample_m
        gs, ds = delay.get(m.truck, (np.zeros(0), np.zeros(0)))

        def when(g_value):
            k = int(np.searchsorted(gs, g_value, side="right"))
            return now + self._free(t, g_value) + (ds[k - 1] if k else 0.0)

        return when(gate), when(min(out, float(t.g[-1])))

    def _issue(self, now, trucks) -> None:
        delay = None
        for m in self.members.values():
            if m.entered or m.issued_s is not None:
                continue
            t = trucks[m.truck]
            if self._free(t, float(t.g[m.lo])) > self.issue_s:
                continue
            if delay is None:
                delay = self.timed(self.crossings, trucks, now) or {}
            enter, leave = self._member_entry(m, trucks, delay, now)
            if enter - now > self.issue_s:
                continue
            m.issued_s, m.deadline_s, m.close_s = now, enter + self.slack_s, leave + self.slack_s
            m.planned_s = enter
            self.counts["issued"] += 1

    def _mirror(self) -> None:
        """The first truck's token on each crossing (for drawing and RA)."""
        if not self.members:
            return
        index = self._index(self.members)
        for c in self.crossings.values():
            ma, mb = self._pair(c, index)
            m = ma if c.first == c.a else mb
            c.issued_s, c.deadline_s, c.close_s, c.planned_s = m.issued_s, m.deadline_s, m.close_s, m.planned_s

    # -- pace ------------------------------------------------------------------
    def pace(self, truck, trucks, now: float) -> float | None:
        """Speed at which truck reaches the end of the grant it will be held
        at just as its leader lets it on; None for no cap."""
        if self._delay is None:
            return None
        if self._seconds is None:
            self._seconds = {}
            for c in self.crossings.values():
                self._seconds.setdefault(c.second, []).append(c)
        best = None
        for c in self._seconds.get(truck.name, ()):
            lo, hi = c.interval(truck.name)
            leader = trucks[c.first]
            need = c.need(truck.name)
            k = int(np.searchsorted(need, leader.progress_m - self.sample_m, side="left"))
            if k > hi - lo or (best is not None and lo + k >= best[0]):
                continue
            release = float(need[k]) + self.sample_m
            gs, ds = self._delay.get(leader.name, (np.zeros(0), np.zeros(0)))
            j = int(np.searchsorted(gs, release, side="right"))
            when = now + self._free(leader, release) + (float(ds[j - 1]) if j else 0.0)
            best = (lo + k, when)
        if best is None:
            return None
        stop, when = best
        room = float(truck.g[max(stop - 1, 0)]) - truck.progress_m
        if room <= self.sample_m or when <= now + _TOL:
            return None
        speed = room / (when - now)
        return speed if speed >= self.pace_min_mps else None

    def summary(self) -> dict:
        out = super().summary()
        out["members"] = len(self.members)
        return out
