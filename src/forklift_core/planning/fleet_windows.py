"""Time-window tokens at route crossings: who goes first where, by time.

Plan: docs/plans/2026-10-11-fleet-time-windows.md. ``TrafficManager(mode=
"windows")`` keeps its claims (no truck drives into another's body or claim)
and replaces fixed ranks and safe stops with this:

- **Crossings.** Where two trucks' remaining routes touch, the touching pairs
  of route samples, grouped by adjacency, make a crossing: an interval on each
  route. The same place passed at two different times is two crossings.
- **Order.** Each crossing has a truck that goes first. The other may hold a
  grant to a sample of its interval only once the first has passed every
  sample of the first's interval that touches it or an earlier sample of the
  interval (``need``: a running maximum, in the first's arc length). Crossing
  and head-on traffic so waits for the whole crossing; following traffic
  follows.
- **No lock.** If keeping every order lets every truck reach the end of its
  route (``feasible``: run the orders without time until nothing moves), then
  however long some trucks stop, one can always go on -- the temporal plan
  graph argument (``_run`` counts a truck held at a grant end as resting
  ``moving_m`` short, as real ones do). Orders are set and changed only while
  that holds -- and every ``period_s`` they are checked again and turned over
  where they no longer do (``_heal``) -- except
  where a truck is already in a crossing: it goes first. A truck that reaches
  the end of its route parks and leaves the orders, as an idle truck does:
  its body is then the claims' to keep clear of (and TA's to go round);
  ``timed`` counts it gone ``park_s`` after it arrives (its next job).
- **Windows.** Each truck has a free-flow time to every route sample;
  ``timed`` adds the waits the orders impose. When the first truck is
  ``issue_s`` from a crossing its token is issued, with a deadline to enter
  (its grant reaching the crossing). A token not used by then expires: the
  order turns over if that lowers the weighted total delay and stays
  feasible, otherwise the same truck gets a new window (counted, not
  listed in ``events``). Once a grant is in,
  the order is fixed; overstaying the window is only recorded.
- **Optimisation.** A new crossing goes first-come in free-flow time; every
  ``period_s`` up to ``evals`` open crossings (no token, no grant in) are
  turned over one at a time and kept turned when the delay falls -- first
  those the second truck stands waiting at, then by time.
- **Held trucks.** Free-flow times start ``held_s`` late (fleet_traffic sets
  it for a truck standing for a reason the orders do not know: a person, a
  pause). So a held truck's tokens expire to the trucks behind it where the
  orders allow, and it waits for a new one -- it is not given the same
  window again and again while it stands.

Trucks are fleet_traffic's ``_Truck`` records (name, rank, g, eta,
progress_m, granted_m, waiting_since_s, held_s). Arc lengths in metres, times in
seconds.
"""

from __future__ import annotations

import copy
import time
from collections import Counter
from dataclasses import dataclass
from math import sqrt

import numpy as np

_TOL = 1e-9


@dataclass
class Crossing:
    """Two trucks' route intervals that touch, and who goes first."""

    id: int
    a: str
    b: str
    ia: tuple[int, int]  # first and last sample index on a's route
    ib: tuple[int, int]
    # When a goes second: per sample of ia, the arc length on b's route that b
    # must have passed (running maximum); need_b the same the other way round.
    need_a: np.ndarray
    need_b: np.ndarray
    first: str
    entered: bool = False
    planned_s: float | None = None  # when the token was issued: the planned grant entry
    issued_s: float | None = None
    deadline_s: float | None = None
    close_s: float | None = None
    overrun: bool = False

    def other(self, name: str) -> str:
        return self.b if name == self.a else self.a

    @property
    def second(self) -> str:
        return self.other(self.first)

    def interval(self, name: str) -> tuple[int, int]:
        return self.ia if name == self.a else self.ib

    def need(self, name: str) -> np.ndarray:
        return self.need_a if name == self.a else self.need_b


def touch_groups(touch: np.ndarray, ga: np.ndarray, gb: np.ndarray) -> list[tuple]:
    """Groups of touching (a sample, b sample) pairs, 8-adjacent in the
    (len(ga), len(gb)) matrix -> (ia, ib, need_a, need_b) each; see Crossing."""
    rows = np.nonzero(touch.any(axis=1))[0]
    runs = []  # (row, first column, last column)
    for r in rows:
        cols = np.nonzero(touch[r])[0]
        cut = np.nonzero(np.diff(cols) > 1)[0]
        for s, e in zip(np.r_[cols[0], cols[cut + 1]], np.r_[cols[cut], cols[-1]], strict=True):
            runs.append((int(r), int(s), int(e)))
    parent = list(range(len(runs)))

    def root(k):
        while parent[k] != k:
            parent[k] = parent[parent[k]]
            k = parent[k]
        return k

    by_row: dict[int, list[int]] = {}
    for k, (r, _, _) in enumerate(runs):
        by_row.setdefault(r, []).append(k)
    for r, ks in by_row.items():
        for k in ks:
            for m in by_row.get(r + 1, ()):
                if runs[m][1] <= runs[k][2] + 1 and runs[k][1] <= runs[m][2] + 1:
                    parent[root(m)] = root(k)
    groups: dict[int, list[int]] = {}
    for k in range(len(runs)):
        groups.setdefault(root(k), []).append(k)
    out = []
    for ks in groups.values():
        part = [runs[k] for k in ks]
        i0, i1 = min(p[0] for p in part), max(p[0] for p in part)
        j0, j1 = min(p[1] for p in part), max(p[2] for p in part)
        need_a = np.full(i1 - i0 + 1, -np.inf)
        need_b = np.full(j1 - j0 + 1, -np.inf)
        for r, s, e in part:
            need_a[r - i0] = max(need_a[r - i0], float(gb[e]))
            np.maximum(need_b[s - j0 : e - j0 + 1], float(ga[r]), out=need_b[s - j0 : e - j0 + 1])
        out.append(
            ((i0, i1), (j0, j1), np.maximum.accumulate(need_a), np.maximum.accumulate(need_b))
        )
    return out


def time_profile(marks: np.ndarray, poses: np.ndarray, speed_mps: float, accel_mps2: float) -> np.ndarray:
    """Free-flow time to each arc length in marks along one chunk's poses:
    trapezoidal speed, a stop at the start, at each gear change and at the end."""
    poses = np.asarray(poses, dtype=float).reshape(-1, 3)
    marks = np.asarray(marks, dtype=float)
    if len(poses) < 2 or speed_mps <= 0:
        return np.zeros(len(marks))
    d = np.diff(poses[:, :2], axis=0)
    step = np.hypot(d[:, 0], d[:, 1])
    s = np.r_[0.0, np.cumsum(step)]
    ahead = d[:, 0] * np.cos(poses[:-1, 2]) + d[:, 1] * np.sin(poses[:-1, 2]) >= 0
    gear = []
    for k in range(len(step)):  # zero-length steps keep the gear before them
        gear.append(ahead[k] if step[k] > _TOL or not gear else gear[-1])
    cuts = [0.0] + [s[k] for k in range(1, len(step)) if gear[k] != gear[k - 1]] + [s[-1]]
    t, start = np.zeros(len(marks)), 0.0
    for lo, hi in zip(cuts[:-1], cuts[1:], strict=True):
        length = hi - lo
        inside = (marks >= lo - _TOL) & (marks <= hi + _TOL)
        if length <= _TOL:
            t[inside] = start
            continue
        peak = min(speed_mps, sqrt(accel_mps2 * length))
        ramp = peak * peak / (2 * accel_mps2)
        total = 2 * peak / accel_mps2 + max(length - 2 * ramp, 0.0) / peak
        x = np.clip(marks[inside] - lo, 0.0, length)
        t[inside] = start + np.where(
            x < ramp,
            np.sqrt(2 * x / accel_mps2),
            np.where(
                x <= length - ramp,
                peak / accel_mps2 + (x - ramp) / peak,
                total - np.sqrt(2 * np.maximum(length - x, 0.0) / accel_mps2),
            ),
        )
        start += total
    return t


class Windows:
    """Crossings, their orders and tokens; see the module docstring."""

    def __init__(
        self,
        *,
        sample_m: float,
        lookahead_m: float,
        issue_s: float = 30.0,
        slack_s: float = 10.0,
        period_s: float = 1.0,
        evals: int = 30,
        age_s: float = 120.0,
        park_s: float = 10.0,
        moving_m: float = 0.05,
    ) -> None:
        self.sample_m = sample_m
        self.lookahead_m = lookahead_m
        self.issue_s = issue_s
        self.slack_s = slack_s
        self.period_s = period_s
        self.evals = evals
        self.age_s = age_s
        self.park_s = park_s
        self.moving_m = moving_m  # a truck held at a grant end may rest this far short
        self.crossings: dict[int, Crossing] = {}
        self.counts: Counter = Counter()
        self.events: list[dict] = []
        self.last_feasible = True
        self._next_id = 0
        self._last_step = -np.inf
        self._seconds: dict | None = None  # truck -> crossings it goes second in
        self.eta_errors: list[float] = []  # actual - planned grant entry, per issued token used
        self.wall_s = 0.0

    # -- building ------------------------------------------------------------
    def replace(self, name: str, groups: dict, trucks: dict, now: float, *, dry: bool = False) -> bool:
        """name's crossings are now groups ({other: touch_groups list}). Orders:
        a truck already in goes first, otherwise first-come in free-flow time,
        then turned over where that is what makes the orders feasible. Returns
        whether they are; with dry, nothing is kept."""
        crossings = copy.deepcopy(self.crossings) if dry else self.crossings
        fresh, clash = self._renew(crossings, name, groups, trucks)
        ok = self._repair(crossings, trucks, fresh) and not clash
        if not dry:
            self.last_feasible = ok
            self._seconds = None
        return ok

    def _renew(self, crossings, name: str, groups: dict, trucks) -> tuple[list[int], bool]:
        """Replace name's crossings in crossings by groups' -> (the new ids,
        whether one has both trucks already in it)."""
        for cid in [c.id for c in crossings.values() if name in (c.a, c.b)]:
            del crossings[cid]
        fresh, clash = [], False
        for other, found in groups.items():
            for ia, ib, need_a, need_b in found:
                c = Crossing(self._next_id, name, other, ia, ib, need_a, need_b, name)
                self._next_id += 1
                inside = [n for n in (c.a, c.b) if self._in(c, n, trucks)]
                clash |= len(inside) == 2  # each stands in the other's way
                if inside:
                    c.first = name if name in inside else other
                    c.entered = True
                else:
                    ta = self._free(trucks[name], trucks[name].g[ia[0]])
                    tb = self._free(trucks[other], trucks[other].g[ib[0]])
                    if (tb, trucks[other].rank) < (ta, trucks[name].rank):
                        c.first = other
                crossings[c.id] = c
                fresh.append(c.id)
        return fresh, clash

    def drop(self, name: str) -> None:
        """name no longer takes part (it stands for good, e.g. parked)."""
        for cid in [c.id for c in self.crossings.values() if name in (c.a, c.b)]:
            del self.crossings[cid]
        self._seconds = None

    def _repair(self, crossings, trucks, fresh) -> bool:
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
            c.first = c.second
            tried.add(c.id)
        return not self._run(crossings, trucks)[1]

    # -- queries -------------------------------------------------------------
    def _in(self, c: Crossing, name: str, trucks) -> bool:
        t = trucks[name]
        return t.granted_m >= t.g[c.interval(name)[0]] - _TOL

    def _passed(self, truck, g_value: float, at_m: float | None = None) -> bool:
        at = truck.progress_m if at_m is None else at_m
        return at - self.sample_m > g_value

    @staticmethod
    def _eta(truck, g_value: float) -> float:
        """Free-flow time from the route's start to arc length g_value (the
        later of two samples at a stage boundary: after a lift or lower)."""
        k = int(np.searchsorted(truck.g, g_value + _TOL, side="right")) - 1
        return float(truck.eta[min(max(k, 0), len(truck.eta) - 1)])

    def _free(self, truck, g_value: float) -> float:
        """Free-flow time from now for truck to reach arc length g_value (held_s
        later for a held truck); a truck standing waiting first has to get going
        again (physics timing)."""
        base = max(self._eta(truck, g_value) - self._eta(truck, truck.progress_m), 0.0)
        if g_value <= truck.progress_m + _TOL:
            return base
        base += getattr(truck, "held_s", 0.0)
        restart = getattr(truck, "restart", None)
        if restart is not None and not getattr(truck, "moving", True):
            base += self._at(truck, restart, truck.progress_m)
        return base

    @staticmethod
    def _at(truck, values, g_value: float) -> float:
        k = int(np.searchsorted(truck.g, g_value + _TOL, side="right")) - 1
        return float(values[min(max(k, 0), len(values) - 1)])

    def limit(self, truck, first: int, stop: int, trucks) -> tuple[int, str | None]:
        """The first sample index in [first, stop) truck may not hold a grant
        to under the orders (stop if none), and the truck it waits for."""
        best, blocker = stop, None
        if self._seconds is None:
            self._seconds = {}
            for c in self.crossings.values():
                self._seconds.setdefault(c.second, []).append(c)
        for c in self._seconds.get(truck.name, ()):
            lo, hi = c.interval(truck.name)
            if hi < first or lo >= best:
                continue
            leader = trucks[c.first]
            k = int(np.searchsorted(c.need(truck.name), leader.progress_m - self.sample_m, side="left"))
            if lo + k < best and lo + k <= hi:
                best, blocker = max(lo + k, first), c.first
        return best, blocker

    def _run(self, crossings, trucks) -> tuple[dict, list[int]]:
        """Run the orders without time: every truck as far as they let it,
        again until nothing moves. -> (where each truck got, the crossings
        holding the trucks short of their ends). A truck run up to a grant
        end counts as resting moving_m short of it, as a real one may: a
        leader 1 cm short of releasing its follower locks them both."""
        pos = {n: t.progress_m for n, t in trucks.items() if len(t.g) > 1}
        rest = dict(pos)  # where each truck may actually come to rest
        seconds: dict[str, list[Crossing]] = {n: [] for n in pos}
        for c in crossings.values():
            if c.second in seconds and c.first in pos:
                seconds[c.second].append(c)
        holding: dict[str, int | None] = {}
        moved = True
        while moved:
            moved = False
            for n in pos:
                t = trucks[n]
                reach, by = t.g[-1], None
                for c in seconds[n]:
                    lo, hi = c.interval(n)
                    lead = trucks[c.first]
                    # A truck at the end of its route parks: it leaves the
                    # orders (its body is the claims' to keep clear of).
                    passed = np.inf if pos[c.first] >= lead.g[-1] - _TOL else rest[c.first] - self.sample_m
                    k = int(np.searchsorted(c.need(n), passed, side="left"))
                    if k <= hi - lo:
                        at = t.g[lo + k - 1] if lo + k > 0 else t.g[0]
                        if at < reach:
                            reach, by = at, c.id
                if reach > pos[n] + _TOL:
                    pos[n], moved = reach, True
                    rest[n] = max(t.progress_m, reach - self.moving_m)
                holding[n] = by if reach < t.g[-1] - _TOL else None
        stuck = [cid for n, cid in holding.items() if cid is not None and pos[n] < trucks[n].g[-1] - _TOL]
        return pos, stuck

    def timed(self, crossings, trucks, now: float) -> dict | None:
        """Waits the orders impose on free-flow times: {truck: (checkpoint arc
        lengths, delay from each on)} -- a crossing's start and end for its
        second truck; None if they never end (the orders lock)."""
        active = {n: t for n, t in trucks.items() if len(t.g) > 1}
        points: dict[str, list] = {n: [] for n in active}
        for c in crossings.values():
            n, f = c.second, c.first
            if n not in active or f not in active:
                continue
            lead, mine = active[f], active[n]
            lo, hi = c.interval(n)
            for k in sorted({0, hi - lo}):
                release = float(c.need(n)[k]) + self.sample_m
                if lead.progress_m > release:
                    continue
                at = float(mine.g[lo + k])
                if release >= lead.g[-1] - _TOL:  # it parks there: gone park_s later
                    release = float(lead.g[-1])
                    base = self._free(lead, release) + self.park_s
                else:
                    base = self._free(lead, release)
                stop = self._at(mine, mine.stopcost, at) if getattr(mine, "stopcost", None) is not None else 0.0
                points[n].append((at, f, release, self._free(mine, at), base, stop))
        for n in points:
            points[n].sort(key=lambda p: p[0])
        gs = {n: np.array([p[0] for p in pts]) for n, pts in points.items()}
        # Delay from checkpoint i of n on: the larger of the delay before it and
        # how late its leader passes the release point -- a graph of
        # checkpoints, each resolved once (depth first); a cycle locks.
        value: dict = {}
        open_: set = set()
        for root in ((n, i) for n, pts in points.items() for i in range(len(pts))):
            stack = [root]
            while stack:
                node = stack[-1]
                if node in value:
                    stack.pop()
                    continue
                n, i = node
                _, f, release, mine, base, stop = points[n][i]
                k = int(np.searchsorted(gs[f], release, side="right"))
                todo = [d for d in ((n, i - 1) if i else None, (f, k - 1) if k else None) if d is not None and d not in value]
                if todo:
                    if node in open_ or any(d in open_ for d in todo):
                        return None
                    open_.add(node)
                    stack.extend(todo)
                    continue
                before = value[(n, i - 1)] if i else 0.0
                lead = value[(f, k - 1)] if k else 0.0
                late = base + lead - mine
                # Held there: it also has to stop and get going again.
                value[node] = late + stop if late > before + 1e-9 else before
                open_.discard(node)
                stack.pop()
        return {n: (gs[n], np.array([value[(n, i)] for i in range(len(pts))])) for n, pts in points.items()}

    def cost(self, crossings, trucks, now: float) -> float:
        """Weighted total delay at the routes' ends (inf if the orders lock)."""
        delay = self.timed(crossings, trucks, now)
        if delay is None:
            return np.inf
        total = 0.0
        for n, (_, ds) in delay.items():
            t = trucks[n]
            waited = 0.0 if t.waiting_since_s is None else now - t.waiting_since_s
            total += (1.0 + waited / self.age_s) * (float(ds[-1]) if len(ds) else 0.0)
        return total

    def _entry(self, c: Crossing, trucks, delay, now: float) -> tuple[float, float]:
        """Planned times the first truck's grant enters c and its body leaves it."""
        t = trucks[c.first]
        lo, hi = c.interval(c.first)
        gate = max(float(t.g[lo]) - self.lookahead_m, t.progress_m)
        out = float(t.g[hi]) + self.sample_m
        gs, ds = delay.get(c.first, (np.zeros(0), np.zeros(0)))

        def when(g_value):
            k = int(np.searchsorted(gs, g_value, side="right"))
            return now + self._free(t, g_value) + (ds[k - 1] if k else 0.0)

        return when(gate), when(min(out, float(t.g[-1])))

    # -- per update ----------------------------------------------------------
    def step(self, now: float, trucks: dict) -> None:
        """Before grants are extended: crossings entered or passed; every
        period_s tokens issued, expired and orders optimised."""
        self._seconds = None
        for c in list(self.crossings.values()):
            if self._passed(trucks[c.first], float(c.need(c.second)[-1])):
                del self.crossings[c.id]
                continue
            if not c.entered and any(self._in(c, n, trucks) for n in (c.a, c.b)):
                c.entered = True
                self.counts["entered"] += 1
                if c.planned_s is not None:
                    self.eta_errors.append(now - c.planned_s)
            if c.entered and c.close_s is not None and not c.overrun and now > c.close_s:
                c.overrun = True
                self.counts["overrun"] += 1
        if now - self._last_step < self.period_s - _TOL:
            return
        self._last_step = now
        t0 = time.perf_counter()
        self._heal(now, trucks)
        self._expire(now, trucks)
        self._optimise(now, trucks)
        self._issue(now, trucks)
        self.wall_s += time.perf_counter() - t0

    def _heal(self, now, trucks) -> None:
        """Orders that no longer let every truck finish (trucks resting short
        of their grants, routes that changed): turn over open crossings
        holding trucks until they do, as far as that goes."""
        stuck = self._run(self.crossings, trucks)[1]
        if not stuck:
            return
        before = {c.id: c.first for c in self.crossings.values()}
        ok = self._repair(self.crossings, trucks, [])
        turned = [c for c in self.crossings.values() if before.get(c.id, c.first) != c.first]
        self.counts["healed" if ok else "heal_failed"] += 1
        self.events.append({"t_s": round(now, 1), "kind": "orders_locked", "healed": ok,
                            "turned": [[c.first, c.second] for c in turned],
                            "held": sorted({self.crossings[cid].second for cid in stuck if cid in self.crossings})})

    def _expire(self, now, trucks) -> None:
        late = [c for c in self.crossings.values() if not c.entered and c.deadline_s is not None and now > c.deadline_s]
        for c in late:
            keep = self.cost(self.crossings, trucks, now)
            c.first = c.second
            turned = self._run(self.crossings, trucks)[1] == [] and self.cost(self.crossings, trucks, now) < keep - _TOL
            if not turned:
                c.first = c.second
            self.counts["expired_turned" if turned else "expired_kept"] += 1
            if turned:  # a kept one is only counted: a truck held elsewhere renews many
                self.events.append({"t_s": round(now, 1), "kind": "window_expired", "truck": c.second, "other": c.first, "turned": True})
            c.issued_s = c.deadline_s = c.close_s = c.planned_s = None

    def _optimise(self, now, trucks) -> None:
        delay = self.timed(self.crossings, trucks, now)
        if delay is None:
            return
        open_ = [c for c in self.crossings.values() if not c.entered and c.issued_s is None]
        # Those a truck stands waiting at first (they cost now), then by time.
        open_.sort(key=lambda c: (getattr(trucks[c.second], "waiting_on", None) != c.first, self._entry(c, trucks, delay, now)[0]))
        best = self.cost(self.crossings, trucks, now)
        for c in open_[: self.evals]:
            c.first = c.second
            if not self._run(self.crossings, trucks)[1]:
                cost = self.cost(self.crossings, trucks, now)
                if cost < best - 1e-6:
                    best = cost
                    self.counts["turned"] += 1
                    continue
            c.first = c.second

    def _issue(self, now, trucks) -> None:
        delay = None
        for c in self.crossings.values():
            if c.entered or c.issued_s is not None:
                continue
            if self._free(trucks[c.first], float(trucks[c.first].g[c.interval(c.first)[0]])) > self.issue_s:
                continue
            if delay is None:
                delay = self.timed(self.crossings, trucks, now) or {}
            enter, leave = self._entry(c, trucks, delay, now)
            if enter - now > self.issue_s:
                continue
            c.issued_s, c.deadline_s, c.close_s = now, enter + self.slack_s, leave + self.slack_s
            c.planned_s = enter
            self.counts["issued"] += 1

    def summary(self) -> dict:
        out = {**dict(self.counts), "wall_s": round(self.wall_s, 1)}
        if self.eta_errors:
            e = np.asarray(self.eta_errors)
            out["eta_error_s"] = {"n": len(e), "mean": round(float(e.mean()), 2), "median_abs": round(float(np.median(np.abs(e))), 2),
                                  "p90_abs": round(float(np.percentile(np.abs(e), 90)), 2)}
        return out
