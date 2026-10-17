"""Zone (tile) reservation with deadlock avoidance, for comparison.

Plan: docs/plans/2026-10-17-fleet-literature-baselines.md (L1), after Fransen,
Reniers and van Eekelen, "Deadlock avoidance algorithm for AGVs on a
tessellated layout", IEEE CASE 2022 (the tile size is ours: the paper gives
none). ``TrafficManager(mode="tiles")`` keeps its claims and replaces the
crossing orders by tiles:

- **Tiles.** The floor is cut into square tiles of ``tile_m``. A route sample
  covers the tiles its footprint grown by the traffic margin (over the sample
  and the place a truck held there may rest) touches: the envelope the claims
  keep, so trucks on tiles of their own never meet in the claims either. The
  default 0.25 m is below the 0.28 m between the envelopes of two trucks in
  neighbouring holding places (world_gen's 1.6 m pitch), so those never share
  a tile. A tile is held by one truck at a time: the tiles under its body and
  those it has reserved ahead.
- **Reservation.** Each update every truck tries, in rank order, to reserve the
  tiles of its route up to ``lookahead_m`` ahead, sample by sample. A grant
  may reach only samples whose tiles the truck holds. A sample needing a tile
  another truck holds stops the reservation there.
- **Deadlock avoidance.** A state is checked as the paper does, over each
  truck's whole remaining route (or ``horizon_m`` of it, if given): each truck
  starts at the end of what it holds, standing on that sample's tiles; one at
  a time, a truck whose remaining route crosses no tile another truck stands
  on drives to its end, where it parks for good on its last sample's tiles
  (beyond a horizon nothing is known, so a truck that gets there goes). A
  truck with no route or at its end stands where it is for good. The trucks
  that never get to go are *stuck*. Among those that may go, one whose
  parking place no other truck's route crosses goes first. Taking a free tile,
  or a new route, is allowed only if it leaves no truck stuck that was not
  stuck before (the paper: only moves into a safe state; ours also lets the
  rest move on when the state has become unsafe some other way -- a truck
  parked across a route, or a wait of the claims the tiles do not see).
  The run is the banker's test, not a search over every order, so a refusal
  may be one a search would allow.
- **New routes.** A route is taken only if it leaves no new truck stuck in
  that sense (as the paper asks of a new path); fleet_traffic's callers hold
  the job or turn the re-plan down otherwise.
- Tiles behind a truck's body are released each update. No times are used.

Arc lengths in metres. The interface is the part of fleet_windows.Windows
that fleet_traffic uses.
"""

from __future__ import annotations

import time
from collections import Counter

import numpy as np

_TOL = 1e-9
_OFF = 1 << 20


def _key(i, j) -> int:
    return (int(i) + _OFF) * (2 * _OFF) + int(j) + _OFF


def route_tiles(rects: np.ndarray, tile_m: float) -> list[frozenset]:
    """The tiles each (N, 6) rect (centre, half length, half width, cos, sin) touches."""
    out = []
    for x, y, hl, hw, c, s in np.asarray(rects, dtype=float).reshape(-1, 6):
        ex, ey = abs(c) * hl + abs(s) * hw, abs(s) * hl + abs(c) * hw  # axis-aligned half extents
        i0, i1 = int(np.floor((x - ex) / tile_m)), int(np.floor((x + ex) / tile_m))
        j0, j1 = int(np.floor((y - ey) / tile_m)), int(np.floor((y + ey) / tile_m))
        h = tile_m / 2
        ii, jj = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1), indexing="ij")
        dx, dy = (ii + 0.5) * tile_m - x, (jj + 0.5) * tile_m - y
        # Separating axes: the rect's two (the tile's two are the bounding box above).
        reach = h * (abs(c) + abs(s)) + _TOL
        keep = (np.abs(dx * c + dy * s) <= hl + reach) & (np.abs(-dx * s + dy * c) <= hw + reach)
        out.append(frozenset(_key(i, j) for i, j in zip(ii[keep], jj[keep], strict=True)))
    return out


class Tiles:
    """Tile reservation with deadlock avoidance; see the module docstring."""

    def __init__(
        self,
        *,
        sample_m: float,
        lookahead_m: float,
        tile_m: float = 0.25,
        horizon_m: float | None = None,
        slack_s: float = 10.0,
        moving_m: float = 0.05,
        **_unused,
    ) -> None:
        self.sample_m = sample_m
        self.lookahead_m = lookahead_m
        self.tile_m = tile_m
        self.horizon_m = horizon_m if horizon_m and horizon_m > 0 else None  # None: the whole route
        self.slack_s = slack_s  # fleet_traffic reads it for held trucks
        self.moving_m = moving_m
        self.crossings: dict = {}  # none; fleet_traffic.window_state draws nothing
        self.counts: Counter = Counter()
        self.events: list[dict] = []
        self.last_feasible = True
        self.wall_s = 0.0
        self.cells: dict[str, list[frozenset]] = {}  # per route sample: compact tile ids
        self.flat: dict[str, tuple[np.ndarray, np.ndarray]] = {}  # the same, concatenated, and offsets
        self.reserved: dict[str, int] = {}  # last route sample whose tiles the truck holds
        self.blocker: dict[str, str | None] = {}
        self.owner: dict[int, str] = {}
        self._ids: dict[int, int] = {}  # tile key -> compact id

    # -- building ------------------------------------------------------------
    def _tiles(self, truck) -> tuple[list[frozenset], tuple[np.ndarray, np.ndarray]]:
        """truck's route samples' tiles as compact ids, and concatenated."""
        ids = self._ids
        cells = [frozenset(ids.setdefault(x, len(ids)) for x in c) for c in route_tiles(truck.span, self.tile_m)]
        lens = np.fromiter((len(c) for c in cells), dtype=np.int64, count=len(cells))
        offs = np.concatenate(([0], np.cumsum(lens)))
        flat = np.fromiter((x for c in cells for x in c), dtype=np.int64, count=int(offs[-1]))
        return cells, (flat, offs)

    def _cells(self, truck) -> list[frozenset]:
        if truck.name not in self.cells:
            self.cells[truck.name], self.flat[truck.name] = self._tiles(truck)
            self.reserved[truck.name] = self._index(truck, truck.progress_m)
        return self.cells[truck.name]

    def replace(self, name: str, groups: dict, trucks: dict, now: float, *, dry: bool = False) -> bool:
        """A new route (fleet_traffic.set_route, or trial_route with dry): its
        tiles are reserved afresh as it drives. Returns whether it leaves no
        truck stuck that was not before (as Fransen et al. ask of a new path;
        fleet_traffic's callers hold a job or turn a re-plan down when not)."""
        t = trucks[name]
        cells, flat = self._tiles(t)
        k = self._index(t, t.progress_m)
        before = set(self._stuck(trucks, name, k, (cells, flat), before=True))
        ok = set(self._stuck(trucks, name, k, (cells, flat))) <= before
        if not dry:
            self.cells[name], self.flat[name] = cells, flat
            self.reserved[name] = k
            self.last_feasible = ok
        return ok

    def drop(self, name: str) -> None:
        for d in (self.cells, self.flat, self.reserved, self.blocker):
            d.pop(name, None)

    @staticmethod
    def _index(truck, g_value: float) -> int:
        return int(np.clip(np.searchsorted(truck.g, g_value + _TOL, side="right") - 1, 0, len(truck.g) - 1))

    def _back(self, truck) -> int:
        return self._index(truck, truck.progress_m - self.sample_m)

    def _holding(self, truck) -> set:
        cells = self._cells(truck)
        r = max(self.reserved[truck.name], self._back(truck))
        self.reserved[truck.name] = r
        out: set = set()
        for k in range(self._back(truck), r + 1):
            out |= cells[k]
        return out

    # -- per update ----------------------------------------------------------
    def step(self, now: float, trucks: dict) -> None:
        t0 = time.perf_counter()
        order = sorted(trucks.values(), key=lambda t: t.key)
        held = {t.name: self._holding(t) for t in order}
        self.owner = {}
        for t in order:
            for x in held[t.name]:
                if self.owner.setdefault(x, t.name) != t.name:
                    self.counts["shared"] += 1  # a route set where another truck stands
        stuck = set(self._stuck(trucks))
        if stuck:
            self.counts["unsafe_steps"] += 1
        for t in order:
            n, cells = t.name, self.cells[t.name]
            target = self._index(t, min(t.g[-1], t.progress_m + self.lookahead_m))
            r, by = self.reserved[n], None
            while r < target:
                new = cells[r + 1] - held[n]
                taken = [self.owner[x] for x in new if self.owner.get(x, n) != n]
                if taken:
                    by = taken[0]
                    break
                if new:
                    after = self._stuck(trucks, n, r + 1)
                    if not set(after) <= stuck:
                        self.counts["refused"] += 1
                        by = next((m for m in after if m not in stuck and m != n), None)
                        break
                    stuck = set(after)
                    self.counts["reserved"] += len(new)
                    held[n] |= new
                    for x in new:
                        self.owner[x] = n
                r += 1
            self.reserved[n], self.blocker[n] = r, by
        self.wall_s += time.perf_counter() - t0

    def _stuck(self, trucks, name: str | None = None, k: int = 0, tiles=None, *, before: bool = False) -> list[str]:
        """The trucks that never get to go in the run of the module docstring,
        in rank order; name held route sample k's tiles too (on the route
        with these tiles, if given). With before, name is taken as it was
        before this new route (trucks[name] already has the new one): on its
        old route, or standing at the new one's start if it had none."""
        run, standing = {}, []
        for t in trucks.values():
            n = t.name
            if before and n == name:
                if n not in self.cells:
                    standing.extend(tiles[0][k])
                    continue
                cells, (flat, offs) = self.cells[n], self.flat[n]
                pos, last = self.reserved[n], len(self.cells[n]) - 1
                end = last if self.horizon_m is None else min(last, pos + int(self.horizon_m / self.sample_m))
            else:
                cells, (flat, offs) = tiles if n == name and tiles is not None else (self._cells(t), self.flat[n])
                last = len(cells) - 1
                if len(t.g) < 2 or t.progress_m >= t.g[-1] - _TOL:
                    i = self._index(t, t.progress_m)
                    standing.extend(cells[i] | cells[self._back(t)])  # parked: stands for good
                    continue
                pos = k if n == name else self.reserved[n]
                end = last if self.horizon_m is None else self._index(t, min(t.g[-1], t.progress_m + self.horizon_m))
            if pos >= end:
                if end == last:
                    standing.extend(cells[last])
                continue
            run[n] = (flat[offs[pos + 1] : offs[end + 1]], cells[pos], cells[last] if end == last else None, t.key)
        if not run:
            return []
        if len(run) > 63:
            raise ValueError("tiles: at most 63 trucks on routes at once")
        bit = {n: 1 << b for b, n in enumerate(run)}
        size = len(self._ids)
        on, park = np.zeros(size, dtype=np.uint64), np.zeros(size, dtype=np.uint64)
        still = np.zeros(size, dtype=bool)
        for n, (_, start, last, _) in run.items():
            on[list(start)] |= np.uint64(bit[n])
            if last is not None:
                park[list(last)] |= np.uint64(bit[n])
        still[standing] = True
        crosses_on, crosses_park, blocked = {}, {}, {}
        for n, (seg, *_rest) in run.items():
            mine = ~bit[n]
            blocked[n] = bool(still[seg].any())
            crosses_on[n] = int(np.bitwise_or.reduce(on[seg])) & mine if len(seg) else 0
            crosses_park[n] = int(np.bitwise_or.reduce(park[seg])) & mine if len(seg) else 0
        left = sorted(run, key=lambda m: run[m][3])
        waiting = sum(bit.values())  # trucks still standing at their start
        parked = 0  # trucks gone to their parking place
        while left:
            ready = [
                n
                for n in left
                if not blocked[n] and not crosses_on[n] & waiting & ~bit[n] and not crosses_park[n] & parked
            ]
            if not ready:
                return left
            needed = 0
            for m in left:
                needed |= crosses_park[m]
            n = next((m for m in ready if not needed & bit[m]), ready[0])
            left.remove(n)
            waiting &= ~bit[n]
            if run[n][2] is not None:
                parked |= bit[n]
        return []

    def limit(self, truck, first: int, stop: int, trucks) -> tuple[int, str | None]:
        """The first sample index in [first, stop) the truck may not hold a
        grant to (beyond its tiles), and the truck it waits for."""
        r = self.reserved.get(truck.name, len(truck.g) - 1)
        if r + 1 >= stop:
            return stop, None
        return max(r + 1, first), self.blocker.get(truck.name)

    def summary(self) -> dict:
        return {**dict(self.counts), "wall_s": round(self.wall_s, 1)}
