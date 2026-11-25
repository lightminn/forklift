"""The observe run's SLAM path (docs/design/2026-11-25-real-sources-pr7.md 6.1, 8; 7c-1c): the real SLAM source on a
real-mode bridge, fed from the observe run's own STATEs and scans -- with no loop, so the odometry history is built
here with the loop's own functions (``odom_history.odometry_step`` / ``odom_entry``), not a copy.

The source starts the session at the registered start pose and is never seeded (the observe run moves nothing; the
hand-push test of 3.1 is the only motion). Before 7a-3 every real scan is diagnostic (no measured first-beam bound):
it is sent and its reply recorded, never treated as evidence. Recorded through ``put`` (the observe record):

- ``slam_event``: every source event -- the scans sent (identity, first-beam interval or unmeasured, ``odom_from_base``),
  the SLAM path's drops with their reasons, the replies (status, ``map_from_odom``, epoch and attempt, receipt, why not
  handed over), the bridge's states (``seeding``, ``ready``, ``not_localized``), late replies;
- on each reply, the match quality: the fraction of its measured endpoints on the survey's walls (L4's own
  ``slam_map.endpoint_fraction``), a diagnostic value nothing stops on (agy 7);
- ``slam_scan``: a feed scan the SLAM path could not take (``from_feed``'s reason);
- ``slam_fatal``: the source's end (the run goes on recording the board; its result says so).
"""

from __future__ import annotations

from collections import Counter, deque

from forklift_core.live.odom_history import odom_entry, odometry_step
from forklift_core.live.real_slam_source import SlamSourceFatal
from forklift_core.localization import slam_map
from forklift_core.localization.slam_pose import compose
from forklift_core.localization.slam_real_scan import from_feed
from forklift_core.localization.start_reloc import base_from_rear


class ObserveSlam:
    def __init__(self, source, put, *, odometry, params, rear_offset_m: float, start_ns: int, grid, base_laser,
                 lidar, state_delay_s: float, v_scan_mps: float, kappa_max: float) -> None:
        self.src, self.put, self.odometry, self.params = source, put, odometry, params
        self.rear_offset_m, self.start_ns, self.grid, self.lidar = float(rear_offset_m), int(start_ns), grid, lidar
        self.base_laser = tuple(float(v) for v in base_laser)  # x, y, z, yaw of the laser in base_link
        self.state_delay_s, self.v_scan_mps, self.kappa_max = float(state_delay_s), float(v_scan_mps), float(kappa_max)
        self.history: deque = deque(maxlen=256)  # the loop's odometry history length
        self.gen = 0  # raised at every break, boot change and new connection (the loop's odom_gen)
        self.pose = (0.0, 0.0, 0.0)
        self.boot = None
        self._prev = None
        self._last = None  # (connection, boot) of the last STATE
        self._events = 0
        self.fatal: str | None = None
        self.counts: Counter = Counter()
        source.odom_history = lambda: self.history
        source.odom_base = lambda: base_from_rear(self.pose, self.rear_offset_m)
        source.odom_gen = lambda: self.gen
        source.boot = lambda: self.boot
        source.reply_hook = self._quality

    # --- the odometry, from the observe run's STATEs ------------------------------------------------------------------
    def on_state(self, state, rx_ns: int, delay_b, conn: int) -> None:
        key = (int(conn), int(state.boot_id))
        if self._last is not None and key != self._last:
            self.gen += 1  # another connection or boot: the history before it is no evidence of this one
        self._last = key
        self.boot = int(state.boot_id)
        step = odometry_step(self.odometry, state)
        self.pose = tuple(float(v) for v in step.pose)
        if step.continuity_broken:
            self.gen += 1
        entry, self._prev = odom_entry(state, step, hi_s=(int(rx_ns) - self.start_ns) / 1e9, delay_b=delay_b,
                                       state_delay_s=self.state_delay_s, prev=self._prev,
                                       has_history=bool(self.history), params=self.params,
                                       v_scan_mps=self.v_scan_mps, kappa_max=self.kappa_max)
        self.history.append(entry)
        if self.fatal is None:
            self.src.odom_step(entry)

    # --- the scans and the replies ------------------------------------------------------------------------------------
    def on_scan(self, scan) -> None:
        """A scan the feed accepted (its identity and generations): offered to the source, or recorded why not."""
        if self.fatal is not None:
            return
        s, why = from_feed(scan, self.lidar)
        if s is None:
            self.counts[f"scan_{why}"] += 1
            self.put({"t": "slam_scan", "scan_id": int(scan.scan_id), "feed_gen": int(scan.feed_gen), "why": why})
            return
        self.counts["offered"] += 1
        self._guard(lambda: self.src.offer(s))

    def cycle(self, now_ns: int) -> None:
        if self.fatal is None:
            self._guard(lambda: self.src.replies(int(now_ns)))
        self.flush()

    def flush(self) -> None:
        ev = self.src.events
        for row in ev[self._events:]:
            self.put({**row, "t": "slam_event", "event": row.get("t")})  # the row's own "kind" (a state's) kept
        self._events = len(ev)

    def _guard(self, fn) -> None:
        try:
            fn()
        except SlamSourceFatal as exc:
            self.fatal = str(exc)
            self.flush()
            self.put({"t": "slam_fatal", "why": self.fatal})

    def _quality(self, scan, rec) -> dict:
        """L4's endpoint fraction for this reply: the laser in the map through the reply's correction."""
        x, y, _z, yaw = self.base_laser
        laser = compose(compose(tuple(rec["map_from_odom"]), tuple(scan.odom_from_base)), (x, y, yaw))
        f = slam_map.endpoint_fraction(self.grid, laser, scan.ranges_m, scan.angle_min_rad, scan.angle_increment_rad,
                                       scan.range_min_m, scan.range_max_m)
        return {"endpoint_fraction": None if f != f else float(f)}

    def summary(self) -> dict:
        t = self.src.target
        return {"fatal": self.fatal, "counts": dict(self.counts), "source_counts": dict(self.src.counts),
                "state": None if t is None else {"id": t["id"], "kind": t["kind"], "state": t["state"]},
                "odom_gen": self.gen}


__all__ = ["ObserveSlam"]
