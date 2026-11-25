"""The real-mode SLAM bridge's state machine without ROS (docs/design/2026-11-25-real-sources-pr7.md 6.1, 2-3).

The ROS node (``forklift_ros.real_slam_bridge``) owns the socket, the slam_toolbox child processes and the ROS
publishers; it hands this core every frame from the executor and every ``/pose`` and ``/map`` from the current
instance, and the core answers through an ``io`` object:

- ``send(frame)``: a frame to the executor;
- ``start_instance(k, start_pose)``: start slam_toolbox instance ``k`` on its own topics with ``map_start_pose`` =
  ``start_pose`` (asynchronous: the node later calls ``on_instance_ready(k)`` -- lifecycle active, scan topic
  subscribed, static TF out -- or ``on_instance_failed(k, why)``);
- ``stop_instance(k)``: stop it (process group SIGTERM then SIGKILL, reaped, gone from the graph, its subscription
  queues drained); False when that did not complete;
- ``publish(k, scan, stamp_ns)``: the ``odom -> base_link`` TF and the scan, stamped ``stamp_ns``;
- ``now_ns()``: the host monotonic clock.

Rules (design 6.1, 2):
- one session per bridge: one ``hello``; its ``map_id`` must be the bundle's;
- every start (``hello``) and ``seed`` starts a new instance ``k``; ``state(id, seeding)`` then, once the instance is
  ready, ``state(id, ready_for_scans)``; the start check is the existing L4 -- seven start scans, a ``/pose`` expected
  for 1, 5, 6 and 7 (slam_toolbox drops 2-4 in a new process), the ``/map`` covering the survey's occupied cells and
  scans 5-7 on the walls near the start pose -- ending in ``ready``, ``not_localized`` (only an L4 match failure) or
  ``fault`` (no map loaded, a lifecycle failure, a protocol violation); no end state is sent past the frame's
  ``deadline_ns`` (the executor judges it);
- one scan in flight; a ``/pose`` for the scan in flight answers it (``processed``); a start scan that expects none is
  answered ``skipped`` after ``WARMUP_S``; a ``/pose`` of the current instance for any other stamp is a protocol
  violation (``fault``); messages of an earlier instance change nothing;
- ``cancel`` stops the seed's instance whatever its phase and answers ``state(seed_id, void, reason)``; the scan in
  flight is dropped with it.
"""

from __future__ import annotations


from forklift_core.localization import slam_link, slam_map
from forklift_core.localization import slam_real_link as rl
from forklift_core.localization.slam_pose import compose, invert

STARTUP_SCANS = 7
EXPECT = (1, 5, 6, 7)
CHECK = (5, 6, 7)  # the bootstrap scans the localization check reads (L4)
WARMUP_S = 0.5  # how long a start scan that expects no /pose waits before it is answered skipped


class BridgeCore:
    def __init__(self, io, bundle: slam_map.MapBundle, laser) -> None:
        self.io, self.bundle = io, bundle
        self.laser = tuple(float(v) for v in laser)  # base_link <- laser: x, y, z, yaw
        self.session: rl.Hello | None = None
        self.k = 0  # the current instance
        self.cur: dict | None = None  # the current start or seed
        self.awaited: dict | None = None  # the scan in flight
        self.last_stamp_ns: int | None = None
        self.correction = (0.0, 0.0, 0.0)  # the last processed reply's map <- odom
        self.closed = False
        self.counts = {"processed": 0, "skipped": 0, "stale_instance": 0, "late_maps": 0}

    # --- outputs ---------------------------------------------------------------------------------------------------
    def _state(self, ident: int, kind: str, reason: str | None = None) -> None:
        self.io.send(rl.encode_state(rl.State(int(ident), kind, reason)))

    def fault(self, reason: str) -> None:
        if self.closed:
            return
        ident = 0 if self.cur is None else self.cur["id"]
        self._state(ident, "fault", reason)
        self.closed = True

    def _reply(self, scan: slam_link.Scan, status: str) -> None:
        ready = self.cur is not None and self.cur["phase"] == "ready"
        self.io.send(slam_link.encode_reply(slam_link.Reply(
            scan.scan_id, scan.stamp_s, status, self.correction, self.k, 0.0,
            map_id=self.bundle.map_id if ready else None)))
        self.counts[status] += 1

    # --- executor frames -------------------------------------------------------------------------------------------
    def on_frame(self, obj) -> None:
        if self.closed:
            return
        if isinstance(obj, rl.Hello):
            if self.session is not None:
                return self.fault("protocol: a second hello")
            if obj.map_id != self.bundle.map_id:
                return self.fault(f"{slam_map.MISMATCH}: hello names map {obj.map_id}, the bundle is "
                                  f"{self.bundle.map_id}")
            self.session = obj
            return self._begin(obj.start_id, "start", obj.start_pose, obj.deadline_ns)
        if self.session is None:
            return self.fault("protocol: a frame before hello")
        if isinstance(obj, rl.Seed):
            if self.cur is not None and self.cur["phase"] in ("starting", "bootstrap"):
                return self.fault("protocol: a seed while another start is in progress")
            if self.awaited is not None:
                return self.fault("protocol: a seed with a scan in flight")
            if not self._stop():
                return
            return self._begin(obj.seed_id, "seed", obj.start_pose, obj.deadline_ns)
        if isinstance(obj, rl.Cancel):
            if self.cur is None or self.cur["kind"] != "seed" or self.cur["id"] != obj.seed_id:
                return self.fault(f"protocol: cancel of seed {obj.seed_id}, which is not the current seed")
            if not self._stop():
                return
            self.awaited = None  # the scan in flight goes with its instance
            self.cur = None
            return self._state(obj.seed_id, "void", obj.reason)
        if isinstance(obj, rl.RealScan):
            return self._scan(obj)
        return self.fault(f"protocol: unexpected frame {type(obj).__name__}")

    def _begin(self, ident: int, kind: str, pose, deadline_ns: int) -> None:
        self.k += 1
        self.cur = {"id": int(ident), "kind": kind, "phase": "starting", "pose": tuple(pose),
                    "deadline_ns": int(deadline_ns), "sent": 0, "first_processed": False, "map": None,
                    "fractions": [], "poses": []}
        self._state(ident, "seeding")
        self.io.start_instance(self.k, tuple(pose))

    def _stop(self) -> bool:
        """Stop the current instance (if any); a stop that did not complete is a fault."""
        if self.k and self.cur is not None:
            if not self.io.stop_instance(self.k):
                self.fault(f"instance {self.k} could not be stopped and reaped")
                return False
        return True

    def _scan(self, rs: rl.RealScan) -> None:
        cur = self.cur
        if cur is None or cur["phase"] not in ("bootstrap", "ready"):
            return self.fault("protocol: a scan with no instance ready for scans")
        if self.awaited is not None:
            return self.fault("protocol: a scan while another is in flight")
        if self.last_stamp_ns is not None and rs.stamp_ns <= self.last_stamp_ns:
            return self.fault("protocol: scan stamps must strictly increase")
        self.last_stamp_ns = rs.stamp_ns
        index = None
        expect = True
        if cur["phase"] == "bootstrap":
            if cur["sent"] >= STARTUP_SCANS:
                return self.fault("protocol: a scan after the start check's seven")
            cur["sent"] += 1
            index = cur["sent"]
            expect = index in EXPECT
        self.awaited = {"scan": rs.scan, "stamp_ns": rs.stamp_ns, "index": index, "expect": expect,
                        "sent_ns": int(self.io.now_ns())}
        self.io.publish(self.k, rs.scan, rs.stamp_ns)

    # --- instance events -------------------------------------------------------------------------------------------
    def on_instance_ready(self, k: int) -> None:
        if self.closed or k != self.k or self.cur is None or self.cur["phase"] != "starting":
            return
        self.cur["phase"] = "bootstrap"
        self._state(self.cur["id"], "ready_for_scans")

    def on_instance_failed(self, k: int, why: str) -> None:
        if not self.closed and k == self.k and self.cur is not None:
            self.fault(f"instance {k}: {why}")

    def on_pose(self, k: int, stamp_ns: int, pose) -> None:
        if self.closed:
            return
        if k != self.k or self.cur is None:
            self.counts["stale_instance"] += 1
            return
        aw = self.awaited
        if aw is None or int(stamp_ns) != aw["stamp_ns"]:
            return self.fault(f"protocol: a /pose for stamp {stamp_ns}, not the scan in flight")
        pose = tuple(float(v) for v in pose)
        scan = aw["scan"]
        self.correction = tuple(float(v) for v in compose(pose, invert(tuple(scan.odom_from_base))))
        cur = self.cur
        self.awaited = None
        if cur["phase"] == "bootstrap":
            cur["first_processed"] = True
            if aw["index"] in CHECK:
                x, y, _z, yaw = self.laser
                laser_pose = compose(pose, (x, y, yaw))
                cur["fractions"].append(slam_map.endpoint_fraction(
                    self.bundle.grid, laser_pose, scan.ranges_m, scan.angle_min_rad, scan.angle_increment_rad,
                    scan.range_min_m, scan.range_max_m))
                cur["poses"].append(pose)
        self._reply(scan, "processed")
        if cur["phase"] == "bootstrap" and aw["index"] == STARTUP_SCANS:
            self._start_check()

    def on_map(self, k: int, grid: slam_map.Grid) -> None:
        if self.closed or k != self.k or self.cur is None:
            self.counts["late_maps"] += 1
            return
        if self.cur["first_processed"] and self.cur["map"] is None:
            self.cur["map"] = grid  # the first map after the first processed scan (L3)

    def tick(self) -> None:
        """A start scan that expects no /pose is answered skipped after WARMUP_S."""
        aw = self.awaited
        if self.closed or aw is None or aw["expect"]:
            return
        if int(self.io.now_ns()) - aw["sent_ns"] >= WARMUP_S * 1e9:
            self.awaited = None
            self._reply(aw["scan"], "skipped")

    def _start_check(self) -> None:
        cur = self.cur
        if int(self.io.now_ns()) > cur["deadline_ns"]:
            cur["phase"] = "expired"  # no end state past the deadline: the executor judges it
            return
        if cur["map"] is None:
            return self.fault(f"{slam_map.NOT_LOADED}: no /map after the first processed scan")
        coverage = slam_map.map_coverage(self.bundle.grid, cur["map"])
        if not coverage >= slam_map.MAP_LOADED_MIN:
            return self.fault(f"{slam_map.NOT_LOADED}: the /map shows {coverage:.3f} of the survey's occupied cells")
        if not slam_map.localized(cur["fractions"], cur["poses"], cur["pose"]):
            cur["phase"] = "not_localized"
            return self._state(cur["id"], "not_localized",
                               f"{slam_map.NOT_LOCALIZED}: endpoint fractions "
                               f"{[round(f, 3) for f in cur['fractions']]}")
        cur["phase"] = "ready"
        self._state(cur["id"], "ready")


__all__ = ["BridgeCore", "CHECK", "EXPECT", "STARTUP_SCANS", "WARMUP_S"]
