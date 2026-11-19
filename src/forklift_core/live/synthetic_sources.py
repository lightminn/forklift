"""The HIL run's synthetic sources (docs/design/2026-11-20-real-hil-6a.md 10.2): a LiDAR ray-cast from the model's
odometry, a SLAM link keeping the 5c-2 epoch contract on the same acquisitions, and a still IMU.

Each is made only for ``Client.HIL`` on a transport whose HIL check holds at that moment (else ``SyntheticRefused``),
and on every call gives nothing while the check is false -- what was acquired meanwhile is dropped, and nothing of an
earlier boot is given after a boot change. PR 7 swaps these for the real sources behind the same calls."""

from __future__ import annotations

import math
import random
from types import SimpleNamespace

import numpy as np

from forklift_core.live.real_layer import RealScan
from forklift_core.live.synthetic import SyntheticRefused
from forklift_core.localization.slam_pose import compose, invert
from forklift_core.mcu_link.build_policy import Client

SCAN_PERIOD_NS = 100_000_000
IMU_PERIOD_NS = 5_000_000
BEAMS = 360


def _require(transport, client) -> None:
    if client is not Client.HIL:
        raise SyntheticRefused(f"synthetic source for {client}")
    if getattr(transport, "hil_plant", False) is not True:
        raise SyntheticRefused("hil_check_not_held")


class _Gated:
    """The HIL check of every call: false, or a boot other than the one this source started on, drops everything."""

    def __init__(self, transport, client) -> None:
        _require(transport, client)
        self.transport, self.boot = transport, transport.hil_boot

    def _open(self) -> bool:
        tr = self.transport
        if getattr(tr, "hil_plant", False) is not True:
            self._drop()
            return False
        if tr.hil_boot != self.boot:
            self.boot = tr.hil_boot
            self._drop()
            return False
        return True

    def _drop(self) -> None:
        pass


class SyntheticScan(_Gated):
    """A revolution every 100 ms from the odometry pose: ``boxes`` are (x, y, half length, half width), axis-aligned in
    the odometry frame (the HIL scene, ``s10a_hil.yaml``). Acquisitions go to the layer's queue and to every subscriber
    (the SLAM side) separately; ``stop_delivery`` stops the layer's feed only (the injection ``scan``)."""

    def __init__(self, transport, client, start_ns: int, *, odom_pose, boxes, sensor: str = "lidar") -> None:
        super().__init__(transport, client)
        self.start_ns, self.odom_pose, self.sensor = int(start_ns), odom_pose, sensor
        self.boxes = [tuple(float(v) for v in b) for b in boxes]
        self.stop_delivery = False
        self.events: list = []  # (kind, sensor) loss / recovery notices, in order
        self._next_ns: int | None = None
        self._queue: list = []
        self.subscribers: list = []

    def _drop(self) -> None:
        self._queue, self._next_ns = [], None

    def _cast(self, first_s: float, last_s: float) -> RealScan:
        x, y, yaw = (float(v) for v in self.odom_pose())
        angles = np.linspace(-math.pi, math.pi, BEAMS, endpoint=False)
        ranges = np.full(BEAMS, np.inf)
        edges = []
        for bx, by, hl, hw in self.boxes:
            c = [(bx + hl, by + hw), (bx + hl, by - hw), (bx - hl, by - hw), (bx - hl, by + hw)]
            edges += list(zip(c, c[1:] + c[:1]))
        for k, a in enumerate(angles):
            dx, dy = math.cos(yaw + a), math.sin(yaw + a)
            best = math.inf
            for (ax, ay), (ex_, ey_) in edges:
                ex, ey = ex_ - ax, ey_ - ay
                den = dx * ey - dy * ex
                if abs(den) < 1e-12:
                    continue
                tt = ((ax - x) * ey - (ay - y) * ex) / den
                uu = ((ax - x) * dy - (ay - y) * dx) / den
                if tt > 0.0 and 0.0 <= uu <= 1.0:
                    best = min(best, tt)
            ranges[k] = best
        valid = np.isfinite(ranges)
        return RealScan(self.sensor, first_s, last_s, angles, ranges, valid, np.zeros(BEAMS, bool),
                        np.full(BEAMS, np.inf))

    def _acquire(self, now_ns: int) -> None:
        if self._next_ns is None:
            self._next_ns = int(now_ns) + SCAN_PERIOD_NS
            return
        if now_ns < self._next_ns:
            return
        # missed periods are skipped, not filled with the present pose (design 10.5): one revolution, ending at the
        # latest period boundary reached
        k = (int(now_ns) - self._next_ns) // SCAN_PERIOD_NS
        end_ns = self._next_ns + k * SCAN_PERIOD_NS
        last_s = (end_ns - self.start_ns) / 1e9
        scan = self._cast(last_s - SCAN_PERIOD_NS / 1e9, last_s)
        self._queue.append(scan)
        for sub in self.subscribers:
            sub(scan)
        self._next_ns = end_ns + SCAN_PERIOD_NS

    def scans(self, now_ns: int) -> list:
        if not self._open():
            return []
        self._acquire(int(now_ns))
        out, self._queue = self._queue[-4:], []
        return [] if self.stop_delivery else out

    def scan_source_events(self, now_ns: int) -> list:
        out, self.events = self.events, []
        return out


class SyntheticSlam(_Gated):
    """The 5c-2 SLAM link on the scan source's acquisitions: one processed reply per acquisition (its stamp the scan's
    first beam), the correction identity until a ``seed`` -- then the seeded map <- odom kept; ``request_processed``
    answers the first acquisition at or after its time; ``stop_delivery`` stops the replies only (the injection
    ``slam``)."""

    def __init__(self, transport, client, scan_source: SyntheticScan, *, map_id: str, odom_base) -> None:
        super().__init__(transport, client)
        self.map_id, self.odom_base = map_id, odom_base
        self.epoch, self.correction = 0, (0.0, 0.0, 0.0)
        self.stop_delivery = False
        self._pending: list = []
        self._requests: list = []
        self._scan_id = 0
        scan_source.subscribers.append(self._on_scan)

    def _drop(self) -> None:
        self._pending = []

    def _on_scan(self, scan) -> None:
        self._scan_id += 1
        self._pending.append((self._scan_id, float(scan.first_s), self.epoch))

    def relocalize(self, epoch) -> None:
        self.epoch = int(epoch)
        self._pending = [p for p in self._pending if p[2] == self.epoch]

    def seed(self, epoch, base_pose, stamp_s) -> None:
        self.epoch = int(epoch)
        self.correction = tuple(float(v) for v in compose(tuple(base_pose), invert(tuple(self.odom_base()))))
        self._pending = []

    def request_processed(self, epoch, attempt, not_before) -> None:
        self._requests.append((int(epoch), attempt, float(not_before)))

    def _reply(self, scan_id, stamp_s, attempt=None):
        r = SimpleNamespace(scan_id=scan_id, stamp_s=float(stamp_s), status="processed", slam_wall_s=None,
                            map_from_odom=tuple(self.correction), epoch=self.epoch, attempt=attempt)
        return (r, {}, None, None)

    def replies(self, now_ns: int) -> list:
        if not self._open():
            return []
        if self.stop_delivery:
            self._pending = []
            return []
        out = []
        for scan_id, stamp, epoch in self._pending:
            if epoch != self.epoch:
                continue
            attempt = None
            for req in list(self._requests):
                if req[0] == self.epoch and stamp >= req[2]:
                    self._requests.remove(req)
                    attempt = req[1]
                    break
            out.append(self._reply(scan_id, stamp, attempt))
        self._pending = []
        return out


class SyntheticImu(_Gated):
    """A still truck's accelerometer: gravity on z with 0.02 m/s^2 noise every 5 ms, host ns stamps."""

    def __init__(self, transport, client, *, seed: int = 0) -> None:
        super().__init__(transport, client)
        self._next_ns: int | None = None
        self._rng = random.Random(seed)

    def _drop(self) -> None:
        self._next_ns = None

    def samples(self, upto_ns: int, limit: int) -> list:
        if not self._open():
            return []
        if self._next_ns is None:
            self._next_ns = int(upto_ns) + IMU_PERIOD_NS
            return []
        out = []
        while self._next_ns <= upto_ns and len(out) < int(limit):
            n = lambda: self._rng.gauss(0.0, 0.02)  # noqa: E731
            out.append((int(self._next_ns), (n(), n(), 9.81 + n())))
            self._next_ns += IMU_PERIOD_NS
        return out


__all__ = ["SyntheticImu", "SyntheticScan", "SyntheticSlam"]
