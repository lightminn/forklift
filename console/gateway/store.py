"""Latest-value store between the ROS thread (writes) and the asyncio server (reads).

Nothing queues: each write replaces the previous value, and each media kind carries a
sequence number so the server sends a frame only once per subscriber.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, replace

from console.gateway.convert import (
    CorrectionTracker,
    GridMessage,
    PoseSample,
    ScanMessage,
    Snapshot,
)


@dataclass(frozen=True)
class CameraFrame:
    jpeg: bytes
    stamp_ns: int  # message header stamp (sensor or sim clock)
    received_s: float


class Store:
    def __init__(self):
        self._lock = threading.Lock()
        self._snap = Snapshot()
        self._corr = CorrectionTracker()
        self._grid: GridMessage | None = None
        self._scan: tuple[ScanMessage, PoseSample | None, str, float] | None = None
        self._camera: CameraFrame | None = None
        self.seq = {"grid": 0, "scan": 0, "camera_rgb": 0}
        self._notes: dict[str, str] = {}

    # ROS thread -----------------------------------------------------------
    def set_base(self, pose: PoseSample | None) -> None:
        with self._lock:
            self._snap.base = pose

    def set_map_to_odom(self, xyyaw, now: float) -> None:
        with self._lock:
            self._corr.update(xyyaw, now)

    def set_speed(self, mps: float, now: float) -> None:
        with self._lock:
            self._snap.speed_mps = mps
            self._snap.speed_received_s = now

    def set_grid(self, grid: GridMessage) -> None:
        with self._lock:
            self._grid = grid
            self._snap.grid_received_s = grid.received_s
            self.seq["grid"] += 1

    def set_scan(
        self,
        scan: ScanMessage,
        laser: PoseSample | None,
        pose_at: str = "scan_stamp",
        delay_s: float = 0.0,
    ) -> None:
        with self._lock:
            self._scan = (scan, laser, pose_at, delay_s)
            self._snap.scan_received_s = scan.received_s
            self.seq["scan"] += 1

    def set_camera(self, frame: CameraFrame) -> bool:
        """False (nothing stored) for a re-sent capture: same stamp as the one held."""
        with self._lock:
            held = self._camera
            if held is not None and frame.stamp_ns and frame.stamp_ns == held.stamp_ns:
                return False
            self._camera = frame
            self.seq["camera_rgb"] += 1
            return True

    def note(self, key: str, text: str | None) -> None:
        """A condition the operator should see (e.g. a rotated map); None clears it."""
        with self._lock:
            if text is None:
                self._notes.pop(key, None)
            else:
                self._notes[key] = text

    # asyncio side ---------------------------------------------------------
    def snapshot(self, now: float) -> Snapshot:
        with self._lock:
            return replace(
                self._snap,
                correction_version=self._corr.version or None,
                correction_age_s=self._corr.age(now),
                notes=sorted(self._notes.values()),
            )

    def latest(self, kind: str):
        """(sequence, value) for 'grid', 'scan' or 'camera_rgb'; value None if none yet."""
        with self._lock:
            value = {
                "grid": self._grid,
                "scan": self._scan,
                "camera_rgb": self._camera,
            }[kind]
            return self.seq[kind], value
