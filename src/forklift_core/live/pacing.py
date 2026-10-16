"""Real-time pacing for a stepped simulation (M-c plan C1).

Pure: the wall time is an argument. Each physics step starts at least ``dt / target_factor`` of wall time after the
previous one started, so a late step is never made up by running the next ones faster (design 7.3: do not run
in bursts) and the simulation never runs faster than the declared factor. ``ok`` has hysteresis so the 10 Hz render
steps do not flip it: it turns true only after the windowed factor has stayed at or above
``min_ratio * target_factor`` for ``ok_hold_s``, and false when the window falls below that or one step took longer
than ``stall_s`` (a synchronous call: planning, a capture's waits).
"""

from __future__ import annotations

from collections import deque


class Pacer:
    def __init__(self, target_factor: float, dt: float, *, window_s: float = 1.0, min_ratio: float = 0.9,
                 ok_hold_s: float = 1.0, stall_s: float = 0.25) -> None:
        if target_factor <= 0 or dt <= 0:
            raise ValueError("target_factor and dt must be positive")
        self.target_factor, self.dt = float(target_factor), float(dt)
        self.window_s, self.min_ratio, self.ok_hold_s, self.stall_s = window_s, min_ratio, ok_hold_s, stall_s
        self.spacing_s = self.dt / self.target_factor
        self._last_start: float | None = None
        self._samples: deque = deque()  # (wall at a step's end, sim steps so far)
        self._steps = 0
        self.ok = False
        self._good_since: float | None = None
        self.lag_s = 0.0
        self.stalls = 0

    def before_step(self, wall_mono: float) -> float:
        """Seconds to sleep before the next physics step (0 when it is already due). The step is taken to start
        at ``wall_mono`` plus that sleep; a late start is not made up."""
        if self._last_start is None:
            self._last_start = wall_mono
            return 0.0
        due = self._last_start + self.spacing_s
        start = max(wall_mono, due)
        gap = start - self._last_start
        self.lag_s = max(0.0, gap - self.spacing_s)
        if gap > self.stall_s:
            self.stalls += 1
            self._fail()
        self._last_start = start
        return start - wall_mono

    def after_step(self, wall_mono: float) -> None:
        """A physics step ended at ``wall_mono``: update the factor and ``ok``."""
        self._steps += 1
        self._samples.append((wall_mono, self._steps))
        while self._samples and wall_mono - self._samples[0][0] > self.window_s:
            self._samples.popleft()
        factor = self.realtime_factor()
        if factor is None or factor < self.min_ratio * self.target_factor:
            self._fail()
            return
        if self._good_since is None:
            self._good_since = wall_mono
        if not self.ok and wall_mono - self._good_since >= self.ok_hold_s:
            self.ok = True

    def _fail(self) -> None:
        self.ok = False
        self._good_since = None

    def realtime_factor(self) -> float | None:
        """Simulated over wall time across the window (None until the window has spanned most of itself)."""
        if len(self._samples) < 2:
            return None
        (w0, n0), (w1, n1) = self._samples[0], self._samples[-1]
        if w1 - w0 < 0.5 * self.window_s:
            return None
        return (n1 - n0) * self.dt / (w1 - w0)

    def report(self) -> dict:
        return {"target_factor": self.target_factor, "realtime_factor": self.realtime_factor(), "ok": self.ok,
                "lag_s": self.lag_s, "stalls": self.stalls}
