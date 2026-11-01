"""The wheel lock on a zero-command stand (docs/plans/2026-11-01-standstill-wheel-hold.md S1).

The wheel drives are velocity drives (stiffness 0): a truck standing on a zero command is held by nothing but their
damping, and a loaded one creeps forwards (riib batch 2: up to 0.20 m in 236 s). The planned dwells already hold all
four wheels where they stand (the runner's DWELL_WHEEL_KP). This keeps one lock state for both:

- ``none``: the original gains;
- ``stand``: a zero command held ``STAND_LOCK_AFTER_S`` in a driving phase with the rear wheels below
  ``STAND_LOCK_WHEEL_RAD_S`` -- the rear (driven) wheels held, or all four for the cause probe (``"all"``);
- ``dwell``: the planned dwell's lock, all four, which wins over ``stand``.

The original gains are read once, when a lock starts from ``none``, and put back on every return to ``none``. Wheel
order: front left, front right, rear left, rear right.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

STAND_LOCK_AFTER_S = 0.5
STAND_LOCK_WHEEL_RAD_S = 0.05  # about 6 mm/s at the 0.125 m wheel
STAND_PHASES = frozenset({"observe", "approach", "insert", "extract", "transport", "withdraw", "return_home"})
WHEEL_KP = 2000.0  # the dwell lock's stiffness (N m/rad), the same torque limit
REAR = (2, 3)


@dataclass(frozen=True)
class HoldOutput:
    gains: tuple | None  # (kps, kds) to set this tick, or None for no change
    q: np.ndarray | None  # the four wheels' position targets while locked, None when not


class WheelHold:
    def __init__(self, stand_mode: str, wheel_indices=(0, 1, 2, 3), kp: float = WHEEL_KP):
        if stand_mode not in ("rear", "all"):
            raise ValueError(f"stand_mode {stand_mode!r}: 'rear' or 'all' (off keeps the runner's dwell lock as is)")
        self.stand_mode, self.wheel_indices, self.kp = stand_mode, np.asarray(wheel_indices), float(kp)
        self.mode = "none"
        self.base = None  # the original (kps, kds), read when a lock starts from none
        self.q = None
        self.zero_since = None
        self.events: list[dict] = []

    def _stand_ok(self, t, wheel_cmd, phase, wheel_rates) -> bool:
        zero = phase in STAND_PHASES and bool(np.all(np.asarray(wheel_cmd, dtype=float) == 0.0))
        if not zero:
            self.zero_since = None
            return False
        if self.zero_since is None:
            self.zero_since = t
        if self.mode == "stand":
            return True  # held: it stays while the command is zero in a driving phase
        rear = [float(wheel_rates[i]) for i in REAR]
        rates_ok = all(math.isfinite(r) for r in rear) and max(abs(r) for r in rear) < STAND_LOCK_WHEEL_RAD_S
        return t - self.zero_since >= STAND_LOCK_AFTER_S - 1e-9 and rates_ok

    def _gains(self, mode):
        kps, kds = (np.asarray(g, dtype=float).copy() for g in self.base)
        if mode == "dwell" or (mode == "stand" and self.stand_mode == "all"):
            kps[self.wheel_indices] = self.kp
        elif mode == "stand":
            kps[self.wheel_indices[list(REAR)]] = self.kp
        return kps, kds

    def step(self, t: float, *, dwell_standing: bool, wheel_cmd, phase: str, wheel_rates, q_now,
             read_gains) -> HoldOutput:
        stand = self._stand_ok(t, wheel_cmd, phase, wheel_rates)
        commanded = not bool(np.all(np.asarray(wheel_cmd, dtype=float) == 0.0))
        # a non-zero wheel command is never applied locked, whatever the dwell flag says (Codex impl review P2-3)
        want = "none" if commanded else ("dwell" if dwell_standing else ("stand" if stand else "none"))
        if want == self.mode:
            return HoldOutput(None, None if self.q is None else self.q.copy())
        q_now = np.asarray(q_now, dtype=float)
        if self.mode == "none":
            self.base = tuple(np.asarray(g, dtype=float).copy() for g in read_gains())
            self.q = q_now.copy()
        elif want == "dwell":  # stand -> dwell: the rear keep their target, the front from now
            self.q = self.q.copy()
            front = [i for i in range(len(self.q)) if i not in REAR]
            self.q[front] = q_now[front]
        self.events.append({"t": float(t), "from": self.mode, "to": want, "phase": phase,
                            "q": None if want == "none" else self.q.tolist()})
        if want == "none":
            gains = tuple(np.asarray(g, dtype=float).copy() for g in self.base)
            self.mode, self.q, self.base = "none", None, None
            return HoldOutput(gains, None)
        self.mode = want
        return HoldOutput(self._gains(want), self.q.copy())


RESTART_MOVE_M = 0.005  # the restart: 5 mm of true travel (the runner's moved_after_s)
RESTART_TAIL_S = 5.0


class RestartTail:
    """Which ticks the S0/S1 probe log keeps: every zero-command tick, and after one the non-zero ticks from the first
    (whose pose is the start) up to the tick that has moved ``RESTART_MOVE_M`` or is ``RESTART_TAIL_S`` past it -- the
    command-start-to-motion time in either lock setting (the S1 batch compares the off arm's with the lock's)."""

    def __init__(self):
        self.armed, self.t0, self.xy = False, None, None

    def keep(self, t: float, zero: bool, xy) -> bool:
        if zero:
            self.armed, self.t0, self.xy = True, None, None
            return True
        if not self.armed:
            return False
        if self.t0 is None:
            self.t0, self.xy = float(t), (float(xy[0]), float(xy[1]))
        if math.hypot(float(xy[0]) - self.xy[0], float(xy[1]) - self.xy[1]) > RESTART_MOVE_M or float(t) - self.t0 > RESTART_TAIL_S:
            self.armed = False  # this tick is the tail's last line
        return True
