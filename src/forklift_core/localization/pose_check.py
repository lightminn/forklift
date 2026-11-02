"""The unconfirmed position state machine (docs/plans/2026-11-02-unconfirmed-runner.md 2.2, design
docs/plans/2026-11-02-unconfirmed-position.md 1.4-1.5). Pure: the runner hands in each tick's odometry (base_link,
odom frame), the applied correction C (map <- odom), the stand and the deadman; the machine says what is open.

- ``checking`` from the first live tick until the start check's result (``start``); a re-match in progress too.
- ``unconfirmed``: only the straight reverse, one stroke at a time. A stroke opens on the deadman, measures its net
  displacement from where it opened (wheel noise at a stand adds nothing) and closes once the distance left -- to
  D_STROKE in the stroke and to D_TOTAL over all of them -- is within the stopping distance; the next one opens after
  a stand with the deadman let go. After D_MIN of net displacement from the start, every new stand asks for a re-match.
- ``confirmed`` only through the SLAM agreement: the applied pose against the evidence carried by the odometry, both
  on the base link and on the rear axle. C_int is the evidence's own correction (evidence o O_scan^-1). The monitor
  then revokes when the correction moves C_int's pose by 0.20 m / 0.05 rad, or a stand's C_still by E_STILL.
- ``ended:<reason>`` cannot be left.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from forklift_core.localization.start_reloc import rear_from_base
from forklift_core.localization.slam_pose import compose, invert

D_STROKE = 1.0  # design 3: one press, odometry
D_TOTAL = 3.0  # design 3: three strokes
D_MIN = 0.5  # design 3: the net displacement before a re-match
STILL_M, STILL_RAD = 0.002, 0.002  # a scan set's own stillness (scan_capture's), and a re-match result's
INT_M, INT_RAD = 0.20, 0.05  # design 1.5: the integrity budget
REMATCH_M = 0.10  # design 1.5: half the revocation gate


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Bounds:
    """The development values (plan 0): the odometry bound dp(d) = a + b d, dth(d) = c + e d of a straight reverse,
    E_STILL and E_SLAM (metres, radians)."""

    odom: tuple
    e_still: tuple
    e_slam: tuple

    def __post_init__(self):
        values = [*self.odom, *self.e_still, *self.e_slam]
        if len(self.odom) != 4 or len(self.e_still) != 2 or len(self.e_slam) != 2:
            raise ValueError("odom takes (a, b, c, e), e_still and e_slam (metres, radians)")
        if not all(math.isfinite(v) and v >= 0 for v in values):
            raise ValueError("the bounds must be finite and non-negative")
        if self.e_still[0] > INT_M or self.e_still[1] > INT_RAD:
            raise ValueError("E_STILL stays within 0.20 m, 0.05 rad (design 3)")

    def dp(self, d: float) -> float:
        return self.odom[0] + self.odom[1] * d

    def dth(self, d: float) -> float:
        return self.odom[2] + self.odom[3] * d


@dataclass
class View:
    status: str
    drive_ready: bool = False  # the unconfirmed straight reverse may drive this tick
    stroke: str | None = None  # ready | active | closed | spent (unconfirmed only)
    stroke_left_m: float | None = None
    total_left_m: float | None = None
    want_rematch: bool = False
    reason: str | None = None  # why nothing is open (for the policy)
    delta_int: tuple | None = None  # (m, rad), the larger of base and rear axle
    d: float | None = None  # net displacement from the start check's scan


def gap(a, b, rear_offset_m: float) -> tuple:
    """(metres, radians) between two base poses, the metres the larger of the base and the rear axle."""
    ra, rb = rear_from_base(a, rear_offset_m), rear_from_base(b, rear_offset_m)
    return (max(math.hypot(a[0] - b[0], a[1] - b[1]), math.hypot(ra[0] - rb[0], ra[1] - rb[1])),
            abs(_wrap(a[2] - b[2])))


def _net(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


@dataclass
class PoseCheck:
    bounds: Bounds
    rear_offset_m: float
    stop_m: float  # the stopping distance at V_REV_U (the obstacle layer's model)
    tick_m: float = 0.0  # the most one control tick can move at V_REV_U: the brake is judged once a tick
    status: str = "checking"
    events: list = field(default_factory=list)

    def __post_init__(self):
        self.origin = None  # O_scan of the start check: the net displacement's start
        self.c_int = None
        self.c_still = None
        self.stroke = "ready"
        self.o_s = None  # where the open stroke started
        self.used_m = 0.0  # the closed strokes' net displacements
        self.standing = None
        self.stand_id = 0
        self.rematched_at = None  # the stand a re-match was last started at
        self._rm = None  # the re-match in progress: (phase, epoch, control_gen)

    # --- transitions -------------------------------------------------------------------------------------------
    def _event(self, t, kind, **kw):
        self.events.append({"t": t, "kind": kind, **kw})

    def end(self, t: float, reason: str) -> None:
        if not self.status.startswith("ended:"):
            self.status = f"ended:{reason}"
            self._event(t, "ended", reason=reason)

    def _agree(self, t, r_scan, o_scan, C, o_now, kind) -> bool:
        """The SLAM agreement: the applied pose against the evidence carried to now; on success C_int."""
        r_now = compose(r_scan, compose(invert(o_scan), o_now))
        c_now = compose(C, o_now)
        m, rad = gap(c_now, r_now, self.rear_offset_m)
        ok = m <= self.bounds.e_slam[0] and rad <= self.bounds.e_slam[1]
        self._event(t, kind, agree_m=m, agree_rad=rad, ok=ok, evidence=list(r_scan), applied=list(c_now))
        if ok:
            self.c_int = compose(r_scan, invert(o_scan))
        return ok

    def start(self, t: float, *, confirmed: bool, r_scan, o_scan, C, o_now, reasons=(), hyps=()) -> None:
        """The start check's result (r_scan: the evidence, at o_scan)."""
        if self.status != "checking" or self._rm is not None:
            return
        self.origin = tuple(o_scan)
        self._event(t, "start", confirmed=confirmed, reasons=list(reasons), hypotheses=len(hyps))
        if not confirmed:
            self.status = "unconfirmed"
        elif self._agree(t, r_scan, o_scan, C, o_now, "slam_agreement"):
            self.status = "confirmed"
        else:
            self.end(t, "pose_slam_disagree")

    def begin_rematch(self, t: float, *, epoch, control_gen) -> None:
        if self.status not in ("unconfirmed", "confirmed"):
            return
        self._rm = (self.status, epoch, control_gen)
        self.rematched_at = self.stand_id
        if self.status == "unconfirmed":
            self.status = "checking"
        self._event(t, "rematch_begin", phase=self._rm[0])

    def rematch_result(self, t: float, outcome: str, g, o_scan, *, o_now, C, epoch, control_gen) -> None:
        """Unconfirmed: ``outcome`` is twin_check.rematch_decision's. Confirmed: ``confirm`` when the matcher confirmed
        and the conservative check passed there, ``none`` otherwise (no evidence)."""
        if self._rm is None or self.status.startswith("ended:"):
            return
        phase, epoch0, gen0 = self._rm
        self._rm = None
        if phase == "unconfirmed":
            self.status = "unconfirmed"
        moved_m, moved_rad = gap(tuple(o_now), tuple(o_scan), self.rear_offset_m)  # base and rear axle (plan 0)
        moved = moved_m > STILL_M or moved_rad > STILL_RAD
        if moved or epoch != epoch0 or (phase == "confirmed" and control_gen != gen0):
            self._event(t, "rematch_discarded", phase=phase, outcome=outcome, moved=moved, epoch=epoch,
                        control_gen=control_gen)
            return
        self._event(t, "rematch", phase=phase, outcome=outcome, g=None if g is None else list(g))
        if phase == "unconfirmed":
            if outcome == "confirm":
                if self._agree(t, g, o_scan, C, o_now, "slam_agreement"):
                    self.status = "confirmed"
                else:
                    self.end(t, "pose_slam_disagree")
            elif outcome in ("twin_preferred", "out_of_set"):
                self.end(t, f"pose_{outcome}")
            return
        if outcome == "confirm" and not self._agree(t, g, o_scan, C, o_now, "rematch_agreement"):
            self.end(t, "pose_revoked:rematch")

    # --- every tick ------------------------------------------------------------------------------------------------
    def update(self, t: float, o, C, *, standing: bool, deadman: bool, mission_active: bool = False,
               near: bool = False) -> View:
        """This tick's view; called again in the same tick with a newer C (after the SLAM replies, before the output)
        it only re-judges the monitor."""
        o = tuple(float(v) for v in o)
        new_stand = standing and not self.standing
        if new_stand:
            self.stand_id += 1
        self.standing = standing
        status = self.status
        if status.startswith("ended:"):
            return View(status, reason="pose_ended")
        if status == "checking":
            return View(status, reason="pose_checking")
        if status == "confirmed":
            view = self._monitor(t, o, C, standing, new_stand, mission_active, near)
            if self._rm is not None and view.status == "confirmed":
                # a confirmed re-match in progress: the controls locked as while checking, the monitor going on
                # (Codex piece 2a P2: it left driving, the forks and a submit open)
                return View("checking", reason="pose_checking", delta_int=view.delta_int)
            return view
        return self._strokes(t, o, standing, deadman)

    def _monitor(self, t, o, C, standing, new_stand, mission_active, near) -> View:
        now = compose(C, o)
        m, rad = gap(now, compose(self.c_int, o), self.rear_offset_m)
        if m > INT_M or rad > INT_RAD:
            self._event(t, "revoke", gate="int", delta_m=m, delta_rad=rad)
            self.end(t, "pose_revoked:int")
            return View(self.status, reason="pose_ended", delta_int=(m, rad))
        if standing:
            if new_stand or self.c_still is None:
                self.c_still = tuple(C)
            sm, srad = gap(now, compose(self.c_still, o), self.rear_offset_m)
            if sm > self.bounds.e_still[0] or srad > self.bounds.e_still[1]:
                self._event(t, "revoke", gate="still", delta_m=sm, delta_rad=srad)
                self.end(t, "pose_revoked:still")
                return View(self.status, reason="pose_ended", delta_int=(m, rad))
        else:
            self.c_still = None
        want = (standing and not near and not mission_active and m > REMATCH_M and self._rm is None
                and self.rematched_at != self.stand_id)
        return View("confirmed", want_rematch=want, delta_int=(m, rad))

    def _strokes(self, t, o, standing, deadman) -> View:
        total_left = D_TOTAL - self.used_m
        if self.stroke == "ready" and deadman:
            self.stroke, self.o_s = "active", o
            self._event(t, "stroke_open", used_m=self.used_m)
        s = _net(o, self.o_s) if self.o_s is not None else 0.0
        if self.stroke == "active":
            left = min(D_STROKE - s, total_left - s)
            # Codex piece 2a P2: judged once a tick, so the brake allows for one tick's travel before the stop
            if left <= self.stop_m + self.tick_m or not deadman:
                self.stroke = "closed"
                self._event(t, "stroke_close", s_m=s, released=not deadman)
        if self.stroke == "closed" and standing and not deadman:
            self.used_m += s
            self.o_s, s = None, 0.0
            total_left = D_TOTAL - self.used_m
            self.stroke = "spent" if total_left <= self.stop_m else "ready"
            self._event(t, "stroke_done", used_m=self.used_m)
        d = _net(o, self.origin)
        want = (standing and not deadman and self.stroke != "active" and d >= D_MIN and self._rm is None
                and self.rematched_at != self.stand_id)
        ready = self.stroke in ("ready", "active")
        reason = None if ready else ("pose_unconfirmed:total" if self.stroke == "spent" else "pose_unconfirmed:stroke")
        return View("unconfirmed", drive_ready=ready, stroke=self.stroke,
                    stroke_left_m=min(D_STROKE - s, total_left - s), total_left_m=total_left - s,
                    want_rematch=want, reason=reason, d=d)


__all__ = ["Bounds", "D_MIN", "D_STROKE", "D_TOTAL", "INT_M", "INT_RAD", "PoseCheck", "REMATCH_M", "STILL_M",
           "STILL_RAD", "View", "gap"]
