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
  then revokes when the correction jumps: the applied pose moved past the window's limit against the correction 2 m of
  path, at most 10 s, ago (revision plan 2026-11-07 9.5) -- or a stand's C_still by E_STILL, taken again at a hold's
  release. Delta_int against C_int goes on, for the re-match's condition and the record; a correction's slow move
  passes both gates by design: there is no cumulative bound on the true error.
- ``ended:<reason>`` cannot be left.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

from forklift_core.localization.start_reloc import rear_from_base
from forklift_core.localization.slam_pose import compose, invert

D_STROKE = 1.0  # design 3: one press, odometry
D_TOTAL = 3.0  # design 3: three strokes
D_MIN = 0.5  # design 3: the net displacement before a re-match
STILL_M, STILL_RAD = 0.002, 0.002  # a scan set's own stillness (scan_capture's), and a re-match result's
INT_M, INT_RAD = 0.20, 0.05  # design 1.5: the budget E_STILL stays within
REMATCH_M = 0.10  # design 1.5: past this Delta_int a stand re-matches
WINDOW_M, WINDOW_S = 2.0, 10.0  # the revision plan 9.5: the monitor's window of path, and its cap in time (9.7)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Bounds:
    """The development values (plan 0): the odometry bound dp(d) = a + b d, dth(d) = c + e d of a straight reverse,
    E_STILL, E_SLAM, the window's limit and the hold release's (metres, radians)."""

    odom: tuple
    e_still: tuple
    e_slam: tuple
    e_window: tuple = (0.153, 0.103)  # the revision plan 9.5; a bounds file must give it
    e_release: tuple = (0.065, 0.076)  # the revision plan 9.13: the 34 releases' largest x 1.5; a bounds file must give it

    def __post_init__(self):
        values = [*self.odom, *self.e_still, *self.e_slam, *self.e_window, *self.e_release]
        if (len(self.odom) != 4 or len(self.e_still) != 2 or len(self.e_slam) != 2 or len(self.e_window) != 2
                or len(self.e_release) != 2):
            raise ValueError("odom takes (a, b, c, e), e_still, e_slam, e_window and e_release (metres, radians)")
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


def tick_bound(speed_mps: float, tick_s: float, wheel_radius_m: float, counts_per_rev: int | None) -> float:
    """The most the odometry can move in one control tick at ``speed_mps``: the travel and, on a counted encoder
    model, one count of quantization (Codex piece 2a re-review P2: team_spec_quantized moved 5 counts, 0.96 mm, in a
    tick at 0.10 m/s -- 0.83 mm of travel). None: the engine's own rates, no counts."""
    count = 0.0 if counts_per_rev is None else 2 * math.pi * wheel_radius_m / counts_per_rev
    return abs(speed_mps) * tick_s + count


def gap(a, b, rear_offset_m: float) -> tuple:
    """(metres, radians) between two base poses, the metres the larger of the base and the rear axle."""
    ra, rb = rear_from_base(a, rear_offset_m), rear_from_base(b, rear_offset_m)
    return (max(math.hypot(a[0] - b[0], a[1] - b[1]), math.hypot(ra[0] - rb[0], ra[1] - rb[1])),
            abs(_wrap(a[2] - b[2])))


def agreement(r_scan, o_scan, C, o_now, rear_offset_m: float) -> tuple:
    """The SLAM agreement's (metres, radians): the applied pose C o O(now) against the evidence r_scan carried by the
    odometry from its scan to now, the metres the larger of the base and the rear axle."""
    r_now = compose(r_scan, compose(invert(o_scan), o_now))
    return gap(compose(C, o_now), r_now, rear_offset_m)


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
        self._window: deque = deque()  # (path, t, C) since the confirmation, cut to the window
        self.release_from: list = []  # hold releases since the last tick: (C before, C after, O), each judged once
        self._path, self._o_last = 0.0, None
        self._holding = False

    # --- transitions -------------------------------------------------------------------------------------------
    def _event(self, t, kind, **kw):
        self.events.append({"t": t, "kind": kind, **kw})

    def end(self, t: float, reason: str) -> None:
        if not self.status.startswith("ended:"):
            self.status = f"ended:{reason}"
            self._event(t, "ended", reason=reason)

    def _agree(self, t, r_scan, o_scan, C, o_now, kind) -> bool:
        """The SLAM agreement: the applied pose against the evidence carried to now; on success C_int."""
        c_now = compose(C, o_now)
        m, rad = agreement(r_scan, o_scan, C, o_now, self.rear_offset_m)
        ok = m <= self.bounds.e_slam[0] and rad <= self.bounds.e_slam[1]
        if ok and not self._window:  # the window starts at the confirmation
            self._window.append((self._path, t, tuple(C)))
            self._o_last = tuple(o_now)
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

    def note_release(self, c_before, c_after, o) -> None:
        """A SLAM hold was released since the last tick: what it replaced, what it applied, at its own odometry. This
        tick's first update judges each one on these, not on the tick's correction, which a later reply may have moved."""
        self.release_from.append((tuple(float(v) for v in c_before), tuple(float(v) for v in c_after),
                                  tuple(float(v) for v in o)))

    # --- every tick ------------------------------------------------------------------------------------------------
    def update(self, t: float, o, C, *, standing: bool, deadman: bool, mission_active: bool = False,
               near: bool = False, holding: bool = False) -> View:
        """This tick's view; called again in the same tick with a newer C (after the SLAM replies, before the output)
        it only re-judges the monitor. ``holding``: the SLAM correction is held: at its release the stand's reference
        is taken again (the release's own move is judged by the window and by note_release's gate, not by the stand)."""
        o = tuple(float(v) for v in o)
        releases, self.release_from = self.release_from, []
        if self._holding and not holding and not releases:
            self.c_still = None
        self._holding = holding
        new_stand = standing and not self.standing
        if new_stand:
            self.stand_id += 1
        self.standing = standing
        status = self.status
        if releases and status != "confirmed":
            self.c_still = None
        if status.startswith("ended:"):
            return View(status, reason="pose_ended")
        if status == "checking":
            return View(status, reason="pose_checking")
        if status == "confirmed":
            view = self._monitor(t, o, C, standing, new_stand, mission_active, near, releases)
            if self._rm is not None and view.status == "confirmed":
                # a confirmed re-match in progress: the controls locked as while checking, the monitor going on
                # (Codex piece 2a P2: it left driving, the forks and a submit open)
                return View("checking", reason="pose_checking", delta_int=view.delta_int)
            return view
        return self._strokes(t, o, standing, deadman)

    def _window_change(self, t, o, C, now) -> tuple:
        """The move of the applied pose at this odometry against the correction the window's start held."""
        if self._o_last is not None:
            self._path += _net(o, self._o_last)
        self._o_last = o
        self._window.append((self._path, t, tuple(C)))
        while len(self._window) > 1 and (self._path - self._window[0][0] > WINDOW_M
                                         or t - self._window[0][1] > WINDOW_S):
            self._window.popleft()
        return gap(now, compose(self._window[0][2], o), self.rear_offset_m)

    def _monitor(self, t, o, C, standing, new_stand, mission_active, near, releases=()) -> View:
        now = compose(C, o)
        m, rad = gap(now, compose(self.c_int, o), self.rear_offset_m)
        wm, wrad = self._window_change(t, o, C, now)
        if wm > self.bounds.e_window[0] or wrad > self.bounds.e_window[1]:
            self._event(t, "revoke", gate="window", delta_m=wm, delta_rad=wrad, delta_int=[m, rad])
            self.end(t, "pose_revoked:window")
            return View(self.status, reason="pose_ended", delta_int=(m, rad))
        for before, after, o_at in releases:
            rm, rrad = gap(compose(after, o_at), compose(before, o_at), self.rear_offset_m)
            if rm > self.bounds.e_release[0] or rrad > self.bounds.e_release[1]:
                self._event(t, "revoke", gate="release", delta_m=rm, delta_rad=rrad, delta_int=[m, rad])
                self.end(t, "pose_revoked:release")
                return View(self.status, reason="pose_ended", delta_int=(m, rad))
            self.c_still = after  # passed: the stand starts from what it applied; the window and C_int stay
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


__all__ = ["Bounds", "agreement", "tick_bound", "D_MIN", "D_STROKE", "D_TOTAL", "INT_M", "INT_RAD", "PoseCheck", "REMATCH_M", "STILL_M",
           "STILL_RAD", "View", "gap"]
