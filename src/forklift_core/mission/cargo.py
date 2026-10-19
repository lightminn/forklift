"""The fork relation and the load of a live session (persistent console plan D5a, 4th ed.).

Session state, not a mission's: what the last mission left the forks in stays after it ends. It is not proven from
geometry -- a live run has no near-field tracking and the map-frame estimates drift with time (plan D5a) -- but set
from where the mission put the forks (its transitions) and the lift/lower verdicts, and made unknown where a pallet
check fails. ``basis`` says so in what is published.

relation: ``free`` (forks outside any pallet), ``constrained`` (inside, the pallet supported elsewhere),
``carrying`` (inside, the pallet lifted -- the lift verdict passed), ``unknown``.
load: ``none``, ``acquiring`` (lifting), ``confirmed``, ``releasing`` (lowering), ``unknown``.
"""

from __future__ import annotations

import math

TRACE_STEP_M = 0.02  # the insert trace's spacing in depth (plan D4 6th ed.)
APPROACH_GAP_M = 0.10  # the approach ends with the fork tips this far before the face: depth -0.10 at the anchor


def _compose(a, b):
    x, y, t = a
    c, s = math.cos(t), math.sin(t)
    return (x + c * b[0] - s * b[1], y + s * b[0] + c * b[1], t + b[2])


def _invert(a):
    x, y, t = a
    c, s = math.cos(t), math.sin(t)
    return (-c * x - s * y, s * x - c * y, -t)


def _along(pose, distance_m):
    x, y, t = pose
    return (x + distance_m * math.cos(t), y + distance_m * math.sin(t), t)

# The transition into a phase, and what it leaves the forks in (plan D5a's table). A phase not listed changes
# nothing: observe, approach, settle and the return keep the forks outside a pallet.
_ON_ENTRY = {
    "insert": ("constrained", "none"),
    "lift": ("constrained", "acquiring"),
    "lower": ("constrained", "releasing"),
}
_ON_STEP = {
    ("lift", "extract"): ("carrying", "confirmed"),  # the lift verdict passed (the pallet is off the floor)
    ("lower", "withdraw"): ("constrained", "none"),  # the lower verdict passed (on the floor, forks still in)
    ("withdraw", "settle"): ("free", "none"),  # the withdrawal straight ran to its end
    ("recover_lower", "recover_withdraw"): ("constrained", "none"),  # lowered, on the floor (plan D4)
}


class CargoState:
    def __init__(self) -> None:
        self.relation = "free"  # a session starts with empty forks outside any pallet (the scene's contract)
        self.load = "none"
        self.pallet_id: str | None = None
        self.invalidated: str | None = None  # the pallet check that made it unknown
        # Plan D4 (6th ed.): the insert's axis and trace, from which the depth and a recovery's retrace are made.
        self.anchor = None  # (rear pose) where the depth was last anchored; the depth there is depth0
        self.depth0 = -APPROACH_GAP_M
        self.trace: list = []  # [(map pose, depth)] every TRACE_STEP_M of depth during the insert
        self.insert_end = None  # the rear pose at the insert's end (insert -> lift)
        self.placed: list | None = None  # the trace placed at the destination (once, at -> lower)
        self.trace_valid = True  # the simulation's truth check on the trace's premise (plan D4)
        self.last_rear = None  # the control pose at the last tick's end
        # Plan D14: what a manual recovery needs once the mission that left the forks in a pocket is gone (the
        # pallet's ID and estimate, the SLAM hold's basis, the stage); None when there is none.
        self.recovery_context: dict | None = None
        self._last_phase = None

    def on_transition(self, from_phase: str | None, to_phase: str, pallet_id: str | None) -> None:
        if (from_phase, to_phase) in _ON_STEP:
            self.relation, self.load = _ON_STEP[(from_phase, to_phase)]
        elif to_phase in _ON_ENTRY and from_phase != to_phase:
            self.relation, self.load = _ON_ENTRY[to_phase]
        else:
            return
        if pallet_id is not None:
            self.pallet_id = pallet_id
        self.invalidated = None

    # --- plan D4: the depth along the insert axis and the recovery's retrace ----------------------------------------
    def on_insert_start(self, rear) -> None:
        self.anchor, self.depth0 = tuple(float(v) for v in rear), -APPROACH_GAP_M
        self.trace = [(self.anchor, -APPROACH_GAP_M)]
        self.insert_end, self.placed, self.trace_valid = None, None, True

    def on_insert_end(self, rear) -> None:
        self.insert_end = tuple(float(v) for v in rear)
        self.depth0 = self.depth(rear)  # fixed while the pallet rides the forks
        self.anchor = self.insert_end

    def on_lower_start(self, rear) -> None:
        """At the destination: the trace placed once (the pallet rode the forks at the insert end's relative pose);
        the depth anchored here with its fixed value."""
        rear = tuple(float(v) for v in rear)
        if self.insert_end is not None:
            change = _compose(rear, _invert(self.insert_end))
            self.placed = [(_compose(change, pose), depth) for pose, depth in self.trace]
        self.anchor = rear

    def observe(self, phase: str | None, rear) -> None:
        """Each control tick's end (live): the anchors at the phase changes (that tick's pose is the transition's --
        the approach's arrival, the insert's end, the lowering's start) and the trace while inserting, by depth."""
        rear = tuple(float(v) for v in rear)
        self.last_rear = rear
        if phase != self._last_phase:
            if phase == "insert":
                self.on_insert_start(rear)
            elif phase == "lift" and self._last_phase == "insert":
                self.on_insert_end(rear)
            elif phase == "lower":
                self.on_lower_start(rear)
            self._last_phase = phase
        if phase != "insert" or self.anchor is None or not self.trace:
            return
        depth = self.depth(rear)
        if depth - self.trace[-1][1] >= TRACE_STEP_M:
            self.trace.append((tuple(float(v) for v in rear), depth))

    def _net_m(self, rear) -> float:
        ax, ay, at = self.anchor
        return (float(rear[0]) - ax) * math.cos(at) + (float(rear[1]) - ay) * math.sin(at)

    def depth(self, rear) -> float:
        """How far the fork tips are past the face (negative: before it) -- the net displacement along the anchor's
        heading, never a sum of moves (a standing truck's odometry noise sums up, its net does not)."""
        return self.depth0 + self._net_m(rear)

    def band(self, rear) -> float:
        """The depth's error band: 0.02 m + 5 % of the net displacement since the anchor (measured, plan 6a)."""
        return 0.02 + 0.05 * abs(self._net_m(rear))

    def _pose_at(self, trace, depth: float):
        first_pose, first_depth = trace[0]
        last_pose, last_depth = trace[-1]
        if depth <= first_depth:
            return _along(first_pose, depth - first_depth)
        if depth >= last_depth:
            return _along(last_pose, depth - last_depth)
        for (p0, d0), (p1, d1) in zip(trace, trace[1:]):
            if d0 <= depth <= d1:
                w = (depth - d0) / (d1 - d0) if d1 > d0 else 0.0
                return tuple(a + w * (b - a) for a, b in zip(p0, p1))
        return _along(last_pose, depth - last_depth)

    def recovery_path(self, rear, step_m: float) -> list:
        """The retrace from the depth now to -(0.10 + band): the trace (the destination's placement once made, else as
        recorded), extended before its start along its heading, cut at the depth now. [] when already there."""
        trace = self.placed if self.placed is not None else self.trace
        if not trace:
            return []
        now, end = self.depth(rear), -(APPROACH_GAP_M + self.band(rear))
        if now <= end:
            return []
        depths = []
        d = now
        while d > end + 1e-6:
            depths.append(d)
            d -= step_m
        if not depths or depths[-1] - end > 1e-6:
            depths.append(end)  # the end itself, never a repeat of the last sample (the tracker refuses repeats)
        poses = [self._pose_at(trace, d) for d in depths]
        return poses if len(poses) >= 2 else []

    def alignment(self, rear) -> tuple[float, float] | None:
        """(lateral, heading) offset of ``rear`` from the pocket axis -- the pose the trace (the destination's
        placement once made) puts at the depth now, which a retrace follows (plan D14). None without a trace."""
        trace = self.placed if self.placed is not None else self.trace
        if not trace or self.anchor is None:
            return None
        rear = tuple(float(v) for v in rear)
        ex, ey, et = self._pose_at(trace, self.depth(rear))
        lateral = -(rear[0] - ex) * math.sin(et) + (rear[1] - ey) * math.cos(et)
        return lateral, math.atan2(math.sin(rear[2] - et), math.cos(rear[2] - et))

    def reexpress(self, change) -> None:
        """Every map pose the cargo keeps, carried by a change of the applied correction (plan D14: the truck's
        relative geometry to the pocket is kept; its own motion comes in through the odometry)."""
        change = tuple(float(v) for v in change)
        move = lambda pose: None if pose is None else _compose(change, tuple(pose))  # noqa: E731
        self.anchor, self.insert_end, self.last_rear = move(self.anchor), move(self.insert_end), move(self.last_rear)
        self.trace = [(move(pose), depth) for pose, depth in self.trace]
        if self.placed is not None:
            self.placed = [(move(pose), depth) for pose, depth in self.placed]

    def cancel_decision(self, rear, *, in_withdraw: bool) -> str:
        """What a live cancel confirmed standing leads to: ``cancelled``, ``recover`` or ``recovery_required``."""
        if self.relation in ("free", "carrying"):
            return "cancelled"
        if self.relation == "unknown" or not self.trace_valid or self.anchor is None:
            return "recovery_required"
        if in_withdraw and self.depth(rear) < -(APPROACH_GAP_M + self.band(rear)):
            self.relation, self.load = "free", "none"  # already out (a late withdraw cancel)
            return "cancelled"
        return "recover"

    def recovered(self) -> None:
        self.relation, self.load = "free", "none"

    def invalidate(self, reason: str) -> None:
        """A pallet check failed (dropped, slipped, displaced, forbidden contact, a measured pallet overlap, its
        tilt): whatever the forks were in is no longer known -- with or without a mission."""
        self.relation, self.load, self.invalidated = "unknown", "unknown", reason

    def submit_refusal(self) -> str | None:
        if self.relation in ("constrained", "unknown"):
            return "recovery_required"
        if self.relation == "carrying":
            return "cargo_held"  # until the cargo submit (plan D5, stage 6c)
        return None

    def release_allowed(self) -> bool:
        """The end SLAM release (plan D1 step 4): the truck may drive on its own after it."""
        return self.relation in ("free", "carrying")

    def loaded_outline(self) -> bool:
        """The outline the permits use: the load's, unless the forks are known to be empty or merely inside."""
        return self.relation == "carrying" or self.load in ("acquiring", "releasing", "unknown")

    def safety_fields(self) -> dict:
        return {"load": {"loaded": self.loaded_outline(), "basis": "phase+cargo", "pallet_id": self.pallet_id,
                         "state": self.load},
                "fork_in_pocket": {"value": self.relation != "free", "basis": "phase+cargo",
                                   "relation": self.relation}}


# Plan D5 6c item 2a: how far the carried pallet may sit from its nominal seat on the forks for a loaded submit (the
# normal transport's guarantee: the nominal loaded outline the plans and permits use covers it as it covers a
# transport that was never cancelled).
LOADED_SEAT_M = 0.02
LOADED_SEAT_RAD = 0.02


def deliverable_as_carried(cargo: CargoState, rel, inserted_offset_m: float, *, supported: bool) -> str | None:
    """The pallet a submit may deliver as it is carried (plan D5 6c items 1 and 2a), else None: the forks carry it,
    its trace evidence holds, the run is one the loaded start supports (``supported``: grid planning with SLAM, the
    known-pallet layer off), and ``rel`` -- the pallet centre relative to the rear axle, from the same source as the
    trace evidence -- is within LOADED_SEAT_M along and across and LOADED_SEAT_RAD in heading (either way round) of
    the nominal seat (``inserted_offset_m`` ahead, centred, aligned)."""
    if cargo.relation != "carrying" or not cargo.trace_valid or cargo.pallet_id is None or not supported:
        return None
    if rel is None:
        return None
    dx, dy, dyaw = float(rel[0]) - inserted_offset_m, float(rel[1]), float(rel[2])
    turned = math.atan2(math.sin(2.0 * dyaw), math.cos(2.0 * dyaw)) / 2.0  # 0 and pi are the same seat
    if abs(dx) > LOADED_SEAT_M or abs(dy) > LOADED_SEAT_M or abs(turned) > LOADED_SEAT_RAD:
        return None
    return cargo.pallet_id
