"""The C2 injection's detour (docs/plans/2026-11-03-near-end-stand-c2-injection.md 1.2, diagnostic only).

``--inject-near-end-detour``: once per run, on the near re-observation leg's final reverse leg at least R1_M long, the
observe tracker reads its input pose through a rigid offset about the original final goal (``D_M`` to the goal
heading's left times -s, ``PSI_RAD`` times s, s the sign of the final leg's last R1_M mean curvature) from R1_M before
the goal; the tracker clears it itself before an update that would leave R2_M or less. The goal, ``paths["observe"]``,
the path arrays, truth, SLAM and the grid are untouched, so C2 plans again to the original target.

This module keeps the decision, the watch and the record (``state["near_end_injections"]``, plain data); the runner
calls ``NearEndDetour.tick`` once a tick ahead of the follower.
"""

from __future__ import annotations

import math

import numpy as np

R1_M = 0.80  # armed this far before the final goal (and the final leg at least this long)
R2_M = 0.10  # cleared inside the tracker before an update that would leave this much or less
D_M = 0.06  # the offset to the goal heading's left, times -s
PSI_RAD = 0.15  # the turn about the goal, times s


def final_leg(tracker) -> tuple[int, float, int]:
    """The final leg's first sample, its length along the path and its gear: the start is the previous leg's end, or 0
    on a path without a cusp (a single leg -- ``_leg_ends`` holds the goal alone; plan 1.2, Codex third review)."""
    ends = tracker._leg_ends
    start = int(ends[-2]) if len(ends) >= 2 else 0
    return start, float(tracker._distance[-1] - tracker._distance[start]), int(tracker._directions[-1])


def curvature_sign(tracker, start: int) -> tuple[float, float]:
    """s, the sign of the mean curvature over the final leg's samples within R1_M of the goal (+1 at zero), and that
    mean."""
    distance = np.asarray(tracker._distance, dtype=float)
    index = np.arange(len(distance))
    mask = (distance >= distance[-1] - R1_M) & (index >= start)
    mean = float(np.mean(np.asarray(tracker._curvatures, dtype=float)[mask]))
    return (1.0 if mean >= 0.0 else -1.0), mean


def decide(tracker) -> dict:
    """The arming decision for a tracker on its final leg: armed with s, D and psi, or not armed with the reason."""
    start, length, gear = final_leg(tracker)
    decision = {"final_leg_start": start, "final_leg_length_m": length, "final_leg_direction": gear}
    if gear > 0:
        return {**decision, "event": "not_armed", "reason": "forward_final_leg"}
    if length < R1_M:
        return {**decision, "event": "not_armed", "reason": "final_leg_short"}
    s, mean = curvature_sign(tracker, start)
    return {**decision, "event": "armed", "kappa_mean_inv_m": mean, "s": s, "dx_left_m": -D_M * s,
            "dpsi_rad": PSI_RAD * s}


def path_distance_m(tracker, xy) -> float:
    """The distance from xy to the tracker's (original) path polyline."""
    p = np.asarray(tracker._poses[:, :2], dtype=float)
    a, v = p[:-1], np.diff(p, axis=0)
    f = np.clip(np.einsum("ij,ij->i", np.asarray(xy, dtype=float) - a, v) / np.einsum("ij,ij->i", v, v), 0.0, 1.0)
    return float(np.min(np.linalg.norm(a + f[:, None] * v - np.asarray(xy, dtype=float), axis=1)))


class NearEndDetour:
    """One run's injection: the decision (once a run), and per mission the watch on the armed tracker and the
    pallet-shift windows. A window's end None means still open where the record ends (the run or mission ended).

    The order inside a tick matters (Codex implementation review 1 P2-2, 2 P2-1): what the follower did in the last
    tick (a judgement, a tracker change, an arrival, the phase) is read first and closes or opens the windows; only
    then do the pallet's shifts since that follower call -- every physics step's largest, a capture's included, kept
    by the runner in ``state["near_end_step_shift_max_m"]``, and this tick's own -- go to the windows still open. The
    steps never write the record itself, so one taken after a mission's end cannot touch what it handed over (2 P2-3).
    """

    def __init__(self) -> None:
        self.decided = False  # once a run, whatever the missions
        self._reset_mission()

    def _reset_mission(self) -> None:
        self.tracker = None  # the armed tracker (by identity); None before, when not armed and after the mission
        self.watch_open = False  # until the stand's first C2 judgement, a tracker change, a failure, an arrival or
        # the observe phase's end
        self.restored = False
        self.install_open = False
        self._install_seen = False
        self._seen_events = 0
        self._record_obj = None
        self._trackers = []  # the observe trackers seen while watching, by identity (their ordinal is recorded)

    def _record(self, state: dict) -> dict:
        if self._record_obj is None:
            self._record_obj = state.setdefault("near_end_injections", {
                "events": [], "pallet_shift_max_m": {"injection": None, "install": None},
                "windows": {"injection": None, "install": None}, "max_path_distance_m": None,
            })
        return self._record_obj

    def _sync_mission(self, state: dict) -> None:
        """A mission's end puts its keys back (MissionBaseline.restore drops near_end_injections): the watch and the
        windows were that mission's (P2-3). The run's one decision stays spent."""
        if self._record_obj is not None and state.get("near_end_injections") is not self._record_obj:
            self._reset_mission()

    def _ordinal(self, tracker) -> int:
        for i, seen in enumerate(self._trackers):
            if seen is tracker:
                return i
        self._trackers.append(tracker)
        return len(self._trackers) - 1

    def _sample(self, pallet_shift_m: float) -> None:
        """A pallet shift (its distance from where it started) for the windows open now."""
        if self._record_obj is None:
            return
        for window, open_ in (("injection", self.watch_open), ("install", self.install_open)):
            if open_:
                old = self._record_obj["pallet_shift_max_m"][window]
                self._record_obj["pallet_shift_max_m"][window] = (
                    float(pallet_shift_m) if old is None else max(old, float(pallet_shift_m)))

    def tick(self, t: float, phase: str, tracker, near_leg: bool, state: dict, pallet_shift_m: float, rear,
             can_arm: bool = True) -> None:
        """Ahead of the follower: decide once, then watch. ``near_leg`` is ``core.near_reobserve_leg()``;
        ``pallet_shift_m`` the pallet's distance from where it started (the runner's pick-up check, run in observe
        too); ``rear`` the control rear-axle pose; ``can_arm`` false on a paused or warming tick."""
        self._sync_mission(state)
        steps_max = state.pop("near_end_step_shift_max_m", None)
        if (not self.decided and can_arm and phase == "observe" and near_leg and tracker is not None
                and not tracker._arrived and not tracker._failed and not tracker._pending
                and tracker.leg_index == len(tracker._leg_ends) - 1
                and tracker.remaining_to_goal_m() <= R1_M):
            self.decided = True
            decision = decide(tracker)
            record = self._record(state)
            record["events"].append({**decision, "time_s": float(t),
                                     "remaining_m": float(tracker.remaining_to_goal_m()),
                                     "progress_m": float(tracker.progress_m()), "segment": int(tracker._segment),
                                     "leg_index": int(tracker.leg_index), "tracker_ordinal": self._ordinal(tracker),
                                     "tracker_sha256": _digest(tracker)})
            if decision["event"] == "armed":
                tracker.set_input_offset(decision["dx_left_m"], decision["dpsi_rad"], R2_M)
                self.tracker, self.watch_open, self._seen_events = tracker, True, len(tracker.input_offset_events)
                record["windows"]["injection"] = [float(t), None]
        if self.watch_open:
            self._watch(t, phase, tracker, state, rear)
        self._install_window(t, phase, state)
        self._sample(pallet_shift_m if steps_max is None else max(float(pallet_shift_m), float(steps_max)))

    def _watch(self, t: float, phase: str, tracker, state: dict, rear) -> None:
        record = self._record_obj
        # the armed tracker's own events: its clear, and an arrival under the offset (never expected)
        for event in self.tracker.input_offset_events[self._seen_events:]:
            record["events"].append({**event, "event": "restored" if event["event"] == "cleared" else event["event"],
                                     "time_s": float(t)})
            self.restored = self.restored or event["event"] == "cleared"
        self._seen_events = len(self.tracker.input_offset_events)
        stands = [r for r in state.get("observe_end_stands", ()) if r.get("kind") == "near"
                  and r.get("time_s", -math.inf) >= record["windows"]["injection"][0]]
        closed = None
        if stands:
            # the stand's first C2 judgement closes the watch (C2's own install replaces the tracker after it, so it
            # is never counted as a supersede -- plan 2 call order)
            closed = float(stands[0]["time_s"])
            record["events"].append({"event": "judged", "time_s": closed, "outcome": stands[0].get("outcome"),
                                     "restored_before": self.restored})
        elif tracker is not self.tracker:
            closed = float(t)
            record["events"].append({"event": "superseded_after_restore" if self.restored else "superseded",
                                     "time_s": closed, "phase": phase,
                                     "tracker_ordinal": None if tracker is None else self._ordinal(tracker)})
        elif self.tracker._failed:
            closed = float(t)
            record["events"].append({"event": "failed", "failure": self.tracker._failure, "time_s": closed,
                                     "under_offset": self.tracker.input_offset_active()})
        elif phase != "observe" or self.tracker._arrived:
            closed = float(t)
            record["events"].append({"event": "arrived" if self.tracker._arrived else "left_observe",
                                     "time_s": closed, "phase": phase})
        if closed is not None:
            self.watch_open = False
            record["windows"]["injection"][1] = closed
            record.pop("watch_last", None)
            return
        # where the watch stood at its last tick: a run that ends inside the follower call (a failure raising at once)
        # leaves no closing event, and this says what the tracker was doing then (review 2 P3)
        record["watch_last"] = {"time_s": float(t), "offset_active": self.tracker.input_offset_active(),
                                "remaining_m": float(self.tracker.remaining_to_goal_m()), "phase": phase}
        if self.tracker.input_offset_active():
            distance = path_distance_m(self.tracker, rear[:2])
            old = record["max_path_distance_m"]
            record["max_path_distance_m"] = distance if old is None else max(old, distance)

    def _install_window(self, t: float, phase: str, state: dict) -> None:
        """Condition 4's pallet window, armed or not: from the mission's first C2 install to the observe phase's end
        (the runner's own pick-up check runs in approach and insert only)."""
        if self.install_open:
            if phase != "observe":
                self.install_open = False
                self._record_obj["windows"]["install"][1] = float(t)
            return
        if self._install_seen:
            return
        first = next((r for r in state.get("observe_end_stands", ())
                      if r.get("kind") == "near" and r.get("outcome") == "installed"), None)
        if first is None:
            return
        self._record(state)["windows"]["install"] = [float(first["time_s"]), None]
        self.install_open = self._install_seen = True
        if phase != "observe":  # the mission left observe within this tick: nothing of the window is left to sample
            self.install_open = False
            self._record_obj["windows"]["install"][1] = float(t)


def _digest(tracker) -> str:
    import hashlib

    poses, directions, curvatures = tracker.path_arrays()
    return hashlib.sha256(np.asarray(poses, dtype=float).tobytes() + np.asarray(directions, dtype=np.int8).tobytes()
                          + np.asarray(curvatures, dtype=float).tobytes()).hexdigest()
