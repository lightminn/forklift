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


def curvature_sign(tracker, start: int) -> float:
    """s: the sign of the mean curvature over the final leg's samples within R1_M of the goal (+1 at zero)."""
    distance = np.asarray(tracker._distance, dtype=float)
    index = np.arange(len(distance))
    mask = (distance >= distance[-1] - R1_M) & (index >= start)
    mean = float(np.mean(np.asarray(tracker._curvatures, dtype=float)[mask]))
    return 1.0 if mean >= 0.0 else -1.0


def decide(tracker) -> dict:
    """The arming decision for a tracker on its final leg: armed with s, D and psi, or not armed with the reason."""
    start, length, gear = final_leg(tracker)
    decision = {"final_leg_start": start, "final_leg_length_m": length, "final_leg_direction": gear}
    if gear > 0:
        return {**decision, "event": "not_armed", "reason": "forward_final_leg"}
    if length < R1_M:
        return {**decision, "event": "not_armed", "reason": "final_leg_short"}
    s = curvature_sign(tracker, start)
    return {**decision, "event": "armed", "s": s, "dx_left_m": -D_M * s, "dpsi_rad": PSI_RAD * s}


def path_distance_m(tracker, xy) -> float:
    """The distance from xy to the tracker's (original) path polyline."""
    p = np.asarray(tracker._poses[:, :2], dtype=float)
    a, v = p[:-1], np.diff(p, axis=0)
    f = np.clip(np.einsum("ij,ij->i", np.asarray(xy, dtype=float) - a, v) / np.einsum("ij,ij->i", v, v), 0.0, 1.0)
    return float(np.min(np.linalg.norm(a + f[:, None] * v - np.asarray(xy, dtype=float), axis=1)))


class NearEndDetour:
    """One run's injection: the decision (once), the watch on the armed tracker and the pallet-shift windows."""

    def __init__(self) -> None:
        self.decided = False
        self.tracker = None  # the armed tracker (by identity); None before and when not armed
        self.watch_open = False  # until the stand's first C2 judgement, an arrival or the observe phase's end
        self.restored = False
        self.judged_at = None
        self.install_open = False
        self._install_seen = False
        self._seen_events = 0

    def _record(self, state: dict) -> dict:
        return state.setdefault("near_end_injections", {
            "events": [], "pallet_shift_max_m": {"injection": None, "install": None},
            "windows": {"injection": None, "install": None}, "max_path_distance_m": None,
        })

    @staticmethod
    def _raise(record: dict, window: str, value: float) -> None:
        old = record["pallet_shift_max_m"][window]
        record["pallet_shift_max_m"][window] = value if old is None else max(old, value)

    def tick(self, t: float, phase: str, tracker, near_leg: bool, state: dict, pallet_shift_m: float, rear) -> None:
        """Ahead of the follower: decide once, then watch. ``near_leg`` is ``core.near_reobserve_leg()``;
        ``pallet_shift_m`` the pallet's distance from where it started (the runner's pick-up check, run in observe
        too); ``rear`` the control rear-axle pose."""
        if not self.decided:
            if (phase == "observe" and near_leg and tracker is not None and not tracker._arrived
                    and not tracker._failed and not tracker._pending
                    and tracker.leg_index == len(tracker._leg_ends) - 1
                    and tracker.remaining_to_goal_m() <= R1_M):
                self.decided = True
                decision = decide(tracker)
                record = self._record(state)
                record["events"].append({**decision, "time_s": float(t),
                                         "remaining_m": float(tracker.remaining_to_goal_m()),
                                         "progress_m": float(tracker.progress_m()), "segment": int(tracker._segment),
                                         "leg_index": int(tracker.leg_index),
                                         "tracker_sha256": _digest(tracker)})
                if decision["event"] == "armed":
                    tracker.set_input_offset(decision["dx_left_m"], decision["dpsi_rad"], R2_M)
                    self.tracker, self.watch_open, self._seen_events = tracker, True, len(tracker.input_offset_events)
                    record["windows"]["injection"] = [float(t), None]
                    self._raise(record, "injection", float(pallet_shift_m))
        self._install_window(t, phase, state, pallet_shift_m)
        if self.tracker is None or (not self.watch_open and not self.tracker.input_offset_active()):
            return  # not armed, or the watch closed with the offset gone
        record = self._record(state)
        # the armed tracker's own events: its clear, and an arrival under the offset (never expected)
        for event in self.tracker.input_offset_events[self._seen_events:]:
            record["events"].append({**event, "event": "restored" if event["event"] == "cleared" else event["event"],
                                     "time_s": float(t)})
            self.restored = self.restored or event["event"] == "cleared"
        self._seen_events = len(self.tracker.input_offset_events)
        if self.tracker.input_offset_active():
            distance = path_distance_m(self.tracker, rear[:2])
            old = record["max_path_distance_m"]
            record["max_path_distance_m"] = distance if old is None else max(old, distance)
        stands = [r for r in state.get("observe_end_stands", ()) if r.get("kind") == "near"
                  and r.get("time_s", -math.inf) >= record["events"][0]["time_s"]]
        if self.watch_open:
            self._raise(record, "injection", float(pallet_shift_m))
            if stands:
                # the stand's first C2 judgement closes the watch (C2's own install replaces the tracker after it,
                # so it is never counted as a supersede -- plan 2 call order)
                self.watch_open, self.judged_at = False, float(stands[0]["time_s"])
                record["windows"]["injection"][1] = self.judged_at
                record["events"].append({"event": "judged", "time_s": self.judged_at,
                                         "outcome": stands[0].get("outcome"), "restored_before": self.restored})
            elif tracker is not self.tracker:
                self.watch_open = False
                record["windows"]["injection"][1] = float(t)
                record["events"].append({"event": "superseded_after_restore" if self.restored else "superseded",
                                         "time_s": float(t), "phase": phase})
            elif phase != "observe" or self.tracker._arrived:
                self.watch_open = False
                record["windows"]["injection"][1] = float(t)
                record["events"].append({"event": "arrived" if self.tracker._arrived else "left_observe",
                                         "time_s": float(t), "phase": phase})

    def _install_window(self, t: float, phase: str, state: dict, pallet_shift_m: float) -> None:
        """Condition 4's pallet window, armed or not: from the run's first C2 install to the observe phase's end (the
        runner's own pick-up check runs in approach and insert only)."""
        if not self.install_open and (self._install_seen or not any(
                r.get("kind") == "near" and r.get("outcome") == "installed" for r in state.get("observe_end_stands", ()))):
            return
        record = self._record(state)
        if not self.install_open:
            first = next(r for r in state["observe_end_stands"] if r.get("kind") == "near" and r.get("outcome") == "installed")
            record["windows"]["install"] = [float(first["time_s"]), None]
            self.install_open = self._install_seen = True
        self._raise(record, "install", float(pallet_shift_m))
        if phase != "observe":
            self.install_open = False
            record["windows"]["install"][1] = float(t)


def _digest(tracker) -> str:
    import hashlib

    poses, directions, curvatures = tracker.path_arrays()
    return hashlib.sha256(np.asarray(poses, dtype=float).tobytes() + np.asarray(directions, dtype=np.int8).tobytes()
                          + np.asarray(curvatures, dtype=float).tobytes()).hexdigest()
