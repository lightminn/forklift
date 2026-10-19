"""The far recognition the Isaac mission reads (docs/plans/2026-10-18-isaac-recognition-far-capture.md).

Per frame: the oracle instance masks as proposals, the instance fit, the single-frame target decision
(the frame's whole fit, failures included, so an ambiguous or unfitted object in the radius still asks to
look again) and the far-range tracker. Within one observation leg the target is chosen once -- by the first
frame whose decision selects a candidate a visible track took -- and never replaced (design §8): at the stop
the selected track has to take the stop frame's selected reading, or the leg ends without a pallet
(target_lost / target_inconsistent). A leg with no choice before the stop lets the stop frame choose. The
chosen candidate becomes the mission's PocketObservation (known shapes of the run's pallet depth only).
SDK-free: the runner supplies the scene, the instance ids, odom<-base and the instruction in base_link.
"""
from __future__ import annotations

import math

import numpy as np

from forklift_core.perception.candidate_tracking import CandidateTracker, Frame
from forklift_core.perception.instance_fit import SceneFit, fit_scene
from forklift_core.perception.instance_proposals import InstanceProposal
from forklift_core.perception.pocket_observation import Pocket, PocketObservation
from forklift_core.perception.target_selection import select_target

PROVENANCE = "synthetic"  # an Isaac render, as the detector's observations are


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def proposals_from_ids(instance_ids) -> list:
    """One oracle proposal per instance id >= 0 (background and unlabelled pixels are negative)."""
    ids = np.asarray(instance_ids)
    return [InstanceProposal(f"i{k}", ids == k, 1.0, "oracle") for k in sorted(int(v) for v in np.unique(ids) if v >= 0)]


def instruction_from_zone(centre_xy, radius_m: float, map_from_base) -> dict:
    """The operator's "the pallet around here" (a zone centre in map) in base_link at the control pose
    (x, y, yaw) -- the frame select_target compares candidates in."""
    x, y, yaw = (float(v) for v in map_from_base)
    dx, dy = float(centre_xy[0]) - x, float(centre_xy[1]) - y
    c, s = math.cos(yaw), math.sin(yaw)
    return {"present": True, "x_m": c * dx + s * dy, "y_m": -s * dx + c * dy, "radius_m": float(radius_m)}


def decide(fit, instruction):
    """The single-frame decision over the frame's whole fit (2B evaluation semantics)."""
    return select_target(fit.candidates, instruction, unresolved=fit.failures)


def face_distance(face, camera_x_m: float) -> float:
    """Distance from the camera (on the base x axis) to a face's pocket midpoint along its insertion axis."""
    yaw = face["insertion_yaw_rad"]
    mid = (np.asarray(face["left_m"][:2]) + np.asarray(face["right_m"][:2])) / 2
    return float((mid - np.array([camera_x_m, 0.0])) @ np.array([math.cos(yaw), math.sin(yaw)]))


def pocket_errors(cand, face) -> dict:
    """Worst-pocket lateral / along, yaw and height of a candidate against a truth face."""
    yaw = face["insertion_yaw_rad"]
    a = np.array([math.cos(yaw), math.sin(yaw)])
    n = np.array([-a[1], a[0]])
    out = {"lateral_m": 0.0, "along_m": 0.0, "height_m": 0.0}
    for side, key in (("left", "left_m"), ("right", "right_m")):
        d = np.subtract(getattr(cand, side).centre_m, face[key])
        out["lateral_m"] = max(out["lateral_m"], abs(float(d[:2] @ n)))
        out["along_m"] = max(out["along_m"], abs(float(d[:2] @ a)))
        out["height_m"] = max(out["height_m"], abs(float(d[2])))
    out["yaw_rad"] = abs(_wrap(cand.insertion_yaw_rad - yaw))
    return out


def far_error(cand, face, *, camera_x_m: float) -> dict:
    """The far estimate against the truth face: the evaluator's errors at the frame's camera distance."""
    return {"distance_m": face_distance(face, camera_x_m), "errors": pocket_errors(cand, face)}


def candidate_to_observation(cand, *, pallet_depth_m: float, stamp_ns: int, clock_domain: str) -> PocketObservation:
    """A known-shape candidate whose entry-axis depth is the run's pallet depth, as the detector's observation:
    front-plane opening centres in insertion-left order (instance_fit._pockets), no sigma (a tracker budget is
    an empirical p95, not a 1-sigma bound)."""
    spec = cand.shape.spec
    if cand.hypothesis.startswith("parametric") or spec is None:
        raise ValueError(f"a known shape is required, got {cand.hypothesis}")
    depth = spec.overall_depth_m if cand.axis == "x" else spec.overall_width_m
    if abs(depth - float(pallet_depth_m)) > 1e-3:
        raise ValueError(f"entry-axis depth {depth:.4f} m is not the run's pallet depth {pallet_depth_m:.4f} m")
    return PocketObservation(
        int(stamp_ns), clock_domain, "base_link", PROVENANCE, "valid",
        Pocket(cand.left.centre_m, cand.left.width_m, cand.left.height_m),
        Pocket(cand.right.centre_m, cand.right.width_m, cand.right.height_m),
        float(cand.insertion_yaw_rad), None, None, None)


def no_pallet(stamp_ns: int, clock_domain: str, reason: str) -> PocketObservation:
    return PocketObservation(int(stamp_ns), clock_domain, "base_link", PROVENANCE, "no_pallet",
                             None, None, None, None, None, reason)


def _candidate_record(c) -> dict:
    return {"candidate_id": c.candidate_id, "proposal_id": c.proposal_id, "hypothesis": c.hypothesis, "geometry": c.geometry,
            "identity": c.identity, "range_m": float(c.range_m), "insertion_yaw_rad": float(c.insertion_yaw_rad),
            "left_m": list(c.left.centre_m), "right_m": list(c.right.centre_m),
            "pallet_centre_m": list(c.pallet_centre_m), "pickable": c.pickable}


class FarRecognition:
    """Frames of one observation leg, its target and the stop decision."""

    def __init__(self, bounds, *, specs: dict, setup, load_for, clock_domain: str = "ros_sim", proposer=None,
                 identity_threshold=None):
        """``proposer(scene, instance_ids) -> (proposals, extra failures)``: the oracle instance masks by default,
        the learned proposer (with its geometric fallback) in stage 3; ``identity_threshold`` is tau on the learned
        score (instance_fit._identity). The instance ids are the evaluation truth -- only the oracle reads them."""
        self.bounds, self.specs, self.setup, self.load_for = bounds, specs, setup, load_for
        self.clock_domain = clock_domain
        self.proposer = proposer or (lambda scene, ids: (proposals_from_ids(ids), ()))
        self.identity_threshold = identity_threshold
        self.last_proposals: list = []
        self.start_leg()

    def start_leg(self) -> None:
        """A new observation leg (a re-observation included): a new tracker and no target."""
        self.tracker = CandidateTracker(self.bounds)
        self._chosen = None  # the leg's target track once chosen; never replaced within the leg
        self.selection_history: list = []
        self.frames: list = []

    def _step(self, scene, instance_ids, odom_from_base, instruction):
        proposals, extra = self.proposer(scene, instance_ids)
        self.last_proposals = list(proposals)
        fit = fit_scene(scene, proposals, specs=self.specs, setup=self.setup, load_for=self.load_for,
                        identity_threshold=self.identity_threshold)
        fit = SceneFit(fit.candidates, tuple(fit.failures) + tuple(extra))
        sel = decide(fit, instruction)
        report = self.tracker.update(Frame(scene.stamp_ns, self.clock_domain, 0, odom_from_base, fit.candidates))
        visible = {t.track_id: t for t in self.tracker.tracks(scene.stamp_ns, odom_from_base)}
        taken = report.accepted.get(sel.candidate_id) if sel.status == "selected" else None
        record = {"stamp_ns": int(scene.stamp_ns), "candidates": [_candidate_record(c) for c in fit.candidates],
                  "failures": [{"proposal_id": f.proposal_id, "reason": f.reason} for f in fit.failures],
                  "decision": {"status": sel.status, "candidate_id": sel.candidate_id, "reasons": list(sel.reasons)},
                  "taken_by": taken, "visible": sorted(visible), "untracked": report.untracked, "held": report.held,
                  "new": report.new, "suppressed": report.suppressed}
        return fit, sel, visible, taken, record

    def _choose(self, track_id: int, stamp_ns: int) -> None:
        self.tracker.select(track_id)
        self._chosen = track_id
        self.selection_history.append({"stamp_ns": int(stamp_ns), "track": track_id})

    def frame(self, scene, instance_ids, odom_from_base, instruction) -> dict:
        """A driving frame: tracked, and the leg's target chosen by the first decision a visible track took."""
        _, _, visible, taken, record = self._step(scene, instance_ids, odom_from_base, instruction)
        # The leg's target deleted (not merely stale) while driving: the caller ends this leg and starts a new
        # re-observation leg with the next frame (A', Codex) -- this frame stays in the ended leg.
        record["leg_lost"] = self._chosen is not None and self.tracker.selected is None
        if record["leg_lost"]:
            self.frames.append(record)
            return record
        if self._chosen is None and taken is not None and taken in visible:
            self._choose(taken, scene.stamp_ns)
        self.frames.append(record)
        return record

    def stop(self, scene, instance_ids, odom_from_base, instruction, *, pallet_depth_m: float):
        """The stop frame's decision: (PocketObservation, record)."""
        fit, sel, visible, taken, record = self._step(scene, instance_ids, odom_from_base, instruction)
        self.frames.append(record)
        out = {"status": sel.status, "reasons": list(sel.reasons), "selection_history": list(self.selection_history),
               "frames": len(self.frames), "source": None, "selected_track": None, "observations": 0,
               "reason": None, "stop": record}

        def fail(reason):
            out["reason"] = reason
            return no_pallet(scene.stamp_ns, self.clock_domain, reason), out

        if sel.status != "selected":
            return fail(sel.reasons[0] if sel.reasons else sel.status)
        single = next(c for c in fit.candidates if c.candidate_id == sel.candidate_id)
        if self._chosen is not None:
            if self.tracker.selected is None:
                return fail("target_lost")
            if taken != self._chosen or taken not in visible:
                return fail("target_inconsistent")
        elif taken is not None and taken in visible:
            self._choose(taken, scene.stamp_ns)
        if self._chosen is not None:
            tracked = visible[self._chosen]
            used, out["source"], out["observations"] = tracked.candidate, "fused", tracked.observations
            out["budget"] = dict(tracked.budget)
            out["selected_track"] = self._chosen
        else:
            used, out["source"], out["observations"] = single, "single", 1
        out["selection_history"] = list(self.selection_history)  # after a choice the stop itself made (Codex P3)
        out["single"] = _candidate_record(single)
        out["used"] = _candidate_record(used)
        try:
            obs = candidate_to_observation(used, pallet_depth_m=pallet_depth_m, stamp_ns=scene.stamp_ns,
                                           clock_domain=self.clock_domain)
        except ValueError as exc:
            return fail(f"unsupported:{exc}")
        return obs, out


def estimate_lag(frame_steps, measured_m, truth_by_step, *, max_lag: int, residual_max_m: float = 0.010,
                 margin_m: float = 0.001) -> dict:
    """The pixel lag tau (physics steps) and a constant bias b with measured(k) = truth(k - tau) + b.

    Accepted only when tau is identifiable -- the best p95 residual is under residual_max_m and every tau at
    least two steps away is worse by margin_m (a constant speed lets the bias absorb any lag) -- and the two
    halves of the frames (start and stop of the leg) give the same tau."""
    truth = np.asarray(truth_by_step, float)
    steps = np.asarray(frame_steps, int)
    meas = np.asarray(measured_m, float)

    def fit(idx):
        scores = {}
        for tau in range(0, max_lag + 1):
            k = steps[idx] - tau
            if (k < 0).any():
                continue
            r = meas[idx] - truth[k]
            b = float(np.mean(r))
            scores[tau] = (float(np.percentile(np.abs(r - b), 95)), b)
        best = min(scores, key=lambda t: scores[t][0])
        others = [scores[t][0] for t in scores if abs(t - best) >= 2]
        unique = bool(others) and min(others) >= scores[best][0] + margin_m
        return best, scores[best], unique

    all_idx = np.arange(len(steps))
    tau, (res, bias), unique = fit(all_idx)
    half = len(steps) // 2
    tau_a, (res_a, _), unique_a = fit(all_idx[:half])
    tau_b, (res_b, _), unique_b = fit(all_idx[half:])
    accepted = bool(unique and unique_a and unique_b and tau_a == tau == tau_b and res <= residual_max_m)
    # each segment identifies tau on its own when it is unique and fits (the whole leg, then each half)
    identified = [bool(u and r <= residual_max_m) for u, r in ((unique, res), (unique_a, res_a), (unique_b, res_b))]
    return {"tau_steps": int(tau), "bias_m": bias, "residual_p95_m": res, "halves": [int(tau_a), int(tau_b)],
            "halves_residual_p95_m": [res_a, res_b], "unique": [unique, unique_a, unique_b],
            "identified": identified, "accepted": accepted}


__all__ = ["FarRecognition", "candidate_to_observation", "decide", "estimate_lag", "face_distance", "far_error",
           "instruction_from_zone", "no_pallet", "pocket_errors", "proposals_from_ids"]
