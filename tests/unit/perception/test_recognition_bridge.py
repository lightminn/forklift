# tests/unit/perception/test_recognition_bridge.py
"""The far recognition bridge into the Isaac mission (docs/plans/2026-10-18-isaac-recognition-far-capture.md)."""
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from forklift_core.geometry import RigidTransform
from forklift_core.perception import recognition_bridge as B
from forklift_core.perception.candidate_tracking import TrackBounds
from forklift_core.perception.fork_fit import ForkSetup, LoadDeclaration
from forklift_core.perception.instance_fit import FitFailure, SceneFit, fit_scene
from tools import evaluate_track_sequences as E
from tools import instance_rig
from tools import recognition_scenes as rs
from tools import recognition_truth as rt

ROOT = Path(__file__).resolve().parents[3]
EPAL, T11 = rs.known_spec("epal6"), rs.known_spec("t11_06")
SPECS = {"epal6": EPAL, "t11_06": T11}
SITE = {s.name: LoadDeclaration(10.0, 0.0, s.overall_depth_m / 2, 0.0, "site_default") for s in (EPAL, T11)}
BOUNDS = TrackBounds.load(ROOT / "config/recognition_track_bounds.yaml")
S = 1_000_000_000


def _load(spec):
    return None if spec is None else SITE.get(spec.name)


def _render(items):
    """items: (spec, Placement) in base_link -> (scene, instance ids)."""
    placed = [rt.placed_boxes(rt.instance_boxes(s, ()), p) for s, p in items]
    out = instance_rig.render_instances(placed)
    return out.scene, out.instance_ids


def _fit(scene, ids):
    return fit_scene(scene, B.proposals_from_ids(ids), specs=SPECS, setup=ForkSetup(0.3725), load_for=_load)


def _odom(x=0.0, y=0.0, yaw=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    return RigidTransform("base_link", "odom", ((c, -s, 0), (s, c, 0), (0, 0, 1)), (x, y, 0.0))


def _far():
    return B.FarRecognition(BOUNDS, specs=SPECS, setup=ForkSetup(0.3725), load_for=_load)


INSTR = {"present": True, "x_m": 2.8, "y_m": 0.0, "radius_m": 1.0}


# ---- proposals, instruction ----------------------------------------------------------------------------

def test_proposals_are_the_oracle_instance_masks_and_skip_background():
    ids = np.array([[-1, 0, 0], [1, 1, -1]])
    props = B.proposals_from_ids(ids)
    assert [p.proposal_id for p in props] == ["i0", "i1"] and {p.source for p in props} == {"oracle"}
    assert props[0].mask.tolist() == [[False, True, True], [False, False, False]]


@pytest.mark.parametrize("base, want", [((2.0, 0.65, 0.0), (1.1, 0.0)), ((3.1, -0.35, math.pi / 2), (1.0, 0.0))])
def test_the_zone_instruction_is_given_in_the_base_frame_at_the_control_pose(base, want):
    instr = B.instruction_from_zone((3.1, 0.65), 1.2, base)
    assert instr["present"] and instr["radius_m"] == 1.2
    assert (instr["x_m"], instr["y_m"]) == pytest.approx(want, abs=1e-9)


# ---- the single-frame decision --------------------------------------------------------------------------

def test_a_fitted_pallet_in_the_radius_is_selected():
    scene, ids = _render([(EPAL, rt.Placement(2.8, 0.0, 0.0, 0.0))])
    sel = B.decide(_fit(scene, ids), INSTR)
    assert sel.status == "selected"


def test_an_unresolved_object_in_the_radius_asks_to_look_again():
    scene, ids = _render([(EPAL, rt.Placement(2.8, 0.0, 0.0, 0.0))])
    fit = _fit(scene, ids)
    fit = SceneFit(fit.candidates, (FitFailure("i9", "no_face", (2.9, 0.3), 0.1),))
    assert B.decide(fit, INSTR).status == "reobserve"


def test_the_floor_contract_excludes_an_unfitted_object_behind_the_pallet():
    """plan 2026-10-24-far-target-occupancy O1: the decision passes the margin to select_target, on a real fit."""
    scene, ids = _render([(EPAL, rt.Placement(2.8, 0.0, 0.0, 0.0))])
    fit = _fit(scene, ids)
    fit = SceneFit(fit.candidates, (FitFailure("i9", "no_face", (3.9, 0.0), 0.2),))
    instr = {"present": True, "x_m": 2.8, "y_m": 0.0, "radius_m": 0.2}
    assert B.decide(fit, instr).status == "reobserve"
    sel = B.decide(fit, instr, occupancy_margin_m=0.05)
    assert sel.status == "selected" and "occupancy_excluded" in sel.reasons and sel.detail["excluded"] == ["i9"]


def test_an_ambiguous_reading_beside_the_target_asks_to_look_again():
    # Codex: tracks alone drop the uncounted candidate -- the frame's whole fit decides
    scene, ids = _render([(EPAL, rt.Placement(2.8, 0.0, 0.0, 0.0))])
    fit = _fit(scene, ids)
    other = replace(fit.candidates[0], candidate_id="c9", geometry="ambiguous", reasons=("test",),
                    pallet_centre_m=(3.0, 0.4, 0.0))
    assert B.decide(SceneFit(fit.candidates + (other,), ()), INSTR).status == "reobserve"


# ---- the observation the mission reads ------------------------------------------------------------------

def test_the_observation_keeps_the_pockets_and_the_pallet_centre():
    scene, ids = _render([(EPAL, rt.Placement(2.8, 0.1, 0.05, 0.0))])
    cand = _fit(scene, ids).candidates[0]
    obs = B.candidate_to_observation(cand, pallet_depth_m=EPAL.overall_depth_m, stamp_ns=5, clock_domain="ros_sim")
    assert obs.status == "valid" and obs.source_provenance == "synthetic"
    assert obs.left.center_m == pytest.approx(cand.left.centre_m) and obs.right.center_m == pytest.approx(cand.right.centre_m)
    assert obs.insertion_yaw_rad == pytest.approx(cand.insertion_yaw_rad)
    assert obs.position_sigma_m is None and obs.yaw_sigma_rad is None  # a p95 budget is not a 1-sigma bound
    left = np.asarray(obs.left.center_m[:2]) - np.asarray(obs.right.center_m[:2])
    a = np.array([math.cos(obs.insertion_yaw_rad), math.sin(obs.insertion_yaw_rad)])
    assert a[0] * left[1] - a[1] * left[0] > 0  # left is to the left of the insertion axis
    from sim.isaac.perception_adapter import estimate_pallet_center_m
    assert estimate_pallet_center_m(obs, EPAL.overall_depth_m) == pytest.approx(cand.pallet_centre_m[:2], abs=0.02)


def test_the_candidate_record_keeps_what_the_side_evidence_targets():
    # Plan 2026-10-19 task 4 (Codex design round 1 P2): the axis and the support height go with the hypothesis.
    scene, ids = _render([(EPAL, rt.Placement(2.8, 0.1, 0.05, 0.0))])
    cand = _fit(scene, ids).candidates[0]
    rec = B._candidate_record(cand)
    assert (rec["hypothesis"], rec["axis"], rec["support_z_m"]) == (cand.hypothesis, cand.axis, cand.support_z_m)
    assert rec["candidate_id"] == cand.candidate_id


def test_a_reading_of_another_depth_is_refused():
    scene, ids = _render([(T11, rt.Placement(2.8, 0.0, 0.0, 0.0))])
    cand = _fit(scene, ids).candidates[0]
    with pytest.raises(ValueError, match="depth"):
        B.candidate_to_observation(cand, pallet_depth_m=EPAL.overall_depth_m, stamp_ns=0, clock_domain="ros_sim")


def test_a_parametric_reading_is_refused():
    scene, ids = _render([(EPAL, rt.Placement(2.8, 0.0, 0.0, 0.0))])
    cand = _fit(scene, ids).candidates[0]
    with pytest.raises(ValueError, match="known"):
        B.candidate_to_observation(replace(cand, hypothesis="parametric:x"), pallet_depth_m=EPAL.overall_depth_m,
                                   stamp_ns=0, clock_domain="ros_sim")


def test_the_far_error_uses_the_evaluators_worst_pocket_and_the_actual_camera():
    # Codex: the evaluator's errors (worst pocket) and _distance with the frame's camera x, not the rig's 0.75 m
    scene, ids = _render([(EPAL, rt.Placement(2.8, 0.0, 0.0, 0.0))])
    cand = _fit(scene, ids).candidates[0]
    face = {"left_m": cand.left.centre_m, "right_m": cand.right.centre_m, "insertion_yaw_rad": 0.0}
    rec = B.far_error(cand, face, camera_x_m=0.619)
    assert rec["distance_m"] == pytest.approx(E._distance(face, camera_x=0.619))
    assert rec["errors"] == pytest.approx(E.errors(cand, face))


# ---- the leg: frames, selection, stop ------------------------------------------------------------------

def _frame_at(x_robot, t, items=((EPAL, 3.5),), far=None):
    """The robot at odom x_robot looking along +x at pallets fixed at odom x (spec, x)."""
    scene, ids = _render([(s, rt.Placement(x - x_robot, 0.0, 0.0, 0.0)) for s, x in items])
    scene = replace(scene, stamp_ns=int(round(t * S)))
    instr = B.instruction_from_zone((3.5 + EPAL.overall_depth_m / 2, 0.0), 1.0, (x_robot, 0.0, 0.0))
    return scene, ids, _odom(x_robot), instr


def test_driving_frames_select_once_and_the_stop_uses_the_fused_selected_track():
    far = _far()
    far.start_leg()
    for k in range(4):
        far.frame(*_frame_at(0.12 * k, 0.2 * k))
    obs, rec = far.stop(*_frame_at(0.48, 0.8), pallet_depth_m=EPAL.overall_depth_m)
    assert rec["source"] == "fused" and rec["status"] == "selected" and rec["observations"] >= 2
    assert obs.status == "valid"
    assert rec["selected_track"] == rec["selection_history"][0]["track"]


def test_without_driving_frames_the_stop_frame_decides_alone():
    far = _far()
    far.start_leg()
    obs, rec = far.stop(*_frame_at(0.0, 0.0), pallet_depth_m=EPAL.overall_depth_m)
    assert obs.status == "valid" and rec["source"] in ("fused", "single") and rec["observations"] == 1


def test_a_selected_track_that_did_not_take_the_stop_reading_is_a_lost_target():
    # Codex: an output 0.2 s old must not stand in for the stop frame's own reading
    far = _far()
    far.start_leg()
    far.frame(*_frame_at(0.0, 0.0))
    scene, ids, odom, instr = _frame_at(0.12, 0.2)
    obs, rec = far.stop(scene, np.full_like(ids, -1), odom, instr, pallet_depth_m=EPAL.overall_depth_m)
    assert obs.status == "no_pallet" and rec["reason"] in ("target_lost", "no_candidate_in_radius")


def test_another_track_taking_the_stop_reading_is_inconsistent_not_a_switch():
    # Codex: the selected track 0 still visible, the stop decision's candidate taken by track 1 -> no switch
    far = _far()
    far.start_leg()
    far.frame(*_frame_at(0.0, 0.0))
    assert far.tracker.selected == 0
    # the stop reading 0.5 m sideways (outside every gate) starts track 1 while track 0 is still active
    scene, ids = _render([(EPAL, rt.Placement(3.38, 0.5, 0.0, 0.0))])
    scene = replace(scene, stamp_ns=int(0.2 * S))
    instr = B.instruction_from_zone((3.74, 0.5), 0.3, (0.12, 0.0, 0.0))
    obs, rec = far.stop(scene, ids, _odom(0.12), instr, pallet_depth_m=EPAL.overall_depth_m)
    assert obs.status == "no_pallet" and rec["reason"] == "target_inconsistent"


def test_a_new_leg_starts_afresh():
    far = _far()
    far.start_leg()
    far.frame(*_frame_at(0.0, 0.0))
    assert far.tracker.selected is not None
    far.start_leg()
    assert far.tracker.selected is None and far.tracker.tracks(0, _odom()) == ()


def test_a_stop_decision_other_than_selected_is_no_pallet():
    far = _far()
    far.start_leg()
    _, _, odom, instr = _frame_at(0.0, 0.0)
    two, two_ids = _render([(EPAL, rt.Placement(3.5, -0.45, 0.0, 0.0)), (EPAL, rt.Placement(3.5, 0.45, 0.0, 0.0))])
    obs, rec = far.stop(replace(two, stamp_ns=0), two_ids, odom, instr, pallet_depth_m=EPAL.overall_depth_m)
    assert obs.status == "no_pallet" and rec["status"] in ("ask_operator", "reobserve")


# ---- the batch pixel lag ---------------------------------------------------------------------------------

def test_the_lag_is_identified_from_a_speed_change_with_a_constant_bias():
    dt = 1 / 120
    steps = np.arange(0, 600)
    speed = np.where(steps < 240, 0.3 * steps / 240, np.where(steps < 420, 0.3, 0.3 * (600 - steps) / 180))
    truth = 4.0 - np.cumsum(speed) * dt
    tau, bias = 11, 0.012
    frames = list(range(60, 560, 24))
    measured = [truth[k - tau] + bias for k in frames]
    fit = B.estimate_lag(frames, measured, truth, max_lag=48)
    assert fit["tau_steps"] == tau and fit["bias_m"] == pytest.approx(bias, abs=1e-3) and fit["accepted"]


def test_a_constant_speed_leaves_the_lag_unidentified():
    dt = 1 / 120
    truth = 4.0 - 0.3 * np.arange(0, 600) * dt
    frames = list(range(60, 560, 24))
    measured = [truth[k - 11] + 0.012 for k in frames]
    fit = B.estimate_lag(frames, measured, truth, max_lag=48)
    assert not fit["accepted"] and not fit["unique"][0]  # the bias absorbs any lag: no tau stands out


def test_a_choice_made_at_the_stop_is_in_the_selection_history():
    # Codex P3: the history was copied before the stop's own choice
    far = _far()
    far.start_leg()
    obs, rec = far.stop(*_frame_at(0.0, 0.0), pallet_depth_m=EPAL.overall_depth_m)
    assert rec["selected_track"] == 0 and [h["track"] for h in rec["selection_history"]] == [0]


def test_a_driving_frame_reports_the_loss_of_the_legs_target():
    # A' (Codex): the chosen track deleted while driving ends the leg; a stale-but-kept track does not
    far = _far()
    far.start_leg()
    far.frame(*_frame_at(0.0, 0.0))
    _, _, odom, instr = _frame_at(0.0, 0.0)
    empty, empty_ids = _render([])
    stale = far.frame(replace(empty, stamp_ns=int(0.8 * S)), empty_ids, odom, instr)
    assert not stale["leg_lost"] and far.tracker.selected == 0
    gone = far.frame(replace(empty, stamp_ns=int(1.21 * S)), empty_ids, odom, instr)
    assert gone["leg_lost"]


# stage 3 task 6: an injected proposer and the identity threshold ---------------------------------------

def test_a_learned_reading_below_tau_is_not_selected():
    from forklift_core.perception.instance_proposals import InstanceProposal

    def proposer(scene, ids):
        return [InstanceProposal(f"l{k}", ids == k, 0.3, "learned") for k in sorted(set(ids[ids >= 0].tolist()))], ()

    for tau, want in ((0.5, False), (0.2, True)):
        far = B.FarRecognition(BOUNDS, specs=SPECS, setup=ForkSetup(0.3725), load_for=_load, proposer=proposer,
                               identity_threshold=tau)
        far.start_leg()
        obs, rec = far.stop(*_frame_at(0.0, 0.0), pallet_depth_m=EPAL.overall_depth_m)
        assert (rec["status"] == "selected") is want, tau
