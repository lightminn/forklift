"""A measured roof may take over only after agreeing with current pockets."""

import importlib.util
import sys
from dataclasses import replace
from pathlib import Path

import pytest

from forklift_core.perception.pocket_observation import Pocket, PocketObservation

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "roof_handoff_test", ROOT / "sim/isaac/pocket_tracking_handoff.py"
)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


def obs(stamp, x=1.32, *, lost=False, provenance="synthetic"):
    value = PocketObservation(
        stamp,
        "synthetic",
        "base_link",
        provenance,
        "valid",
        Pocket((x, 0.18625, 0.061), 0.2275, 0.078),
        Pocket((x, -0.18625, 0.061), 0.2275, 0.078),
        0.0,
        0.01,
        0.001,
        None,
    )
    return (
        replace(
            value,
            status="invalid",
            left=None,
            right=None,
            insertion_yaw_rad=None,
            position_sigma_m=None,
            yaw_sigma_rad=None,
            reason="depth_loss",
        )
        if lost
        else value
    )


def test_three_current_agreements_are_required_before_roof_drives():
    handoff = module.RoofHandoff()
    for stamp in [100, 200]:
        result = handoff.select(obs(stamp), obs(stamp, x=1.33))
        assert result.mode == "front_pockets"
        assert result.observation.left.center_m[0] == 1.32
    result = handoff.select(obs(300), obs(300, x=1.33))
    assert result.mode == "roof_model"
    assert result.observation.left.center_m[0] == 1.33
    assert result.confirmed_frames == 3


def test_loss_after_handoff_never_falls_back_to_front_or_cached_roof():
    handoff = module.RoofHandoff()
    for stamp in [100, 200, 300]:
        handoff.select(obs(stamp), obs(stamp))
    result = handoff.select(obs(400), obs(400, lost=True))
    assert result.mode == "roof_model"
    assert result.observation.status == "invalid"


@pytest.mark.parametrize(
    "bad", ["duplicate", "wrong_stamp", "mismatch", "loss", "ground_truth", "far"]
)
def test_broken_agreement_resets_confirmation(bad):
    handoff = module.RoofHandoff()
    handoff.select(obs(100), obs(100))
    front, roof = obs(200), obs(200)
    if bad == "duplicate":
        front, roof = obs(100), obs(100)
    if bad == "wrong_stamp":
        roof = obs(199)
    if bad == "mismatch":
        roof = obs(200, x=1.36)
    if bad == "loss":
        roof = obs(200, lost=True)
    if bad == "ground_truth":
        roof = obs(200, provenance="synthetic_ground_truth")
    if bad == "far":
        front, roof = obs(200, x=1.6), obs(200, x=1.6)
    result = handoff.select(front, roof)
    assert result.confirmed_frames == 0
    assert result.mode == "front_pockets"
    assert handoff.select(obs(300), obs(300)).confirmed_frames == 1


def test_unqualified_roof_cannot_rescue_lost_front():
    result = module.RoofHandoff().select(obs(100, lost=True), obs(100))
    assert result.observation.status == "invalid"
    assert result.mode == "front_pockets"


@pytest.mark.parametrize("change", ["symmetric_centers", "width", "height"])
def test_matching_midpoint_does_not_hide_pocket_shape_disagreement(change):
    handoff = module.RoofHandoff()
    for stamp in [100, 200, 300]:
        front, roof = obs(stamp), obs(stamp)
        if change == "symmetric_centers":
            front = replace(
                front,
                left=replace(front.left, center_m=(1.32, 0.145, 0.061)),
                right=replace(front.right, center_m=(1.32, -0.145, 0.061)),
            )
        elif change == "width":
            front = replace(front, left=replace(front.left, width_m=0.15))
        else:
            front = replace(front, right=replace(front.right, height_m=0.11))
        result = handoff.select(front, roof)
    assert result.mode == "front_pockets"
    assert result.confirmed_frames == 0
