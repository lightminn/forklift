"""SDK-free replay of a front plane biased by nearby noncoplanar returns."""

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from forklift_core.geometry import RigidTransform
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from forklift_core.perception.rgbd_snapshot import scene_input_from_rgbd_snapshot
from forklift_core.sensors.rgbd import PinholeIntrinsics

ROOT = Path(__file__).resolve().parents[2]
FIXTURE = ROOT / "tests/fixtures/isaac_epal6_near"


@pytest.mark.parametrize("median_plane_offset", [False, True])
def test_near_coplanar_evidence_recovery_requires_explicit_opt_in(median_plane_offset):
    metadata = json.loads((FIXTURE / "metadata.json").read_text())
    provenance = json.loads((FIXTURE / "provenance.json").read_text())
    with np.load(FIXTURE / "depth_m.npz", allow_pickle=False) as archive:
        depth = archive["depth_m"].astype(np.float64)
    assert (
        hashlib.sha256(depth.tobytes(order="C")).hexdigest()
        == provenance["original_float64_depth_sha256"]
    )
    scene = scene_input_from_rgbd_snapshot(
        rgb=np.zeros((*depth.shape, 3), dtype=np.uint8),
        depth_m=depth,
        intrinsics=PinholeIntrinsics(**metadata["intrinsics"]),
        base_from_optical=RigidTransform(**metadata["base_from_optical"]),
        pixel_frame=metadata["intrinsics"]["frame_id"],
        **{
            key: metadata[key]
            for key in (
                "stamp_ns",
                "clock_domain",
                "source_provenance",
                "rectified",
                "rgb_registered_to_depth_grid",
                "depth_kind",
                "depth_unit",
            )
        },
    )
    prior = load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")
    params = DetectorParams.derived_for(
        prior, max_plane_candidates=12, median_plane_offset=median_plane_offset
    )
    result = detect_pockets(scene, prior, params)

    if not median_plane_offset:
        assert result.observation.status == "no_pallet"
        assert result.observation.reason == "no_opening_pattern"
        return
    assert result.observation.status == "valid", result.observation.reason
    assert result.observation.left is not None
    assert result.observation.right is not None
    assert result.diagnostics.selected_lower >= params.min_band_points
