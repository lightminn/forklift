"""Portable planner CLI must consume and retain explicit pallet geometry."""

import hashlib
import json
import subprocess
import sys
from dataclasses import asdict
from pathlib import Path

import pytest

from forklift_core.planning import PlannerConfig

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools/benchmark_transport_planning.py"


@pytest.mark.parametrize(
    "shape, depth, width, offset",
    [
        ("epal6", 0.60, 0.80, 1.23),
        ("t11_06", 0.66, 0.66, 1.26),
    ],
)
def test_benchmark_uses_and_snapshots_pallet_yaml(
    tmp_path, shape, depth, width, offset
):
    assets = tmp_path / "assets.json"
    assets.write_text(
        json.dumps(
            [
                {"filename": name, "min": [0, 0, 0], "max": [0.3, 0.3, 0.5]}
                for name in [
                    "SM_BarelPlastic_A_01.usd",
                    "SM_CratePlastic_D_01.usd",
                    "SM_CardBoxA_02.usd",
                ]
            ]
        )
    )
    source = ROOT / f"config/pallet_geometry_{shape}.yaml"
    output = tmp_path / "report"
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--assets-json",
            str(assets),
            "--pallet-geometry",
            str(source),
            "--output",
            str(output),
            "--count",
            "1",
            "--obstacles",
            "0",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    report = json.loads((output / "results.json").read_text())
    assert report["planner_config"] == asdict(
        PlannerConfig(primitive_length_m=0.25, clearance_m=0.10, max_expansions=30000)
    )
    assert report["approach_clearance_m"] == 0.05
    geometry = report["mission_geometry"]
    assert geometry["pallet_depth_m"] == depth
    assert geometry["pallet_width_m"] == width
    assert geometry["inserted_offset_m"] == pytest.approx(offset)
    assert geometry["axle_to_fork_tip_m"] == 1.29
    assert report["pallet_geometry_evidence"] == {
        "input_path": str(source),
        "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "copied_to": "pallet_geometry.yaml",
    }
    assert (output / "pallet_geometry.yaml").read_bytes() == source.read_bytes()
    assert report["benchmark_complete"]
    assert report["results"][0]["success"], report["results"]


def test_benchmark_requires_explicit_pallet_geometry(tmp_path):
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--assets-json",
            "unused.json",
            "--output",
            str(tmp_path / "report"),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 2
    assert "--pallet-geometry" in result.stderr
