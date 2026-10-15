"""The setup snapshot of a format-2 record (docs/plans/2026-10-16-m-a1-replay.md, R3): drive's inputs by kind."""

import importlib.util
import json
import sys
from argparse import Namespace
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim/isaac"))
import replay_setup as RS  # noqa: E402


def test_drive_inputs_are_its_prologue_names():
    spec = importlib.util.spec_from_file_location("run_transport_inputs", ROOT / "sim/isaac/run_transport.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    inputs = RS.drive_inputs(module.drive)
    assert {"args", "state", "obstacle", "robot", "geometry", "plan_transport"} <= set(inputs)


def test_the_snapshot_writes_each_name_by_kind_and_copies_the_files(tmp_path):
    robot, layer = object(), object()
    data = tmp_path / "geometry.yaml"
    data.write_text("depth: 1.2\n")
    names = {
        "robot": robot, "obstacle": {"layer": layer, "version": 3}, "plan_transport": lambda *a: None,
        "isaac": SimpleNamespace(action=lambda **k: k), "args": Namespace(seed=1, pallet_geometry=data),
        "lift_index": np.array([5]), "q": object(), "json": json, "unused": 1,
    }
    audit = RS.write_setup_snapshot(names, tmp_path / "boundary", [str(data)], services={"Obstacles": layer},
                                    wanted={"robot", "obstacle", "plan_transport", "isaac", "args", "lift_index", "q",
                                            "json"})
    snap = json.loads((tmp_path / "boundary/setup_snapshot.json").read_text())
    kinds = {k: v["kind"] for k, v in snap["names"].items()}
    assert kinds == {"robot": "service", "obstacle": "value", "plan_transport": "port", "isaac": "adapters",
                     "args": "value", "lift_index": "value", "q": "excluded", "json": "import"}
    assert snap["names"]["obstacle"]["node"]["$map"]["layer"] == {"$service": "Obstacles"}
    assert snap["names"]["json"]["path"] == "json" and audit["opaque"] == []
    (copied,) = snap["files"]
    assert (tmp_path / "boundary/setup_files" / copied["sha256"]).read_text() == "depth: 1.2\n"
