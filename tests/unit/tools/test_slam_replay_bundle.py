"""slam-bounded-graph plan candidate A: a map bundle from a replay's graph dump (map.posegraph, map.data, the karto
grid of tools/slam_graph_dump.cpp) that forklift_core.localization.slam_map.load_bundle accepts, its grid in the
bundle's int8 convention (-1 unknown, 0 free, 100 occupied)."""

import json

import numpy as np
import pytest

from forklift_core.localization.slam_map import load_bundle
from tools import slam_replay_bundle as rb


def _replay_dump(root):
    bridge = root / "bridge"
    (bridge / "bundle").mkdir(parents=True)
    (bridge / "dump").mkdir()
    (bridge / "bundle" / "map.posegraph").write_bytes(b"graph")
    (bridge / "bundle" / "map.data").write_bytes(b"data")
    cells = np.array([[0, 100, 255], [255, 255, 0]], np.uint8)  # karto: 0 unknown, 100 occupied, 255 free
    (bridge / "dump" / "grid.u8").write_bytes(cells.tobytes())
    (bridge / "dump" / "graph.json").write_text(json.dumps({"scans": 2, "vertices": 2, "edges": 1, "width": 3,
                                                            "height": 2, "origin": [-1.5, 2.0], "resolution_m": 0.05}))
    return bridge


def test_a_replay_bundle_loads_and_its_grid_follows_the_bundle_convention(tmp_path):
    bridge = _replay_dump(tmp_path)
    manifest = rb.make_bundle(bridge, tmp_path / "bundle", source={"run": "lf_x", "units": [0]})
    b = load_bundle(tmp_path / "bundle", manifest["map_id"])
    assert b.grid.data.tolist() == [[-1, 100, 0], [0, 0, -1]]
    assert b.grid.origin == (-1.5, 2.0, 0.0) and b.grid.resolution_m == pytest.approx(0.05)
    assert manifest["source"] == {"run": "lf_x", "units": [0]} and manifest["replay"] is True


def test_a_dump_without_its_graph_files_is_refused(tmp_path):
    bridge = _replay_dump(tmp_path)
    (bridge / "bundle" / "map.data").unlink()
    with pytest.raises(SystemExit):
        rb.make_bundle(bridge, tmp_path / "bundle", source={})
    bridge = _replay_dump(tmp_path / "b")
    (bridge / "dump" / "grid.u8").write_bytes(b"\x00")  # not width x height
    with pytest.raises(SystemExit):
        rb.make_bundle(bridge, tmp_path / "b2", source={})
