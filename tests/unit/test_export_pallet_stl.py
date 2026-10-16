# tests/unit/test_export_pallet_stl.py
import json
import struct
from collections import Counter

import numpy as np
import pytest

from forklift_core.perception.pallet_model import PalletBox
from tools import export_pallet_stl as ex
from tools import recognition_scenes as rs


def _volume(tri):
    return float(np.sum(np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2]))) / 6.0)


def _edges_even(tri):
    count = Counter()
    for t in tri:
        for a, b in ((0, 1), (1, 2), (2, 0)):
            count[tuple(sorted((tuple(np.round(t[a], 9)), tuple(np.round(t[b], 9)))))] += 1
    return all(n % 2 == 0 for n in count.values())


def test_a_box_becomes_a_closed_outward_mesh():
    tri = ex.union_mesh([PalletBox("b", (0.2, 0.1, 0.05), (0.0, 0.0, 0.025))])
    assert len(tri) == 12 and _volume(tri) == pytest.approx(0.2 * 0.1 * 0.05) and _edges_even(tri)


def test_touching_boxes_merge_without_inner_faces():
    boxes = [PalletBox("a", (0.1, 0.1, 0.1), (0.0, 0.0, 0.05)), PalletBox("b", (0.1, 0.1, 0.1), (0.1, 0.0, 0.05))]
    tri = ex.union_mesh(boxes)
    assert _volume(tri) == pytest.approx(0.002) and _edges_even(tri)
    assert not any(np.allclose(tr[:, 0], 0.05) for tr in tri)  # no face on the shared plane x = 0.05


@pytest.mark.parametrize("shape", ["epal6", "t11_06"])
def test_a_known_pallet_keeps_its_size_and_volume(shape):
    spec = rs.known_spec(shape)
    boxes = ex.pallet_boxes(spec, ())
    tri = ex.union_mesh(boxes)
    lo, hi = tri.reshape(-1, 3).min(axis=0), tri.reshape(-1, 3).max(axis=0)
    assert hi - lo == pytest.approx((spec.overall_depth_m, spec.overall_width_m, spec.overall_height_m))
    assert lo[2] == pytest.approx(0.0) and _volume(tri) == pytest.approx(ex.union_volume(boxes), rel=1e-9)
    assert _edges_even(tri)


def test_tiles_fit_the_bed_and_keep_the_volume():
    boxes = ex.pallet_boxes(rs.known_spec("epal6"), ())
    tiles = ex.tile_meshes(boxes, bed_m=(0.25, 0.25, 0.25))
    assert len(tiles) == 12  # 600 x 800 mm -> 3 x 4 tiles of 200 x 200 mm
    for tile in tiles:
        size = tile.triangles.reshape(-1, 3).max(axis=0) - tile.triangles.reshape(-1, 3).min(axis=0)
        assert np.all(size <= np.array([0.25, 0.25, 0.25]) + 1e-9) and _edges_even(tile.triangles)
    assert sum(_volume(t.triangles) for t in tiles) == pytest.approx(ex.union_volume(boxes), rel=1e-9)


def test_a_pallet_too_tall_for_the_bed_is_refused():
    with pytest.raises(ValueError, match="height"):
        ex.tile_meshes(ex.pallet_boxes(rs.known_spec("epal6"), ()), bed_m=(0.25, 0.25, 0.1))


def test_damage_removes_material():
    recipe = next(r for r in rs.catalogue() if r.case == "S5" and r.instances[0].damage)
    inst = recipe.instances[0]
    intact = ex.union_volume(ex.pallet_boxes(inst.spec, ()))
    damaged = ex.union_volume(ex.pallet_boxes(inst.spec, inst.damage))
    assert damaged < intact


def test_the_cli_writes_binary_stl_parts_and_a_manifest(tmp_path):
    ex.main(["--shape", "t11_06", "--scale", "0.5", "--bed", "200,200,200", "--out", str(tmp_path / "t11")])
    manifest = json.loads((tmp_path / "t11" / "manifest.json").read_text())
    assert manifest["scale"] == 0.5 and manifest["overall_mm"] == pytest.approx([330.0, 330.0, 45.0])
    assert len(manifest["parts"]) == 4  # 330 x 330 mm on a 200 mm bed -> 2 x 2
    part = tmp_path / "t11" / manifest["parts"][0]["file"]
    data = part.read_bytes()
    (count,) = struct.unpack("<I", data[80:84])
    assert len(data) == 84 + 50 * count and count == manifest["parts"][0]["triangles"]
    assert sum(p["volume_cm3"] for p in manifest["parts"]) == pytest.approx(manifest["volume_cm3"], rel=1e-6)


def test_members_mode_writes_one_part_per_board_or_block(tmp_path):
    ex.main(["--shape", "epal6", "--split", "members", "--out", str(tmp_path / "m")])
    manifest = json.loads((tmp_path / "m" / "manifest.json").read_text())
    names = [b.name for b in ex.pallet_boxes(rs.known_spec("epal6"), ())]
    parts = manifest["parts"]
    assert {p["name"].split("_tile")[0] for p in parts} == set(names) and len(parts) > len(names)
    assert all(max(p["size_mm"][:2]) <= 256 + 1e-6 for p in parts)  # 600 and 800 mm members are cut
    assert sum(p["volume_cm3"] for p in parts) == pytest.approx(
        sum(np.prod(b.size_m) for b in ex.pallet_boxes(rs.known_spec("epal6"), ())) * 1e6, rel=1e-9)
    assert {"name", "file", "size_mm", "position_mm"} <= set(parts[0])


def test_a_filament_test_plate(tmp_path):
    ex.main(["--plate", "200,200,5", "--out", str(tmp_path / "plate")])
    manifest = json.loads((tmp_path / "plate" / "manifest.json").read_text())
    assert manifest["parts"][0]["size_mm"] == [200.0, 200.0, 5.0]
