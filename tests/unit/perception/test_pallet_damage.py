# tests/unit/perception/test_pallet_damage.py
from pathlib import Path

import pytest

from forklift_core.perception.pallet_damage import DamageOp, apply_damage, removed_volume_fraction
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_model import spec_boxes, spec_from_geometry

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def epal_boxes():
    g = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
    return spec_boxes(spec_from_geometry(g, entry_axes=("x",)))


def _by_name(boxes):
    return {b.name: b for b in boxes}


def test_missing_board_removes_exactly_that_board(epal_boxes):
    out = apply_damage(epal_boxes, [DamageOp("top_board_missing", "top_board_3", ())])
    assert "top_board_3" not in _by_name(out) and len(out) == len(epal_boxes) - 1


@pytest.mark.parametrize(("keep", "length", "front_x"), [(0.0, 0.24, -0.30), (1.0, 0.36, -0.06)])
def test_broken_board_cuts_at_the_fraction_from_the_front(epal_boxes, keep, length, front_x):
    op = DamageOp("top_board_broken", "top_board_0", (("fraction", 0.4), ("keep", keep)))
    piece = _by_name(apply_damage(epal_boxes, [op]))["top_board_0"]
    assert piece.size_m[0] == pytest.approx(length)
    assert piece.centre_m[0] - piece.size_m[0] / 2 == pytest.approx(front_x)


def test_chipped_corner_leaves_a_notch_on_the_entry_face(epal_boxes):
    op = DamageOp("block_corner_chipped", "block_x0_y0", (("depth", 0.03), ("width", 0.02), ("side", 1.0)))
    out = apply_damage(epal_boxes, [op])
    parts = [b for b in out if b.name.startswith("block_x0_y0")]
    front = min(parts, key=lambda b: b.centre_m[0])
    assert len(parts) == 2 and front.size_m[1] == pytest.approx(0.100 - 0.02)
    assert 0 < removed_volume_fraction(epal_boxes, out) < 0.01


def test_split_block_leaves_a_gap(epal_boxes):
    parts = [b for b in apply_damage(epal_boxes, [DamageOp("block_split", "block_x0_y1", (("gap", 0.01),))])
             if b.name.startswith("block_x0_y1")]
    assert len(parts) == 2 and sum(p.size_m[1] for p in parts) == pytest.approx(0.145 - 0.01)


@pytest.mark.parametrize(
    ("kind", "target", "params"),
    [
        ("top_board_broken", "top_board_0", (("fraction", 0.9), ("keep", 0.0))),
        ("block_corner_chipped", "block_x0_y0", (("depth", 0.08), ("width", 0.08), ("side", 0.0))),
        ("block_split", "block_x0_y0", (("gap", 0.05),)),
        ("top_board_missing", "block_x0_y0", ()),
    ],
)
def test_out_of_range_or_wrong_target_is_rejected(kind, target, params):
    with pytest.raises(ValueError):
        DamageOp(kind, target, params)


def test_damaging_a_missing_box_is_rejected(epal_boxes):
    once = apply_damage(epal_boxes, [DamageOp("top_board_missing", "top_board_1", ())])
    with pytest.raises(ValueError, match="No box"):
        apply_damage(once, [DamageOp("top_board_missing", "top_board_1", ())])
