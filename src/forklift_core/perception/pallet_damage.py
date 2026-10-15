# src/forklift_core/perception/pallet_damage.py
"""Damage operators on a pallet box list (design §5.2). Render, physics and mass
all come from the same output list, so a damaged pallet is one object."""

from collections.abc import Sequence
from dataclasses import dataclass

from forklift_core.perception.pallet_geometry import PalletBox

DAMAGE_KINDS = ("top_board_missing", "top_board_broken", "block_corner_chipped", "block_split")
_RANGES = {
    ("top_board_broken", "fraction"): (0.2, 0.6),
    ("block_corner_chipped", "depth"): (0.010, 0.040),
    ("block_corner_chipped", "width"): (0.010, 0.040),
    ("block_split", "gap"): (0.005, 0.015),
}


@dataclass(frozen=True)
class DamageOp:
    kind: str
    target: str
    params: tuple

    def __post_init__(self) -> None:
        if self.kind not in DAMAGE_KINDS:
            raise ValueError(f"Unknown damage kind {self.kind!r}")
        prefix = "top_board_" if self.kind.startswith("top_board") else "block_"
        if not self.target.startswith(prefix):
            raise ValueError(f"{self.kind} must target a {prefix}* box")
        object.__setattr__(self, "params", tuple((str(k), float(v)) for k, v in self.params))
        for name, value in self.params:
            bounds = _RANGES.get((self.kind, name))
            if bounds and not bounds[0] <= value <= bounds[1]:
                raise ValueError(f"{self.kind}.{name}={value} outside {bounds}")

    def get(self, name: str) -> float:
        for key, value in self.params:
            if key == name:
                return value
        raise ValueError(f"{self.kind} needs parameter {name!r}")

    def to_json(self) -> dict:
        return {"kind": self.kind, "target": self.target, "params": [list(p) for p in self.params]}


def _volume(box: PalletBox) -> float:
    x, y, z = box.size_m
    return x * y * z


def _apply_one(box: PalletBox, op: DamageOp) -> list[PalletBox]:
    (sx, sy, sz), (cx, cy, cz) = box.size_m, box.centre_m
    front = cx - sx / 2
    if op.kind == "top_board_missing":
        return []
    if op.kind == "top_board_broken":
        cut = front + sx * op.get("fraction")
        if op.get("keep") == 0.0:
            return [PalletBox(box.name, (cut - front, sy, sz), ((front + cut) / 2, cy, cz))]
        back = cx + sx / 2
        return [PalletBox(box.name, (back - cut, sy, sz), ((cut + back) / 2, cy, cz))]
    if op.kind == "block_corner_chipped":
        depth, width = op.get("depth"), op.get("width")
        if depth * width * sz > 0.3 * _volume(box):
            raise ValueError("A chipped corner may remove at most 30 % of the block")
        sign = 1.0 if op.get("side") == 1.0 else -1.0
        rear = PalletBox(f"{box.name}_rear", (sx - depth, sy, sz), (front + depth + (sx - depth) / 2, cy, cz))
        notch = PalletBox(f"{box.name}_front", (depth, sy - width, sz), (front + depth / 2, cy - sign * width / 2, cz))
        return [rear, notch]
    gap = op.get("gap")
    half = (sy - gap) / 2
    return [
        PalletBox(f"{box.name}_a", (sx, half, sz), (cx, cy - (gap + half) / 2, cz)),
        PalletBox(f"{box.name}_b", (sx, half, sz), (cx, cy + (gap + half) / 2, cz)),
    ]


def apply_damage(boxes: Sequence[PalletBox], ops: Sequence[DamageOp]) -> list[PalletBox]:
    out = list(boxes)
    for op in ops:
        names = [b.name for b in out]
        if op.target not in names:
            raise ValueError(f"No box named {op.target!r} (already damaged?)")
        index = names.index(op.target)
        out[index : index + 1] = _apply_one(out[index], op)
    return out


def removed_volume_fraction(before: Sequence[PalletBox], after: Sequence[PalletBox]) -> float:
    total = sum(_volume(b) for b in before)
    return (total - sum(_volume(b) for b in after)) / total
