# src/forklift_core/perception/pallet_model.py
"""Generalised box-model pallet: N block/stringer columns, any entry axes.

Pallet frame: x along the depth (forks travel +x through an x-entry face at
x = -depth/2), y across the width, z up from the support surface. Bottom boards
run under the columns only, as in ``pallet_boxes``, so x-face fork lanes are
open to the support surface.
"""

from dataclasses import asdict, dataclass, fields

from forklift_core._validation import _finite_scalar
from forklift_core.perception.pallet_geometry import PalletBox, PalletGeometry

KNOWN_ENTRY_AXES = {"epal6": ("x",), "t11_06": ("x", "y")}
TOP_STYLES = ("boards", "flat")
_EPS = 1e-9


def _intervals(centres, widths):
    return tuple((c - w / 2, c + w / 2) for c, w in zip(centres, widths, strict=True))


def _check_members(centres, widths, span, label):
    if len(centres) != len(widths) or len(centres) < 3:
        raise ValueError(f"{label} needs at least three members with widths")
    for index, centre in enumerate(centres):
        _finite_scalar(centre, f"{label} centre[{index}]")
    for index, width in enumerate(widths):
        if _finite_scalar(width, f"{label} width[{index}]") <= 0:
            raise ValueError(f"{label} widths must be positive")
    intervals = _intervals(centres, widths)
    if any(b[0] < a[1] - _EPS for a, b in zip(intervals, intervals[1:])):
        raise ValueError(f"{label} members overlap or are out of order")
    if abs(intervals[0][0] + span / 2) > 1e-6 or abs(intervals[-1][1] - span / 2) > 1e-6:
        raise ValueError(f"{label} outer members must touch the edges")


@dataclass(frozen=True)
class PalletSpec:
    name: str
    known: bool
    overall_width_m: float
    overall_depth_m: float
    deck_bottom_m: float
    opening_height_m: float
    stringer_m: float
    top_thickness_m: float
    column_centres_y_m: tuple
    column_widths_m: tuple
    row_centres_x_m: tuple
    row_depth_m: float
    top_style: str
    top_board_count: int
    top_board_width_m: float
    entry_axes: tuple

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("name must be nonempty text")
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name.endswith("_m") and not isinstance(value, (tuple, list)):
                if _finite_scalar(value, field.name) <= 0:
                    raise ValueError(f"{field.name} must be positive")
        for name in ("column_centres_y_m", "column_widths_m", "row_centres_x_m", "entry_axes"):
            value = tuple(getattr(self, name))
            object.__setattr__(self, name, value if name == "entry_axes" else tuple(map(float, value)))
        _check_members(self.column_centres_y_m, self.column_widths_m, self.overall_width_m, "columns")
        _check_members(self.row_centres_x_m, (self.row_depth_m,) * len(self.row_centres_x_m),
                       self.overall_depth_m, "rows")
        if self.top_style not in TOP_STYLES:
            raise ValueError(f"top_style must be one of {TOP_STYLES}")
        if self.top_style == "boards":
            if type(self.top_board_count) is not int or self.top_board_count < 2:
                raise ValueError("top_board_count must be at least two boards")
            if self.top_board_count * self.top_board_width_m > self.overall_width_m + _EPS:
                raise ValueError("Top boards must fit across the width")
        if not self.entry_axes or set(self.entry_axes) - {"x", "y"}:
            raise ValueError("entry_axes must name x and/or y")

    @property
    def overall_height_m(self) -> float:
        return self.deck_bottom_m + self.opening_height_m + self.stringer_m + self.top_thickness_m

    def to_json(self) -> dict:
        obj = asdict(self)
        for name in ("column_centres_y_m", "column_widths_m", "row_centres_x_m", "entry_axes"):
            obj[name] = list(obj[name])
        return obj


def spec_from_geometry(geometry: PalletGeometry, *, entry_axes) -> PalletSpec:
    g = geometry
    return PalletSpec(
        name=g.geometry_version, known=True,
        overall_width_m=g.overall_width_m, overall_depth_m=g.overall_depth_m,
        deck_bottom_m=g.deck_bottom_m, opening_height_m=g.block_height_m,
        stringer_m=g.stringer_m, top_thickness_m=g.top_board_thickness_m,
        column_centres_y_m=tuple(g.block_centres_y_m()), column_widths_m=tuple(g.block_widths_m),
        row_centres_x_m=tuple(g.block_centres_x_m()), row_depth_m=g.block_depth_m,
        top_style="boards", top_board_count=g.top_board_count, top_board_width_m=g.top_board_width_m,
        entry_axes=tuple(entry_axes),
    )


def _top_board_centres_y(spec: PalletSpec) -> list[float]:
    pitch = (spec.overall_width_m - spec.top_board_width_m) / (spec.top_board_count - 1)
    start = -(spec.overall_width_m - spec.top_board_width_m) / 2
    return [start + i * pitch for i in range(spec.top_board_count)]


def spec_boxes(spec: PalletSpec) -> list[PalletBox]:
    """Same names and order as ``pallet_boxes`` for three-column known shapes."""
    s = spec
    boxes: list[PalletBox] = []
    columns = list(zip(s.column_centres_y_m, s.column_widths_m, strict=True))
    for index, (y, width) in enumerate(columns):
        boxes.append(PalletBox(f"bottom_board_{index}", (s.overall_depth_m, width, s.deck_bottom_m),
                               (0.0, y, s.deck_bottom_m / 2)))
    block_z = s.deck_bottom_m + s.opening_height_m / 2
    for xi, x in enumerate(s.row_centres_x_m):
        for yi, (y, width) in enumerate(columns):
            boxes.append(PalletBox(f"block_x{xi}_y{yi}", (s.row_depth_m, width, s.opening_height_m), (x, y, block_z)))
    stringer_z = s.deck_bottom_m + s.opening_height_m + s.stringer_m / 2
    for xi, x in enumerate(s.row_centres_x_m):
        boxes.append(PalletBox(f"stringer_{xi}", (s.row_depth_m, s.overall_width_m, s.stringer_m), (x, 0.0, stringer_z)))
    top_z = s.overall_height_m - s.top_thickness_m / 2
    if s.top_style == "flat":
        boxes.append(PalletBox("top_deck", (s.overall_depth_m, s.overall_width_m, s.top_thickness_m), (0.0, 0.0, top_z)))
    else:
        for index, y in enumerate(_top_board_centres_y(s)):
            boxes.append(PalletBox(f"top_board_{index}", (s.overall_depth_m, s.top_board_width_m, s.top_thickness_m),
                                   (0.0, y, top_z)))
    return boxes


@dataclass(frozen=True)
class FaceLayout:
    axis: str
    width_m: float
    depth_m: float
    member_intervals_m: tuple
    opening_intervals_m: tuple
    pocket_band_m: tuple
    lane_floor_m: float
    lane_ceiling_m: float


def face_layout(spec: PalletSpec, axis: str) -> FaceLayout:
    """Entering an x face (+x travel), left is +y. Entering a y face (+y travel),
    left is -x. The opposite face is reached by turning the pallet yaw by pi."""
    if axis not in spec.entry_axes:
        raise ValueError(f"{axis!r} is not an entry axis of {spec.name}")
    if axis == "x":
        members = _intervals(spec.column_centres_y_m, spec.column_widths_m)
        width, depth = spec.overall_width_m, spec.overall_depth_m
        band = (spec.deck_bottom_m, spec.deck_bottom_m + spec.opening_height_m)
        floor, ceiling = 0.0, band[1]
    else:
        rows = _intervals(spec.row_centres_x_m, (spec.row_depth_m,) * len(spec.row_centres_x_m))
        members = tuple(sorted((-hi, -lo) for lo, hi in rows))
        width, depth = spec.overall_depth_m, spec.overall_width_m
        band = (spec.deck_bottom_m, spec.deck_bottom_m + spec.opening_height_m + spec.stringer_m)
        floor, ceiling = band
    openings = tuple((a[1], b[0]) for a, b in zip(members, members[1:]))
    return FaceLayout(axis, width, depth, members, openings, band, floor, ceiling)


def pair_margin(layout: FaceLayout, pair, spacing_m: float, blade_width_m: float) -> tuple[float, float]:
    (l0, h0), (l1, h1) = (layout.opening_intervals_m[i] for i in pair)
    centre = ((l0 + h0) / 2 + (l1 + h1) / 2) / 2
    half, w = spacing_m / 2, blade_width_m / 2
    blades = ((centre - half - w, centre - half + w), (centre + half - w, centre + half + w))
    margins = (blades[0][0] - l0, h0 - blades[0][1], blades[1][0] - l1, h1 - blades[1][1])
    return min(margins), centre


def choose_pocket_pair(layout: FaceLayout, spacing_m: float, blade_width_m: float) -> tuple[int, int]:
    n = len(layout.opening_intervals_m)
    if n < 2:
        raise ValueError("A face needs two openings")
    best = None
    for i in range(n):
        for j in range(i + 1, n):
            margin, centre = pair_margin(layout, (i, j), spacing_m, blade_width_m)
            key = (-round(margin, 9), round(abs(centre), 9), i, j)
            if best is None or key < best[0]:
                best = (key, (i, j))
    return best[1]
