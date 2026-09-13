"""Strict pallet dimensions and derived openings, independent of simulators."""

import math
from dataclasses import dataclass, fields
from pathlib import Path

from forklift_core._validation import _finite_scalar
from forklift_core.perception.pallet_prior import PalletPrior


@dataclass(frozen=True)
class PalletGeometry:
    """Physical pallet shape. Loaded from YAML only; no dataclass defaults.

    The fork openings are open to the floor: the bottom boards run under the
    block columns, not under the openings. A continuous lower slab would put a
    surface there that a real EPAL pallet does not have.
    """

    source_provenance: str
    geometry_version: str
    overall_width_m: float
    overall_depth_m: float
    overall_height_m: float
    deck_bottom_m: float
    block_height_m: float
    stringer_m: float
    top_board_thickness_m: float
    block_widths_m: tuple[float, float, float]
    block_depth_m: float
    bottom_board_widths_m: tuple[float, float, float]
    top_board_count: int
    top_board_width_m: float

    def __post_init__(self) -> None:
        text = {"source_provenance", "geometry_version"}
        sequences = {"block_widths_m", "bottom_board_widths_m"}
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name in text:
                if not isinstance(value, str) or not value.strip():
                    raise ValueError(f"{field.name} must be nonempty text")
            elif field.name in sequences:
                if len(value) != 3:
                    raise ValueError(f"{field.name} must name three block columns")
                for index, item in enumerate(value):
                    if _finite_scalar(item, f"{field.name}[{index}]") <= 0:
                        raise ValueError(f"{field.name}[{index}] must be positive")
            elif field.name == "top_board_count":
                if type(value) is not int or value < 2:
                    raise ValueError("top_board_count must be at least two boards")
            elif _finite_scalar(value, field.name) <= 0:
                raise ValueError(f"{field.name} must be positive")
        if tuple(self.block_widths_m) != tuple(self.bottom_board_widths_m):
            raise ValueError("Bottom boards must sit under the block columns")
        if sum(self.block_widths_m) >= self.overall_width_m:
            raise ValueError("Blocks must leave two open channels across the width")
        if 3 * self.block_depth_m >= self.overall_depth_m:
            raise ValueError("Blocks must leave two open channels along the depth")
        if self.top_board_count * self.top_board_width_m > self.overall_width_m:
            raise ValueError("Top boards must fit across the width")

    @property
    def deck_top_m(self) -> float:
        """Everything above the opening: stringers plus top boards."""
        return self.stringer_m + self.top_board_thickness_m

    @property
    def opening_width_m(self) -> float:
        return (self.overall_width_m - sum(self.block_widths_m)) / 2

    @property
    def centre_block_width_m(self) -> float:
        return self.block_widths_m[1]

    @property
    def opening_centre_offset_m(self) -> float:
        return (self.centre_block_width_m + self.opening_width_m) / 2

    @property
    def opening_centre_spacing_m(self) -> float:
        return 2 * self.opening_centre_offset_m

    @property
    def opening_centre_height_m(self) -> float:
        return self.deck_bottom_m + self.block_height_m / 2

    @property
    def opening_z_band_m(self) -> tuple[float, float]:
        return self.deck_bottom_m, self.deck_bottom_m + self.block_height_m

    @property
    def top_board_pitch_m(self) -> float:
        span = self.overall_width_m - self.top_board_width_m
        return span / (self.top_board_count - 1)

    def block_centres_y_m(self) -> list[float]:
        """Column centres across the face; the outer columns touch the edges."""
        left, centre, right = self.block_widths_m
        return [
            -(self.overall_width_m - left) / 2,
            0.0,
            (self.overall_width_m - right) / 2,
        ]

    def block_centres_x_m(self) -> list[float]:
        offset = (self.overall_depth_m - self.block_depth_m) / 2
        return [-offset, 0.0, offset]

    def top_board_centres_y_m(self) -> list[float]:
        start = -(self.overall_width_m - self.top_board_width_m) / 2
        return [
            start + index * self.top_board_pitch_m
            for index in range(self.top_board_count)
        ]

    def to_pallet_prior(
        self,
        *,
        opening_width_tolerance_m: float = 0.02,
        centre_spacer_tolerance_m: float = 0.015,
    ) -> PalletPrior:
        """Derive an asymmetric-deck prior with symmetric recognition tolerances."""
        for name, tolerance, nominal in (
            ("opening_width", opening_width_tolerance_m, self.opening_width_m),
            ("centre_spacer", centre_spacer_tolerance_m, self.centre_block_width_m),
        ):
            if not 0 < _finite_scalar(tolerance, name) < nominal:
                raise ValueError(f"{name} tolerance must be positive and below nominal")
        if self.overall_width_m <= (
            2 * (self.opening_width_m + opening_width_tolerance_m)
            + self.centre_block_width_m
            + centre_spacer_tolerance_m
        ):
            raise ValueError("Tolerances must leave room for outer supports")
        return PalletPrior(
            height_m=self.overall_height_m,
            deck_bottom_m=self.deck_bottom_m,
            deck_top_m=self.deck_top_m,
            opening_height_m=self.block_height_m,
            opening_width_min_m=self.opening_width_m - opening_width_tolerance_m,
            opening_width_max_m=self.opening_width_m + opening_width_tolerance_m,
            centre_spacer_min_m=self.centre_block_width_m - centre_spacer_tolerance_m,
            centre_spacer_max_m=self.centre_block_width_m + centre_spacer_tolerance_m,
            overall_width_m=self.overall_width_m,
            overall_depth_m=self.overall_depth_m,
            source_provenance=self.source_provenance,
            catalogue_version=self.geometry_version,
        )


def load_pallet_geometry(path: Path) -> PalletGeometry:
    """Reject unknown fields and layers that do not add up to the total height."""
    import yaml

    try:
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError("Malformed pallet geometry YAML") from exc
    expected = {field.name for field in fields(PalletGeometry)}
    if not isinstance(data, dict) or set(data) != expected:
        raise ValueError("Pallet geometry must contain exactly the documented fields")
    for name in ("block_widths_m", "bottom_board_widths_m"):
        if not isinstance(data.get(name), list):
            raise ValueError(f"{name} must be a list of three widths")
        data[name] = tuple(float(v) for v in data[name])
    geometry = PalletGeometry(**data)
    layers = (
        geometry.deck_bottom_m
        + geometry.block_height_m
        + geometry.stringer_m
        + geometry.top_board_thickness_m
    )
    if not math.isclose(layers, geometry.overall_height_m, rel_tol=0, abs_tol=1e-9):
        raise ValueError("overall_height_m must equal the four stacked layers")
    return geometry


@dataclass(frozen=True)
class ForkFit:
    """Whether a given fork set can enter this pallet, with the margins."""

    lateral_ok: bool
    vertical_ok: bool
    lateral_inner_margin_m: float
    lateral_outer_margin_m: float
    lift_required_m: float
    reach_fraction: float

    @property
    def fits(self) -> bool:
        return self.lateral_ok and self.vertical_ok


def check_fork_fit(
    geometry: PalletGeometry,
    *,
    fork_spacing_m: float,
    fork_width_m: float,
    fork_thickness_m: float,
    fork_centre_height_m: float,
    fork_length_m: float,
    lift_travel_m: float,
    floor_clearance_m: float = 0.004,
) -> ForkFit:
    """Check aligned horizontal blades; lift only upwards within available travel.

    Margins are relative to the opening, not the pallet's outside envelope.
    Reach is blade length / pallet depth; this does not validate the carriage,
    fork heels, payload, contact, or approach trajectory.
    """
    for name, value in (
        ("fork_spacing_m", fork_spacing_m),
        ("fork_width_m", fork_width_m),
        ("fork_thickness_m", fork_thickness_m),
        ("fork_length_m", fork_length_m),
    ):
        if _finite_scalar(value, name) <= 0:
            raise ValueError(f"{name} must be positive")
    for name, value in (
        ("fork_centre_height_m", fork_centre_height_m),
        ("lift_travel_m", lift_travel_m),
        ("floor_clearance_m", floor_clearance_m),
    ):
        if _finite_scalar(value, name) < 0:
            raise ValueError(f"{name} must be nonnegative")
    inner = (fork_spacing_m - fork_width_m - geometry.centre_block_width_m) / 2
    outer = (
        geometry.centre_block_width_m / 2
        + geometry.opening_width_m
        - (fork_spacing_m + fork_width_m) / 2
    )
    bottom, top = geometry.opening_z_band_m
    lift = max(
        0.0,
        bottom + floor_clearance_m - (fork_centre_height_m - fork_thickness_m / 2),
    )
    vertical_ok = (
        lift <= lift_travel_m
        and fork_centre_height_m + lift + fork_thickness_m / 2 < top
    )
    return ForkFit(
        lateral_ok=inner > 0 and outer > 0,
        vertical_ok=vertical_ok,
        lateral_inner_margin_m=inner,
        lateral_outer_margin_m=outer,
        lift_required_m=lift,
        reach_fraction=fork_length_m / geometry.overall_depth_m,
    )
