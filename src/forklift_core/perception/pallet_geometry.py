"""Strict pallet dimensions and derived openings, independent of simulators."""

import math
from dataclasses import dataclass, fields
from pathlib import Path

from forklift_core._validation import _finite_scalar
from forklift_core.perception.pallet_prior import PalletPrior


@dataclass(frozen=True)
class PalletGeometry:
    """Physical pallet shape. Loaded from YAML only; no dataclass defaults."""

    source_provenance: str
    geometry_version: str
    overall_width_m: float
    overall_depth_m: float
    overall_height_m: float
    block_width_m: float
    block_depth_m: float
    block_height_m: float
    deck_bottom_m: float
    block_count_across: int
    block_count_deep: int

    def __post_init__(self) -> None:
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name in {"source_provenance", "geometry_version"}:
                if not isinstance(value, str) or not value.strip():
                    raise ValueError(f"{field.name} must be nonempty text")
            elif field.name.startswith("block_count_"):
                if type(value) is not int or value != 3:
                    raise ValueError(f"{field.name} must be 3 for two fork openings")
            elif _finite_scalar(value, field.name) <= 0:
                raise ValueError(f"{field.name} must be positive")
        if self.deck_top_m <= 0:
            raise ValueError("deck_top_m must be positive")
        if 3 * self.block_width_m >= self.overall_width_m:
            raise ValueError("Blocks must leave two open channels across the width")
        if 3 * self.block_depth_m >= self.overall_depth_m:
            raise ValueError("Blocks must leave two open channels along the depth")

    @property
    def deck_top_m(self) -> float:
        return self.overall_height_m - self.deck_bottom_m - self.block_height_m

    @property
    def opening_width_m(self) -> float:
        return (self.overall_width_m - 3 * self.block_width_m) / 2

    @property
    def opening_centre_offset_m(self) -> float:
        return (self.block_width_m + self.opening_width_m) / 2

    @property
    def opening_centre_spacing_m(self) -> float:
        return 2 * self.opening_centre_offset_m

    @property
    def opening_centre_height_m(self) -> float:
        return self.deck_bottom_m + self.block_height_m / 2

    @property
    def opening_z_band_m(self) -> tuple[float, float]:
        return self.deck_bottom_m, self.deck_bottom_m + self.block_height_m

    def block_centres_y_m(self) -> list[float]:
        offset = (self.overall_width_m - self.block_width_m) / 2
        return [-offset, 0.0, offset]

    def block_centres_x_m(self) -> list[float]:
        offset = (self.overall_depth_m - self.block_depth_m) / 2
        return [-offset, 0.0, offset]

    def to_pallet_prior(
        self,
        *,
        opening_width_tolerance_m: float = 0.02,
        centre_spacer_tolerance_m: float = 0.015,
    ) -> PalletPrior:
        """Derive an asymmetric-deck prior with symmetric recognition tolerances."""
        for name, tolerance, nominal in (
            ("opening_width", opening_width_tolerance_m, self.opening_width_m),
            ("centre_spacer", centre_spacer_tolerance_m, self.block_width_m),
        ):
            if not 0 < _finite_scalar(tolerance, name) < nominal:
                raise ValueError(f"{name} tolerance must be positive and below nominal")
        if self.overall_width_m <= (
            2 * (self.opening_width_m + opening_width_tolerance_m)
            + self.block_width_m
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
            centre_spacer_min_m=self.block_width_m - centre_spacer_tolerance_m,
            centre_spacer_max_m=self.block_width_m + centre_spacer_tolerance_m,
            overall_width_m=self.overall_width_m,
            overall_depth_m=self.overall_depth_m,
            source_provenance=self.source_provenance,
            catalogue_version=self.geometry_version,
        )


def load_pallet_geometry(path: Path) -> PalletGeometry:
    """Reject unknown fields and inconsistent component/total dimensions."""
    import yaml

    try:
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError("Malformed pallet geometry YAML") from exc
    expected = {field.name for field in fields(PalletGeometry)} | {"deck_top_m"}
    if not isinstance(data, dict) or set(data) != expected:
        raise ValueError("Pallet geometry must contain exactly the documented fields")
    top = _finite_scalar(data.pop("deck_top_m"), "deck_top_m")
    geometry = PalletGeometry(**data)
    if top <= 0 or not math.isclose(top, geometry.deck_top_m, rel_tol=0, abs_tol=1e-9):
        raise ValueError("overall_height_m must equal both decks plus block_height_m")
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
    inner = (fork_spacing_m - fork_width_m - geometry.block_width_m) / 2
    outer = (
        geometry.block_width_m / 2
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
