"""Load explicitly supplied pallet dimensions; no synthetic shape defaults."""

import math
from dataclasses import dataclass
from pathlib import Path

from forklift_core._validation import _finite_scalar
from forklift_core.perception.pocket_observation import PROVENANCES


@dataclass(frozen=True)
class PalletPrior:
    """Shape prior in metres, with provenance independent of sensor provenance."""

    height_m: float
    deck_bottom_m: float
    deck_top_m: float
    opening_height_m: float
    opening_width_min_m: float
    opening_width_max_m: float
    centre_spacer_min_m: float
    centre_spacer_max_m: float
    overall_width_m: float
    overall_depth_m: float
    source_provenance: str
    catalogue_version: str

    @property
    def opening_centre_height_m(self) -> float:
        """Prior-derived centre height, not a measured vertical position."""
        return self.deck_bottom_m + self.opening_height_m / 2


def load_pallet_prior(path: Path) -> PalletPrior:
    """Read a strict YAML shape file; malformed values raise ValueError."""
    import yaml

    try:
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError("Malformed pallet prior YAML") from exc
    dimensions = (
        "height_m",
        "deck_bottom_m",
        "deck_top_m",
        "opening_height_m",
        "overall_width_m",
        "overall_depth_m",
    )
    ranges = ("opening_width_range", "centre_spacer_range")
    tolerance_keys = {"opening_width_tolerance_m", "centre_spacer_tolerance_m"}
    optional = set(data) & tolerance_keys if isinstance(data, dict) else set()
    if optional and optional != tolerance_keys:
        raise ValueError("Both generation tolerances must be supplied together")
    if not isinstance(data, dict) or set(data) - optional != {
        *dimensions,
        *ranges,
        "source_provenance",
        "catalogue_version",
    }:
        raise ValueError("Pallet prior must contain exactly the documented fields")

    def positive(value, name):
        value = _finite_scalar(value, name)
        if value <= 0:
            raise ValueError(f"{name} must be positive")
        return value

    values = {name: positive(data[name], name) for name in dimensions}
    for name in ranges:
        pair = data[name]
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise ValueError(f"{name} must have two bounds")
        low, high = (positive(value, name) for value in pair)
        if low >= high:
            raise ValueError(f"{name} must have min < max")
        prefix = name.removesuffix("_range")
        values[f"{prefix}_min_m"] = low
        values[f"{prefix}_max_m"] = high
    if not math.isclose(
        values["height_m"],
        values["deck_bottom_m"] + values["opening_height_m"] + values["deck_top_m"],
        rel_tol=0,
        abs_tol=1e-9,
    ):
        raise ValueError(
            "height_m must equal deck_bottom_m plus opening_height_m plus deck_top_m"
        )
    if values["overall_width_m"] <= (
        2 * values["opening_width_max_m"] + values["centre_spacer_max_m"]
    ):
        raise ValueError("overall_width_m must leave room for outer supports")
    for prefix in ("opening_width", "centre_spacer"):
        if optional:
            tolerance = positive(data[f"{prefix}_tolerance_m"], f"{prefix}_tolerance_m")
            if not math.isclose(
                values[f"{prefix}_max_m"] - values[f"{prefix}_min_m"],
                2 * tolerance,
                rel_tol=0,
                abs_tol=1e-9,
            ):
                raise ValueError(f"{prefix} range must match its symmetric tolerance")
    provenance = data["source_provenance"]
    if not isinstance(provenance, str) or provenance not in PROVENANCES | {
        "epal6_published_standard"
    }:
        raise ValueError("Unsupported source_provenance")
    version = data["catalogue_version"]
    if not isinstance(version, str) or not version.strip():
        raise ValueError("catalogue_version must be a nonempty string")
    return PalletPrior(
        **values, source_provenance=provenance, catalogue_version=version
    )
