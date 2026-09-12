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
    deck_m: float
    opening_height_m: float
    opening_width_min_m: float
    opening_width_max_m: float
    centre_spacer_min_m: float
    centre_spacer_max_m: float
    overall_width_m: float
    source_provenance: str
    catalogue_version: str

    @property
    def opening_centre_height_m(self) -> float:
        """Prior-derived centre height, not a measured vertical position."""
        return self.deck_m + self.opening_height_m / 2


def load_pallet_prior(path: Path) -> PalletPrior:
    """Read a strict YAML shape file; malformed values raise ValueError."""
    import yaml

    try:
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError("Malformed pallet prior YAML") from exc
    dimensions = ("height_m", "deck_m", "opening_height_m", "overall_width_m")
    ranges = ("opening_width_range", "centre_spacer_range")
    if not isinstance(data, dict) or set(data) != {
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
        2 * values["deck_m"] + values["opening_height_m"],
        rel_tol=0,
        abs_tol=1e-9,
    ):
        raise ValueError("height_m must equal two decks plus opening_height_m")
    if values["overall_width_m"] <= (
        2 * values["opening_width_max_m"] + values["centre_spacer_max_m"]
    ):
        raise ValueError("overall_width_m must leave room for outer supports")
    provenance = data["source_provenance"]
    if not isinstance(provenance, str) or provenance not in PROVENANCES:
        raise ValueError("Unsupported source_provenance")
    version = data["catalogue_version"]
    if not isinstance(version, str) or not version.strip():
        raise ValueError("catalogue_version must be a nonempty string")
    return PalletPrior(
        **values, source_provenance=provenance, catalogue_version=version
    )
