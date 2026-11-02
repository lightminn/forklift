"""The simulated pallet's mass for a run (docs/plans/2026-11-01-pallet-mass.md).

A settings file's ``pallet_mass_kg`` is either one number for every pallet (the provisional and older settings) or a
table by pallet shape -- the loaded geometry's ``geometry_version``, not its file name (Codex pallet-mass review P2-1)
-- for the measured chassis, whose runs carry EPAL 6 and the T11 test articles on one settings file. A run resolves it
to the number before anything reads the settings, so ``settings_synthetic`` and every reader keep seeing one number;
the table and the shape it was read for go into the run's ``pallet_mass`` record. No Isaac import: the runners'
preflights call it.
"""

from __future__ import annotations

import math

KEY = "pallet_mass_kg"


def resolve_pallet_mass(settings: dict, shape: str | None) -> tuple[dict, dict]:
    """(the settings with ``pallet_mass_kg`` as the run's number, the record of where it came from). A table needs
    the run's shape and an entry for it; a table entry and a plain number must be positive and finite. ``shape`` None:
    a runner that loads no pallet geometry, which can take a plain number only."""
    value = settings[KEY]
    if isinstance(value, dict):
        if shape is None:
            raise ValueError(f"{KEY} is a table by pallet shape, and this runner loads no pallet geometry: "
                             "give a settings file with one number")
        if shape not in value:
            raise ValueError(f"{KEY} has no entry for the pallet shape {shape!r} (it has {sorted(value)})")
        kg, source = value[shape], "settings_by_shape"
    else:
        kg, source = value, "settings_number"
    if isinstance(kg, bool) or not isinstance(kg, (int, float)) or not (math.isfinite(kg) and kg > 0):
        raise ValueError(f"{KEY} for {shape!r} must be a positive finite number, not {kg!r}")
    record = {"kg": float(kg), "shape": shape, "source": source, "settings_value": value}
    # a number stays the settings as read, so a run on the older settings records them as before
    return (settings if source == "settings_number" else {**settings, KEY: float(kg)}), record
