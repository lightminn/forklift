"""Look-alike places of a placed start's preview (docs/plans/2026-10-31-operator-placed-start.md; the lead's screen aid
of 2026-10-31). Plain math, no numpy: the console's mock vehicle uses it too."""

from __future__ import annotations

import math

TWIN_AWAY_M, TWIN_AWAY_RAD = 0.30, 0.10  # another place, not the preview refined: B-P2 4.0's "wrong"


def twin_count(cands, preview_base, r_amb: float) -> int | None:
    """The look-alike places of a placement's preview among the global candidates shown to the operator ((base,
    score) pairs, map): those scoring at least ``r_amb`` of the best and lying away from the preview. A screen aid
    (lead 2026-10-31), no part of any decision; None when no global candidates were shown."""
    if not cands:
        return None
    top = max(float(score) for _, score in cands)
    if top <= 0:
        return 0
    n = 0
    for base, score in cands:
        if float(score) / top < r_amb:
            continue
        dyaw = abs(math.atan2(math.sin(base[2] - preview_base[2]), math.cos(base[2] - preview_base[2])))
        if math.hypot(base[0] - preview_base[0], base[1] - preview_base[1]) > TWIN_AWAY_M or dyaw > TWIN_AWAY_RAD:
            n += 1
    return n


__all__ = ["TWIN_AWAY_M", "TWIN_AWAY_RAD", "twin_count"]
