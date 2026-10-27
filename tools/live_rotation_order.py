#!/usr/bin/env python3
"""The one-hour live order of the live-default plan's Isaac confirmation (docs/plans/2026-10-26-slam-localization-default.md
step 3): one pallet carried round the session's destinations, the other pallets left at their first places (two pallets
over three destinations block line_m's re-pickup straight -- Codex round 2). Prints the probe's --cycle list.

    python tools/live_rotation_order.py PALLET DEST [DEST ...] --legs N
"""

from __future__ import annotations

import argparse


def rotation(pallet_id: str, destinations: list[str], legs: int) -> list[tuple[str, str]]:
    """``legs`` (pallet, destination) legs: the pallet to each destination in turn, never to where it lies."""
    if len(destinations) < 2:
        raise ValueError("a rotation needs two destinations or more")
    return [(pallet_id, destinations[i % len(destinations)]) for i in range(legs)]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("pallet")
    ap.add_argument("destinations", nargs="+")
    ap.add_argument("--legs", type=int, default=24)
    a = ap.parse_args(argv)
    print(",".join(f"{p}:{d}" for p, d in rotation(a.pallet, a.destinations, a.legs)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
