"""Which implementation the hot obstacle and permission kernels use (real-time plan D2,
docs/plans/2026-10-17-realtime-sim.md).

- ``reference``: the original NumPy code, kept as the oracle every other implementation is compared with;
- ``numpy``: the vectorised NumPy code (S1), bit for bit the reference's results;
- ``numba``: compiled kernels (S3) where they exist, the ``numpy`` code elsewhere; needs the optional ``numba``.

``FORKLIFT_KERNELS`` picks one. Unset (or ``auto``) a process uses ``reference``, so a judged batch run computes
as before -- its planners spend wall-clock budgets, and faster kernels could let them weigh other candidates (Codex
real-time round 6 P2); a real-time run calls ``prefer_fast`` to take ``numba`` (or ``numpy`` without it). ``set_mode``
changes it in a running process (tests and ``tools/perf_bench.py``).
"""

from __future__ import annotations

import importlib.util
import os

MODES = ("reference", "numpy", "numba")


def _fastest() -> str:
    return "numba" if importlib.util.find_spec("numba") is not None else "numpy"


def _resolve(name: str) -> str:
    if name == "auto":
        return "reference"
    if name not in MODES:
        raise ValueError(f"FORKLIFT_KERNELS must be one of auto, {', '.join(MODES)}: {name!r}")
    if name == "numba" and importlib.util.find_spec("numba") is None:
        raise ValueError("FORKLIFT_KERNELS=numba but numba is not installed")
    return name


_ASKED = os.environ.get("FORKLIFT_KERNELS", "auto")
_mode = _resolve(_ASKED)


def mode() -> str:
    return _mode


def set_mode(name: str) -> str:
    """Set the implementation; returns the one it replaced."""
    global _mode
    previous, _mode = _mode, _resolve(name)
    return previous


def prefer_fast() -> str:
    """The fastest implementation here, unless FORKLIFT_KERNELS names one (a real-time run's default)."""
    if _ASKED == "auto":
        set_mode(_fastest())
    return _mode


def warm_up() -> float:
    """Compile (or load from the on-disk cache) every numba kernel now, on tiny inputs, so no compilation happens
    in a real-time run's control loop (real-time plan D3). Returns the seconds it took; 0 unless numba is chosen."""
    if _mode != "numba":
        return 0.0
    import time

    import numpy as np

    from forklift_core.control import permission_numba as pn
    from forklift_core.perception import grid_numba as gn

    start = time.perf_counter()
    one = np.ones(1)
    parts = (one * 1.0, one * 0.2, one * 0.3, one * 0.0, one * 0.0)
    state = np.ones((8, 8), dtype=np.uint8)
    stamp = np.full((8, 8), 0.5)
    box = pn.own_lookup(0.0, 0.0, 0.05, 8, 8, 0.2, 0.2, 1.0, 0.0, *parts)
    pn.walk(state, stamp, 0.0, 0.0, 0.05, one * 0.2, one * 0.2, one, one * 0.0, one * 0.0, one * 0.01, 1, False,
            *parts, True, box[0], box[1], box[2], box[3], box[4], box[5], box[6], True, np.zeros((8, 8), dtype=bool))
    mask = np.zeros((8, 8), dtype=bool)
    gn.ray_mask(8, 8, 0.0, 0.0, 0.05, 0.2, 0.2, one, one * 0.0, one * 0.1, np.arange(0.0, 0.1, 0.025))
    gn.line_close(mask, 2, 1)
    offsets = np.zeros((1, 2), dtype=np.int64)
    gn.erode(mask, offsets), gn.dilate(mask, offsets)
    rows = gn.row_intervals(offsets)
    gn.erode_rows(mask, rows), gn.dilate_rows(mask, rows)
    return time.perf_counter() - start


__all__ = ["MODES", "mode", "prefer_fast", "set_mode", "warm_up"]
