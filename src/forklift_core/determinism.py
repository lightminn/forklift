"""The deterministic-run switch (docs/plans/2026-10-25-deterministic-sim-mode.md): an Isaac test-only mode under which
the same code, seed and inputs give the same records. Off (the default) nothing here changes a result.

On, every wall-clock budget a planner is given (a deadline, a SearchBudget, the return turnaround's limit) becomes
the wall cap, because a budget changes the result only when it is reached and reaching it depends on the machine's
load (D6); reaching the cap does not switch to another path -- it ends the run as ``DeterministicBroken``. The reasons
a run stopped being deterministic are kept in order (``broken``) for its result record. The runner turns it on with
``--deterministic``; a worker process reads ``FORKLIFT_DETERMINISTIC``."""

from __future__ import annotations

import os
import time

ENV = "FORKLIFT_DETERMINISTIC"
WALL_CAP_S = 300.0

_enabled = False
_broken: list[str] = []


class DeterministicBroken(RuntimeError):
    """A deterministic run met something that depends on the wall clock or the machine; it ends here."""


def configure(on: bool) -> None:
    global _enabled
    _enabled = bool(on)
    _broken.clear()


def configure_from_env() -> None:
    configure(os.environ.get(ENV, "0") == "1")


def enabled() -> bool:
    return _enabled


def budget(seconds: float) -> float:
    """A wall budget the caller would give: itself, or the cap when deterministic."""
    return WALL_CAP_S if _enabled else float(seconds)


def deadline(seconds: float, *, now: float | None = None) -> float:
    """An absolute monotonic deadline ``seconds`` from now (the cap when deterministic)."""
    return (time.monotonic() if now is None else float(now)) + budget(seconds)


def mark(reason: str) -> None:
    """Note why the run is no longer deterministic (ignored when off)."""
    if _enabled:
        _broken.append(str(reason))


def timed_out(where: str) -> None:
    """A wall budget ran out: off, the caller's timeout stands; on, the run ends."""
    if _enabled:
        mark(f"wall_cap:{where}")
        raise DeterministicBroken(f"wall_cap:{where}")


def broken() -> list[str]:
    return list(_broken)
