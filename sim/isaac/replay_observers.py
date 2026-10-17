"""Output-effect observers for a format-2 boundary record (docs/plans/2026-10-16-m-a1-replay.md, section 3.5).

From the hand-over on, a recording runner writes its files through these: the run's output directory becomes an
:func:`observed_output` path (``write_text``/``write_bytes`` on it or on any path made from it), and the runner's
``np`` becomes a :class:`NumpyObserver` (``save``, ``savez``, ``savez_compressed``). Each write made while a tick
is open and no recorded call runs is a ``Sink.<method>`` entry with its path relative to the output directory and
the payload's digest (:func:`forklift_core.mission.boundary_record.sink_effect`), and is then made as before. The
writes are seen where they happen, the same way in every source that installs the observers, without wrapping
the code that writes them (its own port reads stay recorded at their depth).
"""

from __future__ import annotations

import types
from pathlib import Path


def _logical(path, root: Path) -> str:
    try:
        return str(Path(path).relative_to(root))
    except ValueError:
        return str(path)


def observed_output(output: Path, gate_fn):
    """``output`` as a path whose writes (and those of every path joined from it) are recorded first."""
    root = Path(output)
    base = type(root)

    def _sink(method, path, payload, write):
        from forklift_core.mission.boundary_record import sink_effect

        return sink_effect(gate_fn(), method, _logical(path, root), payload, write)

    class ObservedPath(base):  # noqa: D401 -- a pathlib path; joined paths keep this type
        def write_text(self, data, *args, **kwargs):
            return _sink("write_text", self, data, lambda: base.write_text(self, data, *args, **kwargs))

        def write_bytes(self, data):
            return _sink("write_bytes", self, bytes(data), lambda: base.write_bytes(self, data))

    return ObservedPath(root)


class NumpyObserver(types.ModuleType):
    """numpy, with its file writers recorded first; every other name is numpy's own."""

    def __init__(self, numpy, gate_fn, root: Path) -> None:
        super().__init__("numpy")
        object.__setattr__(self, "_np", numpy)
        object.__setattr__(self, "_gate_fn", gate_fn)
        object.__setattr__(self, "_root", Path(root))

    def __getattr__(self, name):
        return getattr(object.__getattribute__(self, "_np"), name)

    def _sink(self, method, file, payload, write):
        from forklift_core.mission.boundary_record import sink_effect

        return sink_effect(object.__getattribute__(self, "_gate_fn")(), method,
                           _logical(file, object.__getattribute__(self, "_root")), payload, write)

    def save(self, file, arr, *args, **kwargs):
        np_ = object.__getattribute__(self, "_np")
        return self._sink("np_save", file, {"array": np_.asarray(arr)}, lambda: np_.save(file, arr, *args, **kwargs))

    def savez(self, file, *args, **kwds):
        np_ = object.__getattribute__(self, "_np")
        return self._sink("np_savez", file, {"args": list(args), "kwds": kwds}, lambda: np_.savez(file, *args, **kwds))

    def savez_compressed(self, file, *args, **kwds):
        np_ = object.__getattribute__(self, "_np")
        return self._sink("np_savez_compressed", file, {"args": list(args), "kwds": kwds},
                          lambda: np_.savez_compressed(file, *args, **kwds))
