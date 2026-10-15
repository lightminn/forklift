"""Boundary record for the M-a replay evidence (docs/plans/2026-10-15-m-a05-boundary.md, section 5).

One record per control tick: its head (tick, loop time, work time, commands, stop kind), every port
call in order (port, method, request, response or exception, changes returned to the core) and its
tail (core outputs, branch names). Values are encoded when the call is recorded, so a service that
changes its arrays afterwards does not change the record. Arrays are stored once by content
(sha256 of dtype, shape and bytes) in ``arrays_NNNNN.npz`` beside ``boundary_NNNNN.jsonl``; ticks are
flushed every ``chunk_ticks`` so the record does not accumulate in memory. Past ``max_bytes`` the
recorder stops writing (the run goes on) and says so in ``boundary_meta.json``.
"""

from __future__ import annotations

import dataclasses
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterator

import numpy as np

__all__ = ["BoundaryRecorder", "read_record"]


class BoundaryRecorder:
    def __init__(self, directory: Path, *, chunk_ticks: int = 1200, max_bytes: int = 2_000_000_000) -> None:
        if chunk_ticks < 1 or max_bytes < 1:
            raise ValueError("chunk_ticks and max_bytes must be positive")
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.chunk_ticks = int(chunk_ticks)
        self.max_bytes = int(max_bytes)
        self._chunk = 0
        self._lines: list[str] = []
        self._chunk_arrays: dict[str, np.ndarray] = {}
        self._written: set[str] = set()
        self._tick: dict | None = None
        self._pending_arrays: dict[str, np.ndarray] = {}
        self._bytes = 0
        self._ticks = 0
        self.stopped_at_tick: int | None = None
        self.stopped_reason: str | None = None

    # -- recording ---------------------------------------------------------------------------------
    def tick_begin(self, tick: int, *, tick_t: float, work_t: float, commands: list, stop_kind: str) -> None:
        if self._tick is not None:
            raise RuntimeError("tick_begin before tick_end")
        self._pending_arrays = {}
        self._tick = {"tick": int(tick), "tick_t": self._encode(tick_t), "work_t": self._encode(work_t),
                      "commands": self._encode(commands), "stop_kind": stop_kind, "calls": []}

    def call(self, port: str, method: str, *, request: Any, response: Any = None, exception: BaseException | None = None,
             changes: Any = None) -> None:
        if self._tick is None:
            raise RuntimeError("boundary call recorded outside a tick")
        entry = {"seq": len(self._tick["calls"]), "port": port, "method": method, "request": self._encode(request)}
        if exception is not None:
            entry["exception"] = {"type": type(exception).__name__, "message": str(exception)}
        else:
            entry["response"] = self._encode(response)
        if changes is not None:
            entry["changes"] = self._encode(changes)
        self._tick["calls"].append(entry)

    def tick_end(self, *, outputs: Any, branches: list[str]) -> None:
        if self._tick is None:
            raise RuntimeError("tick_end without tick_begin")
        tick, self._tick = self._tick, None
        tick["outputs"] = self._encode(outputs)
        tick["branches"] = list(branches)
        if self.stopped_at_tick is not None:
            return
        line = json.dumps(tick, separators=(",", ":"), allow_nan=True)
        size = len(line) + 1 + sum(a.nbytes for a in self._pending_arrays.values())
        if self._bytes + size > self.max_bytes:
            self.stopped_at_tick, self.stopped_reason = tick["tick"], "max_bytes"
            return
        self._bytes += size
        self._lines.append(line)
        self._chunk_arrays.update(self._pending_arrays)
        self._written.update(self._pending_arrays)
        self._ticks += 1
        if len(self._lines) >= self.chunk_ticks:
            self._flush()

    def close(self) -> None:
        if self._tick is not None:
            raise RuntimeError("close inside a tick")
        self._flush()
        (self.directory / "boundary_meta.json").write_text(json.dumps({
            "format": "forklift_boundary_record/1", "ticks": self._ticks, "chunks": self._chunk,
            "bytes": self._bytes, "stopped_at_tick": self.stopped_at_tick, "stopped_reason": self.stopped_reason,
        }, indent=1) + "\n")

    # -- internals ---------------------------------------------------------------------------------
    def _flush(self) -> None:
        if not self._lines:
            return
        name = f"{self._chunk:05d}"
        (self.directory / f"boundary_{name}.jsonl").write_text("\n".join(self._lines) + "\n")
        if self._chunk_arrays:
            np.savez_compressed(self.directory / f"arrays_{name}.npz", **self._chunk_arrays)
        self._lines, self._chunk_arrays = [], {}
        self._chunk += 1

    def _array_ref(self, value: np.ndarray) -> dict:
        array = np.ascontiguousarray(value).copy()
        digest = hashlib.sha256()
        digest.update(str(array.dtype).encode())
        digest.update(repr(array.shape).encode())
        digest.update(array.tobytes())
        key = digest.hexdigest()
        if key not in self._written and key not in self._pending_arrays:
            self._pending_arrays[key] = array
        return {"$array": key, "dtype": str(array.dtype), "shape": list(array.shape)}

    def _encode(self, value: Any) -> Any:
        if isinstance(value, np.ndarray):
            return self._array_ref(value)
        if isinstance(value, (np.floating,)):
            return float(value)
        if isinstance(value, (np.integer,)):
            return int(value)
        if isinstance(value, (np.bool_,)):
            return bool(value)
        if isinstance(value, float) and not math.isfinite(value):
            return {"$float": repr(value)}
        if dataclasses.is_dataclass(value) and not isinstance(value, type):
            return {"$dataclass": type(value).__name__,
                    **{f.name: self._encode(getattr(value, f.name)) for f in dataclasses.fields(value)}}
        if isinstance(value, dict):
            return {str(k): self._encode(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [self._encode(v) for v in value]
        if value is None or isinstance(value, (bool, int, float, str)):
            return value
        raise TypeError(f"cannot record a {type(value).__name__}")


def read_record(directory: Path) -> Iterator[dict]:
    """The recorded ticks in order, arrays restored (non-finite floats as floats)."""
    directory = Path(directory)
    arrays: dict[str, np.ndarray] = {}
    for path in sorted(directory.glob("arrays_*.npz")):
        with np.load(path, allow_pickle=False) as z:
            arrays.update({k: z[k] for k in z.files})

    def decode(value: Any) -> Any:
        if isinstance(value, dict):
            if "$array" in value:
                return arrays[value["$array"]]
            if "$float" in value:
                return float(value["$float"])
            return {k: decode(v) for k, v in value.items()}
        if isinstance(value, list):
            return [decode(v) for v in value]
        return value

    for path in sorted(directory.glob("boundary_*.jsonl")):
        for line in path.read_text().splitlines():
            if line:
                yield decode(json.loads(line))
