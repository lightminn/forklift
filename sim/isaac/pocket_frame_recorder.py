"""Plan D8b recording: every near-field depth read, the truth of every physics tick, and
the calibration dwells -- pure (no Isaac), so the bookkeeping is testable on the CPU.

The analysis (tools/d8b_calibration.py) measures the pixel delay from these records
without the detector, so nothing here filters a read: frames without a time, from the
future, out of order or with the depth of the previous read are all kept and labelled
(Codex D8b reviews: a dropped standing or late frame biases the delay band).

Depth is stored as float32 metres exactly as normalised (invalid = NaN): millimetre
steps would equal one tick of motion at 0.055 m/s. The truth is the physics state --
base_link pose (reduced-coordinate articulation, so the carriage follows from it and
the lift joint), lift joint position and the pallet's rigid-body pose -- plus the
camera prim's own world pose as the renderer may see it, for diagnosis only.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

DWELL_S = 2.0
PHASES = ("other", "observe", "approach", "insert")
RESULTS = ("ok", "no_depth", "bad_depth", "no_time", "future", "backwards", "same_stamp")


def parse_gaps(text: str) -> tuple[float, ...]:
    """Comma-separated camera-face distances, finite, positive and strictly decreasing."""
    try:
        gaps = tuple(float(v) for v in text.split(","))
    except ValueError as exc:
        raise ValueError(f"dwell gaps must be numbers: {text!r}") from exc
    if not gaps or not all(math.isfinite(g) and g > 0 for g in gaps):
        raise ValueError(f"dwell gaps must be finite and positive: {text!r}")
    if any(b >= a for a, b in zip(gaps, gaps[1:])):
        raise ValueError(f"dwell gaps must be strictly decreasing: {text!r}")
    return gaps


def classify_stamp(stamp_s: float | None, now_s: float, last_stamp_s: float | None) -> str:
    """The read's time label, in the runner's D5 order: no time, future, then order."""
    if stamp_s is None or not math.isfinite(stamp_s):
        return "no_time"
    if stamp_s > now_s + 1e-6:
        return "future"
    if last_stamp_s is not None and stamp_s < last_stamp_s:
        return "backwards"
    if last_stamp_s is not None and stamp_s == last_stamp_s:
        return "same_stamp"
    return "ok"


def fabric_id(frame_id):
    """``rendering_frame`` as JSON: a fabric time dict as [numerator, denominator]."""
    if isinstance(frame_id, dict):
        return [int(frame_id["referenceTimeNumerator"]), int(frame_id["referenceTimeDenominator"])]
    if frame_id is None:
        return None
    try:
        return int(frame_id)
    except (TypeError, ValueError):
        return str(frame_id)  # kept as given; the analysis then cannot use it


def json_default(value):
    """numpy scalars and arrays in the run's meta (the near capture record carries them)."""
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(f"{type(value).__name__} is not JSON serialisable")


def pose7(position, quaternion_wxyz) -> list[float]:
    return [float(v) for v in (*np.asarray(position, dtype=float), *np.asarray(quaternion_wxyz, dtype=float))]


class PocketFrameRecorder:
    """Writes ``pocket_frames/frame_NNNNN.npz`` per read; ``close`` writes the index and truth."""

    def __init__(self, output: Path):
        self.directory = Path(output) / "pocket_frames"
        self.directory.mkdir(parents=True, exist_ok=True)
        self.reads: list[dict] = []
        self.ticks: dict[str, list] = {"stamp_s": [], "rendered": [], "phase": [], "base_pose": [],
                                       "lift_m": [], "pallet_pose": [], "camera_prim_pose": [],
                                       "stop_now": [], "odom_speed_mps": [], "odom_yaw_rate_rps": [],
                                       "applied_speed_mps": [], "applied_steering_rad": []}
        # The camera prim pose is diagnosis only: its failures are counted, never fatal.
        self.camera_pose_errors = 0
        self.camera_pose_first_error: str | None = None
        self.tick_error: str | None = None
        self.phase = "other"
        self.last_stamp_s: float | None = None
        self.last_hash: str | None = None
        self.closed = False

    def record_read(self, *, read_s: float, stamp_s: float | None, rendering_frame, depth_m,
                    lift_m: float, control_rear=None, control_stamp_s: float | None = None,
                    odom_speed_mps: float | None = None, odom_yaw_rate_rps: float | None = None,
                    no_depth: bool = False, error: str | None = None) -> dict:
        """One read of the near-field camera. ``depth_m`` is the normalised depth; with
        none, ``no_depth`` says the camera returned none and ``error`` that it did not
        normalise. The time label is kept either way (Codex D8b impl P2)."""
        time_label = classify_stamp(stamp_s, read_s, self.last_stamp_s)
        result = "bad_depth" if error is not None else "no_depth" if no_depth or depth_m is None else time_label

        def opt(v):
            return None if v is None else float(v)

        row = {
            "index": len(self.reads), "read_s": float(read_s),
            "stamp_s": None if stamp_s is None or not math.isfinite(stamp_s) else float(stamp_s),
            "rendering_frame": fabric_id(rendering_frame), "result": result, "time_label": time_label,
            "error": error, "phase": self.phase, "lift_m": float(lift_m), "file": None, "duplicate": None,
            # The control pose of the loop tick read after: its own stamp (before the step),
            # not read_s (after it) -- 5 mm apart at 0.60 m/s (Codex D8b impl P3).
            "control_rear": None if control_rear is None else [float(v) for v in control_rear],
            "control_stamp_s": opt(control_stamp_s),
            "odom_speed_mps": opt(odom_speed_mps), "odom_yaw_rate_rps": opt(odom_yaw_rate_rps),
        }
        if depth_m is not None:
            depth = np.ascontiguousarray(np.asarray(depth_m, dtype=np.float32))
            digest = hashlib.sha256(depth.tobytes()).hexdigest()
            row["duplicate"] = digest == self.last_hash
            self.last_hash = digest
            name = f"frame_{row['index']:05d}.npz"
            np.savez_compressed(self.directory / name, depth_m=depth)
            row["file"] = name
        if result == "ok":
            self.last_stamp_s = row["stamp_s"]
        self.reads.append(row)
        return row

    def camera_pose_failed(self, exc: BaseException) -> None:
        self.camera_pose_errors += 1
        if self.camera_pose_first_error is None:
            self.camera_pose_first_error = repr(exc)

    def record_tick(self, *, stamp_s: float, rendered: bool, base_pose, lift_m: float, pallet_pose,
                    camera_prim_pose=None, stop_now=None, odom_speed_mps=None, odom_yaw_rate_rps=None,
                    applied_speed_mps=None, applied_steering_rad=None) -> None:
        """One physics tick. applied_* is the command the wheels and steering ran this tick
        (after the creep, slew and caps -- plan D8 S4a-1 ⓪: the drive terms are measured
        from it, not from the 0.1 s samples)."""
        self.ticks["stop_now"].append(-1 if stop_now is None else int(bool(stop_now)))
        self.ticks["applied_speed_mps"].append(math.nan if applied_speed_mps is None else float(applied_speed_mps))
        steering = [math.nan, math.nan] if applied_steering_rad is None else [float(v) for v in applied_steering_rad]
        if len(steering) != 2:
            raise ValueError("applied_steering_rad must hold the two steering joints")
        self.ticks["applied_steering_rad"].append(steering)
        self.ticks["odom_speed_mps"].append(math.nan if odom_speed_mps is None else float(odom_speed_mps))
        self.ticks["odom_yaw_rate_rps"].append(math.nan if odom_yaw_rate_rps is None else float(odom_yaw_rate_rps))
        self.ticks["stamp_s"].append(float(stamp_s))
        self.ticks["rendered"].append(bool(rendered))
        self.ticks["phase"].append(PHASES.index(self.phase) if self.phase in PHASES else 0)
        self.ticks["base_pose"].append(base_pose)
        self.ticks["lift_m"].append(float(lift_m))
        self.ticks["pallet_pose"].append(pallet_pose)
        self.ticks["camera_prim_pose"].append([math.nan] * 7 if camera_prim_pose is None else camera_prim_pose)

    def counts(self) -> dict:
        out = {r: 0 for r in RESULTS}
        for row in self.reads:
            out[row["result"]] += 1
        out["duplicate"] = sum(1 for row in self.reads if row["duplicate"])
        out["reads"] = len(self.reads)
        out["camera_pose_errors"] = self.camera_pose_errors
        out["camera_pose_first_error"] = self.camera_pose_first_error
        out["tick_error"] = self.tick_error
        return out

    def close(self, meta: dict, dwells: list[dict]) -> dict:
        """Write index.json and truth.npz once; returns the summary for the run record. A
        failed write raises and leaves it open, so a retry writes again (Codex D8b impl P1)."""
        if self.closed:
            return self.counts()
        np.savez_compressed(
            self.directory / "truth.npz",
            stamp_s=np.asarray(self.ticks["stamp_s"], dtype=float),
            rendered=np.asarray(self.ticks["rendered"], dtype=bool),
            phase=np.asarray(self.ticks["phase"], dtype=np.int8),
            base_pose=np.asarray(self.ticks["base_pose"], dtype=float).reshape(-1, 7),
            lift_m=np.asarray(self.ticks["lift_m"], dtype=float),
            pallet_pose=np.asarray(self.ticks["pallet_pose"], dtype=float).reshape(-1, 7),
            camera_prim_pose=np.asarray(self.ticks["camera_prim_pose"], dtype=float).reshape(-1, 7),
            stop_now=np.asarray(self.ticks["stop_now"], dtype=np.int8),
            odom_speed_mps=np.asarray(self.ticks["odom_speed_mps"], dtype=float),
            odom_yaw_rate_rps=np.asarray(self.ticks["odom_yaw_rate_rps"], dtype=float),
            applied_speed_mps=np.asarray(self.ticks["applied_speed_mps"], dtype=float),
            applied_steering_rad=np.asarray(self.ticks["applied_steering_rad"], dtype=float).reshape(-1, 2),
            phases=np.asarray(PHASES),
        )
        summary = {**self.counts(), "ticks": len(self.ticks["stamp_s"]), "dwells": len(dwells)}
        (self.directory / "index.json").write_text(
            json.dumps({"meta": meta, "summary": summary, "dwells": dwells, "reads": self.reads}, indent=1,
                       default=json_default) + "\n"
        )
        self.closed = True
        return summary


@dataclass
class DwellSchedule:
    """Plan D8b calibration dwells on the approach straight (0.055 m/s recording runs).

    One at the start of the straight (the truck stands after the near capture), one each
    time the control estimate of the camera-face distance first reaches a gap, and one
    after the approach arrives, before the switch to the insertion (the tracker arrives
    8 mm short and the runner switches at once, so a distance gap there never fires --
    Codex D8b 4th review). Each holds the truck at zero for ``hold_s`` from its first
    stop: the noise-aware stop detector flickers while the truck stands (smoke run 1856:
    18 changes in one stand), and restarting on every flicker stretched a dwell to 11 s.
    Which frames are static is the analysis's call, from the truth.
    """

    gaps_m: tuple[float, ...]
    hold_s: float = DWELL_S
    pending: list = field(init=False)
    active: dict | None = field(init=False, default=None)
    records: list = field(init=False, default_factory=list)
    started: bool = field(init=False, default=False)
    arrival_done: bool = field(init=False, default=False)

    def __post_init__(self) -> None:
        if not (math.isfinite(self.hold_s) and self.hold_s > 0):
            raise ValueError("hold_s must be finite and positive")
        self.pending = list(self.gaps_m)

    def _begin(self, kind: str, t: float, d_est_m: float) -> None:
        self.active = {"kind": kind, "trigger_s": float(t), "d_est_m": float(d_est_m), "still_since_s": None}

    def update(self, t: float, *, d_est_m: float, still: bool, arrived: bool) -> tuple[bool, bool]:
        """(hold, released) for this approach tick: hold -- command zero; released -- a dwell
        that held a moving truck just ended (restart the tracker's speed slew)."""
        if self.active is None:
            if not self.started:
                self.started = True
                self._begin("start", t, d_est_m)
            elif self.pending and d_est_m <= self.pending[0]:
                self.pending.pop(0)
                self._begin("gap", t, d_est_m)
            elif arrived and not self.arrival_done:
                self.arrival_done = True
                self._begin("arrival", t, d_est_m)
            else:
                return False, False
        dwell = self.active
        if dwell["still_since_s"] is None:
            if not still:
                return True, False
            dwell["still_since_s"] = float(t)
        if t - dwell["still_since_s"] < self.hold_s - 1e-9:
            return True, False
        self.records.append({**dwell, "end_s": float(t)})
        self.active = None
        return False, dwell["kind"] != "arrival"

    def held_s(self, t: float) -> float:
        """Time spent in dwells so far, for the runner's tracking timeout."""
        done = sum(r["end_s"] - r["trigger_s"] for r in self.records)
        return done + (t - self.active["trigger_s"] if self.active is not None else 0.0)


@dataclass
class SafetyStopSchedule:
    """Plan D8 S4a-1 ⓪: scheduled safety stops on the approach straight -- the stop the
    near-field tracker will make (command zero at once, steering held, no wheel lock),
    released ``hold_s`` after the noise-aware stop detector first sees the truck stand,
    then a slewed restart. Each fires the first time the near-capture estimate of the
    camera-face distance reaches its gap; the drive terms of the travel budget are
    measured from these runs."""

    gaps_m: tuple[float, ...]
    hold_s: float = 0.5
    pending: list = field(init=False)
    active: dict | None = field(init=False, default=None)
    records: list = field(init=False, default_factory=list)

    def __post_init__(self) -> None:
        gaps = [float(g) for g in self.gaps_m]
        if not gaps or any(not math.isfinite(g) or g <= 0 for g in gaps) or gaps != sorted(gaps, reverse=True):
            raise ValueError("gaps must be positive, finite and decreasing")
        if not (math.isfinite(self.hold_s) and self.hold_s >= 0):
            raise ValueError("hold_s must be finite and non-negative")
        self.pending = gaps

    def update(self, t: float, *, d_est_m: float, still: bool) -> tuple[bool, bool]:
        """(hold, released) for this approach tick: hold -- command zero, steering held;
        released -- the stop just ended (restart the tracker's speed slew)."""
        if self.active is None:
            if not (self.pending and d_est_m <= self.pending[0]):
                return False, False
            gap = self.pending.pop(0)
            self.active = {"gap_m": gap, "trigger_s": float(t), "d_est_m": float(d_est_m), "still_since_s": None}
        stop = self.active
        if stop["still_since_s"] is None:
            if not still:
                return True, False
            # Counted from the first stop, as the D8b dwells: the detector flickers while
            # the truck stands (one dwell, 18 changes), and the command stays zero.
            stop["still_since_s"] = float(t)
        if t - stop["still_since_s"] < self.hold_s - 1e-9:
            return True, False
        self.records.append({**stop, "end_s": float(t)})
        self.active = None
        return False, True

    def held_s(self, t: float) -> float:
        done = sum(r["end_s"] - r["trigger_s"] for r in self.records)
        return done + (t - self.active["trigger_s"] if self.active is not None else 0.0)

