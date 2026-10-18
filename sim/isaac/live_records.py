"""The live run's spooled records (persistent console plan D8): the session lists that grow every step or scan go to
the record spool with their last item kept (the only one the running code reads), and the end-of-run summaries are
made from the chunks one at a time -- the same numbers a batch run makes from the whole list."""

from __future__ import annotations

import math

import numpy as np

JOINT = ("joint_stamps_s", "wheel_rates_rad_s", "steering_rad", "base_pose_world", "wheel_angles_rad",
         "wheel_counts", "wheel_increments_rad", "steering_measured_rad")  # one entry each per physics step
SCAN = ("scan_stamps_s", "scan_ranges_m", "laser_pose_world")  # one entry each per scan sent
NEAR_FRAMES_KEPT = 12_000  # a live mission's near-field frame records: 20 minutes at 10 Hz (plan D8)


def keep_near_frame(near: dict, row: dict, kept: int | None) -> None:
    """One near-field frame record: kept up to ``kept`` a mission (the mission's record, handed over at its end),
    the rest counted (Codex D8 design round 2 P2-1: a STOP while tracking pauses the mission, frames are still
    read). ``kept`` None (a batch run) keeps every frame, as before."""
    if kept is None or len(near["frames"]) < kept:
        near["frames"].append(row)
    else:
        near["frames_overflow"] = near.get("frames_overflow", 0) + 1


def register(spool, *, slam_log, slam, obstacle, state, video) -> list[str]:
    """Register what grows (keep 1); returns the names in order. ``obstacle["plans"]`` stays: its last entry is
    written right after it is added, and a mission adds a few. ``video`` is None unless the run records video."""
    names = []

    def add(name, items):
        if items is not None:
            spool.register(name, items, keep=1)
            names.append(name)

    if slam_log is not None:
        for name, keys in (("slam_log.joint", JOINT), ("slam_log.scan", SCAN)):
            spool.register_group(name, {k: slam_log[k] for k in keys if k in slam_log}, keep=1)
            names.append(name)
        add("slam_log.silenced", slam_log["silenced_scan_stamps_s"])
    if slam is not None:
        add("slam.control", slam["control"])
        add("slam.records", slam["records"])
    if obstacle is not None:
        add("obstacle.scans", obstacle["scans"])
    add("state.samples", state["samples"])
    add("state.tyre_contact_rows", state.get("tyre_contact_rows"))
    if video is not None:
        add("video.frame_audit", video["frame_audit"])
        add("video.frames", video["video_frames"])
        add("video.grids", video["overlay"]["grids"])
        add("video.plan_history", video["overlay"]["plan_history"])
    return names


def slam_summary(control_chunks, record_chunks) -> dict:
    """The run's SLAM summary fields (run_transport's slam_summary) from chunks of control rows ``[t, rear x, y, yaw,
    truth x, y, yaw]`` and of SLAM records, one chunk in memory at a time."""
    n, sq, sq_yaw, worst, worst_yaw = 0, 0.0, 0.0, None, None
    for chunk in control_chunks:
        control = np.asarray(chunk, dtype=float).reshape(-1, 7)
        if not len(control):
            continue
        error = np.hypot(control[:, 1] - control[:, 4], control[:, 2] - control[:, 5])
        yaw_error = np.abs(np.angle(np.exp(1j * (control[:, 3] - control[:, 6]))))
        n += len(control)
        sq += float(np.sum(error**2))
        sq_yaw += float(np.sum(yaw_error**2))
        worst = float(error.max()) if worst is None else max(worst, float(error.max()))
        worst_yaw = float(yaw_error.max()) if worst_yaw is None else max(worst_yaw, float(yaw_error.max()))
    counts: dict = {}
    for chunk in record_chunks:
        for record in chunk:
            counts[record["status"]] = counts.get(record["status"], 0) + 1
    return {"control_samples": n,
            "raw_position_rmse_m": float(np.sqrt(sq / n)) if n else None, "raw_position_max_m": worst,
            "raw_yaw_rmse_rad": float(np.sqrt(sq_yaw / n)) if n else None, "raw_yaw_max_rad": worst_yaw,
            "scans": sum(counts.values()), "processed": counts.get("processed", 0),
            "skipped": counts.get("skipped", 0), "link_failures": counts.get("link_failure", 0)}


def scan_wall_stats(scan_chunks) -> dict:
    """The obstacle layer's scans and wall seconds per scan (mean, max) from chunks of its scan records."""
    n, total, worst = 0, 0.0, 0.0
    for chunk in scan_chunks:
        for record in chunk:
            wall = float(record["total_wall_s"])
            n, total, worst = n + 1, total + wall, max(worst, wall)
    return {"scans": n, "wall_s_per_scan": {"mean": total / n if n else 0.0, "max": worst}}


FLOOR_PHASES = ("insert", "withdraw", "recover_withdraw")  # the pallet on the floor, the forks in its pockets
RIDING_PHASES = ("lift", "extract", "transport", "lower")  # the pallet on the forks
TRACE_BOUNDS = (0.02, 0.01, 0.01)  # longitudinal m, lateral m, heading rad (plan D4 6th ed.)


def _relative(frame, pose):
    fx, fy, fyaw = frame
    c, s = math.cos(fyaw), math.sin(fyaw)
    dx, dy = pose[0] - fx, pose[1] - fy
    return (c * dx + s * dy, -s * dx + c * dy, math.atan2(math.sin(pose[2] - fyaw), math.cos(pose[2] - fyaw)))


class TraceEvidence:
    """The simulation's truth check on the premise a recovery's retrace rests on (persistent console plan D4): the
    pallet on the floor stays where it was (its map pose against the insert's start, or the lower verdict); riding
    the forks it stays where it sat on them (its pose relative to the truck against the insert's end -- the instant
    the trace's placement uses). Beyond TRACE_BOUNDS the trace is no evidence: ``valid`` false until the next
    insert. Not a failure: a recovery then refuses before moving."""

    def __init__(self) -> None:
        self.valid = True
        self.floor_ref = None
        self.rel_ref = None
        self.riding = False
        self.phase = None

    def update(self, phase, base, pallet) -> bool:
        base, pallet = tuple(float(v) for v in base), tuple(float(v) for v in pallet)
        if phase != self.phase:
            if phase == "insert":
                self.valid, self.floor_ref, self.rel_ref, self.riding = True, pallet, None, False
            elif phase == "lift" and self.phase == "insert":
                self.rel_ref, self.riding = _relative(base, pallet), True
            elif phase in ("withdraw", "recover_withdraw"):
                self.floor_ref, self.riding = pallet, False  # the lower (or the recovery's ground) verdict
            elif phase == "recover_lower":
                self.riding = self.phase in RIDING_PHASES  # the pallet rode the forks when the recovery began
            self.phase = phase
        if not self.valid:
            return False
        if phase in RIDING_PHASES or (phase == "recover_lower" and self.riding):
            if self.rel_ref is not None:
                now = _relative(base, pallet)
                d = (now[0] - self.rel_ref[0], now[1] - self.rel_ref[1],
                     math.atan2(math.sin(now[2] - self.rel_ref[2]), math.cos(now[2] - self.rel_ref[2])))
                self.valid = all(abs(v) <= b for v, b in zip(d, TRACE_BOUNDS))
        elif phase in FLOOR_PHASES or phase == "recover_lower":
            if self.floor_ref is not None:
                self.valid = all(abs(v) <= b for v, b in zip(_relative(self.floor_ref, pallet), TRACE_BOUNDS))
        return self.valid
