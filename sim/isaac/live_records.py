"""The live run's spooled records (persistent console plan D8): the session lists that grow every step or scan go to
the record spool with their last item kept (the only one the running code reads), and the end-of-run summaries are
made from the chunks one at a time -- the same numbers a batch run makes from the whole list."""

from __future__ import annotations

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
