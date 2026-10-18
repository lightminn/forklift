"""The mission baseline of a live session (persistent console plan D2; state ownership table D0,
docs/design/2026-10-18-live-mission-state.md).

A live session runs many missions on one core. What a mission owns (the table's M0 rows, and its records) is taken
once before the session's first mission; each adoption puts it back -- the same objects (closures and the core hold
them), their mission keys replaced in place by fresh copies, keys made since dropped -- so a second mission starts
as the first did (use case U21). The last mission's values come back from ``restore`` for its record. What the table
calls the session's is never touched; the table's API rows (the worker, the capture, the pocket check, the SLAM
hold, the wheel lock) are closed by their owners at the mission's end, and its N rows are set by the start itself.
"""

from __future__ import annotations

import copy

_ABSENT = object()

# D0 sections 3-7: the mission's keys by owner. A key absent at capture is absent after each restore.
MISSION_KEYS = {
    "state": (
        "paths", "paths_replaced", "planning_status", "planning_target_source", "planning_traces", "planning_wall_s",
        "observation_attempts", "observation_candidates", "observation_waypoint_selected", "observation_drops",
        "observe_replans", "observe_replan_leg", "observe_loose_arrival", "observe_loose_arrivals",
        "perception", "near_capture", "near_capture_history", "near_capture_refused",
        "cusp_replans", "stall_replans", "docking_retries", "cusp_heading_watch", "cusp_stalls", "cusp_heading_hold",
        "async_plans", "async_plan_drops",
        "delivery_docking", "insertion_error", "insertion_measured", "insertion_truth_yaw_rad",
        "withdraw_certificate", "withdraw_tracking", "lift_tilt_peak", "slam_error_insert_end", "return_home_error",
        "tracking_failure", "handoff",
        "pocket_check", "forbidden_pocket_contacts", "pocket_geometry_checks", "repeat_capture", "phase",
        "lateral_clearance_min_m", "live_first_replans",
    ),
    "obstacle": (
        "pickup_estimate", "pallet_rel", "delivered_pending", "withdraw_line",
        "live_replanned", "live_retry_after_s", "live_plans",
        "replans", "replan_wait", "escapes", "backoffs", "backoff", "tight_replans", "mission_replans", "progress",
        "blocked_ticks", "path_blocked_ticks", "force_replan", "deadline", "permission_cap",
        "dumps", "dumped_phases", "recovery_dumps", "backoff_records", "return_reprojected_at_s",
    ),
    "slam": ("docking", "transport_scenario", "return_scenario", "transport_config"),
    "near": (
        "tracker", "start", "stop_state", "correction", "path_revision", "first_target", "retries", "retry_stand",
        "mismatch", "unbounded", "protect", "arrival", "drop_latch", "stale_waits", "last_stamp", "last_fed_stamp",
        "injection_closed", "stand_record", "terminal_standing_since",
    ),
    "pocket": (
        "capture", "history", "planning_pickup", "await_still_frame", "mid_still_done", "zero_since",
        "control_stamped", "still_frame_s", "still_frames", "last_reason",
        "frames_read", "frames_bad_time", "frames_without_time", "silenced_frames",
    ),
}
# Inside a session dict, the mission's fields only (D0 7: near_tracking's run fields; its settings are the session's).
NESTED_KEYS = {
    ("state", "near_tracking"): ("realigns", "realign", "retry", "outcome", "start", "starts", "arrival",
                                 "insert_arrival", ("injection", "captures")),
}
# What a mission's record keeps (the records the table hands to the mission's record, D1 step 7).
RECORD_OWNERS = ("state", "obstacle", "pocket")
# The table's session rows that share an owner with mission rows (checked by the tests: never in MISSION_KEYS).
SESSION_KEYS = {
    "state": ("samples", "tyre_contact_rows", "leg_switches", "tracker_versions", "transitions", "applied_command",
              "command_speed", "preempt_seen_s", "frames", "tracker_configs", "near_tracking"),
    "obstacle": ("layer", "version", "applied", "control_rear", "last_stamp", "last_raw", "ticks", "min_allowed",
                 "slowed", "reasons", "unpermitted", "events", "event_objects", "run_wall_start", "scans", "plans",
                 "reprojections", "silence_trace", "plan_cache", "memory_events"),
    "slam": ("link", "async_link", "tracker", "holds", "records", "control", "scan_id", "stop", "stop_now"),
    "near": ("bounds", "episode", "history", "commands", "events", "frames", "ticks", "path_revisions",
             "corrections", "stop_events"),
    "pocket": ("check",),
}
MS_FIELDS = ("observe_stall_ticks", "slam_stall_ticks", "dock_stop_ticks", "cusp_stop_ticks", "next_candidate_index",
             "last_tracking")
CORE_FIELDS = ("release_replan_needed", "fresh_depth_wait", "last_permission")


def _plain(value, where: str):
    """A deep copy of plain data; anything else is refused (a mission value must not alias a live object)."""
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, (list, tuple)):
        return type(value)(_plain(v, where) for v in value)
    if isinstance(value, dict):
        return {k: _plain(v, where) for k, v in value.items()}
    raise TypeError(f"mission baseline: {where} holds {type(value).__name__}, not plain data")


def _get(d: dict, path):
    path = path if isinstance(path, tuple) else (path,)
    for key in path:
        if not isinstance(d, dict) or key not in d:
            return _ABSENT
        d = d[key]
    return d


def _put(d: dict, path, value) -> None:
    path = path if isinstance(path, tuple) else (path,)
    for key in path[:-1]:
        if value is _ABSENT and not isinstance(d.get(key), dict):
            return  # absent then, and its parent is absent now: nothing to drop, nothing to make
        d = d.setdefault(key, {})
    if value is _ABSENT:
        d.pop(path[-1], None)
    else:
        d[path[-1]] = copy.deepcopy(value)


class MissionBaseline:
    def __init__(self, values: dict) -> None:
        self._values = values

    @classmethod
    def capture(cls, core) -> "MissionBaseline":
        values = {}
        for owner, keys in MISSION_KEYS.items():
            d = getattr(core, owner)
            if d is None:
                continue
            for key in keys:
                v = d.get(key, _ABSENT)
                values[(owner, key)] = v if v is _ABSENT else _plain(v, f"{owner}[{key!r}]")
        for (owner, key), fields in NESTED_KEYS.items():
            d = getattr(core, owner)
            inner = None if d is None else d.get(key)
            if isinstance(inner, dict):
                for field in fields:
                    v = _get(inner, field)
                    values[(owner, key, field)] = v if v is _ABSENT else _plain(v, f"{owner}[{key!r}]{field!r}")
        for name in MS_FIELDS:
            values[("ms", name)] = _plain(getattr(core.ms, name), f"ms.{name}")
        for name in CORE_FIELDS:
            values[("core", name)] = _plain(getattr(core, name), name)
        return cls(values)

    def restore(self, core) -> dict:
        """Put the mission's keys back; returns the last mission's record keys (moved out, not copied)."""
        records: dict = {owner: {} for owner in RECORD_OWNERS}
        for slot, value in self._values.items():
            owner = slot[0]
            if owner == "ms":
                setattr(core.ms, slot[1], copy.deepcopy(value))
            elif owner == "core":
                setattr(core, slot[1], copy.deepcopy(value))
            elif len(slot) == 2:
                d = getattr(core, owner)
                old = d.get(slot[1], _ABSENT)
                if owner in RECORD_OWNERS and old is not _ABSENT:
                    records[owner][slot[1]] = old
                _put(d, slot[1], value)
            else:
                inner = getattr(core, owner).get(slot[1])
                if isinstance(inner, dict):
                    _put(inner, slot[2], value)
        return records
