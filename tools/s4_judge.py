"""Plan D8 S4 development judgement: the consumed-bound checks (i)-(vii) from a run's
physics truth (no Isaac).

docs/plans/2026-10-04-lidar-obstacle-map.md, 판정 변경 ③ and "S4a-2 구현 설계" ⑤. A run
recorded with --record-pocket-frames keeps the truth of every physics tick
(pocket_frames/truth.npz: base and pallet poses, lift, phase, applied command);
--near-tracking keeps near_tracking.json (every tick's stopping contract, each near-field
stop with what the stopping model was given, and when the S4a-2 protection started).

  (vi)  contact: no carriage box (everything on the carriage but the blades, heels
        included) shares volume with a pallet box at any approach or insert tick (exact
        OBB SAT at zero clearance, insertion_geometry.InsertionGeometry) -- applies to
        every run, an intended stop included; depth: both blades' true insertion at the
        insertion end (the last insert tick before the lift) within 0.300-0.346 m --
        applies when the run reached it.
  (vii) every stop command in the near-field section (a step to a zero applied command
        whose command tick lies between the section's entry and the last approach or
        insert tick): over the whole interval the command stays zero (to the tick it
        rises again or the approach and insertion end), the true rear-axle path length
        within the stopping contract the runner consumed, S = d_stop(vbar) + Delta_s, and,
        from the protection's start on, each carriage front corner's largest true travel
        along the pallet axis within k S and every blade corner's largest true travel
        across the body's axis at the command within the recorded lateral sweep; at the
        interval's end the truck stands -- over the last 0.1 s the rear axle's mean speed
        is at most 1 mm/s and its mean yaw rate at most 1 mrad/s. The contract is the
        near-field stop's own record, else the tick record at the command, else (before
        the protection only) the contract recomputed from the drive terms.

  (i)   every tick of the section (from the armed tick on): the carried estimate's true
        wall error (across
        the true pallet axis) at the gap watch's points -- each pocket's two walls at depths 0
        and the carriage limit -- within b_t of that tick (its latest accepted observation's
        bound, the odometry at its age, rho to the farthest point);
  (ii)  every tick of the section: the control-truth relative rear-axle odometry from the
        tick's pixel time to now within the measured table at that age;
  (iii) every timed read 12 ticks after the previous and one tick after its stamp, and the
        run's L (method 1 of the calibration, single-piece frames, the calibration's largest
        dwell bias bound for every distance it covers) within the measured band;
  (iv)  for every accepted observation that is the latest at some tick of the section, the
        true rear-axle travel from its conservative pixel time (stamp - L_hi) to the next
        accepted arrival (or the section's end) within 0.03 m;
  (v)   the pallet's true pose from the near capture to the insertion end (or the last
        approach or insert tick) within 0.01 mm and 0.01 mrad of its pose at the capture;
  (viii) S4b runs (a record with correction points): both points (1.0, 0.6 m) accepted once,
        in order, each opened on the first approach tick before entry whose estimate is
        within it, decided standing (unbroken from t_s) on the latest front observation of
        this hold taken at or after t_s, zero commands on the install and release ticks
        (their t + dt rows), every revision's paths equal to their content and to the
        followers' copies, every near-field tick on the revision installed before it, the
        final paths.json the last revision. An S4b smoke passes on (i)-(viii), success and
        phase "complete" ("s4b_pass").

Rule version v2 (2026-10-10, after the first results -- plan ③(vii)): a zero command
the near-field stop did not give (its tick record shows no hold, reason or terminal) is the
follower's; when it rises again inside the section it is "interrupted" -- its interval's
travel and sweeps are checked, standing is not required and it is counted apart from the
completed stops. v1 required standing at the end of every zero interval.

A run whose records are incomplete (a missing near-field record, drive terms or tick
record, ticks not contiguous at 120 Hz over the approach and insertion, non-finite values,
event or transition times off the ticks or outside the record, an insertion end that does
not match the recorded phases, sweeps missing after the protection) is "unverified",
never passed. Every input is hashed from the bytes that are parsed.

    python tools/s4_judge.py --runs <run dir> ... --output <dir>/s4_judge.json
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

RULES = "v2"
DEPTH_RANGE_M = (0.300, 0.346)
PHASES_CHECKED = ("approach", "insert")
TICK_S = 1.0 / 120.0
TIME_TOL_S = 1e-6  # loop and record times share one clock: a tick matches exactly
STOP_LATENCY_S = 0.15 + TICK_S
DECEL_MPS2 = 1.5
RECENT_S = 0.3  # the applied-command window of vbar (S4a-1)
STAND_SPEED_MPS = 0.001  # standing: the rear axle's mean speed over STAND_WINDOW_S
STAND_YAW_RATE_RPS = 0.001  # ... and its mean yaw rate
STAND_WINDOW_S = 0.1


class _NoTracker(Exception):
    """(ix): the injection's tracker never started (plan D8 S4c-3, an open realignment)."""


class _SkipEpisode(Exception):
    """A mismatch run: the anchor episode's (i)-(v) and (viii) are not applied."""


class Unverified(Exception):
    """The records cannot carry the judgement."""


def quat_matrix(q) -> np.ndarray:
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def stop_distance_m(v: float) -> float:
    v = abs(v)
    return v * STOP_LATENCY_S + v * v / (2 * DECEL_MPS2)


def yaw_of(base7_row: np.ndarray) -> float:
    r = quat_matrix(base7_row[3:])
    return math.atan2(r[1, 0], r[0, 0])


def rear_axle(base7: np.ndarray, rear_offset_m: float) -> np.ndarray:
    """World xy of the rear axle centre for each pose7 row."""
    out = np.empty((len(base7), 2))
    for i, row in enumerate(base7):
        r = quat_matrix(row[3:])
        out[i] = (row[:3] + r @ np.array([-abs(rear_offset_m), 0.0, 0.0]))[:2]
    return out


def body_point(base7_row: np.ndarray, point_base: tuple) -> np.ndarray:
    r = quat_matrix(base7_row[3:])
    return (base7_row[:3] + r @ np.array([point_base[0], point_base[1], 0.0]))[:2]


def body_axes(base7_row: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    r = quat_matrix(base7_row[3:])
    x = r[:2, 0] / np.linalg.norm(r[:2, 0])
    return x, np.array([-x[1], x[0]])


def pallet_axis(pallet7_row: np.ndarray, base7_row: np.ndarray) -> np.ndarray:
    """The pallet's insertion axis (world xy), pointing away from the truck."""
    r = quat_matrix(pallet7_row[3:])
    x = r[:2, 0] / np.linalg.norm(r[:2, 0])
    return -x if float(np.dot(body_axes(base7_row)[0], x)) < 0 else x


def checked_ticks(truth: dict) -> np.ndarray:
    phases = [str(p) for p in truth["phases"]]
    codes = [phases.index(p) for p in PHASES_CHECKED if p in phases]
    return np.flatnonzero(np.isin(truth["phase"], codes))


def tick_index(t: np.ndarray, time_s, what: str) -> int:
    """The record index of a time that must fall on a tick of the record."""
    if time_s is None or not math.isfinite(float(time_s)):
        raise Unverified(f"{what} has no time")
    i = int(np.argmin(np.abs(t - float(time_s))))
    if abs(t[i] - float(time_s)) > TIME_TOL_S:
        raise Unverified(f"{what} at {time_s} is off the record's ticks")
    return i


def record_problems(
    truth: dict, retry_spans=(), open_retry: bool = False, realign_span=None
) -> list[str]:
    """Why the truth record cannot carry the judgement over the approach and insertion (and
    over a realignment, plan D8 S4c-3 stage 2: realign_span = (first, last) truth ticks)."""
    problems = []
    keys = (
        "stamp_s",
        "base_pose",
        "pallet_pose",
        "lift_m",
        "phase",
        "phases",
        "applied_speed_mps",
    )
    missing = [k for k in keys if k not in truth]
    if missing:
        return [f"truth record lacks {missing}"]
    n = len(truth["stamp_s"])
    if any(len(truth[k]) != n for k in keys if k != "phases"):
        return ["truth arrays differ in length"]
    t = np.asarray(truth["stamp_s"], dtype=float)
    if n < 2 or not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0):
        return ["truth stamps are not finite and increasing"]
    if realign_span is not None:
        lo_, hi_ = int(realign_span[0]), int(realign_span[1])
        if not 0 <= lo_ <= hi_ < n:
            return ["the realignment is not inside the truth record"]
        span_r = slice(lo_, hi_ + 1)
        if np.any(np.abs(np.diff(t[span_r]) - TICK_S) > 0.05 * TICK_S):
            problems.append(
                "the truth ticks are not contiguous at 120 Hz over the realignment"
            )
        for key in ("base_pose", "pallet_pose", "lift_m", "applied_speed_mps"):
            if not np.all(np.isfinite(np.asarray(truth[key], dtype=float)[span_r])):
                problems.append(f"non-finite {key} over the realignment")
    ticks = checked_ticks(truth)
    if not len(ticks):
        if realign_span is not None:
            return problems  # a run that ended in its realignment: no approach to check
        return ["no approach or insert tick in the truth record"]
    # One tick past the last, when recorded: the command given at the last tick. A run
    # that ends on a near-field terminal stop or at the S4a-1 approach end records nothing
    # after it (its zero command simply holds to the end).
    # open_retry (plan D8 S4c-2, X_RULES v2): the run ended inside its retry -- the record is
    # checked to its end, where (x) and (v) judge it (Codex S4c-2 results review P2).
    span = slice(int(ticks[0]), n if open_retry else min(int(ticks[-1]) + 2, n))
    if np.any(np.abs(np.diff(t[span]) - TICK_S) > 0.05 * TICK_S):
        problems.append(
            "the truth ticks are not contiguous at 120 Hz over the approach and insertion"
        )
    for key in ("base_pose", "pallet_pose", "lift_m", "applied_speed_mps"):
        if not np.all(np.isfinite(np.asarray(truth[key], dtype=float)[span])):
            problems.append(f"non-finite {key} over the approach and insertion")
    phases = [str(p) for p in truth["phases"]]
    codes = [phases.index(p) for p in PHASES_CHECKED if p in phases]
    other = [
        int(i)
        for i in range(int(ticks[0]), int(ticks[-1]) + 1)
        if int(truth["phase"][i]) not in codes
    ]
    # Plan D8 S4c-2: between a retried pickup's episodes the truck stands, re-observes and
    # drives again; other phases are allowed there and only there (the ticks stay checked
    # contiguous and finite above, across the retry).
    if any(not any(lo < i < hi for lo, hi in retry_spans) for i in other):
        problems.append("another phase interrupts the approach and insertion ticks")
    return problems


def check_vi(
    truth: dict, geometry, pallet_depth_m: float, insert_end_index: int | None
) -> dict:
    """Carriage contact over the approach and insertion ticks; the final blade depths."""
    ticks = checked_ticks(truth)
    overlaps = []
    min_gap = math.inf
    phases = [str(p) for p in truth["phases"]]
    insert_code = phases.index("insert") if "insert" in phases else None
    for i in ticks:
        base, pallet, lift = (
            truth["base_pose"][i],
            truth["pallet_pose"][i],
            float(truth["lift_m"][i]),
        )
        hits = geometry.forbidden_contacts(
            base[:3],
            base[3:],
            lift,
            pallet[:3],
            pallet[3:],
            clearance_m=0.0,
            boxes=geometry.carriage_boxes,
        )
        if hits:
            overlaps.append(
                {"t": float(truth["stamp_s"][i]), "pairs": [list(h) for h in hits]}
            )
        if insert_code is not None and int(truth["phase"][i]) == insert_code:
            m = geometry.insertion_measures(
                base[:3], base[3:], lift, pallet[:3], pallet[3:], pallet_depth_m
            )
            min_gap = min(min_gap, m["carriage_face_gap_m"])
    contact = {
        "ticks_checked": int(len(ticks)),
        "carriage_overlap_ticks": len(overlaps),
        "first_overlaps": overlaps[:5],
        "min_carriage_face_gap_m": None
        if not math.isfinite(min_gap)
        else float(min_gap),
        "passed": bool(len(ticks)) and not overlaps,
    }
    depth = {"applicable": insert_end_index is not None}
    if insert_end_index is not None:
        i = insert_end_index
        base, pallet, lift = (
            truth["base_pose"][i],
            truth["pallet_pose"][i],
            float(truth["lift_m"][i]),
        )
        m = geometry.insertion_measures(
            base[:3], base[3:], lift, pallet[:3], pallet[3:], pallet_depth_m
        )
        depths = {side: m[side]["insertion_m"] for side in ("left", "right")}
        depth.update(
            end_t=float(truth["stamp_s"][i]),
            depths_m=depths,
            depth_range_m=list(DEPTH_RANGE_M),
            carriage_face_gap_at_end_m=m["carriage_face_gap_m"],
            passed=all(
                DEPTH_RANGE_M[0] - 1e-12 <= d <= DEPTH_RANGE_M[1] + 1e-12
                for d in depths.values()
            ),
        )
    passed = contact["passed"] and (depth["passed"] if depth["applicable"] else True)
    return {"contact": contact, "depth": depth, "passed": bool(passed)}


def standing_at(rear: np.ndarray, base: np.ndarray, t: np.ndarray, i: int) -> bool:
    """Over the STAND_WINDOW_S before tick i the rear axle's mean speed and the body's
    mean yaw rate are both within the standing thresholds."""
    k = int(round(STAND_WINDOW_S / TICK_S))
    if i - k < 0:
        return False
    path = float(np.sum(np.hypot(*np.diff(rear[i - k : i + 1], axis=0).T)))
    yaws = np.unwrap([yaw_of(base[j]) for j in range(i - k, i + 1)])
    turn = float(np.sum(np.abs(np.diff(yaws))))
    dt = t[i] - t[i - k]
    return (
        path / dt <= STAND_SPEED_MPS + 1e-12 and turn / dt <= STAND_YAW_RATE_RPS + 1e-12
    )


def drive_contract_m(applied: np.ndarray, t: np.ndarray, i0: int, terms: dict) -> float:
    """The stopping contract recomputed from the drive terms at a command tick: vbar = the
    largest |applied command| of the intervals ending within RECENT_S + delta_v."""
    recent = np.abs(applied[(t > t[i0] - RECENT_S + 1e-9) & (t <= t[i0] + 1e-9)])
    vbar = (float(recent.max()) if recent.size else 0.0) + float(terms["delta_v_mps"])
    k = int(math.ceil(STOP_LATENCY_S / float(terms["tick_s"]) - 1e-9))
    return stop_distance_m(vbar) + float(terms["delta_s_m"][k])


def check_vii(
    truth: dict,
    near: dict,
    terms: dict,
    span: tuple,
    rear_offset_m: float,
    corners: tuple,
    blades: tuple,
) -> dict:
    """Every stop command in the near-field section, against its contract and sweeps.

    span: (first, last) truth index of the section (the armed tick, the last approach or
    insert tick). Raises Unverified when a stop's records are missing."""
    t = np.asarray(truth["stamp_s"], dtype=float)
    base = np.asarray(truth["base_pose"], dtype=float)
    pallet = np.asarray(truth["pallet_pose"], dtype=float)
    applied = np.asarray(truth["applied_speed_mps"], dtype=float)
    rear = rear_axle(base, rear_offset_m)
    blade_corners = [
        (x, y) for x0, x1, y0, y1 in blades for x in (x0, x1) for y in (y0, y1)
    ]
    first, last = span
    if not 0 < first <= last <= len(t) - 1:
        raise Unverified("the near-field section is empty or outside the record")
    top = min(last + 1, len(t) - 1)  # the last record index a section command can reach
    # The two records end together: the near-field tick record runs every loop tick of the
    # approach and insertion, so its last tick is the truth's last approach or insert tick
    # -- also when the run ends there (a terminal stop, the S4a-1 approach end). A truth
    # record cut short would end earlier (Codex judge review).
    # Every time is first checked finite and on a record tick (a NaN compares false).
    near_ticks = [
        tick_index(t, row.get("t"), "a near-field tick record")
        for row in near.get("ticks") or []
    ]
    if not near_ticks or max(near_ticks) != last:
        raise Unverified(
            "the truth and the near-field tick records do not end at the same tick"
        )
    # One tick row for every truth tick of the section: a missing row would join two stops
    # into one (Codex judge review).
    if len(set(near_ticks)) != len(near_ticks):
        raise Unverified("two near-field tick records on one tick")
    missing_rows = sorted(set(range(first, last + 1)) - set(near_ticks))
    if missing_rows:
        raise Unverified(
            f"{len(missing_rows)} section ticks have no near-field tick record"
        )
    terminal = [
        tick_index(t, e.get("time_s"), "the terminal stop")
        for e in near.get("events") or []
        if e.get("event") == "terminal_standing"
    ]
    if terminal and (len(terminal) > 1 or terminal[0] != len(t) - 1):
        raise Unverified("the terminal stop is not the record's last tick")
    terminal_t = [float(t[i]) for i in terminal]
    tick_rows = near.get("ticks") or []
    ticks_by_t = {}
    for row in tick_rows:
        key = round(float(row["t"]), 9)
        if key in ticks_by_t:
            raise Unverified(f"two tick records at {row['t']}")
        ticks_by_t[key] = row
    protect = near.get("protect")
    protect_t = None
    if protect is not None:
        protect_t = float(
            t[tick_index(t, protect.get("time_s"), "the protection's start")]
        )
    protected = [float(row["t"]) for row in tick_rows if row.get("protect")]
    if (protect_t is None) != (not protected) or (
        protected and abs(min(protected) - protect_t) > TIME_TOL_S
    ):
        raise Unverified("the protection's start does not match the tick records")
    # The record at index j carries the command of the interval that began at t[j - 1]:
    # a step to zero at j is a command given at tick j - 1, in the section when first <=
    # j - 1 <= last.
    steps = [
        j
        for j in range(first + 1, top + 1)
        if abs(applied[j]) <= 1e-12 and abs(applied[j - 1]) > 1e-12
    ]
    events = {}
    for event in near.get("stop_events") or []:
        i0 = tick_index(t, event.get("start_s"), "a near-field stop")
        if i0 in events:
            raise Unverified(f"two near-field stops at {t[i0]:.4f} s")
        events[i0] = event
    event_ticks = set(
        events
    )  # every stop record's start tick (events is consumed below)
    results = []

    def stop_state(tick: dict, at: float) -> bool:
        """Whether a tick record shows a near-field stop; missing fields cannot say."""
        if "hold" not in tick or "reason" not in tick:
            raise Unverified(f"the tick record at {at:.4f} s lacks its stop state")
        return (
            bool(tick["hold"])
            or tick["reason"] is not None
            or tick.get("terminal") is not None
        )

    # Every near-field stop has its record: over the whole tick record, each tick where a
    # stop state begins (after a tick without one) is a stop record's start tick -- whatever
    # zero interval it falls in (Codex judge review).
    previous_state = False
    for row in sorted(tick_rows, key=lambda r: float(r["t"])):
        at = float(row["t"])
        state_now = stop_state(row, at)
        if (
            state_now
            and not previous_state
            and tick_index(t, at, "a tick record") not in event_ticks
        ):
            raise Unverified(
                f"a near-field stop begins at {at:.4f} s with no stop record"
            )
        previous_state = state_now

    def follower_interval_ok(i0: int, end: int) -> bool:
        """A follower's zero interval: every near-field stop that begins inside it must have
        its stop record at the tick it began (a hold run's first tick); returns whether the
        interval held no near-field stop at all."""
        clean = True
        previous = False
        for j in range(i0, min(end, last) + 1):  # the tick records end with the section
            tick = ticks_by_t.get(round(float(t[j]), 9))
            if tick is None:
                raise Unverified(
                    f"no tick record at {t[j]:.4f} s inside a zero command"
                )
            holding = stop_state(tick, float(t[j]))
            if holding and not previous and j not in event_ticks:
                raise Unverified(
                    f"a near-field stop begins at {t[j]:.4f} s with no stop record"
                )
            clean = clean and not holding
            previous = holding
        return clean

    def judge(i0: int, record: dict, sweep, ks, lateral) -> dict:
        # The positive command recorded at `rise` drove [t[rise - 1], t[rise]]: the zero
        # interval ends at rise - 1 (or at the section's last reachable tick).
        rise = next(
            (j for j in range(i0 + 2, top + 1) if abs(applied[j]) > 1e-12), top + 1
        )
        end = max(min(rise - 1, top), i0)
        stands = standing_at(rear, base, t, end)
        travel = float(np.sum(np.hypot(*np.diff(rear[i0 : end + 1], axis=0).T)))
        v_cmd = float(applied[i0])
        row = {
            **record,
            "command_t": float(t[i0]),
            "end_t": float(t[end]),
            "rose": rise <= top,
            "standing_at_end": stands,
            "travel_m": travel,
            "sweep_m": sweep,
            "travel_ok": travel <= sweep + 1e-9,
            "d_stop_cmd_m": stop_distance_m(v_cmd),
            "v_cmd_mps": v_cmd,
        }
        if ks is not None:
            axis = pallet_axis(pallet[i0], base[i0])
            _, lateral_axis = body_axes(base[i0])
            corner_travel = []
            for corner in corners:
                p0 = body_point(base[i0], corner)
                corner_travel.append(
                    max(
                        0.0,
                        max(
                            float(np.dot(body_point(base[j], corner) - p0, axis))
                            for j in range(i0, end + 1)
                        ),
                    )
                )
            blade_travel = max(
                abs(
                    float(
                        np.dot(
                            body_point(base[j], c) - body_point(base[i0], c),
                            lateral_axis,
                        )
                    )
                )
                for c in blade_corners
                for j in range(i0, end + 1)
            )
            row.update(
                corner_axial_m=corner_travel,
                corner_bound_m=[k * sweep for k in ks],
                corner_ok=all(
                    c <= k * sweep + 1e-9
                    for c, k in zip(corner_travel, ks, strict=True)
                ),
                blade_lateral_m=blade_travel,
                blade_bound_m=lateral,
                blade_ok=blade_travel <= lateral + 1e-9,
            )
        # v2 (after the first results, plan ③(vii)): a follower's zero command that rose again
        # inside the section is "interrupted" -- no stop to complete, its interval's travel and
        # sweeps still within the contract; near-field stops and intervals that end at the
        # section's or the record's end must stand.
        # Interrupted only when the restart command began inside the section (rise <= last:
        # the zero interval ends before the section's last tick) and no near-field stop
        # held during the interval -- a boundary end must stand (Codex judge review).
        row["interrupted"] = bool(
            record.get("follower") and rise <= last and follower_interval_ok(i0, end)
        )
        if record.get("follower") and not row["interrupted"]:
            follower_interval_ok(i0, end)  # still check the records inside the interval
        row["passed"] = bool(
            (stands or row["interrupted"])
            and row["travel_ok"]
            and row.get("corner_ok", True)
            and row.get("blade_ok", True)
        )
        return row

    for j in steps:
        i0 = j - 1
        after = protect_t is not None and t[i0] >= protect_t - TIME_TOL_S
        if i0 in events:
            event = events.pop(i0)
            sweep, ks, lateral = (
                event.get("sweep_m"),
                event.get("corner_sweep_k"),
                event.get("lateral_sweep_m"),
            )
            record = {"source": "near_stop", "reason": event.get("reason")}
        else:
            tick = ticks_by_t.get(round(float(t[i0]), 9))
            # v2: a zero command the near-field stop did not give -- its tick says no stop
            # (no hold, reason or terminal) -- is the follower's own; a tick that says a
            # stop with no stop record, or no tick at all, cannot be classified.
            if tick is None:
                raise Unverified(f"no tick record at the zero command of {t[i0]:.4f} s")
            if stop_state(tick, float(t[i0])):
                raise Unverified(
                    f"the tick at {t[i0]:.4f} s holds a near-field stop with no stop record"
                )
            protection = (tick or {}).get("protection") or {}
            sweep = (tick or {}).get("sweep_m")
            ks, lateral = (
                protection.get("corner_sweep_k"),
                protection.get("lateral_sweep_m"),
            )
            record = {
                "source": "zero_step",
                "contract": "tick" if sweep is not None else "drive_terms",
                "follower": True,
            }
            if sweep is None:
                if after:
                    raise Unverified(
                        f"no tick record of the stopping contract at {t[i0]:.4f} s after the protection"
                    )
                sweep = drive_contract_m(applied, t, i0, terms)
        if sweep is None or (after and (ks is None or lateral is None)):
            raise Unverified(f"the stop at {t[i0]:.4f} s lacks its contract or sweeps")
        results.append(
            judge(
                i0,
                record,
                float(sweep),
                ks if after else None,
                lateral if after else None,
            )
        )
    for i0, event in sorted(events.items()):
        # A stop that began while the command was already zero (a new reason on a standing
        # truck, or right after a release): judged from its own tick with its own record.
        # A terminal stop that begins on the record's last tick (a new terminal reason on a
        # standing truck) has no next tick: its zero command is the last one recorded.
        ends_run = (
            i0 == len(t) - 1 and terminal_t and abs(terminal_t[0] - t[i0]) <= TIME_TOL_S
        )
        if abs(applied[i0]) > 1e-12 or (
            not ends_run and (i0 + 1 > top or abs(applied[i0 + 1]) > 1e-12)
        ):
            raise Unverified(
                f"the near-field stop at {t[i0]:.4f} s has no zero command"
            )
        if not first <= i0 <= last:
            raise Unverified(
                f"the near-field stop at {t[i0]:.4f} s is outside the section"
            )
        after = protect_t is not None and t[i0] >= protect_t - TIME_TOL_S
        sweep, ks, lateral = (
            event.get("sweep_m"),
            event.get("corner_sweep_k"),
            event.get("lateral_sweep_m"),
        )
        if sweep is None or (after and (ks is None or lateral is None)):
            raise Unverified(f"the stop at {t[i0]:.4f} s lacks its contract or sweeps")
        results.append(
            judge(
                i0,
                {"source": "near_stop_held", "reason": event.get("reason")},
                float(sweep),
                ks if after else None,
                lateral if after else None,
            )
        )
    return {
        "stops": results,
        "zero_steps_in_section": len(steps),
        "completed_stops": sum(
            1 for r in results if not r["interrupted"] and r["standing_at_end"]
        ),
        "unfinished_stops": sum(
            1 for r in results if not r["interrupted"] and not r["standing_at_end"]
        ),
        "interrupted": sum(1 for r in results if r["interrupted"]),
        "passed": all(r["passed"] for r in results),
    }


def insert_end_index(truth: dict, result: dict) -> int | None:
    """The last insert tick, which must be the tick of the insert -> lift transition."""
    lift = next(
        (
            tr["time_s"]
            for tr in result.get("transitions") or []
            if tr.get("from") == "insert" and tr.get("to") == "lift"
        ),
        None,
    )
    if lift is None:
        return None
    t = np.asarray(truth["stamp_s"], dtype=float)
    phases = [str(p) for p in truth["phases"]]
    inserts = (
        np.flatnonzero(np.asarray(truth["phase"]) == phases.index("insert"))
        if "insert" in phases
        else []
    )
    if not len(inserts):
        raise Unverified("an insert -> lift transition without insert ticks")
    k = tick_index(t, lift, "the insert -> lift transition")
    if k != int(inserts[-1]):
        raise Unverified(
            "the insertion end does not match the last recorded insert tick"
        )
    return k


WARMUP_STAND_M = (
    0.001  # (ix): the truth travel allowed before a mismatch run's first row
)
WARMUP_STAND_RAD = 0.001


def mismatch_section(
    truth: dict,
    near: dict,
    rear_offset_m: float,
    end_index: int | None = None,
    begin_index: int | None = None,
) -> tuple[int, int]:
    """Plan D8 S4c-1: a mismatch run's section -- from the first near-field tick (the anchor
    episode ends before any armed section) to the last approach or insert tick. The ticks of
    the approach before that first row (the SLAM hold's warm-up) must not have driven: a
    record missing its start is unverified (Codex S4c-1 review P2)."""
    ticks = near.get("ticks") or []
    if not ticks:
        raise Unverified("no near-field tick record")
    t = np.asarray(truth["stamp_s"], dtype=float)
    first_row = tick_index(t, ticks[0]["t"], "the first near-field tick")
    checked = checked_ticks(truth)
    # begin_index: a later episode's first approach tick (plan D8 S4c-2).
    first_approach = int(checked[0]) if begin_index is None else int(begin_index)
    applied = np.asarray(truth["applied_speed_mps"], dtype=float)
    if first_row < first_approach or np.any(
        applied[first_approach : first_row + 1] != 0.0
    ):
        raise Unverified("the near-field record does not start with the approach")
    # A zero command is not a standstill: the truck must truly stand over that warm-up, so a
    # record cut at its front cannot hide a stop's travel (Codex S4c-1 2nd review P2).
    base = np.asarray(truth["base_pose"], dtype=float)
    rear = rear_axle(base, rear_offset_m)[first_approach : first_row + 1]
    path = float(np.sum(np.hypot(*np.diff(rear, axis=0).T))) if len(rear) > 1 else 0.0
    yaws = np.unwrap([yaw_of(base[j]) for j in range(first_approach, first_row + 1)])
    turn = float(np.sum(np.abs(np.diff(yaws)))) if len(yaws) > 1 else 0.0
    if path > WARMUP_STAND_M or turn > WARMUP_STAND_RAD:
        raise Unverified(
            f"the truck moved {1e3 * path:.2f} mm before the first near-field tick"
        )
    return first_row, int(checked[-1]) if end_index is None else int(end_index)


def near_section(
    truth: dict, near: dict, end_index: int | None = None
) -> tuple[int, int]:
    """Truth indices of the near-field section: the armed tick to the last approach or
    insert tick (or, for an episode of a retried pickup, end_index -- its last tick)."""
    armed = next(
        (e["time_s"] for e in near.get("events") or [] if e.get("event") == "armed"),
        None,
    )
    if armed is None:
        raise Unverified("the near-field section was never entered (no armed event)")
    t = np.asarray(truth["stamp_s"], dtype=float)
    first = tick_index(t, armed, "the armed event")
    last = int(checked_ticks(truth)[-1]) if end_index is None else int(end_index)
    if first > last:
        raise Unverified("the armed event is after the approach and insertion")
    return first, last


# (i)-(v) ---------------------------------------------------------------------------------
INVARIANT_M = (
    0.03  # D8e: travel from a result's conservative pixel time to the next result
)
PALLET_POS_M = 1e-5  # (v): the pallet's true motion against the anchor, 0.01 mm
PALLET_ROT_RAD = 1e-5  # ... and 0.01 mrad
PALLET_TERM_M = (
    0.00005  # b_t's pallet term (plan D8d), as the runner's gap watch uses it
)


def accepted_states(near: dict, start: dict, bounds) -> list[dict]:
    """The tracker's latest accepted observation over time: the near capture (the anchor,
    from the tracker's start record) then every fed frame that accepted, each with the time
    it became the latest (the frame's read time), its stamp, held pockets, widths and bound."""
    from forklift_core.perception.near_field_tracking import held_pockets
    from tools.d8c_replay import observation_from_dict

    observation = observation_from_dict(start["observation"])
    held = held_pockets(observation, tuple(start["pose"]))
    states = [
        {
            "since_s": float(start["built_s"]),
            "stamp_s": float(start["aligned_s"]) + bounds.align_latency_s,
            "held": held,
            "widths": (observation.left.width_m, observation.right.width_m),
            "bound": dict(bounds.near_capture),
            "source": "near_capture",
        }
    ]
    for row in near.get("frames") or []:
        if "skipped" in row or not row.get("accepted"):
            continue
        states.append(
            {
                "since_s": float(row["read_s"]),
                "stamp_s": float(row["latest_stamp_s"]),
                "held": row["latest_held"],
                "widths": tuple(row["latest_widths_m"]),
                "bound": dict(row["latest_bound"]),
                "source": row.get("latest_source"),
            }
        )
    for st in states:
        what = f"the accepted observation ({st['source']}) at {st['since_s']}"
        require_finite(f"{what}: times", (st["since_s"], st["stamp_s"]))
        try:
            held = [*st["held"]["left"], *st["held"]["right"], st["held"]["yaw"]]
            bound = [st["bound"]["wall_m"], st["bound"]["yaw_rad"]]
        except (KeyError, TypeError) as exc:
            raise Unverified(f"{what}: held pockets or bound incomplete") from exc
        require_finite(f"{what}: held pockets", held, size=7)
        require_finite(f"{what}: bound", [*bound, *st["bound"].values()])
        widths = require_finite(f"{what}: widths", st["widths"], size=2)
        if np.any(widths <= 0.0):
            raise Unverified(f"{what}: widths are not positive")
    if any(
        b["since_s"] < a["since_s"] for a, b in zip(states, states[1:], strict=False)
    ):
        raise Unverified("accepted observations out of time order")
    return states


def require_finite(what: str, values, size: int | None = None) -> np.ndarray:
    """The values as a float array, or Unverified when any is missing or not finite."""
    try:
        arr = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise Unverified(f"{what} is not numeric") from exc
    if (size is not None and arr.size != size) or not np.all(np.isfinite(arr)):
        raise Unverified(f"{what} is missing or not finite")
    return arr


def require_shape(what: str, value, shape: tuple) -> np.ndarray:
    """require_finite with an exact array shape (a scalar is shape ())."""
    arr = require_finite(what, value)
    if arr.shape != shape:
        raise Unverified(f"{what} has shape {arr.shape}, not {shape}")
    return arr


def require_count(what: str, value) -> int:
    """A record's integer (a JSON int, not a bool or a float) -- else unverified."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise Unverified(f"{what} {value!r} is not an integer")
    return value


def state_at(states: list[dict], t: float) -> dict:
    """The latest accepted observation at time t (results are delivered before the tick)."""
    out = None
    for st in states:
        if st["since_s"] <= t + TIME_TOL_S:
            out = st
        else:
            break
    if out is None:
        raise Unverified(f"no accepted observation at {t:.4f} s")
    return out


def check_i(
    run,
    near: dict,
    states: list[dict],
    bounds,
    geometry,
    section: tuple,
    carriage_limit_m: float,
    rear_x_m: float,
) -> dict:
    """(i): every tick of the section (from the armed tick on -- the plan's "after entry"),
    the carried estimate's true wall error at the gap watch's points (depths 0 and the
    carriage limit) within that tick's b_t."""
    import near_tracking as NT

    from forklift_core.perception.near_field_tracking import wall_erosion_m
    from tools import d8b_observations as OBS

    t = run.truth.t
    first, last = section
    worst = {"ratio": 0.0}
    checked = 0
    for row in near.get("ticks") or []:
        i = tick_index(t, row.get("t"), "a near-field tick record")
        if not first <= i <= last:
            continue
        st = state_at(states, float(row["t"]))
        base = require_finite("near-field tick base", row.get("base"), size=3)
        age = float(row["t"]) - (float(st["stamp_s"]) - bounds.max_latency_s)
        require_finite("observation age", age)
        odometry = bounds.odometry(age)
        if odometry is None:
            return {
                "ticks": checked,
                "passed": False,
                "reason": f"age {age:.3f} s past the odometry table",
            }
        face = NT.carried_face(st["held"], st["widths"], tuple(base))
        points = [
            (side, wall, depth, face[side][wall] + depth * face["axis"])
            for side in ("left", "right")
            for wall in ("outer", "inner")
            for depth in (0.0, carriage_limit_m)
        ]
        require_finite("carried wall points", [p for *_, p in points])
        rho = max(math.hypot(p[0] - rear_x_m, p[1]) for *_, p in points)
        truth = OBS.truth_in_base(run, float(row["t"]), geometry)
        require_finite("true pallet yaw", truth["yaw"])
        axis = np.array([math.cos(truth["yaw"]), math.sin(truth["yaw"])])
        lateral = np.array([-axis[1], axis[0]])
        for side, wall, depth, p in points:
            true_p = np.asarray(truth[f"{side}_walls"][wall][:2]) + depth * axis
            error = abs(float(np.dot(p - true_p, lateral)))
            allowed = wall_erosion_m(st["bound"], odometry, depth, rho, PALLET_TERM_M)
            require_finite("wall error and b_t", (error, allowed))
            if allowed <= 0.0:
                raise Unverified(f"b_t {allowed:.6g} m is not positive at {row['t']} s")
            if error / allowed > worst["ratio"]:
                worst = {
                    "ratio": error / allowed,
                    "t": float(row["t"]),
                    "side": side,
                    "wall": wall,
                    "depth_m": depth,
                    "error_m": error,
                    "allowed_m": allowed,
                    "age_s": age,
                }
        checked += 1
    return {"ticks": checked, "worst": worst, "passed": worst["ratio"] <= 1.0 + 1e-12}


def check_ii(run, near: dict, states: list[dict], bounds, section: tuple) -> dict:
    """(ii): every tick of the section, the control-truth relative rear-axle odometry over the
    age it used (its latest result's conservative pixel time to now) within the table."""
    from tools import d8b_calibration as CAL

    if run.control is None:
        raise Unverified("no slam_control.npy")
    c = run.control
    ct = c[:, 0]
    t = run.truth.t
    first, last = section

    def pose(cols, at):
        k = int(np.searchsorted(ct, at))
        exact = k < len(ct) and abs(ct[k] - at) <= TIME_TOL_S
        if not exact and not (0 < k < len(ct) and ct[k] - ct[k - 1] <= 1.5 * TICK_S):
            raise Unverified(f"the control record does not cover {at:.4f} s")
        yaw = np.unwrap(c[:, cols[2]])
        return np.array(
            [
                np.interp(at, ct, c[:, cols[0]]),
                np.interp(at, ct, c[:, cols[1]]),
                np.interp(at, ct, yaw),
            ]
        )

    worst = {"e_ratio": 0.0, "psi_ratio": 0.0}
    checked = 0
    for row in near.get("ticks") or []:
        i = tick_index(t, row.get("t"), "a near-field tick record")
        if not first <= i <= last:
            continue
        now = float(row["t"])
        pixel = float(
            require_finite("the tick's pixel time", row.get("pixel_s"), size=1)
        )
        # The recorded pixel time must be the reconstructed latest observation's (Codex
        # judge review P2: a forged pixel time would zero the age).
        expected = state_at(states, now)["stamp_s"] - bounds.max_latency_s
        if abs(pixel - expected) > TIME_TOL_S:
            raise Unverified(
                f"the tick at {now:.4f} s records pixel time {pixel:.6f}, the latest result gives "
                f"{expected:.6f}"
            )
        age = now - pixel
        bound = bounds.odometry(age)
        if bound is None:
            return {
                "ticks": checked,
                "passed": False,
                "reason": f"age {age:.3f} s past the odometry table",
            }
        d = CAL.relative(pose((1, 2, 3), pixel), pose((1, 2, 3), now)) - CAL.relative(
            pose((4, 5, 6), pixel), pose((4, 5, 6), now)
        )
        require_finite("the control and truth poses over the age", d, size=3)
        require_finite("the odometry bound", bound, size=2)
        e, psi = math.hypot(d[0], d[1]), abs(float(CAL.wrap(d[2])))
        for key, value, limit in (("e", e, bound[0]), ("psi", psi, bound[1])):
            ratio = value / limit if limit > 0 else (math.inf if value > 0 else 0.0)
            if ratio > worst[f"{key}_ratio"]:
                worst[f"{key}_ratio"] = ratio
                worst[f"{key}_at"] = {
                    "t": now,
                    "age_s": age,
                    "value": value,
                    "bound": limit,
                }
        checked += 1
    passed = worst["e_ratio"] <= 1.0 + 1e-12 and worst["psi_ratio"] <= 1.0 + 1e-12
    return {"ticks": checked, "worst": worst, "passed": passed}


def check_iii(run, bounds_data: dict, calibration: dict, model, spans=None) -> dict:
    """(iii): every read of the approach and insertion 12 ticks after the previous one and one
    tick after its stamp, and the run's L (the calibration's method ①, single-piece frames,
    with the calibration's largest dwell bias bound for every distance it covers) within the
    measured band."""
    from tools import d8b_calibration as CAL

    reads = sorted(
        (
            r
            for r in run.reads
            if r.get("time_label", r["result"]) == "ok" and r["stamp_s"] is not None
        ),
        key=lambda r: r["read_s"],
    )
    if len(reads) < 2:
        raise Unverified("fewer than two timed reads")
    require_finite("the read times", [(r["read_s"], r["stamp_s"]) for r in reads])
    delays = [r["read_s"] - r["stamp_s"] for r in reads]
    if spans is None:
        gaps = [
            b["stamp_s"] - a["stamp_s"] for a, b in zip(reads, reads[1:], strict=False)
        ]
    else:
        # Plan D8 S4c-2: the cadence within each episode -- no read is made between them.
        if any(
            not any(
                lo - TIME_TOL_S <= r["read_s"] <= hi + TIME_TOL_S for lo, hi in spans
            )
            for r in reads
        ):
            raise Unverified("a read outside every episode")
        gaps = []
        for lo, hi in spans:
            inside = [
                r for r in reads if lo - TIME_TOL_S <= r["read_s"] <= hi + TIME_TOL_S
            ]
            gaps += [
                b["stamp_s"] - a["stamp_s"]
                for a, b in zip(inside, inside[1:], strict=False)
            ]
    latency = bounds_data["latency"]
    a_s, period = require_finite(
        "the read delay and render period",
        (latency["read_delay_s"], latency["render_period_s"]),
        size=2,
    )
    cadence_ok = all(abs(d - a_s) <= 0.25 * TICK_S for d in delays) and all(
        abs(g - period) <= 0.25 * TICK_S for g in gaps
    )
    entries = []
    for dwells in calibration["dwells"].values():
        for rec in dwells:
            if "bias_m" not in rec or rec.get("distance_m") is None:
                continue
            low, high = rec.get("distance_range_m") or (
                rec["distance_m"],
                rec["distance_m"],
            )
            entries.append(
                (rec["distance_m"], abs(rec["bias_m"]) + rec["max_shift_m"], low, high)
            )
    if len(entries) < 2:
        raise Unverified("the calibration holds fewer than two measured dwells")
    require_finite("the calibration dwells", entries)
    global_bound = max(e[1] for e in entries)  # seed-independent: the largest measured
    entries = [(d, global_bound, lo, hi) for d, _, lo, hi in entries]
    rows = CAL.analyse_frames(run, model, entries)
    summary = CAL.summarise_run(rows)
    band = require_finite("the latency band", latency["band_s"], size=2).tolist()
    l_run = summary["L_run_s"]
    l_ok = l_run is not None and band[0] - 1e-12 <= l_run <= band[1] + 1e-12
    # The calibration's own failure conditions for a run (tools/d8b_calibration.py verdict):
    # enough eligible and single-piece frames, no mismatch, no piece past IN_RANGE_S, none at
    # the search edge (Codex judge review P1).
    frame_failures = []
    if (
        summary["eligible"] < CAL.MIN_ELIGIBLE
        or summary["single_piece"] < CAL.MIN_SINGLE
    ):
        frame_failures.append(
            f"{summary['eligible']} eligible, {summary['single_piece']} single-piece frames"
        )
    for key in ("mismatch", "out_of_range", "edge"):
        if summary[key]:
            frame_failures.append(f"{summary[key]} {key} frames")
    return {
        "reads": len(reads),
        "cadence_ok": cadence_ok,
        "max_delay_error_s": max(abs(d - a_s) for d in delays),
        "max_gap_error_s": max(abs(g - period) for g in gaps) if gaps else 0.0,
        "L_run_s": l_run,
        "band_s": band,
        "frames": {k: v for k, v in summary.items() if k != "L_run_s"},
        "global_bias_bound_m": global_bound,
        "frame_failures": frame_failures,
        "passed": bool(cadence_ok and l_ok and not frame_failures),
    }


def rear_at_time(t: np.ndarray, rear: np.ndarray, at: float) -> np.ndarray:
    """The true rear axle at a time between ticks (linear)."""
    k = int(np.clip(np.searchsorted(t, at), 1, len(t) - 1))
    a = float(np.clip((at - t[k - 1]) / (t[k] - t[k - 1]), 0.0, 1.0))
    return rear[k - 1] + a * (rear[k] - rear[k - 1])


def window_travel(t: np.ndarray, rear: np.ndarray, start: float, end: float) -> float:
    """The true rear-axle path length over [start, end], its ends interpolated between ticks."""
    inner = np.flatnonzero((t > start + TIME_TOL_S) & (t < end - TIME_TOL_S))
    points = [rear_at_time(t, rear, start), *rear[inner], rear_at_time(t, rear, end)]
    return float(np.sum(np.hypot(*np.diff(np.asarray(points), axis=0).T)))


def check_iv(
    truth: dict, states: list[dict], section: tuple, bounds, rear_offset_m: float
) -> dict:
    """(iv): for every accepted observation that is the latest at some tick of the section,
    the true rear-axle travel from its conservative pixel time (stamp - L_hi) to the next
    accepted arrival (or the section's end) within 0.03 m -- the whole window, also past a
    standstill (stricter than the plan's "or a stop"); the window's ends interpolated."""
    t = np.asarray(truth["stamp_s"], dtype=float)
    rear = rear_axle(np.asarray(truth["base_pose"], dtype=float), rear_offset_m)
    first, last = section
    t_last = float(t[last])
    used = {id(state_at(states, float(t[i]))) for i in range(first, last + 1)}
    worst = {"travel_m": 0.0}
    checked = 0
    for k, st in enumerate(states):
        if id(st) not in used:
            continue
        start = st["stamp_s"] - bounds.max_latency_s
        end = min(states[k + 1]["since_s"] if k + 1 < len(states) else t_last, t_last)
        require_finite("the window's ends", (start, end), size=2)
        if start < t[0] - TIME_TOL_S:
            raise Unverified(f"the pixel time {start:.4f} s is before the record")
        if end < start - TIME_TOL_S:
            raise Unverified(f"the window {start:.4f}-{end:.4f} s runs backwards")
        travel = window_travel(t, rear, start, end)
        require_finite(f"the travel from {start:.4f} s", travel, size=1)
        checked += 1
        if travel > worst["travel_m"]:
            worst = {
                "travel_m": travel,
                "from_s": start,
                "to_s": end,
                "source": st["source"],
            }
    return {
        "observations": checked,
        "worst": worst,
        "limit_m": INVARIANT_M,
        "passed": worst["travel_m"] <= INVARIANT_M + 1e-12,
    }


def check_v(truth: dict, anchor_t: float, end_index: int) -> dict:
    """(v): the pallet's true pose from the near capture to the insertion end (or the last
    approach or insert tick) against its pose at the near capture."""
    t = np.asarray(truth["stamp_s"], dtype=float)
    pallet = np.asarray(truth["pallet_pose"], dtype=float)
    i0 = tick_index(t, anchor_t, "the near capture")
    if end_index < i0:
        raise Unverified("the insertion end is before the near capture")
    require_finite("the pallet pose from the near capture", pallet[i0 : end_index + 1])
    r0, p0 = quat_matrix(pallet[i0, 3:]), pallet[i0, :3]
    pos, rot = 0.0, 0.0
    for i in range(i0, end_index + 1):
        pos = max(pos, float(np.linalg.norm(pallet[i, :3] - p0)))
        r = r0.T @ quat_matrix(pallet[i, 3:])
        rot = max(rot, math.acos(max(-1.0, min(1.0, (float(np.trace(r)) - 1.0) / 2.0))))
    return {
        "from_t": float(t[i0]),
        "to_t": float(t[end_index]),
        "max_translation_m": pos,
        "max_rotation_rad": rot,
        "passed": pos <= PALLET_POS_M + 1e-15 and rot <= PALLET_ROT_RAD + 1e-15,
    }


VIII_RULES = "v1"  # plan D8 S4b (viii), fixed before the first S4b run


def check_viii(
    truth: dict,
    near: dict,
    start: dict,
    paths_record: dict,
    states: list[dict],
    bounds,
    rear_offset_m: float,
    ended: str | None = None,
) -> dict:
    """(viii), plan D8 S4b: both correction points accepted once, in order; each opened on
    the first approach tick before entry whose estimate (rebuilt from the accepted
    observations) is within it; decided standing (the stop detector unbroken from t_s) on
    the front observation of this hold that was the latest, its pixel time at or after
    t_s; the install and release ticks' commands zero (the t + dt rows); every revision's
    paths equal to their recorded content and to the followers' own copies; every
    near-field tick's follower on the revision installed before it; the final paths.json
    the last revision. Reports the true rear-axle travel from the observation's pixel
    time to the decision, the target changes and the stop times."""
    import near_tracking as NT

    t = np.asarray(truth["stamp_s"], dtype=float)
    phases = [str(p) for p in truth["phases"]]
    applied = np.asarray(truth["applied_speed_mps"], dtype=float)
    rear = rear_axle(np.asarray(truth["base_pose"], dtype=float), rear_offset_m)
    corrections, revisions = near["corrections"], near["path_revisions"]
    tip_x, episode = float(start["fork_tip_x_m"]), start["episode"]
    rows = {
        tick_index(t, r.get("t"), "a near-field tick record"): r
        for r in near.get("ticks") or []
    }
    failures, report = [], []

    def row_at(time_s, what):
        i = tick_index(t, time_s, what)
        if i not in rows:
            raise Unverified(f"{what} at {time_s} has no near-field tick record")
        return i, rows[i]

    def distance_at(i):
        st = state_at(states, float(t[i]))
        face = NT.carried_face(
            st["held"],
            st["widths"],
            tuple(require_finite("tick base", rows[i]["base"], size=3)),
        )
        d = (
            min(
                face[s_][w_][0] for s_ in ("left", "right") for w_ in ("outer", "inner")
            )
            - tip_x
        )
        return d, NT.entered(face, tip_x)

    names = ("approach", "insert", "extract")

    def need_path(what, rec):
        """A recorded path's three arrays, (N, 3), (N,), (N,) and finite -- else unverified."""
        try:
            poses = require_finite(f"{what} poses", rec["poses"])
            dirs = require_finite(f"{what} directions", rec["directions"])
            curv = require_finite(f"{what} curvatures", rec["curvatures_inv_m"])
        except (KeyError, TypeError) as exc:
            raise Unverified(f"{what}: {exc!r} missing") from exc
        if (
            poses.ndim != 2
            or poses.shape[1:] != (3,)
            or len(poses) < 2
            or dirs.shape != (len(poses),)
            or curv.shape != (len(poses),)
        ):
            raise Unverified(f"{what}: the arrays are not (N, 3), (N,), (N,)")
        return poses, dirs, curv

    for r in revisions:
        what = f"revision {r.get('revision')}"
        try:
            require_count(f"{what} number", r["revision"])
            for key in ("time_s", "point_m"):
                require_shape(f"{what} {key}", r[key], ())
            for key in ("target", "previous_target"):
                require_shape(f"{what} {key}", r[key], (3,))
            for name in names:
                need_path(f"{what} {name}", r["paths"][name])
                if not isinstance(
                    r["paths"][name].get("sha256"), str
                ) or not isinstance(r["trackers"].get(name), str):
                    raise Unverified(f"{what} {name}: no path or follower digest")
        except (KeyError, TypeError, ValueError) as exc:
            raise Unverified(f"{what}: its record is incomplete ({exc!r})") from exc
    points = [c.get("point_m") for c in corrections]
    accepted = [c.get("accepted") for c in corrections]
    if ended is None:
        if points != list(NT.CORRECTION_POINTS_M) or accepted != [True] * len(
            NT.CORRECTION_POINTS_M
        ):
            failures.append(
                f"corrections {points} accepted {accepted}: not every point once, accepted"
            )
    else:
        # Plan D8 S4c-2: an episode that ended in a retry -- the points in order up to where it
        # ended; after a refusal the last one refused, after a mismatch none open.
        want = [True] * len(points)
        if ended == "replan_rejected" and want:
            want[-1] = False
        if (
            points != list(NT.CORRECTION_POINTS_M[: len(points)])
            or accepted != want
            or (ended == "replan_rejected" and not points)
        ):
            failures.append(
                f"corrections {points} accepted {accepted}: not the points in order up to the {ended}"
            )
    if [r["revision"] for r in revisions] != list(range(1, len(revisions) + 1)):
        raise Unverified("path revisions are not numbered 1, 2, ...")
    previous_closed = None
    for c in corrections:
        if not c.get("accepted"):
            continue
        try:
            keys = (
                "point_m",
                "trigger_s",
                "decision_s",
                "release_s",
                "closed_s",
                "face_distance_m",
            )
            point, trigger, decision, release, closed, recorded_d = (
                float(require_shape(f"correction {c.get('point_m')} {k}", c[k], ()))
                for k in keys
            )
            standing_ = require_finite(
                f"correction {point} standing_s", c["standing_s"]
            )
            if standing_.ndim != 1 or not standing_.size:
                raise Unverified(f"correction {point}: no t_s")
            t_s = float(standing_[-1])
            obs = c["observation"]
            obs_stamp, obs_pixel = (
                float(require_shape(f"correction {point} observation {k}", obs[k], ()))
                for k in ("stamp_s", "pixel_s")
            )
            number = require_count(f"correction {point} revision", c["revision"])
            if not 1 <= number <= len(revisions):
                raise Unverified(
                    f"correction {point}: revision {number} is not recorded"
                )
            revision = revisions[number - 1]
        except (KeyError, IndexError, TypeError, ValueError) as exc:
            raise Unverified(
                f"correction {c.get('point_m')}: its record is incomplete ({exc!r})"
            ) from exc
        i_trig, _ = row_at(trigger, "the trigger")
        if phases[int(truth["phase"][i_trig])] != "approach":
            failures.append(
                f"correction {point}: triggered in {phases[int(truth['phase'][i_trig])]}"
            )
        d, inside = distance_at(i_trig)
        if abs(d - recorded_d) > 1e-6:
            raise Unverified(
                f"correction {point}: the recorded distance {recorded_d} is not the rebuilt {d}"
            )
        if not d <= point + 1e-12 or inside:
            failures.append(
                f"correction {point}: opened at an estimate of {d:.4f} m (entered {inside})"
            )
        # The first crossing: no earlier applicable tick (from the first near-field tick, or
        # from the previous point's close) had the estimate within the point.
        first = (
            min(rows)
            if previous_closed is None
            else tick_index(t, previous_closed, "the previous point's close")
        )
        for i in range(first, i_trig):
            if phases[int(truth["phase"][i])] != "approach":
                continue
            if i not in rows:
                raise Unverified(
                    f"correction {point}: no tick record at {t[i]:.4f} s before its trigger"
                )
            d_i, _ = distance_at(i)
            if d_i <= point - 1e-12:
                failures.append(
                    f"correction {point}: the estimate was already {d_i:.4f} m at {t[i]:.4f} s"
                )
                break
        if previous_closed is not None and trigger <= previous_closed:
            failures.append(
                f"correction {point}: opened before the previous point closed"
            )
        if not trigger - TIME_TOL_S <= t_s <= decision + TIME_TOL_S:
            failures.append(
                f"correction {point}: t_s {t_s} outside its trigger and decision"
            )
        i_ts, i_dec = tick_index(t, t_s, "t_s"), tick_index(t, decision, "the decision")
        for i in range(i_ts, i_dec + 1):
            if i not in rows:
                raise Unverified(
                    f"correction {point}: no tick record at {t[i]:.4f} s between t_s and the decision"
                )
            if not rows[i].get("standing"):
                failures.append(
                    f"correction {point}: not standing at {t[i]:.4f} s after t_s"
                )
                break
        if i_ts > i_trig and rows.get(i_ts - 1, {}).get("standing"):
            failures.append(
                f"correction {point}: the stop detector already stood before the recorded t_s"
            )
        pixel = obs_stamp - bounds.max_latency_s
        if obs.get("source") != "front" or obs.get("frame_version") != episode:
            failures.append(
                f"correction {point}: observation {obs.get('source')} in hold {obs.get('frame_version')}"
            )
        if abs(pixel - obs_pixel) > 1e-9 or not pixel >= t_s - TIME_TOL_S:
            failures.append(
                f"correction {point}: pixel time {pixel:.4f} s against t_s {t_s:.4f} s"
            )
        latest = state_at(states, decision)
        if latest["source"] != "front" or abs(latest["stamp_s"] - obs_stamp) > 1e-9:
            failures.append(
                f"correction {point}: the observation used was not the latest accepted at the decision"
            )
        i_rel, row_rel = row_at(release, "the release")
        if abs(closed - float(t[min(i_dec + 1, len(t) - 1)])) > TIME_TOL_S:
            failures.append(
                f"correction {point}: closed at {closed:.4f} s, not the tick after its decision"
            )
        released_at = [
            i for i in range(i_dec + 1, i_rel + 1) if rows.get(i, {}).get("released")
        ]
        release_events = [
            e
            for e in near.get("events") or []
            if e.get("event") == "release"
            and abs(float(e.get("time_s", math.nan)) - release) <= TIME_TOL_S
        ]
        if (
            not release > decision + TIME_TOL_S
            or released_at != [i_rel]
            or not release_events
        ):
            failures.append(
                f"correction {point}: release_s {release:.4f} s is not the first release after its decision"
            )
        for name, i in (("install", i_dec), ("release", i_rel)):
            if i + 1 >= len(t):
                raise Unverified(f"correction {point}: no record after its {name} tick")
            if applied[i + 1] != 0.0:
                failures.append(
                    f"correction {point}: {name} tick command {applied[i + 1]} m/s"
                )
        if (
            abs(float(revision["time_s"]) - decision) > TIME_TOL_S
            or revision.get("point_m") != point
        ):
            failures.append(
                f"correction {point}: its revision {revision.get('revision')} is not this decision's"
            )
        for name in names:
            rec = revision["paths"][name]
            digest = NT.path_digest(
                *need_path(f"revision {revision['revision']} {name}", rec)
            )
            if not digest == rec["sha256"] == revision["trackers"][name]:
                failures.append(
                    f"revision {revision['revision']} {name}: path and follower contents differ"
                )
        prev_target = np.asarray(revision["previous_target"], dtype=float)
        new_target = np.asarray(revision["target"], dtype=float)
        c_, s_ = math.cos(prev_target[2]), math.sin(prev_target[2])
        delta = new_target[:2] - prev_target[:2]
        report.append(
            {
                "point_m": point,
                "stop_s": release - trigger,
                "wait_after_standing_s": decision - t_s,
                "observation_to_decision_travel_m": window_travel(
                    t, rear, pixel, decision
                )
                if pixel >= t[0]
                else None,
                "target_change": {
                    "along_m": float(delta @ (c_, s_)),
                    "lateral_m": float(delta @ (-s_, c_)),
                    "yaw_rad": float(
                        math.atan2(
                            math.sin(new_target[2] - prev_target[2]),
                            math.cos(new_target[2] - prev_target[2]),
                        )
                    ),
                },
            }
        )
        previous_closed = closed
    installs = [float(r["time_s"]) for r in revisions]
    for i, row in sorted(rows.items()):
        if "path_revision" not in row:
            raise Unverified(
                f"the tick at {t[i]:.4f} s does not record its path revision"
            )
        expected = sum(1 for at in installs if at < float(t[i]) - TIME_TOL_S)
        if row["path_revision"] != expected:
            failures.append(
                f"the tick at {t[i]:.4f} s followed revision {row['path_revision']}, not {expected}"
            )
            break
    if revisions and ended is None:  # the final paths.json is the final episode's
        last = revisions[-1]["paths"]
        for name in names:
            final = need_path(f"paths.json {name}", paths_record.get(name) or {})
            same = all(
                np.array_equal(a, b)
                for a, b in zip(
                    final, need_path(f"last revision {name}", last[name]), strict=True
                )
            )
            if not same:
                failures.append(f"paths.json {name} is not the last revision")
    return {
        "applicable": True,
        "rules": VIII_RULES,
        "corrections": report,
        "failures": failures,
        "passed": not failures,
    }


IX_RULES = "v1"  # plan D8 S4c-1 (ix), fixed before the first S4c run
COMPARE_REASONS = ("left_lateral", "right_lateral", "left_along", "right_along", "yaw")


def _site_from_observation(obs: dict, base: tuple, depth_m: float) -> np.ndarray:
    """estimate_pallet_center_m then estimate_world_pallet_site, from a recorded observation."""
    psi = float(obs["insertion_yaw_rad"])
    left, right = (
        np.asarray(obs[k]["center_m"], dtype=float) for k in ("left", "right")
    )
    centre = (left[:2] + right[:2]) / 2 + depth_m / 2 * np.array(
        [math.cos(psi), math.sin(psi)]
    )
    c, s_ = math.cos(base[2]), math.sin(base[2])
    yaw = math.atan2(math.sin(base[2] + psi), math.cos(base[2] + psi))
    return np.array(
        [
            base[0] + c * centre[0] - s_ * centre[1],
            base[1] + s_ * centre[0] + c * centre[1],
            math.pi if yaw == -math.pi else yaw,
        ]
    )


def _biased_site(site, lateral: float, yaw: float) -> np.ndarray:
    c, s_ = math.cos(site[2]), math.sin(site[2])
    y = math.atan2(math.sin(site[2] + yaw), math.cos(site[2] + yaw))
    return np.array(
        [site[0] - lateral * s_, site[1] + lateral * c, math.pi if y == -math.pi else y]
    )


def _same(a, b, tol=1e-9) -> bool:
    """Equal within tol; a site's yaw difference is wrapped."""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    if a.shape != b.shape:
        return False
    d = a - b
    if d.shape == (3,):
        d = d.copy()
        d[2] = math.atan2(math.sin(d[2]), math.cos(d[2]))
    return bool(np.all(np.abs(d) <= tol))


OBSERVATION_FIELDS = (
    "stamp_ns",
    "clock_domain",
    "frame_id",
    "source_provenance",
    "status",
    "position_sigma_m",
    "yaw_sigma_rad",
    "reason",
)


def _same_observation(a: dict, b: dict, numbers_b=None, tol=1e-9) -> bool:
    """Two recorded observations alike: valid, every non-geometric field equal, the
    geometry (or numbers_b in its place) within tol."""
    if a.get("status") != "valid" or any(
        a.get(k) != b.get(k) for k in OBSERVATION_FIELDS
    ):
        return False
    want = _observation_numbers(b) if numbers_b is None else numbers_b
    return _same(_observation_numbers(a), want, tol)


def _observation_numbers(obs: dict) -> np.ndarray:
    return np.array(
        [
            *obs["left"]["center_m"],
            obs["left"]["width_m"],
            obs["left"]["height_m"],
            *obs["right"]["center_m"],
            obs["right"]["width_m"],
            obs["right"]["height_m"],
            obs["insertion_yaw_rad"],
        ],
        dtype=float,
    )


def check_ix(
    result: dict,
    near: dict,
    truth: dict,
    retry: dict | None = None,
    start: dict | None = None,
) -> dict:
    """(ix), plan D8 S4c-1: the injection the run asked for went in exactly (common
    precondition), then by type -- correction: the near capture accepted, a correction
    accepted and the bias taken off by the last revision (|lateral| or |yaw| difference from
    the raw near-capture target within half the bias); mismatch: three consecutive
    association rejections before entry and the mismatch terminal; lost: the drop latched
    on the first tick within D, no frame fed after it, lost_wait before the terminal lost.
    Returns status pass / fail / not_run with the failures."""
    sys.path.insert(0, str(ROOT / "sim" / "isaac"))
    import near_tracking as NT

    from forklift_core.perception.pocket_observation import Pocket, PocketObservation

    nt = result.get("near_tracking") or {}
    inj = nt.get("injection")
    args = result.get("arguments") or {}
    if inj is None:
        if (
            args.get("near_inject") is not None
            or args.get("near_drop_frames_from_m") is not None
        ):
            raise Unverified("the run asked for an injection it did not record")
        return {"applicable": False}
    for key in (
        "near_inject",
        "near_inject_lateral_m",
        "near_inject_yaw_rad",
        "near_drop_frames_from_m",
    ):
        if key not in args:
            raise Unverified(f"the run's arguments lack {key}")
    try:
        kind, drop = inj["kind"], inj["drop_from_m"]
        lateral, yaw = (
            float(require_shape(f"injection {k}", inj[k], ()))
            for k in ("lateral_m", "yaw_rad")
        )
    except (KeyError, TypeError) as exc:
        raise Unverified(f"the injection record lacks {exc!r}") from exc
    failures = []
    report = {"kind": kind, "lateral_m": lateral, "yaw_rad": yaw, "drop_from_m": drop}
    t = np.asarray(truth["stamp_s"], dtype=float)
    events = near.get("events") or []
    outcome = nt.get("outcome")
    starts_ = nt.get("starts") or ([nt["start"]] if nt.get("start") else [])

    def done(status):
        return {
            "applicable": True,
            "rules": IX_RULES,
            "status": status,
            "failures": failures,
            **report,
        }

    asked = (
        args["near_inject"],
        float(args["near_inject_lateral_m"]),
        float(args["near_inject_yaw_rad"]),
        args["near_drop_frames_from_m"],
    )
    if asked != (kind, lateral, yaw, drop):
        failures.append("the run's arguments are not the recorded injection")
    if kind is not None:
        try:
            depth = float(result["geometry"]["pallet_depth_m"])
            captures = inj["captures"]
            phases = [c["phase"] for c in captures]
            # Only refused captures the injection reached count (a refusal after the first tracker
            # -- in a retry episode -- was never biased; Codex S4c-3 stage-2 review P2-5).
            injected_attempts = {c.get("attempt") for c in captures}
            refused_near = sum(
                1
                for c in result.get("near_capture_refused") or []
                if c.get("refused_attempt") in injected_attempts
            )
            if nt.get("realign") is not None and not starts_:
                # Plan D8 S4c-3: a run that ended in its realignment -- far captures, the
                # refused near capture(s), and what followed it.
                if phases.count("near") < refused_near or "far" not in phases:
                    failures.append(
                        f"captures {phases}: not the realignment's captures"
                    )
            elif (
                phases.count("near") != 1 + refused_near
                or phases[-1] != "near"
                or "far" not in phases
            ):
                failures.append(
                    f"captures {phases}: not far captures then one near capture"
                    + (f" after {refused_near} refused" if refused_near else "")
                )
            for c in captures:
                base = tuple(require_shape("capture base", c["capture_base"], (3,)))
                raw_target = _site_from_observation(c["raw_observation"], base, depth)
                if not _same(raw_target, c["raw_target"]):
                    failures.append(
                        f"capture {c['attempt']}: the raw target is not its raw observation's"
                    )
                if not _same(c["target"], _biased_site(raw_target, lateral, yaw)):
                    failures.append(
                        f"capture {c['attempt']}: the target is not the raw target biased by L, Y"
                    )
                if not _same(c.get("planned_target", [math.nan] * 3), c["target"]):
                    failures.append(
                        f"capture {c['attempt']}: the planner did not use the injected target"
                    )
                ro = c["raw_observation"]
                if kind == "correction":
                    want = _observation_numbers(ro)
                else:
                    pockets = [
                        Pocket(
                            tuple(ro[k]["center_m"]),
                            ro[k]["width_m"],
                            ro[k]["height_m"],
                        )
                        for k in ("left", "right")
                    ]
                    raw = PocketObservation(
                        ro["stamp_ns"],
                        ro["clock_domain"],
                        ro["frame_id"],
                        ro["source_provenance"],
                        ro["status"],
                        *pockets,
                        ro["insertion_yaw_rad"],
                        ro["position_sigma_m"],
                        ro["yaw_sigma_rad"],
                        ro["reason"],
                    )
                    b = NT.biased_observation(raw, depth, lateral, yaw)
                    want = np.array(
                        [
                            *b.left.center_m,
                            b.left.width_m,
                            b.left.height_m,
                            *b.right.center_m,
                            b.right.width_m,
                            b.right.height_m,
                            b.insertion_yaw_rad,
                        ]
                    )
                    if not _same(
                        _site_from_observation(c["observation"], base, depth),
                        c["target"],
                    ):
                        failures.append(
                            f"capture {c['attempt']}: the target is not its biased observation's"
                        )
                if not _same_observation(c["observation"], ro, want):
                    failures.append(
                        f"capture {c['attempt']}: the observation is not the {kind} injection's"
                    )
            # A retried run's record keeps the last episode's start; the injection's
            # tracker is the first episode's (Codex S4c-2 implementation review P2).
            if not starts_:
                raise _NoTracker
            start_obs = (start if start is not None else nt["start"])["observation"]
            if not _same_observation(
                start_obs,
                captures[-1]["observation"],
                tol=1e-12,
            ):
                failures.append(
                    "the tracker did not start from the near capture's observation"
                )
        except _NoTracker:
            pass  # Plan D8 S4c-3: the run ended in its realignment, no tracker started
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            raise Unverified(f"the injection record is incomplete ({exc!r})") from exc
    if kind == "correction":
        first_near = (
            result.get("near_capture_history") or [result.get("near_capture") or {}]
        )[0]
        if (first_near or {}).get("status") != "done" and nt.get("realign") is None:
            return done(
                "not_run"
            )  # the first acceptance refused: an injection design error
        accepted = [c for c in near.get("corrections") or [] if c.get("accepted")]
        revisions = near.get("path_revisions") or []
        if retry is not None and not accepted:
            pass  # Plan D8 S4c-2: refused at the first point -- (x) judges the retry instead
        elif not accepted or not revisions:
            failures.append("no correction accepted")
        else:
            # The bias was taken off: the first correction's target is the one its own front
            # observation gives (the tracker sees the raw frames), not the biased plan. A
            # threshold on the remaining difference from the raw near target cannot judge it:
            # the near capture's own lateral error reached 14.9 mm without a bias (S4b dev9
            # seed 4), past half a 20 mm bias.
            first = accepted[0]
            try:
                stamp = float(
                    require_shape(
                        "the correction's observation stamp",
                        first["observation"]["stamp_s"],
                        (),
                    )
                )
                decided = float(
                    require_shape(
                        "the correction's decision time", first["decision_s"], ()
                    )
                )
                number = require_count(
                    "the first correction's revision", first["revision"]
                )
                revision = revisions[number - 1]
            except (KeyError, TypeError, ValueError, IndexError) as exc:
                raise Unverified(
                    f"the first correction's record is incomplete ({exc!r})"
                ) from exc
            # The observation the decision had: the last accepted frame read by the decision
            # tick -- a later frame with the same stamp must not stand in (Codex S4c-1 review).
            frames_ = near.get("frames") or []
            if any(f.get("accepted") and "skipped" in f for f in frames_):
                raise Unverified("a frame is both accepted and skipped")
            before = [
                f
                for f in frames_
                if f.get("accepted")
                and "skipped" not in f
                and float(f["read_s"]) <= decided + TIME_TOL_S
            ]
            if (
                not before
                or before[-1].get("latest_source") != "front"
                or before[-1].get("latest_stamp_s") != stamp
            ):
                raise Unverified(
                    "the first correction's observation is not the latest accepted front frame at its decision"
                )
            try:
                held_ = before[-1]["latest_held"]
                held = {
                    "left": require_shape("held left pocket", held_["left"], (3,)),
                    "right": require_shape("held right pocket", held_["right"], (3,)),
                    "yaw": float(require_shape("held yaw", held_["yaw"], ())),
                }
            except (KeyError, TypeError) as exc:
                raise Unverified(
                    f"the first correction's held pockets are incomplete ({exc!r})"
                ) from exc
            depth = float(result["geometry"]["pallet_depth_m"])
            mid = (
                np.asarray(held["left"][:2], dtype=float)
                + np.asarray(held["right"][:2], dtype=float)
            ) / 2
            psi = float(held["yaw"])
            observed = np.array(
                [*(mid + depth / 2 * np.array([math.cos(psi), math.sin(psi)])), psi]
            )
            if not _same(
                require_shape("the first revision's target", revision["target"], (3,)),
                observed,
            ):
                failures.append(
                    "the first correction's target is not its observation's (the bias was carried on)"
                )
            planned = require_shape(
                "the near capture's planned target",
                inj["captures"][-1]["planned_target"],
                (3,),
            )
            raw = require_shape(
                "the raw near target", inj["captures"][-1]["raw_target"], (3,)
            )
            final = require_shape(
                "the last revision target", revisions[-1]["target"], (3,)
            )
            n = np.array([-math.sin(raw[2]), math.cos(raw[2])])

            def wrap(a):
                return math.atan2(math.sin(a), math.cos(a))

            report.update(
                first_change_lateral_m=float(
                    (np.asarray(revision["target"][:2]) - planned[:2]) @ n
                ),
                first_change_yaw_rad=wrap(float(revision["target"][2]) - planned[2]),
                left_lateral_m=float((final[:2] - raw[:2]) @ n),
                left_yaw_rad=wrap(final[2] - raw[2]),
            )
    elif kind == "mismatch":
        frames = [f for f in near.get("frames") or [] if "skipped" not in f]
        k = next((i for i, f in enumerate(frames) if f.get("mismatch")), None)
        if k is None or k < 2:
            failures.append(
                "no mismatch after three rejected frames (the expected branch was not taken)"
            )
        else:
            three = frames[k - 2 : k + 1]
            counts = [f.get("consecutive_rejects") for f in three]
            reasons = [str(f.get("reason") or "") for f in three]
            report.update(
                rejections=[
                    {"read_s": f["read_s"], "reason": r}
                    for f, r in zip(three, reasons, strict=True)
                ]
            )
            if counts != [1, 2, 3] or not all(f.get("rejected") for f in three):
                failures.append(
                    f"the mismatch did not follow three consecutive rejections ({counts})"
                )
            if not all(r.removeprefix("anchor_") in COMPARE_REASONS for r in reasons):
                failures.append(f"rejections {reasons} are not association comparisons")
            marks = [e for e in events if e.get("event") == "mismatch"]
            stamp3 = float(
                require_shape(
                    "the third rejected frame's stamp", three[-1].get("stamp_s"), ()
                )
            )
            if not marks or not _same(
                require_shape(
                    "the mismatch event's stamp", marks[0].get("stamp_s"), ()
                ),
                stamp3,
                0.0,
            ):
                failures.append("no mismatch event on the third rejected frame")
            before = [
                r
                for r in near.get("ticks") or []
                if float(r["t"]) <= float(three[-1]["read_s"]) + TIME_TOL_S
            ]
            if not before or not float(before[-1].get("face_distance_m", -1.0)) > 0.0:
                failures.append(
                    "the mismatch was not before the fork tips reached the face"
                )
        terminal = [e for e in events if e.get("event") == "terminal_standing"]
        if retry is not None:
            # Plan D8 S4c-2: the mismatch ended the episode in a retry, standing.
            if retry.get("reason") != "mismatch":
                failures.append(
                    f"the episode ended in a {retry.get('reason')} retry, not on the mismatch"
                )
            else:
                report.update(terminal_s=float(retry["time_s"]))
        elif (
            outcome != "near_field_mismatch"
            or not terminal
            or terminal[-1].get("reason") != "mismatch"
        ):
            failures.append(f"the run ended {outcome}, not standing on the mismatch")
        else:
            report.update(terminal_s=float(terminal[-1]["time_s"]))
    if drop is not None:
        latch = next((e for e in events if e.get("event") == "drop_latch"), None)
        if latch is None:
            return done("not_run")  # ended before the injection point
        rows = dict(
            sorted(
                (tick_index(t, r.get("t"), "a near-field tick record"), r)
                for r in near.get("ticks") or []
            )
        )
        i_latch = tick_index(t, latch["time_s"], "the drop latch")
        try:
            d_latch = float(
                require_shape(
                    "the latch distance", rows[i_latch]["face_distance_m"], ()
                )
            )
            d_before = float(
                require_shape(
                    "the distance before the latch",
                    rows[i_latch - 1]["face_distance_m"],
                    (),
                )
            )
        except KeyError as exc:
            raise Unverified(
                "the latch tick or the one before has no tick record"
            ) from exc
        report.update(latch_s=float(latch["time_s"]), latch_distance_m=d_latch)
        earlier = [
            float(r.get("face_distance_m", math.nan))
            for i, r in rows.items()
            if i < i_latch
        ]
        if not (
            d_latch <= drop + 1e-12
            and d_before > drop
            and all(d > drop for d in earlier)
        ):
            failures.append(
                f"latched at {d_latch:.4f} m after {d_before:.4f} m: not the first tick within {drop}"
            )
        after = [
            f
            for f in near.get("frames") or []
            # A frame read at the latch's own time was fed before that tick latched (frames
            # are read after the step, the next tick decides) -- only later reads count.
            if float(f["read_s"]) > float(latch["time_s"]) + TIME_TOL_S
        ]
        fed = [f for f in after if f.get("skipped") != "dropped"]
        report.update(dropped=len(after) - len(fed))
        if fed or not after:
            failures.append(
                f"{len(fed)} frames reached the tracker after the latch, {len(after) - len(fed)} dropped"
            )
        terminal = [e for e in events if e.get("event") == "terminal_standing"]
        end_t = float(terminal[-1]["time_s"]) if terminal else math.inf
        waited = [
            r
            for i, r in rows.items()
            if i >= i_latch
            and float(r["t"]) < end_t - TIME_TOL_S
            and "lost_wait" in (r.get("waits") or [])
        ]
        if not waited:
            failures.append("no lost_wait between the latch and the terminal")
        if (
            outcome != "near_field_lost"
            or not terminal
            or terminal[-1].get("reason") != "lost"
        ):
            failures.append(f"the run ended {outcome}, not standing on the loss")
    return done("fail" if failures else "pass")


def s4c_outcome(
    verdict: str,
    checks: dict,
    result: dict,
    vi: dict | None,
    contacts,
    vii: dict | None = None,
) -> dict | None:
    """Plan D8 S4c-1: the run's result for its group -- correction: (i)-(ix) and the mission
    complete; mismatch: (vi) contact, (vii) and (ix) on the anchor episode; lost: (i)-(v),
    (vii), (viii), (ix) and (vi) contact; every group also needs no forbidden pocket
    contact. A run that never reached its injection point is not_run."""
    ix = checks.get("ix")
    if not ix or not ix.get("applicable"):
        return None
    group = ix["kind"] or "lost"
    if ix["status"] == "not_run":
        return {"group": group, "status": "not_run"}
    no_contact = contacts == [] and bool(vi) and vi["contact"]["passed"]
    ok = verdict == "pass" and ix["status"] == "pass" and no_contact
    if (checks.get("xi") or {}).get("applicable"):
        # Plan D8 S4c-3 stage 2: a realignment -- (xi) and the mission complete.
        ok = (
            ok
            and checks["xi"]["passed"]
            and result.get("success") is True
            and result.get("phase") == "complete"
        )
    if "x" in checks:
        # Plan D8 S4c-2: a retried pickup -- the retry, every episode and the mission complete.
        ok = (
            ok
            and checks["x"]["passed"]
            and checks["episodes"]["passed"]
            and result.get("success") is True
            and result.get("phase") == "complete"
        )
    if ix["kind"] != "mismatch":
        # Correction and lost runs need both correction points judged (Codex S4c-1 review P2).
        ok = ok and bool((checks.get("viii") or {}).get("passed"))
    else:
        # The terminal must lie in a judged near-field stop that passed and ended standing --
        # an empty stop list passes (vii) vacuously (Codex S4c-1 2nd review P2).
        end = ix.get("terminal_s")
        ok = (
            ok
            and end is not None
            and any(
                str(st.get("source", "")).startswith("near_stop")
                and st["command_t"] <= end + TIME_TOL_S <= st["end_t"] + 2 * TIME_TOL_S
                and st.get("standing_at_end")
                and st.get("passed")
                for st in (vii or {}).get("stops") or []
            )
        )
    if ix["kind"] == "correction":
        ok = ok and result.get("success") is True and result.get("phase") == "complete"
    return {"group": group, "status": "pass" if ok else "fail"}


# Plan D8 S4c-2 (x): v1 fixed before the first retry run; v2 after the first smoke (seed 1,
# mismatch yaw -0.05 ended near_capture_misaligned inside its retry and v1 called it
# unverified) -- a run that ends inside its retry span is judged, and fails (x); v3 (plan
# D8 S4c-3, a strengthening found by the stage-2 design review) -- the re-observation
# stood in place and its recorded capture pose is the estimate at that capture.
X_RULES = "v3"
STAND_M = (
    0.001  # (x) v3: the truth travel allowed from the release to the re-observation
)
STAND_RAD = 0.001


def split_episodes(near: dict, nt: dict) -> list[dict]:
    """Plan D8 S4c-2: a retried pickup's near-field record by episode (SLAM hold) -- each with
    its start, tick rows, frames, events, stop records, corrections, revisions and protection,
    and the retry that ended it."""
    starts = nt.get("starts") or []
    retries = sorted(
        (e for e in near.get("events") or [] if e.get("event") == "retry_start"),
        key=lambda e: float(e["time_s"]),
    )
    # One more start than retries, or as many when the run ended inside its last retry.
    if len(starts) not in (len(retries), len(retries) + 1) or not starts:
        raise Unverified(f"{len(starts)} episode starts for {len(retries)} retries")
    views = []
    for k, st in enumerate(starts):
        try:
            e = st["episode"]
            t0 = float(require_shape("an episode's build time", st["built_s"], ()))
            t1 = float(retries[k]["time_s"]) if k < len(retries) else math.inf
        except (KeyError, TypeError) as exc:
            raise Unverified(f"episode start {k} is incomplete ({exc!r})") from exc
        if k < len(retries) and retries[k].get("episode") != e:
            raise Unverified(f"the retry at {t1} does not end episode {e}")
        rows = [r for r in near.get("ticks") or [] if r.get("episode") == e]
        if not rows or any(
            not t0 - TIME_TOL_S <= float(r["t"]) <= t1 + TIME_TOL_S for r in rows
        ):
            raise Unverified(f"episode {e}'s tick rows are missing or outside it")

        def inside(at, t0=t0, t1=t1):
            return t0 - TIME_TOL_S <= float(at) <= t1 + TIME_TOL_S

        events = [ev for ev in near.get("events") or [] if inside(ev["time_s"])]
        frames = [f for f in near.get("frames") or [] if f.get("episode") == e]
        for f in frames:
            if not inside(f["read_s"]):
                raise Unverified(f"a frame of episode {e} read outside it")
            if (
                "skipped" not in f
                and f.get("accepted")
                and f.get("latest_frame_version") != e
            ):
                raise Unverified(
                    f"an accepted frame of episode {e} holds another episode's observation"
                )
        view = {
            "ticks": rows,
            "frames": frames,
            "events": events,
            "stop_events": [
                x for x in near.get("stop_events") or [] if x.get("episode") == e
            ],
            "corrections": [
                c for c in near.get("corrections") or [] if c.get("episode") == e
            ],
            "path_revisions": [
                r for r in near.get("path_revisions") or [] if r.get("episode") == e
            ],
            "protect": next(
                (ev for ev in events if ev.get("event") == "protect"), None
            ),
        }
        views.append(
            {
                "episode": e,
                "start": st,
                "near": view,
                "retry": retries[k] if k < len(retries) else None,
            }
        )
    # Nothing outside the episodes: every frame and event belongs to one (Codex S4c-2
    # implementation review P2 -- a record between them must not vanish).
    episodes_ = {v["episode"] for v in views}
    if any(f.get("episode") not in episodes_ for f in near.get("frames") or []):
        raise Unverified("a frame outside every episode")
    placed = sum(len(v["near"]["events"]) for v in views)
    if placed != len(near.get("events") or []):
        raise Unverified("an event outside every episode")
    return views


def _span_checks(
    result: dict, truth: dict, geometry, i_from: int, i_to: int, what: str
) -> list[str]:
    """Over the ticks i_from..i_to (a retry or a realignment): no fork, heel or carriage box
    sharing volume with the pallet, and no unpermitted entry in a complete permission record
    (plan D8 S4c-2 (x), shared with S4c-3 (xi))."""
    t = np.asarray(truth["stamp_s"], dtype=float)
    failures = []
    base = np.asarray(truth["base_pose"], dtype=float)
    pallet = np.asarray(truth["pallet_pose"], dtype=float)
    lift = np.asarray(truth["lift_m"], dtype=float)
    boxes = tuple(geometry.fork_boxes) + tuple(geometry.carriage_boxes)
    if not all(np.all(np.isfinite(a[i_from : i_to + 1])) for a in (base, pallet, lift)):
        raise Unverified(f"non-finite truth over {what}")
    touching = [
        float(t[i])
        for i in range(i_from, i_to + 1)
        if geometry.forbidden_contacts(
            base[i, :3],
            base[i, 3:],
            float(lift[i]),
            pallet[i, :3],
            pallet[i, 3:],
            clearance_m=0.0,
            boxes=boxes,
        )
    ]
    if touching:
        failures.append(
            f"the truck touched the pallet during {what} ({len(touching)} ticks)"
        )
    layer = result.get("obstacle_layer")
    if (
        layer is None
        or "unpermitted" not in layer
        or "unpermitted_entries" not in layer
    ):
        raise Unverified(f"no permission record to judge {what} by")
    listed = layer["unpermitted"]
    total = require_count("the unpermitted entries", layer["unpermitted_entries"])
    # Every kept entry placed in time before any is filtered out (Codex S4c-2 implementation
    # review 2 P2: a missing or NaN time dropped the entry silently).
    times = [
        float(require_shape("an unpermitted entry's time", u.get("time_s"), ()))
        for u in listed
    ]
    t_end = float(t[i_to]) + TIME_TOL_S
    # The runner keeps the first 50 entries: complete over the retry only when nothing was cut
    # or the last kept one is after it (Codex S4c-2 implementation review P2).
    if total < len(listed) or (
        total > len(listed) and not (times and times[-1] > t_end)
    ):
        raise Unverified(
            f"the permission record is cut ({len(listed)} of {total}) before {what} ends"
        )
    unpermitted = [
        u
        for u, at_ in zip(listed, times, strict=True)
        if float(t[i_from]) - TIME_TOL_S <= at_ <= t_end
    ]
    if unpermitted:
        failures.append(f"{len(unpermitted)} unpermitted entries during {what}")
    return failures


def _release_checks(
    record: dict, result: dict, at: float, event: str, jump, what: str
) -> list[str]:
    """The release of a retry or a realignment as the plan asks: the hold record's release
    (event) at that time with the record's jump, the frame change the one between the
    recorded estimates and its turn the yaw jump, and the grid re-projected on T composed
    with the correction before (plan D8 S4c-2 (x), shared with S4c-3 (xi))."""
    failures = []
    layer = result.get("obstacle_layer") or {}
    # The release and the re-projection as the plan asks (Codex S4c-2 implementation review
    # P2): the hold record's release at the retry with the same jump, the frame change the
    # one between the recorded estimates, its turn the yaw jump, and the grid re-projected.
    from forklift_core.localization.slam_pose import compose, invert

    try:
        before = tuple(
            require_finite("the estimate before", record["rear_before"], size=3)
        )
        after = tuple(
            require_finite("the estimate after", record["rear_after"], size=3)
        )
        change = tuple(
            require_finite("the frame change", record["frame_change"], size=3)
        )
        holds = (result.get("slam_summary") or {})["holds"]
    except (KeyError, TypeError) as exc:
        raise Unverified(f"the release record is incomplete ({exc!r})") from exc
    releases = [h for h in holds if h.get("event") == event]
    if (
        len(releases) != 1
        or not _same(releases[0].get("time_s"), at, TIME_TOL_S)
        or not _same(
            [releases[0].get("jump_m"), releases[0].get("jump_rad")], list(jump), 0.0
        )
    ):
        failures.append(f"the SLAM record has no release at {what} with its jump")
    if not _same(change, compose(after, invert(before)), 1e-9) or not _same(
        abs(math.atan2(math.sin(change[2]), math.cos(change[2]))), float(jump[1]), 1e-9
    ):
        failures.append("the frame change is not the released estimates' change")
    # The grid re-projected with the released correction: applied_new = T o applied_old, as
    # every estimate moves by T (Codex S4c-2 implementation review 2 P2).
    reproj = record.get("reprojected")
    try:
        applied_before = require_finite(
            "the correction before the release", record["applied_before"], size=3
        )
    except (KeyError, TypeError) as exc:
        raise Unverified(f"the release record is incomplete ({exc!r})") from exc
    if not isinstance(reproj, dict):
        failures.append("the grid was not re-projected at the release")
    else:
        try:
            version = require_count("the grid version", reproj["version"])
            count = require_count("the re-projections", reproj["reprojections"])
            applied = require_finite("the grid's correction", reproj["applied"], size=3)
            run_count = require_count(
                "the run's re-projections", layer["reprojections"]
            )
        except (KeyError, TypeError) as exc:
            raise Unverified(
                f"the re-projection record is incomplete ({exc!r})"
            ) from exc
        if (
            version < 1
            or not 1 <= count <= run_count
            or not _same(applied, compose(change, tuple(applied_before)), 1e-9)
        ):
            failures.append(
                "the grid was not re-projected with the released correction"
            )
    return failures


def _reobservation_checks(
    record: dict,
    result: dict,
    truth: dict,
    i_from: int,
    attempt,
    seen_valid: bool,
    control,
    target_of,
    what: str,
    observation_numbers=None,
) -> tuple[list[str], dict]:
    """A retry's or a realignment's re-observation (plan D8 S4c-2 (x) v3, shared with S4c-3
    (xi)): a valid one as placed gives the target the planner used (target_of(observation,
    base, depth) is the target the rules expect), and it was taken in place and bound to the
    truth by its control sample -- see the comment below. Returns (failures, report)."""
    t = np.asarray(truth["stamp_s"], dtype=float)
    base = np.asarray(truth["base_pose"], dtype=float)
    attempts = result.get("observation_attempts") or []
    failures, report = [], {}
    # The re-observation as placed gives the target the planner used (a valid one only).
    if seen_valid:
        try:
            obs = record["observation"]
            cbase = tuple(
                require_finite(
                    "the re-observation base", record["capture_base"], size=3
                )
            )
            target = require_finite(
                "the re-observation target", record["target"], size=3
            )
            planned = require_finite(
                "the target planned", record["planned_target"], size=3
            )
            depth = float(result["geometry"]["pallet_depth_m"])
        except (KeyError, TypeError) as exc:
            raise Unverified(
                f"the re-observation record is incomplete ({exc!r})"
            ) from exc
        seen = attempts[attempt - 1].get("pocket_observation") or {}
        # observation_numbers(raw): the observation the rules expect from the attempt's raw one
        # (a mismatch injection biases it, plan D8 S4c-3 (xi)); None: the raw one itself.
        same_obs = (
            _same_observation(obs, seen, tol=0.0)
            if observation_numbers is None
            else _same_observation(obs, seen, observation_numbers(seen), tol=1e-9)
        )
        if (
            not same_obs
            or not _same(target, target_of(obs, cbase, depth))
            or not _same(planned, target)
        ):
            failures.append(
                f"the plan after {what} is not the re-observation's own target"
            )
    # In place (X_RULES v3, Codex S4c-3 stage-2 reviews): the re-observation's control sample
    # (capture_control: its end time, the control estimate and the truth then) binds the
    # capture to the truth record -- no command from the retry to the capture's end, the truck
    # still from the retry to the tick before the command first rises after it (the record's
    # end when it never does), the accepted truth pose and the sample's truth the truth at
    # that tick, the capture pose and the SLAM error the sample's, and, when the control record
    # has a row of that very instant, the sample's estimate that row. With no re-observation
    # the stand runs to the record's end. (No bound between the rows around the instant: the
    # odometry noise moves a standing estimate out and back between them -- Codex x-v3
    # review 4 -- and the capture has no control rows to bound it by.)
    off_x = abs(float((result.get("arguments") or {}).get("rear_axle_offset_m", -0.34)))
    applied_ = np.asarray(truth["applied_speed_mps"], dtype=float)
    cc = None
    if attempt is not None:
        try:
            t_cap0 = float(
                require_shape(
                    "the re-observation's start",
                    attempts[attempt - 1]["t_before_capture_s"],
                    (),
                )
            )
            cc = attempts[attempt - 1]["capture_control"]
            t_ce = float(require_shape("the re-observation's end", cc["time_s"], ()))
            cc_est = require_finite(
                "the re-observation's estimate", cc["control_rear"], size=3
            )
            cc_truth = require_finite(
                "the re-observation's truth", cc["truth_rear"], size=3
            )
        except (KeyError, TypeError) as exc:
            raise Unverified(
                f"the re-observation's control sample is not recorded ({exc!r})"
            ) from exc
        i_ce = tick_index(t, t_ce, "the re-observation's end")
        if not (t[i_from] - TIME_TOL_S <= t_cap0 <= t_ce + TIME_TOL_S):
            raise Unverified(
                f"the re-observation is not after {what} in the truth record"
            )
        # The sample's time is the capture's own end: the capture's elapsed physics time on
        # the same clock (Codex S4c-3 x-v3 review 3 P2-1: a sample dated inside the capture
        # cut the stand short).
        try:
            diag = attempts[attempt - 1]["capture_diagnostics"]
            elapsed = float(
                require_shape("the capture's end", diag["physics_time_after_s"], ())
            ) - float(
                require_shape("the capture's start", diag["capture_start_time_s"], ())
            )
        except (KeyError, TypeError) as exc:
            raise Unverified(f"the capture's times are not recorded ({exc!r})") from exc
        if abs(t_ce - (t_cap0 + elapsed)) > 1e-6:
            failures.append(
                "the re-observation's control sample is not at the capture's end"
            )
    else:
        i_ce = len(t) - 1
    if np.any(applied_[i_from : i_ce + 1] != 0.0):
        failures.append(
            f"a command was given between {what} and the re-observation's end"
        )
    moving = np.nonzero(applied_[i_ce + 1 :] != 0.0)[0]
    i_stand = i_ce + int(moving[0]) if len(moving) else len(t) - 1
    rear_s = rear_axle(base[i_from : i_stand + 1], off_x)
    yaws_s = np.unwrap([yaw_of(base[j]) for j in range(i_from, i_stand + 1)])
    travel_s = (
        float(np.sum(np.hypot(*np.diff(rear_s, axis=0).T))) if len(rear_s) > 1 else 0.0
    )
    turn_s = float(np.sum(np.abs(np.diff(yaws_s)))) if len(yaws_s) > 1 else 0.0
    if travel_s > STAND_M or turn_s > STAND_RAD:
        failures.append(
            f"the re-observation was not taken in place ({1e3 * travel_s:.2f} mm, "
            f"{1e3 * turn_s:.2f} mrad over the stand)"
        )
    report.update(stand_travel_m=travel_s, stand_turn_rad=turn_s)
    if cc is not None:
        try:
            accepted = attempts[attempt - 1]["accepted_pose"]
            pos = require_finite(
                "the re-observation's accepted position", accepted[0], size=3
            )
            quat = require_finite(
                "the re-observation's accepted orientation", accepted[1], size=4
            )
            err = attempts[attempt - 1]["slam_error"]
            err_v = require_finite(
                "the re-observation's SLAM error",
                [err["along_m"], err["lateral_m"], err["yaw_rad"]],
                size=3,
            )
        except (KeyError, TypeError, IndexError) as exc:
            raise Unverified(
                f"the re-observation's pose is not recorded ({exc!r})"
            ) from exc

        def wrapped(a):
            return math.atan2(math.sin(a), math.cos(a))

        truth_yaw = yaw_of(np.concatenate((pos, quat)))
        # The runner's rear axle: the base less the offset along the heading (yaw only -- a
        # pitched pose is not rotated, Codex S4c-3 x-v3 review 3 P2-3).
        tick_yaw = yaw_of(base[i_ce])
        tick_rear = base[i_ce, :2] - off_x * np.array(
            [math.cos(tick_yaw), math.sin(tick_yaw)]
        )
        if not (
            _same(pos[:2], base[i_ce, :2], 1e-6)
            and abs(wrapped(truth_yaw - yaw_of(base[i_ce]))) <= 1e-6
            and _same(cc_truth[:2], tick_rear, 1e-6)
            and abs(wrapped(cc_truth[2] - yaw_of(base[i_ce]))) <= 1e-6
        ):
            failures.append("the re-observation's truth is not the truth at its end")
        c_, s_ = math.cos(cc_truth[2]), math.sin(cc_truth[2])
        d_ = cc_est[:2] - cc_truth[:2]
        own_err = [
            c_ * d_[0] + s_ * d_[1],
            -s_ * d_[0] + c_ * d_[1],
            wrapped(cc_est[2] - cc_truth[2]),
        ]
        own_base = [
            cc_est[0] + off_x * math.cos(cc_est[2]),
            cc_est[1] + off_x * math.sin(cc_est[2]),
            cc_est[2],
        ]
        # 1e-6: the runner's error comes from the float32 truth quaternion by its own operation
        # order (the first real run differed by 1.8e-9 rad from the rebuilt value).
        if not _same(err_v, own_err, 1e-6) or (
            seen_valid and not _same(cbase, own_base, 1e-9)
        ):
            failures.append(
                "the re-observation's capture pose or SLAM error is not its control sample"
            )
        if control is None:
            raise Unverified(
                "no control record to bind the re-observation's estimate to"
            )
        rows = np.asarray(control, dtype=float)
        require_finite("the control record", rows)
        same_time = rows[np.abs(rows[:, 0] - t_ce) <= TIME_TOL_S]
        if len(same_time):
            # A control row of that very instant: the sample must be it (Codex S4c-3 x-v3
            # review 3 P2-2 -- a later correction widened the bracket).
            r0 = same_time[-1]
            if not (
                _same(cc_est[:2], r0[1:3], 1e-9)
                and abs(wrapped(cc_est[2] - r0[3])) <= 1e-9
            ):
                failures.append(
                    "the re-observation's estimate is not the control row of its instant"
                )
    return failures, report


def check_x(
    result: dict,
    near: dict,
    truth: dict,
    views: list[dict],
    geometry,
    control=None,
) -> dict:
    """(x), plan D8 S4c-2: exactly one retry, for a mismatch or a refused correction, ending
    an episode that never started its protection and stood at least RETRY_MIN_FACE_M from
    the estimated face; the release recorded; the re-observation valid and never biased; over
    the retry span (the retry tick to the next episode's first tick) no truck box -- blades,
    heels or carriage -- shares volume with the pallet and no unpermitted entry was
    recorded."""
    import near_tracking as NT

    t = np.asarray(truth["stamp_s"], dtype=float)
    failures, report = [], {"rules": X_RULES}
    if (
        not views
        or views[0]["retry"] is None
        or len(views) > 2
        or any(v["retry"] is not None for v in views[1:])
    ):
        failures.append(f"{len(views)} episodes, not one retry")
        return {"applicable": True, "failures": failures, "passed": False, **report}
    first = views[0]
    last = views[1] if len(views) == 2 else None  # None: the run ended inside the retry
    retry = first["retry"]
    record = (result.get("near_tracking") or {}).get("retry") or {}
    if last is None:
        failures.append(
            f"the retry never reached a new episode (the run ended {result.get('failure_reason')})"
        )
    try:
        reason = retry["reason"]
        at = float(require_shape("the retry time", retry["time_s"], ()))
        attempt = (
            require_count("the re-observation attempt", record["capture_attempt"])
            if last is not None or "capture_attempt" in record
            else None
        )
        jump = require_finite(
            "the release jump", (record["jump_m"], record["jump_rad"]), size=2
        )
        require_shape("the release frame change", record["frame_change"], (3,))
    except (KeyError, TypeError) as exc:
        raise Unverified(f"the retry record is incomplete ({exc!r})") from exc
    if reason not in NT.RETRY_REASONS or record.get("reason") != reason:
        failures.append(f"a retry for {reason}")
    rows = first["near"]["ticks"]
    distances = [
        float(require_shape("an episode distance", r.get("face_distance_m"), ()))
        for r in rows
    ]
    if first["near"]["protect"] is not None or min(distances) <= NT.PROTECT_W_M:
        failures.append("the retried episode had started its protection")
    i_retry = tick_index(t, at, "the retry")
    end_row = max(rows, key=lambda r: float(r["t"]))
    if tick_index(t, end_row["t"], "the episode's last tick") != i_retry:
        failures.append("the retry is not the episode's last tick")
    elif not float(end_row["face_distance_m"]) >= NT.RETRY_MIN_FACE_M:
        failures.append(f"retried at an estimate of {end_row['face_distance_m']:.3f} m")
    attempts = result.get("observation_attempts") or []
    seen_valid = False
    if attempt is None:
        failures.append("the run ended before the re-observation")
    else:
        if not 1 <= attempt <= len(attempts):
            raise Unverified(f"the re-observation attempt {attempt} is not recorded")
        seen_valid = (attempts[attempt - 1].get("pocket_observation") or {}).get(
            "status"
        ) == "valid" and record.get("capture_status") == "valid"
        if not seen_valid:
            failures.append("the re-observation did not see the pallet")
        inj = (result.get("near_tracking") or {}).get("injection") or {}
        if any(int(c.get("attempt", 0)) >= attempt for c in inj.get("captures") or []):
            failures.append("a capture at or after the re-observation was biased")
    # The retry span: to the next episode's first tick, or to the record's end.
    i_next = (
        tick_index(
            t,
            min(float(r["t"]) for r in last["near"]["ticks"]),
            "the next episode's first tick",
        )
        if last is not None
        else len(t) - 1
    )
    base = np.asarray(truth["base_pose"], dtype=float)
    failures += _span_checks(result, truth, geometry, i_retry, i_next, "the retry")
    failures += _release_checks(
        record, result, at, "release_pickup_retry", jump, "the retry"
    )
    reob_failures, reob_report = _reobservation_checks(
        record,
        result,
        truth,
        i_retry,
        attempt,
        seen_valid,
        control,
        lambda obs, cbase, depth: _site_from_observation(obs, cbase, depth),
        "the retry",
    )
    failures += reob_failures
    report.update(reob_report)
    rear = rear_axle(
        base, float((result.get("arguments") or {}).get("rear_axle_offset_m", -0.34))
    )
    report.update(
        reason=reason,
        retry_s=at,
        span_s=float(t[i_next] - t[i_retry]),
        travel_m=float(
            np.sum(np.hypot(*np.diff(rear[i_retry : i_next + 1], axis=0).T))
        ),
        jump_m=float(jump[0]),
        jump_rad=float(jump[1]),
        re_observation_attempt=attempt,
    )
    return {"applicable": True, "failures": failures, "passed": not failures, **report}


def judge_retried(
    truth,
    near,
    nt,
    result,
    raw,
    terms_bytes,
    geometry,
    pallet_geo,
    corners,
    blades,
    carriage_limit,
    end,
    data_root,
    directory,
    checks,
    provenance_extra,
) -> dict:
    """Plan D8 S4c-2 (x): a retried pickup judged by episode -- the first episode by its own
    section (a mismatch group's biased anchor by the (ix) mismatch contract and (vii); a
    refused correction by (i)-(v), (vii) and (viii) up to the refusal), the retry by (x), the
    last episode by (i)-(v), (vii) and (viii); (iii)'s cadence within the episodes, (v) on one
    anchor from the first near capture to the insertion end. Returns the combined (vii) and
    fills checks (the last episode's (i)-(v) and (viii), "episodes", "ix", "x")."""
    from forklift_core.perception.near_field_bounds import NearFieldBounds
    from tools import d8b_calibration as CAL

    sys.path.insert(0, str(ROOT / "sim" / "isaac"))
    from near_tracking import snapshot_relative

    args = result.get("arguments") or {}
    rear_offset = float(args["rear_axle_offset_m"])
    t = np.asarray(truth["stamp_s"], dtype=float)
    views = split_episodes(near, nt)
    bounds_bytes = (ROOT / snapshot_relative(str(nt.get("bounds")))).read_bytes()
    if sha256_bytes(bounds_bytes) != nt.get("bounds_sha256"):
        raise Unverified("the bounds file is not the one the run used")
    bounds_data = json.loads(bounds_bytes)
    bounds = NearFieldBounds.from_dict(bounds_data)
    cal_ref = bounds_data["provenance"]["calibration"]
    cal_path = data_root / cal_ref["path"]
    cal_bytes = cal_path.read_bytes() if cal_path.exists() else None
    if cal_bytes is None or sha256_bytes(cal_bytes) != cal_ref["sha256"]:
        raise Unverified("the calibration the bounds came from is missing or changed")
    calibration = json.loads(cal_bytes)
    provenance_extra.update(
        bounds_sha256=sha256_bytes(bounds_bytes),
        calibration={"path": str(cal_path), "sha256": cal_ref["sha256"]},
    )
    if "paths.json" not in raw:
        raise Unverified("no paths.json")
    paths_record = json.loads(raw["paths.json"])
    terms = json.loads(terms_bytes)
    run = CAL.Run.load(directory)
    kind = (nt.get("injection") or {}).get("kind")
    checked = checked_ticks(truth)
    episodes, stops, spans = [], [], []
    for k, view in enumerate(views):
        view_near, start = view["near"], view["start"]
        last_row = tick_index(
            t, max(float(r["t"]) for r in view_near["ticks"]), "an episode's last tick"
        )
        # (iii)'s span from the build: the tracker is built in the tick the approach begins and
        # that tick's frame is read after it, a tick before the first near-field row.
        spans.append((float(start["built_s"]), float(t[last_row])))
        ended = view["retry"]["reason"] if view["retry"] is not None else None
        biased = k == 0 and kind == "mismatch"
        out = {"episode": view["episode"], "ended": ended}
        armed = any(e.get("event") == "armed" for e in view_near["events"])
        if biased or not armed:
            # The biased anchor, or an episode that ended before its armed section: from its
            # first near tick, its approach before that standing (plan v4, Codex S4c-2
            # implementation review P2).
            begin = (
                None
                if k == 0
                else int(min(i for i in checked if i > episodes[-1]["last_row"]))
            )
            section = mismatch_section(
                truth, view_near, rear_offset, end_index=last_row, begin_index=begin
            )
        else:
            section = near_section(truth, view_near, end_index=last_row)
        out.update(section=[int(section[0]), int(section[1])], last_row=last_row)
        out["vii"] = check_vii(
            truth, view_near, terms, section, rear_offset, corners, blades
        )
        stops += out["vii"]["stops"]
        if not biased:
            states = accepted_states(view_near, start, bounds)
            out["i"] = check_i(
                run,
                view_near,
                states,
                bounds,
                pallet_geo,
                section,
                carriage_limit,
                -abs(rear_offset),
            )
            out["ii"] = check_ii(run, view_near, states, bounds, section)
            out["iv"] = check_iv(truth, states, section, bounds, rear_offset)
            out["viii"] = check_viii(
                truth,
                view_near,
                start,
                paths_record,
                states,
                bounds,
                rear_offset,
                ended=ended,
            )
        episodes.append(out)
    # (iii) applies with (i)-(v): a run whose only episode is the biased anchor (it ended
    # inside its retry, X_RULES v2) has none to apply it to, as an unretried mismatch run.
    checks["iii"] = (
        check_iii(
            run,
            bounds_data,
            calibration,
            CAL.PalletModel.from_geometry(pallet_geo),
            spans=spans,
        )
        if any("i" in e for e in episodes)
        else {"applicable": False}
    )
    # (v) on one anchor to the insertion end -- or, when the run ended inside its retry, to the
    # record's end, over the whole retry (Codex S4c-2 results review).
    realign_ = nt.get("realign")
    checks["v"] = check_v(
        truth,
        min(  # plan D8 S4c-3: the earliest anchor -- a realignment's refused capture first
            [float(views[0]["start"]["aligned_s"])]
            + ([float(realign_["time_s"])] if realign_ is not None else [])
        ),
        len(t) - 1
        if views[-1]["retry"] is not None
        else (end if end is not None else int(checked_ticks(truth)[-1])),
    )
    for key in ("i", "ii", "iv", "viii"):
        # The last episode's -- or, when the run ended inside its retry, the latest that has one.
        checks[key] = next(
            (e[key] for e in reversed(episodes) if key in e), {"applicable": False}
        )
    checks["episodes"] = {
        "applicable": True,
        "passed": all(
            all(
                e[key]["passed"] for key in ("vii", "i", "ii", "iv", "viii") if key in e
            )
            for e in episodes
        ),
        "list": episodes,
    }
    checks["ix"] = check_ix(
        result,
        views[0]["near"],
        truth,
        retry=views[0]["retry"],
        start=views[0]["start"],
    )
    control_path = Path(directory) / "slam_control.npy"
    checks["x"] = check_x(
        result,
        near,
        truth,
        views,
        geometry,
        control=np.load(control_path) if control_path.exists() else None,
    )
    return {
        "passed": all(e["vii"]["passed"] for e in episodes),
        "stops": stops,
        "episodes": len(episodes),
    }


REDRAW_RULES = "v1"  # plan D8 S4c-3 stage 1, fixed before the first stage-1 run


def mission_geometry(record: dict):
    """The run's SyntheticMissionGeometry from its result's geometry record."""
    from forklift_core.planning.geometry import Footprint
    from forklift_core.planning.pallet_mission import SyntheticMissionGeometry

    def footprint(d):
        return Footprint(d["front_m"], d["rear_m"], d["half_width_m"])

    geo = SyntheticMissionGeometry(
        unloaded_footprint=footprint(record["unloaded_footprint"]),
        **{
            k: record[k]
            for k in (
                "pallet_depth_m",
                "pallet_width_m",
                "axle_to_fork_tip_m",
                "approach_gap_m",
                "alignment_straight_m",
                "delivery_straight_m",
                "extraction_m",
                "withdrawal_m",
                "spawn_clearance_m",
                "carriage_limit_m",
                "insertion_reserve_m",
            )
        },
    )
    for k in ("prealign_offset_m", "approach_offset_m", "inserted_offset_m"):
        if not _same(getattr(geo, k), record[k], 1e-12):
            raise Unverified(f"the geometry record's {k} is not its own derivation")
    return geo


def check_near_redraw(result: dict, nt: dict) -> dict:
    """Plan D8 S4c-3 stage 1: the S4b re-draw at the near capture. A near capture accepted
    after its box or dry run refused must have an accepted re-draw entry, and only such a
    capture may (Codex S4c-3 implementation review P2: a deleted or relabelled entry must
    not pass); a refused entry installs nothing and starts no tracker. Each accepted entry:
    its record (the box 0.05 m / 0.12 rad, both dry runs at the baseline and driven cruise
    arrived within +6 mm, 6 mm and 15 mrad with their swept footprints clear, both
    correction points of both runs inside the box -- each point's pose the first within it
    by the correction trigger's own distance, recomputed with the pose before it) and its
    chain, bound to the tracker's start of that attempt: the start's pose and observation
    are the entry's, the observation placed at it is the raw target, the planning target
    that raw target (moved by the correction injection when this attempt was injected),
    the line its prealign and approach, the installed approach the straight from the
    recorded rear, every installed insert and extract pose on that target's straight with
    its direction and no curvature, the installed contents their digests and the
    followers' copies, and the tracker started on them (paths0)."""
    sys.path.insert(0, str(ROOT / "sim" / "isaac"))
    from dataclasses import asdict

    import near_tracking as NT

    from forklift_core.control.path_tracking import TrackerConfig
    from forklift_core.control.rollout import bicycle_rollout
    from forklift_core.perception.near_field_tracking import held_pockets
    from forklift_core.planning.geometry import Pose2D
    from forklift_core.planning.pallet_mission import (
        PalletSite,
        site_poses,
        straight_from_pose,
    )
    from tools.d8c_replay import observation_from_dict

    caps = list(result.get("near_capture_history") or [])
    if result.get("near_capture"):
        caps.append(result["near_capture"])
    # The refused captures of a realignment keep their refused entries (Codex S4c-3 stage-2
    # review P2-5: moving them must not hide them).
    caps += list(result.get("near_capture_refused") or [])
    entries = [
        e for c in caps for e in c.get("recoveries") or [] if e.get("stage") == "redraw"
    ]

    box_yaw = float(
        (result.get("arguments") or {}).get("near_capture_box_yaw_rad", 0.05)
    )
    if not 0.0 < box_yaw <= 0.05:
        raise Unverified(
            f"the near capture's heading box {box_yaw} is not in (0, 0.05]"
        )

    def refused_first(cap) -> bool:
        """The near capture's own acceptance refused it, with the run's heading box."""
        return _acceptance_refused(cap, box_yaw)

    done = [c for c in caps if c.get("status") == "done"]
    needed = {c.get("near_attempt") for c in done if refused_first(c)}
    if not entries and not needed:
        return {"applicable": False}
    failures, report = [], {"rules": REDRAW_RULES, "entries": []}
    args = result.get("arguments") or {}
    try:
        geo = mission_geometry(result["geometry"])
        depth = float(result["geometry"]["pallet_depth_m"])
        baseline = float(result["settings_synthetic"]["approach_speed_mps"])
        offset = abs(float(args["rear_axle_offset_m"]))
    except (KeyError, TypeError) as exc:
        raise Unverified(
            f"the run's geometry or speeds are not recorded ({exc!r})"
        ) from exc
    driven = args.get("approach_straight_speed_mps")
    driven = baseline if driven is None else float(driven)
    starts = nt.get("starts") or ([nt["start"]] if nt.get("start") else [])
    inj = nt.get("injection") or {}
    injected = {c.get("attempt") for c in inj.get("captures") or []}
    accepted_attempts = {e.get("attempt") for e in entries if e.get("accepted") is True}
    done_attempts = {c.get("near_attempt") for c in done}
    for attempt in needed - accepted_attempts:
        failures.append(
            f"the near capture at attempt {attempt} was refused yet has no accepted re-draw"
        )
    for attempt in accepted_attempts - needed:
        failures.append(
            f"the re-draw at attempt {attempt} ran for a capture its acceptance did not refuse"
        )
    for e in entries:
        what = f"the re-draw at attempt {e.get('attempt')}"
        try:
            rec = e["record"]
            accepted = e["accepted"]
            if accepted not in (True, False):
                raise TypeError(f"accepted {accepted!r}")
            off = rec["offsets"]
            off_v = require_finite(
                f"{what}: box offsets",
                [off["lateral_m"], off["yaw_rad"], off["length_left_m"]],
                size=3,
            )
            dries = rec["dry_runs"]
            points = e["correction_points"]
            speeds = require_finite(
                f"{what}: dry-run cruises", e["dry_run_cruise_mps"], size=2
            )
            base = tuple(
                require_finite(f"{what}: capture base", e["capture_base"], size=3)
            )
            rear = tuple(require_finite(f"{what}: rear", e["rear"], size=3))
            raw = require_finite(f"{what}: raw target", e["raw_target"], size=3)
            target = require_finite(f"{what}: target", e["target"], size=3)
            line = require_finite(f"{what}: line", e["line"], size=6).reshape(2, 3)
            tip_x = float(require_shape(f"{what}: fork tip x", e["fork_tip_x_m"], ()))
            obs = e["observation"]
        except (KeyError, TypeError, ValueError) as exc:
            raise Unverified(f"{what}: the record is incomplete ({exc!r})") from exc
        report["entries"].append(
            {
                "attempt": e.get("attempt"),
                "accepted": accepted,
                "refused": rec.get("refused"),
            }
        )
        start = next(
            (st for st in starts if st.get("attempt") == e.get("attempt")), None
        )
        if not accepted:
            # Nothing installed and no tracker started on this attempt.
            if (
                "installed" in e
                or start is not None
                or e.get("attempt") in done_attempts
            ):
                failures.append(
                    f"{what}: refused, yet it installed paths or started a tracker"
                )
            continue
        if (
            rec.get("refused") is not None
            or abs(off_v[0]) > NT.REDRAW_MAX_LATERAL_M
            or abs(off_v[1]) > NT.CORRECTION_MAX_YAW_RAD
            or off_v[2] < 0.3
        ):
            failures.append(f"{what}: accepted outside the box ({off})")
        if not _same(speeds, [baseline, driven], 1e-12) or len(dries) != 2:
            failures.append(
                f"{what}: dry runs {speeds} are not the baseline and driven cruise"
            )
        for d in dries:
            try:
                end_v = require_finite(
                    f"{what}: dry-run end",
                    [d["end_ahead_m"], d["end_lateral_m"], d["end_yaw_rad"]],
                    size=3,
                )
            except (KeyError, TypeError) as exc:
                raise Unverified(f"{what}: a dry run is incomplete ({exc!r})") from exc
            if not (
                d.get("status") == "arrived"
                and end_v[0] <= 0.006
                and abs(end_v[1]) <= 0.006
                and abs(end_v[2]) <= 0.015
                and d.get("swept_clear") is True
            ):
                failures.append(f"{what}: a dry run did not arrive clear ({d})")
        # The rear the straight starts from is the capture base's own rear axle (the runner
        # derives both from one pose at the capture -- Codex S4c-3 implementation review 2 P2).
        if not _same(
            rear,
            [
                base[0] - offset * math.cos(base[2]),
                base[1] - offset * math.sin(base[2]),
                base[2],
            ],
            1e-9,
        ):
            failures.append(f"{what}: the rear is not the capture base's rear axle")
        # The tracker's start of this attempt is this capture: the chain's input.
        if start is None:
            failures.append(f"{what}: no tracker started on this capture")
        elif not (
            _same(start.get("pose", [math.nan] * 3), base, 0.0)
            and _same_observation(start.get("observation") or {}, obs, tol=0.0)
            and _same(start.get("fork_tip_x_m", math.nan), tip_x, 0.0)
        ):
            failures.append(f"{what}: not the capture the tracker started from")
        held = held_pockets(observation_from_dict(obs), base)
        widths = (float(obs["left"]["width_m"]), float(obs["right"]["width_m"]))
        # The chain: observation -> raw target -> planning target -> line -> installed paths.
        if not _same(_site_from_observation(obs, base, depth), raw):
            failures.append(f"{what}: the raw target is not its observation's")
        want = raw
        if inj.get("kind") == "correction" and e.get("attempt") in injected:
            want = _biased_site(raw, float(inj["lateral_m"]), float(inj["yaw_rad"]))
        if not _same(target, want):
            failures.append(
                f"{what}: the target is not the raw target (with its injection)"
            )
        sites = site_poses(PalletSite(*(float(v) for v in target)), geo)

        def site(k, sites=sites):
            return [sites[k].x_m, sites[k].y_m, sites[k].yaw_rad]

        if not (_same(line[0], site("prealign")) and _same(line[1], site("approach"))):
            failures.append(
                f"{what}: the line is not the target's prealign and approach"
            )
        straight, _ = straight_from_pose(
            Pose2D(*rear),
            sites["prealign"],
            sites["approach"],
            max_lateral_m=NT.REDRAW_MAX_LATERAL_M,
            max_yaw_rad=NT.CORRECTION_MAX_YAW_RAD,
            min_length_m=0.3,
        )
        try:
            installed = e["installed"]
            arrays = {
                name: tuple(
                    np.asarray(installed[name][k], dtype=float)
                    for k in ("poses", "directions", "curvatures_inv_m")
                )
                for name in ("approach", "insert", "extract")
            }
            for name, (poses, dirs, curvs) in arrays.items():
                require_finite(f"{what}: installed {name}", poses)
                if (
                    poses.ndim != 2
                    or poses.shape[1] != 3
                    or len(dirs) != len(poses)
                    or len(curvs) != len(poses)
                ):
                    raise ValueError(f"installed {name} shapes")
        except (KeyError, TypeError, ValueError) as exc:
            raise Unverified(
                f"{what}: the installed paths are not recorded ({exc!r})"
            ) from exc
        if straight is None or not (
            arrays["approach"][0].shape == np.asarray(straight.poses).shape
            and np.allclose(arrays["approach"][0], straight.poses, rtol=0.0, atol=1e-9)
            and np.array_equal(
                arrays["approach"][1], np.asarray(straight.directions, dtype=float)
            )
            and np.array_equal(
                arrays["approach"][2],
                np.asarray(straight.curvatures_inv_m, dtype=float),
            )
        ):
            failures.append(
                f"{what}: the installed approach is not the straight from the recorded rear"
            )
        # Both dry runs rolled out again from the recorded rear on that straight with their
        # recorded settings: their ends and their correction-point crossings must be the
        # record's (Codex S4c-3 implementation review 3 P2: points taken from elsewhere).
        try:
            configs = [TrackerConfig(**c) for c in e["dry_run_configs"]]
            approach_cfg = dict(result["tracker_configs"]["approach"])
        except (KeyError, TypeError) as exc:
            raise Unverified(
                f"{what}: the dry runs' settings are not recorded ({exc!r})"
            ) from exc
        # The driven dry run is the run's own approach follower; the baseline one the same at
        # the baseline cruise (Codex S4c-3 implementation review 4 P2: settings changed in the
        # record only, e.g. curvature 0.40 for 0.36, passed the 1.0 m point at 46.6 mm).
        if len(configs) != 2 or not (
            asdict(configs[1]) == approach_cfg
            and asdict(configs[0]) == {**approach_cfg, "cruise_speed_mps": baseline}
        ):
            failures.append(
                f"{what}: the dry runs' settings are not the run's approach follower"
            )
        if (
            len(configs) != 2
            or not _same(
                [c.cruise_speed_mps for c in configs], [baseline, driven], 1e-12
            )
            or {k: v for k, v in asdict(configs[0]).items() if k != "cruise_speed_mps"}
            != {k: v for k, v in asdict(configs[1]).items() if k != "cruise_speed_mps"}
        ):
            failures.append(
                f"{what}: the dry runs' settings are not the baseline and driven cruise of one follower"
            )
        if straight is not None and len(configs) == 2 and len(dries) == 2:
            end = sites["approach"]
            u_end = np.array([math.cos(end.yaw_rad), math.sin(end.yaw_rad)])
            rollouts = [
                bicycle_rollout(
                    straight.poses,
                    straight.directions,
                    straight.curvatures_inv_m,
                    c,
                    np.asarray(rear, dtype=float),
                )
                for c in configs
            ]
            for d, ro in zip(dries, rollouts, strict=True):
                last = np.asarray(ro.trajectory[-1], dtype=float)
                rel = last[:2] - np.array([end.x_m, end.y_m])
                own = [
                    float(rel @ u_end),
                    float(rel @ np.array([-u_end[1], u_end[0]])),
                    math.atan2(
                        math.sin(last[2] - end.yaw_rad), math.cos(last[2] - end.yaw_rad)
                    ),
                ]
                if d.get("status") != ro.status or not _same(
                    [d["end_ahead_m"], d["end_lateral_m"], d["end_yaw_rad"]], own, 1e-9
                ):
                    failures.append(
                        f"{what}: a dry run's record is not its own roll-out"
                    )
            regenerated, inside = NT.correction_points_check(
                [ro.trajectory for ro in rollouts],
                sites["prealign"],
                NT.rollout_face_distance(held, widths, tip_x, offset),
            )

            def key(pt):
                return (int(pt.get("dry_run", -1)), float(pt.get("point_m", math.nan)))

            def numbers(pt):
                return [
                    *pt["pose"],
                    *pt["previous_pose"],
                    pt["face_distance_m"],
                    pt["previous_face_distance_m"],
                    pt["offsets"]["lateral_m"],
                    pt["offsets"]["yaw_rad"],
                ]

            try:
                for pt in points:  # a NaN in the record is unverified, not a mismatch
                    require_finite(
                        f"{what}: correction-point offsets", numbers(pt), size=10
                    )
                recorded = sorted(points, key=key)
                regenerated = sorted(regenerated, key=key)
                same = len(recorded) == len(regenerated) and all(
                    key(a) == key(b) and _same(numbers(a), numbers(b), 1e-9)
                    for a, b in zip(recorded, regenerated, strict=False)
                )
            except (KeyError, TypeError) as exc:
                same = False
                failures.append(f"{what}: a correction point is incomplete ({exc!r})")
            if not (same and inside):
                failures.append(
                    f"{what}: the correction points are not both roll-outs' crossings inside the box"
                )
        for name, a, b, direction in (
            ("insert", "approach", "inserted", 1.0),
            ("extract", "inserted", "extracted", -1.0),
        ):
            poses, dirs, curvs = arrays[name]
            start_, end_ = np.asarray(site(a)), np.asarray(site(b))
            axis = end_[:2] - start_[:2]
            length = float(np.hypot(*axis))
            u = axis / length if length > 0 else np.array([math.nan, math.nan])
            rel = poses[:, :2] - start_[:2]
            along = rel @ u
            across = rel @ np.array([-u[1], u[0]])
            heading = np.abs(
                np.arctan2(
                    np.sin(poses[:, 2] - start_[2]), np.cos(poses[:, 2] - start_[2])
                )
            )
            if not (
                len(poses) >= 2
                and _same(poses[0], start_)
                and _same(poses[-1], end_)
                and np.all(np.abs(across) <= 1e-9)
                and np.all(heading <= 1e-9)
                and np.all(np.diff(along) >= -1e-12)
                and np.all(dirs == direction)
                and np.all(curvs == 0.0)
            ):
                failures.append(
                    f"{what}: the installed {name} is not the target's straight"
                )
        for name, (poses, dirs, curvs) in arrays.items():
            digest = NT.path_digest(poses, dirs, curvs)
            if (
                not digest
                == installed[name].get("sha256")
                == installed[name].get("tracker_sha256")
            ):
                failures.append(f"{what}: the installed {name} and its follower differ")
        if start is not None and any(
            (start.get("paths0") or {}).get(name)
            != installed[name].get("tracker_sha256")
            for name in ("approach", "insert", "extract")
        ):
            failures.append(f"{what}: the tracker did not start on the installed paths")
    return {"applicable": True, "failures": failures, "passed": not failures, **report}


XI_RULES = "v1"  # plan D8 S4c-3 stage 2 (xi), fixed before the first stage-2 run


def check_xi(result: dict, nt: dict, truth: dict, geometry, control) -> dict:
    """(xi), plan D8 S4c-3 stage 2: the realignment -- once per pickup; its refused capture
    refused by its own acceptance and (unless the test-only --near-capture-redraw off) by
    stage 1; the release and re-projection; the re-observation valid, in place and bound to
    the truth, its target the planned one (a correction injection on that attempt moves it);
    every observe path installed during it recorded, under 6.0 m and installed while SLAM
    tracked (no held recovery); the re-capture accepted with at least 1.5 m of straight; over
    its span (the realignment's tick to the re-capture, or the record's end) no truck box in
    the pallet and no unpermitted entry. An open realignment (no re-capture) fails."""
    sys.path.insert(0, str(ROOT / "sim" / "isaac"))
    import near_tracking as NT

    rec = nt.get("realign")
    if rec is None:
        if nt.get("realigns") or result.get("near_capture_refused"):
            raise Unverified(
                "a realignment counted or a capture refused without its record"
            )
        return {"applicable": False}
    t = np.asarray(truth["stamp_s"], dtype=float)
    failures, report = [], {"rules": XI_RULES}
    args = result.get("arguments") or {}
    try:
        at = float(require_shape("the realignment's time", rec["time_s"], ()))
        refused_attempt = require_count(
            "the refused capture's attempt", rec["refused_attempt"]
        )
        jump = require_finite(
            "the realignment's release jump", (rec["jump_m"], rec["jump_rad"]), size=2
        )
        installs = list(rec["installs"])
        end = rec["end"]
        realigns = require_count("the realignments", nt["realigns"])
    except (KeyError, TypeError) as exc:
        raise Unverified(f"the realignment record is incomplete ({exc!r})") from exc
    if realigns != 1:
        failures.append(f"{realigns} realignments in one pickup")
    refused = list(result.get("near_capture_refused") or [])
    caps = [c for c in refused if c.get("refused_attempt") == refused_attempt]
    if len(refused) != 1 or len(caps) != 1:
        failures.append("the refused capture is not the realignment's")
    else:
        cap = caps[0]
        box_yaw = float(args.get("near_capture_box_yaw_rad", 0.05))
        if not _acceptance_refused(cap, box_yaw):
            failures.append(
                "the realignment's capture was not refused by its own acceptance"
            )
        stage1 = [e for e in cap.get("recoveries") or [] if e.get("stage") == "redraw"]
        if args.get("near_capture_redraw", "on") == "on":
            if not (
                len(stage1) == 1
                and stage1[0].get("attempt") == refused_attempt
                and stage1[0].get("accepted") is False
            ):
                failures.append("the realignment's capture has no stage-1 refusal")
        elif stage1:
            failures.append("stage 1 ran in a --near-capture-redraw off run")
    i_from = tick_index(t, at, "the realignment")
    # Its start is the refused capture's own tick (Codex S4c-3 stage-2 implementation review 2
    # P2-1: a start moved later would leave the span's first ticks unjudged).
    attempts_ = result.get("observation_attempts") or []
    if not 1 <= refused_attempt <= len(attempts_):
        raise Unverified(
            f"the refused capture's attempt {refused_attempt} is not recorded"
        )
    t_refused = float(
        require_shape(
            "the refused capture's attempt time",
            attempts_[refused_attempt - 1].get("t_before_capture_s"),
            (),
        )
    )
    if not _same(t_refused, at, 1e-9):
        failures.append("the realignment did not start at its refused capture")
    recaptured = isinstance(end, dict) and end.get("reason") == "recaptured"
    if recaptured:
        try:
            t_end = float(require_shape("the re-capture's time", end["time_s"], ()))
            left = float(
                require_shape("the re-capture's straight", end["length_left_m"], ())
            )
            end_attempt = require_count("the re-capture's attempt", end["attempt"])
        except (KeyError, TypeError) as exc:
            raise Unverified(f"the re-capture record is incomplete ({exc!r})") from exc
        i_to = tick_index(t, t_end, "the re-capture")
        if left < NT.REALIGN_MIN_LEFT_M:
            failures.append(f"the re-capture kept {left:.3f} m of straight")
        done = list(result.get("near_capture_history") or []) + (
            [result["near_capture"]] if result.get("near_capture") else []
        )
        accepted_cap = next(
            (
                c
                for c in done
                if c.get("status") == "done" and c.get("near_attempt") == end_attempt
            ),
            None,
        )
        if accepted_cap is None or not 1 <= end_attempt <= len(attempts_):
            failures.append("the re-capture is not an accepted near capture")
        else:
            # Its time is its attempt's, after the refusal and the re-observation, and the tracker
            # started on it then (Codex S4c-3 stage-2 implementation review P2-3, and review 2
            # P2-1: the tracker's own times too, so the end cannot be moved before them).
            t_att = float(
                require_shape(
                    "the re-capture's attempt time",
                    attempts_[end_attempt - 1].get("t_before_capture_s"),
                    (),
                )
            )
            reobs = rec.get("capture_attempt")
            starts_ = nt.get("starts") or ([nt["start"]] if nt.get("start") else [])
            start_ = next(
                (st for st in starts_ if st.get("attempt") == end_attempt), None
            )
            if (
                not _same(t_att, t_end, 1e-9)
                or not (reobs is not None and refused_attempt < reobs < end_attempt)
                or start_ is None
                or not _same(start_.get("built_s", math.nan), t_end, 1e-9)
                or not _same(start_.get("aligned_s", math.nan), t_att, 1e-9)
            ):
                failures.append(
                    "the re-capture is not its attempt's, after the re-observation, with its tracker"
                )
            # Its straight: what the accepting rule recorded -- stage 1's offsets when the re-draw
            # accepted it, else the capture's own (P2-4: the end record alone could hide 0.4 m).
            redraws = [
                e
                for e in accepted_cap.get("recoveries") or []
                if e.get("stage") == "redraw" and e.get("accepted") is True
            ]
            off_used = (
                (redraws[-1].get("record") or {}).get("offsets")
                if redraws
                else accepted_cap.get("offsets_to_new_line")
            ) or {}
            used_left = float(
                require_shape(
                    "the accepted straight's length", off_used.get("length_left_m"), ()
                )
            )
            if not _same(used_left, left, 1e-6) or used_left < NT.REALIGN_MIN_LEFT_M:
                failures.append(
                    f"the re-capture's straight is {used_left:.3f} m by its acceptance, recorded {left:.3f} m"
                )
            if start_ is not None:
                # And recomputed (review 2 P2-2): the straight from the capture's rear axle to the
                # line of the target its tracker's start observation gives (moved by a correction
                # injection on that attempt; a mismatch one already moved the observation). Only
                # the offsets compare: the runner draws the line from its planned approach, the
                # same line to rounding, so the arrays' digest cannot.
                from forklift_core.planning.geometry import Pose2D
                from forklift_core.planning.pallet_mission import (
                    PalletSite,
                    site_poses,
                    straight_from_pose,
                )

                try:
                    geo = mission_geometry(result["geometry"])
                    depth = float(result["geometry"]["pallet_depth_m"])
                    offset = abs(float(args["rear_axle_offset_m"]))
                    base = tuple(
                        require_finite(
                            "the re-capture's start pose", start_["pose"], size=3
                        )
                    )
                    obs_ = start_["observation"]
                    target_ = _site_from_observation(obs_, base, depth)
                except (KeyError, TypeError, ValueError) as exc:
                    raise Unverified(
                        f"the re-capture's start or geometry is not recorded ({exc!r})"
                    ) from exc
                inj_ = nt.get("injection") or {}
                if inj_.get("kind") == "correction" and end_attempt in {
                    c.get("attempt") for c in inj_.get("captures") or []
                }:
                    target_ = _biased_site(
                        target_, float(inj_["lateral_m"]), float(inj_["yaw_rad"])
                    )
                sites_ = site_poses(PalletSite(*(float(v) for v in target_)), geo)
                rear_ = Pose2D(
                    base[0] - offset * math.cos(base[2]),
                    base[1] - offset * math.sin(base[2]),
                    base[2],
                )
                _, off_re = straight_from_pose(
                    rear_,
                    sites_["prealign"],
                    sites_["approach"],
                    max_lateral_m=NT.REDRAW_MAX_LATERAL_M,
                    max_yaw_rad=NT.CORRECTION_MAX_YAW_RAD,
                    min_length_m=0.3,
                )
                recomputed = [
                    off_re[k] for k in ("lateral_m", "yaw_rad", "length_left_m")
                ]
                recorded = [
                    off_used.get(k, math.nan)
                    for k in ("lateral_m", "yaw_rad", "length_left_m")
                ]
                report["re_capture_recomputed"] = recomputed
                if not _same(recomputed, recorded, 1e-6):
                    failures.append(
                        f"the re-capture's straight recomputed from its capture is {recomputed}, "
                        f"recorded {recorded}"
                    )
    else:
        i_to = len(t) - 1
        failures.append(
            f"the realignment never re-captured (the run ended {result.get('failure_reason')})"
        )
    if i_to < i_from:
        raise Unverified("the re-capture is before the realignment")
    failures += _span_checks(result, truth, geometry, i_from, i_to, "the realignment")
    failures += _release_checks(
        rec, result, at, "release_near_realign", jump, "the realignment"
    )
    attempts = result.get("observation_attempts") or []
    attempt = rec.get("capture_attempt")
    seen_valid = False
    if attempt is None:
        failures.append("the run ended before the realignment's re-observation")
    else:
        attempt = require_count("the re-observation attempt", attempt)
        if not 1 <= attempt <= len(attempts):
            raise Unverified(f"the re-observation attempt {attempt} is not recorded")
        seen_valid = (attempts[attempt - 1].get("pocket_observation") or {}).get(
            "status"
        ) == "valid" and rec.get("capture_status") == "valid"
        if not seen_valid:
            failures.append("the realignment's re-observation did not see the pallet")
    inj = nt.get("injection") or {}
    injected = {c.get("attempt") for c in inj.get("captures") or []}

    def target_of(obs, cbase, depth):
        raw = _site_from_observation(obs, cbase, depth)
        if inj.get("kind") == "correction" and attempt in injected:
            return _biased_site(raw, float(inj["lateral_m"]), float(inj["yaw_rad"]))
        return raw

    def observation_numbers(raw):
        from forklift_core.perception.pocket_observation import (
            Pocket,
            PocketObservation,
        )

        pockets = [
            Pocket(tuple(raw[k]["center_m"]), raw[k]["width_m"], raw[k]["height_m"])
            for k in ("left", "right")
        ]
        b = NT.biased_observation(
            PocketObservation(
                raw["stamp_ns"],
                raw["clock_domain"],
                raw["frame_id"],
                raw["source_provenance"],
                raw["status"],
                *pockets,
                raw["insertion_yaw_rad"],
                raw["position_sigma_m"],
                raw["yaw_sigma_rad"],
                raw["reason"],
            ),
            float(result["geometry"]["pallet_depth_m"]),
            float(inj["lateral_m"]),
            float(inj["yaw_rad"]),
        )
        return np.array(
            [
                *b.left.center_m,
                b.left.width_m,
                b.left.height_m,
                *b.right.center_m,
                b.right.width_m,
                b.right.height_m,
                b.insertion_yaw_rad,
            ]
        )

    biased_obs = inj.get("kind") == "mismatch" and attempt in injected
    reob_failures, reob_report = _reobservation_checks(
        rec,
        result,
        truth,
        i_from,
        attempt,
        seen_valid,
        control,
        target_of,
        "the realignment",
        observation_numbers if biased_obs else None,
    )
    failures += reob_failures
    report.update(reob_report)
    # Every observe path installed during the realignment.
    if not installs or installs[0].get("reason") != "stand":
        failures.append("the realignment's installations do not start with its stand")
    if [ins.get("n") for ins in installs] != list(range(len(installs))):
        failures.append("the realignment's installations are not numbered in order")
    lengths = []
    for ins in installs:
        try:
            poses = require_finite("an installed path", ins["poses"])
            dirs = np.asarray(ins["directions"], dtype=float)
            curvs = np.asarray(ins["curvatures_inv_m"], dtype=float)
            t_ins = float(require_shape("an installation's time", ins["time_s"], ()))
            if (
                poses.ndim != 2
                or poses.shape[1] != 3
                or len(dirs) != len(poses)
                or len(curvs) != len(poses)
            ):
                raise ValueError("installed path shapes")
        except (KeyError, TypeError, ValueError) as exc:
            raise Unverified(f"an installation is incomplete ({exc!r})") from exc
        length = (
            float(np.sum(np.hypot(*np.diff(poses[:, :2], axis=0).T)))
            if len(poses) > 1
            else 0.0
        )
        lengths.append(length)
        digest = NT.path_digest(poses, dirs, curvs)
        if not (
            length <= NT.REALIGN_MAX_LENGTH_M + 1e-9
            and _same(length, ins.get("length_m", math.nan), 1e-9)
            and digest == ins.get("sha256") == ins.get("tracker_sha256")
            and ins.get("slam_mode") == "tracking"
            and t[i_from] - TIME_TOL_S <= t_ins <= t[i_to] + TIME_TOL_S
        ):
            failures.append(
                f"installation {ins.get('n')} ({ins.get('reason')}) is too long, held, "
                "outside the realignment or not its content"
            )
    # Complete (P2-2): every observe replan, obstacle replan or backoff in the observe phase and
    # candidate move recorded in the span has its installation, one near leg after the
    # re-observation, and the last installation is the follower the re-capture came by.
    t_lo, t_hi = float(t[i_from]) - TIME_TOL_S, float(t[i_to]) + TIME_TOL_S
    reasons = [ins.get("reason") for ins in installs]

    def in_span(r, key, what):
        # A missing or non-finite time is unverified, not out of the span (review 2 P2-3).
        return t_lo <= float(require_shape(what, r.get(key), ())) <= t_hi

    observe_replans = [
        r
        for r in result.get("observe_replans") or []
        if in_span(r, "time_s", "an observe replan's time")
    ]
    obstacle_observe = [
        r
        for r in (result.get("obstacle_layer") or {}).get("replans") or []
        if r.get("phase") == "observe"
        and in_span(r, "time_s", "an observe-phase obstacle replan's time")
    ]
    reobs_attempt = rec.get("capture_attempt")
    invalid_after = [
        k + 1
        for k, a in enumerate(result.get("observation_attempts") or [])
        if reobs_attempt is not None
        and k + 1 > reobs_attempt
        and in_span(a, "t_before_capture_s", "an observation attempt's time")
        and (a.get("pocket_observation") or {}).get("status") != "valid"
    ]
    want_counts = {
        "observe_replan": len(observe_replans),
        "obstacle_replan": sum(
            1
            for r in obstacle_observe
            if not r.get("failed") and not r.get("same_path")
        ),
        "backoff": sum(1 for r in obstacle_observe if r.get("backoff")),
        "observe_candidate": len(invalid_after),
    }
    for reason_, want_ in want_counts.items():
        if reasons.count(reason_) != want_:
            failures.append(
                f"{reasons.count(reason_)} {reason_} installations for {want_} recorded"
            )
    if attempt is not None and seen_valid and reasons.count("near_leg") != 1:
        failures.append(
            f"{reasons.count('near_leg')} near legs after the re-observation"
        )
    if recaptured:
        if "observe_tracker_sha256" not in end:
            raise Unverified("the re-capture does not record the follower it came by")
        if (
            not installs
            or installs[-1].get("tracker_sha256") != end["observe_tracker_sha256"]
        ):
            failures.append(
                "the last installation is not the follower the re-capture came by"
            )
    if str(result.get("failure_reason") or "").startswith(
        "near_capture_realign_held_recovery"
    ):
        failures.append("a recovery was needed after the realignment's hold")
    report.update(
        realign_s=at,
        span_s=float(t[i_to] - t[i_from]),
        installs=[ins.get("reason") for ins in installs],
        longest_install_m=max(lengths) if lengths else None,
        jump_m=float(jump[0]),
        jump_rad=float(jump[1]),
        re_observation_attempt=attempt,
    )
    return {"applicable": True, "failures": failures, "passed": not failures, **report}


def _acceptance_refused(cap: dict, box_yaw: float) -> bool:
    """A near capture's own acceptance refused it: its recorded offsets outside the run's
    box (0.05 m, box_yaw, 0.3 m -- recorded as the capture's box) or its dry run failing."""
    try:
        off = cap["offsets_to_new_line"]
        lat, yaw, left = (
            float(require_shape(f"near capture {k}", off[k], ()))
            for k in ("lateral_m", "yaw_rad", "length_left_m")
        )
        box = cap.get("box", {"lateral_m": 0.05, "yaw_rad": 0.05, "length_m": 0.3})
    except (KeyError, TypeError) as exc:
        raise Unverified(
            f"a near capture's offsets are not recorded ({exc!r})"
        ) from exc
    if box != {"lateral_m": 0.05, "yaw_rad": box_yaw, "length_m": 0.3}:
        raise Unverified(f"a near capture's box {box} is not the run's")
    if abs(lat) > 0.05 or abs(yaw) > box_yaw or left < 0.3:
        return True
    dry = cap.get("dry_run")
    if dry is None:
        raise Unverified("a near capture inside the box has no dry run")
    return not (
        dry.get("status") == "arrived"
        and float(require_shape("dry run error", dry["position_error_m"], ())) <= 0.006
        and abs(float(require_shape("dry run yaw", dry["yaw_error_rad"], ()))) <= 0.015
    )


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def loaded_sources() -> dict:
    """Every repository source loaded by this judgement, by content."""
    out = {}
    for module in list(sys.modules.values()):
        path = getattr(module, "__file__", None)
        if not path:
            continue
        p = Path(path).resolve()
        if p.suffix == ".py" and ROOT in p.parents:
            out[p.relative_to(ROOT).as_posix()] = sha256_bytes(p.read_bytes())
    return dict(sorted(out.items()))


def run_files_digest(directory: Path) -> str:
    """One hash over every run file the judgement reads (the depth frames included), taken
    before and after: the CPU tools (i)-(iii) reuse read them on their own."""
    names = [
        "result.json",
        "near_tracking.json",
        "paths.json",
        "slam_control.npy",
        "pocket_frames/index.json",
        "pocket_frames/truth.npz",
    ]
    names += sorted(
        p.relative_to(directory).as_posix()
        for p in (directory / "pocket_frames").glob("frame_*.npz")
    )
    digest = hashlib.sha256()
    for name in names:
        path = directory / name
        digest.update(
            name.encode()
            + b"\0"
            + (sha256_bytes(path.read_bytes()) if path.exists() else "-").encode()
        )
    return digest.hexdigest()


def judge_run(directory: Path, data_root: Path = ROOT) -> dict:
    sys.path.insert(0, str(ROOT / "sim" / "isaac"))
    import insertion_geometry as IG
    from near_tracking import snapshot_relative

    from forklift_core.perception.near_field_bounds import NearFieldBounds
    from forklift_core.perception.pallet_geometry import load_pallet_geometry
    from tools import d8b_calibration as CAL

    before = run_files_digest(directory)

    raw = {
        name: (directory / name).read_bytes()
        for name in (
            "result.json",
            "pocket_frames/truth.npz",
            "near_tracking.json",
            "paths.json",
        )
        if (directory / name).exists()
    }
    if "result.json" not in raw or "pocket_frames/truth.npz" not in raw:
        raise SystemExit(f"{directory}: result.json or the truth record is missing")
    result = json.loads(raw["result.json"])
    args = result.get("arguments") or {}
    nt = result.get("near_tracking") or {}
    models = {}
    for key, sha_key in (
        ("forklift_urdf", "forklift_urdf_sha256"),
        ("pallet_urdf", "pallet_urdf_sha256"),
        ("pallet_geometry", "pallet_geometry_sha256"),
    ):
        data_ = (ROOT / args[key]).read_bytes()
        if sha256_bytes(data_) != result.get(sha_key):
            raise SystemExit(f"{directory}: {key} is not the file the run used")
        models[key] = data_
    problems, vi, vii = [], None, None
    checks, provenance_extra = {}, {}
    data = np.load(io.BytesIO(raw["pocket_frames/truth.npz"]))
    truth = {k: data[k] for k in data.files}
    recorded = (result.get("pocket_recording") or {}).get("ticks")
    if recorded != len(truth.get("stamp_s", [])):
        problems.append(
            f"the truth record holds {len(truth.get('stamp_s', []))} ticks, the run recorded {recorded}"
        )
    near = (
        json.loads(raw["near_tracking.json"]) if "near_tracking.json" in raw else None
    )
    if near is None:
        problems.append("no near_tracking.json")
    retried = near is not None and any(
        e.get("event") == "retry_start" for e in near.get("events") or []
    )
    retry_spans, open_retry = [], False
    if retried and len(truth.get("stamp_s", [])):
        # Plan D8 S4c-2: the spans between a retried pickup's episodes may hold other phases.
        t_all = np.asarray(truth["stamp_s"], dtype=float)
        try:
            views_ = split_episodes(near, nt)
            for a_, b_ in zip(views_, views_[1:], strict=False):
                retry_spans.append(
                    (
                        tick_index(t_all, a_["retry"]["time_s"], "a retry"),
                        tick_index(
                            t_all,
                            min(float(r["t"]) for r in b_["near"]["ticks"]),
                            "an episode start",
                        ),
                    )
                )
            open_retry = views_[-1]["retry"] is not None
        except Unverified as exc:
            problems.append(str(exc))
    realign_ = nt.get("realign")
    realign_span = None
    if realign_ is not None and len(truth.get("stamp_s", [])):
        # Plan D8 S4c-3 stage 2: the realignment's ticks are checked contiguous and finite too.
        t_all = np.asarray(truth["stamp_s"], dtype=float)
        try:
            end_ = realign_.get("end")
            realign_span = (
                tick_index(t_all, realign_["time_s"], "the realignment"),
                tick_index(t_all, end_["time_s"], "the re-capture")
                if isinstance(end_, dict) and end_.get("reason") == "recaptured"
                else len(t_all) - 1,
            )
        except (Unverified, KeyError, TypeError) as exc:
            problems.append(f"the realignment's span: {exc}")
    problems += record_problems(truth, retry_spans, open_retry, realign_span)
    open_realign = realign_ is not None and not (nt.get("starts") or nt.get("start"))
    terms_bytes = (
        ROOT / str(nt.get("drive_terms") or "config/near_field_drive_terms.json")
    ).read_bytes()
    terms_sha = sha256_bytes(terms_bytes)
    if terms_sha != nt.get("drive_terms_sha256"):
        problems.append("the drive terms are not the file the run used")
    if not problems:
        # The models are loaded from the very bytes that were hashed.
        with tempfile.TemporaryDirectory() as tmp:
            paths = {}
            for key, data_ in models.items():
                paths[key] = Path(tmp) / Path(args[key]).name
                paths[key].write_bytes(data_)
            geometry = IG.InsertionGeometry.from_urdfs(
                paths["forklift_urdf"], paths["pallet_urdf"]
            )
            pallet_geo = load_pallet_geometry(paths["pallet_geometry"])
            corners = IG.read_carriage_front_corners_m(paths["forklift_urdf"])
            blades = IG.read_fork_blades_m(paths["forklift_urdf"])
            carriage_limit = float(IG.read_carriage_limit_m(paths["forklift_urdf"]))
        try:
            end = insert_end_index(truth, result)
            vi = check_vi(truth, geometry, pallet_geo.overall_depth_m, end)
            checks["redraw"] = check_near_redraw(result, nt)  # plan D8 S4c-3 stage 1
            control_path = Path(directory) / "slam_control.npy"
            checks["xi"] = check_xi(  # plan D8 S4c-3 stage 2
                result,
                nt,
                truth,
                geometry,
                np.load(control_path) if control_path.exists() else None,
            )
            if open_realign:
                # Plan D8 S4c-3 stage 2: the run ended in its realignment before any tracker --
                # the injection's transforms, the pallet still to the record's end, and (xi).
                checks["ix"] = check_ix(result, near or {}, truth)
                checks["v"] = check_v(
                    truth, float(realign_["time_s"]), len(truth["stamp_s"]) - 1
                )
                vii = {"applicable": False, "passed": True, "stops": []}
                raise _SkipEpisode
            if retried:
                vii = judge_retried(
                    truth,
                    near,
                    nt,
                    result,
                    raw,
                    terms_bytes,
                    geometry,
                    pallet_geo,
                    corners,
                    blades,
                    carriage_limit,
                    end,
                    data_root,
                    directory,
                    checks,
                    provenance_extra,
                )
                raise _SkipEpisode
            early = (nt.get("injection") or {}).get("kind") == "mismatch"
            if early:
                # Plan D8 S4c-1 (ix): a mismatch run's biased anchor episode -- its stops from
                # the first real near-field tick, before any armed section; (i)-(v) and (viii)
                # do not apply to an anchor that is wrong by design.
                section_ = mismatch_section(
                    truth, near, float(args["rear_axle_offset_m"])
                )
            else:
                section_ = near_section(truth, near)
            vii = check_vii(
                truth,
                near,
                json.loads(terms_bytes),
                section_,
                float(args["rear_axle_offset_m"]),
                corners,
                blades,
            )
            checks["ix"] = check_ix(result, near, truth)
            # (i)-(v): the measured bounds and the calibration they came from, by content.
            bounds_bytes = (
                ROOT / snapshot_relative(str(nt.get("bounds")))
            ).read_bytes()
            if sha256_bytes(bounds_bytes) != nt.get("bounds_sha256"):
                raise Unverified("the bounds file is not the one the run used")
            bounds_data = json.loads(bounds_bytes)
            bounds = NearFieldBounds.from_dict(bounds_data)
            cal_ref = bounds_data["provenance"]["calibration"]
            cal_path = data_root / cal_ref["path"]  # relative paths: from the data root
            cal_bytes = cal_path.read_bytes() if cal_path.exists() else None
            if cal_bytes is None or sha256_bytes(cal_bytes) != cal_ref["sha256"]:
                raise Unverified(
                    "the calibration the bounds came from is missing or changed"
                )
            calibration = json.loads(
                cal_bytes
            )  # parsed from the bytes that were hashed
            provenance_extra.update(
                bounds_sha256=sha256_bytes(bounds_bytes),
                calibration={"path": str(cal_path), "sha256": cal_ref["sha256"]},
            )
            if early:
                if realign_ is not None:
                    # Plan D8 S4c-3: the pallet still from the realignment on (P2-6), the biased
                    # anchor's own (v) still not applied.
                    checks["v"] = check_v(
                        truth,
                        float(realign_["time_s"]),
                        end if end is not None else int(checked_ticks(truth)[-1]),
                    )
                raise _SkipEpisode  # after the bounds' provenance (Codex S4c-1 review P2)
            if nt.get("injection") is not None and "corrections" not in near:
                raise Unverified("an S4c run without its correction records")
            run = CAL.Run.load(directory)
            section = near_section(truth, near)
            states = accepted_states(near, nt["start"], bounds)
            rear_offset = float(args["rear_axle_offset_m"])
            checks["i"] = check_i(
                run,
                near,
                states,
                bounds,
                pallet_geo,
                section,
                carriage_limit,
                -abs(rear_offset),
            )
            checks["ii"] = check_ii(run, near, states, bounds, section)
            checks["iii"] = check_iii(
                run, bounds_data, calibration, CAL.PalletModel.from_geometry(pallet_geo)
            )
            checks["iv"] = check_iv(truth, states, section, bounds, rear_offset)
            checks["v"] = check_v(
                truth,
                min(  # the earliest anchor: a realignment's refused capture came first
                    [float(nt["start"]["aligned_s"])]
                    + ([float(realign_["time_s"])] if realign_ is not None else [])
                ),
                end if end is not None else section[1],
            )
            if (
                "corrections" in near
            ):  # an S4b record; earlier runs have no correction points
                if "paths.json" not in raw:
                    raise Unverified("no paths.json")
                checks["viii"] = check_viii(
                    truth,
                    near,
                    nt["start"],
                    json.loads(raw["paths.json"]),
                    states,
                    bounds,
                    rear_offset,
                )
        except _SkipEpisode:
            pass
        except Unverified as exc:
            problems.append(str(exc))
    if run_files_digest(directory) != before:  # judged on the files the digest names
        problems.append("the run's files changed during the judgement")
    contacts = result.get("forbidden_pocket_contacts")
    passed = (
        not problems
        and vi["passed"]
        and vii["passed"]
        and all(
            c.get("passed", c.get("status") == "pass")
            for c in checks.values()
            if c.get("applicable", True)
        )
    )
    verdict = "unverified" if problems else ("pass" if passed else "fail")
    return {
        "run": str(directory),
        "rules": RULES,
        "verdict": verdict,
        "problems": problems,
        "success": result.get("success"),
        "phase": result.get("phase"),
        "failure_reason": result.get("failure_reason"),
        "arrival_reason": (nt.get("insert_arrival") or {}).get("reason"),
        "i": checks.get("i"),
        "ii": checks.get("ii"),
        "iii": checks.get("iii"),
        "iv": checks.get("iv"),
        "v": checks.get("v"),
        "vi": vi,
        "vii": vii,
        "viii": checks.get("viii") or {"applicable": False},
        "s4b_pass": (
            verdict == "pass"
            and "viii" in checks
            and result.get("success") is True
            and result.get("phase") == "complete"
        ),
        "ix": checks.get("ix") or {"applicable": False},
        "episodes": checks.get("episodes") or {"applicable": False},  # plan D8 S4c-2
        "x": checks.get("x") or {"applicable": False},
        "redraw": checks.get("redraw") or {"applicable": False},  # plan D8 S4c-3
        "xi": checks.get("xi") or {"applicable": False},
        "forbidden_pocket_contacts": contacts,
        "s4c": s4c_outcome(verdict, checks, result, vi, contacts, vii),
        "inputs_sha256": {name: sha256_bytes(data_) for name, data_ in raw.items()},
        "run_files_digest": before,
        **provenance_extra,
        "models_sha256": {key: sha256_bytes(data_) for key, data_ in models.items()}
        | {"drive_terms": terms_sha},
        "sources_sha256": loaded_sources(),
    }


def json_value(value):
    """numpy scalars (a comparison of numpy floats is a numpy bool) as plain JSON values."""
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"{type(value).__name__} is not JSON serializable")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--runs", nargs="+", required=True, type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--data-root",
        type=Path,
        default=ROOT,
        help="where the bounds file's provenance paths are relative to",
    )
    args = parser.parse_args(argv)
    runs = [judge_run(d, args.data_root) for d in args.runs]
    args.output.write_text(
        json.dumps({"runs": runs}, indent=1, default=json_value) + "\n"
    )
    for r in runs:
        vi, vii = r["vi"] or {}, r["vii"] or {}
        print(
            json.dumps(
                {
                    "run": r["run"],
                    "verdict": r["verdict"],
                    "problems": r["problems"],
                    "success": r["success"],
                    "contact": (vi.get("contact") or {}).get("passed"),
                    "depth": vi.get("depth"),
                    "stops": len(vii.get("stops") or []),
                    "vii": vii.get("passed"),
                    **{
                        k: (r[k] or {}).get("passed")
                        for k in ("i", "ii", "iii", "iv", "v", "viii")
                    },
                    "ix": (r.get("ix") or {}).get("status"),
                    "s4c": r.get("s4c"),
                },
                default=json_value,
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
