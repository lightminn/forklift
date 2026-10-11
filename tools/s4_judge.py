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


def record_problems(truth: dict) -> list[str]:
    """Why the truth record cannot carry the judgement over the approach and insertion."""
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
    ticks = checked_ticks(truth)
    if not len(ticks):
        return ["no approach or insert tick in the truth record"]
    # One tick past the last, when recorded: the command given at the last tick. A run
    # that ends on a near-field terminal stop or at the S4a-1 approach end records nothing
    # after it (its zero command simply holds to the end).
    span = slice(int(ticks[0]), min(int(ticks[-1]) + 2, n))
    if np.any(np.abs(np.diff(t[span]) - TICK_S) > 0.05 * TICK_S):
        problems.append(
            "the truth ticks are not contiguous at 120 Hz over the approach and insertion"
        )
    for key in ("base_pose", "pallet_pose", "lift_m", "applied_speed_mps"):
        if not np.all(np.isfinite(np.asarray(truth[key], dtype=float)[span])):
            problems.append(f"non-finite {key} over the approach and insertion")
    phase = np.asarray(truth["phase"])[int(ticks[0]) : int(ticks[-1]) + 1]
    phases = [str(p) for p in truth["phases"]]
    if np.any(
        ~np.isin(phase, [phases.index(p) for p in PHASES_CHECKED if p in phases])
    ):
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


def near_section(truth: dict, near: dict) -> tuple[int, int]:
    """Truth indices of the near-field section: the armed tick to the last approach or
    insert tick."""
    armed = next(
        (e["time_s"] for e in near.get("events") or [] if e.get("event") == "armed"),
        None,
    )
    if armed is None:
        raise Unverified("the near-field section was never entered (no armed event)")
    t = np.asarray(truth["stamp_s"], dtype=float)
    first = tick_index(t, armed, "the armed event")
    last = int(checked_ticks(truth)[-1])
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


def check_iii(run, bounds_data: dict, calibration: dict, model) -> dict:
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
    gaps = [b["stamp_s"] - a["stamp_s"] for a, b in zip(reads, reads[1:], strict=False)]
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
    if points != list(NT.CORRECTION_POINTS_M) or accepted != [True] * len(
        NT.CORRECTION_POINTS_M
    ):
        failures.append(
            f"corrections {points} accepted {accepted}: not every point once, accepted"
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
    if revisions:
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
    problems += record_problems(truth)
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
            vii = check_vii(
                truth,
                near,
                json.loads(terms_bytes),
                near_section(truth, near),
                float(args["rear_axle_offset_m"]),
                corners,
                blades,
            )
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
                float(nt["start"]["aligned_s"]),
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
        except Unverified as exc:
            problems.append(str(exc))
    if run_files_digest(directory) != before:  # judged on the files the digest names
        problems.append("the run's files changed during the judgement")
    passed = (
        not problems
        and vi["passed"]
        and vii["passed"]
        and all(c["passed"] for c in checks.values())
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
                },
                default=json_value,
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
