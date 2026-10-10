"""Plan D8 S4 development judgement: the consumed-bound checks (vi) and (vii) from a run's
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


def judge_run(directory: Path) -> dict:
    sys.path.insert(0, str(ROOT / "sim" / "isaac"))
    import insertion_geometry as IG

    from forklift_core.perception.pallet_geometry import load_pallet_geometry

    raw = {
        name: (directory / name).read_bytes()
        for name in ("result.json", "pocket_frames/truth.npz", "near_tracking.json")
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
        except Unverified as exc:
            problems.append(str(exc))
    verdict = (
        "unverified"
        if problems
        else ("pass" if vi["passed"] and vii["passed"] else "fail")
    )
    return {
        "run": str(directory),
        "rules": RULES,
        "verdict": verdict,
        "problems": problems,
        "success": result.get("success"),
        "phase": result.get("phase"),
        "failure_reason": result.get("failure_reason"),
        "arrival_reason": (nt.get("insert_arrival") or {}).get("reason"),
        "vi": vi,
        "vii": vii,
        "inputs_sha256": {name: sha256_bytes(data_) for name, data_ in raw.items()},
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
    args = parser.parse_args(argv)
    runs = [judge_run(d) for d in args.runs]
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
                },
                default=json_value,
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
