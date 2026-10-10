"""Plan D8 S4a: the runner side of near-field pocket tracking -- pure (no Isaac), so the
depth contract, the control pose history, the wait for the next result, the travel
budget, the section entry and the contract check are testable on the CPU.

docs/plans/2026-10-04-lidar-obstacle-map.md, "D8 S4 실행기 연결 설계" ②–⑤. The tracker
itself is forklift_core.perception.near_field_tracking (D8c 3판, replayed in ⑥ by
tools/d8c_replay.py with the same adapter rules).
"""

from __future__ import annotations

import math
import re
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

TICK_S = 1.0 / 120.0
HISTORY_S = 1.0  # the control pose history the frames are placed with
MAX_ROW_GAP_S = 1.5 * TICK_S  # no interpolation across a gap (Codex D8b impl P2)
SECTION_M = 1.5  # fork tip to face: the near-field section (plan D8e)
STOP_LATENCY_S = 0.15 + TICK_S  # the drive's reaction plus the tick that notices
DECEL_MPS2 = 1.5
BUDGET_M = 0.03
RECENT_S = 0.3  # the applied-command maximum the speed bound looks back over


def quantize_depth_mm(depth_m) -> np.ndarray:
    """The ③ depth contract: metres rounded to 1 mm, invalid pixels NaN, no noise."""
    depth = np.asarray(depth_m, dtype=float)
    finite = np.isfinite(depth)
    return np.where(
        finite, np.round(np.where(finite, depth, 0.0) * 1000.0) / 1000.0, np.nan
    )


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class RearHistory:
    """Rows (rear_stamp, control rear axle pose, hold episode) of the last HISTORY_S.

    A frame's pose is the rear axle interpolated (yaw unwrapped) between two rows of the
    same hold episode at most MAX_ROW_GAP_S apart, moved forward by the rear axle offset
    to base_link -- the formula of ⑥'s control_base. A release empties it (the tracker is
    discarded with it)."""

    def __init__(self, rear_offset_m: float, keep_s: float = HISTORY_S):
        if not (math.isfinite(rear_offset_m) and math.isfinite(keep_s) and keep_s > 0):
            raise ValueError("rear offset and history length must be finite")
        self.offset = abs(float(rear_offset_m))
        self.keep_s = float(keep_s)
        self.rows: deque = deque()

    def add(self, stamp_s: float, rear, episode: int) -> None:
        rear = tuple(float(v) for v in rear)
        if not (
            math.isfinite(stamp_s)
            and len(rear) == 3
            and all(math.isfinite(v) for v in rear)
        ):
            raise ValueError("a history row must be a finite time and pose")
        if self.rows and stamp_s < self.rows[-1][0] - 1e-12:
            raise ValueError("history rows must come in time order")
        if self.rows and abs(stamp_s - self.rows[-1][0]) <= 1e-12:
            self.rows.pop()  # the same instant again (a capture's tick): the latest pose
        self.rows.append((float(stamp_s), rear, int(episode)))
        while self.rows and self.rows[0][0] < stamp_s - self.keep_s - 1e-9:
            self.rows.popleft()

    def clear(self) -> None:
        self.rows.clear()

    def _bracket(self, t: float, episode: int):
        times = [r[0] for r in self.rows]
        k = int(np.searchsorted(times, t))
        if k < len(times) and abs(times[k] - t) <= 1e-9:
            row = self.rows[k]
            return (row, row) if row[2] == episode else None
        if not 0 < k < len(times):
            return None
        a, b = self.rows[k - 1], self.rows[k]
        if a[2] != episode or b[2] != episode or b[0] - a[0] > MAX_ROW_GAP_S + 1e-9:
            return None
        return a, b

    def bracket_times(self, t: float, episode: int) -> list | None:
        """The stamps of the two rows a pose at t is interpolated between (for the record)."""
        pair = self._bracket(t, episode)
        return None if pair is None else [pair[0][0], pair[1][0]]

    def rear_at(self, t: float, episode: int) -> tuple | None:
        """The rear axle pose at t, or None when t is not covered (outside the history,
        across a gap or across a hold episode)."""
        pair = self._bracket(t, episode)
        if pair is None:
            return None
        (t0, p0, _), (t1, p1, _) = pair
        a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        yaw1 = p0[2] + _wrap(p1[2] - p0[2])
        return (
            p0[0] + a * (p1[0] - p0[0]),
            p0[1] + a * (p1[1] - p0[1]),
            _wrap(p0[2] + a * (yaw1 - p0[2])),
        )

    def base_at(self, t: float, episode: int) -> tuple | None:
        """held_from_base_link at t (x, y, yaw), or None when not covered."""
        rear = self.rear_at(t, episode)
        if rear is None:
            return None
        x, y, yaw = rear
        return (x + self.offset * math.cos(yaw), y + self.offset * math.sin(yaw), yaw)

    def path_length_since(self, t: float, episode: int) -> float | None:
        """The control rear axle's path length from t to the latest row, or None when the
        span is not covered row to row in this episode."""
        if not self.rows or self.rows[-1][2] != episode:
            return None
        start = self.rear_at(t, episode)
        if start is None:
            return None
        points = [start[:2]]
        previous = t
        for stamp, rear, ep in self.rows:
            if stamp <= t + 1e-9:
                continue
            if ep != episode or stamp - previous > MAX_ROW_GAP_S + 1e-9:
                return None
            points.append(rear[:2])
            previous = stamp
        xy = np.asarray(points, dtype=float)
        return float(np.sum(np.hypot(*np.diff(xy, axis=0).T))) if len(xy) > 1 else 0.0


def result_wait_s(
    now_s: float, last_stamp_s: float, period_s: float, read_delay_s: float
) -> float:
    """τ: the time from now until the next result can arrive, (last stamp + k period +
    read delay) - now for the smallest k >= 1 that is still ahead. Every result due by now
    has been delivered when this is asked, so a due frame that did not come moves τ on to
    the next one (plan S4a ⑤, 1차 검토 P3)."""
    for value in (now_s, last_stamp_s, period_s, read_delay_s):
        if not math.isfinite(value):
            raise ValueError("times must be finite")
    if period_s <= 0 or read_delay_s < 0:
        raise ValueError("the period must be positive and the read delay non-negative")
    k = max(1, math.floor((now_s - last_stamp_s - read_delay_s) / period_s + 1e-9) + 1)
    return last_stamp_s + k * period_s + read_delay_s - now_s


@dataclass(frozen=True)
class DriveTerms:
    """The measured drive terms (tools/s4_drive_terms.py, confirmed matrix only)."""

    delta_len_m: tuple  # running maximum per window length, index = ticks
    delta_s_m: tuple
    delta_v_mps: float
    cruise_mps: float
    condition: dict  # the measurement runs' arguments, settings and input files
    tick_s: float = TICK_S

    @classmethod
    def from_dict(cls, data: dict, bounds_sha256: str, latency: dict) -> DriveTerms:
        """Only a confirmed matrix measured against this bounds file and its latency and
        render period (Codex S4a-1 review P2)."""
        if data.get("status") != "confirmed" or "cruise_mps" not in data:
            raise ValueError(
                "drive terms must come from the confirmed measurement matrix"
            )
        if (data.get("provenance") or {}).get("bounds", {}).get(
            "sha256"
        ) != bounds_sha256:
            raise ValueError(
                "the drive terms were measured against another bounds file"
            )
        for key, want in (
            ("align_age_s", latency["max_s"] + latency["read_delay_s"]),
            ("render_period_s", latency["render_period_s"]),
        ):
            have = data.get(key)
            if (
                isinstance(have, bool)
                or not isinstance(have, (int, float))
                or not math.isfinite(have)
                or abs(have - want) > 1e-12
            ):
                raise ValueError(
                    f"the drive terms' {key} is missing or differs from the bounds file"
                )
        cruise = data["cruise_mps"]
        if (
            isinstance(cruise, bool)
            or not isinstance(cruise, (int, float))
            or not (
                math.isfinite(cruise)
                and 0 < cruise <= float(data.get("cruise_cap_mps", math.nan))
            )
        ):
            raise ValueError(
                "the drive terms' cruise must be finite, positive and within the measured speeds"
            )
        if not isinstance(data.get("condition"), dict):
            raise ValueError("the drive terms carry no measurement condition")
        terms = cls(
            tuple(float(v) for v in data["delta_len_m"]),
            tuple(float(v) for v in data["delta_s_m"]),
            float(data["delta_v_mps"]),
            float(cruise),
            data["condition"],
            float(data["tick_s"]),
        )
        for table in (terms.delta_len_m, terms.delta_s_m):
            if (
                not table
                or not all(math.isfinite(v) and v >= 0 for v in table)
                or any(b < a for a, b in zip(table, table[1:], strict=False))
            ):
                raise ValueError(
                    "drive term tables must be finite, non-negative running maxima"
                )
        if not (
            math.isfinite(terms.delta_v_mps)
            and terms.delta_v_mps >= 0
            and abs(terms.tick_s - TICK_S) < 1e-12
        ):
            raise ValueError(
                "delta_v must be finite and non-negative, the tick 1/120 s"
            )
        return terms

    def at(self, table: tuple, window_s: float) -> float | None:
        """The running maximum at the first stored window at or above window_s; None past
        the measured windows (the caller stops)."""
        k = max(int(math.ceil(window_s / self.tick_s - 1e-9)), 0)
        return None if k >= len(table) else table[k]


@dataclass(frozen=True)
class Budget:
    d_m: float | None  # control path since the pixel time + delta_len(age)
    travel_m: (
        float | None
    )  # vbar (tau + latency) + Delta_s(tau + latency) + vbar^2 / (2 decel)
    total_m: float | None
    ok: bool
    reason: str | None  # None, "budget", "uncovered" or "unmeasured"
    len_term_m: float | None = None  # delta_len(age)
    s_term_m: float | None = None  # Delta_s(horizon)
    horizon_s: float | None = None  # tau + stop latency


def travel_budget(
    path_since_pixel_m: float | None,
    age_s: float,
    tau_s: float,
    vbar_mps: float,
    terms: DriveTerms,
) -> Budget:
    """Plan S4a ⑤: d̄ + the travel until a stop after the next result can arrive, as a bound
    on the real travel, against BUDGET_M."""
    if path_since_pixel_m is None:
        return Budget(None, None, None, False, "uncovered")
    horizon = tau_s + STOP_LATENCY_S
    len_term, s_term = (
        terms.at(terms.delta_len_m, age_s),
        terms.at(terms.delta_s_m, horizon),
    )
    if len_term is None or s_term is None:
        return Budget(None, None, None, False, "unmeasured", len_term, s_term, horizon)
    d = path_since_pixel_m + len_term
    travel = vbar_mps * horizon + s_term + vbar_mps * vbar_mps / (2 * DECEL_MPS2)
    total = d + travel
    ok = total <= BUDGET_M + 1e-12
    return Budget(
        d, travel, total, ok, None if ok else "budget", len_term, s_term, horizon
    )


class CommandWindow:
    """|applied speed command| per physics interval, keyed by the interval's end (as the
    recorder stamps it and tools/s4_drive_terms.py measures delta_v against), for
    vbar = the largest command of the intervals ending within RECENT_S before now, or
    the command about to be applied, + delta_v."""

    def __init__(self):
        self.rows: deque = deque()

    def add(self, end_s: float, speed_mps: float) -> None:
        self.rows.append((float(end_s), abs(float(speed_mps))))
        while self.rows and self.rows[0][0] <= end_s - RECENT_S - TICK_S + 1e-9:
            self.rows.popleft()

    def vbar(self, now_s: float, delta_v_mps: float, pending_mps: float = 0.0) -> float:
        recent = [v for end, v in self.rows if end > now_s - RECENT_S + 1e-9]
        return max([abs(pending_mps), *recent]) + delta_v_mps


def face_min_x_m(held: dict, widths: tuple, base_now) -> float:
    """The smallest base_link x of the four face wall points (each pocket's centre +- half
    its width across the insertion axis) of an accepted observation placed in the held
    frame (``Accepted.held``), seen from the control base now -- ⑥'s estimate_walls."""
    yaw = held["yaw"] - base_now[2]
    lateral = (-math.sin(yaw), math.cos(yaw))
    c, s = math.cos(base_now[2]), math.sin(base_now[2])
    xs = []
    for side, width in zip(("left", "right"), widths, strict=True):
        dx, dy = held[side][0] - base_now[0], held[side][1] - base_now[1]
        x = c * dx + s * dy
        xs += [x + width / 2 * lateral[0], x - width / 2 * lateral[0]]
    return min(xs)


def section_entered(front_min_x_m: float, fork_tip_x_m: float) -> bool:
    """The near-field section (plan S4a ⑤): the estimate's front wall, moved to the base
    now, at most SECTION_M ahead of the fork tips."""
    return front_min_x_m - fork_tip_x_m <= SECTION_M + 1e-12


# Input files and arguments, by content (shared with tools/s4_drive_terms.py).
def snapshot_relative(value):
    """An argument path inside a snapshot, from the snapshot root (runs from different
    snapshots name the same file differently)."""
    if isinstance(value, str) and "/snapshots/" in value:
        rest = value.split("/snapshots/", 1)[1]
        return rest.split("/", 1)[1] if "/" in rest else rest
    return value


def snapshot_root(arguments: dict) -> Path | None:
    """The snapshot a run ran from, read off its absolute snapshot paths."""
    for value in arguments.values():
        if isinstance(value, str) and "/snapshots/" in value:
            head, rest = value.split("/snapshots/", 1)
            return Path(head) / "snapshots" / rest.split("/", 1)[0]
    return None


# Arguments naming inputs the run must have read: both sides must resolve and match.
REQUIRED_FILE_ARGUMENTS = (
    "base_scene",
    "pallet_urdf",
    "pallet_geometry",
    "forklift_urdf",
    "settings",
    "factory_layout",
    "lidar",
    "obstacle_layer",
    "pallet_prior",
)
FILE_SUFFIXES = (".json", ".yaml", ".yml", ".usd", ".usda", ".urdf", ".npz")
TEXT_SUFFIXES = (".json", ".yaml", ".yml")
REFERENCE = re.compile(r"[A-Za-z0-9_./-]+\.(?:json|ya?ml|usda?|urdf|npz)\b")


def argument_path(value, root: Path | None) -> Path | None:
    """The path an argument names (relative ones from the snapshot root), or None when
    it is not a path (a URL, a name) or a relative one has no root."""
    if not isinstance(value, str) or "://" in value:
        return None
    if "/" not in value and not value.endswith(FILE_SUFFIXES):
        return None
    path = Path(value)
    if not path.is_absolute():
        if root is None:
            return None
        path = root / path
    return path


def file_sha256(path: Path) -> str:
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def label_of(path: Path, root: Path | None) -> str:
    if root is not None:
        try:
            return path.resolve().relative_to(root.resolve()).as_posix()
        except ValueError:
            pass
    return str(path)


def config_paths(data) -> list[str]:
    """Every string value of a parsed YAML/JSON document that names a file by a relative
    or absolute path -- what a loader can read (config/obstacle_layer_single.yaml's
    odometry_age). Comments are not values, so paths mentioned in them are not inputs."""
    if isinstance(data, dict):
        return [p for v in data.values() for p in config_paths(v)]
    if isinstance(data, list):
        return [p for v in data for p in config_paths(v)]
    return [data] if isinstance(data, str) and REFERENCE.fullmatch(data) else []


def input_files_skipping(
    arguments: dict, root: Path | None, skip
) -> dict[str, str | None]:
    """label -> sha256 (None: missing) of every file an argument names and, transitively,
    every file a YAML or JSON input names as a value (from the snapshot root, else the
    naming file's folder) -- config/obstacle_layer_single.yaml makes the runner read
    config/obstacle_odometry_age.json (Codex S4a-1 ⓪ 4th and 5th reviews). All of them
    are required: the caller refuses a file missing on either side."""
    import yaml

    out: dict[str, str | None] = {}
    queue: list[Path] = []
    for name, value in sorted(arguments.items()):
        if name in skip:
            continue
        path = argument_path(value, root)
        if path is None:
            if name in REQUIRED_FILE_ARGUMENTS:
                out[f"argument {name}"] = None
            continue
        exists = path.is_file()
        out[f"argument {name}"] = file_sha256(path) if exists else None
        if exists and path.suffix in TEXT_SUFFIXES:
            queue.append(path)
    seen: set = set()
    while queue:
        source = queue.pop()
        if source.resolve() in seen:
            continue
        seen.add(source.resolve())
        for ref in sorted(set(config_paths(yaml.safe_load(source.read_text())))):
            path = Path(ref)
            if not path.is_absolute():
                found = [
                    c
                    for c in (
                        (root / ref) if root is not None else None,
                        source.parent / ref,
                    )
                    if c is not None and c.is_file()
                ]
                path = (
                    found[0]
                    if found
                    else (root / ref if root is not None else source.parent / ref)
                )
            label = f"file {label_of(path, root)}"
            if not path.is_file():
                out[label] = None
                continue
            out[label] = file_sha256(path)
            if path.suffix in TEXT_SUFFIXES:
                queue.append(path)
    return out


# Arguments that differ between runs by design: output places, the seed and its noise
# seed (checked equal to each other), the speed under test, instrumentation, D5 (its own
# stops) and the near-tracking options themselves.
RUN_SPECIFIC_ARGUMENTS = (
    "output",
    "slam_feedback",
    "slam_map_dir",
    "seed",
    "slam_noise_seed",
    "approach_straight_speed_mps",
    "measure_safety_stops_m",
    "record_dwell_gaps",
    "record_pocket_frames",
    "pocket_check",
    "near_tracking",
    "near_tracking_stop_at",
    "near_bounds",
    "near_drive_terms",
)
CONDITION_FIELDS = (
    "settings_synthetic",
    "detector_params",
    "planner_config",
    "travel_planner_config",
    "lidar_synthetic",
    "chassis_model",
)


SPEED_DEPENDENT_TRACKER_FIELDS = (
    ("approach", "cruise_speed_mps"),
)  # the speed under test


def tracker_configs_shared(configs) -> dict | None:
    """The follower configs without the fields that follow the straight speed under test
    (between the 0.055 and 0.08 m/s measurement runs only approach.cruise_speed_mps
    differs)."""
    if not isinstance(configs, dict):
        return None
    out = {k: dict(v) if isinstance(v, dict) else v for k, v in configs.items()}
    for phase, field in SPEED_DEPENDENT_TRACKER_FIELDS:
        if isinstance(out.get(phase), dict):
            out[phase].pop(field, None)
    return out


def tracker_config_problems(expected, observed) -> list[str]:
    """The followers' configs (speed fields aside) against the measurement runs'."""
    want, have = tracker_configs_shared(expected), tracker_configs_shared(observed)
    if want is None or have is None:
        return ["tracker_configs missing"]
    return [
        f"tracker_configs {phase} differs from the measurement runs"
        for phase in sorted(set(want) | set(have))
        if want.get(phase) != have.get(phase)
    ]


def contract_arguments(arguments: dict) -> dict:
    """The arguments a run must share with the measurement runs (paths inside a snapshot
    from the snapshot root)."""
    return {
        k: snapshot_relative(v)
        for k, v in arguments.items()
        if k not in RUN_SPECIFIC_ARGUMENTS
    }


def run_condition(result: dict, root: Path | None) -> dict:
    """What a run is measured under, as tools/s4_drive_terms.py exports it and the runner
    checks it: its shared arguments, recorded settings, follower configs (speed fields
    aside) and input files by content -- an argument naming a file is compared by that
    file's content, not by how its path is written (Codex S4a-1 2nd review P3)."""
    arguments = result.get("arguments") or {}
    files = input_files_skipping(arguments, root, RUN_SPECIFIC_ARGUMENTS)
    by_content = {
        label.removeprefix("argument ")
        for label in files
        if label.startswith("argument ")
    }
    return {
        "arguments": {
            k: v
            for k, v in contract_arguments(arguments).items()
            if k not in by_content
        },
        **{name: result.get(name) for name in CONDITION_FIELDS},
        # The URDF by content (input_files), not by its path string.
        "chassis_model": {
            k: v
            for k, v in (result.get("chassis_model") or {}).items()
            if k != "forklift_urdf"
        }
        or None,
        "tracker_configs": tracker_configs_shared(result.get("tracker_configs")),
        "input_files": files,
    }


def condition_problems(expected: dict, observed: dict, skip=()) -> list[str]:
    """Differences between the measurement runs' condition and this run's (both JSON);
    ``skip`` names fields checked elsewhere (the runner checks tracker_configs once the
    mission is planned)."""
    problems = []
    for name in sorted((set(expected) | set(observed)) - set(skip)):
        want, have = expected.get(name), observed.get(name)
        if (
            name in ("arguments", "input_files")
            and isinstance(want, dict)
            and isinstance(have, dict)
        ):
            for key in sorted(set(want) | set(have)):
                if want.get(key) != have.get(key) or (
                    name == "input_files" and have.get(key) is None
                ):
                    problems.append(f"{name} {key} differs from the measurement runs")
        elif want != have or want is None:
            problems.append(f"{name} differs from the measurement runs")
    return problems


# The bounds file's condition the run must match (plan S4a, 적용 계약).
# The pallet geometry and URDF are compared by content (ASSET_HASH_KEYS), not by how
# their paths are written (Codex S4a-1 3rd review P3).
CONDITION_KEYS = (
    "video",
    "fps",
    "mount",
    "camera",
    "rear_axle_offset_m",
)
ASSET_HASH_KEYS = (
    "pallet_geometry_sha256",
    "forklift_urdf_sha256",
    "pallet_urdf_sha256",
)


def contract_problems(condition: dict, observed: dict) -> list[str]:
    """Why this run may not use the measured bounds (empty: it may). ``observed`` holds the
    CONDITION_KEYS as the runner records them in the pocket-frame meta, the asset hashes,
    ``seed`` / ``slam_noise_seed`` and ``slam_feedback``."""
    problems = []
    for key in CONDITION_KEYS:
        if observed.get(key) != condition.get(key):
            problems.append(f"{key} differs from the bounds condition")
    hashes = condition.get("source_sha256") or {}
    for key in ASSET_HASH_KEYS:
        if observed.get(key) is None or observed.get(key) != hashes.get(key):
            problems.append(f"{key} differs from the bounds condition")
    if not observed.get("slam_feedback"):
        problems.append("tracking needs --slam-feedback")
    if observed.get("slam_noise_seed") is None or observed.get(
        "slam_noise_seed"
    ) != observed.get("seed"):
        problems.append("the SLAM odometry noise seed must be the scene seed")
    return problems


__all__ = [
    "BUDGET_M",
    "Budget",
    "CommandWindow",
    "DriveTerms",
    "RearHistory",
    "condition_problems",
    "contract_arguments",
    "tracker_config_problems",
    "contract_problems",
    "face_min_x_m",
    "run_condition",
    "quantize_depth_mm",
    "result_wait_s",
    "section_entered",
    "travel_budget",
]
