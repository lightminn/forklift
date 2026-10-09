"""Plan D8b offline calibration of the near-field depth stream (no Isaac).

docs/plans/2026-10-04-lidar-obstacle-map.md, "D8b 구현 설계" (7판). Inputs are the
runner's --record-pocket-frames directories (pocket_frames/index.json, truth.npz,
frame_*.npz) and slam_control.npy. This module computes

  ① the pixel delay without the detector: for each eligible frame and each of the 133
     tick candidates tau in [-0.1, 1.0] s, an unquantised rigid render of the pallet at
     the recorded physics poses of the camera and the pallet at s - tau, against the
     recorded float32 depth over the outer front-face pixels of that tau's render; the
     feasible set {tau : |r(tau)| <= eps + B(d)} kept as connected pieces;
  the dwell biases b_k and B(d), and the pass rule (i)-(v);
  ② the control rear-axle pose error at s - L;
  ④ relative odometry error tables (position and heading separately, by age and by
     distance, and from the near-capture anchor).

③ (observation errors) and ⑤ (CPU-Isaac correspondence) are a later stage.

    python tools/d8b_calibration.py --seed-runs 1:<dir>,<dir>,... --output <dir>
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools import scene_rig  # noqa: E402

TICK_S = 1.0 / 120.0
# Every eligible frame's residual is evaluated over the fixed -0.1 .. 1.0 s (Codex D8b 4th
# P2: never widened per frame), but a frame is eligible on the motion of [s - 0.4, s + 0.1]
# and the delay band is the part up to IN_RANGE_S: any piece beyond it fails the
# calibration (confirmation review P1 -- a shrunk search alone let a 0.6 s delay alias
# into a valid piece at 0.3 s). Matrix 1883 used the 1.1 s motion history: every piece of
# its 2,436 eligible frames lay in 84-99 ms, and that history left the first dwell bin
# 6-7 fast frames.
TAUS = -0.1 + np.arange(133) * TICK_S
IN_RANGE_S = 0.4
EPS_M = 0.2e-3
M_U_M = 0.1e-3  # unverified between-dwell allowance (D8b 5th review)
ERODE_PX = 3
MIN_PIXELS = 300
# The tau render must explain nearly all of its front pixels: at the true delay the smoke
# runs (1857) kept 100 % within 20 mm, while an alias -- the inner block faces 0.2275 m
# behind, seen through the pockets, lining up with a 0.25 m-earlier outer face -- kept
# 0.0-2.9 % yet over 300 pixels and made a second piece at 0.85 s.
MIN_COVERAGE = 0.9
OUTLIER_M = 0.020
V_MIN_MPS = 0.05
# A standing truck still creeps 0.03-0.08 mm against the pallet over a 1.1 s window
# (smoke 1857), so a static frame may move 0.1 mm (eps / 2); the dwell's largest such
# shift is added to its bias in B, so the creep is carried, not ignored.
STATIC_POS_M = 0.1e-3
STATIC_ROT_RAD = 1e-4
STATIC_MIN = 5
BIAS_LIMIT_M = 0.25e-3
SPREAD_LIMIT_M = 0.1e-3
MIN_ELIGIBLE = 30
MIN_SINGLE = 20
CELL_MIN = 8
SLOW_MPS = (0.05, 0.10)
FAST_MPS = 0.15
FACE_TOL_M = 1e-6


# Poses ---------------------------------------------------------------------------------
def quat_matrix(q_wxyz) -> np.ndarray:
    w, x, y, z = np.asarray(q_wxyz, dtype=float) / np.linalg.norm(q_wxyz)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def slerp(q0, q1, a: float) -> np.ndarray:
    q0, q1 = np.asarray(q0, dtype=float), np.asarray(q1, dtype=float)
    dot = float(np.dot(q0, q1))
    if dot < 0:
        q1, dot = -q1, -dot
    if dot > 0.9995:
        q = q0 + a * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = math.acos(min(dot, 1.0))
    return (math.sin((1 - a) * theta) * q0 + math.sin(a * theta) * q1) / math.sin(theta)


def pose_at(times: np.ndarray, poses7: np.ndarray, t: float) -> tuple[np.ndarray, np.ndarray]:
    """(R, p) at time t, linear in position and slerp in rotation between ticks."""
    if t < times[0] - 1e-9 or t > times[-1] + 1e-9:
        raise ValueError(f"time {t} outside the recorded ticks")
    k = int(np.clip(np.searchsorted(times, t), 1, len(times) - 1))
    t0, t1 = times[k - 1], times[k]
    a = 0.0 if t1 <= t0 else float(np.clip((t - t0) / (t1 - t0), 0.0, 1.0))
    p = poses7[k - 1, :3] + a * (poses7[k, :3] - poses7[k - 1, :3])
    return quat_matrix(slerp(poses7[k - 1, 3:], poses7[k, 3:], a)), p


@dataclass(frozen=True)
class Mount:
    """base_link <- optical at lift 0; fork_lift is a z prismatic joint at the base origin."""

    rotation: np.ndarray
    translation_m: np.ndarray

    @classmethod
    def from_meta(cls, mount: dict) -> "Mount":
        return cls(np.asarray(mount["rotation"], dtype=float), np.asarray(mount["translation_m"], dtype=float))


class Truth:
    """The physics truth of a recorded run (pocket_frames/truth.npz)."""

    def __init__(self, data, mount: Mount):
        self.t = np.asarray(data["stamp_s"], dtype=float)
        self.base = np.asarray(data["base_pose"], dtype=float)
        self.lift = np.asarray(data["lift_m"], dtype=float)
        self.pallet = np.asarray(data["pallet_pose"], dtype=float)
        self.phase = np.asarray(data["phase"])
        self.phases = [str(p) for p in data["phases"]]
        self.mount = mount

    def camera(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """World <- optical (R, p) of the near-field camera at time t."""
        r_wb, p_wb = pose_at(self.t, self.base, t)
        q = float(np.interp(t, self.t, self.lift))
        return r_wb @ self.mount.rotation, p_wb + r_wb @ (self.mount.translation_m + np.array([0.0, 0.0, q]))

    def pallet_pose(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        return pose_at(self.t, self.pallet, t)

    def camera_in_pallet(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        r_wc, p_wc = self.camera(t)
        r_wp, p_wp = self.pallet_pose(t)
        return r_wp.T @ r_wc, r_wp.T @ (p_wc - p_wp)

    def phase_at(self, t: float) -> str:
        k = int(np.clip(np.searchsorted(self.t, t, side="right") - 1, 0, len(self.t) - 1))
        return self.phases[int(self.phase[k])]


# Rendering -----------------------------------------------------------------------------
@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_meta(cls, k: dict) -> "Intrinsics":
        return cls(float(k["fx"]), float(k["fy"]), float(k["cx"]), float(k["cy"]), int(k["width"]), int(k["height"]))


@dataclass(frozen=True)
class PalletModel:
    boxes: tuple  # (centre, size) in the pallet's floor-centred frame
    depth_m: float
    width_m: float
    height_m: float

    @classmethod
    def from_geometry(cls, geometry) -> "PalletModel":
        from forklift_core.perception.pallet_geometry import pallet_boxes

        boxes = tuple((np.asarray(b.centre_m, dtype=float), np.asarray(b.size_m, dtype=float))
                      for b in pallet_boxes(geometry))
        return cls(boxes, geometry.overall_depth_m, geometry.overall_width_m, geometry.overall_height_m)

    def corners(self) -> np.ndarray:
        d, w, h = self.depth_m / 2, self.width_m / 2, self.height_m
        return np.array([[x, y, z] for x in (-d, d) for y in (-w, w) for z in (0.0, h)])

    def face_rectangles(self, face_x: float) -> np.ndarray:
        """(y0, y1, z0, z1) of every box face lying in the outer plane x = face_x."""
        rects = []
        for centre, size in self.boxes:
            near = centre[0] - size[0] / 2 if face_x < 0 else centre[0] + size[0] / 2
            if abs(near - face_x) <= FACE_TOL_M:
                rects.append((centre[1] - size[1] / 2, centre[1] + size[1] / 2,
                              centre[2] - size[2] / 2, centre[2] + size[2] / 2))
        return np.asarray(rects, dtype=float)


def roi(k: Intrinsics, r_pc: np.ndarray, p_pc: np.ndarray, model: PalletModel, margin: int = 6):
    """Pixel window holding the pallet's projection (whole image if a corner is behind)."""
    cam = (model.corners() - p_pc) @ r_pc  # pallet -> optical
    if np.any(cam[:, 2] < 0.05):
        return 0, k.height, 0, k.width
    u = k.fx * cam[:, 0] / cam[:, 2] + k.cx
    v = k.fy * cam[:, 1] / cam[:, 2] + k.cy
    u0, u1 = int(max(0, math.floor(u.min()) - margin)), int(min(k.width, math.ceil(u.max()) + margin + 1))
    v0, v1 = int(max(0, math.floor(v.min()) - margin)), int(min(k.height, math.ceil(v.max()) + margin + 1))
    return v0, max(v0, v1), u0, max(u0, u1)


def render(k: Intrinsics, r_pc: np.ndarray, p_pc: np.ndarray, model: PalletModel, window, face_x: float,
           floor_world=None):
    """Unquantised optical-axis depth of the pallet over ``window`` and the outer front-face mask.

    ``r_pc, p_pc``: pallet <- optical. ``floor_world``: (R_wp, p_wp) to add the z = 0 floor.
    Returns (depth, front) as full-size arrays (NaN / False outside the window).
    """
    v0, v1, u0, u1 = window
    depth = np.full((k.height, k.width), np.nan)
    front = np.zeros((k.height, k.width), dtype=bool)
    if v1 <= v0 or u1 <= u0:
        return depth, front
    v, u = np.mgrid[v0:v1, u0:u1]
    optical = np.stack(((u - k.cx) / k.fx, (v - k.cy) / k.fy, np.ones(u.shape)), axis=-1)
    rays = optical @ r_pc.T  # in the pallet frame, optical-z parametrised
    best = np.full(u.shape, np.inf)
    for centre, size in model.boxes:
        best = np.fmin(best, scene_rig._box_depth(p_pc, rays, centre, size, np.eye(3)))
    hit_x = p_pc[0] + best * rays[..., 0]
    on_front = np.isfinite(best) & (np.abs(hit_x - face_x) <= FACE_TOL_M)
    if floor_world is not None:
        r_wp, p_wp = floor_world
        origin_z = (r_wp @ p_pc + p_wp)[2]
        ray_z = rays @ r_wp[2]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = -origin_z / ray_z
        t = np.where((ray_z < 0) & (t > 0), t, np.inf)
        on_front &= best <= t
        best = np.fmin(best, t)
    depth[v0:v1, u0:u1] = np.where(np.isfinite(best), best, np.nan)
    front[v0:v1, u0:u1] = on_front
    return depth, front


def erode(mask: np.ndarray, radius: int) -> np.ndarray:
    """Binary erosion by a (2r+1)^2 square; pixels near the array edge are dropped."""
    if radius <= 0:
        return mask.copy()
    n = 2 * radius + 1
    out = np.zeros_like(mask)
    h, w = mask.shape
    if h < n or w < n:
        return out
    integral = np.pad(mask.astype(np.int32), ((1, 0), (1, 0))).cumsum(0).cumsum(1)
    total = integral[n:, n:] - integral[:-n, n:] - integral[n:, :-n] + integral[:-n, :-n]
    out[radius:h - radius, radius:w - radius] = total == n * n
    return out


def front_plane(k: Intrinsics, r_pc: np.ndarray, p_pc: np.ndarray, model: PalletModel, window, face_x: float):
    """Depth and mask of the outer front plane over ``window`` (crop-sized arrays).

    The plane x = face_x is the pallet's surface nearest the camera: every other box
    point lies beyond it along x, and a ray from the camera's side crosses it first, so a
    crossing inside one of its face rectangles is the first hit -- the same pixels and
    depths as ``render``'s front mask (tested), without 22 slab tests per ray."""
    v0, v1, u0, u1 = window
    v, u = np.mgrid[v0:v1, u0:u1]
    depth = np.full(u.shape, np.nan)
    mask = np.zeros(u.shape, dtype=bool)
    # The first-hit argument needs the camera outside the face plane (Codex D8b impl P3).
    if (face_x < 0 and p_pc[0] >= face_x) or (face_x > 0 and p_pc[0] <= face_x):
        return depth, mask
    optical = np.stack(((u - k.cx) / k.fx, (v - k.cy) / k.fy, np.ones(u.shape)), axis=-1)
    rays = optical @ r_pc.T
    toward = rays[..., 0] * np.sign(face_x - p_pc[0]) > 1e-12  # parallel rays never cross
    t = np.zeros(u.shape)
    t[toward] = (face_x - p_pc[0]) / rays[..., 0][toward]
    y = p_pc[1] + t * rays[..., 1]
    z = p_pc[2] + t * rays[..., 2]
    inside = np.zeros(u.shape, dtype=bool)
    for y0, y1, z0, z1 in model.face_rectangles(face_x):
        inside |= (y >= y0) & (y <= y1) & (z >= z0) & (z <= z1)
    mask = toward & inside
    depth[mask] = t[mask]
    return depth, mask


def face_side(p_pc: np.ndarray, model: PalletModel) -> float:
    """The outer face the camera looks at: x = -depth/2 or +depth/2 in the pallet frame."""
    return -model.depth_m / 2 if p_pc[0] < 0 else model.depth_m / 2


def residual(isaac: np.ndarray, k: Intrinsics, r_pc, p_pc, model: PalletModel, face_x: float):
    """(r, n): median Isaac - CPU over this render's eroded outer-front pixels where Isaac
    is valid and within OUTLIER_M of it; r is NaN below MIN_PIXELS or when fewer than
    MIN_COVERAGE of all those front pixels are (same-surface check)."""
    v0, v1, u0, u1 = window = roi(k, r_pc, p_pc, model, margin=ERODE_PX + 3)
    cpu, front = front_plane(k, r_pc, p_pc, model, window, face_x)
    crop = isaac[v0:v1, u0:u1]
    predicted = erode(front, ERODE_PX)
    # Coverage over every predicted front pixel: one with no Isaac return is unexplained
    # too, not dropped from the denominator (Codex D8b impl P2).
    diff = crop[predicted] - cpu[predicted]
    with np.errstate(invalid="ignore"):
        kept = diff[np.isfinite(diff) & (np.abs(diff) <= OUTLIER_M)]
    if kept.size < max(MIN_PIXELS, MIN_COVERAGE * int(predicted.sum())):
        return math.nan, int(kept.size)
    return float(np.median(kept)), int(kept.size)


# Feasible delay pieces -------------------------------------------------------------------
def pieces(r: np.ndarray, tol: float, taus: np.ndarray = TAUS) -> list[tuple[float, float]]:
    """Connected pieces of {tau : |r(tau)| <= tol}, r linear between defined neighbours.

    Solved per segment, so a crossing between two ticks is found even when neither tick
    is within tolerance (a 5 mm-per-tick slope at 0.60 m/s)."""
    spans: list[tuple[float, float]] = []
    n = len(taus)
    for j in range(n):
        if math.isfinite(r[j]) and abs(r[j]) <= tol:
            spans.append((taus[j], taus[j]))
        if j + 1 < n and math.isfinite(r[j]) and math.isfinite(r[j + 1]):
            a, b = r[j], r[j + 1]
            lo, hi = 0.0, 1.0
            slope = b - a
            if abs(slope) < 1e-15:
                if abs(a) > tol:
                    continue
            else:
                s1, s2 = (-tol - a) / slope, (tol - a) / slope
                lo, hi = max(lo, min(s1, s2)), min(hi, max(s1, s2))
                if lo > hi:
                    continue
            spans.append((taus[j] + lo * (taus[j + 1] - taus[j]), taus[j] + hi * (taus[j + 1] - taus[j])))
    spans.sort()
    merged: list[list[float]] = []
    for lo, hi in spans:
        if merged and lo <= merged[-1][1] + 1e-12:
            merged[-1][1] = max(merged[-1][1], hi)
        else:
            merged.append([lo, hi])
    return [(float(lo), float(hi)) for lo, hi in merged]


# Runs ------------------------------------------------------------------------------------
@dataclass
class Run:
    directory: Path
    meta: dict
    reads: list
    dwells: list
    truth: Truth
    intrinsics: Intrinsics
    control: np.ndarray | None
    result: dict  # the runner's result.json: seed, arguments, input hashes

    @classmethod
    def load(cls, directory: Path) -> "Run":
        directory = Path(directory).resolve()
        index = json.loads((directory / "pocket_frames/index.json").read_text())
        meta = index["meta"]
        truth = Truth(np.load(directory / "pocket_frames/truth.npz"), Mount.from_meta(meta["mount"]))
        control_path = directory / "slam_control.npy"
        control = np.load(control_path) if control_path.exists() else None
        # Every number the analysis reads must be finite: a NaN would vanish from a max
        # and report a bound of 0 (Codex D8b impl P2). The camera prim pose is diagnosis.
        for name, values in (("truth stamp_s", truth.t), ("truth base_pose", truth.base),
                             ("truth lift_m", truth.lift), ("truth pallet_pose", truth.pallet),
                             ("slam_control", control)):
            if values is not None and not np.all(np.isfinite(values)):
                raise ValueError(f"{directory}: non-finite values in {name}")
        if control is not None and (control.ndim != 2 or control.shape[1] != 7 or np.any(np.diff(control[:, 0]) <= 0)):
            raise ValueError(f"{directory}: slam_control.npy is not (t, control x y yaw, truth x y yaw) in time order")
        result = json.loads((directory / "result.json").read_text())
        return cls(directory, meta, index["reads"], index["dwells"], truth,
                   Intrinsics.from_meta(meta["camera"]["intrinsics"]), control, result)

    @property
    def key(self) -> str:
        return str(self.directory)

    def depth(self, read: dict) -> np.ndarray:
        return np.load(self.directory / "pocket_frames" / read["file"])["depth_m"].astype(float)

    def phase_span(self, phases=("approach", "insert")) -> tuple[float, float] | None:
        """First and last tick in these phases: the near-capture anchor to the insertion end."""
        names = self.truth.phases
        codes = [names.index(p) for p in phases if p in names]
        ticks = np.flatnonzero(np.isin(self.truth.phase, codes))
        return None if not len(ticks) else (float(self.truth.t[ticks[0]]), float(self.truth.t[ticks[-1]]))


# Runs used together share one camera and one render cadence (Codex D8b impl P2): these
# fields must agree; the D5 flag, the approach speed and the dwells may differ by design.
CONTRACT_KEYS = ("video", "fps", "mount", "camera", "pallet_geometry", "forklift_urdf", "rear_axle_offset_m",
                 "lift_joint")
# ... and the same code and assets by content, not by path (Codex D8b impl P2): the
# runner's own hashes in result.json.
HASH_KEYS = ("script_sha256", "source_sha256", "pallet_geometry_sha256", "forklift_urdf_sha256",
             "pallet_urdf_sha256")
REQUIRED_SEEDS = (1, 3, 5)
REQUIRED_SPEEDS = (0.055, 0.08, 0.15, 0.30, 0.60)
DWELL_SPEED = 0.055
D5_RUN = (1, 0.08)
REQUIRED_DWELLS = ("start",) + ("gap",) * 7 + ("arrival",)


def is_sha256(value) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def check_hashes(run: Run) -> None:
    """Every content hash present and well formed: missing ones would compare equal
    across runs and pass (Codex D8b impl P2)."""
    for key in HASH_KEYS:
        value = run.result.get(key)
        ok = (isinstance(value, dict) and value and all(is_sha256(v) for v in value.values())
              if key == "source_sha256" else is_sha256(value))
        if not ok:
            raise ValueError(f"{run.key}: {key} missing or malformed in result.json")


def check_contract(runs: list[Run]) -> dict:
    for run in runs:
        check_hashes(run)
    shared = {key: runs[0].meta.get(key) for key in CONTRACT_KEYS}
    shared.update({key: runs[0].result.get(key) for key in HASH_KEYS})
    for run in runs[1:]:
        for key in CONTRACT_KEYS:
            if run.meta.get(key) != shared[key]:
                raise ValueError(f"{run.key}: meta {key} differs from {runs[0].key}; calibrate them apart")
        for key in HASH_KEYS:
            if run.result.get(key) != shared[key]:
                raise ValueError(f"{run.key}: {key} differs from {runs[0].key}; calibrate them apart")
    return shared


def check_seed(run: Run, seed: int) -> None:
    """The run is the seed it is filed under: scene seed and odometry noise seed both
    (Codex D8b impl P1 -- a mislabelled run would complete the matrix)."""
    scene = run.result.get("seed")
    noise = (run.result.get("arguments") or {}).get("slam_noise_seed")
    if scene != seed or noise != seed:
        raise ValueError(f"{run.key}: recorded seed {scene} / slam noise seed {noise}, filed under {seed}")


def manifest(runs_by_seed: dict[int, list[Run]]) -> list[str]:
    """What the plan's run matrix still lacks: seeds 1, 3, 5 x five speeds (the 0.055 run
    with the nine dwells), plus the D5-on run of seed 1 at 0.08 (Codex D8b impl P1)."""
    missing = []
    for seed in REQUIRED_SEEDS:
        runs = runs_by_seed.get(seed, [])
        if not runs:
            missing.append(f"seed {seed}: no runs")
            continue
        for speed in REQUIRED_SPEEDS:
            found = [r for r in runs if not r.meta.get("pocket_check")
                     and r.meta.get("approach_straight_speed_mps") is not None
                     and abs(r.meta["approach_straight_speed_mps"] - speed) < 1e-9]
            if len(found) != 1:
                missing.append(f"seed {seed}: {len(found)} runs at {speed} m/s (D5 off), need 1")
                continue
            kinds = tuple(d["kind"] for d in found[0].dwells)
            if speed == DWELL_SPEED and kinds != REQUIRED_DWELLS:
                missing.append(f"seed {seed}: dwells {kinds}, need {REQUIRED_DWELLS}")
            if speed != DWELL_SPEED and kinds:
                missing.append(f"seed {seed}: dwells in the {speed} m/s run")
        if seed == D5_RUN[0]:
            d5 = [r for r in runs if r.meta.get("pocket_check")
                  and abs((r.meta.get("approach_straight_speed_mps") or 0.0) - D5_RUN[1]) < 1e-9]
            if len(d5) != 1:
                missing.append(f"seed {seed}: {len(d5)} D5-on runs at {D5_RUN[1]} m/s, need 1")
    for seed in sorted(set(runs_by_seed) - set(REQUIRED_SEEDS)):
        missing.append(f"seed {seed}: not in the plan's matrix")
    return missing


def face_distance(truth: Truth, t: float, model: PalletModel) -> float:
    _, p_pc = truth.camera_in_pallet(t)
    return abs(p_pc[0] - face_side(p_pc, model))


@dataclass(frozen=True)
class Motion:
    """Camera-in-pallet motion over the eligibility window [s - IN_RANGE_S, s - TAUS[0]]."""

    min_speed_mps: float  # approach speed along the face normal, the slowest step
    speed_at_s_mps: float
    max_shift_m: float  # from the pose at s
    max_turn_rad: float
    distance_range_m: tuple[float, float]  # camera-face distances the window spans


def window_times(truth: Truth, s: float, back_s: float = IN_RANGE_S) -> np.ndarray | None:
    """The ticks inside [s - back_s, s - TAUS[0]] with both ends exactly, or None when the record
    does not hold the whole window (no tick of slack, impl P2)."""
    lo, hi = s - back_s, s - TAUS[0]
    if truth.t[0] > lo + 1e-9 or truth.t[-1] < hi - 1e-9:
        return None
    inner = truth.t[(truth.t > lo + 1e-9) & (truth.t < hi - 1e-9)]
    return np.r_[max(lo, truth.t[0]), inner, min(hi, truth.t[-1])]


def motion_window(truth: Truth, s: float, model: PalletModel, back_s: float = IN_RANGE_S) -> Motion | None:
    """The window's motion from the truth alone, so whether a frame is used never depends
    on its delay (Codex D8b 3rd P1). ``back_s``: IN_RANGE_S for eligibility; a static
    frame is judged over the whole evaluated range (TAUS[-1])."""
    times = window_times(truth, s, back_s)
    if times is None:
        return None
    poses = [truth.camera_in_pallet(t) for t in times]
    r_s, p_s = truth.camera_in_pallet(s)
    face_x = face_side(p_s, model)
    dist = np.array([abs(p[0] - face_x) for _, p in poses])
    steps = np.diff(times)
    speed = -np.diff(dist)[steps > 1e-12] / steps[steps > 1e-12]
    k = int(np.clip(np.searchsorted(times, s), 1, len(times) - 1))
    return Motion(
        float(np.min(speed)), float(-(dist[k] - dist[k - 1]) / max(times[k] - times[k - 1], 1e-12)),
        max(float(np.linalg.norm(p - p_s)) for _, p in poses),
        max(float(np.arccos(np.clip((np.trace(r_s.T @ r) - 1) / 2, -1.0, 1.0))) for r, _ in poses),
        (float(dist.min()), float(dist.max())),
    )


def frame_residuals(run: Run, read: dict, model: PalletModel, taus: np.ndarray = TAUS) -> np.ndarray:
    isaac = run.depth(read)
    s = read["stamp_s"]
    _, p0 = run.truth.camera_in_pallet(s)
    face_x = face_side(p0, model)
    out = np.full(len(taus), np.nan)
    for j, tau in enumerate(taus):
        r_pc, p_pc = run.truth.camera_in_pallet(s - tau)
        out[j], _ = residual(isaac, run.intrinsics, r_pc, p_pc, model, face_x)
    return out


def usable(read: dict) -> bool:
    return read["result"] == "ok" and read["file"] is not None and read["stamp_s"] is not None


def bias_bound(d_lo: float, d_hi: float, dwell_biases: list[tuple[float, float]]) -> float | None:
    """B over the camera-face distances [d_lo, d_hi] a frame's window spans: the largest
    neighbouring-dwell |b| among the dwell intervals it touches, plus the allowance; None
    when part of it lies outside the dwells (no bound there). Taken over the whole window
    because the pixels were taken at s - tau, not at s (Codex D8b impl P1)."""
    ordered = sorted(dwell_biases, reverse=True)
    if not ordered or d_hi > ordered[0][0] + 1e-9 or d_lo < ordered[-1][0] - 1e-9:
        return None
    worst = None
    for (d_top, b_top), (d_bottom, b_bottom) in zip(ordered, ordered[1:]):
        if d_bottom - 1e-9 <= d_hi and d_lo <= d_top + 1e-9:
            value = max(abs(b_top), abs(b_bottom))
            worst = value if worst is None else max(worst, value)
    return None if worst is None else worst + M_U_M


def analyse_dwells(run: Run, model: PalletModel) -> list[dict]:
    """Per dwell of the 0.055 m/s run: its static frames' r(0), b_k, spread and distance.
    Static = the camera moved <= 0.05 mm and turned <= 5e-5 rad against the pallet over
    the whole window, so r does not depend on the delay."""
    out = []
    for dwell in run.dwells:
        rows, windows = [], []
        for read in run.reads:
            if not usable(read) or not (dwell["trigger_s"] <= read["stamp_s"] <= dwell["end_s"]):
                continue
            # Static over the whole evaluated range [s - 1.0, s + 0.1], as planned: a
            # shorter window would pass a creeping stand (confirmation review P2).
            motion = motion_window(run.truth, read["stamp_s"], model, TAUS[-1])
            if motion is not None:
                windows.append((motion.max_shift_m, motion.max_turn_rad))
            if motion is None or motion.max_shift_m > STATIC_POS_M or motion.max_turn_rad > STATIC_ROT_RAD:
                continue
            r0 = frame_residuals(run, read, model, taus=np.array([0.0]))[0]
            if math.isfinite(r0):
                rows.append((read["index"], r0, face_distance(run.truth, read["stamp_s"], model), motion.max_shift_m))
        record = {"kind": dwell["kind"], "trigger_s": dwell["trigger_s"], "end_s": dwell["end_s"],
                  "static_frames": len(rows), "frames": len(windows),
                  # Each frame window's camera shift and turn against the pallet, so a dwell
                  # short of static frames shows how far it moved (confirmation review P3).
                  "window_shift_m": None if not windows else {"min": min(w[0] for w in windows),
                                                              "median": float(np.median([w[0] for w in windows]))},
                  "window_turn_rad": None if not windows else {"min": min(w[1] for w in windows),
                                                               "median": float(np.median([w[1] for w in windows]))}}
        if rows:
            r = np.array([x[1] for x in rows])
            b = float(np.median(r))
            record.update(bias_m=b, spread_p95_m=float(np.percentile(np.abs(r - b), 95)),
                          distance_m=float(np.median([x[2] for x in rows])), max_shift_m=max(x[3] for x in rows))
        out.append(record)
    return out


def analyse_frames(run: Run, model: PalletModel, dwell_biases: list[tuple[float, float]]) -> list[dict]:
    """① per frame: eligibility (truth only), residual curve, pieces under eps + B."""
    if len(dwell_biases) < 2:
        raise ValueError("the delay needs at least two dwell biases of the seed")
    rows = []
    for read in run.reads:
        if not usable(read):
            continue
        s = read["stamp_s"]
        row = {"index": read["index"], "stamp_s": s, "eligible": False, "phase": run.truth.phase_at(s)}
        motion = motion_window(run.truth, s, model)
        row["distance_m"] = face_distance(run.truth, s, model)
        if motion is None:
            row["reason"] = "window_outside_record"
            rows.append(row)
            continue
        # B over every distance the whole evaluated range [s - 1.0, s + 0.1] spans, so a
        # piece beyond IN_RANGE_S is looked for at its own distance's bound too.
        evaluated = window_times(run.truth, s, TAUS[-1])
        span = None
        if evaluated is not None:
            d_eval = [face_distance(run.truth, t, model) for t in evaluated]
            span = (min(min(d_eval), motion.distance_range_m[0]), max(max(d_eval), motion.distance_range_m[1]))
        bound = None if span is None else bias_bound(*span, dwell_biases)
        row.update(speed_mps=motion.speed_at_s_mps, min_window_speed_mps=motion.min_speed_mps,
                   window_distance_m=list(motion.distance_range_m), evaluated_distance_m=None if span is None else list(span))
        if row["phase"] != "approach":
            row["reason"] = f"phase:{row['phase']}"
        elif motion.min_speed_mps < V_MIN_MPS:
            row["reason"] = "slow_window"
        elif span is None:
            row["reason"] = "record_short"
        elif bound is None:
            row["reason"] = "outside_dwells"
        else:
            row["eligible"] = True
            tol = EPS_M + bound
            r = frame_residuals(run, read, model)
            found = pieces(r, tol)
            # The band is the part up to IN_RANGE_S; anything beyond is a failure, never
            # cut off (confirmation review P1).
            inside = [(lo, hi) for lo, hi in found if hi <= IN_RANGE_S + 1e-9]
            row.update(tol_m=tol, pieces=inside, residual_m=[None if not math.isfinite(x) else x for x in r],
                       out_of_range=[(lo, hi) for lo, hi in found if hi > IN_RANGE_S + 1e-9],
                       edge=any(lo <= TAUS[0] + 1e-9 or hi >= TAUS[-1] - 1e-9 for lo, hi in found))
        rows.append(row)
    return rows


def summarise_run(rows: list[dict]) -> dict:
    eligible = [r for r in rows if r["eligible"]]
    single = [r for r in eligible if len(r["pieces"]) == 1]
    mids = [(r["pieces"][0][0] + r["pieces"][0][1]) / 2 for r in single]
    lows = [lo for r in eligible for lo, _ in r["pieces"]]
    highs = [hi for r in eligible for _, hi in r["pieces"]]
    reasons: dict[str, int] = {}
    for r in rows:
        if not r["eligible"]:
            reasons[r["reason"]] = reasons.get(r["reason"], 0) + 1
    return {
        "frames": len(rows), "eligible": len(eligible), "single_piece": len(single),
        "multi_piece": sum(1 for r in eligible if len(r["pieces"]) > 1),
        "mismatch": sum(1 for r in eligible if not r["pieces"] and not r["out_of_range"]),
        "out_of_range": sum(1 for r in eligible if r["out_of_range"]),
        "edge": sum(1 for r in eligible if r["edge"]),
        "not_eligible": reasons,
        "L_run_s": float(np.median(mids)) if mids else None,
        "band_s": [min(lows), max(highs)] if lows else None,
    }


def consistency_cells(rows: list[dict], dwell_biases: list[tuple[float, float]]) -> list[dict]:
    """(iv): per bin between neighbouring dwells, single-piece midpoints by actual speed
    (a consistency check, about 0.7 mm of systematic bias at 0.055 vs 0.15 m/s)."""
    edges = sorted((d for d, _ in dwell_biases), reverse=True)
    cells = []
    for d_hi, d_lo in zip(edges, edges[1:]):
        slow, fast = [], []
        for r in rows:
            if not r["eligible"] or len(r["pieces"]) != 1 or not (d_lo - 1e-9 <= r["distance_m"] <= d_hi + 1e-9):
                continue
            mid = (r["pieces"][0][0] + r["pieces"][0][1]) / 2
            if SLOW_MPS[0] <= r["speed_mps"] < SLOW_MPS[1]:
                slow.append(mid)
            elif r["speed_mps"] >= FAST_MPS:
                fast.append(mid)
        cell = {"bin_m": [d_hi, d_lo], "slow": len(slow), "fast": len(fast)}
        if slow and fast:
            cell["median_gap_s"] = abs(float(np.median(slow)) - float(np.median(fast)))
        cell["ok"] = len(slow) >= CELL_MIN and len(fast) >= CELL_MIN and cell.get("median_gap_s", 1.0) <= TICK_S + 1e-12
        cells.append(cell)
    return cells


def verdict(dwells: dict, runs: dict, cells: dict, missing: list[str], l_s: float | None) -> dict:
    """The pass rule (i)-(v), fixed before the runs (plan D8b 7판). A submitted set that is
    not the plan's whole matrix is reported, never passed (Codex D8b impl P1)."""
    failures = []
    for seed, records in dwells.items():
        if len(records) != len(REQUIRED_DWELLS):
            failures.append(f"(i) seed {seed}: {len(records)} dwells, need {len(REQUIRED_DWELLS)}")
        for rec in records:
            if rec["static_frames"] < STATIC_MIN:
                failures.append(f"(i) seed {seed} {rec['kind']}@{rec['trigger_s']:.2f}: {rec['static_frames']} static")
            elif abs(rec["bias_m"]) > BIAS_LIMIT_M or rec["spread_p95_m"] > SPREAD_LIMIT_M:
                failures.append(f"(i) seed {seed} {rec['kind']}@{rec['trigger_s']:.2f}: bias/spread")
    l_runs = []
    for name, s in runs.items():
        if s["eligible"] < MIN_ELIGIBLE or s["single_piece"] < MIN_SINGLE:
            failures.append(f"(ii) {name}: {s['eligible']} eligible, {s['single_piece']} single")
        if s["mismatch"]:
            failures.append(f"(ii) {name}: {s['mismatch']} mismatch")
        if s["out_of_range"]:
            failures.append(f"(ii) {name}: {s['out_of_range']} frames with a piece beyond {IN_RANGE_S} s")
        if s["edge"]:
            failures.append(f"(ii) {name}: {s['edge']} at the search edge")
        if s["L_run_s"] is not None:
            l_runs.append(s["L_run_s"])
    if l_runs and max(l_runs) - min(l_runs) > TICK_S + 1e-12:
        failures.append(f"(iii) L_run spread {max(l_runs) - min(l_runs):.4f} s")
    for seed, seed_cells in cells.items():
        for cell in seed_cells:
            if not cell["ok"]:
                failures.append(f"(iv) seed {seed} bin {cell['bin_m']}: {cell}")
    if l_s is None:
        failures.append("(v) no L")
    elif l_s < 0:
        failures.append("(v) L < 0")
    return {"pass": not failures and not missing, "complete": not missing, "missing": missing, "failures": failures}


# ② / ④ -----------------------------------------------------------------------------------
def wrap(a):
    return (np.asarray(a) + math.pi) % (2 * math.pi) - math.pi


def relative(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """a^-1 o b for (x, y, yaw) rows."""
    dx, dy = b[..., 0] - a[..., 0], b[..., 1] - a[..., 1]
    c, s = np.cos(a[..., 2]), np.sin(a[..., 2])
    return np.stack((c * dx + s * dy, -s * dx + c * dy, wrap(b[..., 2] - a[..., 2])), axis=-1)


def thin(values: np.ndarray, every: int = 12) -> list:
    """Every ``every``-th entry and always the last one (the anchor table's end, impl P1)."""
    index = list(range(0, len(values), every))
    if index[-1] != len(values) - 1:
        index.append(len(values) - 1)
    return [float(values[i]) for i in index]


def odometry_tables(control: np.ndarray, start_s: float, end_s: float, max_age_ticks: int = 36) -> dict:
    """④ over [start_s, end_s] (the near-capture anchor to the insertion end): the control
    relative transform C1^-1 C2 against the truth's T1^-1 T2 for the recorded control
    pairs, position e and heading psi apart, as conservative tables (max over ages <= a),
    and from the anchor by elapsed time and distance travelled."""
    rows = control[(control[:, 0] >= start_s - 1e-9) & (control[:, 0] <= end_s + 1e-9)]
    if len(rows) < 2:
        return {"rows": len(rows)}
    t, c, g = rows[:, 0], rows[:, 1:4], rows[:, 4:7]
    by_age = []
    running_e, running_psi = 0.0, 0.0
    for k in range(1, max_age_ticks + 1):
        target = k * TICK_S
        i, j = [], []
        for a in range(len(t)):
            b = int(np.searchsorted(t, t[a] + target - TICK_S / 2))
            if b < len(t) and abs(t[b] - t[a] - target) <= TICK_S / 2:
                i.append(a)
                j.append(b)
        if not i:
            by_age.append({"age_s": target, "pairs": 0, "e_m": None, "psi_rad": None})
            continue
        d = relative(c[i], c[j]) - relative(g[i], g[j])
        running_e = max(running_e, float(np.max(np.hypot(d[:, 0], d[:, 1]))))
        running_psi = max(running_psi, float(np.max(np.abs(wrap(d[:, 2])))))
        by_age.append({"age_s": target, "pairs": len(i), "e_m": running_e, "psi_rad": running_psi})
    d = relative(c[[0] * len(t)], c) - relative(g[[0] * len(t)], g)
    travelled = np.r_[0.0, np.cumsum(np.hypot(*np.diff(g[:, :2], axis=0).T))]
    anchor = {"anchor_s": float(t[0]), "age_s": thin(t - t[0]), "distance_m": thin(travelled),
              "e_m": thin(np.maximum.accumulate(np.hypot(d[:, 0], d[:, 1]))),
              "psi_rad": thin(np.maximum.accumulate(np.abs(wrap(d[:, 2])))),
              "max_age_s": float(t[-1] - t[0]), "max_distance_m": float(travelled[-1])}
    return {"rows": len(rows), "by_age": by_age, "anchor": anchor}


def read_samples(run: Run, model: PalletModel) -> list[dict]:
    """② targets: every read with a valid time in the approach or the insertion, whether
    or not its depth came back (Codex D8b impl P1), with the truth distance and speed."""
    out = []
    for read in run.reads:
        if read.get("time_label", read["result"]) != "ok" or read["stamp_s"] is None:
            continue
        s = read["stamp_s"]
        phase = run.truth.phase_at(s)
        if phase not in ("approach", "insert"):
            continue
        k = int(np.clip(np.searchsorted(run.truth.t, s), 1, len(run.truth.t) - 1))
        t0, t1 = run.truth.t[k - 1], run.truth.t[k]
        speed = (face_distance(run.truth, t0, model) - face_distance(run.truth, t1, model)) / max(t1 - t0, 1e-12)
        out.append({"stamp_s": s, "phase": phase, "distance_m": face_distance(run.truth, s, model), "speed_mps": speed})
    return out


def control_error(control: np.ndarray, frames: list[dict], l_s: float, dwell_biases, span=None) -> dict:
    """② at the global L: the control rear-axle pose at s - L against the truth's, for
    every read sample (read_samples), position and heading apart, split moving / standing
    and counted by distance bin and speed class; samples outside the control record are
    counted, not dropped silently (impl P1/P2). With ``span`` (the near-capture anchor to
    the insertion end), a sample aligned before the anchor shows the capture itself, which
    the control record skips by design (its ticks run inside SensorCapture, and the
    estimate moves ~0.8 mm across them, smoke 1857): it is counted as before_anchor."""
    t = control[:, 0]
    yaw_c, yaw_g = np.unwrap(control[:, 3]), np.unwrap(control[:, 6])
    edges = sorted((d for d, _ in dwell_biases), reverse=True)
    groups: dict[str, list] = {}
    outside, before = 0, 0
    for f in frames:
        at = f["stamp_s"] - l_s
        if span is not None and at < span[0] - 1e-9:
            before += 1
            continue
        k = int(np.searchsorted(t, at))
        exact = k < len(t) and abs(t[k] - at) <= 1e-9
        bracketed = 0 < k < len(t) and t[k] - t[k - 1] <= 1.5 * TICK_S
        if not (exact or bracketed):
            outside += 1  # outside the record or across a gap in it: no interpolation
            continue
        dx = np.interp(at, t, control[:, 1]) - np.interp(at, t, control[:, 4])
        dy = np.interp(at, t, control[:, 2]) - np.interp(at, t, control[:, 5])
        e = (math.hypot(dx, dy), abs(float(wrap(np.interp(at, t, yaw_c) - np.interp(at, t, yaw_g)))))
        speed = f.get("speed_mps")
        motion = "standing" if speed is not None and abs(speed) < 0.005 else "moving"
        band = next((f"{hi:.2f}-{lo:.2f}" for hi, lo in zip(edges, edges[1:])
                     if lo - 1e-9 <= f["distance_m"] <= hi + 1e-9), "outside")
        cls = "unknown" if speed is None else "slow" if speed < SLOW_MPS[1] else "fast" if speed >= FAST_MPS else "mid"
        for key in ("all", f"{f['phase']}:{motion}", f"bin {band}:{cls}"):
            groups.setdefault(key, []).append(e)
    out = {"expected": len(frames), "before_anchor": before, "outside_control": outside}
    for key, values in groups.items():
        a = np.array(values)
        out[key] = {"samples": len(a), "position_m": {"max": float(a[:, 0].max()), "p95": float(np.percentile(a[:, 0], 95))},
                    "yaw_rad": {"max": float(a[:, 1].max()), "p95": float(np.percentile(a[:, 1], 95))}}
    return out


# CLI -------------------------------------------------------------------------------------
def parse_seed_runs(text: str) -> tuple[int, list[Path]]:
    seed, _, dirs = text.partition(":")
    return int(seed), [Path(d) for d in dirs.split(",") if d]


def analyse(seed_runs: list[tuple[int, list[Path]]], model: PalletModel,
            geometry_sha256: str | None = None) -> tuple[dict, dict]:
    """The whole calibration. ``geometry_sha256``: the analysed pallet geometry file's hash,
    checked against the runs' before any frame is evaluated."""
    runs_by_seed: dict[int, list[Run]] = {}
    keys = set()
    for seed, dirs in seed_runs:
        for d in dirs:
            run = Run.load(d)
            if run.key in keys:
                raise ValueError(f"{run.key} given twice")
            check_seed(run, seed)
            keys.add(run.key)
            runs_by_seed.setdefault(seed, []).append(run)
    contract = check_contract([r for runs in runs_by_seed.values() for r in runs])
    if geometry_sha256 is not None and contract["pallet_geometry_sha256"] != geometry_sha256:
        raise ValueError(f"the pallet geometry analysed (sha256 {geometry_sha256[:12]}) is not the one the runs "
                         f"used ({str(contract['pallet_geometry_sha256'])[:12]})")
    dwell_out, run_out, cell_out, frames_out, biases_by_seed = {}, {}, {}, {}, {}
    for seed, runs in runs_by_seed.items():
        with_dwells = [r for r in runs if r.dwells]
        if len(with_dwells) != 1:
            raise ValueError(f"seed {seed}: exactly one run must carry the dwells, found {len(with_dwells)}")
        dwell_out[seed] = analyse_dwells(with_dwells[0], model)
        # |b_k| plus the creep its static frames allowed (a bound, so the sign is dropped).
        biases = [(rec["distance_m"], abs(rec["bias_m"]) + rec["max_shift_m"]) for rec in dwell_out[seed]
                  if "bias_m" in rec]
        biases_by_seed[seed] = biases
        seed_rows = []
        for run in runs:
            rows = analyse_frames(run, model, biases)
            run_out[run.key] = {"seed": seed, **summarise_run(rows)}
            frames_out[run.key] = rows
            seed_rows.extend(rows)
        cell_out[seed] = consistency_cells(seed_rows, biases)
    eligible = [r for rows in frames_out.values() for r in rows if r["eligible"]]
    single = [r for r in eligible if len(r["pieces"]) == 1]
    l_s = float(np.median([(r["pieces"][0][0] + r["pieces"][0][1]) / 2 for r in single])) if single else None
    extra, analysis_missing = {}, []
    for seed, runs in runs_by_seed.items():
        for run in runs:
            span = run.phase_span()
            if run.control is None:
                analysis_missing.append(f"{run.key}: no slam_control.npy (② ④ not computed)")
                continue
            if span is None or l_s is None:
                analysis_missing.append(f"{run.key}: no approach/insert span or no L (② ④ not computed)")
                continue
            errors = control_error(run.control, read_samples(run, model), l_s, biases_by_seed[seed], span)
            odometry = odometry_tables(run.control, *span)
            extra[run.key] = {"control_error": errors, "odometry": odometry}
            # Coverage, not just presence (Codex D8b impl P2).
            if errors["outside_control"] or errors["expected"] == errors["before_anchor"]:
                analysis_missing.append(f"{run.key}: ② {errors['outside_control']} of {errors['expected']} reads "
                                        "outside the control record")
            t_c = run.control[:, 0]
            inside = t_c[(t_c >= span[0] - 1e-9) & (t_c <= span[1] + 1e-9)]
            # The rows inside the span must reach both of its ends: a record that goes on
            # outside it can still miss the anchor itself (Codex D8b impl P2).
            if (not len(inside) or inside[0] > span[0] + TICK_S + 1e-9
                    or inside[-1] < span[1] - TICK_S - 1e-9):
                covered = f"{inside[0]:.3f}-{inside[-1]:.3f}" if len(inside) else "none"
                analysis_missing.append(f"{run.key}: ④ control covers {covered} s of the span "
                                        f"{span[0]:.3f}-{span[1]:.3f} s")
            gaps = np.diff(inside)
            if len(inside) < 2 or np.any(gaps > 1.5 * TICK_S):
                analysis_missing.append(f"{run.key}: ④ control has gaps up to "
                                        f"{float(gaps.max()) if len(gaps) else float('nan'):.3f} s in the span")
            empty = [row["age_s"] for row in odometry.get("by_age", []) if not row["pairs"]]
            if "by_age" not in odometry or empty:
                analysis_missing.append(f"{run.key}: ④ no control pairs at ages {empty[:3]}")
    result = {
        "L_s": l_s,
        "band_s": [min(lo for r in eligible for lo, _ in r["pieces"]), max(hi for r in eligible for _, hi in r["pieces"])]
        if any(r["pieces"] for r in eligible) else None,
        "inputs": {run.key: {"seed": seed, "approach_straight_speed_mps": run.meta.get("approach_straight_speed_mps"),
                             "pocket_check": run.meta.get("pocket_check"), "dwells": [d["kind"] for d in run.dwells],
                             "reads": len(run.reads)} for seed, runs in runs_by_seed.items() for run in runs},
        "contract": contract,
        "dwells": dwell_out, "runs": run_out, "cells": cell_out, "extra": extra,
        "analysis_missing": analysis_missing,
    }
    result["verdict"] = verdict(dwell_out, run_out, cell_out, manifest(runs_by_seed), l_s)
    result["verdict"]["analysis_complete"] = not analysis_missing
    return result, frames_out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--seed-runs", type=parse_seed_runs, action="append", required=True,
                        metavar="SEED:DIR,DIR,...", help="the runs of one seed; the one with dwells gives b_k")
    parser.add_argument("--pallet-geometry", type=Path, default=ROOT / "config/pallet_geometry_epal6.yaml")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    from forklift_core.perception.pallet_geometry import load_pallet_geometry

    model = PalletModel.from_geometry(load_pallet_geometry(args.pallet_geometry))
    result, frames_out = analyse(args.seed_runs, model,
                                 geometry_sha256=hashlib.sha256(args.pallet_geometry.read_bytes()).hexdigest())
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "d8b_calibration.json").write_text(json.dumps(result, indent=1) + "\n")
    (args.output / "d8b_frames.json").write_text(json.dumps(frames_out) + "\n")
    print(json.dumps({k: result[k] for k in ("L_s", "band_s", "verdict")}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
