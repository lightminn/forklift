"""The planned paths and the destination rings, drawn into the video frames and never into the scene
(docs/plans/2026-11-14-path-display-perception-leak.md 2).

A display prim in the stage is seen by every camera, the perception cameras included (#509 3: the planned-path line
changed the near capture's depth), and Isaac 5.1 offers no per-render-product exclusion. So by default (``overlay``)
the runner keeps the display here and every video sink projects it onto its own copy of the frame. The state is module
level on purpose: ``run()`` binds no new name that ``drive()`` would read, so a boundary replay of an older record finds
nothing missing; a replay's Isaac adapters never call ``add_path``, so the list stays empty and nothing here reads a
camera. Camera poses and intrinsics come from the Isaac camera objects, which are not recording proxies.

Modes (``--path-display``): ``overlay`` (default), ``off`` (no display at all), and two diagnostic modes reachable only
with the environment variable ``FORKLIFT_PATH_DISPLAY_DIAGNOSTIC``: ``stage`` (the display prims as before the fix, to
reproduce older records) and ``both`` (prims and overlay, to measure how the two line up). A record carries the mode as
``arguments.path_display``, except ``stage``, which records nothing, as every record before the fix.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np

MODES = ("overlay", "stage", "off")
DIAGNOSTIC_ENV = "FORKLIFT_PATH_DISPLAY_DIAGNOSTIC"
DIAGNOSTIC_VALUES = ("stage", "both")
LINE_Z_M = 0.012  # as the BasisCurves were (scene.add_path_display)
RING_Z_M = 0.006  # as the destination ring was (scene.add_destination)
RING_RADIUS_M = 0.63  # the middle of the ring's 0.59-0.67 m band
RING_SEGMENTS = 96
NEAR_M = 0.05
MAX_POINTS = 200  # live_runtime.PlanFeed._thin's cap
HISTORY = 256  # physics steps of pose history per moving camera (about 2 s at 120 Hz)

_items: dict[str, dict] = {}
_intrinsics: dict[str, tuple] = {}
_moving: dict[str, object] = {}
_history: dict[str, list] = {}
_clock = {"fn": None}


@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


def reset() -> None:
    """Forget every line, camera and pose (a new run, a test)."""
    _items.clear()
    _intrinsics.clear()
    _moving.clear()
    _history.clear()
    _clock["fn"] = None


def set_clock(fn) -> None:
    """The simulation clock note_moving stamps poses with: the one a camera frame's ``rendering_time`` is on
    (isaacsim.core.nodes' sim time), so a frame can be matched to the pose it was rendered at."""
    _clock["fn"] = fn


def isaac_sim_clock():
    """isaacsim.core.nodes' sim time, the clock of Camera.get_current_frame()["rendering_time"] (None outside Isaac)."""
    try:
        from isaacsim.core.nodes.bindings import _isaacsim_core_nodes
    except ImportError:
        return None
    interface = _isaacsim_core_nodes.acquire_interface()
    return interface.get_sim_time


def effective_mode(cli_mode: str, environ=None) -> str:
    """The mode a run uses: the command line's, unless the diagnostic variable names ``stage`` or ``both``.
    ``stage`` on the command line without the variable is refused (diagnostic only, never in an operating or
    evaluation run)."""
    environ = os.environ if environ is None else environ
    diagnostic = environ.get(DIAGNOSTIC_ENV)
    if diagnostic is not None and diagnostic not in DIAGNOSTIC_VALUES:
        raise ValueError(f"{DIAGNOSTIC_ENV} is one of {', '.join(DIAGNOSTIC_VALUES)}, not {diagnostic!r}")
    if cli_mode not in MODES:
        raise ValueError(f"--path-display is one of {', '.join(MODES)}, not {cli_mode!r}")
    if cli_mode == "stage" and diagnostic != "stage":
        raise ValueError(f"--path-display stage is diagnostic only (it puts the display line into the perception "
                         f"cameras' renders, #509): it needs {DIAGNOSTIC_ENV}=stage")
    return diagnostic or cli_mode


def draws_stage(mode: str) -> bool:
    return mode in ("stage", "both")


def draws_overlay(mode: str) -> bool:
    return mode in ("overlay", "both")


def recorded_value(mode: str):
    """``arguments.path_display`` in a record: nothing for ``stage`` (as every record before the fix), else the mode."""
    return None if mode == "stage" else mode


def empty() -> bool:
    return not _items


def items() -> dict:
    return _items


def thin(points: np.ndarray, max_points: int = MAX_POINTS) -> np.ndarray:
    """At most ``max_points`` of ``points``, evenly by index, both ends kept."""
    points = np.asarray(points, dtype=float)
    if len(points) <= max_points:
        return points
    keep = np.unique(np.round(np.linspace(0, len(points) - 1, max_points)).astype(int))
    return points[keep]


def _colour255(colour) -> tuple:
    return tuple(int(round(255 * min(max(float(c), 0.0), 1.0))) for c in colour)


def add_path(name: str, poses, colour) -> None:
    """A planned path (poses of x, y[, ...]) under ``name``, replacing that name's earlier line."""
    xy = np.array([(float(p[0]), float(p[1])) for p in poses], dtype=float).reshape(-1, 2)
    points = np.column_stack([xy, np.full(len(xy), LINE_Z_M)])
    _items[name] = {"points": thin(points), "colour": _colour255(colour), "closed": False, "width": 1.0}


def add_ring(name: str, x_m: float, y_m: float, colour) -> None:
    """A destination ring at (x, y), drawn as its middle circle, twice as wide as a path."""
    a = np.linspace(0.0, 2 * np.pi, RING_SEGMENTS, endpoint=False)
    points = np.column_stack([x_m + RING_RADIUS_M * np.cos(a), y_m + RING_RADIUS_M * np.sin(a),
                              np.full(len(a), RING_Z_M)])
    _items[name] = {"points": points, "colour": _colour255(colour), "closed": True, "width": 2.0}


def quaternion_wxyz_from_matrix(r: np.ndarray) -> np.ndarray:
    r = np.asarray(r, dtype=float)
    w = np.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = np.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2
    y = np.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2
    z = np.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2
    x = np.copysign(x, r[2, 1] - r[1, 2])
    y = np.copysign(y, r[0, 2] - r[2, 0])
    z = np.copysign(z, r[1, 0] - r[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def matrix_from_quaternion_wxyz(q) -> np.ndarray:
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def project_polyline(points_world, position, quaternion_wxyz, k: Intrinsics, closed: bool = False) -> list:
    """Pixel runs of a world polyline seen by a camera at ``position`` with ROS optical axes ``quaternion_wxyz``
    (camera to world; z forward, x right, y down). Every segment is cut at the plane z = NEAR_M; a polyline that
    passes behind it splits into several runs."""
    pts = np.asarray(points_world, dtype=float)
    if closed and len(pts) > 1:
        pts = np.vstack([pts, pts[:1]])
    if len(pts) < 2:
        return []
    r = matrix_from_quaternion_wxyz(quaternion_wxyz)
    cam = (pts - np.asarray(position, dtype=float)) @ r  # world -> camera: R^T (p - t), row form
    runs, run = [], []

    def pix(p):
        return (k.fx * p[0] / p[2] + k.cx, k.fy * p[1] / p[2] + k.cy)

    for a, b in zip(cam[:-1], cam[1:]):
        za, zb = a[2], b[2]
        if za < NEAR_M and zb < NEAR_M:
            if len(run) > 1:
                runs.append(run)
            run = []
            continue
        if za < NEAR_M:  # enters: start at the plane
            a = a + (b - a) * (NEAR_M - za) / (zb - za)
        elif zb < NEAR_M:  # leaves: end at the plane
            b = a + (b - a) * (NEAR_M - za) / (zb - za)
        if not run:
            run = [pix(a)]
        run.append(pix(b))
        if zb < NEAR_M:
            if len(run) > 1:
                runs.append(run)
            run = []
    if len(run) > 1:
        runs.append(run)
    return runs


def line_width(image_width: int) -> int:
    """3 px at 1280 px across, in proportion, at least 1 px."""
    return max(1, int(round(3 * image_width / 1280)))


def _camera_key(camera) -> str:
    return str(getattr(camera, "prim_path", id(camera)))


def _intrinsics_for(camera, width: int, height: int) -> Intrinsics:
    key = _camera_key(camera)
    if key not in _intrinsics:
        k = np.asarray(camera.get_intrinsics_matrix(), dtype=float)
        _intrinsics[key] = (float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2]),
                            tuple(int(v) for v in camera.get_resolution()))
    fx, fy, cx, cy, (res_w, res_h) = _intrinsics[key]
    sx, sy = width / res_w, height / res_h  # an image sent smaller than the camera renders (the live stream)
    return Intrinsics(fx * sx, fy * sy, cx * sx, cy * sy)


def register_moving(camera) -> None:
    """A camera that moves during the run (chase, robot, live): its pose is kept per physics step (note_moving), so
    a frame rendered some steps before it is written is drawn from the pose it was rendered at."""
    _moving[_camera_key(camera)] = camera


def note_moving(time_s: float | None = None) -> None:
    """Keep every moving camera's pose at this physics step (nothing while there is no line to draw, or without a
    clock when no time is given)."""
    if not _items or not _moving:
        return
    if time_s is None:
        if _clock["fn"] is None:
            return
        time_s = _clock["fn"]()
    for key, camera in _moving.items():
        position, q = camera.get_world_pose(camera_axes="ros")
        h = _history.setdefault(key, [])
        h.append((float(time_s), np.asarray(position, dtype=float), np.asarray(q, dtype=float)))
        if len(h) > HISTORY:
            del h[: len(h) - HISTORY]


def pose_history_span_s(camera) -> float:
    h = _history.get(_camera_key(camera)) or []
    return h[-1][0] - h[0][0] if h else 0.0


def _pose_at(camera, frame_time_s):
    h = _history.get(_camera_key(camera)) if frame_time_s is not None else None
    if h:
        times = np.array([e[0] for e in h])
        i = int(np.argmin(np.abs(times - float(frame_time_s))))
        return h[i][1], h[i][2]
    return camera.get_world_pose(camera_axes="ros")


def compose(rgb, camera, *, frame_time_s: float | None = None):
    """``rgb`` with every line drawn, on a copy; ``rgb`` itself when there is nothing to draw or no frame (nothing is
    read then). ``frame_time_s``: the frame's render time, for a moving camera's kept pose."""
    if rgb is None or not _items or camera is None:
        return rgb
    import cv2

    out = np.array(rgb[:, :, :3], dtype=np.uint8, copy=True)
    height, width = out.shape[:2]
    k = _intrinsics_for(camera, width, height)
    position, q = _pose_at(camera, frame_time_s)
    base = line_width(width)
    for item in _items.values():
        for run in project_polyline(item["points"], position, q, k, closed=item["closed"]):
            pts = np.round(np.asarray(run)).clip(-1e5, 1e5).astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(out, [pts], False, item["colour"], max(1, int(round(base * item["width"]))), cv2.LINE_AA)
    return out


__all__ = [
    "MODES", "DIAGNOSTIC_ENV", "LINE_Z_M", "RING_Z_M", "RING_RADIUS_M", "NEAR_M", "MAX_POINTS", "Intrinsics",
    "reset", "effective_mode", "draws_stage", "draws_overlay", "recorded_value", "empty", "items", "thin",
    "add_path", "add_ring", "project_polyline", "line_width", "register_moving", "note_moving",
    "pose_history_span_s", "compose", "set_clock", "isaac_sim_clock", "quaternion_wxyz_from_matrix",
    "matrix_from_quaternion_wxyz",
]
