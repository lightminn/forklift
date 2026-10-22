"""Top-down CPU animations of the Riibotics demo scenarios, drawn the way their drawings are.

Each scenario is planned on the CPU with run_transport's mission settings for the measured chassis
(riibotics_isaac_scenes: kappa 0.29, clearance 0.10, alignment straight 2.1 m, delivery straight
1.5 m, D7 options): the observation leg to the first candidate that plans, then plan_transport
back to the start. The truck is drawn along the planned poses with a trapezoid speed profile per
driving direction; the time is not a simulation and planning time is left out. A sequence
(line, random) chains its scenes: pallet k is carried with the earlier ones already delivered.

The scenes are the riibotics_isaac_scenes ones in the drawing frame (x right, y up, before they are
turned into the hall), plus the conveyor scenario (2) read off its drawing. The conveyor height is
not modelled: the planner sees the conveyors as floor obstacles with the pick and drop cells left
open (the forks would pass above them), so (2) shows the floor motion only.

    python tools/riibotics_cpu_video.py --output <dir> [--only aisle,line] [--speedup 8] [--fps 30]
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "sim/isaac"))

import riibotics_isaac_scenes as S  # noqa: E402
from bay_planning_sweep import mission_geometry  # noqa: E402
from riibotics_scenario_paths import PALLETS, UP, prop  # noqa: E402

from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402
from forklift_core.planning import Pose2D  # noqa: E402
from forklift_core.planning.geometry import Bounds  # noqa: E402
from forklift_core.planning.pallet_mission import (  # noqa: E402
    D7_PLANNER_OPTIONS,
    PalletSite,
    TransportScenario,
    make_transport_planner_config,
    plan_observation_leg,
    plan_transport,
)

# Speeds of the stages (config/isaac_transport_measured.yaml cruise values) and the lift
SPEED = {"observe": 0.60, "approach": 0.60, "insert": 0.055, "extract": 0.18, "transport": 0.30,
         "withdraw": 0.12, "return_home": 0.60}
ACCEL, LIFT_S = 0.30, 0.20 / 0.06
LOADED = {"extract", "transport"}
# Measured chassis in the rear-axle frame: body -0.17..0.88, half width 0.315; forks 0.93-1.29 at +-0.145
BODY, FORKS = (0.88, 0.17, 0.315), [(0.93, 1.29, c - 0.0275, c + 0.0275) for c in (-0.145, 0.145)]
COL = {"floor": "#ffffff", "zone": "#ececec", "zone_edge": "#9aa0a6", "unload": "#e6eef7", "wall": "#4b5157",
       "rack": "#d9772b", "pallet": "#c8964f", "pallet_edge": "#7a5527", "truck": "#f2c200", "truck_edge": "#5a4a00",
       "fork": "#4b5157", "conveyor": "#c9ced4", "roller": "#9aa3ad", "path": "#7d848c"}


# ---- the conveyor scenario (2), from its drawing (9.0 mm/px, y down) ----

def conveyor_scene():
    pallet = load_pallet_geometry(PALLETS[S.PALLET])
    m = S.PX_M
    pick = PalletSite(339 * m, -400 * m, math.pi)          # forks pointing left into conveyor A
    drop = PalletSite(1490 * m, -185 * m, UP)               # forks pointing up into conveyor B
    tip_x, tip_y = S._drawing(697, 637)
    start = Pose2D(tip_x, tip_y - S.TRUCK_FRONT_M, UP)
    open_m = pallet.overall_width_m / 2 + 0.15               # the cell left open around each pallet (truck 0.36 + 0.10)
    a = (263 * m, 415 * m, -470 * m, -12 * m)               # conveyor A: x0, x1, y0, y1
    b = (1414 * m, 1872 * m, -257 * m, -113 * m)            # conveyor B
    h = 0.70 * S.SCALES["length"]
    props = [prop("conveyor", (a[0] + a[1]) / 2, (pick.y_m + open_m + a[3]) / 2, a[1] - a[0], a[3] - pick.y_m - open_m, 0, h),
             prop("conveyor", (a[0] + a[1]) / 2, (a[2] + pick.y_m - open_m) / 2, a[1] - a[0], pick.y_m - open_m - a[2], 0, h),
             prop("conveyor", (b[0] + drop.x_m - open_m) / 2, (b[2] + b[3]) / 2, drop.x_m - open_m - b[0], b[3] - b[2], 0, h),
             prop("conveyor", (drop.x_m + open_m + b[1]) / 2, (b[2] + b[3]) / 2, b[1] - drop.x_m - open_m, b[3] - b[2], 0, h)]
    bounds = Bounds(a[0] - 1.5, b[1] + 1.0, start.y_m - 2.0, a[3] + 1.0)
    scenario = TransportScenario(0, start, pick, drop, tuple(props), bounds)
    visuals = [S._visual("conveyor", (a[0] + a[1]) / 2, (a[2] + a[3]) / 2, a[1] - a[0], a[3] - a[2], 0.0),
               S._visual("conveyor", (b[0] + b[1]) / 2, (b[2] + b[3]) / 2, b[1] - b[0], b[3] - b[2], 0.0),
               S._visual("pallet", 339 * m, -280 * m, pallet.overall_depth_m, pallet.overall_width_m, math.pi),
               S._visual("pallet", 1610 * m, -185 * m, pallet.overall_depth_m, pallet.overall_width_m, UP)]
    return scenario, visuals


def scenes() -> dict[str, list]:
    """Name -> the scenes carried one after another (each (scenario, visuals))."""
    def two(built):
        return built[0], built[1]
    return {
        "aisle": [two(S.aisle_scene())],
        "conveyor": [conveyor_scene()],
        "line": [two(S.line_scene(k)) for k in range(3)],
        "random": [two(S.random_scene(k)) for k in range(3)],
        "trailer": [two(S.trailer_scene())],
    }


# ---- planning ----

def plan(scenario: TransportScenario) -> list[dict]:
    pallet = load_pallet_geometry(PALLETS[S.PALLET])
    geometry = mission_geometry(S.URDF, pallet, S.RESERVE_M, **S.STRAIGHTS)
    config = make_transport_planner_config(curvature_limit_inv_m=S.KAPPA, clearance_m=S.CLEARANCE_M,
                                           max_expansions=30000, **D7_PLANNER_OPTIONS)
    observe = None
    for extended in (False, True):
        for w in S.observation_waypoints(scenario.pickup):
            leg = plan_observation_leg(scenario, Pose2D(*w), config, geometry=geometry, extended=extended)
            if leg.success:
                observe = leg
                break
        if observe is not None:
            break
    if observe is None:
        raise RuntimeError("no observation candidate plans")
    end = observe.poses[-1]
    mission = plan_transport(scenario, config, geometry=geometry, target_pickup=scenario.pickup,
                             start_rear=Pose2D(float(end[0]), float(end[1]), float(end[2])),
                             return_to=scenario.start_rear)
    if not mission.success:
        raise RuntimeError(f"mission does not plan: {mission.status}")
    legs = [("observe", observe)] + [(n, getattr(mission, n)) for n in S.STAGES]
    return [{"stage": n, "poses": np.asarray(p.poses, float), "directions": np.asarray(p.directions, int)}
            for n, p in legs if p is not None and len(p.poses) > 1]


# ---- timeline ----

def _runs(leg):
    """Runs of one driving direction as pose index ranges: the step into pose i is driven with directions[i]."""
    d, out, first = leg["directions"], [], 1
    for i in range(2, len(d) + 1):
        if i == len(d) or d[i] != d[first]:
            out.append((first - 1, i - 1))
            first = i
    return out


def _trapezoid(length, v):
    """Arc length s(t) and duration of a run driven as a trapezoid at cruise v."""
    ta = v / ACCEL
    if length < v * ta:   # triangle
        ta = math.sqrt(length / ACCEL)
        v = ACCEL * ta
        return (lambda t: 0.5 * ACCEL * min(t, ta) ** 2 + (v * (t - ta) - 0.5 * ACCEL * (t - ta) ** 2 if t > ta else 0)), 2 * ta
    tc = (length - v * ta) / v

    def s(t):
        if t < ta:
            return 0.5 * ACCEL * t * t
        if t < ta + tc:
            return 0.5 * v * ta + v * (t - ta)
        u = t - ta - tc
        return 0.5 * v * ta + v * tc + v * u - 0.5 * ACCEL * u * u
    return s, 2 * ta + tc


def timeline(missions):
    """[(duration, sampler(t) -> (pose, loaded, pallet_world or None, mission index))]."""
    segs = []
    for k, (legs, pickup) in enumerate(missions):
        rel = None
        for leg in legs:
            stage, poses = leg["stage"], leg["poses"]
            for a, b in _runs(leg):
                seg = poses[a:b + 1]
                if len(seg) < 2:
                    continue
                ds = np.hypot(np.diff(seg[:, 0]), np.diff(seg[:, 1]))
                s_cum = np.concatenate([[0.0], np.cumsum(ds)])
                if s_cum[-1] < 1e-6:
                    continue
                s_of, dur = _trapezoid(s_cum[-1], SPEED[stage])
                segs.append((dur, ("drive", seg, s_cum, s_of, stage, k)))
            if stage == "insert":
                end = poses[-1]
                c, s_ = math.cos(end[2]), math.sin(end[2])
                dx, dy = pickup.x_m - end[0], pickup.y_m - end[1]
                rel = (c * dx + s_ * dy, -s_ * dx + c * dy, pickup.yaw_rad - end[2])
                segs.append((LIFT_S, ("hold", end, k, "lift")))
            if stage == "transport":
                segs.append((LIFT_S, ("hold", poses[-1], k, "lower")))
        missions[k] = (legs, pickup, rel)
    return segs


def pose_at(seg, t):
    kind = seg[0]
    if kind == "hold":
        return seg[1], seg[3]
    _, poses, s_cum, s_of, stage, _ = seg
    s = min(max(s_of(t), 0.0), s_cum[-1])
    i = min(np.searchsorted(s_cum, s, side="right") - 1, len(s_cum) - 2)
    f = 0.0 if s_cum[i + 1] == s_cum[i] else (s - s_cum[i]) / (s_cum[i + 1] - s_cum[i])
    p, q = poses[i], poses[i + 1]
    dyaw = math.atan2(math.sin(q[2] - p[2]), math.cos(q[2] - p[2]))
    return np.array([p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1]), p[2] + f * dyaw]), stage


# ---- drawing ----

def _corners(x, y, yaw, front, rear, left, right):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * a - s * b, y + s * a + c * b) for a, b in ((front, left), (front, -right), (-rear, -right), (-rear, left))]


def render(name, chain, out: Path, speedup: float, fps: int) -> dict:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    pallet = load_pallet_geometry(PALLETS[S.PALLET])
    missions = []
    for scenario, _ in chain:
        missions.append((plan(scenario), scenario.pickup))
    segs = timeline(missions)
    # bounds over every scene of the chain
    # The view: every planned pose grown by the truck's length, and every drawn item, inside the
    # planner's bounds plus 0.3 m (walls stand just past them) -- not the whole planning area.
    b = chain[0][0].bounds
    xs, ys = [], []
    for legs, _, _ in missions:
        for leg in legs:
            xs += [leg["poses"][:, 0].min() - 1.5, leg["poses"][:, 0].max() + 1.5]
            ys += [leg["poses"][:, 1].min() - 1.5, leg["poses"][:, 1].max() + 1.5]
    for _, visuals in chain:
        for v in visuals:
            r = math.hypot(v["length_m"], v["width_m"]) / 2
            xs += [v["x_m"] - r, v["x_m"] + r]
            ys += [v["y_m"] - r, v["y_m"] + r]
    pad = 0.3
    x0, x1 = max(min(xs), b.x_min_m - pad), min(max(xs), b.x_max_m + pad)
    y0, y1 = max(min(ys), b.y_min_m - pad), min(max(ys), b.y_max_m + pad)
    w_px = 960
    h_px = int(round(w_px * (y1 - y0) / (x1 - x0) / 2) * 2)
    fig = plt.figure(figsize=(w_px / 100, h_px / 100), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.patch.set_facecolor(COL["floor"])
    static, moving = [], []

    def draw_scene(k):
        for art in static:
            art.remove()
        static.clear()
        scenario, visuals = chain[k]
        for v in visuals:
            kind = v["kind"]
            pts = _corners(v["x_m"], v["y_m"], v["yaw_rad"], v["length_m"] / 2, v["length_m"] / 2, v["width_m"] / 2, v["width_m"] / 2)
            if kind == "floor":
                fc = COL["unload"] if tuple(v["colour"]) == tuple(S.UNLOAD_BLUE) else COL["zone"]
                static.append(ax.add_patch(Polygon(pts, fc=fc, ec=COL["zone_edge"], lw=1.0, zorder=1)))
            elif kind == "wall":
                fc = COL["rack"] if tuple(v["colour"]) == tuple(S.RACK_ORANGE) else COL["wall"]
                static.append(ax.add_patch(Polygon(pts, fc=fc, ec="none", zorder=3)))
            elif kind == "conveyor":
                static.append(ax.add_patch(Polygon(pts, fc=COL["conveyor"], ec=COL["roller"], lw=1.2, zorder=1)))
                n = int(max(v["length_m"], v["width_m"]) / 0.12)
                for i in range(1, n):
                    f = i / n
                    if v["length_m"] >= v["width_m"]:
                        xx = v["x_m"] - v["length_m"] / 2 + f * v["length_m"]
                        static.extend(ax.plot([xx, xx], [v["y_m"] - v["width_m"] / 2, v["y_m"] + v["width_m"] / 2], color=COL["roller"], lw=0.6, zorder=1))
                    else:
                        yy = v["y_m"] - v["width_m"] / 2 + f * v["width_m"]
                        static.extend(ax.plot([v["x_m"] - v["length_m"] / 2, v["x_m"] + v["length_m"] / 2], [yy, yy], color=COL["roller"], lw=0.6, zorder=1))
            elif kind == "pallet":
                static.append(ax.add_patch(Polygon(pts, fc=COL["pallet"], ec=COL["pallet_edge"], lw=1.0, zorder=2)))
        legs = missions[k][0]
        path = np.concatenate([leg["poses"][:, :2] for leg in legs])
        static.extend(ax.plot(path[:, 0], path[:, 1], color=COL["path"], lw=1.0, ls=(0, (3, 3)), zorder=2))

    def pallet_patch(x, y, yaw):
        pts = _corners(x, y, yaw, pallet.overall_depth_m / 2, pallet.overall_depth_m / 2,
                       pallet.overall_width_m / 2, pallet.overall_width_m / 2)
        return ax.add_patch(Polygon(pts, fc=COL["pallet"], ec=COL["pallet_edge"], lw=1.0, zorder=4))

    def truck(pose):
        x, y, yaw = pose
        arts = [ax.add_patch(Polygon(_corners(x, y, yaw, BODY[0], BODY[1], BODY[2], BODY[2]),
                                     fc=COL["truck"], ec=COL["truck_edge"], lw=1.2, zorder=6))]
        for f0, f1, l0, l1 in FORKS:
            c, s = math.cos(yaw), math.sin(yaw)
            pts = [(x + c * a - s * bb, y + s * a + c * bb) for a, bb in ((f1, l1), (f1, l0), (f0, l0), (f0, l1))]
            arts.append(ax.add_patch(Polygon(pts, fc=COL["fork"], ec="none", zorder=5)))
        return arts

    frame_dt = speedup / fps
    total = sum(d for d, _ in segs)
    proc = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgba", "-s", f"{w_px}x{h_px}",
                             "-r", str(fps), "-i", "-", "-an", "-c:v", "libx264", "-preset", "slow", "-crf", "26",
                             "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(out)], stdin=subprocess.PIPE)
    shown_k, carried = None, {}
    t_global, si, t_in = 0.0, 0, 0.0
    frames = 0
    end_hold = int(1.5 * fps)
    while si < len(segs):
        dur, seg = segs[si]
        k = seg[-1] if seg[0] == "drive" else seg[2]
        if k != shown_k:
            draw_scene(k)
            shown_k = k
        pose, stage = pose_at(seg, t_in)
        rel = missions[k][2]
        pickup = missions[k][1]
        if seg[0] == "hold":
            loaded = seg[3] == "lower"   # lifting: not loaded before, lowering: loaded until it ends
            if seg[3] == "lift":
                loaded = t_in >= dur
        else:
            loaded = stage in LOADED
        before_pick = seg[0] == "drive" and stage in ("observe", "approach", "insert")
        for art in moving:
            art.remove()
        moving.clear()
        if loaded and rel is not None:
            c, s = math.cos(pose[2]), math.sin(pose[2])
            moving.append(pallet_patch(pose[0] + c * rel[0] - s * rel[1], pose[1] + s * rel[0] + c * rel[1], pose[2] + rel[2]))
        elif before_pick or (seg[0] == "hold" and seg[3] == "lift"):
            moving.append(pallet_patch(pickup.x_m, pickup.y_m, pickup.yaw_rad))
        else:   # delivered
            dest = chain[k][0].destination
            moving.append(pallet_patch(dest.x_m, dest.y_m, dest.yaw_rad))
        moving.extend(truck(pose))
        fig.canvas.draw()
        proc.stdin.write(np.asarray(fig.canvas.buffer_rgba()).tobytes())
        frames += 1
        t_in += frame_dt
        while si < len(segs) and t_in > segs[si][0]:
            t_in -= segs[si][0]
            si += 1
    for _ in range(end_hold):
        proc.stdin.write(np.asarray(fig.canvas.buffer_rgba()).tobytes())
        frames += 1
    proc.stdin.close()
    proc.wait()
    plt.close(fig)
    gear = sum(int(np.sum(np.diff(leg["directions"][1:]) != 0)) for legs, _, _ in missions for leg in legs)
    return {"scene": name, "missions": len(chain), "sim_s": round(total, 1), "video_s": round(frames / fps, 1),
            "frames": frames, "size_px": [w_px, h_px], "speedup": speedup, "gear_changes_in_legs": gear}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--only")
    parser.add_argument("--speedup", type=float, default=8.0)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    only = set(args.only.split(",")) if args.only else None
    records = []
    for name, chain in scenes().items():
        if only and name not in only:
            continue
        rec = render(name, chain, args.output / f"{name}.mp4", args.speedup, args.fps)
        print(json.dumps(rec, ensure_ascii=False), flush=True)
        records.append(rec)
    (args.output / "videos.json").write_text(json.dumps(records, indent=1, ensure_ascii=False) + "\n")


if __name__ == "__main__":
    main()
