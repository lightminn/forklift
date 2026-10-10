"""Plan D8c 3판: the measured near-field bounds the tracker reads (no Isaac).

docs/plans/2026-10-04-lidar-obstacle-map.md, "D8c 3판 구현 설계". From the D8b third
matrix -- tools/d8b_calibration.py's d8b_calibration.json, tools/d8b_observations.py's
d8b_observations.json and the recorded runs themselves -- this writes one JSON file:

  latency      L, the band [L_lo, L_hi], the read delay A and the render period, the last
               two checked on every recorded read (all must agree);
  observation  per 0.1 m camera-face bin and source (front, roof): the largest absolute
               lateral, along, yaw, wall and (front) width error over both pockets and
               every run, with the valid count; a bin with no valid observation is
               unbounded. Vertical is left out: the front's centre height is the prior's
               and the roof's comes from the pallet model;
  odometry     the relative odometry error of every pair of control ticks inside each
               run's [near-capture anchor, insertion end] (sliding windows), rear-axle
               position e and heading psi apart, as the running maximum over age, every
               tick to 0.5 s and every 0.1 s after (a lookup takes the first grid age at
               or above the query: the running maximum makes that conservative);
  near_capture the near-capture observation the runner used against the truth at both
               ends of its capture window; its bound is the larger of the front bound
               over its distance interval (interval_bound) and the captures' own errors.

Bounds are sample maxima of one synthetic camera model under the recorded contract
(1 mm depth, no added noise, render cadence, video off), not proven limits; the file
carries that contract and the hashes of everything it was computed from.

    python tools/export_near_field_bounds.py --calibration <dir>/d8b_calibration.json \
        --observations <dir>/d8b_observations.json --output config/near_field_bounds_measured.json
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
from itertools import pairwise
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from forklift_core.perception.near_field_bounds import (  # noqa: E402
    COMPONENTS,
    SCHEMA,
    interval_bound,
)
from tools import d8b_calibration as CAL  # noqa: E402

TICK_S = CAL.TICK_S
FINE_UNTIL_S = 0.5  # every tick up to here, then COARSE_S
COARSE_S = 0.1
CADENCE_TOLERANCE_S = (
    1e-4  # off the control tick grid (recorded stamps sit within ~1e-8 s)
)
ODOMETRY_MATCH = 1e-9  # the calibration's 0.3 s table against this computation
RAW_FILES = (
    "slam_control.npy",
    "pocket_frames/truth.npz",
    "pocket_frames/index.json",
    "result.json",
)
SOURCES = ("front", "roof")
BIN_M = 0.1


def artifact_path(path) -> str:
    """A recorded path from its artifacts/ component on: the tracked file names the run,
    not the host it was read on; the replay matches runs by this suffix."""
    text = str(path)
    index = text.find("/artifacts/")
    return text[index + 1 :] if index >= 0 else text


def sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


# Observation bounds ------------------------------------------------------------------------
def observation_bounds(summary: dict, key: str = "bins") -> dict:
    """Per source, the bins (high edge first) with the largest absolute error of each
    component over both pockets and every run; ``key`` picks the clean bins or the noisy
    reference (``noisy_reference.bins``). Runs cover different distance ranges (a run
    starts at its own near capture, the D5 run stops inside the pallet), so the bins are
    the union keyed by their edges; every run must use the same 0.1 m edges."""
    per_edges: dict[tuple, list] = {}
    for run in summary["runs"].values():
        bins = run
        for part in key.split("."):
            bins = bins[part]
        for cell in bins:
            hi, lo = cell["bin_m"]
            if not math.isclose(hi - lo, BIN_M, abs_tol=1e-6) or not math.isclose(
                lo / BIN_M, round(lo / BIN_M), abs_tol=1e-6
            ):
                raise ValueError(f"bin {cell['bin_m']} is not on the {BIN_M} m grid")
            per_edges.setdefault((round(hi, 3), round(lo, 3)), []).append(cell)
    if not per_edges:
        raise ValueError("no bins")
    out = {}
    for source in SOURCES:
        rows = []
        for edges in sorted(per_edges, key=lambda e: -e[0]):
            cell = {"bin_m": list(edges), "valid": 0}
            comp = {"lateral_m": 0.0, "along_m": 0.0, "yaw_rad": 0.0, "wall_m": 0.0}
            if source == "front":
                comp["width_m"] = 0.0
            for run_cell in per_edges[edges]:
                entry = run_cell.get(source)
                if not entry or entry.get("unbounded") or not entry.get("valid"):
                    continue
                cell["valid"] += int(entry["valid"])
                comp["yaw_rad"] = max(comp["yaw_rad"], entry["yaw_rad"]["max"])
                for side in ("left", "right"):
                    e = entry[side]
                    comp["lateral_m"] = max(comp["lateral_m"], e["lateral_m"]["max"])
                    comp["along_m"] = max(comp["along_m"], e["along_m"]["max"])
                    walls = [
                        e[m]["max"]
                        for m in ("wall_m_moving", "wall_m_standing")
                        if e.get(m)
                    ]
                    if not walls:
                        raise ValueError(
                            f"{source} bin {edges}: valid observations without a wall error"
                        )
                    comp["wall_m"] = max(comp["wall_m"], *walls)
                    if source == "front":
                        comp["width_m"] = max(comp["width_m"], e["width_m"]["max"])
            if cell["valid"]:
                cell.update(comp)
            else:
                cell["unbounded"] = True
            rows.append(cell)
        out[source] = rows
    return out


# Odometry -----------------------------------------------------------------------------------
def odometry_running_max(
    control: np.ndarray, start_s: float, end_s: float
) -> tuple[np.ndarray, np.ndarray]:
    """e[k], psi[k]: the largest relative-transform error over every pair of control rows k
    ticks apart inside [start_s, end_s] (index 0 is age 0). The rows must sit on the
    control tick grid without gaps: a gap would silently drop the pairs that span it."""
    rows = control[(control[:, 0] >= start_s - 1e-9) & (control[:, 0] <= end_s + 1e-9)]
    if len(rows) < 2:
        raise ValueError("fewer than two control rows in the span")
    if abs(rows[0, 0] - start_s) > TICK_S / 2 or abs(rows[-1, 0] - end_s) > TICK_S / 2:
        raise ValueError("the control record does not reach both ends of the span")
    ticks = (rows[:, 0] - rows[0, 0]) / TICK_S
    if np.any(np.abs(ticks - np.round(ticks)) > 0.05) or np.any(
        np.diff(np.round(ticks)) != 1
    ):
        raise ValueError("the control record is not one row per tick across the span")
    c, g = rows[:, 1:4], rows[:, 4:7]
    n = len(rows)
    e = np.zeros(n)
    psi = np.zeros(n)
    for k in range(1, n):
        d = CAL.relative(c[:-k], c[k:]) - CAL.relative(g[:-k], g[k:])
        e[k] = float(np.max(np.hypot(d[:, 0], d[:, 1])))
        psi[k] = float(np.max(np.abs(CAL.wrap(d[:, 2]))))
    return e, psi


def anchor_table(control: np.ndarray, start_s: float, end_s: float) -> dict:
    """The calibration's anchor table recomputed: the control-against-truth error from
    the span's first row, running maxima, every 12th row and the last (CAL.thin)."""
    rows = control[(control[:, 0] >= start_s - 1e-9) & (control[:, 0] <= end_s + 1e-9)]
    t, c, g = rows[:, 0], rows[:, 1:4], rows[:, 4:7]
    first = [0] * len(t)
    d = CAL.relative(c[first], c) - CAL.relative(g[first], g)
    travelled = np.r_[0.0, np.cumsum(np.hypot(*np.diff(g[:, :2], axis=0).T))]
    return {
        "age_s": CAL.thin(t - t[0]),
        "distance_m": CAL.thin(travelled),
        "e_m": CAL.thin(np.maximum.accumulate(np.hypot(d[:, 0], d[:, 1]))),
        "psi_rad": CAL.thin(np.maximum.accumulate(np.abs(CAL.wrap(d[:, 2])))),
    }


def merge_max(tables: list[np.ndarray]) -> np.ndarray:
    out = np.zeros(max(len(t) for t in tables))
    for t in tables:
        out[: len(t)] = np.maximum(out[: len(t)], t)
    return np.maximum.accumulate(out)


def age_grid(e: np.ndarray, psi: np.ndarray) -> dict:
    """The running maxima on the stored grid: every tick to FINE_UNTIL_S, then every
    COARSE_S; the last grid age is the longest pair (the table's range)."""
    n = len(e)
    fine = int(round(FINE_UNTIL_S / TICK_S))
    step = int(round(COARSE_S / TICK_S))
    index = list(range(1, min(fine, n - 1) + 1))
    index += list(range(fine + step, n, step))
    if index[-1] != n - 1:
        index.append(n - 1)
    return {
        "age_s": [round(k * TICK_S, 9) for k in index],
        "e_m": [float(e[k]) for k in index],
        "psi_rad": [float(psi[k]) for k in index],
    }


# Cadence ------------------------------------------------------------------------------------
def cadence(reads_by_run: dict) -> dict:
    """The render period and read delay in ticks over every timed approach/insert read:
    one value each or refused (the D8e budget's tau assumes a fixed cadence)."""
    periods, delays, total = set(), set(), 0
    worst = 0.0
    for reads in reads_by_run.values():
        kept = [
            r
            for r in reads
            if r.get("phase") in ("approach", "insert")
            and r.get("result") == "ok"
            and r.get("stamp_s") is not None
            and not r.get("duplicate")
        ]
        stamps = [r["stamp_s"] for r in kept]
        for value, bucket in [(b - a, periods) for a, b in pairwise(stamps)] + [
            (r["read_s"] - r["stamp_s"], delays) for r in kept
        ]:
            ticks = round(value / TICK_S)
            worst = max(worst, abs(value - ticks * TICK_S))
            bucket.add(int(ticks))
        total += len(kept)
    if worst > CADENCE_TOLERANCE_S:
        raise ValueError(
            f"a stamp interval or read delay is {worst * 1e3:.3f} ms off the tick grid"
        )
    if len(periods) != 1 or len(delays) != 1:
        raise ValueError(
            f"the cadence is not fixed: periods {sorted(periods)} ticks, delays {sorted(delays)} ticks"
        )
    return {
        "reads": total,
        "render_period_ticks": periods.pop(),
        "read_delay_ticks": delays.pop(),
    }


# Depth frames -------------------------------------------------------------------------------
def frame_manifest(run) -> dict:
    """Every depth frame file the index names (the replay's input): their relative paths,
    count and one sha256 over "path sha256" lines in index order. A replay recomputes it
    and refuses a run whose frames differ (Codex D8c 3판 4th re-review P2)."""
    names = [f"pocket_frames/{r['file']}" for r in run.reads if r.get("file")]
    if len(set(names)) != len(names):
        raise ValueError(f"{run.key}: the index names a depth frame twice")
    lines = "".join(f"{n} {sha256(run.directory / n)}\n" for n in names)
    return {
        "files": names,
        "count": len(names),
        "sha256": hashlib.sha256(lines.encode()).hexdigest(),
    }


# Near capture -------------------------------------------------------------------------------
def near_capture_check(run: CAL.Run, geometry, front_rows: list[dict]) -> dict:
    """The runner's near-capture observation against the truth at both ends of its capture
    window [t_before_capture, anchor] (the truck is meant to stand; the truth's motion over
    the window is reported as the timing uncertainty), with the front bound of its bin."""
    from types import SimpleNamespace

    from tools import d8b_observations as OBS

    perception = run.result["perception"]
    o = perception["pocket_observation"]
    obs = SimpleNamespace(
        status=o["status"],
        insertion_yaw_rad=o["insertion_yaw_rad"],
        **{
            side: SimpleNamespace(
                center_m=o[side]["center_m"],
                width_m=o[side]["width_m"],
                height_m=o[side]["height_m"],
            )
            for side in ("left", "right")
        },
    )
    span = run.phase_span()
    ends = (float(perception["t_before_capture_s"]), span[0])
    worst = {
        "lateral_m": 0.0,
        "along_m": 0.0,
        "wall_m": 0.0,
        "width_m": 0.0,
        "yaw_rad": 0.0,
    }
    distance = None
    for t in ends:
        e = OBS.pocket_errors(obs, OBS.truth_in_base(run, t, geometry))
        for side in ("left", "right"):
            worst["lateral_m"] = max(worst["lateral_m"], abs(e[side]["lateral_m"]))
            worst["along_m"] = max(worst["along_m"], abs(e[side]["along_m"]))
            worst["width_m"] = max(worst["width_m"], abs(e[side]["width_m"]))
            worst["wall_m"] = max(
                worst["wall_m"], *(abs(v) for v in e[side]["wall_m"].values())
            )
        worst["yaw_rad"] = max(worst["yaw_rad"], abs(e["yaw_rad"]))
        model = CAL.PalletModel.from_geometry(geometry)
        distance = CAL.face_distance(run.truth, t, model)
    _, p0 = CAL.pose_at(run.truth.t, run.truth.base, ends[0])
    _, p1 = CAL.pose_at(run.truth.t, run.truth.base, ends[1])
    bound = interval_bound(front_rows, distance)
    if bound is None:
        raise ValueError(
            f"{run.key}: the near capture at {distance:.3f} m has no front bound"
        )
    excess = {k: max(0.0, worst[k] - bound[k]) for k in COMPONENTS}
    return {
        "run": artifact_path(run.key),
        "window_s": list(ends),
        "truth_motion_m": float(np.linalg.norm(p1 - p0)),
        "distance_m": distance,
        "error": worst,
        "front_bound": bound,
        "excess": excess,
    }


def near_capture_bound(checks: list[dict]) -> dict:
    """The near-capture observation's bound: per component the larger of every capture's
    front bound (interval_bound at its true distance) and every capture's own error -- the near captures are
    samples of the same quantity (a sample maximum is exceeded by new samples: three
    captures of the third matrix sit 0.04 mm and 0.03 mrad above their bin)."""
    return {
        k: max(max(c["front_bound"][k], c["error"][k]) for c in checks)
        for k in COMPONENTS
    }


# Assembly -----------------------------------------------------------------------------------
def build(
    calibration: dict,
    calibration_sha: str,
    observations: dict,
    observations_sha: str,
    runs: list,
    geometry,
    geometry_path: Path,
    commit: str | None,
    calibration_mtime: float,
) -> dict:
    if not calibration.get("verdict", {}).get("pass"):
        raise ValueError("the calibration did not pass")
    if observations.get("calibration_sha256") != calibration_sha:
        raise ValueError("the observation summary belongs to another calibration")
    if not observations.get("complete") or not observations.get(
        "calibration_matrix_complete"
    ):
        raise ValueError("the observation summary is not the complete matrix")
    keys = sorted(r.key for r in runs)
    if keys != sorted(calibration["inputs"]) or keys != sorted(observations["runs"]):
        raise ValueError(
            "the runs differ from the calibration's or the observations' inputs"
        )
    from tools import d8b_observations as OBS

    for run in runs:
        # The run is the one calibrated: contract, hashes, seeds, speed, dwells and reads
        # (the D8b observation tool's check), and the geometry and URDF read here are the
        # ones it used, by content.
        OBS.check_calibration(run, run.key, calibration)
        OBS.check_assets(run, geometry_path, ROOT / run.meta["forklift_urdf"])
    observation = observation_bounds(observations)
    rate = cadence({r.key: r.reads for r in runs})
    tables_e, tables_psi, files = [], [], {}
    for run in runs:
        span = run.phase_span()
        e, psi = odometry_running_max(run.control, *span)
        # Numerical consistency with the calibration's own ④ tables (computed from the
        # record then) -- evidence, not proof, that this is the record calibrated: the
        # running maxima to 0.3 s and the anchor table over the whole span (every 0.1 s).
        odometry = calibration["extra"][run.key]["odometry"]
        mine_e, mine_psi = np.maximum.accumulate(e), np.maximum.accumulate(psi)
        for k, row in enumerate(odometry["by_age"], start=1):
            if (
                abs(mine_e[k] - row["e_m"]) > ODOMETRY_MATCH
                or abs(mine_psi[k] - row["psi_rad"]) > ODOMETRY_MATCH
            ):
                raise ValueError(
                    f"{run.key}: the control record does not reproduce the calibration's "
                    f"odometry table at {row['age_s']:.4f} s"
                )
        mine = anchor_table(run.control, *span)
        theirs = odometry["anchor"]
        for name in ("age_s", "distance_m", "e_m", "psi_rad"):
            if len(mine[name]) != len(theirs[name]) or any(
                abs(a - b) > ODOMETRY_MATCH
                for a, b in zip(mine[name], theirs[name], strict=True)
            ):
                raise ValueError(
                    f"{run.key}: the control record does not reproduce the calibration's "
                    f"anchor table ({name})"
                )
        # No raw file -- the four records and every depth frame the replay reads --
        # changed after the calibration was written.
        frames = frame_manifest(run)
        newer = [
            n
            for n in (*RAW_FILES, *frames["files"])
            if (run.directory / n).stat().st_mtime > calibration_mtime
        ]
        if newer:
            raise ValueError(f"{run.key}: {newer[:3]} changed after the calibration")
        tables_e.append(e)
        tables_psi.append(psi)
        files[run.key] = {name: sha256(run.directory / name) for name in RAW_FILES}
        files[run.key]["frames"] = {k: frames[k] for k in ("count", "sha256")}
    odometry = age_grid(merge_max(tables_e), merge_max(tables_psi))
    checks = [near_capture_check(run, geometry, observation["front"]) for run in runs]
    contract = calibration["contract"]
    return {
        "schema": SCHEMA,
        "condition": {
            "depth": "recorded float32 rounded to 1 mm, no added noise (D8b ③ contract)",
            "video": contract["video"],
            "fps": contract["fps"],
            "mount": contract["mount"],
            "camera": contract["camera"],
            "pallet_geometry": contract["pallet_geometry"],
            "forklift_urdf": contract["forklift_urdf"],
            "rear_axle_offset_m": contract["rear_axle_offset_m"],
            "source_sha256": {k: contract[k] for k in contract if k.endswith("sha256")},
            "speeds_mps": "0.055 (nine dwells), 0.08, 0.15, 0.30, 0.60; seeds 1, 3, 5",
            "note": "sample maxima of one synthetic camera model, not proven bounds; not for the real D435i",
        },
        "latency": {
            "align_s": calibration["L_s"],
            "band_s": calibration["band_s"],
            "max_s": calibration["band_s"][1],
            "read_delay_s": rate["read_delay_ticks"] * TICK_S,
            "render_period_s": rate["render_period_ticks"] * TICK_S,
            "cadence": rate,
        },
        "observation": {
            "bin_m": 0.1,
            "distance": "camera optical centre to the front face along the pallet axis",
            **observation,
        },
        # The same chain on sigma(z) = 0.0036 z^2 noisy depth (one noise seed): the plan's
        # boundary for D8g, never read by the tracker.
        "observation_noisy_reference": observation_bounds(
            observations, "noisy_reference.bins"
        ),
        "odometry": {
            "tick_s": TICK_S,
            "reference": "rear axle centre",
            "windows": "every pair inside each run's "
            "[near-capture anchor, insertion end]",
            "lookup": "first grid age at or above the query; "
            "beyond the last age unbounded",
            **odometry,
        },
        "near_capture": {"bound": near_capture_bound(checks), "captures": checks},
        "provenance": {
            "calibration": {"sha256": calibration_sha},
            "observations": {"sha256": observations_sha},
            "runs": {artifact_path(k): v for k, v in files.items()},
            "tool_sha256": sha256(Path(__file__)),
            "commit": commit,
        },
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--observations", type=Path, required=True)
    parser.add_argument(
        "--pallet-geometry",
        type=Path,
        default=ROOT / "config/pallet_geometry_epal6.yaml",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    from forklift_core.perception.pallet_geometry import load_pallet_geometry

    calibration = json.loads(args.calibration.read_text())
    observations = json.loads(args.observations.read_text())
    runs = [CAL.Run.load(Path(k)) for k in calibration["inputs"]]
    try:
        commit = subprocess.run(
            ["git", "-C", str(ROOT), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        commit = None
    out = build(
        calibration,
        sha256(args.calibration),
        observations,
        sha256(args.observations),
        runs,
        load_pallet_geometry(args.pallet_geometry),
        args.pallet_geometry,
        commit,
        args.calibration.stat().st_mtime,
    )
    out["provenance"]["calibration"]["path"] = artifact_path(args.calibration)
    out["provenance"]["observations"]["path"] = artifact_path(args.observations)
    args.output.write_text(json.dumps(out, indent=1) + "\n")
    print(
        f"wrote {args.output}: {len(out['odometry']['age_s'])} ages to {out['odometry']['age_s'][-1]:.2f} s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
