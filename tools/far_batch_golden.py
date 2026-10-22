"""The batch far recognition's fixed-input record (docs/plans/2026-10-22-live-far-recognition.md task 11).

Splitting the far recognition's per-frame work from its tracker update (so a live run can do the first off the
control thread) must leave the batch run's results and records as they were. ``run_all`` drives the CPU rig's
scenarios -- an oracle leg, a leg ended by a loss and the leg after it, a learned leg falling back to the geometric
proposals, a zone narrowed for a later leg, a stop the forks do not fit -- through ``FarRuntime``'s public calls
and returns every observation, record, summary and saved array (as hashes). The fixture next to the test was made
with the code of ec4bece, before the split:

    python tools/far_batch_golden.py --out tests/fixtures/far_batch_golden_ec4bece.json

The model's wall times are not compared (they are measured, not computed); their count is.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import sys
import tempfile
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools import instance_rig  # noqa: E402
from tools import recognition_scenes as rs  # noqa: E402
from tools import recognition_truth as rt  # noqa: E402

DT = 1 / 120
PALLET = (4.2, 0.05, 0.02)


def load_runtime(root: Path = ROOT):
    spec = importlib.util.spec_from_file_location("far_recognition_runtime", root / "sim/isaac/far_recognition_runtime.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TruthModel:
    """The learned model stand-in of tests/unit/test_far_recognition_runtime.py: the rendered pallet's mask."""

    def __init__(self, score=0.9, fail_after=None):
        self.score, self.calls, self.fail_after, self._ids = score, 0, fail_after, None

    def propose(self, scene):
        from forklift_core.perception.instance_proposals import InstanceProposal

        self.calls += 1
        if self.fail_after is not None and self.calls > self.fail_after:
            raise RuntimeError("model gone")
        _, ids = self._ids
        return [InstanceProposal("l0", ids == 0, self.score, "learned")] if (ids == 0).any() else []


def _render(R, base):
    epal = rs.known_spec("epal6")
    x, y, yaw = R.relative(base, PALLET)
    placed = [rt.placed_boxes(rt.instance_boxes(epal, ()), rt.Placement(x, y, yaw, 0.0))]
    out = instance_rig.render_instances(placed)
    return out.scene, out.instance_ids


def _leg(n=480):
    v = np.where(np.arange(n) < 120, 0.3 * np.arange(n) / 120,
                 np.where(np.arange(n) < 360, 0.3, 0.3 * (n - np.arange(n)) / 120))
    return np.concatenate(([0.0], np.cumsum(v) * DT))[:n]


def _runtime(R, *, spacing=0.3725, **kw):
    epal = rs.known_spec("epal6")
    return R.FarRuntime(fork_spacing=spacing, pallet_depth_m=epal.overall_depth_m, pallet_shape="epal6",
                        zone_centre_xy=PALLET[:2], zone_radius_m=0.6,
                        camera_x_m=instance_rig.PERCEPTION_CAMERA.xyz_m[0], dt=DT, **kw)


def _drive(R, run, xs, lag, model=None, ticks=None):
    for tick in (range(len(xs)) if ticks is None else ticks):
        base = (float(xs[tick]), 0.0, 0.0)
        run.record_step(tick, tick * DT, base, base, PALLET)
        if run.due(tick, observing=True) and tick >= lag:
            scene, ids = _render(R, (float(xs[tick - lag]), 0.0, 0.0))
            if model is not None:
                model._ids = (scene, ids)
            run.drive_frame(tick, tick * DT, scene.rgb, scene.depth_m, ids, scene.intrinsics, scene.base_from_optical)


def _stop(R, run, base, stamp_s, model=None):
    scene, ids = _render(R, base)
    if model is not None:
        model._ids = (scene, ids)
    scene = replace(scene, stamp_ns=int(round(stamp_s * 1e9)))
    return run.stop(scene, ids, initial_time_s=0.0, control_base=base, odom_base=base, truth_base=base,
                    truth_pallet=PALLET)


def _plain(value):
    return json.loads(json.dumps(value, default=float))


def _summary(run) -> dict:
    s = _plain(run.summary())
    timing = s.get("proposer", {}).get("model_ms")
    if timing is not None:
        s["proposer"]["model_ms"] = {"n": timing["n"]}  # measured wall times: the count only
    return s


def _files(out: Path) -> dict:
    files = {}
    for path in sorted(out.glob("*.npz")):
        with np.load(path) as z:
            files[path.name] = {k: hashlib.sha256(np.ascontiguousarray(z[k]).tobytes()
                                                  + str(z[k].dtype).encode() + str(z[k].shape).encode()).hexdigest()
                                for k in sorted(z.files)}
    return files


def _stops(pairs) -> list:
    return [{"observation": _plain(asdict(obs)), "record": _plain(rec)} for obs, rec in pairs]


def scenario_oracle(R, out: Path) -> dict:
    run = _runtime(R, lag_steps=11, out_dir=out)
    xs = _leg()
    _drive(R, run, xs, 11)
    stops = [_stop(R, run, (float(xs[-1]), 0.0, 0.0), len(xs) * DT + 0.1)]
    return {"stops": _stops(stops), "summary": _summary(run)}


def scenario_leg_lost(R, out: Path) -> dict:
    run = _runtime(R, lag_steps=11, out_dir=out)
    xs = _leg()
    _drive(R, run, xs, 11, ticks=range(200))
    still = (float(xs[199]), 0.0, 0.0)
    scene, ids = _render(R, still)
    empty = np.full_like(ids, -1)
    for k in range(200, 480):
        run.record_step(k, k * DT, still, still, PALLET)
        if run.due(k, observing=True):
            run.drive_frame(k, k * DT, scene.rgb, scene.depth_m, empty if k < 360 else ids, scene.intrinsics,
                            scene.base_from_optical)
    stops = [_stop(R, run, still, 480 * DT + 0.1)]
    return {"stops": _stops(stops), "summary": _summary(run)}


def scenario_learned_fallback(R, out: Path) -> dict:
    model = TruthModel(0.9, fail_after=8)
    source = R.LearnedSource(model)
    run = _runtime(R, lag_steps=11, out_dir=out, proposer=source, identity_threshold=0.5,
                   proposer_record={"kind": "learned", "tau": 0.5})
    xs = _leg()
    _drive(R, run, xs[:240], 11, model=model)
    stops = [_stop(R, run, (float(xs[239]), 0.0, 0.0), 240 * DT + 0.1, model=model)]
    _drive(R, run, xs, 11, model=model, ticks=range(240, 480))
    stops.append(_stop(R, run, (float(xs[-1]), 0.0, 0.0), 480 * DT + 0.1, model=model))
    return {"stops": _stops(stops), "summary": _summary(run), "fallbacks": list(source.fallbacks),
            "calls": model.calls}


def scenario_learned(R, out: Path) -> dict:
    model = TruthModel(0.9)
    source = R.LearnedSource(model)
    run = _runtime(R, lag_steps=11, out_dir=out, proposer=source, identity_threshold=0.5,
                   proposer_record={"kind": "learned", "tau": 0.5})
    xs = _leg()
    _drive(R, run, xs, 11, model=model)
    stops = [_stop(R, run, (float(xs[-1]), 0.0, 0.0), 480 * DT + 0.1, model=model)]
    return {"stops": _stops(stops), "summary": _summary(run), "calls": model.calls}


def scenario_zone(R, out: Path) -> dict:
    run = _runtime(R, out_dir=out)
    xs = _leg()
    _drive(R, run, xs, 0)
    stops = [_stop(R, run, (float(xs[-1]), 0.0, 0.0), len(xs) * DT + 0.1)]
    run.set_zone((PALLET[0] + 0.4, PALLET[1]), 0.3)
    _drive(R, run, xs, 0)
    stops.append(_stop(R, run, (float(xs[-1]), 0.0, 0.0), len(xs) * DT + 0.1))
    return {"stops": _stops(stops), "summary": _summary(run)}


def scenario_not_pickable(R, out: Path) -> dict:
    run = _runtime(R, spacing=0.29, out_dir=out)
    xs = _leg()
    _drive(R, run, xs, 0)
    stops = [_stop(R, run, (float(xs[-1]), 0.0, 0.0), len(xs) * DT + 0.1)]
    return {"stops": _stops(stops), "summary": _summary(run)}


SCENARIOS = {"oracle": scenario_oracle, "leg_lost": scenario_leg_lost, "learned": scenario_learned,
             "learned_fallback": scenario_learned_fallback, "zone": scenario_zone, "not_pickable": scenario_not_pickable}


def run_all(R) -> dict:
    out = {}
    for name, fn in SCENARIOS.items():
        with tempfile.TemporaryDirectory() as tmp:
            record = fn(R, Path(tmp) / "recognition")
            record["files"] = _files(Path(tmp) / "recognition")
        out[name] = _plain(record)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--root", type=Path, default=ROOT, help="the checkout whose sim/isaac/far_recognition_runtime.py runs")
    args = ap.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(run_all(load_runtime(args.root)), indent=1, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
