"""Replay the stored pocket evaluations under two plane-candidate budgets.

The regression closeout G4 asks of any detector change (docs/plans/
2026-09-21-perception-detection-closeout.md): every scene of the stored
evaluation runs, detected with that run's recorded parameters except
``max_plane_candidates``, once at each budget. It reports how many
observations change, and -- so that "nothing changed" is not vacuous -- how
many scenes extract more planes than the smaller budget allows.

    python -m tools.compare_plane_budget --before 5 --after 6 artifacts/*_pocket_eval_*
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
from collections.abc import Sequence
from pathlib import Path

from forklift_core.perception import pocket_detector as det
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from forklift_core.perception.scene_dataset import load_scene_sample


def _observation(result) -> dict:
    record = json.loads(
        json.dumps(dataclasses.asdict(result.observation), default=list)
    )
    return record


def compare_run(run_dir: Path, before: int, after: int) -> dict:
    run = json.loads((run_dir / "run.json").read_text())
    prior = load_pallet_prior(Path(run["prior_path"]))
    params = DetectorParams(**run["params"])
    dataset = Path(run["dataset_dir"]) / "scenes"
    changed, more_planes, rows = [], 0, []
    for scene_id in run["scene_ids"]:
        sample = load_scene_sample(dataset / scene_id)
        first = detect_pockets(
            sample.input,
            prior,
            dataclasses.replace(params, max_plane_candidates=before),
        )
        second = detect_pockets(
            sample.input, prior, dataclasses.replace(params, max_plane_candidates=after)
        )
        a, b = _observation(first), _observation(second)
        if second.diagnostics.candidate_plane_count > before:
            more_planes += 1
        if a != b:
            changed.append(scene_id)
            rows.append(
                {
                    "scene": scene_id,
                    "before": {"status": a["status"], "reason": a["reason"]},
                    "after": {"status": b["status"], "reason": b["reason"]},
                }
            )
    import hashlib

    return {
        # A current-code A/B on the stored scenes: today's detector with the run's
        # recorded parameters, only the budget changed. Not a replay of the
        # code that produced the stored observations.
        "effective_params": dataclasses.asdict(params),
        "before_budget": before,
        "after_budget": after,
        "prior_path": run["prior_path"],
        "prior_sha256_now": hashlib.sha256(
            Path(run["prior_path"]).read_bytes()
        ).hexdigest(),
        "prior_sha256_recorded": run.get("prior_sha256"),
        "detector_sha256_now": hashlib.sha256(
            Path(det.__file__).read_bytes()
        ).hexdigest(),
        "run": run_dir.name,
        "scenes": len(run["scene_ids"]),
        "changed": changed,
        "changes": rows,
        "scenes_with_more_planes": more_planes,
        "recorded_budget": params.max_plane_candidates,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--before", type=int, required=True)
    parser.add_argument("--after", type=int, required=True)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args(argv)
    results = [compare_run(run, args.before, args.after) for run in args.runs]
    total = sum(r["scenes"] for r in results)
    changed = sum(len(r["changed"]) for r in results)
    for r in results:
        print(
            f"{r['run']:45s} scenes {r['scenes']:3d} changed {len(r['changed']):3d} "
            f"more-than-{args.before}-planes {r['scenes_with_more_planes']:3d} {r['changes']}"
        )
    print(
        f"# total {total} observations, {changed} changed ({args.before} -> {args.after})"
    )
    if args.json:
        args.json.write_text(json.dumps(results, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
