#!/usr/bin/env python3
"""Render the recognition catalogue on the CPU rig into schema-v2 scene dirs."""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from forklift_core.perception.depth_noise import NoiseModel, apply_noise
from forklift_core.perception.fork_fit import ForkSetup, load_site_defaults
from forklift_core.perception.recognition_dataset import write_recognition_scene
from tools import recognition_scenes as rs
from tools.recognition_truth import scene_truths


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--cases", default=",".join(rs.CASES))
    p.add_argument("--limit", type=int, default=None, help="first N recipes per case and shape")
    p.add_argument("--noise", choices=("default", "none"), default="default")
    p.add_argument("--digest-only", action="store_true")
    a = p.parse_args(argv)
    recipes = rs.catalogue()
    if a.digest_only:
        print(json.dumps({"catalogue_sha256": rs.catalogue_digest(recipes), "numpy": np.__version__}))
        return
    cases, counts = set(a.cases.split(",")), {}
    site = load_site_defaults(Path(__file__).resolve().parents[1] / "config/site_load_defaults.yaml",
                              {k: rs.known_spec(k) for k in ("epal6", "t11_06")})
    for recipe in recipes:
        if recipe.case not in cases:
            continue
        key = (recipe.case, recipe.shape_key)
        counts[key] = counts.get(key, 0) + 1
        if a.limit is not None and counts[key] > a.limit:
            continue
        instr = recipe.instruction
        holder = instr.target if instr.target is not None else 0

        def load_for(k, inst):
            if k == holder and instr.load is not None:
                return instr.load
            return site.get(recipe.shape_key) if inst.spec.known else None

        truths, rendered = scene_truths(
            [(i.spec, i.placement, i.damage, load_for(k, i)) for k, i in enumerate(recipe.instances)],
            ForkSetup(recipe.fork_spacing_m), distractors=recipe.distractors,
            layers=[i.layer for i in recipe.instances])
        scene, ids, noise = rendered.scene, rendered.instance_ids, None
        if a.noise == "default":
            noise_seed = int(np.random.SeedSequence(recipe.seed).spawn(3)[2].generate_state(1)[0])
            model = NoiseModel(seed=noise_seed)
            scene, ids = apply_noise(scene, ids, model)
            noise = model.to_json()
        write_recognition_scene(
            a.out / recipe.scene_id, scene, ids, origin="rig_render_pose",
            meta={"scene_id": recipe.scene_id, "case": recipe.case, "shape_key": recipe.shape_key, "split": recipe.split,
                  "seed": recipe.seed, "rank": recipe.rank, "render_source": "rig", "noise": noise,
                  "fork_spacing_m": recipe.fork_spacing_m, "touching": recipe.touching},
            truth={"instances": [t.to_json() for t in truths],
                   "instruction_truth": {"kind": instr.kind, "target": instr.target}},
            instruction={"present": instr.kind != "none", "x_m": instr.x_m, "y_m": instr.y_m,
                         "radius_m": instr.radius_m, "layer": instr.layer,
                         "load": None if instr.load is None else instr.load.to_json()},
        )


if __name__ == "__main__":
    main()
