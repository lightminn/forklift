#!/usr/bin/env python3
"""Apply a fitted depth-noise config to noise-free recognition scenes (stage 1B).

scale 1 is the official noise population; any other scale is a sensitivity variant.
"""

import argparse
import hashlib
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from forklift_core.perception.depth_noise import NoiseModel, apply_noise
from forklift_core.perception.recognition_dataset import load_recognition_sample, write_recognition_scene
from tools.generate_recognition_scenes import noise_seed


def apply_to_scene(src_dir: Path, out_root: Path, config: Path, scale: float) -> None:
    sample = load_recognition_sample(src_dir)
    if sample.meta.get("noise") is not None:
        raise ValueError(f"{src_dir.name} already carries noise")
    scene = sample.input.scene
    model = NoiseModel.from_yaml(config, seed=noise_seed(sample.meta["seed"]), target_fx=scene.intrinsics.fx,
                                 scale=scale)
    noisy, ids = apply_noise(scene, sample.instance_ids, model)
    meta = dict(sample.meta)
    meta["noise"] = {**model.to_json(), "config": config.name, "scale": scale,
                     "config_sha256": hashlib.sha256(config.read_bytes()).hexdigest(),
                     "role": "official" if scale == 1.0 else "sensitivity"}
    write_recognition_scene(out_root / src_dir.name, noisy, ids, origin=sample.origin, meta=meta,
                            truth=sample.truth, instruction=sample.instruction)


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--src", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--noise-config", type=Path, required=True)
    p.add_argument("--scale", type=float, default=1.0)
    a = p.parse_args(argv)
    for scene_json in sorted(a.src.glob("*/scene.json")):
        apply_to_scene(scene_json.parent, a.out, a.noise_config, a.scale)


if __name__ == "__main__":
    main()
