#!/usr/bin/env python3
"""Apply one level of a depth-noise config to noise-free recognition scenes (stage 1B).

The role comes from the config: a provisional config gives provisional scenes; an
official config gives official scenes at scale 1 and sensitivity scenes otherwise.
"""

import argparse
import hashlib
import sys

import yaml
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from forklift_core.perception.depth_noise import NoiseModel, apply_noise
from forklift_core.perception.recognition_dataset import load_recognition_sample, write_recognition_scene
from tools.generate_recognition_scenes import noise_seed


def role_of(config: Path, scale: float) -> str:
    role = yaml.safe_load(config.read_text(encoding="utf-8")).get("role")
    if role == "official":
        return "official" if scale == 1.0 else "sensitivity"
    return "provisional"


def apply_to_scene(src_dir: Path, out_root: Path, config: Path, scale: float, level: str = "mid") -> None:
    sample = load_recognition_sample(src_dir)
    if sample.meta.get("noise") is not None:
        raise ValueError(f"{src_dir.name} already carries noise")
    scene = sample.input.scene
    model = NoiseModel.from_yaml(config, seed=noise_seed(sample.meta["seed"]), target_fx=scene.intrinsics.fx,
                                 level=level, scale=scale)
    noisy, ids = apply_noise(scene, sample.instance_ids, model)
    meta = dict(sample.meta)
    meta["noise"] = {**model.to_json(), "config": config.name, "scale": scale, "level": level,
                     "config_sha256": hashlib.sha256(config.read_bytes()).hexdigest(),
                     "role": role_of(config, scale)}
    write_recognition_scene(out_root / src_dir.name, noisy, ids, origin=sample.origin, meta=meta,
                            truth=sample.truth, instruction=sample.instruction)


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--src", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--noise-config", type=Path, required=True)
    p.add_argument("--scale", type=float, default=1.0)
    p.add_argument("--level", default="mid")
    a = p.parse_args(argv)
    for scene_json in sorted(a.src.glob("*/scene.json")):
        apply_to_scene(scene_json.parent, a.out, a.noise_config, a.scale, a.level)


if __name__ == "__main__":
    main()
