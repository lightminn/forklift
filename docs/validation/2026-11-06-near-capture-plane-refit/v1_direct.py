"""The 100 stored v1 scenes re-detected by one source tree: observation JSON per scene (compared across trees)."""
import dataclasses, json, sys
from pathlib import Path

src, out = Path(sys.argv[1]), Path(sys.argv[2])
sys.path.insert(0, str(src / "src"))
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from forklift_core.perception.scene_dataset import load_scene_sample

ART = Path("/home/light/school/2-2/embedded-design/forklift/artifacts")
rows = {}
for name in ("20260912T170442Z_pocket_eval_dev_02", "20260912T170558Z_pocket_eval_eval_01"):
    run = json.loads((ART / name / "run.json").read_text())
    prior = load_pallet_prior(Path(run["prior_path"]))
    params = DetectorParams(**run["params"])
    for scene_id in run["scene_ids"]:
        sample = load_scene_sample(Path(run["dataset_dir"]) / "scenes" / scene_id)
        rows[f"{name}/{scene_id}"] = json.dumps(dataclasses.asdict(detect_pockets(sample.input, prior, params).observation),
                                                sort_keys=True, default=list)
out.write_text(json.dumps(rows, sort_keys=True))
print(src.name, len(rows))
