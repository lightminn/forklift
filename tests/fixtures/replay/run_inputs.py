"""TEST ONLY: the inputs of a replay run (PR 7 7r-2) -- a map bundle with an empty grid over the S10-A field, an S10-A
file on that bundle's map id with a non-zero start, and a synthetic observe record whose ``slam_header`` names that
bundle, map id and start, the chassis model's rear-axle offset and the laser of ``lidar_test.yaml``. Never a
measurement: the record is synthetic and the configs are outside ``config/real/``."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

import make_record as mk

REPO = mk.REPO
HIL_YAML = REPO / "config" / "real" / "s10a_hil.yaml"
START_REAR = (0.4, -0.3, 0.2)  # a non-zero start on the map (x, y, yaw at the rear axle)
GRID_ORIGIN = (-1.5, -3.0, 0.0)
GRID_RES_M = 0.05


def chassis_rear_offset() -> float:
    from forklift_core.live.real_run import chassis_reference
    from forklift_core.plant.real_vehicle import chassis_model_path

    urdf = chassis_model_path(REPO / "config" / "real" / "vehicle.yaml").parent / "forklift.urdf"
    return float(chassis_reference(urdf)[1])


def write_bundle(folder) -> str:
    """An empty survey grid over the S10-A field ([-1, 6] x [-2.5, 2.5] m with margin); returns its map id."""
    from forklift_core.localization import slam_map as sm

    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "map.posegraph").write_bytes(b"TEST ONLY posegraph")
    (folder / "map.data").write_bytes(b"TEST ONLY data")
    data = np.zeros((int(6.0 / GRID_RES_M), int(8.0 / GRID_RES_M)), np.int8)  # rows = y, columns = x
    np.savez(folder / "grid.npz", data=data, origin=np.asarray(GRID_ORIGIN), resolution_m=GRID_RES_M)
    hashes = {name: sm.file_sha256(folder / name) for name in sm.BUNDLE_FILES}
    map_id = sm.map_id_of(hashes)
    (folder / "manifest.json").write_text(json.dumps({"format": sm.FORMAT, "map_id": map_id, "files": hashes,
                                                      "seed": 0}))
    return map_id


def write_s10a(path, map_id: str, start_rear=START_REAR) -> Path:
    """The HIL S10-A file on the bundle's map id with ``start_rear``."""
    text = HIL_YAML.read_text()
    old_map = "map_id: hil-s10a-1"
    old_start = "start_rear: [0.0, 0.0, 0.0]"
    assert old_map in text and old_start in text
    text = text.replace(old_map, f"map_id: {map_id}").replace(old_start, f"start_rear: {list(start_rear)}")
    Path(path).write_text(text)
    return Path(path)


def write_run_inputs(tmp_path, *, start_rear=START_REAR, record_start_rear=None, slam=None, **record_kw):
    """Bundle, S10-A file and record under ``tmp_path``. ``record_start_rear``: the record's own start when it is to
    differ from the S10-A file's; ``slam``: slam_header keys over the consistent ones."""
    from forklift_core.localization.slam_real_scan import base_from_laser
    from forklift_core.localization.start_reloc import base_from_rear

    tmp_path = Path(tmp_path)
    off = chassis_rear_offset()
    bundle = tmp_path / "bundle"
    map_id = write_bundle(bundle)
    s10a = write_s10a(tmp_path / "s10a_replay.yaml", map_id, start_rear)
    rec_start = start_rear if record_start_rear is None else record_start_rear
    header = {"bundle": str(bundle), "map_id": map_id, "rear_offset_m": off,
              "start_pose": [float(v) for v in base_from_rear(tuple(rec_start), off)],
              "base_laser": [float(v) for v in base_from_laser(mk.lidar_config().mount, off)], **(slam or {})}
    record = tmp_path / "observe.jsonl"
    record_kw.setdefault("seconds", 42.0)
    info = mk.write_record(record, slam=header, **record_kw)
    return SimpleNamespace(record=record, s10a=s10a, bundle=bundle, map_id=map_id, lidar=mk.LIDAR_YAML,
                           imu=mk.IMU_YAML, start_rear=tuple(start_rear), rear_offset_m=off, slam_header=header,
                           info=info)


__all__ = ["START_REAR", "chassis_rear_offset", "write_bundle", "write_run_inputs", "write_s10a"]
