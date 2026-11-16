#!/usr/bin/env python3
"""G8 entry: one M1-M3 measurement day's record -> the registration's per-trial values (docs/design/2026-11-15-m-analyze-
entry.md 2, 5; docs/plans/2026-11-15-m1-m3-registration.md).

Reads the teleop bridge record, the IMU record, the truth file (or --no-truth), the drive profile and optionally the
field log; prints one ``m_analyze/1`` JSON line on stdout (with the synthetic marks first when any input is
synthetic). A refusal -- a synthetic input without --allow-synthetic, an input that is not the same run, a profile
that is not the record's -- prints ``{"result": "refused", "why": ...}`` and exits 2. Judgement thresholds are the
registration's; this tool only says which values can be judged.

usage: m_analyze.py --record TELEOP.jsonl --imu IMU.jsonl --profile config/profiles/m1_m3/<name>.yaml
                    (--truth TRUTH.jsonl | --no-truth) (--mount G3.json | --up X,Y,Z)
                    [--field-log FIELD.jsonl] [--vehicle config/real/vehicle.yaml] [--trial N] [--allow-synthetic]
                    [--block-record OTHER.jsonl ...]

--truth with method "tape" (registration revision 4, 11.2): a truth/1 file whose header names
body_point "left_rear_axle_end", frame "start_tape_line" and sigma {"pos_m": 0.02}, and one row per trial
{"t": "tape", "trial": N, "s_cm": distance along the start line, "y_cm": offset to its left}. Each valid trial then
carries "rev4" (the 11.3 cm/degree values with their bands); tools/m_summary.py makes the day's table from the outputs.

--block-record: another teleop record of the same IMU stretch (the same day's G3, M1, M2, M3 or q-step records);
repeat it for each. Their standing windows enter the in-trial bias-slope fit (registration revision 3, 10.1); no
values are computed for their trials.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from forklift_core.live.synthetic import SyntheticRefused  # noqa: E402
from forklift_core.plant import m_entry  # noqa: E402
from forklift_core.plant.real_vehicle import load_real_vehicle  # noqa: E402

VEHICLE = REPO / "config" / "real" / "vehicle.yaml"


def _rows(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def _ident(path: Path) -> dict:
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def _up(args) -> tuple[list[float], str]:
    if args.up is not None:
        v = [float(x) for x in args.up.split(",")]
        if len(v) != 3:
            raise ValueError("--up takes three numbers: X,Y,Z")
        return v, "argument"
    g3 = json.loads(Path(args.mount).read_text(encoding="utf-8"))
    v = g3.get("up_omega") or g3.get("u_omega") or g3.get("up")
    if not (isinstance(v, list) and len(v) == 3):
        raise ValueError("the G3 file names no up axis (up_omega)")  # G3 output schema: entry design 7 item 9
    return [float(x) for x in v], "g3"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--record", type=Path, required=True)
    ap.add_argument("--imu", type=Path, required=True)
    ap.add_argument("--profile", type=Path, required=True)
    truth = ap.add_mutually_exclusive_group(required=True)
    truth.add_argument("--truth", type=Path)
    truth.add_argument("--no-truth", action="store_true")
    mount = ap.add_mutually_exclusive_group(required=True)
    mount.add_argument("--mount", type=Path)
    mount.add_argument("--up")
    ap.add_argument("--field-log", type=Path)
    ap.add_argument("--vehicle", type=Path, default=VEHICLE)
    ap.add_argument("--trial", type=int)
    ap.add_argument("--allow-synthetic", action="store_true")
    ap.add_argument("--block-record", type=Path, action="append", default=[])
    args = ap.parse_args(argv)

    def refused(why: str) -> int:
        print(json.dumps({"result": "refused", "why": why}))
        return 2

    try:
        import yaml

        profile = yaml.safe_load(args.profile.read_bytes())
        up, up_source = _up(args)
        cfg = load_real_vehicle(args.vehicle)
        rec, imu = _rows(args.record), _rows(args.imu)
        truth_rows = None if args.truth is None else _rows(args.truth)
        field_rows = None if args.field_log is None else _rows(args.field_log)
        block_rows = [_rows(b) for b in args.block_record]
    except (OSError, ValueError, json.JSONDecodeError) as e:
        return refused(f"input: {e}")
    try:
        out = m_entry.analyze(rec, imu, truth_rows, field_rows, profile, cfg, up,
                              allow_synthetic=args.allow_synthetic, no_truth=args.no_truth, trial=args.trial,
                              block_rows=block_rows)
    except SyntheticRefused as e:
        return refused(f"synthetic: {e}")
    except m_entry.Refused as e:
        return refused(e.why)
    inputs = {"record": _ident(args.record), "imu": _ident(args.imu), "profile": _ident(args.profile),
              "vehicle": {**_ident(args.vehicle), "calibration_id": cfg.calibration_id},
              "truth": None if args.truth is None else _ident(args.truth),
              "field_log": None if args.field_log is None else _ident(args.field_log),
              "up": {"vector": up, "source": up_source}, "block_records": [_ident(b) for b in args.block_record]}
    header_cfg = (rec[0].get("config") or {}) if rec else {}
    if header_cfg:
        # the record names its configuration (teleop bridge header, PR #530): a different vehicle or profile file is
        # refused rather than silently analysed with other numbers
        veh = header_cfg.get("vehicle") or {}
        if veh.get("calibration_id") not in (None, cfg.calibration_id):
            return refused("vehicle calibration_id differs from the record's")
        prof = header_cfg.get("profile") or {}
        if prof.get("sha256") not in (None, inputs["profile"]["sha256"]):
            return refused("profile file differs from the record's")
    else:
        inputs["config_check"] = "record header names no configuration (older record): not compared"
    result = {**out, "inputs": inputs}
    print(json.dumps(result, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
