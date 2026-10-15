#!/usr/bin/env python3
"""Capture static D435i depth frames for the depth-noise measurement (stage 1B amendment 1).

Saves the raw Z16 values (no rounding to millimetres), the device's depth scale and
every setting that changes the noise: stream mode, intrinsics, firmware and SDK,
exposure, gain, emitter, preset, warm-up time, frame numbers and timestamps, plus
what was in front of the camera and how its distance was measured. RealSense
post-processing filters are host-side and opt-in; none are applied here.

Repeated frames of one static view estimate the temporal noise only: their mean
still holds the real surface shape and any fixed sensor bias, which need a flat
reference and an independent distance (docs/validation/2026-10-16-d435i-noise-capture-procedure.md).

  python tools/capture_d435i_noise.py --out <dir> --label wall_2m --distance-m 2.003 \
      --distance-method "laser rangefinder" --surface wall [--roi x0,y0,x1,y1] [--frames 30]
"""

import argparse
import hashlib
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

OPTIONS = ("visual_preset", "enable_auto_exposure", "exposure", "gain", "emitter_enabled", "laser_power")


@dataclass(frozen=True)
class Frame:
    z16: np.ndarray
    number: int
    timestamp_ms: float
    domain: str


def check_setup(setup: dict, *, width: int, height: int) -> None:
    for name in ("label", "distance_method", "surface"):
        if not isinstance(setup.get(name), str) or not setup[name]:
            raise ValueError(f"setup needs a non-empty {name}")
    d = setup.get("distance_m")
    if isinstance(d, bool) or not isinstance(d, (int, float)) or not math.isfinite(d) or d <= 0:
        raise ValueError("setup needs the measured distance_m")
    roi = setup.get("roi")
    if roi is not None:
        if len(roi) != 4 or not (0 <= roi[0] < roi[2] <= width and 0 <= roi[1] < roi[3] <= height):
            raise ValueError("roi must be x0,y0,x1,y1 inside the image")


def frame_report(frames) -> dict:
    numbers = [f.number for f in frames]
    repeated = sum(b == a for a, b in zip(numbers, numbers[1:]))
    gaps = [{"after": a, "missing": b - a - 1} for a, b in zip(numbers, numbers[1:]) if b - a > 1]
    return {"frames": len(frames), "repeated": repeated, "gaps": gaps}


def write_capture(out, frames, device: dict, setup: dict, *, warmup_s: float) -> None:
    from PIL import Image

    if device.get("post_processing"):
        raise ValueError(f"post-processing must be off, found {device['post_processing']}")
    stream = device["stream"]
    check_setup(setup, width=stream["width"], height=stream["height"])
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    for i, f in enumerate(frames):
        if f.z16.dtype != np.uint16 or f.z16.shape != (stream["height"], stream["width"]):
            raise ValueError("frames must be uint16 at the stream resolution")
        Image.fromarray(f.z16).save(out / f"z16_{i:06d}.png")
    record = {"device": device, "setup": setup, "warmup_s": warmup_s, "report": frame_report(frames),
              "frames": [{"file": f"z16_{i:06d}.png", "number": f.number, "timestamp_ms": f.timestamp_ms,
                          "domain": f.domain} for i, f in enumerate(frames)],
              "captured": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    (out / "capture.json").write_text(json.dumps(record, indent=1))


def capture(args) -> None:
    import pyrealsense2 as rs

    pipeline, config = rs.pipeline(), rs.config()
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    profile = pipeline.start(config)
    try:
        dev = profile.get_device()
        sensor = dev.first_depth_sensor()
        for name, value in (("visual_preset", args.preset), ("enable_auto_exposure", args.auto_exposure),
                            ("exposure", args.exposure), ("emitter_enabled", args.emitter),
                            ("laser_power", args.laser_power)):
            if value is not None:
                sensor.set_option(getattr(rs.option, name), value)
        end = time.monotonic() + args.warmup_s
        while time.monotonic() < end:
            pipeline.wait_for_frames()
        frames = []
        for _ in range(args.frames):
            depth = pipeline.wait_for_frames().get_depth_frame()
            frames.append(Frame(np.asanyarray(depth.get_data()).copy(), int(depth.get_frame_number()),
                                float(depth.get_timestamp()), str(depth.get_frame_timestamp_domain())))
        intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        options = {name: float(sensor.get_option(getattr(rs.option, name))) for name in OPTIONS
                   if sensor.supports(getattr(rs.option, name))}
        device = {
            "name": dev.get_info(rs.camera_info.name),
            "serial_sha256": hashlib.sha256(dev.get_info(rs.camera_info.serial_number).encode()).hexdigest(),
            "firmware": dev.get_info(rs.camera_info.firmware_version), "sdk": rs.__version__,
            "depth_scale_m": float(sensor.get_depth_scale()),
            "stream": {"width": args.width, "height": args.height, "fps": args.fps, "format": "z16"},
            "intrinsics": {"fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy, "model": str(intr.model),
                           "coeffs": list(intr.coeffs)},
            "options": options, "aligned_to_color": False, "post_processing": [],
        }
    finally:
        pipeline.stop()
    roi = None if args.roi is None else [int(v) for v in args.roi.split(",")]
    setup = {"label": args.label, "distance_m": args.distance_m, "distance_method": args.distance_method,
             "surface": args.surface, "reference_flat": args.reference_flat, "roi": roi, "notes": args.notes}
    write_capture(args.out / args.label, frames, device, setup, warmup_s=args.warmup_s)
    print(json.dumps({"saved": str(args.out / args.label), **frame_report(frames)}))


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--label", required=True)
    p.add_argument("--distance-m", type=float, required=True, help="measured camera-to-surface distance")
    p.add_argument("--distance-method", required=True, help="e.g. 'laser rangefinder' or 'tape'")
    p.add_argument("--surface", required=True, help="wall, reference_flat, pallet_front, ...")
    p.add_argument("--reference-flat", action="store_true", help="the surface is a known-flat reference board")
    p.add_argument("--roi", default=None, help="x0,y0,x1,y1 of the measured surface in the depth image")
    p.add_argument("--frames", type=int, default=30)
    p.add_argument("--warmup-s", type=float, default=300.0)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--preset", type=float, default=None)
    p.add_argument("--auto-exposure", type=float, default=None)
    p.add_argument("--exposure", type=float, default=None)
    p.add_argument("--emitter", type=float, default=None)
    p.add_argument("--laser-power", type=float, default=None)
    p.add_argument("--notes", default="")
    capture(p.parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
