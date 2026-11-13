#!/usr/bin/env python3
"""Record the D435i IMU with device and host time per sample (docs/design/2026-11-14-imu-record-and-mount.md 2, G2).

    imu_record.py --out data/imu_check/check.jsonl [--seconds S] [--gyro-hz 400] [--accel-hz 200] [--global-time off|on]

Rows: a header (the device), a ``profile`` row (the streams as started: actual rates, built-in calibration), then
``{"t": "imu", "s": "g"|"a", "dev_us", "ts_ms", "dom", "host_ns", "x", "y", "z"}`` -- ``dev_us`` the frame's
hardware timestamp metadata (32-bit µs), ``ts_ms`` ``get_timestamp()`` in the domain ``dom``, ``host_ns`` the host
monotonic time when the callback ran. Gyro rad/s, accel m/s^2, SDK axes (x right, y down, z forward). The callback only
queues (``RecordWriter``); a header, temperature rows and a footer go around them. An existing file is refused. Ctrl-C
ends the record with its footer.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

from forklift_core.plant.record_writer import MAX_ROWS, RecordWriter, host_boot_id
from forklift_core.sensors.imu_record import SCHEMA, WRAP_US

TEMP_EVERY_S = 60.0
STALL_S = 2.0  # no sample for this long after the start: the device is gone (design 2)


class RealSenseSource:
    """The first RealSense device's motion streams through a pipeline callback (as the 2026-10-23 desk recorder)."""

    def __init__(self, *, gyro_hz: int, accel_hz: int, global_time: bool) -> None:
        import pyrealsense2 as rs  # here, so that the module loads without the SDK (tests, ws1)

        self.rs = rs
        self.gyro_hz, self.accel_hz, self.global_time = gyro_hz, accel_hz, global_time
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) == 0:
            raise RuntimeError("no RealSense device found")
        self.dev = devices[0]
        name = rs.camera_info.name
        self.motion = next((s for s in self.dev.query_sensors() if "Motion" in s.get_info(name)), None)
        if self.motion is None:
            raise RuntimeError("the device has no motion module")
        self.stereo = next((s for s in self.dev.query_sensors() if "Stereo" in s.get_info(name)), None)
        # correction off: axes are still aligned to the depth frame, but the built-in sensitivity and bias are not
        # applied (design 2); each option is recorded as supported / set / read back
        self.options = {"enable_motion_correction": self._set(rs.option.enable_motion_correction, 0),
                        "global_time_enabled": self._set(rs.option.global_time_enabled, 1 if global_time else 0)}
        self.pipe = rs.pipeline(ctx)
        self.profile = None
        self.fallback = None

    def _set(self, option, value) -> dict:
        if not self.motion.supports(option):
            return {"supported": False, "set": None, "read_back": None}
        self.motion.set_option(option, value)
        return {"supported": True, "set": value, "read_back": self.motion.get_option(option)}

    def _nearest_rates(self) -> tuple[int, int]:
        """The device's motion rates nearest to the requested ones."""
        rs = self.rs
        offered = {rs.stream.gyro: set(), rs.stream.accel: set()}
        for sp in self.motion.get_stream_profiles():
            if sp.stream_type() in offered and sp.format() == rs.format.motion_xyz32f:
                offered[sp.stream_type()].add(sp.fps())
        pick = []
        for stream, want in ((rs.stream.gyro, self.gyro_hz), (rs.stream.accel, self.accel_hz)):
            if not offered[stream]:
                raise RuntimeError(f"the motion module offers no {stream} profile")
            pick.append(min(offered[stream], key=lambda f: (abs(f - want), -f)))
        return pick[0], pick[1]

    def info(self) -> dict:
        """The device, before the streams start (the header)."""
        rs = self.rs
        ci = rs.camera_info
        return {"device": self.dev.get_info(ci.name), "serial": self.dev.get_info(ci.serial_number),
                "firmware": self.dev.get_info(ci.firmware_version), "usb": self.dev.get_info(ci.usb_type_descriptor),
                "sdk": rs.__version__, "motion_correction": 0}

    def profile_info(self) -> dict:
        """The streams as started: actual rates, built-in IMU calibration, a rate fallback (the ``profile`` row)."""
        rs = self.rs
        out = {"actual": {}, "imu_intrinsics": {}, "rate_fallback": self.fallback, "options": self.options}
        if self.profile is not None:
            for sp in self.profile.get_streams():
                key = "gyro_hz" if sp.stream_type() == rs.stream.gyro else "accel_hz"
                out["actual"][key] = sp.fps()
                try:
                    mi = sp.as_motion_stream_profile().get_motion_intrinsics()
                    out["imu_intrinsics"][key[:-3]] = {"data": [list(r) for r in mi.data],
                                                       "noise_variances": list(mi.noise_variances),
                                                       "bias_variances": list(mi.bias_variances)}
                except RuntimeError as e:
                    out["imu_intrinsics"][key[:-3]] = f"unavailable: {e}"
        return out

    def temperature(self) -> dict | None:
        rs = self.rs
        out = {}
        for sensor in (self.stereo, self.motion):
            if sensor is None:
                continue
            for name in ("asic_temperature", "projector_temperature", "motion_module_temperature"):
                opt = getattr(rs.option, name, None)
                if opt is not None and name not in out and sensor.supports(opt):
                    try:
                        out[name] = sensor.get_option(opt)
                    except RuntimeError:
                        pass
        return out or None

    def start(self, on_sample) -> None:
        rs = self.rs
        meta = rs.frame_metadata_value.frame_timestamp

        def cb(f):
            d = f.as_motion_frame().get_motion_data()
            dev_us = int(f.get_frame_metadata(meta)) if f.supports_frame_metadata(meta) else None
            s = "g" if f.get_profile().stream_type() == rs.stream.gyro else "a"
            dom = str(f.get_frame_timestamp_domain()).rsplit(".", 1)[-1]
            on_sample(s, dev_us, f.get_timestamp(), dom, (d.x, d.y, d.z))

        def config(g, a):
            cfg = rs.config()
            cfg.enable_device(self.dev.get_info(rs.camera_info.serial_number))
            cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, g)
            cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, a)
            return cfg

        try:
            self.profile = self.pipe.start(config(self.gyro_hz, self.accel_hz), cb)
        except RuntimeError as e:  # a rate the device does not offer: take the nearest offered and say so (design 2)
            g, a = self._nearest_rates()
            self.fallback = {"error": str(e), "chosen": {"gyro_hz": g, "accel_hz": a}}
            self.profile = self.pipe.start(config(g, a), cb)

    def stop(self) -> None:
        self.pipe.stop()


class StreamStats:
    """Per-stream counts kept in the callback (a few comparisons per sample): samples, missing device times, NaNs,
    repeated and backward device times, the largest forward step, the time domains seen."""

    def __init__(self) -> None:
        self.n = self.missing_dev = self.nan = self.dup = self.back = 0
        self.max_gap_us = 0
        self.last = None
        self.domains: set = set()

    def add(self, dev_us, dom, xyz) -> None:
        self.n += 1
        self.domains.add(dom)
        if any(v != v for v in xyz):
            self.nan += 1
        if dev_us is None:
            self.missing_dev += 1
            return
        if self.last is not None:
            step = (dev_us - self.last) % WRAP_US  # forward across a wrap; a step back reads as nearly a turn
            if step == 0:
                self.dup += 1
            elif step > WRAP_US // 2:
                self.back += 1
            else:
                self.max_gap_us = max(self.max_gap_us, step)
        if self.last is None or 0 < (dev_us - self.last) % WRAP_US <= WRAP_US // 2:
            self.last = dev_us

    def summary(self) -> dict:
        return {"n": self.n, "missing_dev": self.missing_dev, "nan": self.nan, "dup": self.dup, "back": self.back,
                "max_gap_us": self.max_gap_us, "domains": sorted(map(str, self.domains))}


def make_callback(sink, *, now_ns=time.monotonic_ns, max_rows: int = MAX_ROWS):
    """(writer, on_sample, per-stream counts): ``on_sample`` stamps the host time and queues the row -- nothing that
    can wait, so the SDK's callback thread is never held up by the disk. ``on_sample.stats`` holds the per-stream
    figures and ``on_sample.last_ns`` the host time of the latest sample."""
    writer = RecordWriter(sink, max_rows=max_rows)
    counts = {"g": 0, "a": 0}
    stats = {"g": StreamStats(), "a": StreamStats()}
    lock = threading.Lock()  # the two streams may call back from different threads

    def on_sample(stream, dev_us, ts_ms, dom, xyz):
        host_ns = now_ns()
        if dev_us is not None and not (0 <= dev_us < WRAP_US):
            dev_us = None  # not a 32-bit device time
        with lock:
            counts[stream] += 1
            stats[stream].add(dev_us, dom, xyz)
            on_sample.last_ns = host_ns
        writer.put_row({"t": "imu", "s": stream, "dev_us": dev_us, "ts_ms": ts_ms, "dom": dom, "host_ns": host_ns,
                        "x": xyz[0], "y": xyz[1], "z": xyz[2]})

    on_sample.stats = stats
    on_sample.last_ns = None
    return writer, on_sample, counts


def record(source, sink, *, seconds: float | None, gyro_hz: int, accel_hz: int, global_time: bool,
           temp_every_s: float = TEMP_EVERY_S, stall_s: float = STALL_S, now_ns=time.monotonic_ns,
           max_rows: int = MAX_ROWS) -> dict:
    """Record until ``seconds`` (None: until Ctrl-C), samples stop for ``stall_s``, or the start fails. Order: header,
    source start, profile row, samples and temperature rows; then source stop (its callbacks end), counts frozen,
    writer close. Returns the footer with the writer's ``close_result``."""
    writer, on_sample, counts = make_callback(sink, now_ns=now_ns, max_rows=max_rows)
    stop_reason, error = "seconds", None
    started = False
    t0 = now_ns()
    requested = {"gyro_hz": gyro_hz, "accel_hz": accel_hz}
    # the header goes first, before any sample can be queued; what only the started streams know follows as ``profile``
    writer.put_row({"t": "header", "schema": SCHEMA, "tool": "imu_record.py", "host_clock": "monotonic",
                    "host_boot_id": host_boot_id(), "start_wall": time.time(), "start_host_ns": t0,
                    "global_time": global_time, "requested": requested, **source.info()})
    try:
        try:
            source.start(on_sample)
        except RuntimeError as e:
            stop_reason, error = "start_failed", str(e)
        else:
            started = True
            prof = source.profile_info()
            actual = prof.get("actual", {})
            mismatch = any(actual.get(k) != v for k, v in requested.items())
            writer.put_row({"t": "profile", "host_ns": now_ns(), "rate_mismatch": mismatch, **prof})
            next_temp = t0
            while seconds is None or (now_ns() - t0) / 1e9 < seconds:
                now = now_ns()
                last = on_sample.last_ns if on_sample.last_ns is not None else t0
                if (now - last) / 1e9 > stall_s:
                    stop_reason = "stalled"
                    break
                if now >= next_temp:
                    temps = source.temperature()
                    if temps:
                        writer.put_row({"t": "temp", "host_ns": now, **temps})
                    next_temp += int(temp_every_s * 1e9)
                time.sleep(0.05 if seconds is None else min(0.05, max(seconds, 0.001)))
    except KeyboardInterrupt:
        stop_reason = "interrupt"
    finally:
        if started:
            source.stop()  # the SDK's callbacks have ended when this returns
        elapsed = (now_ns() - t0) / 1e9
        footer = {"stop": stop_reason, "seconds": elapsed, "samples": dict(counts),
                  "observed_hz": {k: (v / elapsed if elapsed > 0 else None) for k, v in counts.items()},
                  "streams": {k: v.summary() for k, v in on_sample.stats.items()}}
        if error is not None:
            footer["error"] = error
        result = writer.close(footer, timeout_s=10.0)
    return {**footer, **result}


def main(argv=None, *, source_factory=RealSenseSource) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--out", required=True, type=Path, help="new JSONL file (an existing one is refused)")
    p.add_argument("--seconds", type=float, default=None, help="stop after this long (default: Ctrl-C)")
    p.add_argument("--gyro-hz", type=int, default=400)
    p.add_argument("--accel-hz", type=int, default=200)
    p.add_argument("--global-time", choices=("off", "on"), default="off",
                   help="off (default): get_timestamp stays in the device clock domain; G7 does the mapping")
    args = p.parse_args(argv)
    if args.out.exists():
        p.error(f"{args.out} exists; a record is never overwritten")
    gt = args.global_time == "on"
    try:
        source = source_factory(gyro_hz=args.gyro_hz, accel_hz=args.accel_hz, global_time=gt)
    except (RuntimeError, ImportError) as e:  # no device, no SDK: nothing recorded, no file made
        print(f"imu_record: cannot open the IMU: {e}", file=sys.stderr)
        return 1
    args.out.parent.mkdir(parents=True, exist_ok=True)
    try:
        sink = open(args.out, "x", encoding="utf-8")  # noqa: SIM115 -- the writer thread closes it
    except FileExistsError:
        p.error(f"{args.out} exists; a record is never overwritten")
    footer = record(source, sink, seconds=args.seconds, gyro_hz=args.gyro_hz, accel_hz=args.accel_hz,
                    global_time=gt)
    hz = {k: round(v, 1) for k, v in footer["observed_hz"].items() if v}
    print(f"{args.out}: stop {footer['stop']}, {footer['samples']} in {footer['seconds']:.1f} s, observed {hz} Hz, "
          f"dropped {footer['dropped']}, write errors {footer['write_errors']}, writer finished {footer['finished']} "
          f"(unwritten {footer['unwritten']})" + (f", error: {footer['error']}" if "error" in footer else ""),
          file=sys.stderr)
    ok = footer["stop"] in ("seconds", "interrupt") and footer["finished"] and not footer["write_errors"]
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
