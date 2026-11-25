"""TEST ONLY: a synthetic observe record (``observe_record/1``) for the replay run's tests (PR 7 7r plan 5) -- the rows
the observe tool writes, made by the same code paths (``state_row``, the adapter's ``SdkStream`` frames judged by a
``ScanFeed``, a recording ``ImuSource``'s rows), plus the ``.ok`` marker. Never a measurement: its header says
``synthetic`` and the replay takes it only with ``allow_synthetic``.

``write_record(path, ...)`` returns what the tests need (the anchor, the configs the record's sources match)."""

from __future__ import annotations

import base64
import dataclasses
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

REPO = Path(__file__).resolve().parents[3]
MS = 1_000_000
OFF = 1_700_000_000_000 * MS  # system - monotonic on the recording host
RUN = "a1b2c3d4e5f60718"
FACTS = {"image_id": "sha256:" + "a" * 64, "usb": "10c4:ea60:0001", "host": "jetson", "ans_type": 132,
         "sample_period_us": 31.25, "baudrate": 256000}


def _adapter():
    sys.path.insert(0, str(REPO / "ros2" / "src" / "forklift_ros" / "forklift_ros"))
    import real_scan_adapter as ra

    return ra


def lidar_config(*, mount_measured: bool = True, err_s: float = 0.002):
    """TEST ONLY: a LiDAR config whose bound applies to this record's frames (basis measured, applies_to = FACTS)."""
    from forklift_core.sensors.lidar_cfg import LidarConfig

    return LidarConfig(mount=(0.1, 0.0, 0.3, 0.0), mount_measured=mount_measured, mask_rad=(), limit_m=5.7,
                       scan_period_s=0.12, first_beam_err_s=err_s, timing_basis="measured",
                       applies_to=tuple(sorted(FACTS.items())))


def imu_config(**over):
    """TEST ONLY: a measured IMU source config."""
    from forklift_core.sensors.imu_source import ImuSourceConfig

    base = dict(accel_hz=200, gyro_hz=200, delay_bound_s=0.02, measured=True, provisional_delay_bound_s=0.03,
                restart_back_us=20_000, receive_gap_s=0.1, queue_max=256)
    base.update(over)
    return ImuSourceConfig(**base)


def _state(host_ns: int, k: int):
    from forklift_core.mcu_link.messages import FaultFlag, McuState, Mode, RejectReason, StateFlag
    from forklift_core.plant.bringup import Sample
    from forklift_core.plant.record_rows import state_row

    st = McuState(mcu_time_us=k * 10_000, boot_id=7, arm_gen=0, last_rx_seq=k & 0xFFFF, mode=Mode.DISARMED,
                  reject_reason=RejectReason.NONE, fault_flags=FaultFlag(0), wheel_count_left=0, wheel_count_right=0,
                  wheel_rate_left_rad_s=0.0, wheel_rate_right_rad_s=0.0, steering_rad=0.0, lift_m=0.0,
                  current_ma=(0, 0, 0, 0), duty_e4=(0, 0, 0, 0), driver_status=(0, 0, 0, 0), rx_errors=0,
                  cycle_overruns=0, flags=StateFlag(0))
    return state_row(Sample(int(host_ns), st))


def write_record(path, *, seconds: float = 45.0, t0_ns: int = 50_000 * MS, anchor_after_s: float = 1.0,
                 scan_hz: float = 10.0, span_s: float = 0.098, arrive_s: float = 0.010, imu_hz: float = 200.0,
                 imu: bool = True, imu_config_in_header: bool = True, events=(), late_scans=(),
                 imu_rewind_at_s: float | None = None, imu_lost_at_s: float | None = None,
                 imu_cfg=None, synthetic: bool = True, result: str = "ok", start_after_s: float = 0.0,
                 restart_at_s: float | None = None, scans_until_s: float | None = None) -> dict:
    """``events``: ``(at_s, kind)`` scan_event rows (``receive_gap``, ``receive_resumed``, ``socket_error``);
    ``late_scans``: scan indexes whose arrival is 300 ms after their last beam; ``restart_at_s``: the adapter's
    node starts a new scan (a new start id: a new producer generation) there."""
    from forklift_core.sensors import scan_feed as sf
    from forklift_core.sensors.imu_source import ImuSource

    ra = _adapter()
    cfg = imu_cfg if imu_cfg is not None else imu_config()
    path = Path(path)
    rows = []
    rows.append({"t": "header", "schema": "observe_record/1", "tool": "real_observe.py", "device": "/dev/ttyACM0",
                 "device_id": None, "synthetic": {"test": "make_record"} if synthetic else None,
                 "start_wall": 0.0, "start_mono_ns": t0_ns, "seconds": seconds,
                 "lidar": {"path": "tests", "mount_measured": True, "first_beam_err_s": 0.002,
                           "timing_basis": "measured"},
                 "imu": None if not imu else {
                     **({"config": dataclasses.asdict(cfg), "config_sha256": "0" * 64} if imu_config_in_header else {}),
                     "path": "tests", "measured": cfg.measured, "delay_bound_s": cfg.delay_bound_s}})
    rows.append({"t": "slam_header", "socket": "/tmp/x", "bundle": "tests/bundle", "map_id": "m" * 64,
                 "start_pose": [1.0, 2.0, 0.5], "rear_offset_m": 0.25, "base_laser": [0.35, 0.0, 0.3, 0.0],
                 "start_ns": t0_ns, "mode": "diagnostic"})
    timed = []  # (host ns, order, row)
    # states every 10 ms; the delay bound stands from anchor_after_s
    end_ns = t0_ns + int(seconds * 1e9)
    k = 0
    for ns in range(t0_ns, end_ns, 10 * MS):
        row = _state(ns, k)
        row["conn"] = 1
        row["delay_bound_s"] = 0.03 if ns >= t0_ns + int(anchor_after_s * 1e9) else None
        timed.append((ns, 0, row))
        k += 1
    # scans: v2 frames of the adapter's stream, judged as the observe tool judges them
    stream = ra.SdkStream(RUN, {k_: v for k_, v in FACTS.items() if k_ in ("image_id", "usb", "host")})
    start_ids = ["fedcba9876543210", "0123456789abcdef"]
    for sid in start_ids:
        stream.on_info(json.dumps({"format": "sdk_mono_v2", "run": RUN, "start": sid,
                                   **{k_: FACTS[k_] for k_ in ("ans_type", "sample_period_us", "baudrate")}}))
    period = int(1e9 / scan_hz)
    n = 16
    scan_rows = []
    j = 0
    first = t0_ns + int(start_after_s * 1e9)
    stop = t0_ns + int((scans_until_s if scans_until_s is not None else seconds) * 1e9) - int(0.5e9)
    while first < stop:
        last = first + int(span_s * 1e9)
        recv = last + (300 * MS if j in late_scans else int(arrive_s * 1e9))
        if scan_rows:  # the node publishes in order: a scan held back holds the ones after it too
            recv = max(recv, scan_rows[-1][0])
        sid = start_ids[1] if (restart_at_s is not None and first >= t0_ns + int(restart_at_s * 1e9)) else start_ids[0]
        msg = SimpleNamespace(ranges=list(np.full(n, 2.0)), intensities=[], angle_min=-np.pi,
                              angle_increment=2 * np.pi / n, time_increment=float(np.float32(span_s / (n - 1))),
                              scan_time=float(np.float32(span_s)), range_min=0.1, range_max=12.0,
                              header=SimpleNamespace(stamp=SimpleNamespace(sec=first // 10**9, nanosec=first % 10**9),
                                                     frame_id=f"laser@sdk_mono_v2#{RUN}.{sid}"))
        out, why = stream.on_scan(first, msg.scan_time, recv, OFF, frame_id=msg.header.frame_id)
        assert why is None, why
        frame = ra.make_frame(msg, scan_id=j + 1, recv_mono_ns=recv, sys_minus_mono_ns=OFF, gen=out["gen"],
                              stamp_kind=out["stamp_kind"], sdk=out["sdk"])
        host = recv + 1 * MS
        scan_rows.append((host, 1, {"t": "scan", "host_recv_ns": host, "frame": base64.b64encode(frame).decode(),
                                    "_bytes": frame}))
        first += period
        j += 1
    for at_s, kind in events:
        ns = t0_ns + int(at_s * 1e9)
        scan_rows.append((ns, 1, {"t": "scan_event", "kind": kind, "host_ns": ns}))
    scan_rows.sort(key=lambda x: x[0])
    feed = sf.ScanFeed()
    for ns, o, row in scan_rows:
        if row["t"] == "scan_event":
            if row["kind"] in ("receive_gap", "socket_error"):
                feed.new_generation(row["kind"])
            row["feed_gen"] = feed.generation
        else:
            _s, why = feed.judge(row.pop("_bytes"), ns)
            row["verdict"] = why
            row["feed_gen"] = feed.generation
        timed.append((ns, o, row))
    # IMU: a recording source's own rows, in the observe tool's cycle order (every 20 ms: the samples received, a
    # loss the device thread queued, then samples() and -- when not capped -- generation()); rows in drain order
    last_gen_ns = None
    if imu:
        clock = {"ns": 0}
        src = ImuSource(cfg, operating=False, record=True, now_ns=lambda: clock["ns"])
        step = int(1e9 / imu_hz)
        dev = 1_000_000
        lost_ns = None if imu_lost_at_s is None else t0_ns + int(imu_lost_at_s * 1e9)
        rewind_ns = None if imu_rewind_at_s is None else t0_ns + int(imu_rewind_at_s * 1e9)
        sample_ns = t0_ns
        cycle = 20 * MS
        order = 2
        for now in range(t0_ns + cycle, end_ns - int(0.2e9), cycle):
            while sample_ns <= now and (lost_ns is None or sample_ns < lost_ns):
                if rewind_ns is not None and abs(sample_ns - rewind_ns) < step // 2:
                    dev -= 50_000  # the device clock goes back 50 ms: a restart
                dev += step // 1000
                clock["ns"] = sample_ns
                src.push(sample_ns, "a", dev, "hardware_clock", (0.0, 0.0, 9.81))
                sample_ns += step
            if lost_ns is not None and now - cycle < lost_ns <= now:
                clock["ns"] = lost_ns
                src.lost("usb_gone", at_ns=lost_ns)
            clock["ns"] = now
            src.samples(now, 64)
            if not src.last_read_capped:
                src.generation()
            for r in src.drain_rows():
                timed.append((now, order, r))
        last_gen_ns = end_ns - int(0.2e9)
        clock["ns"] = last_gen_ns
        src.samples(last_gen_ns, 1 << 30)
        src.generation()
        for r in src.drain_rows():
            timed.append((last_gen_ns, order, r))
    timed.sort(key=lambda x: (x[0], x[1]))
    rows += [r for _ns, _o, r in timed]
    rows.append({"t": "end", "result": result, "stopped_confirmed": True, "complete": True, "record_errors": 0,
                 "imu": None if not imu else {"last_generation_ns": last_gen_ns}})
    with open(path, "w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r) + "\n")
    from forklift_core.live.observe_record import file_sha256

    Path(str(path) + ".ok").write_text(file_sha256(path) + "\n", encoding="ascii")
    anchor = t0_ns + int(anchor_after_s * 1e9)
    return {"anchor_ns": anchor, "t0_ns": t0_ns, "end_ns": end_ns, "imu_cfg": cfg, "scans": j}


__all__ = ["FACTS", "imu_config", "lidar_config", "write_record"]
