"""Synthetic M1-M3 trials for the G8 entry's tests (docs/design/2026-11-15-m-analyze-entry.md 6, PR 1).

One call makes what a measurement day leaves behind for one trial, from a drive profile and a TRUE plant the analysis
must recover:

- the teleop bridge record -- header, ``profile_start``, the profile rows of the real ``ProfileRunner`` (it is run
  against the plant, so its rows, segment boundaries, deadman waits and voids are the runner's own), STATE rows by
  ``state_row`` every 10 ms, ``profile_end``, footer;
- an ``imu_record/1`` file -- gyro 400 Hz, accelerometer 200 Hz, a 32-bit microsecond device clock;
- a ``truth/1`` file -- the true start and end standstill poses (``stop_pose``) of the rear-axle centre; with
  ``truth_method="B_camera"`` also the M3 moving points ``merge_end`` and ``run_042`` (the pose at the cycle the
  runner left ``merge`` / ``cont``, with its ``host_ns`` and covariance).

The plant: the encoder speed follows the command with the profile's acceleration limit; the true speed is the encoder
speed divided by ``true_radius_scale`` (the encoders over-read distance by it); the true heading turns by the held
curvature of the measured steering (``delay_s`` late: the wheel-speed difference and the body yaw follow the
measured q that long after it) plus ``true_yaw_per_m`` per metre; the gyro reads the true yaw rate on the body
``up`` axis plus ``bias`` (``bias_step`` added from the first wheel motion) plus white noise of density ``arw``; the
accelerometer reads the upward reaction ``G * up``. Headers say ``"synthetic": true`` unless ``synthetic=False``
(only refusal tests ask for that). Deterministic for a seed.
"""

from __future__ import annotations

import copy
import math

import numpy as np

from forklift_core.live import drive_profile as dprof
from forklift_core.live import real_teleop as rt
from forklift_core.mcu_link.messages import FaultFlag, McuState, Mode, RejectReason, StateFlag
from forklift_core.plant.bringup import Sample
from forklift_core.plant.kinematics import RealKinematics, teleop_limit_overrides
from forklift_core.plant.real_vehicle import advance_arc
from forklift_core.plant.record_rows import state_row
from forklift_core.safety.teleop_drive import TeleopLimits
from forklift_core.sensors.imu_record import WRAP_US

ARW_DESK = 2.054e-4  # rad/sqrt(s): the bench gyro y (registration 1.1)
G = 9.80665
STATE_DT_NS = 10_000_000  # one STATE per 10 ms
CYCLE_STATES = 2  # a bridge cycle every second STATE (20 ms)
GYRO_HZ, ACCEL_HZ = 400.0, 200.0
BOOT = 0x5E57
T0_NS = 100_000_000_000  # the record's start (host monotonic)
DEV0_US = 123_456_789  # the IMU device clock at T0_NS - IMU_PAD_NS (a block's trials share it)
PRE_S, POST_S = 1.5, 1.0  # standing before the trial starts (the runner wants >= 1 s) and after it ends
IMU_PAD_NS = 500_000_000
VOIDS = ("deadman", "ENCODER_JUMP_L", "STEER_SENSOR_INVALID", "boot_change", "time_not_increasing", "encoder_gap")
TRUTH_SIGMA = {"pose_yaw_rad": 5e-4, "pose_xy_m": 1e-3, "source": "synthetic"}
TRUTH_METHODS = ("synthetic", "B_camera")
MOVING_COV = [[2.5e-7, 0.0, 0.0], [0.0, 2.5e-7, 0.0], [0.0, 0.0, 9e-8]]  # B camera moving point: 0.5 mm, 0.3 mrad
MOVING_POINTS = {"merge": "merge_end", "cont": "run_042"}  # the point taken when the runner leaves the segment


def unit(v) -> np.ndarray:
    a = np.asarray(v, float)
    n = float(np.linalg.norm(a))
    if a.shape != (3,) or not math.isfinite(n) or n == 0.0:
        raise ValueError("a 3-vector of non-zero length is needed")
    return a / n


def profile_limits(cfg) -> dprof.ProfileLimits:
    """The limits the teleop bridge checks a profile against (tools/teleop_bridge.py Bridge.profile_limits)."""
    import pathlib

    kin = RealKinematics(cfg)
    params = rt.load_bridge_params(pathlib.Path(__file__).resolve().parents[3] / "config" / "real" /
                                   "teleop_bridge.yaml", cfg)
    cap = rt.command_cap_mps(cfg.teleop_speed_limit_mps, params)
    lim = rt.real_teleop_limits(TeleopLimits(**teleop_limit_overrides(kin)), cap)
    return dprof.ProfileLimits(cap_mps=min(lim.direct_forward_mps, lim.direct_reverse_mps),
                               kappa_max=lim.curvature_inv_m, kappa_rate=lim.curvature_rate_inv_m_s,
                               q_rate=kin.steering_rate_rad_s, accel_mps2=lim.accel_mps2,
                               q_of_kappa=cfg.steering.angle, kappa_of_q=lambda q: kin.held_curvature((q,)))


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _state(t_us, boot, counts, rates, q, faults=FaultFlag(0), mode=Mode.ARMED) -> McuState:
    return McuState(int(t_us), boot, 1, 0, mode, RejectReason.NONE, faults, int(counts[0]), int(counts[1]),
                    float(rates[0]), float(rates[1]), float(q), 0.0, (0, 0, 0, 0), (0, 0, 0, 0), (0xC0,) * 4, 0, 0,
                    StateFlag.DRIVERS_READY | StateFlag.ZERO_SEEN)


def synth_trial(profile: dict, cfg, *, true_radius_scale: float = 1.0, true_yaw_per_m: float = 0.0,
                gain: float = 1.0, q_offset: float = 0.0, q_short_drive: float = 0.0,
                up=(0.0, -1.0, 0.0), bias=(0.0, 0.0, 0.0), bias_step=None, arw: float = ARW_DESK,
                still_s: float | None = None, void: str | None = None, seed: int = 0,
                mcu_imu_offset_s: float = 0.0, imu_late_sample: bool = False, dev_us_wrap: bool = False,
                deadman_wait_s: float = 0.0, synthetic: bool = True, trial: int = 1,
                start_pose=(0.0, 0.0, 0.0), delay_s: float = 0.0, truth_method: str = "synthetic",
                q_slew_rad_s: float | None = None, t0_ns: int = T0_NS, host_boot: str | None = None,
                bias_slope: float = 0.0) -> dict:
    """``{"record": rows, "imu": rows, "truth": rows}`` for one trial of ``profile`` (a drive_profile/1 dict).
    ``still_s`` replaces every hold segment's length (tests that need no 60 s windows). ``void`` puts one defect
    into the drive (one of ``VOIDS``).

    ``gain``: the true curvature is ``gain`` times the held curvature of the MEASURED q (M2). ``q_offset``: the
    measured q is the commanded q minus it while steering is commanded (the runner's ``drive_q`` stays the commanded
    q). ``q_short_drive``: in drive segments the measured |q| stops this far short of the commanded one (an
    ``under_reached`` maker; presteer still reaches).

    ``delay_s``: the wheel-speed difference and the true yaw follow the measured q this late (linear between ticks;
    the M3 q-step delay the entry must recover). ``truth_method``: ``"synthetic"`` (stop poses only) or
    ``"B_camera"`` (the header says B_camera, still marked synthetic, and the M3 moving points are added).
    ``q_slew_rad_s``: the measured q moves toward its target at this rate (None: it is there at once).

    Block use (``synth_block``): ``t0_ns`` starts the record later on the same host and MCU clocks (the MCU time,
    the IMU device clock and the bias slope all count from ``T0_NS``, so trials of one block line up), ``host_boot``
    names the host boot, ``bias_slope`` (rad/s^2) adds a bias growing along the up axis."""
    if not (true_radius_scale > 0 and math.isfinite(true_radius_scale)):
        raise ValueError("true_radius_scale must be positive")
    if not (delay_s >= 0 and math.isfinite(delay_s)):
        raise ValueError("delay_s must be non-negative")
    if truth_method not in TRUTH_METHODS:
        raise ValueError(f"truth_method must be one of {TRUTH_METHODS}")
    if void is not None and void not in VOIDS:
        raise ValueError(f"void must be one of {VOIDS}")
    up = unit(up)
    bias = np.asarray(bias, float)
    step = None if bias_step is None else np.asarray(bias_step, float)
    rng = np.random.default_rng(seed)
    data = copy.deepcopy(profile)
    if still_s is not None:
        for seg in data["segments"]:
            if "hold_s" in seg:
                seg["hold_s"] = float(still_s)
    lim = profile_limits(cfg)
    runner = dprof.ProfileRunner(dprof.parse_profile(data, lim), lim)
    runner.trial = trial - 1
    kin = RealKinematics(cfg)
    host_boot = host_boot or f"synthetic-{seed}"
    t0 = int(t0_ns)
    rec = [{"t": "header", "schema": "teleop_bridge_record/1", "tool": "m_synth", "device": "synthetic",
            "device_id": None, "synthetic": bool(synthetic), "host_boot_id": host_boot, "start_wall": 0.0,
            "start_mono_ns": t0}]

    # the plant
    pose = tuple(float(v) for v in start_pose)
    v_enc = 0.0  # the encoder's signed speed (what the bridge sees)
    q = 0.0  # the measured steering angle
    q_cmd = 0.0  # the commanded one (what the bridge applied)
    counts = [0.0, 0.0]
    boot, last_us = BOOT, None
    omega: list[float] = []  # the true yaw rate over each 10 ms tick
    ticks_ns: list[int] = []
    deadman = True
    started = ended = False
    start_ns = end_ns = None
    truth_start = truth_end = None
    seg_drive_entry = None
    moving_since = moving_last = None
    gap_left = 0
    defect_done = False
    post_left = int(POST_S * 1e9 / STATE_DT_NS)
    q_hist: list[float] = []  # the measured q of every tick (for the delayed response)
    moving_pts: dict = {}
    last_label = None
    k = 0
    while True:
        host = t0 + k * STATE_DT_NS
        cycle = k % CYCLE_STATES == 0
        # the bridge cycle (before the plant moves this tick: it decides on what it saw)
        if cycle:
            if not started and host - t0 >= PRE_S * 1e9:
                why = runner.start(host, standstill_s=(host - t0) / 1e9, armed=True)
                assert why is None, why
                started, start_ns, truth_start = True, host, pose
                rec.append({"t": "profile_start", "host_ns": host, "trial": runner.trial,
                            "profile": runner.profile.name, "refused": None})
            if started and not ended:
                seg = runner.segment
                label = None if seg is None else seg.label
                if last_label in MOVING_POINTS and label != last_label and MOVING_POINTS[last_label] not in moving_pts:
                    moving_pts[MOVING_POINTS[last_label]] = (host, pose)
                last_label = label
                if seg is not None and seg.kind == "drive" and seg_drive_entry is None:
                    seg_drive_entry = host
                deadman = True
                if seg_drive_entry is not None and host < seg_drive_entry + deadman_wait_s * 1e9:
                    deadman = False
                if void == "deadman" and moving_since is not None and host >= moving_since + 1e9:
                    deadman = False
                if runner.state == "zeroing":
                    deadman = False
                applied = runner.req_mps
                obs = dprof.Observation(dt_s=CYCLE_STATES * STATE_DT_NS / 1e9, speed_valid=True, speed_mps=v_enc,
                                        meas_q=q, armed=True, mode="TELEOP", target_mps=applied,
                                        applied_mps=applied, applied_q=q_cmd, permit=(1.0, "ok"), port=None,
                                        direction_cut=None, stop=None, deadman=deadman, policy=None)
                row = runner.step(host, obs)
                if row is not None:
                    rec.append(row)
                    if row["result"] is not None and end_ns is None:
                        end_ns = host
                        rec.append({"t": "profile_end", "host_ns": host, "trial": row["trial"],
                                    "result": row["result"]})
                if runner.state == "idle" and end_ns is not None:
                    ended = True
        # the plant over this tick
        cmd = 0.0 if (runner.zero_now or runner.state != "running") else runner.req_mps
        runner.zero_now = False
        dv = lim.accel_mps2 * STATE_DT_NS / 1e9
        v_enc = min(cmd, v_enc + dv) if cmd >= v_enc else max(cmd, v_enc - dv)
        if runner.state == "running":
            q_cmd = lim.q_of_kappa(runner.req_kappa)
            target = q_cmd - q_offset
            if q_short_drive and q_cmd != 0.0 and runner.segment is not None and runner.segment.kind == "drive":
                target -= math.copysign(q_short_drive, q_cmd)
            if q_slew_rad_s is None:
                q = target
            else:
                dq = q_slew_rad_s * STATE_DT_NS / 1e9
                q = min(target, q + dq) if target >= q else max(target, q - dq)
        dt = STATE_DT_NS / 1e9
        q_hist.append(q)
        lag = delay_s / dt
        j = len(q_hist) - 1 - lag
        if j <= 0:
            q_resp = q_hist[0]
        else:
            j0 = int(math.floor(j))
            f = j - j0
            q_resp = q_hist[j0] if f == 0.0 else q_hist[j0] * (1 - f) + q_hist[j0 + 1] * f
        kappa = kin.held_curvature((q_resp,)) or 0.0
        v_true = v_enc / true_radius_scale
        ds_true = v_true * dt
        curv = gain * kappa + true_yaw_per_m
        omega.append(v_true * curv)
        ticks_ns.append(host)
        pose = advance_arc(*pose, ds_true, curv)
        if v_enc != 0.0:
            if moving_since is None:
                moving_since = host
            moving_last = host
        ds_enc = v_enc * dt  # what the encoders count (scaled)
        rates = []
        for i, side in enumerate((-1.0, 1.0)):
            d = ds_enc * (1.0 + side * kappa * cfg.track_m / 2.0)
            counts[i] += d / (2 * math.pi * cfg.wheel_radius_m[i]) * cfg.counts_per_wheel_rev[i]
            rates.append(d / dt / cfg.wheel_radius_m[i])
        # the STATE of this tick (with the asked defect once, in the drive)
        t_us = 1_000_000 + (host - T0_NS) // 1000
        faults = FaultFlag(0)
        in_drive = moving_since is not None and host >= moving_since + 500_000_000 and not ended \
            and runner.state == "running"
        emit = True
        if in_drive and not defect_done and void in VOIDS[1:]:
            defect_done = True
            if void in ("ENCODER_JUMP_L", "STEER_SENSOR_INVALID"):
                faults = FaultFlag[void]
            elif void == "boot_change":
                boot = BOOT + 1
            elif void == "time_not_increasing":
                t_us = last_us
            elif void == "encoder_gap":
                gap_left = 8
        if gap_left > 0:
            gap_left -= 1
            emit = False
        if emit:
            st = _state(t_us, boot, (round(counts[0]), round(counts[1])), rates, q, faults)
            row = state_row(Sample(host, st))
            row["read_ns"] = host
            rec.append(row)
            last_us = t_us
        k += 1
        if ended:
            if truth_end is None and abs(v_enc) < 1e-12:
                truth_end = pose
            post_left -= 1
            if post_left <= 0 and truth_end is not None:
                break
        if k > 10_000_000:
            raise RuntimeError("the synthetic trial did not end")
    rec.append({"t": "footer", "host_ns": t0 + k * STATE_DT_NS, "finished": True, "unwritten": 0, "dropped": 0,
                "write_errors": 0})

    # the IMU
    ticks = np.asarray(ticks_ns, dtype=np.int64)
    om = np.asarray(omega, float)
    imu_t0 = t0 - IMU_PAD_NS
    imu_t1 = ticks_ns[-1] + IMU_PAD_NS
    off_ns = int(round(mcu_imu_offset_s * 1e9))
    mid = moving_since + (moving_last - moving_since) // 2 if moving_since is not None else (imu_t0 + imu_t1) // 2
    dev0 = (WRAP_US - (mid - imu_t0) // 1000) if dev_us_wrap else DEV0_US + (imu_t0 - (T0_NS - IMU_PAD_NS)) // 1000
    rows = []
    for s, hz in (("g", GYRO_HZ), ("a", ACCEL_HZ)):
        n = int((imu_t1 - imu_t0) * hz / 1e9)
        t = imu_t0 + np.round(np.arange(n) * 1e9 / hz).astype(np.int64)
        if s == "g":
            # the gyro reads the tick whose STATE time is nearest: the rate a STATE reports at ``host`` is the
            # tick that starts there, so the staircase is centred on the STATE times (no half-tick lag against them)
            idx = np.clip(np.searchsorted(ticks, t + STATE_DT_NS // 2, side="right") - 1, 0, len(om) - 1)
            w = np.where((t >= ticks[0]) & (t < ticks[-1] + STATE_DT_NS), om[idx], 0.0)
            xyz = w[:, None] * up[None, :] + bias[None, :] + (bias_slope * (t - T0_NS) / 1e9)[:, None] * up[None, :]
            if step is not None and moving_since is not None:
                xyz = xyz + (t >= moving_since)[:, None] * step[None, :]
            xyz = xyz + rng.normal(0.0, arw * math.sqrt(hz), size=(n, 3))
        else:
            xyz = np.repeat((G * up)[None, :], n, axis=0)
        for ti, v in zip(t.tolist(), xyz.tolist()):
            rows.append({"t": "imu", "s": s, "dev_us": int((dev0 + (ti - imu_t0) // 1000) % WRAP_US),
                         "ts_ms": (ti - imu_t0) / 1e6, "dom": "GLOBAL_TIME", "host_ns": ti + off_ns,
                         "x": v[0], "y": v[1], "z": v[2]})
    rows.sort(key=lambda r: (r["host_ns"], r["s"]))
    if imu_late_sample:
        gi = [i for i, r in enumerate(rows) if r["s"] == "g" and r["host_ns"] - off_ns >= mid]
        i, j = gi[0], gi[3]
        late = rows.pop(i)
        late = {**late, "host_ns": rows[j - 1]["host_ns"]}  # arrived with the third sample after it
        rows.insert(j, late)
    imu = [{"t": "header", "schema": "imu_record/1", "tool": "m_synth", "host_clock": "monotonic",
            "host_boot_id": host_boot, "start_wall": 0.0, "start_host_ns": imu_t0 + off_ns,
            "synthetic": bool(synthetic)},
           {"t": "profile", "host_ns": imu_t0 + off_ns, "rate_mismatch": False,
            "actual": {"gyro_hz": GYRO_HZ, "accel_hz": ACCEL_HZ},
            "options": {"enable_motion_correction": {"requested": 0, "read_back": 0},
                        "global_time_enabled": {"requested": 1, "read_back": 1}}},
           *rows,
           {"t": "footer", "stop": "seconds", "seconds": (imu_t1 - imu_t0) / 1e9, "dropped": 0,
            "write_errors": 0}]

    # the truth
    def point(p):
        return {"kind": "stop_pose", "x_m": p[0], "y_m": p[1], "yaw_rad": _wrap(p[2])}

    points = {"start": point(truth_start), "end": point(truth_end)}
    if truth_method == "B_camera":
        for pid, (h, p) in moving_pts.items():
            points[pid] = {**point(p), "kind": "moving_point", "host_ns": int(h), "cov": copy.deepcopy(MOVING_COV)}
    truth = [{"t": "header", "schema": "truth/1", "method": truth_method, "synthetic": bool(synthetic),
              "record": {"host_boot_id": host_boot, "start_mono_ns": t0}, "body_point": "rear_axle_center",
              "frame": "floor", "sigma": dict(TRUTH_SIGMA)},
             {"t": "truth", "trial": runner.trial, "points": points}]
    return {"record": rec, "imu": imu, "truth": truth, "imu_span": (int(imu_t0), int(imu_t1))}


def _imu_still(t_from: int, t_to: int, *, host_boot, up, bias, bias_slope, arw, rng, synthetic) -> list:
    """Gyro and accelerometer rows of a standing truck over [t_from, t_to) on the block's clocks."""
    rows = []
    for s_, hz in (("g", GYRO_HZ), ("a", ACCEL_HZ)):
        n = int((t_to - t_from) * hz / 1e9)
        t = t_from + np.round(np.arange(n) * 1e9 / hz).astype(np.int64)
        if s_ == "g":
            xyz = bias[None, :] + (bias_slope * (t - T0_NS) / 1e9)[:, None] * up[None, :]
            xyz = xyz + rng.normal(0.0, arw * math.sqrt(hz), size=(n, 3))
        else:
            xyz = np.repeat((G * up)[None, :], n, axis=0)
        for ti, v in zip(t.tolist(), xyz.tolist(), strict=True):
            rows.append({"t": "imu", "s": s_, "dev_us": int((DEV0_US + (ti - (T0_NS - IMU_PAD_NS)) // 1000) % WRAP_US),
                         "ts_ms": (ti - t_from) / 1e6, "dom": "GLOBAL_TIME", "host_ns": ti, "x": v[0], "y": v[1],
                         "z": v[2]})
    return rows


def synth_still(cfg, *, seconds: float, t0_ns: int = T0_NS, host_boot: str = "synthetic-block",
                up=(0.0, -1.0, 0.0), bias=(0.0, 0.0, 0.0), bias_slope: float = 0.0, arw: float = ARW_DESK,
                seed: int = 0, synthetic: bool = True) -> dict:
    """A record of a truck standing ``seconds`` with no profile (a G3 stop, or any standstill of the day): STATE
    rows every 10 ms with no wheel motion, and its IMU rows. Same keys as ``synth_trial`` (truth None)."""
    up = unit(up)
    rng = np.random.default_rng(seed)
    t0 = int(t0_ns)
    n = int(seconds * 1e9 / STATE_DT_NS)
    rec = [{"t": "header", "schema": "teleop_bridge_record/1", "tool": "m_synth", "device": "synthetic",
            "device_id": None, "synthetic": bool(synthetic), "host_boot_id": host_boot, "start_wall": 0.0,
            "start_mono_ns": t0}]
    for k in range(n):
        host = t0 + k * STATE_DT_NS
        row = state_row(Sample(host, _state(1_000_000 + (host - T0_NS) // 1000, BOOT, (0, 0), (0.0, 0.0), 0.0)))
        row["read_ns"] = host
        rec.append(row)
    rec.append({"t": "footer", "host_ns": t0 + n * STATE_DT_NS, "finished": True, "unwritten": 0, "dropped": 0,
                "write_errors": 0})
    imu_t0, imu_t1 = t0 - IMU_PAD_NS, t0 + n * STATE_DT_NS + IMU_PAD_NS
    rows = _imu_still(imu_t0, imu_t1, host_boot=host_boot, up=up, bias=np.asarray(bias, float),
                      bias_slope=bias_slope, arw=arw, rng=rng, synthetic=synthetic)
    return {"record": rec, "imu": rows, "truth": None, "imu_span": (imu_t0, imu_t1)}


def synth_block(items: list, cfg, *, host_boot: str = "synthetic-block", bias_slope: float = 0.0,
                arw: float = ARW_DESK, up=(0.0, -1.0, 0.0), bias=(0.0, 0.0, 0.0), seed: int = 0,
                synthetic: bool = True) -> dict:
    """One IMU file over several records of one day (entry design 3.10; registration revision 3, 10.1): each item is
    ``{"profile": p, **synth_trial kwargs}`` or ``{"stand_s": seconds}``, laid end to end on the same host and MCU
    clocks so the IMU runs on without a gap. Returns ``{"records": [rows...], "truths": [rows or None...], "imu":
    rows}``; the bias grows by ``bias_slope`` throughout."""
    records, truths, imu = [], [], []
    cursor = T0_NS
    common = dict(host_boot=host_boot, bias_slope=bias_slope, arw=arw, up=up, bias=bias, synthetic=synthetic)
    for i, item in enumerate(items):
        item = dict(item)
        if "stand_s" in item:
            out = synth_still(cfg, seconds=float(item.pop("stand_s")), t0_ns=cursor, seed=seed + i, **common, **item)
            body = out["imu"]
        else:
            prof = item.pop("profile")
            out = synth_trial(prof, cfg, t0_ns=cursor, seed=seed + i, trial=1, **common, **item)
            body = [r for r in out["imu"] if r.get("t") == "imu"]
        records.append(out["record"])
        truths.append(out["truth"])
        imu.extend(body)
        cursor = out["imu_span"][1] + IMU_PAD_NS  # the next IMU span starts where this one ended
    imu.sort(key=lambda r: (r["host_ns"], r["s"]))
    t_from, t_to = imu[0]["host_ns"], imu[-1]["host_ns"]
    head = [{"t": "header", "schema": "imu_record/1", "tool": "m_synth", "host_clock": "monotonic",
             "host_boot_id": host_boot, "start_wall": 0.0, "start_host_ns": t_from, "synthetic": bool(synthetic)},
            {"t": "profile", "host_ns": t_from, "rate_mismatch": False,
             "actual": {"gyro_hz": GYRO_HZ, "accel_hz": ACCEL_HZ},
             "options": {"enable_motion_correction": {"requested": 0, "read_back": 0},
                         "global_time_enabled": {"requested": 1, "read_back": 1}}}]
    foot = {"t": "footer", "stop": "seconds", "seconds": (t_to - t_from) / 1e9, "dropped": 0, "write_errors": 0}
    return {"records": records, "truths": truths, "imu": head + imu + [foot]}


__all__ = ["ARW_DESK", "G", "TRUTH_METHODS", "VOIDS", "profile_limits", "synth_block", "synth_still", "synth_trial",
           "unit"]
