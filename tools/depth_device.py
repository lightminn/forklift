"""The D435i opened once for its motion module and its depth stream (PR 7 7e-1, docs/plans/2026-11-26-7e-depth-pocket-check.md
1): ``RealSenseDepthSource`` extends ``imu_record.RealSenseSource`` (the motion module on the same sensor object whose
options were set) with the depth sensor of the same device.

- Making it only finds the device and its sensors (no option written before the executor lock, as 7b).
- ``start(on_sample, on_depth, on_lost)``: the motion module (when asked for) as before, then the stereo module's
  options from ``depth.yaml sensor`` (a configured option the sensor does not support refuses), the exact Z16 profile
  (no fallback: the noise model and the check's camera are that resolution), the depth callback -- which copies the
  frame's pixels into its own array, reads the metadata it needs and never keeps the SDK frame -- and then every
  configured option read back: a difference refuses the start (``DepthSettingsMismatch``). A failure after one sensor
  started stops everything that was started before it raises (a single sensor never runs alone).
- The context's devices-changed callback reports the device's removal to ``on_lost`` (both sources take it).
- ``depth_profile_info()``: the profile as opened, its intrinsics and calibration id, the depth scale, every option
  read back and the settings' sha256 (the record's ``depth_profile`` row).
- ``stop()``: the depth sensor, then the motion module, each stopped then closed as far as it was reached."""

from __future__ import annotations

import hashlib
import json

import numpy as np

from imu_record import RealSenseSource

from forklift_core.sensors.depth_cfg import SENSOR_KEYS, calib_id

META_KEYS = ("frame_timestamp", "sensor_timestamp", "time_of_arrival", "backend_timestamp", "actual_exposure",
             "frame_laser_power", "frame_emitter_mode")


class DepthSettingsMismatch(RuntimeError):
    pass


class RealSenseDepthSource(RealSenseSource):
    def __init__(self, *, depth, motion: bool = True, gyro_hz: int = 200, accel_hz: int = 200,
                 global_time: bool = False, rs=None) -> None:
        super().__init__(gyro_hz=gyro_hz, accel_hz=accel_hz, global_time=global_time, rs=rs)
        if self.stereo is None:
            raise RuntimeError("the device has no stereo (depth) module")
        self.depth_cfg, self.use_motion = depth, bool(motion)
        self.ctx = self.rs.context()
        self.depth_profile = None
        self.depth_intr: dict | None = None
        self.depth_open_state = self.depth_start_state = None
        self.depth_cb_errors = 0
        self._on_lost = None

    # --- options and profile --------------------------------------------------------------------------------------------
    def _depth_options(self) -> dict:
        out = {}
        for name in SENSOR_KEYS:
            opt = getattr(self.rs.option, name, None)
            sup = opt is not None and bool(self.stereo.supports(opt))
            want = self.depth_cfg.sensor.get(name)
            out[name] = {"supported": sup, "set": want if sup else None,
                         "read_back": float(self.stereo.get_option(opt)) if sup else None}
        return out

    def _pick_depth_profile(self):
        rs, c = self.rs, self.depth_cfg
        offered = []
        for sp in self.stereo.get_stream_profiles():
            if sp.stream_type() != rs.stream.depth or sp.format() != rs.format.z16:
                continue
            v = sp.as_video_stream_profile()
            offered.append((v.width(), v.height(), sp.fps()))
            if (v.width(), v.height(), sp.fps()) == (c.width, c.height, c.fps):
                return sp
        raise RuntimeError(f"no Z16 {c.width}x{c.height}@{c.fps} depth profile (offered {sorted(set(offered))})")

    def _intrinsics(self, sp) -> dict:
        i = sp.as_video_stream_profile().get_intrinsics()
        ci = self.rs.camera_info
        return {"serial": self.dev.get_info(ci.serial_number), "firmware": self.dev.get_info(ci.firmware_version),
                "stream": "depth", "width": int(i.width), "height": int(i.height), "fx": float(i.fx),
                "fy": float(i.fy), "ppx": float(i.ppx), "ppy": float(i.ppy),
                "model": str(i.model).rsplit(".", 1)[-1], "coeffs": [float(x) for x in i.coeffs]}

    # --- start / stop ---------------------------------------------------------------------------------------------------
    def start(self, on_sample=None, on_depth=None, on_lost=None) -> None:
        rs = self.rs
        self._on_lost = on_lost
        try:
            self.ctx.set_devices_changed_callback(self._devices_changed)
        except (RuntimeError, AttributeError):
            pass  # a backend without the notice: the receive gap still opens a generation
        if self.use_motion:
            super().start(on_sample)
        try:
            for name in SENSOR_KEYS:
                want = self.depth_cfg.sensor.get(name)
                if want is None:
                    continue
                opt = getattr(rs.option, name, None)
                if opt is None or not self.stereo.supports(opt):
                    raise RuntimeError(f"the stereo module does not support {name}")
                self.stereo.set_option(opt, float(want))
            sp = self._pick_depth_profile()
            self.depth_profile, self.depth_intr = sp, self._intrinsics(sp)
            meta_keys = [(n, getattr(rs.frame_metadata_value, n)) for n in META_KEYS
                         if hasattr(rs.frame_metadata_value, n)]
            intr = dict(self.depth_intr)

            def cb(f):
                try:
                    df = f.as_depth_frame()
                    pixels = np.array(np.asanyarray(df.get_data()), dtype=np.uint16, copy=True)
                    meta = {n: int(f.get_frame_metadata(k)) for n, k in meta_keys if f.supports_frame_metadata(k)}
                    dom = str(f.get_frame_timestamp_domain()).rsplit(".", 1)[-1]
                    on_depth(int(f.get_frame_number()), float(f.get_timestamp()), dom, meta, float(df.get_units()),
                             intr, pixels)
                except Exception:  # noqa: BLE001 -- never raised into the SDK's thread; counted
                    self.depth_cb_errors += 1

            self.depth_open_state = "calling"
            try:
                self.stereo.open(sp)
            except RuntimeError:
                self.depth_open_state = "failed"
                raise
            self.depth_open_state = "done"
            self.depth_start_state = "calling"
            # a start that raised may still have begun streaming (librealsense turns streaming on before it starts the
            # backend's callbacks): it stays "calling", so the cleanup stops it before it closes (Codex 7e-1 P2-5)
            self.stereo.start(cb)
            self.depth_start_state = "done"
            bad = {n: o for n, o in self._depth_options().items()
                   if o["set"] is not None and o["read_back"] != float(o["set"])}
            if bad:
                raise DepthSettingsMismatch(f"depth options read back differently: {bad}")
        except BaseException:
            try:
                self.stop()  # a single sensor never runs alone (plan 1)
            except RuntimeError:
                pass
            raise

    def _devices_changed(self, info) -> None:
        try:
            removed = info.was_removed(self.dev)
        except Exception:  # noqa: BLE001
            return
        fn = self._on_lost
        if removed and fn is not None:
            fn("device_removed")

    def _others_streaming(self) -> bool:
        return self.depth_start_state is not None or self.depth_open_state is not None

    def depth_profile_info(self) -> dict:
        opts = self._depth_options()
        settings = {n: o["read_back"] for n, o in opts.items()}
        sp = self.depth_profile
        try:
            scale = float(self.stereo.as_depth_sensor().get_depth_scale())
        except (RuntimeError, AttributeError):
            scale = None
        ci = self.rs.camera_info

        def info(key):
            k = getattr(ci, key, None)
            try:
                return None if k is None else str(self.dev.get_info(k))
            except RuntimeError:
                return None

        port = info("physical_port")
        # the capture's facts a timing bound applies to (depth.yaml timing.applies_to, plan 2): the USB link, and the
        # backend as the port's form shows it (a V4L2 device's port is its sysfs path; the libusb backend's is not)
        backend = None if port is None else ("v4l2" if port.startswith("/sys/") else "libusb")
        return {"actual": {"width": self.depth_intr["width"], "height": self.depth_intr["height"],
                           "fps": None if sp is None else int(sp.fps())},
                "capture": {"serial": self.depth_intr["serial"], "firmware": self.depth_intr["firmware"],
                            "usb": info("usb_type_descriptor"), "physical_port": port, "backend": backend,
                            "sdk": getattr(self.rs, "__version__", None)},
                "intrinsics": self.depth_intr, "calib_id": calib_id(self.depth_intr), "depth_scale": scale,
                "options": opts,
                "settings_sha256": hashlib.sha256(json.dumps(settings, sort_keys=True).encode()).hexdigest(),
                "callback_errors": self.depth_cb_errors}

    def stop(self) -> None:
        errors = []
        stop_error = None
        if self.depth_start_state in ("calling", "done"):
            try:
                self.stereo.stop()
                self.depth_start_state = None
            except RuntimeError as e:
                stop_error = f"depth stop ({self.depth_start_state}): {e}"
        # a start that raised ("calling") whose stop failed may never have streamed: the close is tried; only a close
        # that succeeds clears both states -- if it fails too, the next stop() tries the stop again (Codex 7e-1
        # rounds 2-3 P2-1)
        may_close = self.depth_start_state in (None, "failed") or (self.depth_start_state == "calling"
                                                                    and stop_error is not None)
        if self.depth_open_state in ("calling", "done") and may_close:
            try:
                self.stereo.close()
                self.depth_open_state = None
                if self.depth_start_state == "calling":
                    self.depth_start_state = None  # closed: nothing of it can stream
                    stop_error = None
            except RuntimeError as e:
                errors.append(f"depth close ({self.depth_open_state}): {e}")
        if stop_error is not None:
            errors.append(stop_error)
        if self.use_motion:
            try:
                super().stop()
            except RuntimeError as e:
                errors.append(str(e))
        if errors:
            raise RuntimeError("; ".join(errors))


__all__ = ["DepthSettingsMismatch", "META_KEYS", "RealSenseDepthSource"]
