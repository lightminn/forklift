"""PR 7 7e-1 (docs/plans/2026-11-26-7e-depth-pocket-check.md 1): ``tools/depth_device.RealSenseDepthSource`` on a fake
SDK -- the stereo module's options written before its start and read back after (a difference refuses), the exact Z16
profile (no fallback), the depth callback copying the pixels (the SDK frame never kept), a single sensor never
running alone, the device's removal reported to the sources."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
import depth_device as dd  # noqa: E402

from forklift_core.sensors.depth_cfg import calib_id, load_depth_config  # noqa: E402


class _Enum:
    def __init__(self, prefix, names):
        for n in names:
            setattr(self, n, f"{prefix}.{n}")


class Intr:
    width, height, fx, fy, ppx, ppy = 640, 480, 388.8, 388.8, 320.5, 240.25
    model = "distortion.brown_conrady"
    coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]


class Profile:
    def __init__(self, stream, fmt="format.z16", w=640, h=480, fps=15):
        self._s, self._f, self._w, self._h, self._fps = stream, fmt, w, h, fps

    def stream_type(self):
        return self._s

    def format(self):
        return self._f

    def fps(self):
        return self._fps

    def as_video_stream_profile(self):
        return self

    def as_motion_stream_profile(self):
        raise RuntimeError("not motion")

    def width(self):
        return self._w

    def height(self):
        return self._h

    def get_intrinsics(self):
        return Intr()


class Frame:
    def __init__(self, n, buf):
        self.n, self.buf = n, buf

    def as_depth_frame(self):
        return self

    def get_data(self):
        return self.buf

    def get_units(self):
        return 0.001

    def get_frame_number(self):
        return self.n

    def get_timestamp(self):
        return 1000.0 + self.n * 66.7

    def get_frame_timestamp_domain(self):
        return "timestamp_domain.hardware_clock"

    def supports_frame_metadata(self, k):
        return k == "frame_metadata_value.frame_timestamp"

    def get_frame_metadata(self, k):
        return 123456 + self.n


class Stereo:
    def __init__(self, profiles, *, readback=None, unsupported=()):
        self.profiles, self.readback, self.unsupported = profiles, readback or {}, set(unsupported)
        self.opts, self.events, self.cb = {}, [], None

    def get_info(self, key):
        return "Stereo Module"

    def supports(self, opt):
        return opt.startswith("option.") and opt.split(".", 1)[1] not in self.unsupported

    def set_option(self, opt, v):
        self.opts[opt] = float(v)

    def get_option(self, opt):
        name = opt.split(".", 1)[1]
        return self.readback.get(name, self.opts.get(opt, 0.0))

    def get_stream_profiles(self):
        return self.profiles

    def open(self, p):
        self.events.append(("open", p.width(), p.height(), p.fps()))

    def start(self, cb):
        self.events.append(("start",))
        self.cb = cb

    def stop(self):
        self.events.append(("stop",))

    def close(self):
        self.events.append(("close",))

    def as_depth_sensor(self):
        return self

    def get_depth_scale(self):
        return 0.001


class Motion(Stereo):
    def get_info(self, key):
        return "Motion Module"


def fake_rs(stereo, motion):
    class Dev:
        def get_info(self, k):
            return {"camera_info.name": "Intel RealSense D435I", "camera_info.serial_number": "SN1",
                    "camera_info.firmware_version": "5.16"}.get(k, "x")

        def query_sensors(self):
            return [stereo, motion]

    dev = Dev()

    class Ctx:
        cb = None

        def query_devices(self):
            return [dev]

        def set_devices_changed_callback(self, cb):
            Ctx.cb = cb

    return type("rs", (), {
        "context": Ctx, "__version__": "fake", "dev": dev,
        "camera_info": _Enum("camera_info", ["name", "serial_number", "firmware_version", "usb_type_descriptor"]),
        "option": _Enum("option", ["enable_motion_correction", "global_time_enabled", "emitter_enabled",
                                   "enable_auto_exposure", "visual_preset", "laser_power", "asic_temperature",
                                   "projector_temperature", "motion_module_temperature"]),
        "stream": _Enum("stream", ["gyro", "accel", "depth"]),
        "format": _Enum("format", ["motion_xyz32f", "z16"]),
        "frame_metadata_value": _Enum("frame_metadata_value", ["frame_timestamp", "sensor_timestamp"]),
    })


def make(stereo=None, motion_used=False, **kw):
    stereo = stereo or Stereo([Profile("stream.depth", w=848), Profile("stream.depth")])
    motion = Motion([])
    rs = fake_rs(stereo, motion)
    src = dd.RealSenseDepthSource(depth=load_depth_config(), motion=motion_used, rs=rs, **kw)
    return src, stereo, rs


def test_options_profile_and_a_callback_that_copies_the_pixels():
    src, stereo, rs = make()
    assert stereo.opts == {}  # making it only finds the device
    got = []
    src.start(on_depth=lambda *a: got.append(a))
    assert ("open", 640, 480, 15) in stereo.events and ("start",) in stereo.events
    assert list(stereo.opts) == ["option.visual_preset", "option.global_time_enabled", "option.emitter_enabled",
                                 "option.laser_power", "option.enable_auto_exposure"]  # the preset first; null left
    assert stereo.opts["option.visual_preset"] == 1.0 and stereo.opts["option.laser_power"] == 150.0
    buf = np.full((480, 640), 7, dtype=np.uint16)
    stereo.cb(Frame(3, buf))
    buf[:] = 9  # the SDK reuses its buffer: the recorded copy does not move
    [(n, ts, dom, meta, units, intr, pixels)] = got
    assert n == 3 and dom == "hardware_clock" and units == 0.001 and meta == {"frame_timestamp": 123459}
    assert pixels.dtype == np.uint16 and int(pixels[0, 0]) == 7
    prof = src.depth_profile_info()
    assert prof["actual"] == {"width": 640, "height": 480, "fps": 15} and prof["calib_id"] == calib_id(intr)
    assert intr["serial"] == "SN1" and intr["model"] == "brown_conrady" and len(prof["settings_sha256"]) == 64
    assert prof["capture"]["serial"] == "SN1" and "backend" in prof["capture"] and "usb" in prof["capture"]


def test_no_exact_profile_refuses_and_nothing_is_left_running():
    src, stereo, _ = make(Stereo([Profile("stream.depth", w=848)]))
    with pytest.raises(RuntimeError, match="no Z16 640x480@15"):
        src.start(on_depth=lambda *a: None)
    assert ("open", 640, 480, 15) not in stereo.events


def test_a_read_back_difference_refuses_and_stops_what_started():
    src, stereo, _ = make(Stereo([Profile("stream.depth")], readback={"emitter_enabled": 0.0}))
    with pytest.raises(dd.DepthSettingsMismatch, match="emitter_enabled"):
        src.start(on_depth=lambda *a: None)
    assert stereo.events[-2:] == [("stop",), ("close",)]


def test_a_configured_option_the_sensor_lacks_refuses():
    src, stereo, _ = make(Stereo([Profile("stream.depth")], unsupported={"emitter_enabled"}))
    with pytest.raises(RuntimeError, match="does not support emitter_enabled"):
        src.start(on_depth=lambda *a: None)


def test_the_devices_removal_reaches_on_lost_and_a_failing_callback_is_counted():
    src, stereo, rs = make()
    lost = []
    src.start(on_depth=lambda *a: (_ for _ in ()).throw(ValueError("x")), on_lost=lost.append)

    class Info:
        def was_removed(self, dev):
            return dev is rs.dev

    rs.context.cb(Info())
    assert lost == ["device_removed"]
    stereo.cb(Frame(1, np.zeros((480, 640), dtype=np.uint16)))
    assert src.depth_cb_errors == 1  # never raised into the SDK's thread


def test_a_start_that_raised_after_streaming_began_is_still_stopped():
    """Codex 7e-1 round 1 P2-5: librealsense turns streaming on before it starts the backend's callbacks -- a start
    that raised stays a cleanup target: stopped before it is closed."""
    class Raising(Stereo):
        def start(self, cb):
            self.events.append(("start",))
            self.cb = cb
            raise RuntimeError("backend failed after streaming began")

    src, stereo, _ = make(Raising([Profile("stream.depth")]))
    with pytest.raises(RuntimeError, match="backend failed"):
        src.start(on_depth=lambda *a: None)
    assert stereo.events[-2:] == [("stop",), ("close",)]


def test_a_start_that_raised_before_streaming_began_still_closes_the_sensor():
    """Codex 7e-1 round 2 P2-1: the SDK raised before it turned streaming on -- its stop says 'not streaming'; the
    opened sensor is closed all the same (and after one that raised after streaming began, above)."""
    class EarlyRaise(Stereo):
        def start(self, cb):
            self.events.append(("start",))
            raise RuntimeError("could not start")

        def stop(self):
            self.events.append(("stop",))
            raise RuntimeError("not streaming")

    src, stereo, _ = make(EarlyRaise([Profile("stream.depth")]))
    with pytest.raises(RuntimeError, match="could not start"):
        src.start(on_depth=lambda *a: None)
    assert stereo.events[-2:] == [("stop",), ("close",)] and src.depth_open_state is None


def test_an_exposure_ceiling_needs_its_toggle_and_auto_exposure(tmp_path):
    import yaml

    from forklift_core.sensors.depth_cfg import DEPTH_YAML

    raw = yaml.safe_load(DEPTH_YAML.read_text())
    raw["sensor"]["auto_exposure_limit"] = 8000
    p = tmp_path / "d.yaml"
    p.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match="auto_exposure_limit_toggle"):
        load_depth_config(p)
    raw["sensor"]["auto_exposure_limit_toggle"] = 1
    p.write_text(yaml.safe_dump(raw))
    assert load_depth_config(p).sensor["auto_exposure_limit"] == 8000


def test_a_stop_that_failed_once_after_a_partial_start_is_tried_again_when_the_close_fails_too():
    """Codex 7e-1 round 3 P2-1: streaming began, the first stop failed, the close was refused under the stream --
    the state is kept, so the next stop() stops it and then closes."""
    class Sticky(Stereo):
        stops = 0

        def start(self, cb):
            self.events.append(("start",))
            raise RuntimeError("backend failed after streaming began")

        def stop(self):
            Sticky.stops += 1
            self.events.append(("stop",))
            if Sticky.stops == 1:
                raise RuntimeError("busy")

        def close(self):
            self.events.append(("close",))
            if Sticky.stops < 2:
                raise RuntimeError("streaming")

    src, stereo, _ = make(Sticky([Profile("stream.depth")]))
    with pytest.raises(RuntimeError, match="backend failed"):
        src.start(on_depth=lambda *a: None)
    assert src.depth_start_state == "calling" and src.depth_open_state == "done"
    src.stop()
    assert stereo.events[-2:] == [("stop",), ("close",)] and src.depth_open_state is None


def test_the_motion_module_alone_reports_the_removal_to_on_lost():
    """Unlock U2a (Codex round 1 P2-4): the operating run's IMU device (``imu_record.RealSenseSource``) takes the
    removal callback without the depth stream."""
    import imu_record

    class MotionOnly(Motion):
        def open(self, profiles):
            self.events.append(("open", len(profiles)))

    motion = MotionOnly([Profile("stream.gyro", "format.motion_xyz32f", fps=200),
                         Profile("stream.accel", "format.motion_xyz32f", fps=200)])
    rs = fake_rs(Stereo([]), motion)
    src = imu_record.RealSenseSource(gyro_hz=200, accel_hz=200, global_time=False, rs=rs)
    lost = []
    src.start(lambda *a: None, on_lost=lost.append)
    assert ("open", 2) in motion.events and ("start",) in motion.events

    class Info:
        def __init__(self, removed):
            self.removed = removed

        def was_removed(self, dev):
            return self.removed and dev is rs.dev

    rs.context.cb(Info(False))
    assert lost == []
    rs.context.cb(Info(True))
    assert lost == ["device_removed"]
    src.stop()
    rs.context.cb(Info(True))
    assert lost == ["device_removed"]  # stopped: the notice reports to no one (Codex U2a round 2 P3)
    src.start(lambda *a: None)  # started again without on_lost
    rs.context.cb(Info(True))
    assert lost == ["device_removed"]
