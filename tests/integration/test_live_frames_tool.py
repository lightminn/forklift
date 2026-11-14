"""tools/live_frames.py: capture the live console's camera frames as the page receives
them, and judge whether the overlay colours show (plan 2026-11-14 path display, S4).

The capture runs against the live runner's own console server (console/live/server.py)
fed by a stand-in thread, and against a small recording server that frames pictures with
the same encoder, to see what the tool sends and how it treats resends and a new session.
"""

import asyncio
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("websockets")
pytest.importorskip("PIL")

from PIL import Image  # noqa: E402
from websockets.asyncio.server import serve  # noqa: E402

from console import wire  # noqa: E402
from console.live.server import (  # noqa: E402
    LiveServer,
    MediaStore,
    camera_item,
    encode_jpeg,
    scan_item,
)
from tools import live_frames  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
EPOCH = 41


def picture(k: int, h: int = 48, w: int = 64) -> np.ndarray:
    rgb = np.full((h, w, 3), 60, dtype=np.uint8)
    rgb[:, k % w] = 200  # a moving column, so frames differ
    return rgb


class Feeder:
    """The runner's side: the live console server and a camera frame every 30 ms."""

    def __init__(self):
        self.store = MediaStore()
        self.server = LiveServer(SimpleNamespace(take_out=lambda: []), self.store, {})
        self.pictures: dict[int, np.ndarray] = {}
        self._stop = threading.Event()

    def run(self):
        k = 0
        while not self._stop.is_set():
            k += 1
            rgb = picture(k)
            self.pictures[k] = rgb
            item = camera_item(k, rgb, time.monotonic(), k * 0.1, EPOCH)
            self.store.put("camera_rgb", k, item)
            time.sleep(0.03)

    def __enter__(self):
        self.port = self.server.start()
        self.url = f"ws://127.0.0.1:{self.port}"
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self.thread.join(2)
        self.server.stop()


@pytest.fixture
def feeder():
    with Feeder() as f:
        yield f


def read_index(out: Path) -> list[dict]:
    return [json.loads(line) for line in (out / "index.jsonl").read_text().splitlines()]


def test_capture_saves_each_frame_as_received_with_an_index_row(tmp_path, feeder):
    out = tmp_path / "cap"
    result = asyncio.run(live_frames.capture(feeder.url, out, max_frames=4))
    assert result["reason"] == "max_frames" and result["kept"] == 4

    rows = read_index(out)
    frames = [r for r in rows if r["t"] == "frame"]
    assert len(frames) == 4
    assert rows[-1]["t"] == "end" and rows[-1]["kept"] == 4
    assert rows[-1]["reason"] == "max_frames" and rows[-1]["frames"] >= 4
    ids = [r["frame_id"] for r in frames]
    assert ids == sorted(ids) and len(set(ids)) == 4
    for r in frames:
        path = out / r["file"]
        assert r["file"] == f"frames/{r['frame_id']}.jpg"
        data = path.read_bytes()
        assert r["bytes"] == len(data)
        # byte for byte what the server encoded for that frame
        assert data == encode_jpeg(feeder.pictures[r["frame_id"]])
        with Image.open(path) as im:
            assert im.size == (64, 48)
        assert r["session_epoch"] == EPOCH
        assert r["stamp_sim_s"] == pytest.approx(r["frame_id"] * 0.1, abs=1e-6)
        assert isinstance(r["acquired_mono"], float)
        assert isinstance(r["recv_mono_ns"], int)
        assert (r["width"], r["height"]) == (64, 48)

    meta = json.loads((out / "meta.json").read_text())
    assert meta["url"] == feeder.url
    assert meta["media_url"] == feeder.url + "/api/v1/media"
    assert meta["version"] == live_frames.TOOL_VERSION
    assert meta["options"]["max_frames"] == 4 and meta["options"]["every"] == 1
    assert "started_utc" in meta


def test_capture_keeps_every_nth_frame(tmp_path, feeder):
    out = tmp_path / "cap"
    result = asyncio.run(live_frames.capture(feeder.url, out, every=3, max_frames=2))
    frames = [r for r in read_index(out) if r["t"] == "frame"]
    assert [r["n"] for r in frames] == [1, 4]
    assert result["frames"] == 4 and result["kept"] == 2
    assert len(list((out / "frames").iterdir())) == 2


def test_capture_refuses_a_non_empty_directory(tmp_path):
    out = tmp_path / "cap"
    out.mkdir()
    (out / "earlier.txt").write_text("x")
    rc = live_frames.main(["capture", "--url", "ws://127.0.0.1:9", "--out", str(out)])
    assert rc == 2
    assert sorted(p.name for p in out.iterdir()) == ["earlier.txt"]


def test_capture_ends_after_its_seconds(tmp_path, feeder):
    out = tmp_path / "cap"
    t0 = time.monotonic()
    result = asyncio.run(live_frames.capture(feeder.url, out, seconds=0.6))
    assert 0.5 < time.monotonic() - t0 < 3.0
    assert result["reason"] == "seconds" and result["kept"] >= 1
    assert read_index(out)[-1]["reason"] == "seconds"


def test_capture_stops_cleanly_when_asked(tmp_path, feeder):
    out = tmp_path / "cap"

    async def run():
        stop = asyncio.Event()
        asyncio.get_running_loop().call_later(0.5, stop.set)
        return await live_frames.capture(feeder.url, out, stop=stop)

    result = asyncio.run(run())
    assert result["reason"] == "interrupted"
    end = read_index(out)[-1]
    assert end["t"] == "end" and end["kept"] == result["kept"] >= 1


def test_capture_ends_when_the_runner_ends(tmp_path, feeder):
    out = tmp_path / "cap"

    async def run():
        loop = asyncio.get_running_loop()
        loop.call_later(0.5, lambda: loop.run_in_executor(None, feeder.server.stop))
        return await live_frames.capture(feeder.url, out, seconds=10)

    result = asyncio.run(run())
    assert result["reason"] == "runner_ended"
    rows = read_index(out)
    assert any(r["t"] == "closed" and r["code"] == 1001 for r in rows)


def test_ctrl_c_writes_the_end_row(tmp_path, feeder):
    out = tmp_path / "cap"
    env = dict(os.environ, PYTHONPATH=str(ROOT / "src"))
    proc = subprocess.Popen(
        [sys.executable, str(ROOT / "tools/live_frames.py"), "capture", "--url", feeder.url,
         "--out", str(out)],
        cwd=tmp_path, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            index = out / "index.jsonl"
            if index.exists() and '"t": "frame"' in index.read_text():
                break
            time.sleep(0.1)
        proc.send_signal(signal.SIGINT)
        stdout, stderr = proc.communicate(timeout=10)
    finally:
        proc.kill()
    assert proc.returncode == 0, stderr.decode()
    end = read_index(out)[-1]
    assert end["t"] == "end" and end["reason"] == "interrupted" and end["kept"] >= 1


class Recorder:
    """A media server that records what a client sends and frames pictures as the live
    server does (the same encoder, the head with jpeg_bytes): a resend of one frame, a
    new session reusing a frame id, a scan, and a body longer than its JPEG."""

    def __init__(self):
        self.paths: list[str] = []
        self.messages: list = []

    async def handler(self, ws):
        self.paths.append(ws.request.path)
        self.messages.append(json.loads(await ws.recv()))
        for epoch, fid in ((1, 1), (1, 1), (1, 2), (2, 1)):
            item = camera_item(fid, picture(fid), 1.0, fid * 0.1, epoch)
            jpeg = encode_jpeg(item["rgb"])
            await ws.send(wire.frame(dict(item["head"], jpeg_bytes=len(jpeg)), jpeg + b"pad"))
            scan = scan_item([1.0, 2.0], 0.0, 0.1, 5.0, {}, epoch, "map")
            await ws.send(wire.frame(scan["head"], scan["body"]))
        await ws.send(json.dumps({"t": "noise"}))
        try:
            async for raw in ws:
                self.messages.append(json.loads(raw))
        except Exception:  # noqa: BLE001 -- the client closing is the end
            pass


def test_capture_is_an_observer_and_mirrors_the_page(tmp_path):
    rec = Recorder()
    out = tmp_path / "cap"

    async def run():
        async with serve(rec.handler, "127.0.0.1", 0, compression=None) as server:
            port = server.sockets[0].getsockname()[1]
            return await live_frames.capture(f"ws://127.0.0.1:{port}", out, max_frames=3)

    result = asyncio.run(run())
    assert rec.paths == ["/api/v1/media"]  # never the control socket
    assert len(rec.messages) == 1
    sub = rec.messages[0]
    assert sub["t"] == "media_subscribe" and set(sub) == {"t", "client_id", "camera_rgb"}
    frames = [r for r in read_index(out) if r["t"] == "frame"]
    assert [(r["session_epoch"], r["frame_id"], r["file"]) for r in frames] == [
        (1, 1, "frames/1.jpg"), (1, 2, "frames/2.jpg"), (2, 1, "frames/1_e2.jpg"),
    ]
    assert result["duplicates"] == 1 and result["reason"] == "max_frames"
    # the body is cut at jpeg_bytes, as the page does
    assert (out / "frames/2.jpg").read_bytes() == encode_jpeg(picture(2))


def test_media_url_takes_a_host_and_port_or_the_media_path():
    assert live_frames.media_url("ws://h:1") == "ws://h:1/api/v1/media"
    assert live_frames.media_url("ws://h:1/") == "ws://h:1/api/v1/media"
    assert live_frames.media_url("ws://h:1/api/v1/media") == "ws://h:1/api/v1/media"
    with pytest.raises(ValueError):
        live_frames.media_url("ws://h:1/api/v1/control")
    with pytest.raises(ValueError):
        live_frames.media_url("http://h:1")


def test_overlay_colours_come_from_the_runner_source():
    colours = live_frames.overlay_colours()
    assert {"Approach", "Transport", "Return", "Observe", "destination", "contrast"} <= set(colours)
    rgbs = [tuple(c["rgb"]) for c in colours.values()]
    assert len(set(rgbs)) == len(rgbs)
    for c in colours.values():
        assert len(c["rgb"]) == 3 and all(isinstance(v, int) and 0 <= v <= 255 for v in c["rgb"])
        assert c["sources"] and all(":" in s for s in c["sources"])


def write_arm(directory: Path, pictures: list[np.ndarray]) -> None:
    (directory / "frames").mkdir(parents=True)
    rows = []
    for i, rgb in enumerate(pictures, start=1):
        data = encode_jpeg(rgb)
        (directory / "frames" / f"{i}.jpg").write_bytes(data)
        rows.append({"t": "frame", "n": i, "frame_id": i, "session_epoch": 1,
                     "file": f"frames/{i}.jpg", "bytes": len(data)})
    rows.append({"t": "end", "frames": len(pictures), "kept": len(pictures), "reason": "test"})
    (directory / "index.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))


def test_judge_reports_each_colour_for_both_arms(tmp_path, capsys):
    colours = live_frames.overlay_colours()
    transport, approach = colours["Transport"]["rgb"], colours["Approach"]["rgb"]
    drawn = []
    for k in range(3):
        rgb = np.full((180, 320, 3), 70, dtype=np.uint8)
        rgb[60 + 10 * k:68 + 10 * k, 20:300] = transport  # an 8 px line
        rgb[20:170, 150 + k:158 + k] = approach
        drawn.append(rgb)
    blank = [np.full((180, 320, 3), 70, dtype=np.uint8) for _ in range(3)]
    write_arm(tmp_path / "a", drawn)
    write_arm(tmp_path / "b", blank)
    (tmp_path / "a" / "frames" / "4.jpg").write_bytes(b"not a jpeg")
    rows = (tmp_path / "a" / "index.jsonl").read_text().splitlines()
    bad = json.dumps({"t": "frame", "n": 4, "frame_id": 4, "file": "frames/4.jpg"})
    (tmp_path / "a" / "index.jsonl").write_text("\n".join(rows[:-1] + [bad, rows[-1]]) + "\n")

    rc = live_frames.main(["judge", str(tmp_path / "a"), "--compare", str(tmp_path / "b"),
                           "--per-frame"])
    assert rc == 0
    report = json.loads(capsys.readouterr().out)
    assert report["tol"] == 40 and report["min_component"] == 50
    arm, other = report["arms"]["arm"], report["arms"]["compare"]
    assert arm["dir"] == str(tmp_path / "a") and other["dir"] == str(tmp_path / "b")
    assert arm["frames"] == 4 and arm["unreadable"] == 1 and other["frames"] == 3
    assert arm["colours"]["Transport"]["frames_with_component"] == 3
    assert arm["colours"]["Transport"]["fraction"] == pytest.approx(1.0)
    assert arm["colours"]["Approach"]["fraction"] == pytest.approx(1.0)
    assert arm["colours"]["Return"]["frames_with_component"] == 0
    assert other["colours"]["Transport"]["fraction"] == pytest.approx(0.0)
    assert report["by_colour"]["Transport"]["arm"]["fraction"] == pytest.approx(1.0)
    assert report["by_colour"]["Transport"]["compare"]["fraction"] == pytest.approx(0.0)
    assert [f["frame_id"] for f in arm["per_frame"]] == [1, 2, 3, 4]
    assert arm["per_frame"][3]["colours"] is None
    assert report["colours"]["Transport"]["rgb"] == transport
    text = json.dumps(report)
    assert '"pass"' not in text and '"verdict"' not in text  # thresholds belong to S4


def test_judge_takes_an_extra_colour(tmp_path, capsys):
    rgb = np.full((60, 80, 3), 70, dtype=np.uint8)
    rgb[20:30, 5:75] = (10, 250, 250)
    write_arm(tmp_path / "a", [rgb])
    rc = live_frames.main(["judge", str(tmp_path / "a"), "--colour", "probe=10,250,250"])
    assert rc == 0
    report = json.loads(capsys.readouterr().out)
    assert report["colours"]["probe"] == {"rgb": [10, 250, 250], "sources": ["--colour"]}
    assert report["arms"]["arm"]["colours"]["probe"]["frames_with_component"] == 1
    assert "compare" not in report["arms"]


# ------------------------------------------------------------------ delta
# A live run's records as the runner leaves them: base_link's pose per physics step in the spooled SLAM log
# (meta.json "spool", slam_log.joint: joint_stamps_s, base_pose_world x y z qw qx qy qz), the lift in the spooled
# state.samples, the camera mount and the live video camera's K in result.json, and each frame's compose step in
# live_log.jsonl's camera lines.

FX = 465.741156  # the rig's focal length (tools/scene_rig.py), the live video camera's at 640 x 480
LAG = 11 / 120
# base_link <- optical (ROS axes): looking along the base's +x, level
LEVEL = [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
LINE_Z = 0.012  # path_overlay.LINE_Z_M


def q_yaw(a):
    return [np.cos(a / 2), 0.0, 0.0, np.sin(a / 2)]


RUN_EPOCH = 7  # the session epoch of make_run's run (its live log's start line) and make_capture's frames


def make_run(run: Path, base_xy_yaw, *, end_s: float, lift_m: float, mount_z: float = 0.4,
             frames=(), intrinsics=True, epoch=RUN_EPOCH) -> None:
    """``base_xy_yaw(t) -> (x, y, yaw)`` at 120 Hz over [0, end_s]; ``frames``: (frame_id, stamp, compose time);
    ``epoch``: the session epoch the live log's start line records (None: no start line)."""
    from forklift_core.live.spool import RecordSpool

    run.mkdir(parents=True)
    stamps = list(np.arange(int(round(end_s * 120)) + 1) / 120)
    poses = [np.array([*base_xy_yaw(t)[:2], 0.0, *q_yaw(base_xy_yaw(t)[2])]) for t in stamps]
    samples = [{"time_s": t, "phase": "approach", "lift_m": lift_m} for t in stamps[::12]]
    spool = RecordSpool(run / "spool", interval_s=1e9)
    spool.register_group("slam_log.joint", {"joint_stamps_s": stamps, "base_pose_world": poses})
    spool.register_group("slam_log.scan", {"scan_stamps_s": [], "scan_ranges_m": []})
    spool.register("slam_log.silenced", [])
    spool.register("state.samples", samples)
    assert spool.close()["complete"]
    (run / "meta.json").write_text(json.dumps({"format": "forklift_slam_log_v1", "spool": {
        "joint": "slam_log.joint", "scan": "slam_log.scan", "silenced": "slam_log.silenced", "root": "spool"}}))
    result = {"perception_mount": {"name": "test", "parent": "/World/Forklift/fork_carriage",
                                   "translation_m": [0.0, 0.0, mount_z], "rotation": LEVEL},
              "spool": {"root": "spool", "complete": True}}
    if intrinsics:
        result["live_video_camera"] = {"intrinsics": {"width": 640, "height": 480, "fx": FX, "fy": FX,
                                                      "cx": 320.0, "cy": 240.0, "frame_id": "camera_optical_frame"}}
    (run / "result.json").write_text(json.dumps(result))
    lines = [{"log_writer": "ordered"}]
    if epoch is not None:
        lines.append({"event": "start", "port": 1, "session_epoch": epoch, "mono": 0.0})
    lines += [{"camera": fid, "render_time": 100.0 + stamp, "sim_t": compose} for fid, stamp, compose in frames]
    lines.append({"log_end": {"dropped_lines": 0, "line_errors": 0}})
    (run / "live_log.jsonl").write_text("".join(json.dumps(x) + "\n" for x in lines))


def make_capture(out: Path, rows) -> None:
    """``rows``: (frame_id, stamp) in RUN_EPOCH or (frame_id, stamp, session_epoch)."""
    out.mkdir(parents=True)
    text = "".join(json.dumps({"t": "frame", "frame_id": r[0], "session_epoch": r[2] if len(r) > 2 else RUN_EPOCH,
                               "stamp_sim_s": r[1], "width": 640, "height": 480,
                               "file": f"frames/{r[0]}.jpg"}) + "\n"
                   for r in rows)
    (out / "index.jsonl").write_text(text + json.dumps({"t": "end"}) + "\n")


def write_events(path: Path, events) -> Path:
    path.write_text("".join(json.dumps(e) + "\n" for e in events))
    return path


def run_delta(capsys, *argv):
    rc = live_frames.main(["delta", *map(str, argv)])
    captured = capsys.readouterr()
    return rc, (json.loads(captured.out) if rc == 0 else captured.err)


def test_delta_of_a_straight_drive_is_near_zero(tmp_path, capsys):
    speed = 0.5
    stamps = [0.5, 1.0, 1.15, 1.5]
    frames = [(i + 1, s, s + LAG) for i, s in enumerate(stamps)]
    make_run(tmp_path / "run", lambda t: (speed * t, 0.0, 0.0), end_s=2.0, lift_m=0.0,
             frames=frames + [(5, 0.05, 0.05 + LAG)])
    make_capture(tmp_path / "cap", [(f[0], f[1]) for f in frames] + [(99, 1.2), (98, None), (5, 0.05)])
    ahead = [[x, 0.0, 0.0] for x in np.linspace(6.0, 16.0, 50)]
    events = write_events(tmp_path / "paths.jsonl", [
        {"time_s": 0.0, "name": "Approach", "poses": ahead},
        {"time_s": 1.2, "name": "Transport", "poses": ahead[25:]},  # after 1.15's render, before its compose
    ])
    rc, report = run_delta(capsys, "--run", tmp_path / "run", "--frames", tmp_path / "cap", "--paths", events,
                           "--per-frame")
    assert rc == 0, report
    assert report["lag_settings_s"] == [0.0, pytest.approx(LAG)]
    assert report["sources"]["paths_time_resolved"] is True
    assert report["sources"]["rides_lift"] is True
    assert report["sources"]["compose_time"].startswith("live_log.jsonl")
    assert report["kind"] == "offline point-sample estimate"
    assert set(report["approximations"]) >= {"pose", "lines"}
    assert report["session_epoch"] == RUN_EPOCH
    assert report["sources"]["session_epoch"].startswith("live_log.jsonl")
    assert report["counts"]["measured"] == 4
    # 99: not in the run's log; 98: no stamp; 5: in the log, but 0.05 - 0.0917 is before the first pose
    assert report["counts"]["skipped"] == {"other_epoch": 0, "no_stamp": 1, "no_compose_entry": 1,
                                           "compose_before_render": 0, "outside_poses": 1, "no_path": 0}
    rows = {r["frame_id"]: r for r in report["per_frame"]}
    assert rows[2]["paths"] == ["Approach"] and rows[3]["paths"] == ["Approach", "Transport"]
    h = 0.4 - LINE_Z
    for fid, stamp in zip((1, 2, 3, 4), stamps, strict=True):
        r = rows[fid]
        assert r["motion"] == "straight" and r["speed_mps"] == pytest.approx(speed)
        # the nearest point (6 m) moves most: only v changes, fy h (1/d_drawn - 1/d_true)
        d_drawn, d_true = 6.0 - speed * stamp, 6.0 - speed * (stamp - LAG)
        assert r["shift"]["0"]["max_px"] == pytest.approx(FX * h * (1 / d_drawn - 1 / d_true), rel=1e-6)
        assert r["shift"]["0"]["max_px"] < 0.5
        assert r["shift"]["0.0916667"]["max_px"] == pytest.approx(0.0, abs=1e-9)
    s = report["settings"]["0"]
    assert s["all"]["measured"] == 4 and s["straight"]["measured"] == 4 and s["turning"]["measured"] == 0
    assert s["all"]["max_px"] == pytest.approx(max(r["shift"]["0"]["max_px"] for r in rows.values()))
    assert s["turning"]["max_px"] is None


def test_delta_of_a_turn_is_fx_tan_omega_lag(tmp_path, capsys):
    omega = 0.3
    stamps = [1.95, 2.0, 2.05, 2.1]
    frames = [(i + 1, s, s + LAG) for i, s in enumerate(stamps)]
    # turning in place about the camera's own vertical, the forks raised 0.05 m
    make_run(tmp_path / "run", lambda t: (0.0, 0.0, omega * (t - 2.0)), end_s=4.0, lift_m=0.05, frames=frames)
    make_capture(tmp_path / "cap", [(f[0], f[1]) for f in frames])
    events = write_events(tmp_path / "paths.jsonl", [
        {"time_s": 0.0, "name": "Approach", "poses": [[x, 0.0, 0.0] for x in np.linspace(2.0, 6.0, 40)]}])
    rc, report = run_delta(capsys, "--run", tmp_path / "run", "--frames", tmp_path / "cap", "--paths", events,
                           "--lag-setting-s", "0", "--lag-setting-s", "0.05", "--per-frame")
    assert rc == 0, report
    assert sorted(report["settings"]) == ["0", "0.05"]
    h = 0.45 - LINE_Z
    for r, stamp in zip(report["per_frame"], stamps, strict=True):
        assert r["motion"] == "turning" and r["yaw_rate_radps"] == pytest.approx(omega)
        for setting, key in ((0.0, "0"), (0.05, "0.05")):
            ya, yb = omega * (stamp - setting - 2.0), omega * (stamp - LAG - 2.0)
            du = FX * (np.tan(ya) - np.tan(yb))  # every point on the ray moves alike across
            dv = FX * h / 2.0 * (1 / np.cos(ya) - 1 / np.cos(yb))  # the nearest point (2 m) most down
            assert r["shift"][key]["max_px"] == pytest.approx(np.hypot(du, dv), rel=1e-6)
        assert r["shift"]["0"]["max_px"] == pytest.approx(FX * np.tan(omega * LAG), rel=0.01)  # about 12.8 px
    turning = report["settings"]["0"]["turning"]
    assert turning["measured"] == 4 and report["settings"]["0"]["straight"]["measured"] == 0
    assert turning["max_px"] == pytest.approx(FX * np.tan(omega * LAG), rel=0.01)
    assert report["settings"]["0.05"]["all"]["max_px"] == pytest.approx(FX * np.tan(omega * (LAG - 0.05)), rel=0.01)


def test_delta_names_what_the_run_does_not_record(tmp_path, capsys):
    frames = [(1, 1.0, 1.0 + LAG)]
    make_run(tmp_path / "run", lambda t: (0.5 * t, 0.0, 0.0), end_s=2.0, lift_m=0.0, frames=frames, intrinsics=False)
    make_capture(tmp_path / "cap", [(1, 1.0)])
    static = tmp_path / "paths.json"  # paths.json as the runner writes it: the last plan per leg, no time
    static.write_text(json.dumps({"approach": {"success": True, "poses": [[x, 0.0, 0.0] for x in (6.0, 8.0)]},
                                  "transport": {"success": True, "poses": [[x, 0.0, 0.0] for x in (9.0, 10.0)]}}))
    base = ["--run", tmp_path / "run", "--frames", tmp_path / "cap"]
    rc, err = run_delta(capsys, *base)
    assert rc == 2 and "--intrinsics-json" in err
    k = tmp_path / "k.json"
    k.write_text(json.dumps({"fx": FX, "fy": FX, "cx": 320.0, "cy": 240.0, "width": 640, "height": 480}))
    rc, err = run_delta(capsys, *base, "--intrinsics-json", k)
    assert rc == 2 and "--paths" in err and "time" in err
    rc, report = run_delta(capsys, *base, "--intrinsics-json", k, "--paths", static)
    assert rc == 0
    assert report["sources"]["paths_time_resolved"] is False
    assert report["sources"]["intrinsics"].startswith("--intrinsics-json")
    assert report["counts"]["measured"] == 1
    assert '"pass"' not in json.dumps(report) and '"verdict"' not in json.dumps(report)
    (tmp_path / "run" / "meta.json").unlink()  # no SLAM log: no pose track
    rc, err = run_delta(capsys, *base, "--intrinsics-json", k, "--paths", static)
    assert rc == 2 and "--record-slam" in err


def test_delta_joins_frames_on_the_runs_session_epoch(tmp_path, capsys):
    # the run (epoch 7) composed frames 1 and 2; the capture also holds epoch 8's frames 1 and 2 (a reconnect to a
    # restarted runner: its ids start again), epoch 8's frame 2 within a second of epoch 7's compose time
    frames = [(1, 1.0, 1.0 + LAG), (2, 1.1, 1.1 + LAG)]
    captured = [(1, 1.0), (2, 1.1), (1, 1.02, 8), (2, 1.15, 8), (3, 1.2)]
    events = write_events(tmp_path / "paths.jsonl", [
        {"time_s": 0.0, "name": "Approach", "poses": [[x, 0.0, 0.0] for x in (4.0, 8.0)]}])
    make_run(tmp_path / "run", lambda t: (0.5 * t, 0.0, 0.0), end_s=2.0, lift_m=0.0, frames=frames)
    make_capture(tmp_path / "cap", captured)
    base = ["--frames", tmp_path / "cap", "--paths", events, "--per-frame"]
    rc, report = run_delta(capsys, "--run", tmp_path / "run", *base)
    assert rc == 0, report
    assert report["session_epoch"] == RUN_EPOCH and report["capture_session_epochs"] == [7, 8]
    assert [(r["session_epoch"], r["frame_id"]) for r in report["per_frame"]] == [(7, 1), (7, 2)]
    skipped = report["counts"]["skipped"]
    assert skipped["other_epoch"] == 2 and skipped["no_compose_entry"] == 1  # 8's two frames; 7's frame 3
    assert report["sources"]["compose_time"].endswith("(session_epoch, frame_id)")
    rc, err = run_delta(capsys, "--run", tmp_path / "run", *base, "--session-epoch", 8)
    assert rc == 2 and "is not this run's session epoch 7" in err

    # a run that does not record its epoch: with two epochs in the capture, --session-epoch is required
    make_run(tmp_path / "bare", lambda t: (0.5 * t, 0.0, 0.0), end_s=2.0, lift_m=0.0, frames=frames, epoch=None)
    rc, err = run_delta(capsys, "--run", tmp_path / "bare", *base)
    assert rc == 2 and "--session-epoch" in err and "[7, 8]" in err
    rc, report = run_delta(capsys, "--run", tmp_path / "bare", *base, "--session-epoch", 7)
    assert rc == 0 and report["counts"]["measured"] == 2 and report["sources"]["session_epoch"] == "--session-epoch"
    # the reloc console's record names it too
    result = json.loads((tmp_path / "bare" / "result.json").read_text())
    result["reloc_console"] = {"port": 1, "session_epoch": 7, "gaps": []}
    (tmp_path / "bare" / "result.json").write_text(json.dumps(result))
    rc, report = run_delta(capsys, "--run", tmp_path / "bare", *base)
    assert rc == 0 and report["sources"]["session_epoch"] == "result.json reloc_console.session_epoch"
    assert report["counts"]["measured"] == 2 and report["counts"]["skipped"]["other_epoch"] == 2


def test_delta_samples_each_line_and_leaves_unseen_frames_out(tmp_path, capsys):
    # a line straight across the view 3 m out, its two ends 20 m to either side: no end point is ever on the
    # picture. At 1.0 s the truck (0.5 m/s) is 2.5 m short of it; at 7.5 s it has passed it.
    frames = [(1, 1.0, 1.0 + LAG), (2, 7.5, 7.5 + LAG)]
    make_run(tmp_path / "run", lambda t: (0.5 * t, 0.0, 0.0), end_s=8.0, lift_m=0.0, frames=frames)
    make_capture(tmp_path / "cap", [(1, 1.0), (2, 7.5)])
    events = write_events(tmp_path / "paths.jsonl", [
        {"time_s": 0.0, "name": "Transport", "points": [[3.0, 20.0], [3.0, -20.0]]}])
    base = ["--run", tmp_path / "run", "--frames", tmp_path / "cap", "--paths", events, "--per-frame"]
    rc, report = run_delta(capsys, *base)
    assert rc == 0, report
    assert report["sample_m"] == pytest.approx(0.02)
    seen, gone = report["per_frame"]
    assert seen["shift"]["0"]["visible_samples"] > 0 and seen["shift"]["0"]["max_px"] > 0
    assert gone["shift"]["0"]["visible_samples"] == 0 and gone["shift"]["0"]["max_px"] is None
    s = report["settings"]["0"]["all"]
    assert s["no_visible"] == 1 and s["measured"] == 1  # the unseen frame is counted, not summarised as 0
    assert s["max_px"] == pytest.approx(seen["shift"]["0"]["max_px"])
    assert s["median_px"] == pytest.approx(seen["shift"]["0"]["max_px"])
    assert report["counts"]["no_visible"] == {"0": 1, "0.0916667": 1}
    # sampled only at its two ends, as the points themselves, the line is never on the picture
    rc, report = run_delta(capsys, *base, "--sample-m", 100)
    assert rc == 0
    s = report["settings"]["0"]["all"]
    assert s["no_visible"] == 2 and s["measured"] == 0 and s["max_px"] is None and s["median_px"] is None
    rc, err = run_delta(capsys, *base, "--sample-m", 0)
    assert rc == 2 and "--sample-m" in err


def test_delta_help_says_what_it_is(capsys):
    with pytest.raises(SystemExit):
        live_frames.main(["--help"])
    assert "offline point-sample estimate" in capsys.readouterr().out
    with pytest.raises(SystemExit):
        live_frames.main(["delta", "--help"])
    text = " ".join(capsys.readouterr().out.split())
    assert "--sample-m" in text and "path_display.jsonl" in text and "{time_s, name, poses|points}" in text


def ring(x, y, n=96, r=0.63):
    a = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    return [[x + r * np.cos(v), y + r * np.sin(v), 0.006] for v in a]


def test_delta_reads_the_runs_path_display_record(tmp_path, capsys):
    """The format path_overlay writes since PR #513: kind path/ring, a ring named by its prim path, time_s null
    when the runner never fixed its origin, the overlay's own counts in path_display.json."""
    frames = [(1, 1.0, 1.0 + LAG), (2, 1.5, 1.5 + LAG)]
    make_run(tmp_path / "run", lambda t: (0.5 * t, 0.0, 0.0), end_s=2.0, lift_m=0.0, frames=frames)
    make_capture(tmp_path / "cap", [(1, 1.0), (2, 1.5)])
    ahead = [[x, 0.0, LINE_Z] for x in np.linspace(3.0, 8.0, 30)]
    header = {"t": "header", "schema": "path_display/1", "mode": "overlay", "pixel_lag_s": 0.2,
              "time_axis": "world.current_time - initial_time", "line_width_px_at_1280": 3}
    green = [0.01, 0.9, 0.04]
    write_events(tmp_path / "run" / "path_display.jsonl", [
        header,
        {"t": "display", "time_s": -0.01, "name": "Approach", "kind": "path", "colour": [0.05, 0.45, 1.0],
         "width_scale": 1.0, "closed": False, "points": ahead},
        {"t": "display", "time_s": 0.0, "name": "/World/Destinations/0", "kind": "ring", "colour": green,
         "width_scale": 2.0, "closed": True, "points": ring(8.0, 0.0)},
        {"t": "display", "time_s": 0.0, "name": "/World/Destinations/1", "kind": "ring", "colour": green,
         "width_scale": 2.0, "closed": True, "points": ring(9.0, 1.0)},
        {"t": "display", "time_s": None, "name": "Observe", "kind": "path", "colour": [0.2, 0.8, 0.8],
         "width_scale": 1.0, "closed": False, "points": ahead},  # untimed: counted, not used
        {"t": "note", "time_s": 0.5},  # a row this reader does not know: counted, not refused
        {"t": "clear", "time_s": 1.3, "name": "Approach"},
        {"t": "display", "time_s": 1.3, "name": "Transport", "kind": "path", "colour": [1.0, 0.65, 0.04],
         "width_scale": 1.0, "closed": False, "points": ahead[10:]},
    ])
    (tmp_path / "run" / "path_display.json").write_text(json.dumps({"compose_failures": 0, "clock_mismatches": 2}))
    rc, report = run_delta(capsys, "--run", tmp_path / "run", "--frames", tmp_path / "cap", "--per-frame")
    assert rc == 0, report
    src = report["sources"]
    assert src["paths"].endswith("run/path_display.jsonl") and src["paths_time_resolved"] is True
    assert src["path_display_header"]["schema"] == "path_display/1"
    assert src["path_display_header"]["pixel_lag_s"] == 0.2 and src["path_display_header"]["ignored_rows"] == 1
    assert src["path_display_header"]["untimed_rows"] == 1
    assert src["path_display_summary"] == {"compose_failures": 0, "clock_mismatches": 2}
    rings = ["/World/Destinations/0", "/World/Destinations/1"]
    first, second = report["per_frame"]
    assert sorted(first["paths"]) == sorted(["Approach", *rings])
    assert sorted(second["paths"]) == sorted(["Transport", *rings])
    assert first["shift"]["0"]["max_px"] is not None
    # the same record given by hand, under another name, is read by its header
    other = tmp_path / "display.jsonl"
    other.write_text((tmp_path / "run" / "path_display.jsonl").read_text())
    rc, again = run_delta(capsys, "--run", tmp_path / "run", "--frames", tmp_path / "cap", "--paths", other,
                          "--per-frame")
    assert rc == 0 and [sorted(r["paths"]) for r in again["per_frame"]] == [sorted(first["paths"]),
                                                                           sorted(second["paths"])]
    # a header that is not path_display's is refused
    (tmp_path / "run" / "path_display.jsonl").write_text(json.dumps({"t": "header", "schema": "other/1"}) + "\n")
    rc, err = run_delta(capsys, "--run", tmp_path / "run", "--frames", tmp_path / "cap")
    assert rc == 2 and "path_display/" in err


class Quiet:
    """A media server whose first connection sends two frames and then goes silent while staying open; the next
    connection sends two more."""

    def __init__(self):
        self.connections = 0

    async def handler(self, ws):
        self.connections += 1
        await ws.recv()
        ids = (1, 2) if self.connections == 1 else (3, 4)
        for fid in ids:
            item = camera_item(fid, picture(fid), 1.0, fid * 0.1, 1)
            jpeg = encode_jpeg(item["rgb"])
            await ws.send(wire.frame(dict(item["head"], jpeg_bytes=len(jpeg)), jpeg))
        await ws.wait_closed()


def test_capture_reconnects_a_silent_connection(tmp_path):
    quiet = Quiet()
    out = tmp_path / "cap"

    async def run():
        async with serve(quiet.handler, "127.0.0.1", 0, compression=None) as server:
            port = server.sockets[0].getsockname()[1]
            return await live_frames.capture(f"ws://127.0.0.1:{port}", out, max_frames=4, retry_s=0.5, seconds=15)

    result = asyncio.run(run())
    assert result["reason"] == "max_frames" and result["kept"] == 4
    assert result["connections"] == 2 and result["silent_reconnects"] == 1
    rows = read_index(out)
    assert [r["frame_id"] for r in rows if r["t"] == "frame"] == [1, 2, 3, 4]
    assert [r["t"] for r in rows if r["t"] != "frame"] == ["silent", "end"]


def test_capture_ends_unreachable_when_nothing_connects(tmp_path):
    import socket

    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]  # closed again: nothing listens there
    result = asyncio.run(live_frames.capture(f"ws://127.0.0.1:{port}", tmp_path / "cap", retry_s=0.3, seconds=10))
    assert result["reason"] == "unreachable" and result["connections"] == 0


def test_judge_refuses_when_a_display_colour_is_not_read(tmp_path, monkeypatch, capsys):
    root = tmp_path / "root"
    for rel in (*live_frames.COLOUR_SOURCES, "sim/isaac/path_overlay.py"):
        text = (ROOT / rel).read_text(encoding="utf-8")
        if rel in live_frames.COLOUR_SOURCES:
            text = text.replace('"Return"', "RETURN_LABEL")  # a refactor: the name is no longer a literal
        (root / rel).parent.mkdir(parents=True, exist_ok=True)
        (root / rel).write_text(text, encoding="utf-8")
    write_arm(tmp_path / "a", [np.full((40, 60, 3), 70, dtype=np.uint8)])
    monkeypatch.setattr(live_frames, "ROOT", root)
    assert "Return" not in live_frames.overlay_colours()
    rc = live_frames.main(["judge", str(tmp_path / "a")])
    err = capsys.readouterr().err
    assert rc == 2 and "['Return']" in err
    # --colour does not stand in for a colour the sources did not give
    rc = live_frames.main(["judge", str(tmp_path / "a"), "--colour", "Return=140,51,217"])
    assert rc == 2 and "Return" in capsys.readouterr().err
    monkeypatch.setattr(live_frames, "ROOT", ROOT)
    assert live_frames.main(["judge", str(tmp_path / "a")]) == 0


# --- fit: which lag puts the display lines on the rendered ones (S4 both diagnostic) --------------------------------

ORANGE = (255, 166, 10)  # Transport, a display colour the fit's rendered-line mask takes
MAGENTA = (255, 0, 255)  # path_overlay.CONTRAST_COLOUR, the diagnostic overlay


def render_both(cap: Path, run: Path, frames, line_pts, *, true_lag: float, overlay_lag: float = 0.0,
                width: int = 8, distractor: bool = False) -> None:
    """Each frame as the diagnostic 'both' shows it: the line rendered (``width`` px -- the 18 mm prim is about 8 px
    at 1 m -- in a display colour) where the camera was
    ``true_lag`` before the render time, and the 1 px magenta overlay drawn from the camera ``overlay_lag`` before
    it -- saved as JPEG q80, as the console sends them."""
    from PIL import ImageDraw

    from forklift_core.live.frame_judge import PoseTrack, camera_pose, densify_polyline, pixel_shift

    stamps, poses, _ = live_frames.load_poses(run)
    track = PoseTrack(stamps, poses)
    result = json.loads((run / "result.json").read_text())
    mount, _, _ = live_frames.load_mount(result, None)
    k, _ = live_frames.load_intrinsics(result, None)
    K = np.array([[k["fx"], 0, k["cx"]], [0, k["fy"], k["cy"]], [0, 0, 1.0]])
    pts = densify_polyline(np.asarray(line_pts, dtype=float), 0.02)
    (cap / "frames").mkdir()
    for fid, stamp, _compose in frames:
        cams = [camera_pose(track.at(stamp - lag), mount, 0.0) for lag in (true_lag, overlay_lag)]
        s = pixel_shift(pts, cams[0], cams[1], K)
        img = Image.new("RGB", (640, 480), (70, 90, 110))
        draw = ImageDraw.Draw(img)
        if distractor:  # a yellow board beside the line, as random_m's dock boards
            draw.rectangle([300, 150, 340, 330], fill=(230, 200, 60))
        draw.line([tuple(p) for p in s["uv_a"]], fill=ORANGE, width=width)
        draw.line([tuple(p) for p in s["uv_b"]], fill=MAGENTA, width=1)
        img.save(cap / "frames" / f"{fid}.jpg", quality=80)


def run_fit(capsys, *argv):
    rc = live_frames.main(["fit", *map(str, argv)])
    captured = capsys.readouterr()
    return rc, (json.loads(captured.out) if rc == 0 else captured.err)


@pytest.mark.parametrize("true_steps, width, distractor", [(11, 8, False), (0, 8, False), (16, 8, False),
                                                           (11, 5, True), (11, 12, False)])
def test_fit_recovers_the_lag_the_frames_were_rendered_with(tmp_path, capsys, true_steps, width, distractor):
    omega = 0.3  # turning in place: the lag moves every point across the picture
    stamps = [1.5 + 0.1 * i for i in range(12)]
    frames = [(i + 1, s, s + LAG) for i, s in enumerate(stamps)]
    make_run(tmp_path / "run", lambda t: (0.0, 0.0, omega * (t - 2.0)), end_s=4.0, lift_m=0.0, frames=frames)
    make_capture(tmp_path / "cap", [(f[0], f[1]) for f in frames])
    line = [[x, y, LINE_Z] for x, y in zip(np.linspace(1.5, 6.0, 30), np.linspace(-0.6, 0.6, 30), strict=True)]
    events = write_events(tmp_path / "paths.jsonl", [{"time_s": 0.0, "name": "Transport", "poses": line}])
    render_both(tmp_path / "cap", tmp_path / "run", frames, line, true_lag=true_steps / 120, width=width,
                distractor=distractor)
    rc, report = run_fit(capsys, "--run", tmp_path / "run", "--frames", tmp_path / "cap", "--paths", events)
    assert rc == 0, report
    by_lag = report["by_lag"]
    best = min(by_lag, key=lambda key: by_lag[key]["turning"]["median_px"])
    assert abs(int(best) - true_steps) <= 2  # JPEG and the 1 px overlay over the line leave a step or two
    cmp_ = report["comparison"]
    assert cmp_["judged_steps"] == [0, 11] and cmp_["frames"] == 12
    assert cmp_["better"] == ("0" if true_steps == 0 else "11")
    assert report["params"]["excluded_colours"] == ["Return", "contrast"]


def test_fit_counts_only_frames_with_enough_samples_and_line_pixels(tmp_path, capsys):
    stamps = [1.5, 1.6]
    frames = [(i + 1, s, s + LAG) for i, s in enumerate(stamps)]
    make_run(tmp_path / "run", lambda t: (0.0, 0.0, 0.0), end_s=3.0, lift_m=0.0, frames=frames)
    make_capture(tmp_path / "cap", [(f[0], f[1]) for f in frames])
    line = [[x, 0.0, LINE_Z] for x in np.linspace(1.5, 6.0, 30)]
    events = write_events(tmp_path / "paths.jsonl", [{"time_s": 0.0, "name": "Transport", "poses": line}])
    render_both(tmp_path / "cap", tmp_path / "run", frames[:1], line, true_lag=0.0)  # 8 px, the overlay on it
    Image.new("RGB", (640, 480), (70, 90, 110)).save(tmp_path / "cap" / "frames" / "2.jpg", quality=80)  # no line
    rc, report = run_fit(capsys, "--run", tmp_path / "run", "--frames", tmp_path / "cap", "--paths", events,
                         "--per-frame")
    assert rc == 0, report
    rows = {r["frame_id"]: r for r in report["per_frame"]}
    assert rows[1]["counted"] is True and rows[2]["counted"] is False and rows[2]["why"] == "few_line_pixels"
    assert report["counts"]["counted"] == 1
