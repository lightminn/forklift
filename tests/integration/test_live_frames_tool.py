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
