#!/usr/bin/env python3
"""Save the live console's camera frames as the page receives them, and count the overlay
colours in them (docs/plans/2026-11-14-path-display-perception-leak.md 4, S4).

In a live Isaac run the planned paths and destination rings are composited into the
console's camera stream (sim/isaac/run_transport.py ``live_camera()`` ->
sim/isaac/path_overlay.py ``compose``). S4 asks whether those lines are visible to the
operator, and how far they sit from where the pixels are.

``capture --url ws://HOST:PORT --out DIR [--seconds S] [--every N] [--max-frames M]``
    Connects to the media socket (/api/v1/media) the way console/web/js/main.js does:
    sends one ``media_subscribe`` and reads binary frames (uint32 big-endian header
    length, JSON header, body; console/wire.py ``frame``). Only ``camera`` frames are
    kept; the JPEG is cut at the header's ``jpeg_bytes`` as the page does. It is an
    observer: it never opens the control socket and sends nothing but the subscription
    (camera only), so it holds no request token and takes no lease. A frame resent on a
    resubscribe (same session epoch and frame id) is saved once. Each kept frame goes to
    DIR/frames/<frame_id>.jpg (``<frame_id>_e<epoch>.jpg`` when a later session reuses
    the id) with a row in DIR/index.jsonl; DIR/meta.json is written first and an
    ``{"t": "end"}`` row last, also on Ctrl-C. A close with 1001 (the runner ended) ends
    the capture; any other close reconnects after 1 s, as the page does, and so does a
    connection that delivers nothing for ``--retry-s`` (closed by the tool, a ``silent`` row).
    DIR must be empty or absent. Prints the end row; exits 0, 1 when it ended ``unreachable``
    (no connection could be opened for ``--retry-s``), 2 on a refused argument or DIR.

``judge DIR [--compare DIR_B] [--tol 40] [--min-component 50] [--colour NAME=R,G,B]``
    Decodes each indexed frame and counts, per overlay colour, the pixels within
    ``--tol`` per channel and the 4-connected components of at least
    ``--min-component`` pixels (forklift_core.live.frame_judge). Prints JSON. It decides
    nothing: the S4 registration sets the thresholds. Exits 2 when one of Approach,
    Transport, Return or destination was not read from its display call (REQUIRED_COLOURS).

``delta --run RUN_DIR --frames DIR [--paths EVENTS] [--lag-setting-s S]... [--true-lag-s L] [--sample-m M]``
    An offline point-sample estimate (report ``kind``) of how far the composited lines sit
    from the pixels. The overlay draws from the camera pose at the frame's render time t
    (stamp_sim_s) minus its pixel-lag setting; the pixels show the scene about 11 physics
    steps before t (run_transport --live-camera-pixel-lag-s). Per frame, each standing line
    (paths and destination rings) is densified to samples every M metres (default 0.02) so a
    segment whose ends are both off the picture still counts; the samples on the picture are
    projected with the camera at t - setting and at t - L, and the largest pixel distance is
    kept (frame_judge.pixel_shift_in_view). A frame with no sample on the picture is counted
    as ``no_visible`` and left out of the max, p95 and median over frames, which are split
    straight/turning by the yaw rate at t - L. The approximations are listed in the report
    (``approximations``): the camera is base_link's truth pose interpolated, times the mount
    and the lift, not the camera prim's pose history the overlay uses; lines are point
    samples. Read from the run: base_link's pose per physics step (slam_log base_pose_world,
    spooled), the camera mount (result.json perception_mount, lift from state.samples) and K
    (result.json live_video_camera.intrinsics); live_log.jsonl's camera lines give each
    frame's compose time, at which the standing lines are taken, joined on (session_epoch,
    frame_id): a run directory is one session epoch's run (its epoch from the live log's
    start line), frame ids restart with each epoch, and a capture may span several, so frames
    of another epoch are skipped and counted; when the run does not record its epoch and the
    capture holds several, ``--session-epoch`` is required. The lines come from
    RUN/path_display.jsonl (the runner's timed display record) unless ``--paths`` is given.
    Prints JSON; exits 2 when a piece is missing, naming it.

The overlay colours are not copied here. They are literals at the runner's and the
mission core's display calls (``show_path``, ``add_path_display``, ``add_path``,
``add_ring``), so they are read from that source with ``ast`` and converted with
path_overlay's own 0-1 -> 0-255 rule; path_overlay.py is loaded by file path (it imports
only NumPy at module level; Isaac is never imported). Its diagnostic CONTRAST_COLOUR is
included as ``contrast``.

Clocks in index.jsonl: ``recv_mono_ns`` is this machine's ``time.monotonic_ns()`` on
receipt; ``acquired_mono`` is the runner's monotonic clock (seconds), which is another
machine's clock when the capture runs elsewhere; ``stamp_sim_s`` is the frame's render
time on the simulation clock.
"""

from __future__ import annotations

import argparse
import ast
import asyncio
import contextlib
import importlib.util
import json
import math
import re
import signal
import socket
import struct
import sys
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import urlsplit, urlunsplit

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT / "src", ROOT):  # console/ lives at the root, forklift_core under src/
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import numpy as np  # noqa: E402

from forklift_core.live.frame_judge import arm_summary, frame_summary  # noqa: E402

TOOL_VERSION = "1"
CAMERA_HZ = 15.0  # what the page asks; the live server caps it (console/live/server.py CAPS_HZ)
RECONNECT_S = 1.0  # the page's delay after a close
RETRY_S = 10.0  # give up when nothing was received or connected for this long
POLL_S = 0.2
MAX_MESSAGE_BYTES = 16 * 1024 * 1024
RUNNER_ENDED = 1001  # LiveServer.stop closes every connection with it
COLOUR_SOURCES = ("src/forklift_core/mission/core.py", "sim/isaac/run_transport.py")
DISPLAY_CALLS = ("show_path", "add_path_display", "add_path", "add_ring")
RING_NAME = "destination"
REQUIRED_COLOURS = ("Approach", "Transport", "Return", RING_NAME)  # judge refuses to run without any of them
# sim/isaac/run_transport.py --live-camera-pixel-lag-s help: the pixels trail the render metadata by 11 physics steps
# (2026-10-17 marker runs) at 120 Hz; that option's own default adds a render of margin (0.2 s)
PIXEL_LAG_S = 11 / 120
TURN_RADPS = 0.05  # |yaw rate| at or above this is "turning" in the delta's split (a report split, no verdict)
RATE_WINDOW_S = 0.1  # the yaw rate's window, centred at t - true lag
COMPOSE_SLACK_S = 1.0  # a frame is read (composed) at or after its render, within this
SAMPLE_M = 0.02  # the delta's world spacing of samples along a displayed line


# ------------------------------------------------------------------ wire


def media_url(url: str) -> str:
    """``ws://HOST:PORT`` (or with the media path already) -> the media socket's URL."""
    from console.wire import MEDIA_PATH

    parts = urlsplit(url)
    if parts.scheme not in ("ws", "wss") or not parts.netloc:
        raise ValueError(f"--url must be ws://HOST:PORT, not {url!r}")
    if parts.path not in ("", "/", MEDIA_PATH):
        raise ValueError(f"--url names {parts.path!r}; only the media socket {MEDIA_PATH} is read")
    return urlunsplit((parts.scheme, parts.netloc, MEDIA_PATH, "", ""))


def parse_frame(data: bytes) -> tuple[dict, bytes]:
    """A media message -> (header, body), as console/web/js/media.js parseFrame."""
    if len(data) < 4:
        raise ValueError("media frame shorter than its length prefix")
    (n,) = struct.unpack(">I", data[:4])
    if 4 + n > len(data):
        raise ValueError("media frame header runs past the message")
    head = json.loads(data[4:4 + n].decode("utf-8"))
    if not isinstance(head, dict):
        raise ValueError("media frame header is not an object")
    return head, data[4 + n:]


# ------------------------------------------------------------------ capture


def _prepare(out: Path) -> None:
    if out.exists() and (not out.is_dir() or any(out.iterdir())):
        raise FileExistsError(f"{out} is not empty; capture writes into an empty directory")
    (out / "frames").mkdir(parents=True, exist_ok=True)


def _safe(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9_-]", "_", text) or "_"


class _Capture:
    """The capture's files and counts (one connection at a time)."""

    def __init__(self, out: Path, every: int, max_frames: int | None):
        self.out, self.every, self.max_frames = out, every, max_frames
        self.index = (out / "index.jsonl").open("a", encoding="utf-8")
        self.seen: set = set()
        self.files: set[str] = set()
        self.counts = {"frames": 0, "kept": 0, "duplicates": 0, "unparsed": 0,
                       "connections": 0, "reconnects": 0, "silent_reconnects": 0}

    def row(self, row: dict) -> None:
        self.index.write(json.dumps(row) + "\n")
        self.index.flush()

    def _file_name(self, frame_id, epoch, n: int) -> str:
        for name in (f"{_safe(str(frame_id))}.jpg", f"{_safe(str(frame_id))}_e{_safe(str(epoch))}.jpg",
                     f"{_safe(str(frame_id))}_e{_safe(str(epoch))}_n{n}.jpg"):
            if name not in self.files:
                return name
        raise RuntimeError("no free file name")  # unreachable: n is unique

    def message(self, raw, recv_mono_ns: int) -> bool:
        """Handle one received message; True once ``max_frames`` frames are kept."""
        if not isinstance(raw, bytes):
            return False  # the media socket sends no text; the page ignores it too
        try:
            head, body = parse_frame(raw)
        except (ValueError, UnicodeDecodeError):
            self.counts["unparsed"] += 1
            return False
        if head.get("t") != "camera":
            return False
        epoch, frame_id = head.get("session_epoch"), head.get("frame_id")
        key = (json.dumps(epoch), json.dumps(frame_id))
        if key in self.seen:  # resent on a resubscribe: the same frame
            self.counts["duplicates"] += 1
            return False
        self.seen.add(key)
        self.counts["frames"] += 1
        n = self.counts["frames"]
        if (n - 1) % self.every:
            return False
        size = head.get("jpeg_bytes")
        jpeg = body[:size] if isinstance(size, int) and not isinstance(size, bool) else body
        name = self._file_name(frame_id, epoch, n)
        (self.out / "frames" / name).write_bytes(jpeg)  # the file before the row that names it
        self.files.add(name)
        capture = head.get("capture") if isinstance(head.get("capture"), dict) else {}
        stamp_ns = capture.get("stamp_ns")
        acquired_ns = head.get("acquired_mono_ns")
        self.row({
            "t": "frame", "n": n, "frame_id": frame_id, "session_epoch": epoch,
            "stamp_sim_s": stamp_ns / 1e9 if capture.get("domain") == "sim" and isinstance(stamp_ns, int)
            else None,
            "stamp_ns": stamp_ns, "capture_domain": capture.get("domain"),
            "acquired_mono": acquired_ns / 1e9 if isinstance(acquired_ns, int) else None,
            "acquired_mono_ns": acquired_ns, "recv_mono_ns": recv_mono_ns,
            "width": head.get("width"), "height": head.get("height"),
            "file": f"frames/{name}", "bytes": len(jpeg),
        })
        self.counts["kept"] += 1
        return self.max_frames is not None and self.counts["kept"] >= self.max_frames


async def _wait(stop: asyncio.Event, timeout_s: float) -> None:
    with contextlib.suppress(asyncio.TimeoutError):
        await asyncio.wait_for(stop.wait(), max(0.0, timeout_s))


async def _loop(cap: _Capture, url: str, subscribe: dict, stop: asyncio.Event,
                deadline: float | None, retry_s: float) -> str:
    from websockets.asyncio.client import connect
    from websockets.exceptions import ConnectionClosed, InvalidHandshake

    def remaining() -> float:
        return float("inf") if deadline is None else deadline - time.monotonic()

    last_up = time.monotonic()  # the last time a connection was open (connected, received or closed)
    while True:
        if stop.is_set():
            return "interrupted"
        if remaining() <= 0:
            return "seconds"
        try:
            async with connect(url, compression=None, max_size=MAX_MESSAGE_BYTES,
                               open_timeout=min(5.0, retry_s), close_timeout=1.0) as ws:
                cap.counts["connections"] += 1
                cap.counts["reconnects"] = cap.counts["connections"] - 1
                last_up = last_msg = time.monotonic()
                await ws.send(json.dumps(subscribe))
                while True:
                    if stop.is_set():
                        return "interrupted"
                    left = remaining()
                    if left <= 0:
                        return "seconds"
                    try:
                        raw = await asyncio.wait_for(ws.recv(), min(POLL_S, left))
                    except asyncio.TimeoutError:
                        # recv() is safe to cancel: nothing is lost. An open socket that stays silent is
                        # dropped and opened again (the page reconnects too); it is not the run's end.
                        if time.monotonic() - last_msg > retry_s:
                            cap.counts["silent_reconnects"] += 1
                            cap.row({"t": "silent", "after_s": time.monotonic() - last_msg,
                                     "recv_mono_ns": time.monotonic_ns()})
                            break
                        continue
                    last_up = last_msg = time.monotonic()
                    if cap.message(raw, time.monotonic_ns()):
                        return "max_frames"
            last_up = time.monotonic()  # closed by us after the silence: connecting again is what is timed
        except ConnectionClosed as exc:
            code = exc.rcvd.code if exc.rcvd is not None else None
            cap.row({"t": "closed", "code": code, "reason": exc.rcvd.reason if exc.rcvd is not None else None,
                     "recv_mono_ns": time.monotonic_ns()})
            if code == RUNNER_ENDED:
                return "runner_ended"
            last_up = time.monotonic()
        except (OSError, asyncio.TimeoutError, InvalidHandshake) as exc:
            print(f"live_frames: cannot connect to {url}: {exc!r}", file=sys.stderr, flush=True)
        if time.monotonic() - last_up > retry_s:
            return "unreachable"
        await _wait(stop, min(RECONNECT_S, remaining()))


async def capture(url: str, out, *, seconds: float | None = None, every: int = 1,
                  max_frames: int | None = None, hz: float = CAMERA_HZ,
                  stop: asyncio.Event | None = None, retry_s: float = RETRY_S) -> dict:
    """Capture camera frames from the live console's media socket into ``out``.

    Ends on ``max_frames`` kept frames, after ``seconds``, when ``stop`` is set
    (``interrupted``), when the server closes with 1001 (``runner_ended``) or when no
    connection could be opened for ``retry_s`` (``unreachable``). An open connection that
    delivers nothing for ``retry_s`` is closed and opened again (a ``silent`` row, counted in
    ``silent_reconnects``), as the page does; it does not end the capture. Returns the end row
    written last to index.jsonl. Raises FileExistsError, before writing anything, when
    ``out`` is not empty.
    """
    if every < 1:
        raise ValueError("--every must be >= 1")
    if max_frames is not None and max_frames < 1:
        raise ValueError("--max-frames must be >= 1")
    if seconds is not None and seconds <= 0:
        raise ValueError("--seconds must be > 0")
    target = media_url(url)
    out = Path(out)
    _prepare(out)
    stop = stop if stop is not None else asyncio.Event()
    started = time.monotonic()
    subscribe = {"t": "media_subscribe", "client_id": f"live_frames-{uuid.uuid4().hex[:12]}",
                 "camera_rgb": {"hz": hz}}
    meta = {
        "tool": "tools/live_frames.py capture", "version": TOOL_VERSION, "url": url,
        "media_url": target, "started_utc": datetime.now(timezone.utc).isoformat(),
        "started_mono_ns": time.monotonic_ns(), "host": socket.gethostname(),
        "options": {"seconds": seconds, "every": every, "max_frames": max_frames, "hz": hz,
                    "retry_s": retry_s},
        "subscribe": subscribe,
        "clocks": {"recv_mono_ns": "this host's time.monotonic_ns() on receipt",
                   "acquired_mono": "the runner's monotonic clock, s (may be another host)",
                   "stamp_sim_s": "render time on the simulation clock, s"},
    }
    (out / "meta.json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
    cap = _Capture(out, every, max_frames)
    reason = "error"
    try:
        reason = await _loop(cap, target, subscribe, stop,
                             None if seconds is None else started + seconds, retry_s)
    except asyncio.CancelledError:
        reason = "cancelled"
        raise
    except Exception as exc:
        reason = f"error: {exc!r}"
        raise
    finally:
        end = {"t": "end", **cap.counts, "reason": reason, "recv_mono_ns": time.monotonic_ns(),
               "elapsed_s": time.monotonic() - started}
        cap.row(end)
        cap.index.close()
    return end


# ------------------------------------------------------------------ colours


def _load_path_overlay():
    """sim/isaac/path_overlay.py by file path under a private name (NumPy only at import,
    never Isaac; sim/isaac stays off sys.path). Registered before it runs: its dataclass
    looks its own module up."""
    name = "_live_frames_path_overlay"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, ROOT / "sim/isaac/path_overlay.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
    except BaseException:
        sys.modules.pop(name, None)
        raise
    return module


def _number(node) -> bool:
    return (isinstance(node, ast.Constant) and isinstance(node.value, (int, float))
            and not isinstance(node.value, bool))


def _colour_literals(node) -> list[tuple]:
    if isinstance(node, ast.Tuple) and len(node.elts) == 3 and all(_number(e) for e in node.elts):
        return [tuple(float(e.value) for e in node.elts)]
    if isinstance(node, ast.IfExp):
        body, orelse = _colour_literals(node.body), _colour_literals(node.orelse)
        if len(body) == 1 and len(orelse) == 1:
            return body + orelse
    return []


def _label_literals(node) -> list[str]:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return [node.value]
    if isinstance(node, ast.IfExp):
        body, orelse = _label_literals(node.body), _label_literals(node.orelse)
        if len(body) == 1 and len(orelse) == 1:
            return body + orelse
    return []


def overlay_colours(root: Path | None = None) -> dict[str, dict]:
    """The overlay's colours by display name: ``{name: {"rgb": [r, g, b], "sources":
    ["file:line", ...]}}``, 0-255 as path_overlay draws them.

    Read from the literal colours at the display calls in COLOUR_SOURCES (a call whose
    colour or name is a variable is skipped; its literal origin is another call). The
    destination ring is ``destination``; path_overlay's CONTRAST_COLOUR is ``contrast``.
    A name drawn in two colours gets one entry per colour, ``name@r,g,b``. ``root``: the
    checkout the sources are read from (default ROOT).
    """
    root = ROOT if root is None else Path(root)
    po = _load_path_overlay()
    found: dict[str, dict[tuple, list[str]]] = {}
    for rel in COLOUR_SOURCES:
        tree = ast.parse((root / rel).read_text(encoding="utf-8"), filename=rel)
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not node.args:
                continue
            func = node.func
            name = func.attr if isinstance(func, ast.Attribute) else getattr(func, "id", None)
            if name not in DISPLAY_CALLS:
                continue
            colours = _colour_literals(node.args[-1])
            if not colours:
                continue
            if name == "add_ring":
                labels = [RING_NAME]
            else:
                labels = next((lab for lab in map(_label_literals, node.args[:-1]) if lab), [])
            if not labels:
                continue
            if len(labels) == 1 < len(colours):
                labels = labels * len(colours)
            if len(colours) == 1 < len(labels):
                colours = colours * len(labels)
            for label, colour in zip(labels, colours, strict=True):
                rgb = tuple(po._colour255(colour))  # the overlay's own conversion
                found.setdefault(label, {}).setdefault(rgb, []).append(f"{rel}:{node.lineno}")
    found.setdefault("contrast", {}).setdefault(tuple(int(v) for v in po.CONTRAST_COLOUR), []).append(
        "sim/isaac/path_overlay.py:CONTRAST_COLOUR")
    out = {}
    for label in sorted(found):
        variants = found[label]
        for rgb, sources in variants.items():
            key = label if len(variants) == 1 else f"{label}@{','.join(map(str, rgb))}"
            out[key] = {"rgb": list(rgb), "sources": sorted(set(sources))}
    return out


def check_colours(colours: dict[str, dict]) -> None:
    """Refuse a colour table without one of REQUIRED_COLOURS (as ``name`` or ``name@r,g,b``):
    a refactor of the display calls must not drop a colour from the judge unnoticed."""
    missing = [n for n in REQUIRED_COLOURS if not any(k == n or k.startswith(n + "@") for k in colours)]
    if missing:
        raise ValueError(f"overlay colours {missing} were not found at the display calls ({', '.join(DISPLAY_CALLS)}) "
                         f"in {', '.join(COLOUR_SOURCES)}: a call site changed so its literal colour or name is no "
                         "longer read; fix overlay_colours to read it")


def _parse_colour(text: str) -> tuple[str, list[int]]:
    name, sep, values = text.partition("=")
    parts = values.split(",")
    if not sep or not name or len(parts) != 3:
        raise argparse.ArgumentTypeError(f"--colour is NAME=R,G,B, not {text!r}")
    try:
        rgb = [int(v) for v in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"--colour is NAME=R,G,B, not {text!r}") from exc
    if not all(0 <= v <= 255 for v in rgb):
        raise argparse.ArgumentTypeError(f"--colour values are 0-255, not {text!r}")
    return name, rgb


# ------------------------------------------------------------------ judge


def _frame_rows(directory: Path) -> list[dict]:
    rows = []
    with (directory / "index.jsonl").open(encoding="utf-8") as f:
        for line in f:
            if line.strip():
                row = json.loads(line)
                if row.get("t") == "frame":
                    rows.append(row)
    return rows


def _decode(path: Path):
    from PIL import Image

    try:
        with Image.open(path) as im:
            return np.asarray(im.convert("RGB"))
    except (OSError, ValueError, SyntaxError):  # PIL's UnidentifiedImageError is an OSError
        return None


def judge_arm(directory: Path, colours: dict[str, list[int]], tol: int, min_component: int,
              per_frame: bool = False) -> dict:
    """One capture directory's counts (frame_judge.arm_summary) with its ``dir``."""
    summaries, details = [], []
    for row in _frame_rows(directory):
        rgb = _decode(directory / row["file"])
        s = None if rgb is None else frame_summary(rgb, colours, tol, min_component)
        summaries.append(s)
        details.append({"frame_id": row.get("frame_id"), "file": row["file"], "colours": s})
    arm = {"dir": str(directory), **arm_summary(summaries, list(colours))}
    if per_frame:
        arm["per_frame"] = details
    return arm


def judge(directory: Path, compare: Path | None = None, *, tol: int = 40, min_component: int = 50,
          extra: list[tuple[str, list[int]]] = (), per_frame: bool = False) -> dict:
    """The judge report: colours, each arm's counts, and the same per colour side by side."""
    if tol < 0 or min_component < 1:
        raise ValueError("--tol must be >= 0 and --min-component >= 1")
    colours = overlay_colours()
    check_colours(colours)  # on what the sources gave, before any --colour
    for name, rgb in extra:
        colours[name] = {"rgb": list(rgb), "sources": ["--colour"]}
    plain = {name: c["rgb"] for name, c in colours.items()}
    arms = {"arm": judge_arm(Path(directory), plain, tol, min_component, per_frame)}
    if compare is not None:
        arms["compare"] = judge_arm(Path(compare), plain, tol, min_component, per_frame)
    return {
        "tool": "tools/live_frames.py judge", "version": TOOL_VERSION, "tol": tol,
        "min_component": min_component, "connectivity": 4, "colours": colours, "arms": arms,
        "by_colour": {name: {label: arm["colours"][name] for label, arm in arms.items()}
                      for name in colours},
    }


# ------------------------------------------------------------------ delta


class RecordMissing(ValueError):
    """A piece the delta needs is not in the run's records (nor given by an option)."""


def _read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _result(run: Path) -> dict | None:
    path = run / "result.json"
    return _read_json(path) if path.exists() else None


def load_poses(run: Path, allow_incomplete: bool = False) -> tuple[np.ndarray, np.ndarray, str]:
    """base_link's world pose per physics step: slam_log ``joint_stamps_s`` and
    ``base_pose_world`` (x, y, z, qw, qx, qy, qz), from a live run's spool (meta.json
    ``spool``) or a batch run's slam_log.npz. The runner logs them only with --record-slam."""
    from forklift_core.live.spool import load_slam_log

    try:
        log = load_slam_log(run, allow_incomplete=allow_incomplete)
    except FileNotFoundError as exc:
        raise RecordMissing(
            f"{run}: no base pose track ({exc}); the runner records base_pose_world per physics step only with "
            "--record-slam (sim/isaac/run_transport.py, slam_log['base_pose_world'])") from exc
    if "joint_stamps_s" not in log or "base_pose_world" not in log:
        raise RecordMissing(f"{run}: the SLAM log has no joint_stamps_s/base_pose_world")
    stamps = np.asarray(log["joint_stamps_s"], dtype=float).reshape(-1)
    poses = np.asarray(log["base_pose_world"], dtype=float).reshape(-1, 7)
    source = "slam_log.npz" if (run / "slam_log.npz").exists() else "spool slam_log.joint"
    return stamps, poses, f"{source}: joint_stamps_s, base_pose_world"


def _transform(translation, rotation) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    m[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return m


def load_mount(result: dict | None, mount_json: Path | None) -> tuple[np.ndarray, bool, str]:
    """base_link <- optical (ROS axes) at lift 0, and whether the camera rides the lift.

    The live video camera is placed with the perception mount under the same parent
    (run_transport, LiveVideoCamera), recorded as result.json ``perception_mount``: on
    ``.../fork_carriage`` it rises with the lift, on ``.../base_link`` (legacy) it does not.
    ``mount_json``: ``{"translation_m": [3], "rotation": [[3x3]], "rides_lift": bool}``."""
    if mount_json is not None:
        m = _read_json(mount_json)
        return _transform(m["translation_m"], m["rotation"]), bool(m["rides_lift"]), f"--mount-json {mount_json}"
    m = (result or {}).get("perception_mount")
    if not m:
        raise RecordMissing("no camera mount: result.json has no perception_mount (a run without --use-perception "
                            "has no live video camera, or the run has not written result.json yet); give --mount-json")
    parent = str(m.get("parent", ""))
    if not parent.endswith(("/fork_carriage", "/base_link")):
        raise RecordMissing(f"perception_mount parent {parent!r} is neither the carriage nor base_link; "
                            "give --mount-json")
    return (_transform(m["translation_m"], m["rotation"]), parent.endswith("/fork_carriage"),
            "result.json perception_mount")


def load_intrinsics(result: dict | None, intrinsics_json: Path | None) -> tuple[dict, str]:
    """The live video camera's K and resolution as the overlay reads them
    (Camera.get_intrinsics_matrix, the SDK's raw K): result.json
    ``live_video_camera.intrinsics``. ``intrinsics_json``: ``{fx, fy, cx, cy, width, height}``."""
    if intrinsics_json is not None:
        k, source = _read_json(intrinsics_json), f"--intrinsics-json {intrinsics_json}"
    else:
        k = ((result or {}).get("live_video_camera") or {}).get("intrinsics")
        source = "result.json live_video_camera.intrinsics"
        if not k:
            raise RecordMissing("no live video camera intrinsics: result.json has no live_video_camera.intrinsics "
                                "(not a --live --use-perception run, or no result.json yet); give --intrinsics-json")
    try:
        out = {n: float(k[n]) for n in ("fx", "fy", "cx", "cy")} | {n: int(k[n]) for n in ("width", "height")}
    except (KeyError, TypeError, ValueError) as exc:
        raise RecordMissing(f"{source}: needs fx, fy, cx, cy, width, height ({exc!r})") from exc
    return out, source


def load_lift(run: Path, result: dict | None, allow_incomplete: bool = False):
    """The lift joint over time: ``state.samples`` rows (time_s, lift_m; every 12 physics
    steps), from the spool of a live run or result.json ``samples``. None without one."""
    from forklift_core.live.spool import load_list

    rows, source = None, None
    if (run / "spool" / "state.samples").is_dir():
        rows, source = load_list(run / "spool", "state.samples", allow_incomplete=allow_incomplete), \
            "spool state.samples"
    elif (result or {}).get("samples"):
        rows, source = result["samples"], "result.json samples"
    rows = [r for r in rows or [] if r.get("time_s") is not None and r.get("lift_m") is not None]
    if not rows:
        return None
    rows.sort(key=lambda r: float(r["time_s"]))
    return (np.array([float(r["time_s"]) for r in rows]), np.array([float(r["lift_m"]) for r in rows]),
            f"{source}: time_s, lift_m")


PATH_DISPLAY_FILE = "path_display.jsonl"
PATH_DISPLAY_SCHEMA = "path_display/"


def parse_path_display(lines: list[str], where: str = PATH_DISPLAY_FILE) -> tuple[list[dict], dict]:
    """The runner's timed display record (``RUN/path_display.jsonl``, written by sim/isaac/path_overlay.py since
    PR #513; this is the one place that reads it) -> (raw events, header).

    Line 1: ``{"t": "header", "schema": "path_display/1", "mode", "pixel_lag_s", "time_axis",
    "line_width_px_at_1280"}``. Then ``{"t": "display", "time_s", "name", "kind": "path"|"ring", "colour",
    "width_scale", "closed", "points": [[x, y, z], ...]}`` -- the points compose draws (a path thinned to 200 or
    fewer at z 0.012, a ring a closed 96-point circle at z 0.006). A path's name is its call's label (Approach,
    Transport, Return, Observe); a ring's is its prim path (``/World/Destination``, ``/World/Destinations/<slot>``),
    so every line has its own name and a repeated name replaces that line. ``time_s`` is on the frames'
    ``stamp_sim_s`` axis and may be null when the runner never fixed its origin: such rows are counted in the
    header's ``untimed_rows`` and not used. A ``clear`` row (none written today) removes its name. Rows of another
    ``t`` are counted in ``ignored_rows``, not refused."""
    rows = [json.loads(line) for line in lines if line.strip()]
    if not rows or rows[0].get("t") != "header" or not str(rows[0].get("schema", "")).startswith(
            PATH_DISPLAY_SCHEMA):
        raise ValueError(f"{where}: the first line is not a {PATH_DISPLAY_SCHEMA}* header")
    header = dict(rows[0])
    events, ignored, untimed = [], 0, 0
    for row in rows[1:]:
        kind = row.get("t")
        if kind not in ("display", "clear"):
            ignored += 1
            continue
        if row.get("time_s") is None:
            untimed += 1
            continue
        event = {"time_s": float(row["time_s"]), "name": str(row["name"])}
        if kind == "clear":
            event["clear"] = True
        else:
            ring = row.get("kind") == "ring"
            event.update(points=row["points"], closed=bool(row.get("closed", ring)), colour=row.get("colour"),
                         width_scale=row.get("width_scale"), ring=ring)
        events.append(event)
    header["ignored_rows"] = ignored
    header["untimed_rows"] = untimed
    return events, header


def path_display_summary(run: Path) -> dict | None:
    """``RUN/path_display.json``: the runner's failure and clock-mismatch counts for the overlay, passed through."""
    f = run / "path_display.json"
    return json.loads(f.read_text(encoding="utf-8")) if f.is_file() else None


def _event_points(event: dict, po) -> np.ndarray:
    if event.get("points") is not None:
        pts = np.asarray(event["points"], dtype=float)
    else:
        pts = np.asarray([p[:2] for p in event["poses"]], dtype=float)
    pts = pts.reshape(len(pts), -1)
    if pts.shape[1] == 2:
        pts = np.column_stack([pts, np.full(len(pts), po.LINE_Z_M)])  # as path_overlay.add_path
    if pts.shape[1] != 3:
        raise ValueError(f"path {event.get('name')!r}: points must be x, y[, z]")
    return po.thin(pts)


def _is_path_display(path: Path) -> bool:
    if path.name == PATH_DISPLAY_FILE:
        return True
    with path.open(encoding="utf-8") as f:
        first = next((line for line in f if line.strip()), "")
    try:
        head = json.loads(first)
    except ValueError:
        return False
    return isinstance(head, dict) and str(head.get("schema", "")).startswith(PATH_DISPLAY_SCHEMA)


def load_path_events(path: Path) -> tuple[list[dict], bool, str, dict | None]:
    """The lines the overlay showed, as display events ``{"time_s", "name", "points"}`` plus
    ``key``, ``closed`` and ``clear`` where given (sim seconds since the run's start, as the
    frames' stamp_sim_s) -> (events, time_resolved, source, path_display header or None).

    ``path``: the runner's ``path_display.jsonl`` (parse_path_display; paths and destination
    rings, with clears); a JSONL file or a directory of them (a spooled list) or a JSON list
    of events ``{time_s, name, poses|points}`` -- time-resolved; or a JSON object ``{name:
    {"poses": [...]} | [...]}`` (paths.json, path_record) -- static: every path stands from the
    start (``time_resolved`` False). Points get path_overlay's line height and thinning, as
    ``add_path`` gives them."""
    po = _load_path_overlay()
    header = None
    if path.is_file() and path.suffix == ".jsonl" and _is_path_display(path):
        raw, header = parse_path_display(path.read_text(encoding="utf-8").splitlines(), str(path))
        resolved = True
    elif path.is_dir():
        lines = [line for f in sorted(path.glob("*.jsonl")) for line in f.read_text().splitlines()]
        raw, resolved = [json.loads(line) for line in lines if line.strip()], True
    elif path.suffix == ".jsonl":
        raw = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
        resolved = True
    else:
        data = _read_json(path)
        if isinstance(data, dict):
            raw = [{"time_s": -math.inf, "name": name, **(v if isinstance(v, dict) else {"poses": v})}
                   for name, v in data.items() if (isinstance(v, dict) and v.get("poses")) or isinstance(v, list)]
            resolved = False
        else:
            raw, resolved = list(data), True
    events = []
    for e in raw:
        if e.get("clear"):
            events.append({"time_s": float(e["time_s"]), "name": str(e["name"]), "clear": True,
                           **({"key": str(e["key"])} if e.get("key") is not None else {})})
            continue
        if "time_s" not in e or "name" not in e or (e.get("points") is None and e.get("poses") is None):
            raise ValueError(f"{path}: a path event needs time_s, name and points or poses: {sorted(e)}")
        event = {"time_s": float(e["time_s"]), "name": str(e["name"]), "points": _event_points(e, po),
                 "closed": bool(e.get("closed", False))}
        if e.get("key") is not None:
            event["key"] = str(e["key"])
        events.append(event)
    if not any(not e.get("clear") for e in events):
        raise RecordMissing(f"{path}: no path in it")
    return events, resolved, str(path), header


def read_run_log(run: Path, allow_incomplete: bool = False) -> list[dict] | None:
    """The run's live_log.jsonl entries (rolled parts included); None without a live log."""
    from forklift_core.live.spool import read_live_log

    if not (run / "live_log.jsonl").exists():
        return None
    return read_live_log(run, allow_incomplete=allow_incomplete)


def run_session_epoch(entries: list[dict] | None, result: dict | None) -> tuple[int | None, str | None]:
    """The session epoch of a run directory (one runner process, one epoch; the frames' header
    ``session_epoch``): the live log's ``{"event": "start", "session_epoch"}`` line
    (live_runtime.LiveRuntime.start), else result.json ``reloc_console.session_epoch`` (a
    relocalizing run's early console). Not the folder's ``epoch_N``: N counts the supervisor's
    restarts, the session epoch is a random 48-bit number. (None, None) when neither is there."""
    found = {e["session_epoch"] for e in entries or () if e.get("event") == "start"
             and e.get("session_epoch") is not None}
    if len(found) > 1:
        raise RecordMissing(f"the live log starts several session epochs {sorted(found)}; give --session-epoch")
    if found:
        return found.pop(), "live_log.jsonl start session_epoch"
    early = ((result or {}).get("reloc_console") or {}).get("session_epoch")
    if early is not None:
        return early, "result.json reloc_console.session_epoch"
    return None, None


def compose_times(entries: list[dict] | None, epoch) -> dict | None:
    """(session_epoch, frame_id) -> (render_time, sim_t) from the live log's ``camera`` lines:
    ``sim_t`` is the end of the physics step whose camera read composed the frame -- the
    overlay draws the lines standing then. A run's log is one epoch's (``epoch``), and frame
    ids restart with each epoch, so the epoch is part of the key. None without a live log."""
    if entries is None:
        return None
    out = {}
    for e in entries:
        if "camera" in e and e.get("sim_t") is not None:
            out[(epoch, e["camera"])] = (e.get("render_time"), float(e["sim_t"]))
    return out


LIFT_HOLD_S = 0.1  # state.samples come every 12 physics steps: past either end, the end value holds this long


def _lift_at(lift, t: float, interpolate_value) -> float | None:
    stamps, values = lift[0], lift[1]
    got = interpolate_value(stamps, values, t)
    if got is not None:
        return got
    if stamps[0] - LIFT_HOLD_S <= t < stamps[0]:
        return float(values[0])
    if stamps[-1] < t <= stamps[-1] + LIFT_HOLD_S:
        return float(values[-1])
    return None


def _lag_key(lag: float) -> str:
    return f"{lag:.6g}"


DELTA_KIND = "offline point-sample estimate"
DELTA_APPROXIMATIONS = {
    "pose": "the camera at a time is base_link's truth pose (slam_log base_pose_world, interpolated: linear position, "
            "spherical rotation) times the perception mount, raised by the lift (state.samples lift_m); the overlay "
            "itself reads the camera prim's own pose history (path_overlay register_moving/note_moving), which this "
            "does not see",
    "lines": "each displayed polyline (and destination ring, closed) is densified at sample_m world spacing and the "
             "samples are projected as points; a sample counts when it is on the picture in either pose; line "
             "width, anti-aliasing and the overlay's own segment clipping are not modelled",
    "pixels": "the pixels are taken to show the scene true_lag_s before the frame's render time (stamp_sim_s); that "
              "lag is an assumption, not read from the frame",
}


def _choose_epoch(session_epoch, run_epoch, run_src, capture_epochs: list) -> tuple:
    """The epoch the delta reads: --session-epoch (which must be the run's when the run
    records one), else the run's, else the capture's only epoch; refused when the capture
    holds several and nothing says which is the run's."""
    if session_epoch is not None:
        if run_epoch is not None and session_epoch != run_epoch:
            raise RecordMissing(f"--session-epoch {session_epoch} is not this run's session epoch {run_epoch} "
                                f"({run_src}); a run directory is one epoch's run")
        return session_epoch, "--session-epoch"
    if run_epoch is not None:
        return run_epoch, run_src
    if len(capture_epochs) <= 1:
        return (capture_epochs[0] if capture_epochs else None,
                "the capture's only session epoch (the run records none)")
    raise RecordMissing(f"the capture holds session epochs {capture_epochs} and the run's own epoch is not recorded "
                        "(no live_log.jsonl start line with session_epoch, no result.json reloc_console); frame ids "
                        "restart with each epoch, so give --session-epoch")


def delta(run, frames_dir, *, lag_settings=(0.0, PIXEL_LAG_S), true_lag_s: float = PIXEL_LAG_S,
          paths: Path | None = None, mount_json: Path | None = None, intrinsics_json: Path | None = None,
          session_epoch=None, turn_radps: float = TURN_RADPS, rate_window_s: float = RATE_WINDOW_S,
          sample_m: float = SAMPLE_M, allow_incomplete: bool = False, per_frame: bool = False) -> dict:
    """An offline point-sample estimate of the overlay misalignment: per captured frame of the
    run's session epoch (render time t = stamp_sim_s), the pixel shift of the standing lines'
    samples between the camera at ``t - setting`` (where the overlay draws them) and at
    ``t - true_lag_s`` (what the pixels show), for each lag setting; summarised over the
    frames with a sample on the picture, and split straight / turning by the yaw rate at
    ``t - true_lag_s``. The approximations are listed in the report (DELTA_APPROXIMATIONS).
    Raises RecordMissing for a piece neither recorded nor given."""
    from forklift_core.live.frame_judge import (
        PoseTrack,
        camera_pose,
        densify_polyline,
        displayed_at,
        interpolate_value,
        pixel_shift_in_view,
        shift_summary,
    )

    run, frames_dir = Path(run), Path(frames_dir)
    lag_settings = [float(v) for v in lag_settings]
    if not lag_settings or any(not (math.isfinite(v) and v >= 0) for v in [*lag_settings, true_lag_s]):
        raise ValueError("lags must be finite and >= 0")
    if not (turn_radps >= 0 and rate_window_s > 0):
        raise ValueError("--turn-threshold-radps must be >= 0 and --rate-window-s > 0")
    if not (math.isfinite(sample_m) and sample_m > 0):
        raise ValueError("--sample-m must be finite and > 0")
    result = _result(run)
    mount, rides_lift, mount_src = load_mount(result, mount_json)
    k, k_src = load_intrinsics(result, intrinsics_json)
    if paths is None:
        if not (run / PATH_DISPLAY_FILE).exists():
            raise RecordMissing(
                f"the run has no {PATH_DISPLAY_FILE} (the runner's timed record of the display calls, PR #513), so "
                "nothing says which lines the overlay showed when: path_overlay.add_path keeps only the latest line "
                "per name in memory; give --paths (display events, JSONL or a directory of them, or a static "
                "paths.json)")
        paths = run / PATH_DISPLAY_FILE
    events, resolved, paths_src, display_header = load_path_events(Path(paths))
    for e in events:
        if not e.get("clear"):
            e["samples"] = densify_polyline(e["points"], sample_m, closed=e.get("closed", False))
    stamps, poses, pose_src = load_poses(run, allow_incomplete)
    track = PoseTrack(stamps, poses)  # checked once: a live session's track is long
    lift = load_lift(run, result, allow_incomplete) if rides_lift else None
    if rides_lift and lift is None:
        raise RecordMissing("the camera rides the lift but the run has no lift record (state.samples lift_m)")
    entries = read_run_log(run, allow_incomplete)
    run_epoch, run_epoch_src = run_session_epoch(entries, result)
    frame_rows = _frame_rows(frames_dir)
    capture_epochs = sorted({r.get("session_epoch") for r in frame_rows}, key=json.dumps)
    epoch, epoch_src = _choose_epoch(session_epoch, run_epoch, run_epoch_src, capture_epochs)
    composed = compose_times(entries, epoch)

    skipped = {k_: 0 for k_ in ("other_epoch", "no_stamp", "no_compose_entry", "compose_before_render",
                                "outside_poses", "no_path")}
    rows = []
    for row in frame_rows:
        if row.get("session_epoch") != epoch:
            skipped["other_epoch"] += 1  # another session's frame ids are another numbering
            continue
        t = row.get("stamp_sim_s")
        if t is None:
            skipped["no_stamp"] += 1
            continue
        compose_t = float(t)
        if composed is not None:
            entry = composed.get((row.get("session_epoch"), row.get("frame_id")))
            if entry is None:
                skipped["no_compose_entry"] += 1
                continue
            if not 0.0 <= entry[1] - float(t) <= COMPOSE_SLACK_S:  # a check of the join, not a second key
                skipped["compose_before_render"] += 1
                continue
            compose_t = entry[1]
        cams, missing = {}, False
        for lag in {*lag_settings, true_lag_s}:
            base = track.at(float(t) - lag)
            lift_m = 0.0 if lift is None else _lift_at(lift, float(t) - lag, interpolate_value)
            if base is None or lift_m is None:
                missing = True
                break
            cams[lag] = camera_pose(base, mount, lift_m)
        if missing:
            skipped["outside_poses"] += 1
            continue
        shown = displayed_at(events, compose_t)
        if not shown:
            skipped["no_path"] += 1
            continue
        points = np.vstack([e["samples"] for e in shown.values()])
        w = int(row.get("width") or k["width"])
        h = int(row.get("height") or k["height"])
        sx, sy = w / k["width"], h / k["height"]  # a picture sent smaller than rendered, as compose scales
        K = np.array([[k["fx"] * sx, 0.0, k["cx"] * sx], [0.0, k["fy"] * sy, k["cy"] * sy], [0.0, 0.0, 1.0]])
        motion = track.motion(float(t) - true_lag_s, rate_window_s)
        kind = ("unknown" if motion is None
                else "turning" if abs(motion["yaw_rate_radps"]) >= turn_radps else "straight")
        shifts = {}
        for lag in lag_settings:
            s = pixel_shift_in_view(points, cams[lag], cams[true_lag_s], K, w, h)
            shifts[_lag_key(lag)] = {"max_px": s["max_px"], "p95_px": s["p95_px"],
                                     "visible_samples": int(s["shift_px"].size)}
        rows.append({"frame_id": row.get("frame_id"), "session_epoch": row.get("session_epoch"),
                     "stamp_sim_s": float(t), "compose_sim_s": compose_t, "motion": kind,
                     "yaw_rate_radps": None if motion is None else motion["yaw_rate_radps"],
                     "speed_mps": None if motion is None else motion["speed_mps"],
                     "paths": sorted(shown), "samples": int(len(points)), "shift": shifts})

    def summary(key: str, subset) -> dict:
        values = [r["shift"][key]["max_px"] for r in subset]
        seen = [v for v in values if v is not None]
        return {"no_visible": len(values) - len(seen), **shift_summary(seen)}  # an unseen frame is never a 0

    settings = {}
    for lag in lag_settings:
        key = _lag_key(lag)
        settings[key] = {"lag_setting_s": lag, "all": summary(key, rows)}
        for kind in ("straight", "turning", "unknown"):
            settings[key][kind] = summary(key, [r for r in rows if r["motion"] == kind])
    report = {
        "tool": "tools/live_frames.py delta", "version": TOOL_VERSION, "kind": DELTA_KIND,
        "approximations": DELTA_APPROXIMATIONS, "run": str(run), "frames": str(frames_dir),
        "true_lag_s": true_lag_s, "lag_settings_s": lag_settings, "turn_threshold_radps": turn_radps,
        "rate_window_s": rate_window_s, "sample_m": sample_m,
        "session_epoch": epoch, "capture_session_epochs": capture_epochs,
        "metric": "per frame, the largest pixel distance, over the displayed lines' samples on the picture, between "
                  "the camera at t - setting and at t - true_lag (t = stamp_sim_s); summaries are over the frames "
                  "with at least one sample on the picture (no_visible counts the others)",
        "sources": {"poses": pose_src, "mount": mount_src, "rides_lift": rides_lift, "intrinsics": k_src,
                    "lift": None if lift is None else lift[2], "paths": paths_src, "paths_time_resolved": resolved,
                    "path_display_header": display_header, "path_display_summary": path_display_summary(run),
                    "session_epoch": epoch_src,
                    "compose_time": "live_log.jsonl camera sim_t, joined on (session_epoch, frame_id)"
                    if composed is not None
                    else "stamp_sim_s (no live_log.jsonl: lines added between render and read are not told apart)"},
        "counts": {"measured": len(rows), "skipped": skipped,
                   "no_visible": {key: settings[key]["all"]["no_visible"] for key in settings}},
        "settings": settings,
    }
    if per_frame:
        report["per_frame"] = rows
    return report


# ------------------------------------------------------------------ fit (S4 both diagnostic)

FIT_KIND = "offline lag fit against the rendered display lines"
FIT_STEPS = tuple(range(21))  # the sweep, in 120 Hz physics steps
FIT_JUDGED = (0, 11)  # the two settings the both-diagnostic registration compares
FIT_LINE_TOL = 40  # Chebyshev RGB tolerance of a rendered line pixel to a display colour (60 took a yellow board)
FIT_EXCLUDED = ("Return", "contrast")  # a JPEG'd magenta drifts to the Return purple; contrast is the overlay itself
FIT_MAGENTA_OVER_G = 60  # magenta-like: R and B both this much over G ...
FIT_MAGENTA_MAX_RB = 40  # ... and within this of each other
FIT_CAP_PX = 40.0  # a rendered pixel's distance is capped here before the mean (a line not displayed, a stray colour)
FIT_MIN_SAMPLES = 20  # samples on the picture in every pose of the sweep
FIT_MIN_LINE_PX = 50  # rendered line pixels measured (within FIT_CAP_PX of the projection)
FIT_INDISTINGUISHABLE_PX = 1.0


def rendered_line_mask(rgb: np.ndarray, colours) -> np.ndarray:
    """The rendered display lines in a decoded frame: pixels within FIT_LINE_TOL of a display colour, not
    magenta-like (the diagnostic overlay), then closed 3 x 3 so the 1 px the overlay covers on a line counts."""
    from scipy.ndimage import binary_closing

    px = rgb.astype(np.int16)
    r, g, b = px[..., 0], px[..., 1], px[..., 2]
    magenta = (r - g >= FIT_MAGENTA_OVER_G) & (b - g >= FIT_MAGENTA_OVER_G) & (np.abs(r - b) <= FIT_MAGENTA_MAX_RB)
    line = np.zeros(magenta.shape, dtype=bool)
    for c in colours:
        line |= np.abs(px - np.asarray(c, dtype=np.int16)).max(axis=2) <= FIT_LINE_TOL
    line &= ~magenta
    return binary_closing(line, structure=np.ones((3, 3), dtype=bool), border_value=0) | line


def fit_frame(mask: np.ndarray, points: np.ndarray, cams: dict, K: np.ndarray) -> dict:
    """Per lag key, the mean distance from each rendered line pixel near the lines (``mask`` within FIT_CAP_PX of
    the projection at some lag of the sweep) to the displayed lines projected with that key's camera (each distance
    capped at FIT_CAP_PX). Measured from the rendered side so a wide rendered line is matched at its middle, and a
    mean so lags a pixel apart are told apart (a median of integer-like distances ties). ``samples``: projected
    samples on the picture in every pose; ``line_px``: the rendered pixels measured."""
    from scipy.ndimage import distance_transform_edt

    from forklift_core.live.frame_judge import NEAR_M, _project

    h, w = mask.shape
    rasters, ok = {}, np.ones(len(points), dtype=bool)
    for key, cam in cams.items():
        z, uv = _project(points, cam, K)
        ui, vi = np.round(uv[:, 0]), np.round(uv[:, 1])
        inside = (z > NEAR_M) & np.isfinite(ui) & np.isfinite(vi) & (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
        ok &= inside
        r = np.zeros(mask.shape, dtype=bool)
        r[vi[inside].astype(int), ui[inside].astype(int)] = True
        rasters[key] = r
    out = {"samples": int(ok.sum()), "line_px": 0, "mean_px": {}}
    if not ok.any():
        return out
    near = mask & (distance_transform_edt(~np.logical_or.reduce(list(rasters.values()))) <= FIT_CAP_PX)
    out["line_px"] = int(near.sum())
    if not near.any():
        return out
    for key, r in rasters.items():
        out["mean_px"][key] = float(np.mean(np.minimum(distance_transform_edt(~r)[near], FIT_CAP_PX)))
    return out


def fit(run, frames_dir, *, paths: Path | None = None, mount_json: Path | None = None,
        intrinsics_json: Path | None = None, session_epoch=None, turn_radps: float = TURN_RADPS,
        rate_window_s: float = RATE_WINDOW_S, sample_m: float = SAMPLE_M, allow_incomplete: bool = False,
        per_frame: bool = False) -> dict:
    """Which pixel lag puts the displayed lines on the rendered ones, under FORKLIFT_PATH_DISPLAY_DIAGNOSTIC=both
    (docs/plans/2026-11-15-s4-both-diagnostic.md): per captured frame, the displayed polylines (RUN/path_display.jsonl
    at the compose time) are projected with the camera at t - k/120 for every k in FIT_STEPS (t = stamp_sim_s, the
    reconstructed camera as in ``delta``) and measured against the frame's rendered line pixels
    (``rendered_line_mask``, ``fit_frame``); summaries are medians of the per-frame means, split straight /
    turning by the yaw rate at t - PIXEL_LAG_S. ``comparison``: the registered steps' turning medians."""
    from PIL import Image

    from forklift_core.live.frame_judge import PoseTrack, camera_pose, densify_polyline, displayed_at, interpolate_value

    run, frames_dir = Path(run), Path(frames_dir)
    result = _result(run)
    mount, rides_lift, mount_src = load_mount(result, mount_json)
    k, k_src = load_intrinsics(result, intrinsics_json)
    if paths is None:
        if not (run / PATH_DISPLAY_FILE).exists():
            raise RecordMissing(f"the run has no {PATH_DISPLAY_FILE}: give --paths")
        paths = run / PATH_DISPLAY_FILE
    events, resolved, paths_src, display_header = load_path_events(Path(paths))
    for e in events:
        if not e.get("clear"):
            e["samples"] = densify_polyline(e["points"], sample_m, closed=e.get("closed", False))
    stamps, poses, pose_src = load_poses(run, allow_incomplete)
    track = PoseTrack(stamps, poses)
    lift = load_lift(run, result, allow_incomplete) if rides_lift else None
    if rides_lift and lift is None:
        raise RecordMissing("the camera rides the lift but the run has no lift record (state.samples lift_m)")
    entries = read_run_log(run, allow_incomplete)
    run_epoch, run_epoch_src = run_session_epoch(entries, result)
    frame_rows = _frame_rows(frames_dir)
    capture_epochs = sorted({r.get("session_epoch") for r in frame_rows}, key=json.dumps)
    epoch, epoch_src = _choose_epoch(session_epoch, run_epoch, run_epoch_src, capture_epochs)
    composed = compose_times(entries, epoch)
    colours = [tuple(v["rgb"]) for name, v in overlay_colours().items() if not name.startswith(FIT_EXCLUDED)]

    rows = []
    for row in frame_rows:
        if row.get("session_epoch") != epoch or row.get("stamp_sim_s") is None:
            continue
        t = float(row["stamp_sim_s"])
        compose_t = t
        if composed is not None:
            entry = composed.get((row.get("session_epoch"), row.get("frame_id")))
            if entry is None or not 0.0 <= entry[1] - t <= COMPOSE_SLACK_S:
                continue
            compose_t = entry[1]
        out = {"frame_id": row.get("frame_id"), "file": row.get("file"), "stamp_sim_s": t, "counted": False}
        rows.append(out)
        cams = {}
        for step in FIT_STEPS:
            base = track.at(t - step / 120)
            lift_m = 0.0 if lift is None else _lift_at(lift, t - step / 120, interpolate_value)
            if base is None or lift_m is None:
                cams = None
                break
            cams[str(step)] = camera_pose(base, mount, lift_m)
        if cams is None:
            out["why"] = "outside_poses"
            continue
        shown = displayed_at(events, compose_t)
        if not shown:
            out["why"] = "no_path"
            continue
        with Image.open(frames_dir / row["file"]) as im:
            rgb = np.asarray(im.convert("RGB"))
        h, w = rgb.shape[:2]
        sx, sy = w / k["width"], h / k["height"]
        K = np.array([[k["fx"] * sx, 0.0, k["cx"] * sx], [0.0, k["fy"] * sy, k["cy"] * sy], [0.0, 0.0, 1.0]])
        mask = rendered_line_mask(rgb, colours)
        motion = track.motion(t - PIXEL_LAG_S, rate_window_s)
        out["motion"] = ("unknown" if motion is None
                         else "turning" if abs(motion["yaw_rate_radps"]) >= turn_radps else "straight")
        f = fit_frame(mask, np.vstack([e["samples"] for e in shown.values()]), cams, K)
        out["samples"], out["line_px"] = f["samples"], f["line_px"]
        if f["samples"] < FIT_MIN_SAMPLES:
            out["why"] = "few_samples"
            continue
        if f["line_px"] < FIT_MIN_LINE_PX:
            out["why"] = "few_line_pixels"
            continue
        out["counted"], out["mean_px"] = True, f["mean_px"]

    counted = [r for r in rows if r["counted"]]

    def med(subset, key):
        v = [r["mean_px"][key] for r in subset]
        return {"frames": len(v), "median_px": float(np.median(v)) if v else None,
                "p90_px": float(np.percentile(v, 90)) if v else None}

    by_lag = {str(step): {kind: med([r for r in counted if kind == "all" or r["motion"] == kind], str(step))
                          for kind in ("all", "turning", "straight")} for step in FIT_STEPS}
    a, b = (str(s) for s in FIT_JUDGED)
    ta, tb = by_lag[a]["turning"]["median_px"], by_lag[b]["turning"]["median_px"]
    better = (None if ta is None or tb is None
              else "indistinguishable" if abs(ta - tb) <= FIT_INDISTINGUISHABLE_PX else (a if ta < tb else b))
    report = {
        "tool": "tools/live_frames.py fit", "version": TOOL_VERSION, "kind": FIT_KIND,
        "run": str(run), "frames": str(frames_dir), "session_epoch": epoch,
        "params": {"steps": list(FIT_STEPS), "line_tol": FIT_LINE_TOL, "excluded_colours": sorted(FIT_EXCLUDED),
                   "line_colours": [list(c) for c in colours], "magenta_over_g": FIT_MAGENTA_OVER_G,
                   "magenta_max_rb": FIT_MAGENTA_MAX_RB, "closing": "3x3", "cap_px": FIT_CAP_PX,
                   "metric": "per frame: mean distance (capped) from rendered line pixels near the lines to the "
                             "projection; per lag: median over the counted frames",
                   "min_samples": FIT_MIN_SAMPLES, "min_line_px": FIT_MIN_LINE_PX, "sample_m": sample_m,
                   "turn_threshold_radps": turn_radps, "motion_at": "t - PIXEL_LAG_S"},
        "sources": {"poses": pose_src, "mount": mount_src, "rides_lift": rides_lift, "intrinsics": k_src,
                    "paths": paths_src, "paths_time_resolved": resolved, "path_display_header": display_header,
                    "session_epoch": epoch_src},
        "counts": {"frames": len(rows), "counted": len(counted),
                   "not_counted": {why: sum(1 for r in rows if r.get("why") == why)
                                   for why in ("outside_poses", "no_path", "few_line_pixels", "few_samples")}},
        "counted_time_s": {kind: (lambda v: {"frames": len(v), "min": min(v) if v else None,
                                             "quartiles": [float(q) for q in np.percentile(v, [25, 50, 75])] if v
                                             else None, "max": max(v) if v else None})(
            [r["stamp_sim_s"] for r in counted if r["motion"] == kind]) for kind in ("turning", "straight")},
        "by_lag": by_lag,
        "comparison": {"judged_steps": list(FIT_JUDGED), "frames": by_lag[a]["turning"]["frames"],
                       "turning_median_px": {a: ta, b: tb},
                       "straight_median_px": {a: by_lag[a]["straight"]["median_px"],
                                              b: by_lag[b]["straight"]["median_px"]},
                       "indistinguishable_within_px": FIT_INDISTINGUISHABLE_PX, "better": better},
    }
    if per_frame:
        report["per_frame"] = rows
    return report


# ------------------------------------------------------------------ command line


def _run_capture(args) -> int:
    async def go():
        stop = asyncio.Event()
        loop = asyncio.get_running_loop()
        handled = []
        for sig in (signal.SIGINT, signal.SIGTERM):
            with contextlib.suppress(NotImplementedError, RuntimeError, ValueError):
                loop.add_signal_handler(sig, stop.set)  # Ctrl-C ends cleanly with the end row
                handled.append(sig)
        try:
            return await capture(args.url, args.out, seconds=args.seconds, every=args.every,
                                 max_frames=args.max_frames, hz=args.hz, stop=stop,
                                 retry_s=args.retry_s)
        finally:
            for sig in handled:
                loop.remove_signal_handler(sig)

    try:
        media_url(args.url)
        end = asyncio.run(go())
    except (FileExistsError, ValueError) as exc:
        print(f"live_frames: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(end))
    return 1 if end["reason"] == "unreachable" else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    c = sub.add_parser("capture", help="save the camera frames the console receives")
    c.add_argument("--url", required=True, help="ws://HOST:PORT of the live console")
    c.add_argument("--out", required=True, type=Path, help="an empty or absent directory")
    c.add_argument("--seconds", type=float, default=None)
    c.add_argument("--every", type=int, default=1, help="keep every Nth new frame")
    c.add_argument("--max-frames", type=int, default=None)
    c.add_argument("--hz", type=float, default=CAMERA_HZ, help="subscribed rate (server caps it)")
    c.add_argument("--retry-s", type=float, default=RETRY_S,
                   help="reconnect a connection silent this long; end as unreachable when no connection "
                        "could be opened for this long")
    j = sub.add_parser("judge", help="count the overlay colours in captured frames (JSON)")
    j.add_argument("dir", type=Path)
    j.add_argument("--compare", type=Path, default=None)
    j.add_argument("--tol", type=int, default=40, help="per-channel absolute tolerance, 0-255")
    j.add_argument("--min-component", type=int, default=50, help="pixels, 4-connected")
    j.add_argument("--colour", action="append", type=_parse_colour, default=[],
                   help="an extra colour NAME=R,G,B (repeatable; replaces a found name)")
    j.add_argument("--per-frame", action="store_true", help="include every frame's counts")
    d = sub.add_parser("delta", help=f"{DELTA_KIND} of the overlay lines' pixel shift for pixel-lag settings "
                                     "(JSON; base truth pose + mount + lift, densified lines)")
    d.add_argument("--run", required=True, type=Path, help="the runner's output directory (epoch_N/run)")
    d.add_argument("--frames", required=True, type=Path, help="a capture directory (index.jsonl)")
    d.add_argument("--paths", type=Path, default=None,
                   help=f"default RUN/{PATH_DISPLAY_FILE} (the runner's timed display record); or display events "
                        "{time_s, name, poses|points} as JSONL, a directory of JSONL or a JSON list; or a static "
                        "paths.json {name: {poses}} (no time: every path stands from the start)")
    d.add_argument("--sample-m", type=float, default=SAMPLE_M,
                   help=f"world spacing of the samples along each displayed line (default {SAMPLE_M})")
    d.add_argument("--lag-setting-s", action="append", type=float, default=None,
                   help=f"the overlay's pixel lag setting (repeatable; default 0 and {PIXEL_LAG_S:.4f})")
    d.add_argument("--true-lag-s", type=float, default=PIXEL_LAG_S,
                   help=f"the assumed lag of the pixels behind the render time (default {PIXEL_LAG_S:.4f} = 11 steps)")
    d.add_argument("--mount-json", type=Path, default=None,
                   help="{translation_m, rotation, rides_lift} instead of result.json perception_mount")
    d.add_argument("--intrinsics-json", type=Path, default=None,
                   help="{fx, fy, cx, cy, width, height} instead of result.json live_video_camera.intrinsics")
    d.add_argument("--session-epoch", type=int, default=None,
                   help="the run's session epoch, when the run does not record it and the capture holds several "
                        "(must match the run's when it does)")
    d.add_argument("--turn-threshold-radps", type=float, default=TURN_RADPS)
    d.add_argument("--rate-window-s", type=float, default=RATE_WINDOW_S)
    d.add_argument("--allow-incomplete", action="store_true",
                   help="read an incomplete spool or live log (what was written)")
    d.add_argument("--per-frame", action="store_true", help="include every frame's shifts")
    ft = sub.add_parser("fit", help=f"{FIT_KIND}: the lag (0-20 steps) that puts the displayed lines on the "
                                     "rendered ones, from a FORKLIFT_PATH_DISPLAY_DIAGNOSTIC=both run (JSON)")
    ft.add_argument("--run", required=True, type=Path, help="the runner's output directory (epoch_N/run)")
    ft.add_argument("--frames", required=True, type=Path, help="a capture directory (index.jsonl)")
    ft.add_argument("--paths", type=Path, default=None, help=f"default RUN/{PATH_DISPLAY_FILE}")
    ft.add_argument("--mount-json", type=Path, default=None)
    ft.add_argument("--intrinsics-json", type=Path, default=None)
    ft.add_argument("--session-epoch", type=int, default=None)
    ft.add_argument("--allow-incomplete", action="store_true")
    ft.add_argument("--per-frame", action="store_true")
    args = parser.parse_args(argv)
    if args.command == "fit":
        try:
            report = fit(args.run, args.frames, paths=args.paths, mount_json=args.mount_json,
                         intrinsics_json=args.intrinsics_json, session_epoch=args.session_epoch,
                         allow_incomplete=args.allow_incomplete, per_frame=args.per_frame)
        except (OSError, ValueError, KeyError) as exc:
            print(f"live_frames: {exc}", file=sys.stderr)
            return 2
        print(json.dumps(report, indent=2))
        return 0
    if args.command == "capture":
        return _run_capture(args)
    if args.command == "delta":
        try:
            report = delta(args.run, args.frames,
                           lag_settings=args.lag_setting_s if args.lag_setting_s else (0.0, PIXEL_LAG_S),
                           true_lag_s=args.true_lag_s, paths=args.paths, mount_json=args.mount_json,
                           intrinsics_json=args.intrinsics_json, session_epoch=args.session_epoch,
                           turn_radps=args.turn_threshold_radps, rate_window_s=args.rate_window_s,
                           sample_m=args.sample_m, allow_incomplete=args.allow_incomplete,
                           per_frame=args.per_frame)
        except (OSError, ValueError, KeyError) as exc:  # RecordMissing is a ValueError
            print(f"live_frames: {exc}", file=sys.stderr)
            return 2
        print(json.dumps(report, indent=2))
        return 0
    try:
        report = judge(args.dir, args.compare, tol=args.tol, min_component=args.min_component,
                       extra=args.colour, per_frame=args.per_frame)
    except (OSError, ValueError) as exc:
        print(f"live_frames: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
