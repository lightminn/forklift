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
    the capture; any other close reconnects after 1 s, as the page does. DIR must be
    empty or absent. Prints the end row; exits 0, 1 when it ended ``unreachable``
    (nothing connected or received for ``--retry-s``), 2 on a refused argument or DIR.

``judge DIR [--compare DIR_B] [--tol 40] [--min-component 50] [--colour NAME=R,G,B]``
    Decodes each indexed frame and counts, per overlay colour, the pixels within
    ``--tol`` per channel and the 4-connected components of at least
    ``--min-component`` pixels (forklift_core.live.frame_judge). Prints JSON. It decides
    nothing: the S4 registration sets the thresholds.

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
                       "connections": 0, "reconnects": 0}

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

    last_up = time.monotonic()
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
                last_up = time.monotonic()
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
                        continue  # recv() is safe to cancel: nothing is lost
                    last_up = time.monotonic()
                    if cap.message(raw, time.monotonic_ns()):
                        return "max_frames"
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
    (``interrupted``), when the server closes with 1001 (``runner_ended``) or when nothing
    could be connected or received for ``retry_s`` (``unreachable``). Returns the end row
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


def overlay_colours(root: Path = ROOT) -> dict[str, dict]:
    """The overlay's colours by display name: ``{name: {"rgb": [r, g, b], "sources":
    ["file:line", ...]}}``, 0-255 as path_overlay draws them.

    Read from the literal colours at the display calls in COLOUR_SOURCES (a call whose
    colour or name is a variable is skipped; its literal origin is another call). The
    destination ring is ``destination``; path_overlay's CONTRAST_COLOUR is ``contrast``.
    A name drawn in two colours gets one entry per colour, ``name@r,g,b``.
    """
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
                   help="end as unreachable after this long with no connection or message")
    j = sub.add_parser("judge", help="count the overlay colours in captured frames (JSON)")
    j.add_argument("dir", type=Path)
    j.add_argument("--compare", type=Path, default=None)
    j.add_argument("--tol", type=int, default=40, help="per-channel absolute tolerance, 0-255")
    j.add_argument("--min-component", type=int, default=50, help="pixels, 4-connected")
    j.add_argument("--colour", action="append", type=_parse_colour, default=[],
                   help="an extra colour NAME=R,G,B (repeatable; replaces a found name)")
    j.add_argument("--per-frame", action="store_true", help="include every frame's counts")
    args = parser.parse_args(argv)
    if args.command == "capture":
        return _run_capture(args)
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
