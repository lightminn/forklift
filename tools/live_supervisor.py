"""The live session's supervisor (persistent console plan D9): one runner epoch after another, by its exit code.

    python tools/live_supervisor.py --out RUN --port-file RUN/live_port.json \\
        --runner '["python", "sim/isaac/run_transport.py", "--live", "--live-supervised", "--live-port", "8798", ...,
                   "--live-port-file", "{port_file}", "--output", "{epoch_dir}/run"]' \\
        [--bridge '[...]' --bridge-stop '[...]' --bridge-ready '{epoch_dir}/bridge/bridge_ready']

Each epoch owns ``<out>/epoch_<n>/``: the runner's output (its --output must not exist yet), the SLAM bridge (it takes
one connection, so a new one each epoch) and the runner. The runner's exit decides what follows:

- 0, the session ended: stop.
- 3, the operator asked for a new scene: the next epoch at once.
With ``--exit-file`` the runner's own code file ({"code", "reason"}) counts when the process ended with 0 -- Isaac's
``app.close()`` ends it with 0 whatever happened; a runner that wrote no file crashed; a process that really failed
counts as that.

- anything else (a session failure, a crash): the port file says ``crashed`` (the runner may have left it
  ``running``), then the next epoch after a backoff; three ends of the same cause in a row stop the supervisor.

A TERM or INT ends the running runner (TERM, then KILL after a grace), removes its bridge and starts no new epoch.

A command is a JSON list of strings; ``{epoch}``, ``{epoch_dir}``, ``{port_file}`` and ``{out}`` are filled in.
Every start and exit goes to ``<out>/supervisor.jsonl``.

With ``--slam-map-bundle`` (SLAM localization plan L4) the bundle is checked and hashed once, at the session's start:
its map id fills ``{expected_map_id}`` and its folder ``{map_bundle}`` in every epoch's commands, so each epoch's
runner and bridge are held to the same map. A bundle that does not load ends the session before the first epoch;
an epoch that ends with ``slam_map_mismatch`` stops the supervisor at once (the bundle cannot change between epochs).
The port file of a failed epoch carries the runner's ``reason`` next to ``state=crashed``.

With ``--standby-port`` (contract section 6) the supervisor serves the console's port while no runner holds it -- the
page and ``/api/v1/session`` (its own state: phase, epoch, the last exit and whether it came before the runner's
server opened) -- so a start check's failure reaches the screen; after a failed end it stays up
``--standby-linger-s``.
"""

from __future__ import annotations

import argparse
import http
import json
import os
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ENDED, NEW_SCENE = 0, 3
STOP_AT_ONCE = ("slam_map_mismatch",)  # causes no retry can change (plan L4)


def _fill(command: list[str], **values) -> list[str]:
    """The named placeholders only (a command may carry braces of its own, e.g. a JSON or Python argument)."""
    filled = []
    for part in command:
        for key, value in values.items():
            part = part.replace("{" + key + "}", str(value))
        filled.append(part)
    return filled


def _log(out: Path, entry: dict) -> None:
    with (out / "supervisor.jsonl").open("a") as log:
        log.write(json.dumps({"wall": time.time(), **entry}) + "\n")


def _mark_crashed(port_file: Path, epoch: int, code: int, reason: str | None = None,
                  stage: str | None = None) -> None:
    """The runner ended without closing its port file (or failed): readers must not take it as running. ``reason``
    (the runner's, when it wrote one) tells the console why (SLAM localization plan L4: a session start check);
    ``stage`` (a start relocalization's, B-P2) where it ended."""
    try:
        info = json.loads(port_file.read_text()) if port_file.exists() else {}
    except (OSError, ValueError):
        info = {}
    info.update(state="crashed", epoch=epoch, exit_code=code)
    if stage is not None:
        info["stage"] = stage
    if reason is not None:
        info["reason"] = reason
    else:
        info.pop("reason", None)  # an earlier epoch's reason is not this one's (Codex S3 review P3)
    tmp = port_file.with_name(port_file.name + ".tmp")
    tmp.write_text(json.dumps(info))
    os.replace(tmp, port_file)


def _cause(code: int, reason: str | None) -> tuple:
    """What a failed epoch is counted by: its code and its reason's head ("slam_link_failed: scan 7082: ..." and
    "slam_link_failed: scan 12: ..." are one cause)."""
    return code, None if not reason else str(reason).split(":", 1)[0].strip()


def _read_exit(path: Path) -> tuple[int, str | None] | None:
    try:
        text = path.read_text().strip()
    except OSError:
        return None
    try:
        value = json.loads(text)
    except ValueError:
        return None
    if isinstance(value, int) and not isinstance(value, bool):
        return value, None
    if isinstance(value, dict) and isinstance(value.get("code"), int):
        return value["code"], value.get("reason")
    return None


# The console's static files the standby serves (the runner's own list, console/wire.py static_response).
_MIME = {".html": "text/html; charset=utf-8", ".css": "text/css; charset=utf-8",
         ".js": "text/javascript; charset=utf-8"}


class Standby:
    """While no runner holds the console's port, the supervisor serves it: the console's page and, at
    ``/api/v1/session``, where the session is -- so a start check's failure (a runner that ends before its server
    opens: SLAM localization plan L4) reaches the screen (contract section 6). Closed while a runner runs; its state
    is the supervisor's own snapshot, not the port file. A page that opens while the runner starts up (between the
    standby's close and the runner's server) gets nothing until the runner serves (user 2026-10-21: accepted)."""

    def __init__(self, host: str, port: int, web_root: Path | None, out: Path):
        self.host, self.port, self.web_root, self.out = host, int(port), web_root, out
        self._lock = threading.Lock()
        self._state = {"standby": True, "phase": "starting", "epoch": 0, "next_epoch": None, "retry": None,
                       "end": None, "last_exit": None, "changed_wall": time.time()}
        self._server = None
        self._thread = None

    def update(self, **fields) -> None:
        with self._lock:
            self._state.update(fields, changed_wall=time.time())

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._state, served_wall=time.time())

    def open(self) -> bool:
        if self._server is not None:
            return True
        standby = self

        class Handler(BaseHTTPRequestHandler):
            timeout = 5.0  # a stuck client does not hold a request thread

            def log_message(self, *args) -> None:  # noqa: D401 -- quiet (the supervisor logs what matters)
                pass

            def _send(self, code: int, body: bytes, kind: str) -> None:
                self.send_response(code)
                self.send_header("Content-Type", kind)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self) -> None:  # noqa: N802 -- http.server's name
                path = self.path.split("?", 1)[0]
                if path == "/api/v1/session":
                    self._send(200, json.dumps(standby.snapshot()).encode(), "application/json")
                    return
                if path in ("/api/v1/control", "/api/v1/media"):  # no runner: its WebSocket is refused
                    self._send(http.HTTPStatus.SERVICE_UNAVAILABLE, b'{"standby": true}', "application/json")
                    return
                rel = "index.html" if path in ("/", "/index.html") else path.lstrip("/")
                if not (rel == "index.html" or rel == "app.css" or (rel.startswith("js/") and rel.endswith(".js")
                                                                      and ".." not in rel)):
                    self._send(404, b"not found\n", "text/plain")
                    return
                target = None if standby.web_root is None else Path(standby.web_root) / rel
                if target is None or not target.is_file():
                    self._send(404, b"not found\n", "text/plain")
                    return
                self._send(200, target.read_bytes(), _MIME[target.suffix])

        try:
            ThreadingHTTPServer.allow_reuse_address = True
            server = ThreadingHTTPServer((self.host, self.port), Handler)
        except OSError as exc:  # logged; never in place of a runner's reason
            _log(self.out, {"event": "standby_bind_failed", "error": f"{type(exc).__name__}: {exc}"})
            return False
        self._server = server
        self._thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.2},
                                        name="standby", daemon=True)
        self._thread.start()
        return True

    def close(self) -> None:
        """Synchronously: no listener on the port when this returns (the runner binds it next)."""
        server, self._server = self._server, None
        if server is None:
            return
        server.shutdown()
        server.server_close()
        self._thread.join(timeout=5.0)


class _NoStandby:
    def update(self, **fields) -> None:
        pass

    def open(self) -> bool:
        return False

    def close(self) -> None:
        pass


def _opened_its_server(port_file: Path, since_wall: float, pid: int) -> bool:
    """This runner (its pid) opened its server after it was started -- its port file carries the server's start time,
    running or at its close (LiveRuntime keeps it; Codex standby review P2-2): a failure after that is a run's,
    before it a start's."""
    try:
        info = json.loads(port_file.read_text())
    except (OSError, ValueError):
        return False
    # B-P2c: a relocalizing runner's early console opens before the localization and says where it is
    return (info.get("pid") == pid and info.get("state") in ("running", "ended", "stop_failed", *EARLY_STATES)
            and float(info.get("started") or 0) >= since_wall - 1)


EARLY_STATES = ("relocalizing", "awaiting_pick", "reloc_failed", "stopped", "localizing")


class _Stop:
    """A TERM or INT for the supervisor (Slurm's --signal reaches the batch shell, which passes it on): the running
    runner is ended, its bridge removed, no further epoch started."""

    def __init__(self):
        self.asked = False
        self.child = None

    def __call__(self, signum, frame):
        self.asked = True
        child = self.child
        if child is not None and child.poll() is None:
            child.terminate()


class _RelocBridge:
    """B-P2 start relocalization (docs/plans/2026-10-29-start-relocalization.md 1.2 steps 8-11): the runner runs
    first; when its estimate (``reloc_pose.json``) appears the bridge starts at it (``{map_start_pose}``), and once
    the bridge is ready ``bridge_go`` lets the runner open its SLAM link. ``step()`` is polled while the runner runs
    and returns why the epoch must end, or None."""

    def __init__(self, out: Path, epoch_dir: Path, values: dict, port_file: Path, bridge, bridge_ready,
                 ready_timeout_s: float, reloc_wait_s: float):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
        from forklift_core.localization import start_reloc

        self.lib, self.out, self.epoch_dir, self.values, self.port_file = start_reloc, out, epoch_dir, values, port_file
        self.bridge, self.bridge_ready, self.ready_timeout_s = bridge, bridge_ready, ready_timeout_s
        self.reloc_wait_s, self.phase, self.pose, self.localized = reloc_wait_s, "pose", None, False
        self.total_until = self.ready_until = None
        start_reloc.clear_epoch(epoch_dir)  # before the runner starts: never a file the runner already wrote

    def started(self) -> None:
        """The runner is running: the whole wait (estimate, operator's pick and bridge, plan 1.2 steps 4-9) starts."""
        self.total_until = time.monotonic() + self.reloc_wait_s

    @property
    def stage(self) -> str:
        return "run" if self.localized else "relocalize"

    def observe(self) -> None:
        """Localization ready, read from the port file -- its state or the runner's own record of it, which survives
        a running -> closed that fell between two polls. No side effects: also after the runner is gone."""
        if self.localized:
            return
        try:
            info = json.loads(self.port_file.read_text())
        except (OSError, ValueError):
            return
        if self.phase in ("running", "done") and (info.get("state") == "running" or info.get("localized_wall")):
            _log(self.out, {"event": "localization_ready", "epoch": self.values["epoch"]})
            self.localized, self.phase = True, "done"

    def _late(self) -> bool:
        return self.total_until is not None and time.monotonic() > self.total_until

    def step(self, alive=lambda: True, asked=lambda: False) -> str | None:
        """While the runner runs (``alive``) and no TERM came (``asked``): start the bridge at its estimate, let it go
        when ready, or say why to end. Every check reads the clock afresh (Codex B-P2a re-review: a bridge command
        that returned after the whole wait must not be let go)."""
        if self.phase in ("pose", "ready") and self._late():
            return "reloc_wait_timeout"  # the whole wait is bounded, a found estimate or a slow bridge too
        if self.phase == "pose":
            data = self.lib.read_reloc_pose(self.epoch_dir)
            if data is None:
                return None
            self.pose = [float(v) for v in data["base"]]
            _log(self.out, {"event": "reloc_pose", "base": self.pose, "source": data.get("source"),
                            "generation": data.get("generation"), "attempts": data.get("attempts")})
            if self.bridge is not None:
                pose = ",".join(repr(v) for v in self.pose)
                try:
                    command = subprocess.Popen(_fill(self.bridge, map_start_pose=pose, **self.values))
                except (OSError, subprocess.SubprocessError) as exc:
                    _log(self.out, {"event": "bridge_start_failed", "error": f"{type(exc).__name__}: {exc}"})
                    return "bridge_start_failed"
                while True:  # short waits: a TERM or the whole wait cancels the command (Codex B-P2a re-review)
                    try:
                        returned = command.wait(timeout=0.2)
                        break
                    except subprocess.TimeoutExpired:
                        if asked() or self._late():
                            command.terminate()
                            _end_child(command, 2.0)
                            _log(self.out, {"event": "bridge_start_cancelled", "term": asked()})
                            return None if asked() else "reloc_wait_timeout"
                if returned != 0:
                    _log(self.out, {"event": "bridge_start_failed", "code": returned})
                    return "bridge_start_failed"
            self.phase, self.ready_until = "ready", time.monotonic() + self.ready_timeout_s
        if self.phase == "ready":
            if self._late():
                return "reloc_wait_timeout"
            flag = None if self.bridge_ready is None else Path(_fill([self.bridge_ready], **self.values)[0])
            if flag is not None and not flag.exists():
                if time.monotonic() > self.ready_until:
                    _log(self.out, {"event": "bridge_not_ready", "epoch": self.values["epoch"]})
                    return "bridge_not_ready"
                return None
            if asked() or not alive():  # the runner is ending: nothing is let go to it
                return None
            self.lib.write_bridge_go(self.epoch_dir, {"map_start_pose": self.pose})
            _log(self.out, {"event": "bridge_go", "map_start_pose": self.pose})
            self.phase = "running"
        if self.phase == "running":
            self.observe()
        return None


def _end_child(child, grace_s: float = 60.0) -> int:
    try:
        return child.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        child.kill()
        return child.wait()


def supervise(*, out: Path, runner: list[str], port_file: Path, backoff_s: float = 10.0, max_failures: int = 3,
              max_epochs: int | None = None, bridge: list[str] | None = None, bridge_stop: list[str] | None = None,
              bridge_ready: str | None = None, ready_timeout_s: float = 120.0, exit_file: str | None = None,
              grace_s: float = 60.0, map_bundle: Path | None = None, expected_map_id: str | None = None,
              slam_choice: str | None = None,
              standby_port: int | None = None, standby_host: str = "127.0.0.1", web_root: Path | None = None, standby_linger_s: float = 0.0,
              relocalize: bool = False, reloc_wait_s: float = 600.0) -> dict:
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    stop = _Stop()  # first: a TERM during the bundle check or a linger ends them too
    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGTERM, signal.SIGINT)}
    standby = Standby(standby_host, standby_port, web_root, out) if standby_port is not None else _NoStandby()
    standby.update(phase="starting", epoch=0, stage="map_check" if map_bundle is not None else None)
    standby.open()

    def stopped(result: dict, linger: bool) -> dict:
        """The session's end, shown by the standby; after a failure it stays up ``standby_linger_s`` (the job holds
        its Slurm slot that long) so the operator reads why -- not after an end asked for, nor on a TERM."""
        standby.update(phase="stopped", end=result["end"], retry=False, next_epoch=None)
        if linger and not stop.asked:
            until = time.monotonic() + standby_linger_s
            while time.monotonic() < until and not stop.asked:
                time.sleep(0.2)
        return result

    try:
        session_values = {}
        if slam_choice is not None:  # live-default plan S3 D2: what the launcher chose and why
            _log(out, {"event": "slam_choice", "choice": slam_choice})
        if map_bundle is not None:
            sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
            from forklift_core.localization.slam_map import MapBundleError, load_bundle

            try:
                # live-default plan S3 D2: the registry's accepted id, when given, against the folder's own
                bundle = load_bundle(Path(map_bundle), expected_map_id)
            except MapBundleError as exc:
                _log(out, {"event": "map_bundle_refused", "reason": str(exc)})
                _mark_crashed(port_file, 0, 1, str(exc))
                standby.update(last_exit={"epoch": 0, "exit_code": 1, "reason": str(exc), "stage": "start"})
                return stopped({"epochs": 0, "end": "map_mismatch", "codes": [], "reason": str(exc)}, True)
            session_values = {"expected_map_id": bundle.map_id, "map_bundle": str(Path(map_bundle).resolve())}
            _log(out, {"event": "map_bundle", **session_values})
        codes, epoch, last_cause, repeats = [], 0, None, 0
        while not stop.asked and (max_epochs is None or epoch < max_epochs):
            epoch += 1
            epoch_dir = out / f"epoch_{epoch}"
            epoch_dir.mkdir()
            values = {"epoch": epoch, "epoch_dir": str(epoch_dir), "port_file": str(port_file), "out": str(out),
                      **session_values}
            standby.update(phase="starting", epoch=epoch, stage="bridge", next_epoch=None, retry=None)
            code, reason, stage = 1, None, "bridge"
            reloc, ended_by = None, None
            try:
                ready = True
                if bridge is not None and not relocalize:  # B-P2: the bridge waits for the runner's estimate
                    subprocess.run(_fill(bridge, **values), check=True)
                    if bridge_ready is not None:
                        flag, end = Path(_fill([bridge_ready], **values)[0]), time.monotonic() + ready_timeout_s
                        while not flag.exists() and time.monotonic() < end and not stop.asked:
                            time.sleep(0.5)
                        ready = flag.exists()
                if stop.asked:
                    code, reason = 1, "terminated"
                elif not ready:  # no bridge: no runner (it would fail its SLAM warm-up); a failed epoch
                    _log(out, {"event": "bridge_not_ready", "epoch": epoch})
                    code, reason = 1, "bridge_not_ready"
                else:
                    standby.close()  # the port is the runner's from here
                    if stop.asked:  # a TERM while the standby closed: no runner (Codex standby design P2-4)
                        code, reason = 1, "terminated"
                    else:
                        _log(out, {"event": "start", "epoch": epoch})
                        started_wall = time.time()
                        if relocalize:  # its files are cleared before the runner can write one
                            reloc = _RelocBridge(out, epoch_dir, values, port_file, bridge, bridge_ready,
                                                 ready_timeout_s, reloc_wait_s)
                        stop.child = subprocess.Popen(_fill(runner, **values))
                        standby.update(phase="running", stage="runner")
                        if reloc is not None:
                            reloc.started()
                        if stop.asked:  # the TERM came as it started: end it now
                            stop.child.terminate()
                        while True:  # short waits, so a TERM arriving meanwhile turns into the grace and the KILL
                            try:
                                process = stop.child.wait(timeout=0.5)
                                break
                            except subprocess.TimeoutExpired:
                                if stop.asked:
                                    process = _end_child(stop.child, grace_s)
                                    break
                                ended_by = None if reloc is None else reloc.step(
                                    alive=lambda: stop.child is not None and stop.child.poll() is None,
                                    asked=lambda: stop.asked)
                                if ended_by is not None:  # B-P2: no estimate in time, or no bridge for it
                                    stop.child.terminate()
                                    process = _end_child(stop.child, grace_s)
                                    break
                        pid = stop.child.pid
                        stop.child = None
                        code = process
                        if exit_file is not None:
                            # Isaac's app.close() ends the process with 0 whatever the run did (ws1 2026-10-18):
                            # with 0 the runner's own file counts (none: it did not get that far, a crash); a
                            # process that really failed (killed, crashed) counts as that, whatever its file says.
                            written = _read_exit(Path(_fill([exit_file], **values)[0]))
                            if process == 0 and written is not None:
                                code, reason = written
                            elif written is not None:
                                reason = written[1]  # a process that failed keeps its code; its own reason too
                            elif process == 0:
                                _log(out, {"event": "no_exit_file", "epoch": epoch})
                                code, reason = 1, "no_exit_file"
                        stage = "run" if _opened_its_server(port_file, started_wall, pid) else "start"
                        if reloc is not None:  # B-P2: the server opens before localization; ready is the split
                            reloc.observe()  # a localization the last poll missed; nothing is started any more
                            stage = reloc.stage
                        if ended_by is not None:
                            code, reason = 1, ended_by
            except (OSError, subprocess.SubprocessError) as exc:  # the bridge or the runner would not start
                _log(out, {"event": "start_failed", "epoch": epoch, "error": f"{type(exc).__name__}: {exc}"})
                code, reason = 1, f"start_failed: {type(exc).__name__}"
                if reloc is not None:
                    stage = reloc.stage
            finally:
                if stop.child is not None:  # interrupted while waiting
                    _end_child(stop.child, grace_s)
                    stop.child = None
                # The runner is gone (reaped): the standby takes the port back at once, with what it ended with,
                # before the bridge's cleanup (which may wait 45 s for the bridge to write its records).
                standby.update(phase="cleanup", last_exit={"epoch": epoch, "exit_code": code, "reason": reason,
                                                           "stage": stage})
                standby.open()
                if bridge_stop is not None:
                    try:
                        cleaned = subprocess.run(_fill(bridge_stop, **values)).returncode
                        if cleaned != 0:
                            _log(out, {"event": "bridge_stop_failed", "epoch": epoch, "code": cleaned})
                    except (OSError, subprocess.SubprocessError) as exc:
                        _log(out, {"event": "bridge_stop_failed", "epoch": epoch, "error": str(exc)})
            codes.append(code)
            _log(out, {"event": "exit", "epoch": epoch, "code": code, "reason": reason})
            if stop.asked:
                if code not in (ENDED, NEW_SCENE):
                    _mark_crashed(port_file, epoch, code, reason, stage if relocalize else None)
                return stopped({"epochs": epoch, "end": "terminated", "codes": codes}, False)
            if code == ENDED:
                return stopped({"epochs": epoch, "end": "ended", "codes": codes}, False)
            if code == NEW_SCENE:
                last_cause, repeats = None, 0
                continue
            _mark_crashed(port_file, epoch, code, reason, stage if relocalize else None)
            cause = _cause(code, reason)
            if cause[1] in STOP_AT_ONCE:
                return stopped({"epochs": epoch, "end": "map_mismatch", "codes": codes, "reason": reason}, True)
            repeats = repeats + 1 if cause == last_cause else 1
            last_cause = cause
            if repeats >= max_failures:  # the same cause, max_failures times in a row (plan D9)
                return stopped({"epochs": epoch, "end": "failing", "codes": codes}, True)
            if max_epochs is not None and epoch >= max_epochs:  # no next epoch: no backoff, and none is announced
                return stopped({"epochs": epoch, "end": "epoch_limit", "codes": codes}, True)
            standby.update(phase="backoff", retry=True, next_epoch=epoch + 1)
            end = time.monotonic() + backoff_s
            while time.monotonic() < end and not stop.asked:
                time.sleep(min(0.5, backoff_s))
        # the end decides the linger, not the last runner's code (an epoch limit after a reset too; Codex standby P2-3)
        return stopped({"epochs": epoch, "end": "terminated" if stop.asked else "epoch_limit", "codes": codes},
                       not stop.asked)
    finally:
        standby.close()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--port-file", type=Path, required=True)
    ap.add_argument("--runner", required=True, help="the runner command, a JSON list of strings")
    ap.add_argument("--bridge", default=None, help="starts the epoch's SLAM bridge (JSON list), before the runner")
    ap.add_argument("--bridge-stop", default=None, help="removes it after the runner (JSON list)")
    ap.add_argument("--bridge-ready", default=None, help="a file the bridge writes when it accepts")
    ap.add_argument("--exit-file", default=None, help="the runner's own exit code file (its process code otherwise)")
    ap.add_argument("--backoff-s", type=float, default=10.0)
    ap.add_argument("--max-failures", type=int, default=3)
    ap.add_argument("--max-epochs", type=int, default=None)
    ap.add_argument("--slam-map-bundle", type=Path, default=None,
                    help="a survey map bundle: checked and hashed once; fills {expected_map_id} and {map_bundle}")
    ap.add_argument("--expected-map-id", default=None,
                    help="the map id the bundle must carry (the registry's accepted bundle); unset: the folder's own")
    ap.add_argument("--slam-choice", default=None, help="the launcher's SLAM mode choice and its reason, logged")
    ap.add_argument("--standby-port", type=int, default=None,
                    help="the console's port: served by the supervisor while no runner holds it (its page and "
                         "/api/v1/session)")
    ap.add_argument("--standby-host", default="127.0.0.1")
    ap.add_argument("--web-root", type=Path, default=None, help="the console's page (console/web)")
    ap.add_argument("--standby-linger-s", type=float, default=0.0,
                    help="after a failed end, how long the standby stays up so the operator reads why (s)")
    ap.add_argument("--relocalize", action="store_true",
                    help="B-P2 start relocalization: the runner first, the bridge at its estimate ({map_start_pose})")
    ap.add_argument("--reloc-wait-s", type=float, default=600.0,
                    help="with --relocalize: how long the runner may take to an estimate, an operator's pick included")
    args = ap.parse_args(argv)
    load = (lambda text: None if text is None else json.loads(text))
    result = supervise(out=args.out, runner=load(args.runner), port_file=args.port_file, backoff_s=args.backoff_s,
                       max_failures=args.max_failures, max_epochs=args.max_epochs, bridge=load(args.bridge),
                       bridge_stop=load(args.bridge_stop), bridge_ready=args.bridge_ready, exit_file=args.exit_file,
                       map_bundle=args.slam_map_bundle, expected_map_id=args.expected_map_id,
                       slam_choice=args.slam_choice,
                       standby_port=args.standby_port,
                       standby_host=args.standby_host, web_root=args.web_root,
                       standby_linger_s=args.standby_linger_s, relocalize=args.relocalize,
                       reloc_wait_s=args.reloc_wait_s)
    print(json.dumps(result), flush=True)
    return 0 if result["end"] == "ended" else 1


if __name__ == "__main__":
    sys.exit(main())
