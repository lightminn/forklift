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
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
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


def _mark_crashed(port_file: Path, epoch: int, code: int, reason: str | None = None) -> None:
    """The runner ended without closing its port file (or failed): readers must not take it as running. ``reason``
    (the runner's, when it wrote one) tells the console why (SLAM localization plan L4: a session start check)."""
    try:
        info = json.loads(port_file.read_text()) if port_file.exists() else {}
    except (OSError, ValueError):
        info = {}
    info.update(state="crashed", epoch=epoch, exit_code=code)
    if reason is not None:
        info["reason"] = reason
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


def _end_child(child, grace_s: float = 60.0) -> int:
    try:
        return child.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        child.kill()
        return child.wait()


def supervise(*, out: Path, runner: list[str], port_file: Path, backoff_s: float = 10.0, max_failures: int = 3,
              max_epochs: int | None = None, bridge: list[str] | None = None, bridge_stop: list[str] | None = None,
              bridge_ready: str | None = None, ready_timeout_s: float = 120.0, exit_file: str | None = None,
              grace_s: float = 60.0, map_bundle: Path | None = None) -> dict:
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    session_values = {}
    if map_bundle is not None:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
        from forklift_core.localization.slam_map import MapBundleError, load_bundle

        try:
            bundle = load_bundle(Path(map_bundle))
        except MapBundleError as exc:
            _log(out, {"event": "map_bundle_refused", "reason": str(exc)})
            _mark_crashed(port_file, 0, 1, str(exc))
            return {"epochs": 0, "end": "map_mismatch", "codes": [], "reason": str(exc)}
        session_values = {"expected_map_id": bundle.map_id, "map_bundle": str(Path(map_bundle).resolve())}
        _log(out, {"event": "map_bundle", **session_values})
    stop = _Stop()
    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGTERM, signal.SIGINT)}
    codes, epoch, last_cause, repeats = [], 0, None, 0
    try:
        while not stop.asked and (max_epochs is None or epoch < max_epochs):
            epoch += 1
            epoch_dir = out / f"epoch_{epoch}"
            epoch_dir.mkdir()
            values = {"epoch": epoch, "epoch_dir": str(epoch_dir), "port_file": str(port_file), "out": str(out),
                      **session_values}
            code, reason = 1, None
            try:
                ready = True
                if bridge is not None:
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
                    _log(out, {"event": "start", "epoch": epoch})
                    stop.child = subprocess.Popen(_fill(runner, **values))
                    while True:  # short waits, so a TERM arriving meanwhile turns into the grace and the KILL
                        try:
                            process = stop.child.wait(timeout=0.5)
                            break
                        except subprocess.TimeoutExpired:
                            if stop.asked:
                                process = _end_child(stop.child, grace_s)
                                break
                    stop.child = None
                    code = process
                    if exit_file is not None:
                        # Isaac's app.close() ends the process with 0 whatever the run did (ws1 2026-10-18): with
                        # 0 the runner's own file counts (none: it did not get that far, a crash); a process that
                        # really failed (killed, crashed) counts as that, whatever its file says.
                        written = _read_exit(Path(_fill([exit_file], **values)[0]))
                        if process == 0 and written is not None:
                            code, reason = written
                        elif written is not None:
                            reason = written[1]  # a process that failed keeps its code; its own reason is kept too
                        elif process == 0:
                            _log(out, {"event": "no_exit_file", "epoch": epoch})
                            code, reason = 1, "no_exit_file"
            except (OSError, subprocess.SubprocessError) as exc:  # the bridge or the runner would not start
                _log(out, {"event": "start_failed", "epoch": epoch, "error": f"{type(exc).__name__}: {exc}"})
                code, reason = 1, f"start_failed: {type(exc).__name__}"
            finally:
                if stop.child is not None:  # interrupted while waiting
                    _end_child(stop.child, grace_s)
                    stop.child = None
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
                _mark_crashed(port_file, epoch, code, reason) if code not in (ENDED, NEW_SCENE) else None
                return {"epochs": epoch, "end": "terminated", "codes": codes}
            if code == ENDED:
                return {"epochs": epoch, "end": "ended", "codes": codes}
            if code == NEW_SCENE:
                last_cause, repeats = None, 0
                continue
            _mark_crashed(port_file, epoch, code, reason)
            cause = _cause(code, reason)
            if cause[1] in STOP_AT_ONCE:
                return {"epochs": epoch, "end": "map_mismatch", "codes": codes, "reason": reason}
            repeats = repeats + 1 if cause == last_cause else 1
            last_cause = cause
            if repeats >= max_failures:  # the same cause, max_failures times in a row (plan D9)
                return {"epochs": epoch, "end": "failing", "codes": codes}
            end = time.monotonic() + backoff_s
            while time.monotonic() < end and not stop.asked:
                time.sleep(min(0.5, backoff_s))
        return {"epochs": epoch, "end": "terminated" if stop.asked else "epoch_limit", "codes": codes}
    finally:
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
    args = ap.parse_args(argv)
    load = (lambda text: None if text is None else json.loads(text))
    result = supervise(out=args.out, runner=load(args.runner), port_file=args.port_file, backoff_s=args.backoff_s,
                       max_failures=args.max_failures, max_epochs=args.max_epochs, bridge=load(args.bridge),
                       bridge_stop=load(args.bridge_stop), bridge_ready=args.bridge_ready, exit_file=args.exit_file,
                       map_bundle=args.slam_map_bundle)
    print(json.dumps(result), flush=True)
    return 0 if result["end"] == "ended" else 1


if __name__ == "__main__":
    sys.exit(main())
