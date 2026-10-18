"""The live session's supervisor (persistent console plan D9): one runner epoch after another, by its exit code.

    python tools/live_supervisor.py --out RUN --port-file RUN/live_port.json \\
        --runner '["python", "sim/isaac/run_transport.py", "--live", "--live-supervised", "--live-port", "8798", ...,
                   "--live-port-file", "{port_file}", "--output", "{epoch_dir}/run"]' \\
        [--bridge '[...]' --bridge-stop '[...]' --bridge-ready '{epoch_dir}/bridge/bridge_ready']

Each epoch owns ``<out>/epoch_<n>/``: the runner's output (its --output must not exist yet), the SLAM bridge (it takes
one connection, so a new one each epoch) and the runner. The runner's exit decides what follows:

- 0, the session ended: stop.
- 3, the operator asked for a new scene: the next epoch at once.
With ``--exit-file`` the runner's own code file counts, not the process's: Isaac's ``app.close()`` ends the process with
0 whatever happened; a runner that wrote no file crashed.

- anything else (a session failure, a crash): the port file says ``crashed`` (the runner may have left it
  ``running``), then the next epoch after a backoff; three such ends in a row stop the supervisor.

A command is a JSON list of strings; ``{epoch}``, ``{epoch_dir}``, ``{port_file}`` and ``{out}`` are filled in.
Every start and exit goes to ``<out>/supervisor.jsonl``.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ENDED, NEW_SCENE = 0, 3


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


def _mark_crashed(port_file: Path, epoch: int, code: int) -> None:
    """The runner ended without closing its port file (or failed): readers must not take it as running."""
    try:
        info = json.loads(port_file.read_text()) if port_file.exists() else {}
    except (OSError, ValueError):
        info = {}
    info.update(state="crashed", epoch=epoch, exit_code=code)
    tmp = port_file.with_name(port_file.name + ".tmp")
    tmp.write_text(json.dumps(info))
    os.replace(tmp, port_file)


def supervise(*, out: Path, runner: list[str], port_file: Path, backoff_s: float = 10.0, max_failures: int = 3,
              max_epochs: int | None = None, bridge: list[str] | None = None, bridge_stop: list[str] | None = None,
              bridge_ready: str | None = None, ready_timeout_s: float = 120.0, exit_file: str | None = None) -> dict:
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    codes, failures, epoch = [], 0, 0
    while max_epochs is None or epoch < max_epochs:
        epoch += 1
        epoch_dir = out / f"epoch_{epoch}"
        epoch_dir.mkdir()
        values = {"epoch": epoch, "epoch_dir": str(epoch_dir), "port_file": str(port_file), "out": str(out)}
        try:
            ready = True
            if bridge is not None:
                subprocess.run(_fill(bridge, **values), check=True)
                if bridge_ready is not None:
                    flag, end = Path(_fill([bridge_ready], **values)[0]), time.monotonic() + ready_timeout_s
                    while not flag.exists() and time.monotonic() < end:
                        time.sleep(0.5)
                    ready = flag.exists()
            if ready:
                _log(out, {"event": "start", "epoch": epoch})
                code = subprocess.run(_fill(runner, **values)).returncode
                if exit_file is not None:
                    # The runner's own code, written before Isaac's app.close() -- which ends the process with 0
                    # whatever the run did (ws1 2026-10-18). No file: it did not get that far, a crash.
                    path = Path(_fill([exit_file], **values)[0])
                    try:
                        code = int(path.read_text().strip())
                    except (OSError, ValueError):
                        _log(out, {"event": "no_exit_file", "epoch": epoch, "process_code": code})
                        code = code if code not in (ENDED, NEW_SCENE) else 1
            else:  # no bridge: no runner (it would fail its SLAM warm-up); counted as a failed epoch
                _log(out, {"event": "bridge_not_ready", "epoch": epoch})
                code = 1
        finally:
            if bridge_stop is not None:
                subprocess.run(_fill(bridge_stop, **values))
        codes.append(code)
        _log(out, {"event": "exit", "epoch": epoch, "code": code})
        if code == ENDED:
            return {"epochs": epoch, "end": "ended", "codes": codes}
        if code == NEW_SCENE:
            failures = 0
            continue
        _mark_crashed(port_file, epoch, code)
        failures += 1
        if failures >= max_failures:
            return {"epochs": epoch, "end": "failing", "codes": codes}
        time.sleep(backoff_s)
    return {"epochs": epoch, "end": "epoch_limit", "codes": codes}


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
    args = ap.parse_args(argv)
    load = (lambda text: None if text is None else json.loads(text))
    result = supervise(out=args.out, runner=load(args.runner), port_file=args.port_file, backoff_s=args.backoff_s,
                       max_failures=args.max_failures, max_epochs=args.max_epochs, bridge=load(args.bridge),
                       bridge_stop=load(args.bridge_stop), bridge_ready=args.bridge_ready, exit_file=args.exit_file)
    print(json.dumps(result), flush=True)
    return 0 if result["end"] == "ended" else 1


if __name__ == "__main__":
    sys.exit(main())
