#!/usr/bin/env python3
"""The arrival-day rehearsal's MCU (docs/design/2026-11-15-arrival-day-rehearsal.md 2.1): the host-only MCU model
(the real firmware core on a synthetic plant) on a pty, for the procedure's tools to link to by path.

The model is given the pty MASTER (``mcu_model --link-fd``); the tools open the slave the bench prints, exactly as
they open ``/dev/forklift-mcu`` on the truck. ``bench.json`` records what the model-peer check needs (the pty, the
model's pid, start time and exe); every tool that writes a record on a pty needs it (``--bench``), and the teleop
bridge's ``--synthetic-permit`` opens only while the check passes. Not for the truck: everything here is synthetic.

usage: rehearsal_bench.py --stage s2|s3|s4|s5|drive [--out-dir DIR] [--build DIR] [--model-arg ARG ...]
                          [--duration-ms N]
   Ctrl-C or TERM ends the model and removes bench.json.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from forklift_core.live import model_peer  # noqa: E402

DEFAULT_BUILD = REPO / "build" / "rehearsal_mcu_model"
# the model's arguments per procedure stage (design 2.1): one model per stage, never reused across stages
STAGE_ARGS = {
    "s2": ["--hand-turn-wheel", "0", "--hand-turn-revs", "2", "--hand-turn-ms", "5000",
           "--hand-turn-duration-ms", "4000"],  # procedure 6.2: a known hand turn of the left wheel
    "s3": [],  # a normal build; add --outputs-locked 1 to rehearse --allow-locked
    "s4": ["--commission-build", "1"],
    "s5": [],  # the scenario's own --estop-at-ms / --silent-at-ms / --fault-at-ms through --model-arg
    "drive": [],  # a normal build: outputs open, e-stop wired -- teleop can arm
}


def build_model(build: Path) -> Path:
    exe = build / "mcu_model"
    if exe.exists():
        return exe
    if shutil.which("cmake") is None:
        raise SystemExit("cmake is needed to build the MCU model")
    env = {**os.environ, "PYTHONPATH": str(REPO / "src")}
    for cmd in (["cmake", "-S", str(REPO / "firmware"), "-B", str(build), "-DFW_MCU_MODEL=ON",
                 f"-DFW_PYTHON={sys.executable}"], ["cmake", "--build", str(build), "--target", "mcu_model", "-j2"]):
        subprocess.run(cmd, check=True, env=env)
    return exe


def bench_record(proc: subprocess.Popen, pts: str, exe: Path, args: list[str], stage: str) -> dict:
    return {"schema": model_peer.BENCH_SCHEMA, "pts": pts, "pts_rdev": os.stat(pts).st_rdev, "model_pid": proc.pid,
            "model_starttime": model_peer.SystemProc().starttime(proc.pid), "model_exe": os.path.realpath(exe),
            "model_args": args, "stage": stage, "synthetic": True}


def start(stage: str, out_dir: Path, build: Path, extra: list[str], duration_ms: int | None):
    if os.geteuid() == 0:
        raise SystemExit("the rehearsal bench does not run as root")
    exe = build_model(build)
    args = STAGE_ARGS[stage] + list(extra) + ([] if duration_ms is None else ["--duration-ms", str(duration_ms)])
    master, slave = os.openpty()
    pts = os.ttyname(slave)
    os.close(slave)
    proc = subprocess.Popen([str(exe), "--link-fd", str(master), *args], pass_fds=(master,))
    os.close(master)  # the model alone holds the master
    try:
        time.sleep(0.2)
        if proc.poll() is not None:
            raise SystemExit(f"the model exited at once ({proc.returncode})")
        bench = bench_record(proc, pts, exe, args, stage)
        why = model_peer.model_peer_refusal(pts, bench)
        if why is not None:
            raise SystemExit(f"the model-peer check refused the bench's own pty: {why}")
    except BaseException:  # an interrupt or a refusal while starting never leaves the model running
        if proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=5)
        raise
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "bench.json"
    path.write_text(json.dumps(bench, indent=1) + "\n", encoding="utf-8")
    return proc, path, bench


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--stage", choices=sorted(STAGE_ARGS), required=True)
    ap.add_argument("--out-dir", type=Path, default=Path("rehearsal"))
    ap.add_argument("--build", type=Path, default=DEFAULT_BUILD)
    ap.add_argument("--model-arg", action="append", default=[], help="an extra mcu_model argument (repeatable)")
    ap.add_argument("--duration-ms", type=int)
    args = ap.parse_args(argv)
    proc, path, bench = start(args.stage, args.out_dir, args.build, args.model_arg, args.duration_ms)
    print(json.dumps({"device": bench["pts"], "bench": str(path), "stage": args.stage, "model_pid": proc.pid,
                      "synthetic": True}), flush=True)
    stop = {"now": False}
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.update(now=True))
    try:
        while not stop["now"] and proc.poll() is None:
            time.sleep(0.2)
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        path.unlink(missing_ok=True)
        print(json.dumps({"model_exit": proc.returncode}), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
