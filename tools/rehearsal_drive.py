#!/usr/bin/env python3
"""The arrival-day rehearsal's G6 drive (docs/design/2026-11-15-arrival-day-rehearsal.md 2.3-2.4): the teleop bridge
on the rehearsal bench's model MCU with a drive profile, and an operator stand-in doing what the console page does.

The bridge runs as the procedure runs it (``--device <bench pty> --bench … --synthetic-permit --record … --profile …``
and the four operator confirmations) but on a pty of this script's, so Enter can start the trial as on the
bridge's own terminal. The operator stand-in (tools/live_console_probe.Probe over console/1): lease, mode TELEOP,
arm, wait for ``ready``, then teleop messages with the deadman held (ratios 0: the profile rewrites them) until the
trial ends, then the deadman let go. Every reply and the bridge's output go to ``OUT/drive.json``. Synthetic: the
model's plant is not the truck; the record is marked and refused by measurement tools without --allow-synthetic.

usage: rehearsal_drive.py --bench REH/bench.json --profile P.yaml --out OUTDIR [--timeout-s 300]
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import select
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "src"))

from live_console_probe import Probe  # noqa: E402

CONFIRMATIONS = ("--empty-forks", "--forks-clear", "--lift-low", "--no-open-recovery")


class BridgeOnPty:
    """The bridge process with its stdin/stdout on a pty this script holds (Enter = a newline written)."""

    def __init__(self, argv: list[str]) -> None:
        self.master, slave = os.openpty()
        self.proc = subprocess.Popen(argv, stdin=slave, stdout=slave, stderr=slave, close_fds=True,
                                     start_new_session=True)
        os.close(slave)
        self.buf = b""
        self.lines: list[str] = []

    def pump(self) -> None:
        while select.select([self.master], [], [], 0)[0]:
            try:
                chunk = os.read(self.master, 65536)
            except OSError:
                return
            if not chunk:
                return
            self.buf += chunk
            *done, self.buf = self.buf.split(b"\n")
            self.lines += [x.decode(errors="replace").rstrip("\r") for x in done]

    def json_line(self, key: str):
        self.pump()
        for line in self.lines:
            try:
                d = json.loads(line)
            except ValueError:
                continue
            if isinstance(d, dict) and key in d:
                return d
        return None

    def enter(self) -> None:
        os.write(self.master, b"\n")

    def stop(self) -> int:
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait()
        self.pump()
        return self.proc.returncode


def profile_end(record: Path):
    """The trial's end: the ``profile_end`` row the G6 design names, or -- rehearsal defect D1, the bridge does not
    write that row -- the first ``profile`` row carrying a result (``found_by`` says which)."""
    if not record.exists():
        return None
    first_result = None
    for line in record.read_text(encoding="utf-8").splitlines():
        try:
            row = json.loads(line)
        except ValueError:
            continue
        if row.get("t") == "profile_end":
            return {**row, "found_by": "profile_end"}
        if first_result is None and row.get("t") == "profile" and row.get("result") is not None:
            first_result = {"trial": row.get("trial"), "result": row["result"], "host_ns": row.get("host_ns"),
                            "found_by": "profile_row_result"}
    return first_result


async def operate(port: int, bridge: BridgeOnPty, record: Path, timeout_s: float, report: dict) -> None:
    p = Probe(port, report)
    p.client_id = "rehearsal-operator"
    await p.open()
    ok = await p.until(lambda s: True, 5.0, label="first_state")
    lease = await p.request("lease_request")
    report["lease"] = lease
    if not lease or lease.get("t") != "ack":
        report["result"] = "no_lease"
        return
    p.lease_id, p.control_gen = lease["lease_id"], lease.get("control_gen", p.control_gen)
    await p.until(lambda s: s.get("stopped_confirmed"), 5.0, label="standing")
    mode = await p.request("mode_request", to="TELEOP")
    report["mode"] = mode
    arm = await p.request("arm")
    report["arm"] = arm
    if not arm or arm.get("t") != "ack":
        report["result"] = "not_armed"
        return
    p.arm_gen = arm.get("arm_gen", p.arm_gen)
    hold = asyncio.create_task(_hold_deadman(p))
    try:
        report["armed_state"] = await p.until(lambda s: s.get("armed"), 10.0, label="armed")
        await asyncio.sleep(1.5)  # the profile starts after a standstill of STILL_START_S
        bridge.enter()
        end = time.monotonic() + timeout_s
        while time.monotonic() < end and profile_end(record) is None:
            await asyncio.sleep(0.2)
        report["profile_end"] = profile_end(record)
        report["profile_start"] = bridge.json_line("profile_start")
    finally:
        hold.cancel()
        for _ in range(20):  # the deadman let go: the runner leaves its zeroing state
            await p.teleop(0.0, 0.0, deadman=False)
            await asyncio.sleep(0.05)
    report["result"] = "done" if report.get("profile_end") else "no_profile_end"


async def _hold_deadman(p: Probe) -> None:
    while True:
        await p.teleop(0.0, 0.0, deadman=True)
        await asyncio.sleep(0.05)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--bench", required=True)
    ap.add_argument("--profile", required=True)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--timeout-s", type=float, default=300.0)
    args = ap.parse_args(argv)
    bench = json.loads(Path(args.bench).read_text(encoding="utf-8"))
    args.out.mkdir(parents=True, exist_ok=True)
    record = args.out / "teleop.jsonl"
    argv_ = [sys.executable, str(REPO / "tools" / "teleop_bridge.py"), "--device", bench["pts"], "--bench",
             str(args.bench), "--synthetic-permit", "--record", str(record), "--profile", str(args.profile),
             "--port", "0", *CONFIRMATIONS]
    bridge = BridgeOnPty(argv_)
    report: dict = {"tool": "rehearsal_drive.py", "synthetic": True, "bridge_argv": argv_}
    try:
        console = None
        deadline = time.monotonic() + 20
        while console is None and time.monotonic() < deadline and bridge.proc.poll() is None:
            console = bridge.json_line("console")
            time.sleep(0.1)
        report["console"] = console
        if console is None:
            report["result"] = "bridge_did_not_start"
        else:
            port = int(console["console"].rstrip("/").rsplit(":", 1)[1])
            asyncio.run(operate(port, bridge, record, args.timeout_s, report))
    finally:
        report["bridge_exit"] = bridge.stop()
        report["bridge_output"] = bridge.lines[-40:]
        (args.out / "drive.json").write_text(json.dumps(report, indent=1, default=str) + "\n", encoding="utf-8")
    print(json.dumps({k: report.get(k) for k in ("result", "profile_end", "bridge_exit")}, default=str))
    return 0 if report.get("result") == "done" else 1


if __name__ == "__main__":
    sys.exit(main())
