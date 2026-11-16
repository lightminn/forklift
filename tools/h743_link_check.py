#!/usr/bin/env python3
"""PC <-> MCU mcu-link/3 check on a bare WeAct H743 board (plan docs/plans/2026-11-17-h743-board-check.md).

Listens to STATE for ``--seconds`` and sends one DISARM every ``--disarm-every-s``: DISARM is always accepted
(contract 3.1) and sets STATE.last_rx_seq, so the time from writing it to the first STATE that names its seq is the
command-to-STATE round trip. DISARM never drives anything. Nothing else is sent (no ARM, no CLEAR_FAULTS).

Every STATE, DISARM and round trip goes to ``--out`` (JSON lines); the last line is the summary with the verdicts,
also printed as one line. Exit 0 when every criterion passes, 1 otherwise, 2 on a usage error.

usage: h743_link_check.py --device /dev/forklift-mcu --out data/h743_check/2026-11-17_link.jsonl [--seconds 600]
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from real_mcu_link import LinkLost, TtyLink, open_raw  # noqa: E402

from forklift_core.mcu_link.messages import VERSION, Disarm, LinkDecoder, McuState, encode  # noqa: E402

STATE_PERIOD_MS = 10.0  # the firmware's state_period_ms (board design 5)
# Criteria, fixed before the run (plan 4). School scale: they catch a broken link, not microseconds.
MIN_STATE_FRACTION = 0.95  # STATEs received / (seconds x 100)
MCU_PERIOD_MS = (9.0, 11.0)  # every MCU-time step between adjacent STATEs: a dropped STATE shows as 20 ms
MCU_PERIOD_MEAN_TOL_MS = 0.05
HOST_GAP_MAX_MS = 100.0  # the executor treats an older STATE as stale (HostConfig.state_max_age_s = 0.1)
RTT_MAX_MS = 50.0  # one 10 ms STATE period plus USB both ways, with margin; the DRIVE validity cap is 150 ms
RTT_TIMEOUT_S = 1.0  # a DISARM not seen in a STATE within this counts as lost


def _pct(xs: list[float], q: float) -> float | None:
    if not xs:
        return None
    s = sorted(xs)
    return s[min(len(s) - 1, int(round(q * (len(s) - 1))))]


def _write_all(fd: int, data: bytes, deadline: float) -> None:
    off = 0
    while off < len(data):
        try:
            off += os.write(fd, data[off:])
        except (BlockingIOError, InterruptedError):
            if time.monotonic() > deadline:
                raise LinkLost("write stalled")
            time.sleep(0.001)


def summarize(states: list[dict], rtts: list[float], lost: int, sent: int, decoder_errors: int, seconds: float,
              elapsed_s: float, link_lost: str | None) -> dict:
    seq_gaps = 0
    mcu_steps, host_steps = [], []
    boots = []
    for a, b in zip(states, states[1:]):
        if b["boot_id"] != a["boot_id"]:
            continue
        seq_gaps += (b["seq"] - a["seq"] - 1) % 0x10000
        mcu_steps.append((b["mcu_time_us"] - a["mcu_time_us"]) / 1000.0)
        host_steps.append((b["host_ns"] - a["host_ns"]) / 1e6)
    for s in states:
        if not boots or boots[-1] != s["boot_id"]:
            boots.append(s["boot_id"])
    first, last = (states[0], states[-1]) if states else (None, None)
    rx_err = None if not states else (last["rx_errors"] - first["rx_errors"]) % 0x10000
    overruns = None if not states else (last["cycle_overruns"] - first["cycle_overruns"]) % 0x10000
    mean_mcu = statistics.fmean(mcu_steps) if mcu_steps else None
    checks = {
        "link_kept": link_lost is None and elapsed_s >= seconds - 1.0,
        "states_flowing": len(states) >= MIN_STATE_FRACTION * seconds * 1000.0 / STATE_PERIOD_MS,
        "no_frame_errors": decoder_errors == 0,  # CRC, COBS, length and version (decode refuses anything not v3)
        "no_seq_gaps": seq_gaps == 0,
        "mcu_period": bool(mcu_steps) and MCU_PERIOD_MS[0] <= min(mcu_steps) and max(mcu_steps) <= MCU_PERIOD_MS[1]
        and abs(mean_mcu - STATE_PERIOD_MS) <= MCU_PERIOD_MEAN_TOL_MS,
        "host_gap": bool(host_steps) and max(host_steps) <= HOST_GAP_MAX_MS,
        "round_trip": sent > 0 and lost == 0 and bool(rtts) and max(rtts) <= RTT_MAX_MS,
        "no_reset": len(boots) == 1,  # a watchdog (or any) reset changes boot_id
        "mcu_rx_errors_zero": rx_err == 0,
        "no_cycle_overrun": overruns == 0,
    }
    return {
        "t": "summary",
        "version": VERSION,
        "seconds": round(elapsed_s, 1),
        "states": len(states),
        "decoder_errors": decoder_errors,
        "seq_gaps": seq_gaps,
        "mcu_period_ms": None if not mcu_steps else
        {"mean": round(mean_mcu, 4), "min": round(min(mcu_steps), 3), "max": round(max(mcu_steps), 3)},
        "host_interval_ms": None if not host_steps else
        {"mean": round(statistics.fmean(host_steps), 3), "max": round(max(host_steps), 3)},
        "round_trip_ms": None if not rtts else
        {"p50": round(_pct(rtts, 0.5), 2), "p95": round(_pct(rtts, 0.95), 2), "max": round(max(rtts), 2)},
        "disarm_sent": sent,
        "disarm_lost": lost,
        "boot_ids": [f"0x{b:08x}" for b in boots],
        "reset_cause": None if first is None else first["reset_cause"],
        "flags": None if last is None else last["flags"],
        "mode": None if last is None else last["mode"],
        "fault_flags": None if last is None else last["fault_flags"],
        "mcu_rx_errors_delta": rx_err,
        "cycle_overruns_delta": overruns,
        "link_lost": link_lost,
        "checks": checks,
        "result": "PASS" if all(checks.values()) else "FAIL",
    }


def one_line(s: dict) -> str:
    failed = [k for k, v in s["checks"].items() if not v]
    m, h, r = s["mcu_period_ms"] or {}, s["host_interval_ms"] or {}, s["round_trip_ms"] or {}
    return (f"{s['result']} v{s['version']} {s['seconds']}s states={s['states']} seq_gaps={s['seq_gaps']} "
            f"frame_errors={s['decoder_errors']} mcu_period={m.get('mean')}/{m.get('max')}ms "
            f"host_gap_max={h.get('max')}ms rtt_p50/max={r.get('p50')}/{r.get('max')}ms "
            f"disarm_lost={s['disarm_lost']}/{s['disarm_sent']} resets={len(s['boot_ids']) - 1 if s['boot_ids'] else '?'}"
            + (f" FAILED={','.join(failed)}" if failed else ""))


def run(device: str, out: Path, seconds: float, disarm_every_s: float, baud: int | None, fd: int | None = None) -> dict:
    """``fd``: an already open endpoint instead of ``device`` (tests: a pty master)."""
    out.parent.mkdir(parents=True, exist_ok=True)
    if fd is None:
        fd = open_raw(device, baud)
    link = TtyLink(fd)
    dec = LinkDecoder()
    states: list[dict] = []
    rtts: list[float] = []
    pending: dict[int, int] = {}  # DISARM seq -> write time (ns)
    lost = sent = 0
    seq = 0
    link_lost = None
    t0 = time.monotonic()
    next_disarm = t0 + 0.5
    with out.open("w", encoding="utf-8") as f:
        f.write(json.dumps({"t": "start", "device": device, "seconds": seconds, "host_ns": time.monotonic_ns(),
                            "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}) + "\n")
        try:
            _write_all(fd, b"\x00", time.monotonic() + 1.0)  # close any half frame the MCU holds (board design 5)
            while time.monotonic() - t0 < seconds:
                now = time.monotonic()
                if now >= next_disarm:
                    seq = (seq + 1) % 0x10000
                    t_w = time.monotonic_ns()
                    _write_all(fd, encode(Disarm(), seq), now + 1.0)
                    pending[seq] = t_w
                    sent += 1
                    f.write(json.dumps({"t": "disarm", "seq": seq, "host_ns": t_w}) + "\n")
                    next_disarm += disarm_every_s
                data = link.read_available()
                recv_ns = time.monotonic_ns()
                for r in dec.feed(data, recv_ns):
                    if not isinstance(r.message, McuState):
                        continue
                    s = r.message
                    row = {"t": "state", "host_ns": recv_ns, "seq": r.seq, "mcu_time_us": s.mcu_time_us,
                           "boot_id": s.boot_id, "last_rx_seq": s.last_rx_seq, "mode": int(s.mode),
                           "reject_reason": int(s.reject_reason), "fault_flags": int(s.fault_flags),
                           "rx_errors": s.rx_errors, "cycle_overruns": s.cycle_overruns, "flags": int(s.flags),
                           "reset_cause": int(s.reset_cause)}
                    states.append(row)
                    f.write(json.dumps(row) + "\n")
                    t_w = pending.pop(s.last_rx_seq, None)
                    if t_w is not None:
                        rtt = (recv_ns - t_w) / 1e6
                        rtts.append(rtt)
                        f.write(json.dumps({"t": "rtt", "seq": s.last_rx_seq, "ms": round(rtt, 3)}) + "\n")
                for k in [k for k, t_w in pending.items() if recv_ns - t_w > RTT_TIMEOUT_S * 1e9]:
                    del pending[k]
                    lost += 1
                    f.write(json.dumps({"t": "rtt_lost", "seq": k}) + "\n")
                if not data:
                    time.sleep(0.0005)
        except LinkLost as exc:
            link_lost = str(exc)
            f.write(json.dumps({"t": "link_lost", "host_ns": time.monotonic_ns(), "why": link_lost}) + "\n")
        finally:
            link.close()
        # still unanswered at the end: one written within the last RTT_TIMEOUT_S may be in flight and is not
        # judged; older ones are lost
        end_ns = time.monotonic_ns()
        for t_w in pending.values():
            if end_ns - t_w > RTT_TIMEOUT_S * 1e9:
                lost += 1
            else:
                sent -= 1
        summary = summarize(states, rtts, lost, sent, dec.errors, seconds, time.monotonic() - t0, link_lost)
        f.write(json.dumps(summary) + "\n")
    return summary


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--device", required=True)
    p.add_argument("--out", required=True, type=Path)
    p.add_argument("--seconds", type=float, default=600.0)
    p.add_argument("--disarm-every-s", type=float, default=1.0)
    p.add_argument("--baud", type=int, default=None, help="UART7 only (460800); USB CDC takes none")
    a = p.parse_args(argv)
    if a.seconds <= 0 or a.disarm_every_s <= 0:
        p.error("--seconds and --disarm-every-s must be positive")
    s = run(a.device, a.out, a.seconds, a.disarm_every_s, a.baud)
    print(one_line(s))
    print(f"record: {a.out}")
    return 0 if s["result"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
