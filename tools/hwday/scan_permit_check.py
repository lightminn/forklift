#!/usr/bin/env python3
"""11-17 (plan A4): the 7b scan permit on the real A2M12, standing, without an MCU.

The A2M12's scans (ScanFeed on the adapter's socket) go into the bridge's own ScanPermit (tools/bridge_scan.py, the
7b code path); the truck's STATEs are a standing stand-in (no MCU yet: counts and steering fixed, MCU time advancing
with the host clock, delay 0) -- so this shows the scan side of the permit, not the odometry side. Each second the
forward and reverse permits go to the timeline; tools/hwday/judge.py permit reads it. Nothing is driven.

usage: scan_permit_check.py --socket SOCK --until 70 [--t0 EPOCH_S] --mount X,Y,Z,YAW_DEG --out TIMELINE.jsonl
       (t_s in the timeline counts from --t0, the moment the wrapper script started: the user's phone timer)
       (the LiDAR on the floor or a box in an open area; the virtual truck's rear axle sits X m behind it)
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "src"))

import bridge_scan  # noqa: E402

from forklift_core.mcu_link.messages import (  # noqa: E402
    FaultFlag,
    McuState,
    Mode,
    RejectReason,
    StateFlag,
)
from forklift_core.plant.bringup import Sample  # noqa: E402
from forklift_core.plant.real_vehicle import load_real_vehicle  # noqa: E402
from forklift_core.sensors.scan_feed import ScanFeed, bind_socket  # noqa: E402

CURVS = [-0.05, 0.0, 0.05]


def standing(t_us: int) -> McuState:
    return McuState(t_us, 1, 1, 0, Mode.DISARMED, RejectReason.NONE, FaultFlag(0), 0, 0, 0.0, 0.0, 0.0, 0.0,
                    (0,) * 4, (0,) * 4, (0xC0,) * 4, 0, 0, StateFlag.DRIVERS_READY)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--socket", required=True)
    ap.add_argument("--until", type=float, default=70.0, help="stop at this t_s")
    ap.add_argument("--t0", type=float, default=None, help="wall-clock epoch seconds t_s counts from (default: now)")
    ap.add_argument("--mount", default="0.9,0,0.3,0")
    ap.add_argument("--mask", default=None)
    ap.add_argument("--out", required=True, type=Path)
    args = ap.parse_args(argv)
    veh = load_real_vehicle(REPO / "config" / "real" / "vehicle.yaml")
    sock = bind_socket(args.socket)
    feed = ScanFeed(sock=sock)
    permit = bridge_scan.ScanPermit(feed, mount=bridge_scan.parse_mount(args.mount),
                                    mask_deg=bridge_scan.parse_mask(args.mask),
                                    config_path=REPO / "config" / "real" / "obstacle_layer_real.yaml",
                                    vehicle=veh, v_max_mps=veh.teleop_speed_limit_mps)
    wall0 = time.time() if args.t0 is None else args.t0
    t0 = time.monotonic_ns() - int((time.time() - wall0) * 1e9)  # the same origin on the monotonic clock
    next_log = time.monotonic_ns()
    with args.out.open("x", encoding="utf-8") as out:
        out.write(json.dumps({"t": "header", "tool": "scan_permit_check.py", "mount": args.mount, "mask": args.mask,
                              "state": "standing stand-in, no MCU"}) + "\n")
        try:
            while (now := time.monotonic_ns()) - t0 < args.until * 1e9:
                permit.on_state(Sample(now, standing(int((now - t0) / 1000) + 1_000_000)), delay_s=0.0)  # standing
                permit.poll(now)
                if now >= next_log:
                    fwd, why_f = permit(CURVS, 1, 0.13)
                    rev, why_r = permit(CURVS, -1, 0.13)
                    row = {"t_s": round((now - t0) / 1e9, 2), "forward": round(fwd, 3), "why_f": why_f,
                           "reverse": round(rev, 3), "why_r": why_r, **permit.record_view()}
                    out.write(json.dumps(row) + "\n")
                    out.flush()
                    print(f"{row['t_s']:6.1f} s  forward {fwd:.3f} ({why_f})  reverse {rev:.3f} ({why_r})", flush=True)
                    next_log += 1_000_000_000
                time.sleep(0.02)
        finally:
            sock.close()
            Path(args.socket).unlink(missing_ok=True)
            out.write(json.dumps({"t": "footer", "status": permit.status(), "refused": dict(feed.refused)}) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
