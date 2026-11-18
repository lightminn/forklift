#!/usr/bin/env python3
"""S2 first-boot check (arrival-day procedure 6.1): reads a bring-up record and judges the first seconds of the boot
-- drivers, current zero, fault bits, KTH axes, still wheels, link -- so nobody reads the JSON lines by hand.
Read only: it opens the record file, never the device.

usage: h743_first_boot.py --record artifacts/<run>/run.jsonl [--window-s 10] [--lift-absent] [--allow-synthetic]

Prints one JSON line (result pass / fail / invalid / synthetic, each check with its numbers). Exit 0 on pass, 1 on
fail or invalid, 2 on a refused or unreadable record."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from forklift_core.live.synthetic import SyntheticRefused, refuse_or_mark
from forklift_core.plant.first_boot import WINDOW_S, first_boot_check
from forklift_core.plant.record_rows import sample_from_row

V3_FIELDS = ("kth_frames", "kth_crc_errors", "kth_alt_crc_errors")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--record", required=True, help="a bringup.py record (run.jsonl) of the board's first boot")
    ap.add_argument("--window-s", type=float, default=WINDOW_S, help="seconds from the first STATE to judge")
    ap.add_argument("--lift-absent", action="store_true",
                    help="no lift KTH7801 is fitted yet (lift method undecided, plan 3.5): the lift axis is not judged")
    ap.add_argument("--allow-synthetic", action="store_true", help="read a rehearsal record (never a pass)")
    args = ap.parse_args(argv)
    try:
        rows = [json.loads(x) for x in Path(args.record).read_text(encoding="utf-8").splitlines() if x.strip()]
    except (OSError, ValueError) as exc:
        print(json.dumps({"result": "refused", "why": f"cannot read the record: {exc}"}))
        return 2
    try:
        marks = refuse_or_mark(rows, allow=args.allow_synthetic)
    except SyntheticRefused as exc:
        print(json.dumps({"result": "refused", "why": str(exc)}))
        return 2
    states = [r for r in rows if r.get("t") == "state"]
    if any(name not in r for r in states for name in V3_FIELDS):
        print(json.dumps({**marks, "result": "invalid", "why": "a STATE row without the v3 KTH counters"}))
        return 1
    res = first_boot_check([sample_from_row(r) for r in states], window_s=args.window_s,
                           lift_absent=args.lift_absent, synthetic_input=bool(marks))
    print(json.dumps({**marks, **res}))
    return 0 if res["result"] == "pass" else 1


if __name__ == "__main__":
    sys.exit(main())
