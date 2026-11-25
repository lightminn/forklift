#!/usr/bin/env python3
"""S2 first-boot check (arrival-day procedure 6.1): reads a bring-up record and judges the first seconds of the boot
-- drivers, current zero, fault bits, KTH axes, still wheels, link -- so nobody reads the JSON lines by hand.
Read only: it opens the record file, never the device.

usage: h743_first_boot.py --record artifacts/<run>/run.jsonl [--window-s 10] [--lift-absent] [--steer-absent]
       [--allow-synthetic] [--allow-bypassed]
(the absent parts are read from the record header -- ``bringup.py record --absent`` writes them when it captures; a
flag here must agree with a header that declares them. A steering-absent result is no gate pass: read it with
--allow-bypassed)

Prints one JSON line (result pass / fail / invalid / synthetic, each check with its numbers). Exit 0 on pass, 1 on
fail or invalid, 2 on a refused or unreadable record."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from forklift_core.live.synthetic import SyntheticRefused, refuse_or_mark
from forklift_core.plant import judgement
from forklift_core.plant.first_boot import WINDOW_S, first_boot_check
from forklift_core.plant.record_rows import sample_from_row

V3_FIELDS = ("kth_frames", "kth_crc_errors", "kth_alt_crc_errors")


def v3_row_problem(row: dict) -> str | None:
    """A STATE row's v3 KTH counters: present, two each, 16-bit unsigned integers (as bringup.py kth reads them)."""
    for name in V3_FIELDS:
        v = row.get(name)
        if not (isinstance(v, list) and len(v) == 2
                and all(isinstance(x, int) and not isinstance(x, bool) and 0 <= x <= 0xFFFF for x in v)):
            return f"{name} missing or malformed in a STATE row (a v3 record is needed)"
    return None


def _absent_parts(rows: list[dict], rmarks: dict, args) -> list[str]:
    """The parts judged absent: the record header's (written at capture) with this run's flags; a flag naming a part
    a header that declares its parts does not name is refused (partial-parts plan v4: the capture is the source)."""
    flags = {p for p, on in (("lift", args.lift_absent), ("steer", args.steer_absent)) if on}
    header = set(rmarks.get("parts_absent", []))
    if any(isinstance(r.get("parts_absent"), list) for r in rows if r.get("t") == "header") and flags - header:
        raise judgement.NotForGate(f"--{sorted(flags - header)[0]}-absent disagrees with the record header's "
                                   f"parts_absent {sorted(header)}")
    return sorted(header | flags)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--record", required=True, help="a bringup.py record (run.jsonl) of the board's first boot")
    ap.add_argument("--window-s", type=float, default=WINDOW_S, help="seconds from the first STATE to judge")
    ap.add_argument("--lift-absent", action="store_true",
                    help="no lift KTH7801 is fitted yet (lift method undecided, plan 3.5): the lift axis is not judged")
    ap.add_argument("--steer-absent", action="store_true",
                    help="no steering KTH7801 is fitted (partial-parts bench): the steering axis is not judged and "
                         "the result is no gate pass")
    ap.add_argument("--allow-synthetic", action="store_true", help="read a rehearsal record (never a pass)")
    ap.add_argument("--allow-bypassed", action="store_true",
                    help="read a record with a bypassed protection or unknown run conditions (never a pass)")
    args = ap.parse_args(argv)
    try:
        rows = [json.loads(x) for x in Path(args.record).read_text(encoding="utf-8").splitlines() if x.strip()]
    except (OSError, ValueError) as exc:
        print(json.dumps({"result": "refused", "why": f"cannot read the record: {exc}"}))
        return 2
    try:
        marks = refuse_or_mark(rows, allow=args.allow_synthetic)
        rmarks = judgement.record_marks(rows)
        absent = _absent_parts(rows, rmarks, args)
        if absent:
            rmarks["parts_absent"] = absent
            if any(p not in judgement.GATE_NEUTRAL_ABSENT for p in absent):
                rmarks["not_for_gate"] = True
        # design 2026-11-25-bench-commission-limits 1.2 (PR-2b): a bypassed or unknown-condition run is no gate
        jmarks = judgement.check(rmarks, allow=args.allow_bypassed)
    except (SyntheticRefused, judgement.NotForGate) as exc:
        print(json.dumps({"result": "refused", "why": str(exc)}))
        return 2
    if sum(1 for r in rows if r.get("t") == "header") > 1:  # bringup.py appends: one run per file (Codex P2-3)
        print(json.dumps({**marks, "result": "invalid",
                          "why": "several runs appended in one file: record the first boot to a new file"}))
        return 1
    states = [r for r in rows if r.get("t") == "state"]
    bad = next((why for r in states if (why := v3_row_problem(r)) is not None), None)
    if bad is not None:
        print(json.dumps({**marks, "result": "invalid", "why": bad}))
        return 1
    try:
        samples = [sample_from_row(r) for r in states]
    except (KeyError, TypeError, ValueError) as exc:
        print(json.dumps({**marks, "result": "invalid", "why": f"a malformed STATE row: {exc!r}"}))
        return 1
    absent = jmarks.get("parts_absent", [])
    res = first_boot_check(samples, window_s=args.window_s, lift_absent="lift" in absent,
                           steer_absent="steer" in absent, synthetic_input=bool(marks))
    if not judgement.eligible(jmarks):
        res = {**res, "computed_result": res["result"], "result": "not_for_gate"}  # S2 reads result and exit 0
    print(json.dumps({**marks, **jmarks, **res}))
    return 0 if res["result"] == "pass" else 1


if __name__ == "__main__":
    sys.exit(main())
