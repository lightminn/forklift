#!/usr/bin/env python3
"""DIAGNOSTIC ONLY (branch diag/kth-raw, never merged): read a `bringup.py record` taken on the diag-kthraw image,
whose STATE kth_alt_crc_errors slot carries the newest raw KTH SPI word per axis (M = steering CS1 PD0, H = lift
CS2 PD1). Prints, per axis, the frames counted, the standard-CRC failures, how many distinct raw words were seen
(sampled at the STATE rate), the most common ones in hex with their bits, and the angle/CRC split.

    python tools/kth_raw.py RUN.jsonl --product "<the board's USB product string, udevadm ATTRS{product}>"

The product string must end in the token ``kthraw`` (the flag itself, not the tag): on any other image the slot is
the alternative-CRC counter, and reading it as raw words would be wrong (Codex diag check P2-2).
"""

from __future__ import annotations

import collections
import json
import sys


def main(path: str, product: str) -> int:
    if not product or product.split()[-1:] != ["kthraw"]:
        print(f"refused: the product string {product!r} does not end in 'kthraw' -- not the raw-word image")
        return 2
    rows = [json.loads(line) for line in open(path, encoding="utf-8")]
    st = [r for r in rows if r.get("t") == "state"]
    if not st:
        print("no STATE rows")
        return 1
    print(f"states {len(st)} modes {sorted({r.get('mode') for r in st})} boots {len({r.get('boot_id') for r in st})}")
    for a, name in ((0, "M steering CS1 PD0"), (1, "H lift CS2 PD1")):
        frames = (st[-1]["kth_frames"][a] - st[0]["kth_frames"][a]) % 65536
        crc = (st[-1]["kth_crc_errors"][a] - st[0]["kth_crc_errors"][a]) % 65536
        raw = collections.Counter(int(r["kth_alt_crc_errors"][a]) for r in st)
        print(f"{name}: frames {frames}, standard CRC failures {crc}, distinct raw words {len(raw)}")
        for word, n in raw.most_common(8):
            print(f"  0x{word:04X}  {word:016b}  angle12 {word >> 4:4d}  crc4 0x{word & 0xF:X}  x{n}")
    return 0


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("record")
    ap.add_argument("--product", required=True)
    a = ap.parse_args()
    sys.exit(main(a.record, a.product))
