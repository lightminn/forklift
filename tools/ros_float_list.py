#!/usr/bin/env python3
"""A comma list of numbers for a ROS list parameter (``-p "name:=[...]"``), every value a float as ROS parses it.

rclpy refuses a list mixing integers and floats ("Sequence should be of same type"): the online-SLAM S2 run a_1
(2026-11-13, job 18809) passed the start pose "-2.612,0,0" and its bridge never started. Each value is written in
fixed point with a decimal point and no exponent (an exponent without a point need not read as a float), up to
15 decimals, trailing zeros dropped; a value that is not a finite number, or a wrong count, is refused.

    python tools/ros_float_list.py COUNT "x,y,yaw"     # prints the list, or exits 2 with the reason
"""

from __future__ import annotations

import math
import sys


def _fixed(x: float) -> str:
    text = f"{x:.15f}".rstrip("0")
    return text + "0" if text.endswith(".") else text


def float_list(text: str, *, count: int) -> str:
    parts = [p.strip() for p in str(text).split(",")]
    if len(parts) != count or any(p == "" for p in parts):
        raise ValueError(f"{text!r}: {count} comma-separated numbers expected")
    values = []
    for p in parts:
        try:
            v = float(p)
        except ValueError:
            raise ValueError(f"{text!r}: {p!r} is not a number") from None
        if not math.isfinite(v):
            raise ValueError(f"{text!r}: {p!r} is not finite")
        values.append(v)
    return ",".join(_fixed(v) for v in values)


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    try:
        print(float_list(argv[1], count=int(argv[0])))
    except ValueError as exc:
        print(f"ros_float_list: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
