#!/usr/bin/env python3
"""Worst-case stack of the MCU firmware from GCC's call graphs (board design 6).

Reads every *.ci file (-fcallgraph-info=su) under a build directory and computes, for each entry
point, the deepest stack along its call graph. The worst case is one entry per preemption level
stacked on each other, plus an exception frame per interrupt level:

    main loop + level-2 handler + level-1 handler + fault/NMI handler + frames

Indirect calls are bounded by the deepest of the candidates given with --indirect CALLER=TARGET
(globs on function names; the first rule whose CALLER matches wins), e.g. the tick callback only
from the TIM6 handler, the fw_hal entries from everywhere else. Functions without a call graph of their own (libc,
libm) count as --unknown bytes each and are listed. A dynamic frame or recursion fails the check.

  python tools/fw_stack_check.py BUILD_DIR --limit 4096 \\
      --level board_main --level TIM6_DAC_IRQHandler --level SPI1_IRQHandler,SPI2_IRQHandler \\
      --level NMI_Handler,HardFault_Handler,Default_Handler \
      --indirect 'TIM6_DAC_IRQHandler=on_tick' --indirect '*=hal_*'
"""
from __future__ import annotations

import argparse
import fnmatch
import json
import re
import sys
from pathlib import Path

NODE = re.compile(r'node: \{ title: "([^"]+)" label: "([^"]*)"')
EDGE = re.compile(r'edge: \{ sourcename: "([^"]+)" targetname: "([^"]+)"')
SIZE = re.compile(r"\\n(\d+) bytes \(([a-z,]+)\)")
EXCEPTION_FRAME = 108  # FPU lazy-stacking frame 104 B + 4 B alignment padding


def short(title: str) -> str:
    return title.rsplit(":", 1)[-1]


def load(build: Path):
    size: dict[str, int] = {}
    kind: dict[str, str] = {}
    calls: dict[str, set[str]] = {}
    for ci in build.rglob("*.ci"):
        text = ci.read_text()
        for title, label in NODE.findall(text):
            m = SIZE.search(label)
            if m:
                size[title] = int(m.group(1))
                kind[title] = m.group(2)
        for src, dst in EDGE.findall(text):
            calls.setdefault(src, set()).add(dst)
    return size, kind, calls


def resolve(name: str, size: dict[str, int]) -> str | None:
    """A callee title: global names as is; a file-static one by its unique short name."""
    if name in size:
        return name
    matches = [t for t in size if short(t) == name]
    return matches[0] if len(matches) == 1 else None


def analyse(build: Path, levels: list[list[str]], indirect: list[str], unknown: int) -> dict:
    size, kind, calls = load(build)
    for title, k in kind.items():
        if "dynamic" in k:
            raise SystemExit(f"dynamic stack frame: {short(title)}")
    rules = []
    for rule in indirect:
        caller, _, target = rule.partition("=")
        rules.append((caller, [t for t in size if fnmatch.fnmatch(short(t), target)]))

    def indirect_targets(caller: str) -> list[str]:
        for pattern, targets in rules:
            if fnmatch.fnmatch(short(caller), pattern):
                return targets
        return []

    unknowns: set[str] = set()
    memo: dict[str, int] = {}

    def depth(title: str, path: tuple[str, ...]) -> int:
        if title in path:
            raise SystemExit("recursion: " + " -> ".join(short(p) for p in path + (title,)))
        if title in memo:
            return memo[title]
        best = 0
        for callee in calls.get(title, ()):
            if callee == "__indirect_call":
                targets = indirect_targets(title)
                if not targets:
                    raise SystemExit(f"indirect call in {short(title)} without --indirect targets")
                d = max(depth(t, path + (title,)) for t in targets)
            else:
                t = resolve(callee, size)
                if t is None:
                    unknowns.add(callee)
                    d = unknown
                else:
                    d = depth(t, path + (title,))
            best = max(best, d)
        memo[title] = size[title] + best
        return memo[title]

    per_level = []
    for names in levels:
        worst, which = 0, None
        for n in names:
            t = resolve(n, size)
            if t is None:
                raise SystemExit(f"entry point without a call graph: {n}")
            d = depth(t, ())
            if d > worst:
                worst, which = d, n
        per_level.append({"entries": names, "worst": worst, "via": which})
    frames = EXCEPTION_FRAME * (len(levels) - 1)
    total = sum(level["worst"] for level in per_level) + frames
    return {"levels": per_level, "exception_frames": frames, "total": total,
            "unknown_callees": sorted(unknowns), "unknown_bytes_each": unknown}


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("build", type=Path)
    ap.add_argument("--level", action="append", required=True,
                    help="comma-separated entry points of one preemption level, lowest first")
    ap.add_argument("--indirect", action="append", default=[], help="CALLER=TARGET globs for indirect calls")
    ap.add_argument("--unknown", type=int, default=256, help="bytes for a callee without a graph")
    ap.add_argument("--limit", type=int, required=True, help="stack bytes allowed in total")
    args = ap.parse_args(argv)
    report = analyse(args.build, [lv.split(",") for lv in args.level], args.indirect, args.unknown)
    report["limit"] = args.limit
    print(json.dumps(report, indent=1))
    return 0 if report["total"] <= args.limit else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
