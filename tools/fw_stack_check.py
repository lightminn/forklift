#!/usr/bin/env python3
"""Worst-case stack of the MCU firmware from GCC's call graphs (board design 6).

Reads every *.ci file (-fcallgraph-info=su) under a build directory and computes, for each entry
point, the deepest stack along its call graph. The worst case is one entry per preemption level
stacked on each other, plus an exception frame per interrupt level:

    main loop + level-2 handler + level-1 handler + fault/NMI handler + frames

Callees without a call graph of their own (libc, libm, libgcc helpers linked from the toolchain)
are measured from the final ELF's disassembly (--elf, --objdump): push/vpush/stmdb/sub sp/
pre-decrement stores give the frame, bl/b.w to other functions are followed. An indirect branch,
a register-sized stack adjustment or a callee found nowhere fails the check (Codex B2b review: no
assumed sizes).

Indirect calls are bounded by the deepest of the candidates given with --indirect CALLER=TARGET
(globs on function names; the first rule whose CALLER matches wins), e.g. the tick callback only
from the TIM6 handler, the fw_hal entries from everywhere else. A dynamic frame or recursion fails the check.

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
import subprocess
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


def entry(name: str, size: dict[str, int]) -> str | None:
    """An entry point given by the user: a global name, or a file-static one by its unique short
    name. Call edges never use this lookup: GCC names their targets exactly."""
    if name in size:
        return name
    matches = [t for t in size if short(t) == name]
    return matches[0] if len(matches) == 1 else None


FUNC = re.compile(r"^([0-9a-f]+) <([^>]+)>:$")
REGS = re.compile(r"\{([^}]*)\}")
CALL = re.compile(r"\b(bl|blx|b\.w|b|b\.n)\s+[0-9a-f]+ <([^>+]+)>")


def reg_count(spec: str) -> list[str]:
    regs = []
    for part in spec.replace(" ", "").split(","):
        if "-" in part:
            a, b = part.split("-")
            kind = a[0]
            regs += [kind] * (int(b[1:]) - int(a[1:]) + 1)
        elif part:
            regs.append(part[0])
    return regs


def disassembly(elf: Path, objdump: str) -> dict[str, list[str]]:
    out = subprocess.run([objdump, "-d", "--no-show-raw-insn", str(elf)], capture_output=True,
                         text=True, check=True).stdout
    funcs: dict[str, list[str]] = {}
    cur = None
    for line in out.splitlines():
        m = FUNC.match(line)
        if m:
            cur = m.group(2)
            funcs[cur] = []
        elif cur and line.strip():
            funcs[cur].append(line.split("\t", 1)[-1].strip())
    return funcs


def binary_frame(name: str, insns: list[str]) -> tuple[int, set[str]]:
    """Stack bytes a function pushes, and the functions it calls or tail-calls."""
    frame, callees = 0, set()
    for ins in insns:
        op = ins.split()[0] if ins else ""
        if op.startswith(("push", "stmdb")) and ("sp!" in ins or op.startswith("push")):
            frame += 4 * len(reg_count(REGS.search(ins).group(1)))
        elif op.startswith("vpush"):
            frame += sum(8 if r == "d" else 4 for r in reg_count(REGS.search(ins).group(1)))
        elif re.match(r"sub(\.w|w)?\s+sp,\s*(sp,\s*)?#(\d+)", ins):
            frame += int(re.search(r"#(\d+)", ins).group(1))
        elif re.search(r"\[sp, #-(\d+)\]!", ins):
            frame += int(re.search(r"\[sp, #-(\d+)\]!", ins).group(1))
        elif re.match(r"(sub|add|mov)(\.w|s)?\s+sp,\s*(sp,\s*)?r", ins) or re.match(r"(sub|mov)\S*\s+sp,\s*(ip|lr)", ins):
            raise SystemExit(f"register-sized stack adjustment in {name}: {ins}")
        elif re.match(r"(blx|bx)\s+r", ins) and not ins.startswith("bx\tlr") and not re.match(r"bx\s+lr", ins):
            raise SystemExit(f"indirect branch in {name}: {ins}")
        m = CALL.search(ins)
        if m:  # branches inside the function read <name+0x..> and do not match; a bare <name> is a
            callees.add(m.group(2))  # call or jump to its own start: recursion (Codex B2b review)
    return frame, callees


def analyse(build: Path, levels: list[list[str]], indirect: list[str], elf: Path | None = None,
            objdump: str = "arm-none-eabi-objdump",
            binary: dict[str, list[str]] | None = None) -> dict:
    """binary: the image's functions as disassembly lines (from elf, or given by a test)."""
    size, kind, calls = load(build)
    if binary is None:
        binary = disassembly(elf, objdump) if elf else {}
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

    measured: dict[str, int] = {}
    not_linked: set[str] = set()
    memo: dict[str, int] = {}

    def binary_depth(name: str, path: tuple[str, ...]) -> int:
        if name in path:
            raise SystemExit("recursion: " + " -> ".join(path + (name,)))
        if name in memo:
            return memo[name]
        if name not in binary:
            if not binary:
                raise SystemExit(f"callee without a call graph: {name} (give --elf)")
            # Named in a call graph but absent from the linked image: the final code does not call
            # it (an unresolved call would have failed the link), so it adds nothing.
            not_linked.add(name)
            return 0
        frame, callees = binary_frame(name, binary[name])
        d = frame + max((binary_depth(c, path + (name,)) if c not in size else depth(c, path)
                         for c in callees), default=0)
        memo[name] = measured[name] = d
        return d

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
            elif callee in size:
                d = depth(callee, path + (title,))
            else:
                d = binary_depth(callee, tuple(short(p) for p in path + (title,)))
            best = max(best, d)
        memo[title] = size[title] + best
        return memo[title]

    per_level = []
    for names in levels:
        worst, which = 0, None
        for n in names:
            t = entry(n, size)
            if t is None:
                raise SystemExit(f"entry point without a call graph: {n}")
            d = depth(t, ())
            if d > worst:
                worst, which = d, n
        per_level.append({"entries": names, "worst": worst, "via": which})
    frames = EXCEPTION_FRAME * (len(levels) - 1)
    total = sum(level["worst"] for level in per_level) + frames
    return {"levels": per_level, "exception_frames": frames, "total": total,
            "measured_from_binary": dict(sorted(measured.items())), "not_linked": sorted(not_linked)}


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("build", type=Path)
    ap.add_argument("--level", action="append", required=True,
                    help="comma-separated entry points of one preemption level, lowest first")
    ap.add_argument("--indirect", action="append", default=[], help="CALLER=TARGET globs for indirect calls")
    ap.add_argument("--elf", type=Path, help="final ELF, for callees without a call graph")
    ap.add_argument("--objdump", default="arm-none-eabi-objdump")
    ap.add_argument("--limit", type=int, required=True, help="stack bytes allowed in total")
    args = ap.parse_args(argv)
    report = analyse(args.build, [lv.split(",") for lv in args.level], args.indirect, args.elf,
                     args.objdump)
    report["limit"] = args.limit
    print(json.dumps(report, indent=1))
    return 0 if report["total"] <= args.limit else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
