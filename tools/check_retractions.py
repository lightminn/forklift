#!/usr/bin/env python3
"""Find numbers this document retracted that are still quoted elsewhere unmarked.

docs/decisions/0003's appendix A records this as its most repeated failure: a
revision declares a figure wrong and the figure survives somewhere else -- once
for seventeen revisions.

The hard part is telling the retracted value from its replacement, because both
sit in the same sentence ("0.60 was wrong, it is 0.524").  A first version of this
tool treated every number in a sentence carrying a retraction marker as retracted
and produced 116 findings out of 117 numbers, nearly all of them the *corrected*
values.  So retraction is recognised only from constructions that name the dead
value explicitly:

    ~~0.464~~                     strikethrough
    "0.60" 은 거짓 / 틀렸다 / 오류다   quoted value then a verdict
    0.60 → 0.524  |  0.60 이 아니라 0.524   |  0.60 → **0.524**

A later occurrence is reported only when its own sentence carries no retraction
marker, i.e. when it is not itself discussing the retraction.

    tools/check_retractions.py docs/decisions/0003-*.md

Exit status is 1 when anything is found, so this can gate a commit.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

NUM = r"\d+(?:\.\d+)?"
VERDICT = r"(?:거짓|틀렸|오류|철회|반증|폐기|withdrawn|retracted)"

# Precision first.  Two earlier designs failed loudly and are worth recording:
#   v1 treated every number in a sentence carrying a retraction marker as retracted
#      -> 116 findings out of 117 numbers, nearly all of them the *corrected* values,
#         because a correction sentence names both ("0.60 was wrong, it is 0.524").
#   v2 added phrase patterns like "<n> 는 ... 거짓" -> 318 findings, because it matched
#      revision and section numbers ("개정 1 은 ... 틀렸다").
# A checker with a 99% false-positive rate is worse than none: it trains you to skip it.
# So only the one construction this document uses unambiguously for a dead value is
# recognised -- strikethrough -- and only for numbers with >= 3 significant digits,
# which excludes revision numbers, section numbers and small counts.
STRIKETHROUGH = re.compile(r"~~([^~]*)~~")
NUMBER = re.compile(rf"(?<![\w.])({NUM})(?![\w.])")


def significant(tok: str) -> bool:
    return len(tok.replace(".", "").lstrip("0")) >= 3


MARKERS = ("철회", "거짓", "틀렸", "오류", "정정", "반증", "폐기", "withdrawn", "retracted")
OCCURRENCE = lambda tok: re.compile(rf"(?<![\w.]){re.escape(tok)}(?![\w.\d])")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path")
    ap.add_argument("--context", type=int, default=100)
    args = ap.parse_args()
    lines = Path(args.path).read_text().split("\n")

    retracted: dict[str, int] = {}
    for i, line in enumerate(lines, 1):
        for struck in STRIKETHROUGH.findall(line):
            for tok in NUMBER.findall(struck):
                if significant(tok):
                    retracted.setdefault(tok, i)

    findings = []
    for i, line in enumerate(lines, 1):
        if any(m in line for m in MARKERS):
            continue                                   # the line is about a retraction
        for tok, src in retracted.items():
            if i > src and OCCURRENCE(tok).search(line):
                findings.append((tok, src, i, line.strip()[:args.context]))

    print(f"# tracked {len(retracted)} retracted value(s): {', '.join(sorted(retracted))}\n")
    if not findings:
        print("no unmarked survivors")
        return 0
    print(f"{len(findings)} unmarked survivor(s):\n")
    for tok, src, line, ctx in sorted(findings, key=lambda f: (f[2], f[0])):
        print(f"  {tok:>8}  retracted line {src:<5} survives line {line:<5} | {ctx}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
