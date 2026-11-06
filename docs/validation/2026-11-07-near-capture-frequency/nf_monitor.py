#!/usr/bin/env python3
"""Blind monitor of the near-capture frequency batch (docs/plans/2026-11-07-near-capture-frequency.md section 2).

Prints how many runs ended before any submission, their names, and how many ended otherwise. It reads the bundle's
results.jsonl and only the existence of a run's submission traces; it never prints a status, a reason or a count of
missions, so nothing about the batch's results reaches the reader before the tally tool is merged.

usage: nf_monitor.py BATCH_DIR
"""
import glob
import json
import os
import sys

STARTUP_FLAGS = ("startup_silent", "startup_viewport", "startup_crash")


def has_traces(run_dir):
    """Any sign the session took a mission: the probe's report with a submission, or a mission's folder."""
    if glob.glob(os.path.join(run_dir, "live", "epoch_*", "run", "missions", "m*")):
        return True
    report = os.path.join(run_dir, "probe.json")
    if not os.path.isfile(report):
        return False
    try:
        return "submit" in open(report, encoding="utf-8").read()
    except OSError:
        return True


def main(batch):
    last = {}
    for path in sorted(glob.glob(os.path.join(batch, "b*", "results.jsonl"))):
        for line in open(path, encoding="utf-8"):
            row = json.loads(line)
            if row.get("name"):
                last[row["name"]] = row
    retry, other = [], 0
    for name, row in sorted(last.items()):
        startup = row.get("status") == "start_failed" or any(row.get(k) for k in STARTUP_FLAGS)
        if startup and not has_traces(os.path.join(batch, name)):
            retry.append(name)
        else:
            other += 1
    print(f"ended before submission {len(retry)}, other ended {other}, runs listed {len(last)}")
    for name in retry:
        print("retry", name)


if __name__ == "__main__":
    main(sys.argv[1])
