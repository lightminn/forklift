"""console/web/js/sendgap.js: the console's teleop send-gap meter (R2-3 (b)) -- recent p99 and max over a window, a
warning past 0.2 s, reset on disarm, and the page state each teleop carries."""

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
NODE = shutil.which("node")
DRIVER = """
import { createSendGapMeter, pageReport } from "file://%s";
const steps = JSON.parse(process.argv[2]);
const m = createSendGapMeter({ windowMs: 10000, warnMs: 200 });
const out = [];
for (const s of steps) {
  if (s.op === "send") out.push({ gap: m.onSend(s.now) });
  else if (s.op === "reset") m.reset();
  else if (s.op === "stats") out.push(m.stats(s.now));
  else if (s.op === "page") out.push(pageReport(s.vis, s.focus, s.gap));
  else if (s.op === "pending") out.push({ pending: m.pending(s.now) });
}
console.log(JSON.stringify(out));
"""

pytestmark = pytest.mark.skipif(NODE is None, reason="needs Node to run console/web/js/sendgap.js")


def run(steps, tmp_path):
    script = tmp_path / "drive.mjs"
    script.write_text(DRIVER % (ROOT / "console/web/js/sendgap.js").as_posix())
    done = subprocess.run([NODE, str(script), json.dumps(steps)], capture_output=True, text=True, check=True)
    return json.loads(done.stdout)


def test_gaps_p99_max_and_the_warning(tmp_path):
    t = [0, 33, 66, 99, 340, 373]  # one 241 ms gap
    out = run([{"op": "send", "now": x} for x in t] + [{"op": "stats", "now": 400}], tmp_path)
    gaps = [o["gap"] for o in out[:-1]]
    assert gaps == [None, 33, 33, 33, 241, 33]
    st = out[-1]
    assert st["count"] == 5 and st["max_ms"] == 241 and st["warn"] is True and st["p99_ms"] == 241


def test_the_window_forgets_old_gaps_and_a_reset_starts_over(tmp_path):
    # a 300 ms gap that ends at t = 300, then steady 33 ms sends up to t = 11,055: at 11,100 the window (last 10 s of
    # gap ends) holds only 33 ms gaps
    sends = [0, 300] + list(range(333, 11056, 33))
    steps = [{"op": "send", "now": x} for x in sends] + [{"op": "stats", "now": 11100}, {"op": "reset"},
                                                         {"op": "send", "now": 20000}, {"op": "stats", "now": 20001}]
    out = run(steps, tmp_path)
    st = out[len(sends)]
    assert st["max_ms"] == 33 and st["warn"] is False
    assert out[-1] == {"count": 0, "p99_ms": None, "max_ms": None, "warn": False}
    assert out[len(sends) + 1]["gap"] is None  # after a reset the first send has no gap


def test_pending_says_the_gap_without_recording_it(tmp_path):
    out = run([{"op": "pending", "now": 5}, {"op": "send", "now": 10}, {"op": "pending", "now": 60},
               {"op": "stats", "now": 60}], tmp_path)
    assert out[0] == {"pending": None} and out[2] == {"pending": 50} and out[3]["count"] == 0


def test_the_page_report_is_what_the_bridge_records(tmp_path):
    out = run([{"op": "page", "vis": "hidden", "focus": False, "gap": 1008.4},
               {"op": "page", "vis": "visible", "focus": True, "gap": None}], tmp_path)
    assert out[0] == {"vis": "hidden", "focus": False, "gap_ms": 1008.4}
    assert out[1] == {"vis": "visible", "focus": True, "gap_ms": None}
