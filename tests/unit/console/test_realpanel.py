"""console/web/js/realpanel.js (5c-1b, real safety wiring design 5.1): the real operator panel's decisions."""

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
NODE = shutil.which("node")
DRIVER = """
import * as P from "file://%s";
const st = (permit, mission, extra = {}) => ({t: "real_status", session_epoch: 3, permit: {state: permit},
  mission: {status: mission}, bypassed: [], ...extra});
const out = {};
out.shown = [P.panelShown({t: "real_status"}), P.panelShown({t: "state"}), P.panelShown(null)];
const reg = {session_epoch: 3, revision: 5};
out.rev = [P.submitRevision(reg, st("OFF", "idle", {targets_revision: 5})),
           P.submitRevision(reg, st("OFF", "idle", {targets_revision: 6})),
           P.submitRevision(reg, {...st("OFF", "idle", {targets_revision: 5}), session_epoch: 4}),
           P.submitRevision(null, st("OFF", "idle", {targets_revision: 5}))];
out.why = [P.closedWhy(st("OFF", "idle"), true), P.closedWhy(st("DRIVING", "running"), true),
           P.closedWhy(st("LATCHED", "running"), true), P.closedWhy(st("OFF", "idle"), false), P.closedWhy(null, true)];
out.submitWhy = [P.submitClosedWhy(st("OFF", "idle"), true, false), P.submitClosedWhy(st("LATCHED", "idle"), true, false),
                 P.submitClosedWhy(st("OFF", "idle", {mode: "STOPPED"}), true, false),
                 P.submitClosedWhy(st("OFF", "idle"), true, true)];
const gens = {session_epoch: 3, lease_id: "L1"};
out.mine = [P.statusIsMine(st("OFF", "idle", {lease: {holder: "me"}}), gens, "me"),
            P.statusIsMine(st("OFF", "idle", {lease: {holder: "other"}}), gens, "me"),
            P.statusIsMine({...st("OFF", "idle", {lease: {holder: "me"}}), session_epoch: 2}, gens, "me"),
            P.statusIsMine(st("OFF", "idle", {lease: {holder: "me"}}), {session_epoch: 3, lease_id: null}, "me")];
out.shownToggle = [P.toggleShown({check: "imu", on: true}, st("OFF", "idle", {bypassed: ["imu"]})),
                   P.toggleShown({check: "imu", on: false}, st("OFF", "idle", {bypassed: ["imu"]})),
                   P.toggleShown({check: "imu", on: false}, st("OFF", "idle"))];
const b = P.bypassButtons(st("OFF", "idle", {bypassed: ["imu"]}), true);
out.buttons = b.map((x) => [x.check, x.bypassed, x.enabled]);
out.closed = P.bypassButtons(st("READY", "running"), true).every((x) => !x.enabled && x.why);
const s7 = st("LATCHED", "running", {operator_request: {kind: "lifted", serial: 7, phase: "lift"}});
const key = P.answerKey(s7);
out.key = key;
out.open = [P.answerStillOpen(key, s7),
            P.answerStillOpen(key, st("LATCHED", "running", {operator_request: {kind: "lifted", serial: 8}})),
            P.answerStillOpen(key, {...s7, session_epoch: 4}),
            P.answerStillOpen(key, st("LATCHED", "running")), P.answerStillOpen(null, s7)];
out.pose = [P.poseOf("3", "0.5", "90"), P.poseOf("x", 0, 0), P.poseOf(1, 2, Infinity)];
console.log(JSON.stringify(out));
"""

pytestmark = pytest.mark.skipif(NODE is None, reason="needs Node to run console/web/js")


def test_the_panel_decisions(tmp_path):
    script = tmp_path / "drive.mjs"
    script.write_text(DRIVER % (ROOT / "console/web/js/realpanel.js").as_posix())
    out = json.loads(subprocess.run([NODE, str(script)], capture_output=True, text=True, check=True).stdout)
    assert out["shown"] == [True, False, False]
    assert out["rev"] == [5, None, None, None]  # a newer vehicle list or another session: register again
    assert out["submitWhy"][0] is None and all(isinstance(w, str) for w in out["submitWhy"][1:])
    assert out["mine"] == [True, False, False, False]
    assert out["shownToggle"] == [True, False, True]
    assert out["why"][0] is None and all(isinstance(w, str) for w in out["why"][1:])
    assert ["imu", True, True] in out["buttons"] and ["slam", False, True] in out["buttons"]
    assert len(out["buttons"]) == 9 and out["closed"] is True
    assert out["key"] == {"session_epoch": 3, "kind": "lifted", "serial": 7}
    assert out["open"] == [True, False, False, False, False]
    assert out["pose"][0][:2] == [3, 0.5] and abs(out["pose"][0][2] - 1.5707963) < 1e-6
    assert out["pose"][1] is None and out["pose"][2] is None


def test_the_panels_bypass_list_is_the_vehicles():
    """The console's list (realpanel.js BYPASSABLE) and the loop's (real_loop.BYPASSABLE) stay the same set."""
    import re

    from forklift_core.live.real_loop import BYPASSABLE

    text = (ROOT / "console/web/js/realpanel.js").read_text()
    js = re.search(r"export const BYPASSABLE = \[(.*?)\];", text, re.S).group(1)
    assert set(re.findall(r'"([a-z_]+)"', js)) == set(BYPASSABLE)
