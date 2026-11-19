"""console/web/js/realstatus.js and protocol.js (5c-1, real safety wiring design 5): real_status updates the control
generation and the lease holder of this session only; the bypass banner lists the bypassed checks."""

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
NODE = shutil.which("node")
DRIVER = """
import { applyRealStatus, bypassBanner } from "file://%s";
import { createSession } from "file://%s";
const out = [];
out.push(applyRealStatus({t: "real_status", session_epoch: 3, control_gen: 9, lease: {holder: "me"}}, 3, "me"));
out.push(applyRealStatus({t: "real_status", session_epoch: 2, control_gen: 9, lease: {holder: "me"}}, 3, "me"));
out.push(applyRealStatus({t: "real_status", session_epoch: 3, control_gen: 4, lease: {holder: null}}, 3, "me"));
out.push(bypassBanner({bypassed: []}));
out.push(bypassBanner({bypassed: ["imu", "layer_timing", "zzz"]}));
out.push(bypassBanner(null));
const sent = [];
const s = createSession({ clientId: "me", send: (m) => sent.push(m), now: () => 0, newId: () => "id" + sent.length });
s.onMessage({t: "welcome", session_epoch: 3, api: "console/1", request_token: 5});
s.onMessage({t: "ack", of: "lease_request", request_id: "x", lease_id: "L1"});
s.onMessage({t: "real_status", session_epoch: 3, control_gen: 7, lease: {holder: "me"}, bypassed: ["imu"]});
out.push({gen: s.generations.control_gen, lease: s.generations.lease_id, bypassed: s.realStatus.bypassed});
s.onMessage({t: "real_status", session_epoch: 3, control_gen: 8, lease: {holder: null}, bypassed: []});
out.push({gen: s.generations.control_gen, lease: s.generations.lease_id});
s.onMessage({t: "real_status", session_epoch: 1, control_gen: 99, lease: {holder: "me"}, bypassed: ["slam"]});
out.push({gen: s.generations.control_gen, bypassed: s.realStatus.bypassed});
console.log(JSON.stringify(out));
"""

pytestmark = pytest.mark.skipif(NODE is None, reason="needs Node to run console/web/js")


def test_real_status_and_the_bypass_banner(tmp_path):
    script = tmp_path / "drive.mjs"
    script.write_text(DRIVER % ((ROOT / "console/web/js/realstatus.js").as_posix(),
                                (ROOT / "console/web/js/protocol.js").as_posix()))
    out = json.loads(subprocess.run([NODE, str(script)], capture_output=True, text=True, check=True).stdout)
    assert out[0] == {"accepted": True, "controlGen": 9, "leaseHeld": True}
    assert out[1] == {"accepted": False}
    assert out[2]["leaseHeld"] is False
    assert out[3] == "" and out[5] == ""
    assert out[4] == "우회 중: IMU, 층 시각, zzz"
    assert out[6] == {"gen": 7, "lease": "L1", "bypassed": ["imu"]}
    assert out[7] == {"gen": 8, "lease": None}
    assert out[8] == {"gen": 8, "bypassed": []}  # another session's status is not taken
