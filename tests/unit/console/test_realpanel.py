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
           P.submitRevision(null, st("OFF", "idle", {targets_revision: 5})),
           P.submitRevision(reg, st("OFF", "idle", {targets_revision: 5}), {targets_revision: 6})];
out.why = [P.closedWhy(st("OFF", "idle"), true), P.closedWhy(st("DRIVING", "running"), true),
           P.closedWhy(st("LATCHED", "running"), true), P.closedWhy(st("OFF", "idle"), false), P.closedWhy(null, true)];
out.submitWhy = [P.submitClosedWhy(st("OFF", "idle"), true, false), P.submitClosedWhy(st("LATCHED", "idle"), true, false),
                 P.submitClosedWhy(st("OFF", "idle", {mode: "STOPPED"}), true, false),
                 P.submitClosedWhy(st("OFF", "idle"), true, true),
                 P.submitClosedWhy(st("OFF", "idle", {restart_required: "RuntimeError:x"}), true, false)];
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
const sA = st("OFF", "idle", {lease: {holder: "me"}});
out.sig = [P.statusSig({...sA, control_gen: 4}) === P.statusSig({...sA, control_gen: 4, permit: {state: "OFF", reason: "x"}}),
           P.statusSig({...sA, control_gen: 4}) === P.statusSig({...sA, control_gen: 5}),  // a second STOP: another
           P.statusSig(sA) === P.statusSig(st("OFF", "idle", {lease: {holder: "me"}, operator_request: {kind: "lifted", serial: 2}}))];
out.pose = [P.poseOf("3", "0.5", "90"), P.poseOf("x", 0, 0), P.poseOf(1, 2, Infinity)];
// 5c-2 (design 6, 8-9)
const rs = st("OFF", "idle", {relocalize_generation: 2, standing: true,
  relocalize: {generation: 2, attempt: 3, kind: "confirm", state: "refused", reason: "not_at_start",
               start_pose_m: 0.153, start_pose_deg: 1.2}});
out.rview = P.relocalizeView(rs);
out.rwhy = [P.relocalizeClosedWhy(rs, true), P.relocalizeClosedWhy({...rs, standing: false}, true),
            P.relocalizeClosedWhy({...rs, relocalize_generation: null}, true),
            P.relocalizeClosedWhy(st("READY", "running", {relocalize_generation: 2, standing: true}), true),
            P.relocalizeClosedWhy(rs, false)];
const ack = {session_epoch: 3, generation: 2, attempt: 3};
out.rresult = [P.relocalizeResult(ack, rs), P.relocalizeResult({...ack, attempt: 2}, rs),
               P.relocalizeResult(ack,
                 {...rs, relocalize_generation: null, relocalize: {...rs.relocalize, state: "confirmed", reason: null}}),
               P.relocalizeResult({...ack, session_epoch: 2}, rs),  // another session's ack
               P.relocalizeResult(ack, {...rs, relocalize_generation: 3})];  // a newer generation open
out.rgapNew = P.relocalizeView({...rs, relocalize_generation: 3}).gap;
out.rsig = P.statusSig(rs) === P.statusSig({...rs, relocalize_generation: 3});
// PR 7 design 6.1, 2: the manual-only notice reads the latch and the bypass, not the SLAM state
const latch = {reason: "not_localized", session_epoch: 3};
out.manual = [P.manualOnlyNotice(st("OFF", "idle")), P.manualOnlyNotice(st("OFF", "idle", {manual_only: latch,
              manual_only_blocking: true})), P.manualOnlyNotice(st("OFF", "idle", {manual_only: latch,
              manual_only_blocking: false, bypassed: ["slam"]})), P.manualOnlyNotice(null)];
console.log(JSON.stringify(out));
"""

pytestmark = pytest.mark.skipif(NODE is None, reason="needs Node to run console/web/js")


def test_the_panel_decisions(tmp_path):
    script = tmp_path / "drive.mjs"
    script.write_text(DRIVER % (ROOT / "console/web/js/realpanel.js").as_posix())
    out = json.loads(subprocess.run([NODE, str(script)], capture_output=True, text=True, check=True).stdout)
    assert out["shown"] == [True, False, False]
    assert out["rev"] == [5, None, None, None, None]  # a newer list (either message) or another session: again
    assert out["submitWhy"][0] is None and all(isinstance(w, str) for w in out["submitWhy"][1:])
    assert "다시 시작" in out["submitWhy"][4]  # 6a: a failed mission collection asks for a restart
    assert out["mine"] == [True, False, False, False]
    assert out["shownToggle"] == [True, False, True]
    assert out["why"][0] is None and all(isinstance(w, str) for w in out["why"][1:])
    assert ["imu", True, True] in out["buttons"] and ["slam", False, True] in out["buttons"]
    assert len(out["buttons"]) == 12 and out["closed"] is True  # start_pose: 5c-2, d5_pocket: 5d, f1: 7c-2b
    assert out["key"] == {"session_epoch": 3, "kind": "lifted", "serial": 7}
    assert out["open"] == [True, False, False, False, False]
    assert out["pose"][0][:2] == [3, 0.5] and abs(out["pose"][0][2] - 1.5707963) < 1e-6
    assert out["pose"][1] is None and out["pose"][2] is None
    assert out["sig"] == [True, False, False]
    assert out["rview"]["open"] and out["rview"]["generation"] == 2 and out["rview"]["gap"] == "15.3 cm · 1.2°"
    assert out["manual"][0] == "" and out["manual"][3] == ""
    assert out["manual"][1].startswith("위치 추정 실패 — 자율은 세션을 다시 시작해야 함")
    assert out["manual"][2] == "우회 중: slam(not_localized)"
    assert out["rwhy"][0] is None and all(isinstance(w, str) for w in out["rwhy"][1:])
    assert out["rresult"] == [{"state": "refused", "reason": "not_at_start"}, None,
                              {"state": "confirmed", "reason": None}, None, None]  # told after the close only
    assert out["rgapNew"] == ""  # generation 2's gap not under generation 3
    assert out["rsig"] is False  # a press drawn for generation 2 is not taken for 3


def test_the_panels_bypass_list_is_the_vehicles():
    """The console's list (realpanel.js BYPASSABLE) and the loop's (real_loop.BYPASSABLE) stay the same set."""
    import re

    from forklift_core.live.real_loop import BYPASSABLE

    text = (ROOT / "console/web/js/realpanel.js").read_text()
    js = re.search(r"export const BYPASSABLE = \[(.*?)\];", text, re.S).group(1)
    assert set(re.findall(r'"([a-z0-9_]+)"', js)) == set(BYPASSABLE)  # d5_pocket has a digit


U3C_DRIVER = """
import * as P from "file://%s";
const base = {t: "real_status", session_epoch: 3, permit: {state: "OFF"}, mission: {status: "idle"}, bypassed: [],
  targets_revision: 5};
const pre = {passed: ["C1", "C2"], load: {lift_target_m: 0.204, test_pallet: "P-S10", pallet_mass_kg: 4.56},
  runtime: [{id: "C7", item: "imu", state: "bypassed", why: "imu:unqualified"},
            {id: "C8", item: "d5_pocket", state: "blocked", why: "d5_pocket:no_depth_source"},
            {id: "C8", item: "pallet_confirm", state: "pass", why: null},
            {id: "C5", item: "scan", state: "note", why: "frame:stale"},
            {id: "C5", item: "scan", state: "note", why: null},
            {id: "C9", item: "odd", state: "weird", why: null}]};
const out = {};
out.rows = P.preconditionRows({...base, preconditions: pre}, (r) => `<${r}>`);
out.noRows = [P.preconditionRows(base), P.preconditionRows(null), P.preconditionRows({...base, preconditions: null})];
const d5 = {...base, register_serial: 2, still_serial: 7, preconditions: pre};
out.texts = P.palletConfirmTexts(d5);
out.noTexts = [P.palletConfirmTexts(base), P.palletConfirmTexts(null)];
out.textsNoLoad = P.palletConfirmTexts({...d5, preconditions: {...pre, load: null}});
const reg = {session_epoch: 3, revision: 5, pallet: "P-S10", pose: [1, 2, 0], serial: 2};
out.fields = [P.startFields(reg, d5), P.startFields(reg, base), P.startFields(null, base),
              P.startFields({...reg, serial: 1}, d5), P.startFields({...reg, revision: 4}, d5),
              P.startFields({...reg, session_epoch: 2}, d5), P.startFields(null, d5),
              P.startFields({...reg, serial: null}, d5), P.startFields(reg, null),
              P.startFields(reg, {...d5, still_serial: null})];  // not still: nothing to confirm
out.state = [P.startStateText({...d5, operator_start: true, operator_start_binding:
               {pallet_id: "P-S10", register_serial: 2, region: [1, 2, 1.8, 0.8, Math.PI / 2]}}),
             P.startStateText({...base, operator_start: true}), P.startStateText({...base, operator_start: false})];
out.sig = P.statusSig(d5) === P.statusSig({...d5, register_serial: 3});
out.newerList = [P.startFields(reg, d5, {targets_revision: 6}), P.startFields(reg, d5, {targets_revision: 5})];
const rr = {pallet_id: "P-S10", register_serial: 2, targets_revision: 5, region: [1, 2, 1.8, 0.8, 0]};
const d5r = {...d5, register_region: rr};
out.region = [P.confirmRegion(reg, d5r), P.confirmRegion({...reg, serial: 1}, d5r),
              P.confirmRegion({...reg, pallet: "Q"}, d5r), P.confirmRegion(reg, d5), P.confirmRegion(null, d5r),
              P.confirmRegion(reg, {...d5r, targets_revision: 6}),   // the list moved on since the registration
              P.confirmRegion({...reg, session_epoch: 2}, d5r),
              P.confirmRegion(reg, d5r, {targets_revision: 6}),     // the targets message is newer
              P.confirmRegion(reg, {...d5r, relocalize_generation: 3}),  // a relocalization open
              P.confirmRegion(reg, {...d5r, still_serial: null})];      // not standing
out.regionText = P.regionText([1, 2, 1.8, 0.8, 0]);
const g = {session_epoch: 3, lease_id: "L1"};
const k0 = P.checksKey(d5r, g, reg, out.texts);
out.keys = [k0 === P.checksKey(d5r, g, reg, out.texts),
            k0 === P.checksKey(d5r, {...g, lease_id: "L2"}, reg, out.texts),          // another lease
            k0 === P.checksKey(d5r, g, {...reg, pose: [1.5, 2, 0]}, out.texts),       // another registered pose
            k0 === P.checksKey({...d5r, relocalize_generation: 4}, g, reg, out.texts), // a relocalization
            k0 === P.checksKey({...d5r, still_serial: 8}, g, reg, out.texts),         // moved and stood again
            P.checksKey(d5r, g, null, out.texts)];
out.wait = [P.confirmWaitText(reg, d5r), P.confirmWaitText(reg, {...d5r, relocalize_generation: 3}),
            P.confirmWaitText(reg, {...d5r, still_serial: null}), P.confirmWaitText(null, d5r),
            P.confirmWaitText(reg, base)];
const opb = (st) => Object.fromEntries(P.bypassButtons(st, true).map((b) => [b.check, b.enabled]));
out.opButtons = [opb({...d5, operating: true}), opb({...base, operating: false})];
console.log(JSON.stringify(out));
"""


def test_the_u3c_precondition_rows_and_pallet_confirmation(tmp_path):
    """U3c (operating plan 2.6, 2.9): the rows' colours and words, the six lines, the start request's fields."""
    script = tmp_path / "u3c.mjs"
    script.write_text(U3C_DRIVER % (ROOT / "console/web/js/realpanel.js").as_posix())
    out = json.loads(subprocess.run([NODE, str(script)], capture_output=True, text=True, check=True).stdout)
    assert [(r["cls"], r["text"]) for r in out["rows"]] == [
        ("pass", "조립 통과: C1 · C2"),
        ("bypassed", "우회 중: C7 IMU (기울기 감시 꺼짐)"),
        ("blocked", "C8 D5 포켓 막힘 · <d5_pocket:no_depth_source>"),
        ("pass", "C8 팔레트 확인 통과"),
        ("note", "C5 스캔 · 최근 거부: <frame:stale>"),
        ("note", "C5 스캔 · 거부 없음"),
        ("blocked", "C9 odd 막힘"),  # a state this console does not know reads as blocked
    ]
    assert out["noRows"] == [[], [], []]
    assert out["texts"] == ["포크가 포켓 밖에 있음", "포켓 둘이 비어 있음", "팔레트 윗면과 등록 사각형 안에 다른 물체 없음",
                            "승강 목표 0.204 m + 팔레트 높이까지 위 공간 비어 있음", "빈 포크·승강 내림",
                            "C11 에서 승인한 시험 팔레트 P-S10, 질량 4.56 kg 이하"]  # as approved, not rounded
    assert out["noTexts"] == [None, None]
    assert out["textsNoLoad"][3] == "승강 목표 — m + 팔레트 높이까지 위 공간 비어 있음"
    assert out["textsNoLoad"][5] == "C11 에서 승인한 시험 팔레트 —, 질량 — kg 이하"
    assert out["fields"][0] == {"pallet_id": "P-S10", "targets_revision": 5, "register_serial": 2, "still_serial": 7}
    assert out["fields"][1] == {} and out["fields"][2] == {}  # no per-pallet confirmation: as before
    assert out["fields"][3:] == [None] * 7  # another serial, revision or session, none registered, no status, moving
    assert out["state"][0] == "확인됨 · P-S10 · 등록 #2 · 영역 중심 (1.00, 2.00) m · 1.80×0.80 m · 90.0°"
    assert out["state"][1:] == ["확인됨", ""]
    assert out["sig"] is False  # a press drawn for one registration is not taken for the next
    assert out["newerList"] == [None, {"pallet_id": "P-S10", "targets_revision": 5, "register_serial": 2,
                                       "still_serial": 7}]
    assert out["region"] == [[1, 2, 1.8, 0.8, 0]] + [None] * 9  # only this console's current registration
    assert out["regionText"].endswith("중심 (1.00, 2.00) m · 1.80×0.80 m · 0.0°")
    assert out["keys"] == [True, False, False, False, False, ""]
    assert out["wait"][0] == "" and out["wait"][1].startswith("재위치가 끝나면") and out["wait"][2].startswith("차가 서면")
    assert out["wait"][3].startswith("팔레트를 (다시) 등록") and out["wait"][4] == ""  # no confirmation on this run  # the ticks are bound to the lease, pose and generation
    assert out["opButtons"][0]["layer_timing"] is False and out["opButtons"][0]["operator_start"] is False
    assert out["opButtons"][0]["imu"] is True and out["opButtons"][1]["layer_timing"] is True
    assert out["opButtons"][1]["operator_start"] is True
