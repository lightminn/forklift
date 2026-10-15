import { test } from "node:test";
import assert from "node:assert/strict";
import { controlProfile, frameAgreement, scanDrawable, staleness, stopInstruction, stopIsRefused } from "../js/profile.js";

test("a vehicle without the control field is a full console (console/1 as before)", () => {
  const p = controlProfile({ mission: true, teleop_direct: true, teleop_video: true, lift: true });
  assert.deepEqual(p, { control: true, mission: true, teleop: true, lift: true });
});

test("control false turns off everything that acts, whatever else is claimed", () => {
  const p = controlProfile({ control: false, mission: true, teleop_direct: true, lift: true });
  assert.deepEqual(p, { control: false, mission: false, teleop: false, lift: false });
});

test("no capabilities yet means nothing can act", () => {
  assert.deepEqual(controlProfile(null), { control: false, mission: false, teleop: false, lift: false });
});

test("teleop needs at least one teleop kind", () => {
  assert.equal(controlProfile({ teleop_direct: false, teleop_video: false }).teleop, false);
  assert.equal(controlProfile({ teleop_video: true }).teleop, true);
});

test("the picture is stale when the state or the pose behind it is older than 1 s", () => {
  assert.equal(staleness({ stateAgeS: 0.2, poseAgeS: 0.1 }).stale, false);
  const a = staleness({ stateAgeS: 1.6, poseAgeS: 0.1 });
  assert.equal(a.stale, true);
  assert.match(a.text, /1\.7 s/); // 1.6 s since received + 0.1 s old when sent
  // the pose was already 3.04 s old when sent, and that state arrived 0.2 s ago
  const b = staleness({ stateAgeS: 0.2, poseAgeS: 3.04 });
  assert.equal(b.stale, true);
  assert.match(b.text, /3\.2 s/);
  assert.equal(staleness({ stateAgeS: 0.6, poseAgeS: 0.6 }).stale, true); // 1.2 s in all
  assert.equal(staleness({ stateAgeS: null, poseAgeS: null }).stale, true);
  assert.equal(staleness({ stateAgeS: 0.1, poseAgeS: null }).stale, false); // no age reported: judged by the state alone
});

test("STOP is refused only on a connection whose own welcome said control false", () => {
  assert.equal(stopIsRefused(null), false); // reconnecting, no welcome yet: STOP is kept and resent
  assert.equal(stopIsRefused({ capabilities: { control: false } }), true);
  assert.equal(stopIsRefused({ capabilities: {} }), false);
  assert.equal(stopIsRefused({ capabilities: { control: true } }), false);
});

test("pose and scan are drawn on the map only in the map's own frame", () => {
  assert.deepEqual(frameAgreement({ gridFrame: "map", poseFrame: "map", scanFrame: "map" }), { pose: true, scan: true, note: null });
  const odom = frameAgreement({ gridFrame: "map", poseFrame: "odom", scanFrame: "odom" });
  assert.equal(odom.pose, false);
  assert.equal(odom.scan, false);
  assert.match(odom.note, /odom/);
  // no grid yet: whatever frame the pose has, it is drawn alone (nothing to misalign with)
  assert.deepEqual(frameAgreement({ gridFrame: null, poseFrame: "odom", scanFrame: "odom" }), { pose: true, scan: true, note: null });
  // a scan in another frame than the pose is never drawn next to it
  assert.equal(frameAgreement({ gridFrame: null, poseFrame: "odom", scanFrame: "map" }).scan, false);
});

test("scan points older than 1 s are not drawn", () => {
  assert.equal(scanDrawable({ ageS: 0.3 }), true);
  assert.equal(scanDrawable({ ageS: 1.2 }), false);
  assert.equal(scanDrawable(null), false);
});

test("the 'cannot stop' notice tells the method the vehicle named, never an assumed e-stop", () => {
  assert.equal(stopInstruction({ capabilities: { stop_instruction: "주 전원 스위치를 끄세요" } }), "주 전원 스위치를 끄세요");
  // a link that names nothing gets a neutral line, not a device that may not exist
  assert.equal(stopInstruction({ capabilities: {} }), "차량에서 직접 멈추는 수단을 쓰세요");
  assert.equal(stopInstruction(null), "차량에서 직접 멈추는 수단을 쓰세요");
});
