import { test } from "node:test";
import assert from "node:assert/strict";
import { cameraHeightM, controlProfile, rejectText, disarmAsked, forbidText, sessionChanges, videoStaleAfterS, frameAgreement, palletKindLabel, permitText, rearmState, resumeState, resumeSteps, scanDrawable, staleness, standWhy, stopInstruction, stopIsRefused, stopKind } from "../js/profile.js";

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

test("resume steps read the same from a list (fake vehicle) or a map (live runner)", () => {
  const list = [{ name: "stop_confirmed", result: "ok" }, { name: "state_verified", result: "failed", reason: "load_changed" }];
  assert.deepEqual(resumeSteps(list), list);
  assert.deepEqual(
    resumeSteps({ stop_confirmed: "ok", state_verified: "failed(load_changed)", frame_reexpressed: "pending" }),
    [
      { name: "stop_confirmed", result: "ok", reason: null },
      { name: "state_verified", result: "failed", reason: "load_changed" },
      { name: "frame_reexpressed", result: "pending", reason: null },
    ],
  );
  assert.equal(resumeSteps(null), null);
});

test("rearm is offered once stopped with the lease; an unconfirmed stop says so instead", () => {
  assert.equal(rearmState({ mode: "STOPPED", stopped_confirmed: true }, true).enabled, true);
  assert.equal(rearmState({ mode: "STOPPED", stopped_confirmed: false }, true).enabled, false);
  assert.match(rearmState({ mode: "STOPPED", stopped_confirmed: false }, true).why, /정지 확인/);
  assert.equal(rearmState({ mode: "STOPPED" }, true).enabled, true); // a vehicle that does not say: let it judge
  assert.equal(rearmState({ mode: "STOPPED", stopped_confirmed: true }, false).enabled, false);
  assert.equal(rearmState({ mode: "PAUSED", stopped_confirmed: true }, true).shown, false);
});

test("stop kinds read the same from the fake vehicle's names and the live arbiter's", () => {
  assert.deepEqual(stopKind("controlled"), stopKind("controlled_decel"));
  assert.deepEqual(stopKind("preempt"), stopKind("safety_preempt"));
  assert.equal(stopKind("preempt").level, "crit");
  assert.equal(stopKind("controlled").level, "warn");
  assert.equal(stopKind("none").level, "");
  assert.match(stopKind(null).label, /—/); // unknown, not "none"
  assert.equal(stopKind("brand_new").label, "brand_new"); // an unknown name is shown as sent
});

test("a pallet kind named after its geometry file reads as the pallet's name", () => {
  assert.equal(palletKindLabel("pallet_geometry_epal6"), "EPAL 6");
  assert.equal(palletKindLabel("pallet_geometry_t11_06"), "T11 ×0.6");
  assert.equal(palletKindLabel("pallet_geometry_other"), "other");
  assert.equal(palletKindLabel("EUR"), "EUR");
  assert.equal(palletKindLabel(undefined), "종류 모름");
});

test("TELEOP entry and rearm take a lower-only stand waiver; a resume does not", () => {
  const st = { mode: "STOPPED", stopped_confirmed: false, stand_waiver: "lower_only" };
  assert.equal(standWhy(st, { waivable: true }), null);
  assert.match(standWhy(st), /정지 확인/);
  assert.equal(rearmState(st, true).enabled, true);
  const mi = { status: "paused" };
  assert.equal(resumeState({ state: { ...st, mode: "PAUSED" }, mission: mi, hasLease: true }).enabled, false);
});

test("requests that need the truck standing wait for stopped_confirmed", () => {
  assert.match(standWhy({ stopped_confirmed: false }), /정지 확인/);
  assert.equal(standWhy({ stopped_confirmed: true }), null);
  assert.equal(standWhy({}), null); // a vehicle that does not say judges the request itself
});

test("resume is offered for a paused or held mission once the truck stands", () => {
  const st = { mode: "PAUSED", stopped_confirmed: true };
  const mi = { status: "paused" };
  assert.equal(resumeState({ state: st, mission: mi, hasLease: true }).enabled, true);
  const moving = resumeState({ state: { ...st, stopped_confirmed: false }, mission: mi, hasLease: true });
  assert.equal(moving.enabled, false);
  assert.match(moving.why, /정지 확인/);
  assert.equal(resumeState({ state: st, mission: mi, hasLease: false }).enabled, false);
  assert.equal(resumeState({ state: { ...st, mode: "STOPPED" }, mission: mi, hasLease: true }).enabled, false);
  assert.equal(resumeState({ state: { ...st, mode: "TELEOP" }, mission: { status: "held" }, hasLease: true }).enabled, false);
  assert.equal(resumeState({ state: st, mission: { status: "running" }, hasLease: true }).enabled, false);
  assert.equal(resumeState({ state: st, mission: null, hasLease: true }).enabled, false);
});

test("a permit whose limit is unknown is shown, not dropped", () => {
  assert.equal(permitText(null), "판정 없음");
  assert.equal(permitText({ reason: "clear", limit_mps: 0.3 }), "clear · 상한 0.30 m/s");
  assert.equal(permitText({ reason: "lift", limit_mps: null }), "lift · 상한 모름");
});

test("every reason the live arbiter gives for refusing manual driving reads in Korean", () => {
  for (const r of ["fork_in_pocket", "near_section", "near_section_unknown", "state_unknown", "pacing",
    "no_feedback:rear_axle_speed", "state_forbids_teleop", "stopped_locked"]) {
    assert.doesNotMatch(forbidText(r), /^[a-z_:]+$/, r);
  }
  assert.match(forbidText("fork_in_pocket"), /포켓/);
  assert.match(forbidText("no_feedback:rear_axle_speed"), /rear_axle_speed/); // the missing feedback is named
  assert.equal(forbidText("brand_new"), "brand_new");
  assert.equal(forbidText(null), "");
});

test("the video-stalled notice waits as long as the vehicle's own limit scale: 1/f in a slowed simulation", () => {
  assert.equal(videoStaleAfterS({ pacing: { target_factor: 0.08 } }), 12.5);
  assert.equal(videoStaleAfterS({ pacing: { target_factor: 1 } }), 1);
  assert.equal(videoStaleAfterS({}), 1); // a real truck: wall time
  assert.equal(videoStaleAfterS({ pacing: { target_factor: 0 } }), 1);
  assert.equal(videoStaleAfterS(null), 1);
});

test("losing the lease and a disarm the page did not ask for are logged", () => {
  const me = "c1";
  const held = { session_epoch: 5, lease: { holder: me }, armed: true };
  const lost = sessionChanges(held, { ...held, lease: { holder: null }, armed: false }, { clientId: me, askedDisarm: false });
  assert.equal(lost.length, 2);
  assert.match(lost[0].text, /조종 권한/);
  assert.match(lost[1].text, /조종 준비/);
  // a disarm this page asked for is not news
  const asked = sessionChanges(held, { ...held, armed: false }, { clientId: me, askedDisarm: true });
  assert.deepEqual(asked, []);
  // nothing changed, or no previous state
  assert.deepEqual(sessionChanges(held, held, { clientId: me }), []);
  assert.deepEqual(sessionChanges(null, held, { clientId: me }), []);
  // a new session is announced by the reconnect itself
  assert.deepEqual(sessionChanges(held, { session_epoch: 6, lease: { holder: null }, armed: false }, { clientId: me }), []);
});

test("a camera on the carriage is drawn from the lift height; unknown lift, no overlay", () => {
  // Codex C3 b review P2: lifted 0.20 m, the floor overlay was ~97 px off
  const model = { base_from_camera: { xyz_m: [0.959, 0, 0.27] }, rides_lift: true };
  assert.equal(cameraHeightM(model, { lift_m: 0.2 }), 0.47);
  assert.equal(cameraHeightM(model, { lift_m: null }), null);
  assert.equal(cameraHeightM(model, {}), null);
  const fixed = { base_from_camera: { xyz_m: [0.7, 0, 1.05] } };
  assert.equal(cameraHeightM(fixed, { lift_m: 0.2 }), 1.05); // a mast camera does not move with the lift
});

test("a disarm the page asked for is excused once, and only in the arm period it was asked in", () => {
  // Codex C3 b review P3: ask, see it disarmed, re-arm, then a vehicle disarm within 3 s was hidden
  const asked = { at: 1000, armGen: 4 };
  assert.equal(disarmAsked(asked, { arm_gen: 4 }, 2000), true);
  assert.equal(disarmAsked(asked, { arm_gen: 5 }, 2000), false); // re-armed since
  assert.equal(disarmAsked(asked, { arm_gen: 4 }, 4500), false); // too long ago
  assert.equal(disarmAsked(null, { arm_gen: 4 }, 2000), false);
});

test("a forbidden drive names the policy's reason, not the permit's code (2026-10-18 demo: pacing read as drive_forbidden)", () => {
  assert.equal(permitText({ reason: "drive_forbidden", limit_mps: 0 }, "pacing"), "시뮬레이션이 실시간을 따라가지 못해 수동 조종을 받지 않습니다 · 상한 0.00 m/s");
  assert.equal(permitText({ reason: "drive_forbidden", limit_mps: 0 }, null), "수동 주행 금지 · 상한 0.00 m/s");
  assert.equal(permitText({ reason: "clear", limit_mps: 0.3 }, "pacing"), "clear · 상한 0.30 m/s");
});

test("a reject without a Korean text shows its code", () => {
  assert.equal(rejectText("mission_active"), "mission_active");
});

test("plan D14: each lift button follows the allowed directions, and the travel is the state's", async () => {
  const { liftControls } = await import("../js/profile.js");
  assert.deepEqual(liftControls({ lift_dirs: [-1, 1], lift_limit_m: 0.28 }), { up: true, down: true, limitMm: 280 });
  assert.deepEqual(liftControls({ lift_dirs: [-1], lift_limit_m: 0.28 }), { up: false, down: true, limitMm: 280 });
  assert.deepEqual(liftControls({ lift_dirs: [], lift_limit_m: 0.2 }), { up: false, down: false, limitMm: 200 });
  // an older back end without lift_dirs: lift_allowed for both, the old 280 mm
  assert.deepEqual(liftControls({ lift_allowed: true }), { up: true, down: true, limitMm: 280 });
});

test("plan D14: the recovery reasons and the straight reverse are said in Korean", async () => {
  const { recoveryNote } = await import("../js/profile.js");
  for (const r of ["recovery:lower_first", "recovery:ground_wait", "recovery:evidence", "recovery:misaligned",
    "recovery:unknown", "recovery:no_lift_measure", "recovery:no_steering", "recovery:steering"]) {
    assert.notEqual(forbidText(r), r, r);
  }
  assert.match(recoveryNote({ drive_mode: "reverse_straight", recovery: { stage: "in_pocket" } }), /곧게.*인증 없음/);
  assert.match(recoveryNote({ drive_mode: "reverse_straight", recovery: { stage: "manual_clear" } }), /구역/);
  assert.equal(recoveryNote({ drive_mode: "normal", recovery: null }), "");
});
