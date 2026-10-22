import { test } from "node:test";
import assert from "node:assert/strict";
import { AlarmBook, badge, healthRows, queueStopText } from "../js/health.js";
import { CALM_REASONS } from "../js/queue.js";

// ---- fixtures: a connected fake vehicle a little after its welcome -------------------------------------------------
const CAPS = { control: true, media: { camera_rgb: { max_hz: 15 }, scan: { max_hz: 10 }, grid: { max_hz: 1 } } };
const STATE = { mode: "PAUSED", stop_kind: "controlled", stopped_confirmed: true, feedback_ok: true,
                permit: { limit_mps: 0.5, reason: "ok" }, scene: { damaged: null },
                lease: { holder: "me" }, pose: { frame: "map", x_m: 1, y_m: 1, yaw_rad: 0, age_s: 0.05 },
                freshness: { grid_free_age_s: null, slam_correction_age_s: null, near_obs_age_s: null } };
const W = 10_000;

function input(over = {}) {
  return {
    nowMs: W + 5_000, welcomeAt: W, link: { up: true, mediaUp: true }, sessionEnding: null,
    caps: CAPS, monitor: false, clientId: "me", hasLease: true,
    state: STATE, stateAt: W + 4_900, cameraAt: W + 4_950, scanAt: W + 4_900, scanReceivedAt: W + 4_900,
    grid: { present: true, withdrawnReason: null }, standby: null, queue: { run: "stopped", stopReason: "시작 전", items: 0 },
    ...over,
  };
}
const row = (rows, key) => rows.find((r) => r.key === key);
const lv = (over, key) => row(healthRows(input(over)), key).level;

// ---- rows ----------------------------------------------------------------------------------------------------------
test("rows follow the existing limits", () => {
  const all = healthRows(input());
  for (const r of all) assert.ok(["ok", "off"].includes(r.level), `${r.key} is ${r.level} on a healthy link`);
  assert.equal(lv({ stateAt: W + 4_100 }, "state"), "ok"); // 0.9 s
  assert.equal(lv({ stateAt: W + 3_500 }, "state"), "warn"); // 1.5 s
  assert.equal(lv({ stateAt: W + 2_500 }, "state"), "crit"); // 2.5 s
  assert.equal(lv({ stateAt: W + 4_200, state: { ...STATE, pose: { ...STATE.pose, age_s: 0.3 } } }, "pose"), "warn");
  assert.equal(lv({ state: { ...STATE, pose: null } }, "pose"), "warn");
  assert.equal(lv({ cameraAt: W + 3_900 }, "camera"), "warn"); // 1.1 s > 1.0 s
  assert.equal(lv({ state: { ...STATE, feedback_ok: false } }, "feedback"), "crit");
  const { feedback_ok: _f, ...noFeedback } = STATE;
  assert.equal(lv({ state: noFeedback }, "feedback"), "unknown");
  assert.equal(lv({ state: { ...STATE, mode: "STOPPED", stop_kind: "controlled" } }, "stop"), "crit");
  assert.equal(lv({ state: { ...STATE, stop_kind: "safety_preempt" } }, "stop"), "crit");
  assert.equal(lv({ state: { ...STATE, stop_kind: "preempt" } }, "stop"), "crit");
  assert.equal(lv({ state: { ...STATE, mode: "AUTO", stop_kind: "none", stopped_confirmed: false } }, "stop"), "ok");
  assert.equal(lv({ state: { ...STATE, permit: { limit_mps: 0, reason: "occupied:0.2m" } } }, "permit"), "warn");
  assert.equal(lv({ state: { ...STATE, permit: null } }, "permit"), "unknown");
  assert.equal(lv({ state: { ...STATE, scene: { damaged: { reason: "contact", unix: 1 } } } }, "scene"), "crit");
  assert.equal(lv({ grid: { present: false, withdrawnReason: "frame change" } }, "grid"), "warn");
  assert.equal(lv({ link: { up: true, mediaUp: false } }, "media"), "warn");
  assert.equal(lv({ link: { up: false, mediaUp: true }, welcomeAt: null }, "control"), "crit");
  for (const st of [STATE, { ...STATE, lease: { holder: "other" } }, { ...STATE, lease: null }, { ...STATE, slam: undefined }]) {
    for (const key of ["lease", "slam", "freshness"]) assert.equal(lv({ state: st, hasLease: false }, key), "off");
  }
});

test("a state from before this welcome does not count; no first state within 2 s is critical", () => {
  assert.equal(lv({ stateAt: W - 100, nowMs: W + 1_900 }, "state"), "unknown");
  assert.equal(lv({ stateAt: W - 100, nowMs: W + 2_100 }, "state"), "crit");
  assert.equal(lv({ stateAt: null, nowMs: W + 2_100 }, "state"), "crit");
  assert.equal(lv({ stateAt: W - 100, nowMs: W + 1_000 }, "pose"), "unknown");
});

test("scan and camera never received: unknown, then warn; not declared: off", () => {
  assert.equal(lv({ cameraAt: null, nowMs: W + 1_500 }, "camera"), "unknown");
  assert.equal(lv({ cameraAt: W - 50, nowMs: W + 2_100 }, "camera"), "warn");
  assert.equal(lv({ scanAt: null, scanReceivedAt: null, nowMs: W + 1_500 }, "scan"), "unknown");
  assert.equal(lv({ scanAt: null, scanReceivedAt: null, nowMs: W + 2_100 }, "scan"), "warn");
  const bare = { control: true, media: {} };
  assert.equal(lv({ caps: bare }, "camera"), "off");
  assert.equal(lv({ caps: bare }, "scan"), "off");
  assert.equal(lv({ caps: bare }, "grid"), "off");
  assert.equal(lv({ grid: { present: false, withdrawnReason: null } }, "grid"), "unknown");
});

test("slow pacing widens the scan limit like the video one", () => {
  const slow = { ...STATE, pacing: { target_factor: 0.05 } };
  assert.equal(lv({ state: slow, scanAt: W + 3_000 }, "scan"), "ok"); // 2.0 s at 0.05x
  assert.equal(lv({ state: slow, scanAt: W - 16_000, scanReceivedAt: W + 100, nowMs: W + 5_000 }, "scan"), "warn"); // 21 s
  assert.equal(lv({ state: slow, cameraAt: W + 3_000 }, "camera"), "ok");
});

test("a monitor-only state is unknown, not ok", () => {
  const mon = { ...STATE, mode: null, stop_kind: null, lease: null, permit: null, feedback_ok: null };
  const rows = healthRows(input({ state: mon, monitor: true, caps: { ...CAPS, control: false }, hasLease: false }));
  assert.equal(row(rows, "stop").level, "unknown");
  assert.match(row(rows, "stop").value, /멈출 수 없음/);
  assert.match(row(rows, "lease").value, /모름/);
  assert.match(row(healthRows(input({ state: { ...STATE, lease: { holder: null } }, hasLease: false })), "lease").value, /아무도/);
  assert.equal(row(rows, "permit").level, "unknown");
  assert.equal(row(rows, "feedback").level, "unknown");
});

test("an intended session end or reset turns the link rows off", () => {
  const ended = input({ sessionEnding: "session_end", link: { up: false, mediaUp: false }, nowMs: W + 60_000 });
  for (const key of ["control", "state", "pose", "media", "camera", "scan"]) {
    assert.equal(row(healthRows(ended), key).level, "off", key);
  }
  assert.equal(lv({ sessionEnding: "sim_reset", link: { up: false, mediaUp: true } }, "control"), "off");
  assert.equal(lv({ link: { up: false, mediaUp: true }, standby: { standby: true, phase: "starting", epoch: 2 } }, "session"), "off");
  assert.equal(lv({ link: { up: false, mediaUp: true },
                    standby: { standby: true, phase: "backoff", retry: true, next_epoch: 3,
                               last_exit: { exit_code: 1, stage: "run", reason: "crash" } } }, "session"), "warn");
  assert.equal(lv({ link: { up: false, mediaUp: true }, standby: { standby: true, phase: "stopped", end: "failing" } }, "session"), "warn");
  assert.equal(lv({ link: { up: false, mediaUp: true }, standby: { standby: true, phase: "stopped", end: "ended" } }, "session"), "off");
  assert.equal(lv({}, "session"), "ok");
});

test("while a standby starts the session or a reset runs, the links are not expected (Codex plan round 2)", () => {
  const starting = { standby: true, phase: "starting", epoch: 2 };
  const down = { link: { up: false, mediaUp: false }, welcomeAt: null, nowMs: W + 30_000 };
  for (const over of [{ standby: starting }, { sessionEnding: "sim_reset" }]) {
    const rows = healthRows(input({ ...down, ...over }));
    for (const key of ["control", "media", "state", "pose", "camera", "scan"]) {
      assert.equal(row(rows, key).level, "off", `${JSON.stringify(over)} ${key}`);
    }
  }
});

test("a failed start shows through a reset in progress (Codex plan round 2)", () => {
  const failed = { standby: true, phase: "stopped", end: "failing", last_exit: { exit_code: 1, stage: "start", reason: "x" } };
  const rows = healthRows(input({ link: { up: false, mediaUp: false }, welcomeAt: null, sessionEnding: "sim_reset", standby: failed }));
  assert.equal(row(rows, "session").level, "warn");
  assert.equal(row(rows, "control").level, "crit");
});

test("before a state of this connection, the state's rows are unknown (Codex plan round 3)", () => {
  const old = { ...STATE, feedback_ok: true, mode: "STOPPED", scene: { damaged: null } };
  const rows = healthRows(input({ state: old, stateAt: W - 100, nowMs: W + 500 }));
  for (const key of ["feedback", "stop", "permit", "scene"]) assert.equal(row(rows, key).level, "unknown", key);
  const down = healthRows(input({ state: old, link: { up: false, mediaUp: true }, welcomeAt: null }));
  for (const key of ["feedback", "stop", "permit", "scene"]) assert.equal(row(down, key).level, "unknown", key);
  const ended = healthRows(input({ state: old, link: { up: false, mediaUp: false }, welcomeAt: null, sessionEnding: "session_end" }));
  for (const key of ["feedback", "stop", "permit", "scene"]) assert.equal(row(ended, key).level, "off", key);
});

test("a permit with no limit is unknown", () => {
  assert.equal(lv({ state: { ...STATE, permit: { limit_mps: null, reason: "no_feedback" } } }, "permit"), "unknown");
});

test("a queue stop reason reads in words, its code kept beside it (Isaac live run 2026-10-22)", () => {
  const code = "observe:all_candidates_exhausted:perception_no_pallet:no_candidate_in_radius:success;success";
  assert.match(queueStopText(`실패: ${code}`), /^실패: .*팔레트를 찾지 못했습니다/);
  assert.match(queueStopText("거부: already_there"), /^거부: 그 팔레트는 이미/);
  assert.equal(queueStopText("연결 끊김"), "연결 끊김");
  assert.equal(queueStopText(null), "—");
  const r = row(healthRows(input({ queue: { run: "stopped", stopReason: `실패: ${code}`, items: 1 } })), "queue");
  assert.match(r.value, /팔레트를 찾지 못했습니다/);
  assert.equal(r.detail, code);
});

test("the queue row warns only for an anomaly stop with items left", () => {
  for (const why of ["실패: test", "연결 끊김", "정지됨(STOP)"]) {
    assert.equal(lv({ queue: { run: "stopped", stopReason: why, items: 1 } }, "queue"), "warn", why);
  }
  for (const why of CALM_REASONS) assert.equal(lv({ queue: { run: "stopped", stopReason: why, items: 2 } }, "queue"), "off", why);
  assert.equal(lv({ queue: { run: "stopped", stopReason: "실패: test", items: 0 } }, "queue"), "off");
  assert.equal(lv({ queue: { run: "running", stopReason: null, items: 1 } }, "queue"), "ok");
  for (const why of ["시작 전", "완료", "콘솔 다시 열림", "운용자 멈춤", "운용자 취소", "운용자 일시정지", "STOP", "수동 지시"]) {
    assert.ok(CALM_REASONS.includes(why), why);
  }
});

// ---- alarm book ----------------------------------------------------------------------------------------------------
const R = (key, level, value = level) => ({ key, label: key, level, value, detail: "" });
// feeds one row every 100 ms from t0 for ms; the unix clock is t/1000
function feed(book, key, level, t0, ms, value = level, epoch = 1) {
  for (let t = t0; t <= t0 + ms; t += 100) book.update([R(key, level, value)], t, t / 1000, { epoch, viewing: true });
  return t0 + ms;
}
const open = (book) => book.alarms.filter((a) => a.cleared_unix === null);

test("an alarm opens after 0.5 s of warn at its first sighting and closes after 1.0 s of ok", () => {
  const b = new AlarmBook();
  feed(b, "state", "warn", 1_000, 400);
  assert.equal(open(b).length, 0);
  feed(b, "state", "warn", 1_500, 100);
  assert.equal(open(b).length, 1);
  assert.equal(open(b)[0].opened_unix, 1.0, "the first sighting");
  feed(b, "state", "ok", 1_700, 900);
  assert.equal(open(b).length, 1);
  feed(b, "state", "ok", 2_700, 200);
  assert.equal(open(b).length, 0);
  assert.equal(b.alarms[0].cleared_note, "복구");
});

test("off closes at once as not applicable; unknown breaks a streak and closes nothing", () => {
  const b = new AlarmBook();
  feed(b, "queue", "warn", 0, 600);
  b.update([R("queue", "off")], 700, 0.7, { epoch: 1, viewing: true });
  assert.equal(open(b).length, 0);
  assert.equal(b.alarms[0].cleared_note, "해당 없어짐");
  const c = new AlarmBook();
  feed(c, "state", "warn", 0, 600);
  feed(c, "state", "unknown", 700, 3_000);
  assert.equal(open(c).length, 1, "unknown is not better");
  const d = new AlarmBook();
  feed(d, "state", "warn", 0, 300);
  feed(d, "state", "unknown", 400, 100);
  feed(d, "state", "warn", 600, 300);
  assert.equal(open(d).length, 0, "the streak started again after unknown");
});

test("an escalation raises the level and keeps the peak text; a drop keeps the level", () => {
  const b = new AlarmBook();
  feed(b, "state", "warn", 0, 600, "1.5 s");
  feed(b, "state", "crit", 700, 200, "2.5 s");
  assert.equal(open(b)[0].level, "crit");
  assert.equal(open(b)[0].peak_text, "2.5 s");
  feed(b, "state", "warn", 1_000, 200, "1.2 s");
  assert.equal(open(b)[0].level, "crit");
  assert.equal(open(b)[0].text, "1.2 s");
  assert.equal(open(b)[0].peak_text, "2.5 s");
});

test("a gap over 2 s restarts every streak", () => {
  const b = new AlarmBook();
  feed(b, "state", "warn", 0, 300);
  feed(b, "state", "warn", 3_400, 300);
  assert.equal(open(b).length, 0);
});

test("a new session closes open alarms with a note", () => {
  const b = new AlarmBook();
  feed(b, "control", "crit", 0, 600);
  b.update([R("control", "crit")], 700, 0.7, { epoch: 2, viewing: true });
  assert.equal(b.alarms.find((a) => a.cleared_note === "새 세션")?.key, "control");
});

test("only the last 100 alarms are kept; open ones stay", () => {
  const b = new AlarmBook();
  feed(b, "scene", "crit", 0, 600); // stays open
  let t = 1_000;
  for (let i = 0; i < 120; i += 1) {
    b.update([R("scene", "crit"), R("state", "warn")], t, t / 1000, { epoch: 1, viewing: true });
    t = feed(b, "state", "warn", t + 100, 600) + 100;
    b.update([R("scene", "crit"), R("state", "off")], t, t / 1000, { epoch: 1, viewing: true });
    t += 100;
  }
  assert.ok(b.alarms.length <= 100);
  assert.ok(b.alarms.some((a) => a.key === "scene" && a.cleared_unix === null));
});

test("the unseen-critical mark survives a reload until the page is seen (Codex plan round 3)", () => {
  const c = new AlarmBook();
  for (let t = 0; t <= 600; t += 100) c.update([R("stop", "crit")], t, t / 1000, { epoch: 1, viewing: false });
  for (let t = 700; t <= 1_800; t += 100) c.update([R("stop", "ok")], t, t / 1000, { epoch: 1, viewing: false });
  const r = AlarmBook.fromStorage(JSON.parse(JSON.stringify(c.toStorage())), 5);
  assert.equal(badge(r).dot, true);
  c.markSeen();
  assert.equal(badge(AlarmBook.fromStorage(JSON.parse(JSON.stringify(c.toStorage())), 5)).dot, false);
});

test("a critical alarm seen while open is not unseen when it closes elsewhere (Codex plan round 3)", () => {
  const c = new AlarmBook();
  for (let t = 0; t <= 600; t += 100) c.update([R("stop", "crit")], t, t / 1000, { epoch: 1, viewing: false });
  c.update([R("stop", "crit")], 700, 0.7, { epoch: 1, viewing: true });
  c.markSeen();
  for (let t = 800; t <= 2_000; t += 100) c.update([R("stop", "ok")], t, t / 1000, { epoch: 1, viewing: false });
  assert.equal(badge(c).dot, false);
});

test("seeing an alarm while it was only a warning does not count as seeing it critical (Codex plan round 4)", () => {
  const c = new AlarmBook();
  for (let t = 0; t <= 600; t += 100) c.update([R("state", "warn")], t, t / 1000, { epoch: 1, viewing: true });
  c.markSeen();
  for (let t = 700; t <= 900; t += 100) c.update([R("state", "crit")], t, t / 1000, { epoch: 1, viewing: false });
  for (let t = 1_000; t <= 2_200; t += 100) c.update([R("state", "ok")], t, t / 1000, { epoch: 1, viewing: false });
  assert.equal(badge(c).dot, true);
});

test("an unseen critical alarm still open at a reload leaves the dot (Codex plan round 4)", () => {
  const c = new AlarmBook();
  for (let t = 0; t <= 600; t += 100) c.update([R("stop", "crit")], t, t / 1000, { epoch: 1, viewing: false });
  assert.equal(badge(AlarmBook.fromStorage(JSON.parse(JSON.stringify(c.toStorage())), 5)).dot, true);
  const d = new AlarmBook();
  for (let t = 0; t <= 600; t += 100) d.update([R("stop", "crit")], t, t / 1000, { epoch: 1, viewing: true });
  assert.equal(badge(AlarmBook.fromStorage(JSON.parse(JSON.stringify(d.toStorage())), 5)).dot, false);
});

test("seeing an open critical alarm changes the version so it is stored (Codex plan round 4)", () => {
  const c = new AlarmBook();
  for (let t = 0; t <= 600; t += 100) c.update([R("stop", "crit")], t, t / 1000, { epoch: 1, viewing: false });
  const v = c.version;
  c.update([R("stop", "crit")], 700, 0.7, { epoch: 1, viewing: true });
  assert.notEqual(c.version, v);
  assert.equal(c.alarms[0].seen, true);
});

test("restored alarms that were open are closed as unseen", () => {
  const b = new AlarmBook();
  feed(b, "control", "crit", 0, 600);
  const r = AlarmBook.fromStorage(JSON.parse(JSON.stringify(b.toStorage())), 99);
  assert.equal(open(r).length, 0);
  assert.equal(r.alarms[0].cleared_note, "화면 다시 열림 — 그 사이 모름");
  assert.equal(r.alarms[0].cleared_unix, 99);
  assert.equal(AlarmBook.fromStorage(null, 1).alarms.length, 0);
});

test("the badge counts open alarms, takes the worst level, and shows a dot for an unseen critical one", () => {
  const b = new AlarmBook();
  assert.deepEqual(badge(b), { count: 0, level: null, dot: false });
  feed(b, "state", "warn", 0, 600);
  assert.deepEqual(badge(b), { count: 1, level: "warn", dot: false });
  const c = new AlarmBook();
  for (let t = 0; t <= 600; t += 100) c.update([R("stop", "crit")], t, t / 1000, { epoch: 1, viewing: false });
  for (let t = 700; t <= 1_800; t += 100) c.update([R("stop", "ok")], t, t / 1000, { epoch: 1, viewing: false });
  assert.deepEqual(badge(c), { count: 0, level: null, dot: true });
  c.markSeen();
  assert.equal(badge(c).dot, false);
  // an open warning does not hide an unseen critical one (Codex plan round 2)
  const d = new AlarmBook();
  for (let t = 0; t <= 600; t += 100) d.update([R("camera", "warn"), R("feedback", "crit")], t, t / 1000, { epoch: 1, viewing: false });
  for (let t = 700; t <= 1_800; t += 100) d.update([R("camera", "warn"), R("feedback", "ok")], t, t / 1000, { epoch: 1, viewing: false });
  assert.deepEqual(badge(d), { count: 1, level: "warn", dot: true });
});
