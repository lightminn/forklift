import { test } from "node:test";
import assert from "node:assert/strict";
import { createSession } from "../js/protocol.js";
import { HIGH_DUTY_DISTANCE, clearedOnChange, rangeMarkBefore, runKind, targetsInWindow, Graph, advice, bypassChips, clearView, createBenchController, fieldState, isBenchWelcome, largePhrase,
  resultView, runFields, wheelDelta } from "../js/bench.js";

// docs/plans/2026-11-27-bench-console.md 4 and 4.1: the bench screen's model on the real console session

const BENCH = { t: "welcome", api: "console/1", session_epoch: 9, session: "bench", backend: "real",
  product: "forklift-mcu commission OPEN v0 rec=0 lim=lr1 bench6", capabilities: { control: true, bench: true },
  request_token: 77 };
const REAL = { t: "welcome", api: "console/1", session_epoch: 9, backend: "real", capabilities: { control: true },
  request_token: 78 };
const ISAAC = { t: "welcome", api: "console/1", session_epoch: 11, backend: "isaac", capabilities: { control: true } };
const HIL = { t: "welcome", api: "console/1", session_epoch: 12, backend: "real", capabilities: { control: true },
  request_token: 79 };

function rig() {
  const sent = [];
  let t = 0, n = 0;
  const s = createSession({ clientId: "tab", send: (m) => sent.push(m), now: () => t, newId: () => `id${++n}` });
  const b = createBenchController({ session: s, now: () => t });
  s.subscribe((msg) => b.onMessage(msg));
  const welcome = (w) => { s.onMessage(w); b.onWelcome(w); };
  const disconnect = () => { s.onDisconnect(); b.onDisconnect(); };
  return { s, b, sent, welcome, disconnect, tick: (dt) => { t += dt; } };
}

const FORM = { axis: "L", duty: "0.1", seconds: "0.6", power_switch_in_hand: true };

test("only a bench welcome shows the bench screen", () => {
  assert.equal(isBenchWelcome(BENCH), true);
  for (const w of [REAL, ISAAC, HIL, null, {}]) assert.equal(isBenchWelcome(w), false);
});

test("welcome before -> bench -> disconnect -> REAL / Isaac / HIL: the screen and its requests follow (plan 4 (a))", () => {
  const { b, sent, welcome, disconnect } = rig();
  assert.equal(b.visible, false);
  assert.equal(b.run(FORM).sent, null); // before any welcome nothing is sent
  welcome(BENCH);
  assert.equal(b.visible, true);
  disconnect();
  assert.equal(b.visible, false);
  for (const w of [REAL, ISAAC, HIL]) {
    const before = sent.length;
    welcome(w);
    assert.equal(b.visible, false);
    assert.equal(b.run(FORM).sent, null);
    assert.equal(b.disarm(), null);
    assert.equal(b.clearFaults("x").sent, null);
    assert.ok(!sent.slice(before).some((m) => String(m.t).startsWith("bench_") || m.t === "clear_faults"));
    disconnect();
  }
});

test("a bench request lost with its connection is never resent; the shared STOP is (plan 4 (a''))", () => {
  for (const next of [BENCH, { ...BENCH, session_epoch: 10, request_token: 90 }]) {
    const { s, b, sent, welcome, disconnect, tick } = rig();
    welcome(BENCH);
    s.onMessage({ t: "ack", request_id: s.request("lease_request") && sent.at(-1).request_id, lease_id: "L1" });
    const id = b.run(FORM).sent;
    assert.ok(id && sent.at(-1).t === "bench_run" && sent.at(-1).request_id === id);
    const clr = b.clearFaults("cause").sent;
    s.stop("console");
    const stopId = sent.at(-1).request_id;
    disconnect(); // no ACK came for any of them
    assert.deepEqual(b.pending, []);
    welcome(next);
    tick(5000);
    s.pump();
    const after = sent.filter((m) => m.request_id === id || m.request_id === clr);
    assert.equal(after.length, 2); // the first sends only: not one resend after the disconnect
    assert.ok(sent.filter((m) => m.request_id === stopId).length >= 2); // STOP keeps being sent until answered
  }
});

test("an answer to a forgotten bench request is not this connection's reply", async () => {
  const { s, b, sent, welcome, disconnect } = rig();
  welcome(BENCH);
  const id = b.run(FORM).sent;
  disconnect();
  welcome(BENCH);
  s.onMessage({ t: "ack", of: "bench_run", request_id: id });
  await new Promise((r) => setImmediate(r));
  assert.deepEqual(b.replies, []);
  assert.ok(sent.length > 0);
});

test("replies are kept with their reason and detail", async () => {
  const { s, b, welcome } = rig();
  welcome(BENCH);
  const id = b.run(FORM).sent;
  s.onMessage({ t: "reject", of: "bench_run", request_id: id, reason: "bench_refused",
    detail: "large run: large_confirm must be exactly 'duty 0.5, 0.6 s, wheels lifted and fixed'" });
  await new Promise((r) => setImmediate(r));
  const r = b.replies.at(-1);
  assert.equal(r.reason, "bench_refused");
  assert.equal(largePhrase(r.detail), "duty 0.5, 0.6 s, wheels lifted and fixed");
  assert.equal(largePhrase("something else"), null);
});

test("fields that do not fit the axis or the source are closed; hold and stop open on the KTH source only", () => {
  const l = fieldState({ axis: "L" });
  assert.deepEqual(l.sources, ["none"]);
  assert.equal(l.source, "none");
  assert.equal(l.windowOpen, false);
  assert.equal(l.bypassOpen.steer_range, false);
  const m = fieldState({ axis: "M" });
  assert.deepEqual(m.sources, ["kth", "none"]);
  assert.equal(m.source, "kth");
  assert.equal(m.windowOpen, true);
  assert.equal(m.bypassOpen.steer_range, true);
  assert.equal(m.bypassOpen.lift_range, false);
  const off = fieldState({ axis: "M", bypass: ["steer_range"] });
  assert.equal(off.windowOpen, false);
  assert.equal(off.rangeOff, true);
  assert.equal(l.holdOpen, false);
  assert.equal(l.stopOpen, false);
  assert.equal(m.holdOpen, true);
  assert.equal(m.stopOpen, true);
  const none = fieldState({ axis: "M", source: "none" });
  assert.equal(none.holdOpen, false);
  assert.equal(none.stopOpen, false);
  for (const f of [l, m]) {
    assert.equal(f.mtOpen, false);
    assert.equal(f.kthInvalidOpen, false);
  }
});

test("the run's fields: booleans only from the boxes, the window only with the range on", () => {
  const { fields, problems } = runFields({ axis: "M", duty: "0.12", seconds: "1", window_lo: "-0.2", window_hi: "0.2",
    power_switch_in_hand: true, bypass: ["lift_range"], physical_setup: "decoupled" });
  assert.deepEqual(problems, []);
  assert.deepEqual(fields.window, [-0.2, 0.2]);
  assert.deepEqual(fields.bypass, []); // lift_range does not apply to steering
  assert.equal(fields.power_switch_in_hand, true);
  assert.equal(fields.wheels_lifted, false);
  assert.equal(fields.physical_setup, "decoupled");
  const off = runFields({ axis: "M", duty: "0.2", seconds: "1", bypass: ["steer_range"], power_switch_in_hand: "yes" });
  assert.equal(off.fields.window, undefined);
  assert.equal(off.fields.power_switch_in_hand, false); // only a real true
  assert.deepEqual(off.fields.bypass, ["steer_range"]);
  assert.deepEqual(runFields({ axis: "M", duty: "x", seconds: "", window_lo: "a" }).problems,
    ["duty", "seconds", "window"]);
});

test("the advice is shown, never a refusal (plan 0.2)", () => {
  assert.deepEqual(advice({ axis: "M", bypass: ["steer_range"] }), ["rangeOff"]);
  assert.deepEqual(advice({ axis: "L" }), []);
  const { problems } = runFields({ axis: "M", duty: "0.2", seconds: "1", bypass: ["steer_range"] });
  assert.deepEqual(problems, []); // a bypass without the decoupling still sends
});

test("the chips keep what the MCU confirmed apart from the host's", () => {
  const c = bypassChips({ bench: { mcu_bypass: ["steer_range"] },
    run: { bypass_asked: ["steer_range"], bypass_host: [] } });
  assert.deepEqual(c, { mcu: ["steer_range"], host: [], asked: ["steer_range"] });
  assert.deepEqual(bypassChips(null), { mcu: [], host: [], asked: [] });
});

test("the result view reads verdict one and leaves two and three to B2", () => {
  const v = resultView({ verdict: { run_cleanup: { ok: false, why: ["aborted:stop"] } }, closed: "ended",
    stop_reason: "end_zero", aborted: "stop", peak_current_ma: [120, 0, 0, 0], record: "/r/x", axis: "L" });
  assert.equal(v.ok, false);
  assert.deepEqual(v.why, ["aborted:stop"]);
  assert.equal(v.stopReason, "end_zero");
  assert.deepEqual(v.peakCurrentMa, [120, 0, 0, 0]);
  assert.equal(v.holdVerdict, null);
  assert.equal(resultView(null), null);
});

test("the stream fills the 10 s graph and only on the bench screen of this epoch", () => {
  const { s, b, welcome, tick } = rig();
  const st = (gen) => ({ t: "bench_state", session_epoch: 9, control_gen: gen, mode: "PAUSED", stops: 0,
    lease: null, board: { duty_e4: [1000, 0, 0, 0], current_ma: [50, 0, 0, 0], steering_rad: null, lift_m: 0.1,
      wheel_count: [100, 0] } });
  s.onMessage(st(1));
  assert.equal(b.graph.samples.length, 0); // not shown yet
  welcome(BENCH);
  for (let k = 0; k < 30; k += 1) {
    s.onMessage({ ...st(1), board: { ...st(1).board, wheel_count: [100 + k, 0] } });
    tick(500);
  }
  assert.ok(b.graph.samples.length <= 21); // 10 s at 2 Hz plus the edge
  const ser = b.graph.series("L");
  assert.equal(ser[0].value, 0);
  assert.equal(ser.at(-1).duty, 0.1);
  s.onMessage({ ...st(1), session_epoch: 8 });
  assert.notEqual(b.state.session_epoch, 8);
});

test("the graph keeps only the last window", () => {
  const g = new Graph(1000);
  for (let t = 0; t <= 3000; t += 100) g.push(t, { board: { duty_e4: [0, 0, 0, 0], current_ma: [0, 0, 0, 0],
    wheel_count: [t, 0] } });
  assert.ok(g.samples[0].t >= 2000);
});

test("bench_state carries the control generation and the lease holder into the session (plan 4.1)", async () => {
  const { s, sent, welcome } = rig();
  welcome(BENCH);
  const p = s.request("lease_request");
  s.onMessage({ t: "ack", of: "lease_request", request_id: sent.at(-1).request_id, lease_id: "L1", control_gen: 3 });
  await p;
  s.onMessage({ t: "bench_state", session_epoch: 9, control_gen: 4, lease: { client_id: "tab", remaining_s: 3 } });
  assert.equal(s.hasLease, true);
  assert.equal(s.generations.control_gen, 4);
  s.request("bench_run", { axis: "L" });
  assert.equal(sent.at(-1).control_gen, 4); // the first bench request carries the current generation
  s.onMessage({ t: "bench_state", session_epoch: 8, control_gen: 99, lease: null }); // another session: not taken
  assert.equal(s.generations.control_gen, 4);
  s.onMessage({ t: "bench_state", session_epoch: 9, control_gen: 5, lease: { client_id: "other" } });
  assert.equal(s.hasLease, false);
  assert.equal(s.benchState.control_gen, 5);
});

test("a non-bench session never has a bench state", () => {
  const { s, welcome } = rig();
  welcome(REAL);
  s.onMessage({ t: "state", session_epoch: 9, control_gen: 2, arm_gen: 0, armed: false, mode: "PAUSED" });
  assert.equal(s.benchState, null);
});

test("after a disconnect, a pump before the next welcome resends no bench request, only the STOP (round 2 P2-6)", () => {
  for (const next of [BENCH, { ...BENCH, session_epoch: 10, request_token: 90 }]) {
    const { s, b, sent, welcome, disconnect, tick } = rig();
    welcome(BENCH);
    s.onMessage({ t: "state", session_epoch: 9, control_gen: 1, arm_gen: 0, armed: false, mode: "PAUSED" });
    s.onMessage({ t: "bench_state", session_epoch: 9, control_gen: 1, lease: null, board: { mode: "FAULT" } });
    const run = b.run(FORM).sent;
    const clr = b.clearFaults("cause").sent;
    s.stop("console");
    const stopId = sent.at(-1).request_id;
    disconnect();
    const before = sent.length;
    tick(2000);
    s.pump(); // the new socket is open, its welcome not here yet
    const resent = sent.slice(before);
    assert.ok(!resent.some((m) => m.request_id === run || m.request_id === clr));
    assert.ok(resent.some((m) => m.request_id === stopId));
    welcome(next);
    tick(2000);
    s.pump();
    assert.ok(!sent.slice(before).some((m) => m.request_id === run || m.request_id === clr));
  }
});

test("one start at a time: a second press while the first waits is not sent (round 2 P2-2)", () => {
  const { b, sent, welcome } = rig();
  welcome(BENCH);
  const first = b.run(FORM);
  const second = b.run(FORM);
  assert.ok(first.sent);
  assert.equal(second.sent, null);
  assert.deepEqual(second.problems, ["busy"]);
  assert.equal(sent.filter((m) => m.t === "bench_run").length, 1);
});

test("a late ACK of a resend never clears the result that already came (round 2 P2-2)", async () => {
  const { s, b, welcome } = rig();
  welcome(BENCH);
  const id = b.run(FORM).sent;
  s.onMessage({ t: "bench_result", request_id: id, closed: "ended", verdict: { run_cleanup: { ok: true, why: [] } } });
  assert.equal(b.result.request_id, id);
  s.onMessage({ t: "ack", of: "bench_run", request_id: id });
  await new Promise((r) => setImmediate(r));
  assert.equal(b.result?.request_id, id);
});

test("another tab's result is the previous one, never this tab's", () => {
  const { s, b, welcome } = rig();
  welcome(BENCH);
  b.run(FORM);
  s.onMessage({ t: "bench_result", request_id: "someone-else", closed: "ended" });
  assert.equal(b.result, null);
  assert.equal(b.previous.request_id, "someone-else");
});

test("the clear's end is this tab's clear_result only, with its before and after (round 2 P2-1)", () => {
  const { s, b, welcome } = rig();
  welcome(BENCH);
  s.onMessage({ t: "bench_state", session_epoch: 9, control_gen: 1, lease: null, board: { mode: "FAULT" } });
  const id = b.clearFaults("over-current on M").sent;
  s.onMessage({ t: "clear_result", request_id: "other", result: "cleared" });
  assert.equal(b.clear, null);
  s.onMessage({ t: "clear_result", request_id: id, result: "not_cleared", why: "reject DRIVER",
    before: { mode: "FAULT", fault_flags: ["OVERCURRENT_M"], driver_status: ["C0", "C0", "E2", "C0"] },
    after: { mode: "FAULT", fault_flags: ["OVERCURRENT_M"], driver_status: ["C0", "C0", "E2", "C0"] } });
  const v = clearView(b.clear);
  assert.equal(v.ok, false);
  assert.equal(v.result, "not_cleared");
  assert.equal(v.before.faults, "OVERCURRENT_M");
  assert.equal(v.after.driver, "C0 C0 E2 C0");
});

test("the answers of an earlier connection are not shown on the next one (P3)", async () => {
  const { s, b, welcome, disconnect } = rig();
  welcome(BENCH);
  const id = b.run(FORM).sent;
  s.onMessage({ t: "reject", of: "bench_run", request_id: id, reason: "bench_refused", detail: "x" });
  await new Promise((r) => setImmediate(r));
  assert.equal(b.replies.length, 1);
  disconnect();
  welcome({ ...BENCH, session_epoch: 10 });
  assert.deepEqual(b.replies, []);
});

test("the count graph follows the firmware's 32-bit step and breaks at a boot change (round 2 P2-4)", () => {
  assert.equal(wheelDelta(0, 2 ** 32 - 1), 1);
  assert.equal(wheelDelta(2 ** 32 - 1, 0), -1);
  assert.equal(wheelDelta(105, 100), 5);
  const g = new Graph(100000);
  const push = (t, boot, c) => g.push(t, { board: { boot_id: boot, duty_e4: [0, 0, 0, 0], current_ma: [0, 0, 0, 0],
    wheel_count: [c, 0] } });
  push(0, 1, 2 ** 32 - 2);
  push(10, 1, 2 ** 32 - 1);
  push(20, 1, 0); // the 32-bit wrap: +1, not a fall
  push(30, 2, 0); // a reboot: the line breaks
  push(40, 2, 5);
  assert.deepEqual(g.series("L").map((p) => p.value), [0, 1, 2, null, 5]);
});

test("a bench session's health reads the bench stream, never 'no state since welcome' (round 2 P2-5)", async () => {
  const { healthRows } = await import("../js/health.js");
  const base = { nowMs: 10000, welcomeAt: 1000, link: { up: true, mediaUp: true }, caps: { control: true },
    state: null, stateAt: null, cameraAt: null, scanAt: null, scanReceivedAt: null, grid: { present: false },
    queue: { run: "idle", items: 0 } };
  const ok = healthRows({ ...base, bench: { stateAt: 9950 } });
  assert.equal(ok.find((r) => r.key === "state").level, "ok");
  assert.equal(ok.find((r) => r.key === "pose").level, "off");
  const silent = healthRows({ ...base, bench: { stateAt: null } });
  assert.equal(silent.find((r) => r.key === "state").level, "crit");
  const vehicle = healthRows({ ...base, bench: null });
  assert.equal(vehicle.find((r) => r.key === "state").level, "crit"); // the vehicle profile as before
});

test("a start stays open after its ACK until its own result: a second press in between is not sent (round 3 P2-1)", async () => {
  const { s, b, sent, welcome } = rig();
  welcome(BENCH);
  const a = b.run(FORM).sent;
  s.onMessage({ t: "ack", of: "bench_run", request_id: a });
  await new Promise((r) => setImmediate(r));
  assert.equal(b.run(FORM).sent, null); // the stream has not shown the run yet, the start is still this tab's
  s.onMessage({ t: "bench_result", request_id: a, closed: "ended" });
  assert.equal(b.result.request_id, a);
  assert.ok(b.run(FORM).sent); // after its result a new start goes
  assert.equal(sent.filter((m) => m.t === "bench_run").length, 2);
});

test("a refused start closes at once", async () => {
  const { s, b, welcome } = rig();
  welcome(BENCH);
  const a = b.run(FORM).sent;
  s.onMessage({ t: "reject", of: "bench_run", request_id: a, reason: "bench_refused", detail: "x" });
  await new Promise((r) => setImmediate(r));
  assert.equal(b.runPending, false);
  assert.ok(b.run(FORM).sent);
});

test("one clear at a time, its own end shown, an earlier success not left on screen (round 3 P2-2)", () => {
  const { s, b, welcome } = rig();
  welcome(BENCH);
  const first = b.clearFaults("one").sent;
  s.onMessage({ t: "clear_result", request_id: first, result: "cleared", before: { mode: "FAULT", fault_flags: [] },
    after: { mode: "DISARMED", fault_flags: [] } });
  assert.equal(clearView(b.clear).ok, true);
  const second = b.clearFaults("two").sent;
  assert.equal(b.clear, null); // the earlier success goes with the new attempt
  assert.deepEqual(b.clearFaults("three").problems, ["busy"]); // a press while it runs is not sent
  s.onMessage({ t: "clear_result", request_id: second, result: "refused", why: "the link changed",
    before: { mode: "FAULT", fault_flags: ["OVERCURRENT_M"] }, after: null });
  const v = clearView(b.clear);
  assert.equal(v.ok, false);
  assert.equal(v.after.faults, "확인 불가"); // the board lost: unknown, not none
});

test("the answers are counted past the kept twenty (round 3 P2-3)", async () => {
  const { s, b, welcome } = rig();
  welcome(BENCH);
  for (let k = 0; k < 25; k += 1) {
    const id = b.disarm();
    s.onMessage({ t: "reject", of: "bench_disarm", request_id: id, reason: "board_not_ready", detail: `n${k}` });
    await new Promise((r) => setImmediate(r));
  }
  assert.equal(b.replies.length, 20);
  assert.equal(b.replySeq, 25);
  assert.equal(b.replies.at(-1).detail, "n24");
});

test("the graph empties by the clock and breaks where the board went away (round 3 P2-4)", () => {
  const g = new Graph(10000);
  const board = (c) => ({ board: { boot_id: 1, duty_e4: [1000, 0, 0, 0], current_ma: [0, 0, 0, 0], wheel_count: [c, 0] } });
  g.push(0, board(0));
  g.push(900, board(10));
  g.push(12000, { board: null }); // the board lost, the socket alive
  assert.ok(g.samples.every((s) => s.t >= 2000)); // the old drive is out of the window
  g.push(12100, board(50));
  g.push(12200, board(60));
  assert.deepEqual(g.series("L").map((p) => p.value), [0, 10]); // nothing older left: no gap to mark
  g.push(12300, { board: null }); // a gap inside the window
  g.push(12400, board(80));
  g.push(12500, board(85));
  assert.deepEqual(g.series("L").map((p) => p.value), [0, 10, null, 0, 5]); // broken, counted again after it
  g.expire(30000);
  assert.equal(g.samples.length, 0);
});

test("main.js wires the bench screen: welcome, disconnect, render and the shared STOP (round 3 P3-4)", async () => {
  const { readFile } = await import("node:fs/promises");
  const src = await readFile(new URL("../js/main.js", import.meta.url), "utf8");
  const welcomeBranch = src.slice(src.indexOf('if (msg.t === "welcome") {'), src.indexOf('} else if (msg.t === "tick")'));
  assert.match(welcomeBranch, /m\.bench\.onWelcome\(msg\);\s*syncBench\(m, benchHooks\(\)\);/);
  const close = src.slice(src.indexOf('ws.addEventListener("close"'), src.indexOf("let media = null"));
  assert.match(close, /m\.bench\.onDisconnect\(\);\s*\/\/[^\n]*\n\s*syncBench\(m, benchHooks\(\)\);/);
  assert.match(src, /m\.session\.subscribe\(\(msg\) => \{\s*m\.bench\.onMessage\(msg\);/);
  assert.match(src, /renderBench\(m, \{ rejectText \}\)/);
  assert.match(src, /const benchHooks = \(\) => \(\{ showView, pressStop, rejectText \}\);/);
});

test("an earlier run's late ACK never clears the next run's result (round 4 P2-1)", async () => {
  const { s, b, welcome } = rig();
  welcome(BENCH);
  const a = b.run(FORM).sent;
  s.onMessage({ t: "bench_result", request_id: a, closed: "ended" });
  const bId = b.run(FORM).sent;
  s.onMessage({ t: "ack", of: "bench_run", request_id: bId });
  await new Promise((r) => setImmediate(r));
  s.onMessage({ t: "bench_result", request_id: bId, closed: "ended" });
  s.onMessage({ t: "ack", of: "bench_run", request_id: a }); // A's resend answered late
  await new Promise((r) => setImmediate(r));
  assert.equal(b.result?.request_id, bId);
});


// ---- B2 (plan 4.4 decisions 2 and 6) -------------------------------------------------------------------------------
const M = { axis: "M", source: "kth", seconds: 2, power_switch_in_hand: true, bypass: ["steer_range"] };

test("a hold: the target, its gains and the sign the operator gave; the cap may be left out", () => {
  const { fields, problems } = runFields({ ...M, mode: "hold", hold_target: "rel", hold_value: "60", hold_kd: "0.02",
    count_sign: "1" });
  assert.deepEqual(problems, []);
  assert.equal(fields.mode, "hold");
  assert.equal(fields.duty, undefined);
  assert.deepEqual(fields.hold, { target: "rel:60", kd: 0.02 });
  assert.equal(fields.count_sign, 1);
  assert.equal(fields.stop, undefined);
  assert.equal(runKind({ ...M, mode: "hold" }), "hold");
});

test("a hold or a stop without its sign is the page's own problem, never a default", () => {
  assert.ok(runFields({ ...M, mode: "hold", hold_target: "here" }).problems.includes("count_sign"));
  assert.ok(runFields({ ...M, duty: "0.2", stop_turns: "0.5" }).problems.includes("count_sign"));
  assert.ok(runFields({ ...M, duty: "0.2", measure: true }).problems.includes("count_sign"));
  assert.deepEqual(runFields({ ...M, duty: "0.2" }).problems, []);  // a plain duty run needs none
});

test("a stop after N turns and a measuring run", () => {
  const d = runFields({ ...M, duty: "0.2", stop_turns: "0.5", count_sign: "-1" });
  assert.deepEqual(d.problems, []);
  assert.deepEqual(d.fields.stop, { turns: 0.5 });
  assert.equal(d.fields.count_sign, -1);
  assert.equal(d.fields.measure, undefined);
  const m = runFields({ ...M, duty: "0.2", measure: true, count_sign: "1" });
  assert.equal(m.fields.measure, true);
  assert.equal(m.fields.stop, undefined);
  // a measuring box left on with a stop typed: the stop wins, the box is not sent
  assert.equal(runKind({ ...M, duty: "0.2", stop_turns: "0.5", measure: true }), "distance");
  assert.ok(runFields({ ...M, duty: "0.2", stop_turns: "-1", count_sign: "1" }).problems.includes("stop_turns"));
});

test("a hold target needs a whole number for abs and rel", () => {
  assert.ok(runFields({ ...M, mode: "hold", hold_target: "abs", hold_value: "12.5", count_sign: "1" })
    .problems.includes("hold_value"));
  assert.ok(runFields({ ...M, mode: "hold", hold_target: "abs", count_sign: "1" }).problems.includes("hold_value"));
  assert.ok(runFields({ ...M, mode: "hold", hold_deadband: "3.5", hold_target: "zero", count_sign: "1" })
    .problems.includes("hold_deadband"));
});

test("the KTH-only fields close on the none source and on L/R, and a mode change drops the stop", () => {
  assert.equal(runKind({ ...M, source: "none", mode: "hold", stop_turns: "1" }), "duty");
  assert.equal(runKind({ axis: "L", mode: "hold" }), "duty");
  assert.deepEqual(clearedOnChange("mode"), ["stop_turns", "measure", "duty"]);  // a duty never carries over
  for (const k of ["count_sign", "hold_target", "stop_turns", "same_setup"]) assert.ok(clearedOnChange("axis").includes(k));
  assert.deepEqual(clearedOnChange("source"), clearedOnChange("axis"));
});

test("a high duty stop is shown, a negative one too, never refused", () => {
  assert.ok(advice({ ...M, duty: "0.45", stop_turns: "0.5" }).includes("highDutyDistance"));
  assert.ok(advice({ ...M, duty: "-0.45", stop_turns: "0.5" }).includes("highDutyDistance"));
  assert.ok(!advice({ ...M, duty: String(HIGH_DUTY_DISTANCE), stop_turns: "0.5" }).includes("highDutyDistance"));
  assert.equal(rangeMarkBefore({ ...M, duty: "0.2", stop_turns: "0.5" }), "unchecked");
  assert.equal(rangeMarkBefore({ ...M, mode: "hold" }), "not_applicable");
  assert.equal(rangeMarkBefore({ ...M, duty: "0.2" }), null);
});

test("the result view carries verdicts 2 and 3 and the B2 labels", () => {
  const v = resultView({ kind: "hold", closed: "ended", stop_reason: "end_zero", start: "accepted", cleanup: "confirmed",
    record_complete: true, verdict: { run_cleanup: { ok: true, why: [], labels: [] } },
    hold: { verdict: { result: "met", end_error: 3, settle: { samples: 100, inside: 97 } }, entries: { label: null },
      settings: { target: "here" } } });
  assert.equal(v.holdVerdict.result, "met");
  assert.equal(v.holdVerdict.endError, 3);
  assert.equal(v.holdVerdict.insideShare, 0.97);
  assert.equal(v.distanceVerdict, null);
  const d = resultView({ kind: "distance", verdict: { run_cleanup: { ok: true, why: [], labels: ["보드 기록만(구동 표본 없음)"] } },
    distance: { result: "reached", stop_counts: 2048, moved_at_cut: 2050 }, rest: { result: "settled_at_100hz", cumulative: 2100 },
    range_mark: "unchecked", late_aborts: ["stop"] });
  assert.equal(d.distanceVerdict.result, "reached");
  assert.deepEqual(d.labels, ["보드 기록만(구동 표본 없음)"]);
  assert.equal(d.rest.cumulative, 2100);
  assert.deepEqual(d.lateAborts, ["stop"]);
  assert.equal(d.rangeMark, "unchecked");
});

test("the graph keeps the run's hold error and breaks on a null", () => {
  const g = new Graph(10000);
  const st = (err) => ({ board: { boot_id: 1, duty_e4: [0, 0, 100, 0], current_ma: [0, 0, 0, 0], steering_rad: 0.1,
    lift_m: 0, wheel_count: [0, 0] }, run: { axis: "M", error_counts: err, deadband: 12 } });
  g.push(0, st(null));
  g.push(10, st(40));
  g.push(20, st(null));  // a pair gap in the middle breaks the line
  g.push(30, st(5));
  const s = g.series("M");
  assert.deepEqual(s.map((x) => x.holdError), [null, 40, null, 5]);
  assert.equal(s[1].deadband, 12);
  assert.equal(g.series("M")[1].target, null);  // no target in these samples
  // another axis's graph never shows the M run's error (B2b r1 P2)
  assert.deepEqual(g.series("H").map((x) => x.holdError), [null, null, null, null]);
});

test("a board gap's sample is null in every field and the legend names each run's target (B2b r3 P2-1, P2-2)", () => {
  const g = new Graph(100000);
  const st = (err, target) => ({ board: { boot_id: 1, duty_e4: [0, 0, 100, 0], current_ma: [0, 0, 0, 0],
    steering_rad: 0.1, lift_m: 0, wheel_count: [0, 0] }, run: { axis: "M", error_counts: err, target, deadband: 12 } });
  g.push(0, st(3, 60));
  g.push(10, { board: null });  // the board lost: no target, no error -- never undefined
  g.push(20, st(2, 60));
  g.push(30, st(null, null));  // the next run, its target not confirmed yet
  g.push(40, st(-8, 120));
  const s = g.series("M");
  assert.deepEqual(s.map((x) => x.target), [60, null, 60, null, 120]);
  assert.deepEqual(s.map((x) => x.holdError), [3, null, 2, null, -8]);
  assert.deepEqual(targetsInWindow(s), [60, 120]);  // the earlier run's line keeps its own number
  assert.deepEqual(targetsInWindow(s.slice(0, 4)), [60]);  // an unconfirmed target adds nothing
});


test("the hold's cap is its own field: a switch to a duty run never sends it as a fixed output", () => {
  const h = runFields({ ...M, mode: "hold", hold_target: "here", hold_cap: "0.13", duty: "0.4", count_sign: "1" });
  assert.equal(h.fields.duty, 0.13);
  const d = runFields({ ...M, mode: "duty", hold_cap: "0.15" });  // back to a duty run: the cap stays in its field
  assert.ok(d.problems.includes("duty"));
  assert.equal(d.fields.duty, undefined);
});

test("a number the browser could not read is a typing error, never an empty field", () => {
  assert.ok(runFields({ ...M, duty: "0.2", stop_turns: "NaN", count_sign: "1" }).problems.includes("stop_turns"));
  assert.equal(runKind({ ...M, duty: "0.2", stop_turns: "NaN" }), "distance");  // still a stop run: never a plain one
  assert.ok(runFields({ ...M, mode: "hold", hold_target: "here", hold_kp: "NaN", count_sign: "1" })
    .problems.includes("hold_kp"));
});

test("values below the wire's resolution: the back end's own rounding, for a stop run only", () => {
  assert.ok(runFields({ ...M, duty: "0.2", stop_turns: "0.0001", count_sign: "1" }).problems.includes("stop_turns_too_small"));
  assert.deepEqual(runFields({ ...M, duty: "0.2", stop_turns: "0.0002", count_sign: "1" }).problems, []);
  // exactly half a unit: Python rounds to the even neighbour (0), as the back end does -- refused here too
  assert.ok(runFields({ ...M, duty: "0.2", stop_turns: String(0.5 / 4096), count_sign: "1" })
    .problems.includes("stop_turns_too_small"));
  assert.ok(runFields({ ...M, duty: "0.00005", stop_turns: "0.5", count_sign: "1" }).problems.includes("duty_too_small"));
  // a plain duty run's or a measuring run's small duty is the back end's to judge
  assert.ok(!runFields({ ...M, duty: "0.00001" }).problems.includes("duty_too_small"));
  assert.ok(!runFields({ ...M, duty: "0.00001", measure: true, count_sign: "1" }).problems.includes("duty_too_small"));
});

test("the bundle's setup mark goes with a speed check only, never beside a contradicting setup", () => {
  assert.equal(runFields({ ...M, duty: "0.2", stop_turns: "0.5", count_sign: "1", same_setup: true })
    .fields.same_setup_as_bundle, true);
  assert.equal(runFields({ ...M, duty: "0.2", stop_turns: "0.5", count_sign: "1", same_setup: true,
    physical_setup: "coupled" }).fields.same_setup_as_bundle, undefined);
  assert.equal(runFields({ ...M, mode: "hold", hold_target: "here", count_sign: "1", same_setup: true })
    .fields.same_setup_as_bundle, undefined);
});

test("a hold needs its target picked, and a target change empties the value", () => {
  assert.ok(runFields({ ...M, mode: "hold", hold_target: "", count_sign: "1" }).problems.includes("hold_target"));
  assert.deepEqual(clearedOnChange("hold_target"), ["hold_value"]);
  assert.ok(clearedOnChange("axis").includes("hold_cap"));
});

test("the result view: a withheld measure, a missing distance result, the previous hold and the run's bypass", () => {
  const v = resultView({ kind: "measure", measure: { moved: -256, status: "withheld:no_reference" },
    bench_result: { result_bypass: ["steer_range"] }, previous_result: { kind: "hold", stop_reason: "end_zero",
      target: 812, hold_verdict: { result: "undecidable" } }, copy_error: "disk", verdict: { run_cleanup: { ok: true } } });
  assert.equal(v.measure.status, "withheld:no_reference");
  assert.deepEqual(v.resultBypass, ["steer_range"]);
  assert.equal(v.previous.target, 812);
  assert.equal(v.previous.holdVerdict, "undecidable");
  assert.deepEqual(resultView({ previous_result: { kind: "hold", hold_verdict: { result: "undecidable",
    why: ["판정 불가(설정·표본 없음)"] } }, verdict: { run_cleanup: { ok: true } } }).previous.holdWhy,
  ["판정 불가(설정·표본 없음)"]);
  assert.deepEqual(resultView({ rest: { result: "unconfirmed", why: "no_sample" }, verdict: { run_cleanup: { ok: true } } })
    .rest, { result: "unconfirmed", cumulative: null, why: "no_sample", note: null });
  // a reopening during the rest position: the back end sends rest null and the note at the top level (r3 P2-3)
  assert.deepEqual(resultView({ rest: null, rest_note: "reopened", verdict: { run_cleanup: { ok: true } } }).rest,
    { result: null, cumulative: null, why: null, note: "reopened" });
  assert.equal(resultView({ rest: { result: "settled_at_100hz", cumulative: 9 }, rest_note: "stop",
    verdict: { run_cleanup: { ok: true } } }).rest.note, "stop");
  assert.equal(resultView({ verdict: { run_cleanup: { ok: true } } }).rest, null);
  assert.equal(v.copyError, "disk");
  assert.equal(resultView({ kind: "distance", verdict: { run_cleanup: { ok: false } } }).distanceMissing, true);
});
