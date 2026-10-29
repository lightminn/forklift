// Wiring: connections, inputs, the per-animation-frame teleop loop and rendering.
// The vehicle judges every command; this page only reports what it read and shows state.

import { drawCamera } from "./camera.js";
import { createFrameTracker } from "./frames.js";
import { chooseClientId, createTeleopGate, shouldReclaim } from "./gate.js";
import { createInputs, hiddenEdge, padSnapshot } from "./inputs.js";
import { drawMap, palletAt } from "./map.js";
import { createMapView, panBy, zoomAt, zoomPercent } from "./mapview.js";
import { decodeScan, parseFrame } from "./media.js";
import { API, createSession } from "./protocol.js";
import { createQueueWire } from "./queuewire.js";
import { AlarmBook, healthRows, queueStopText } from "./health.js";
import { log, render, renderPallets, renderTemplates, renderWizard } from "./ui.js";
import { controlProfile, disarmAsked, frameAgreement, rejectText, scanDrawable, sessionChanges, staleness, stopIsRefused } from "./profile.js";
import { blockReason, nextAllowed, selectable, templatesFor } from "./wizard.js";
import { ConfirmGate, ResultsBook, standbyText } from "./results.js";
import { candidateAt, pickKey, relocActions, relocOf } from "./reloc.js";

const TELEOP_MIN_INTERVAL_MS = 33; // ~30 Hz cap: a fast display must not flood ticks
const $ = (id) => document.getElementById(id);

function randomId() {
  return crypto.randomUUID ? crypto.randomUUID() : `${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

// One unpredictable id per tab, kept across reloads so a refresh can reclaim the lease
// inside the vehicle's grace period (contract 2); a window opened from this tab gets
// its own id (gate.chooseClientId). Storage may be unavailable.
function tabClientId() {
  let stored = null;
  try {
    stored = sessionStorage.getItem("console.client_id");
  } catch {
    stored = null;
  }
  const { id, windowName } = chooseClientId({ stored, windowName: window.name, newId: randomId });
  window.name = windowName;
  try {
    sessionStorage.setItem("console.client_id", id);
  } catch {
    /* storage unavailable: the id lives for this page only */
  }
  return id;
}

function storedFlag(key) {
  try {
    return sessionStorage.getItem(key) === "1";
  } catch {
    return false;
  }
}

function storeFlag(key, on) {
  try {
    if (on) sessionStorage.setItem(key, "1");
    else sessionStorage.removeItem(key);
  } catch {
    /* storage unavailable */
  }
}

const m = {
  results: new ResultsBook(), // plan D13
  confirm: new ConfirmGate(),
  sessionNote: null,
  standbyNote: null, // the supervisor's standby between runners (contract 6), cleared by a welcome
  sessionEnding: null,
  session: null,
  link: { up: false, tickGapMs: null, lastTickAt: null, mediaUp: false },
  welcomeAt: null, // status panel plan D2: when this control connection's welcome came (only what follows counts)
  gridWithdrawn: null, // the reason of a withdrawn grid
  standby: null, // the standby's own answer (contract 6), beside its note
  grid: null,
  scan: null, // {frame, points, at, approx}
  mediaEpoch: null,
  frameNote: null,
  chassis: null,
  cameraModel: null,
  camera: { bitmap: null, frameId: null, lastAt: null },
  teleop: { gate: createTeleopGate({
      needConfirm: storedFlag("console.need_direct_confirm"),
      videoUsed: storedFlag("console.video_used"),
    }), lastSentAt: 0, lastRead: null },
  padStopHeld: false,
  cache: { phaseKey: "", targetsRev: null, loadKey: "" },
  dropUntil: 0,
  view: createMapView(),
  layers: { grid: true, lidar: true, plan: true, sweep: true },
  wz: { open: false, step: 1, template: null, query: "", confirmed: null, note: "" },
  stateAt: null,
  linkWelcome: null, // welcome of the control connection open now; null while reconnecting
};
const tok = {};
function readTokens() {
  const cs = getComputedStyle(document.documentElement);
  for (const k of ["map-bg", "map-grid", "map-free", "obst", "lidar", "accent", "warn", "ok", "crit", "robot", "robot-ink", "dest", "teleop", "fg", "muted", "panel", "line-2", "road", "road-in", "node", "station", "station-fill"]) {
    tok[k] = cs.getPropertyValue(`--${k}`).trim();
  }
  tok.font = getComputedStyle(document.body).fontFamily;
}

let control = null;
function send(msg) {
  if (control && control.readyState === WebSocket.OPEN) control.send(JSON.stringify(msg));
}

// The camera counts as on screen only while at least half of it is inside the viewport
// (IntersectionObserver also reports a collapsed window or a hidden view as outside) and
// the tab is shown. Video teleop relies on this: the vehicle sees the displayed frame age.
let camOnScreen = false;
new IntersectionObserver(
  (entries) => {
    for (const e of entries) {
      const was = camOnScreen;
      camOnScreen = e.isIntersecting && e.intersectionRatio >= 0.5;
      if (was && !camOnScreen) cameraLeftScreen();
    }
  },
  { threshold: [0, 0.5, 1] },
).observe(document.getElementById("cam"));
const cameraVisible = () => camOnScreen && !document.hidden;
// video teleop must not go on with the camera off screen: ask the vehicle to disarm now
// (the displayed frame also stops advancing, so the vehicle would expire it anyway)
function cameraLeftScreen() {
  if (m.teleop.gate.mode === "video" && m.session.armed) {
    askDisarm();
    log("조종 준비 해제 요청 · 카메라가 화면에서 사라짐", "warn");
  }
}
const frames = createFrameTracker({ raf: (cb) => requestAnimationFrame(cb), visible: cameraVisible });
const clientId = tabClientId();
// the vehicle no longer knows this connection's request token (persistent console plan D7): a new connection gets
// a new one (the close handler reconnects)
m.session = createSession({ clientId, send, now: () => performance.now(), newId: randomId,
  onStaleToken: () => control?.close() });
// mission queue plan D7: the queue in this tab's sessionStorage, the saved transports in localStorage (either may be
// unavailable: the queue then says it is not stored)
const storage = (name) => {
  try {
    return window[name];
  } catch {
    return null;
  }
};
m.queue = createQueueWire({ session: m.session, tabStore: storage("sessionStorage"), sharedStore: storage("localStorage"),
  clientId, nowMs: () => performance.now(), nowUnix: () => Date.now() / 1000, newId: randomId });
// status panel plan D3: the alarm book, kept in this tab's sessionStorage once the vehicle is known
m.alarms = new AlarmBook();
m.health = { rows: [], key: null, saved: -1 };
try {
  if (shouldReclaim(sessionStorage.getItem("console.held_lease"), clientId)) m.session.expectReclaim();
} catch {
  /* storage unavailable */
}

m.session.subscribe((msg) => {
  if (msg.t === "welcome") {
    if (msg.session_epoch !== m.mediaEpoch) {
      // a new vehicle session: nothing of the old one stays on screen
      m.grid = null;
      m.scan = null;
      m.gridWithdrawn = null;
      m.mediaEpoch = msg.session_epoch;
      m.cache.targetsRev = null; // the same revision number in a new session is a new list
      $("pallet-list").innerHTML = "";
      $("dest").innerHTML = "";
    }
    m.results.welcome(msg.session_epoch); // plan D13: the results seq starts again in a new session
    m.welcomeAt = performance.now();
    m.standby = null;
    if (m.health.key === null && typeof msg.vehicle_id === "string") {
      m.health.key = `console.alarms.${msg.vehicle_id}.${clientId}`;
      try {
        m.alarms = AlarmBook.fromStorage(JSON.parse(sessionStorage.getItem(m.health.key) ?? "null"), Date.now() / 1000);
      } catch {
        m.alarms = new AlarmBook();
      }
    }
    m.queue.onWelcome(msg); // mission queue plan D6: a new session stops it
    standbyAsk += 1; // an answer still on its way is older than this welcome (Codex standby design P2-2)
    m.standbyNote = null;
    if (m.sessionEnding === "sim_reset" && msg.session_epoch !== m.endingEpoch) {
      m.sessionEnding = null; // the reset scene's new session is up
      m.sessionNote = null;
    }
    subscribeMedia(); // (re)subscribe now that the epoch is known: frames sent before it were dropped
    m.linkWelcome = msg;
    if (!stopIsRefused(msg)) $("nostop").hidden = true; // a vehicle that can stop: no stale notice
    frames.reset(msg.session_epoch);
    m.chassis = msg.capabilities?.chassis ?? null;
    m.cameraModel = msg.capabilities?.camera_model ?? null;
    m.camera = { bitmap: null, frameId: null, lastAt: null };
    log(`${msg.vehicle_id}에 연결 · session ${msg.session_epoch}`);
  } else if (msg.t === "tick") {
    const now = performance.now();
    if (m.link.lastTickAt !== null) m.link.tickGapMs = now - m.link.lastTickAt;
    m.link.lastTickAt = now;
    const echo = m.session.echoMessage();
    if (echo) send(echo); // keeps the lease alive (contract 2)
  } else if (msg.t === "targets") {
    if (msg.targets_revision !== m.cache.targetsRev) {
      m.cache.targetsRev = msg.targets_revision;
      renderPallets(msg, () => {}, m.session.state?.load);
      if (m.wz.open) refreshWizard();
    }
  } else if (msg.t === "pick") {
    m.queue.onPick(msg); // CT-2: the vehicle's next pallet and destination for the queue's auto item
  } else if (msg.t === "missions") {
    m.results.update(msg); // plan D13: the results book (contract 5.2)
    m.queue.onResults(msg.results ?? []);
  } else if (msg.t === "event") {
    log(`${msg.kind}${msg.reason ? ` · ${msg.reason}` : ""}${msg.phase ? ` · ${msg.phase}` : ""}${msg.status ? ` · ${msg.status}` : ""}`, msg.kind.includes("stop") ? "crit" : "");
  } else if (msg.t === "ack" && typeof msg.lease_id === "string") {
    try {
      sessionStorage.setItem("console.held_lease", clientId); // a reload right after this reclaims
    } catch {
      /* storage unavailable */
    }
  } else if (msg.t === "state") {
    m.stateAt = performance.now();
    m.queue.onState(msg); // mission queue plan D3, D4: silence, anomalies, the lease, last_result
    healthTick(); // status panel plan D3: also on each state (a background tab slows the timers, not the socket)
    const asked = disarmAsked(m.askedDisarm, m.prevState, performance.now());
    for (const c of sessionChanges(m.prevState, msg, { clientId, askedDisarm: asked })) log(c.text, c.level);
    if (m.prevState?.armed && !msg.armed) m.askedDisarm = null; // the excuse is used up by that disarm
    m.prevState = msg;
    // plan D5 6c item 7: the pallet on the forks becomes pickable when the load names it -- the targets revision does
    // not change with it, so the list is drawn again here
    const loadKey = `${msg.load?.state ?? ""}|${msg.load?.pallet_id ?? ""}`;
    if (loadKey !== m.cache.loadKey) {
      m.cache.loadKey = loadKey;
      if (m.session.targets) {
        renderPallets(m.session.targets, () => {}, msg.load);
        if (m.wz.open) refreshWizard();
      }
    }
    if (msg.mode === "TELEOP" && m.lastMode !== "TELEOP") showTab("tele"); // controls on screen when teleop starts
    m.lastMode = msg.mode;
    m.teleop.gate.onArmed(Boolean(msg.armed) && m.session.hasLease);
    storeFlag("console.need_direct_confirm", m.teleop.gate.needConfirm); // survives a reload
    storeFlag("console.video_used", m.teleop.gate.videoUsed);
    try {
      // remembered across a reload so the page reclaims its lease (contract 2)
      if (m.session.hasLease) sessionStorage.setItem("console.held_lease", clientId);
      else if (msg.lease && msg.lease.holder !== clientId) sessionStorage.removeItem("console.held_lease");
    } catch {
      /* storage unavailable */
    }
  }
});

// Between runners the supervisor's standby serves the port (contract 6): ask it where the session is. An answer is
// used only if no welcome came since and it is the newest question; it is a note, never the vehicle's state.
let standbyAsk = 0;
async function askStandby() {
  const ask = ++standbyAsk;
  const abort = new AbortController();
  const timer = setTimeout(() => abort.abort(), 2000); // the request and its body
  let note = null;
  let raw = null;
  try {
    const r = await fetch("/api/v1/session", { cache: "no-store", signal: abort.signal });
    if (r.ok) {
      raw = await r.json();
      note = standbyText(raw);
    }
  } catch {
    note = null; // no standby (or no answer in time): the reconnects go on as before
    raw = null;
  } finally {
    clearTimeout(timer);
  }
  if (ask === standbyAsk && !m.linkWelcome) {
    m.standbyNote = note;
    m.standby = raw;
  }
}

function connectControl() {
  const ws = new WebSocket(`ws://${location.host}/api/v1/control`);
  control = ws;
  ws.addEventListener("open", () => {
    m.link.up = true;
    m.queue.onOpen();
    ws.send(JSON.stringify({ t: "hello", api: API, client_id: clientId }));
  });
  ws.addEventListener("message", (ev) => {
    try {
      m.session.onMessage(JSON.parse(ev.data));
    } catch {
      /* malformed message: ignore */
    }
  });
  ws.addEventListener("close", () => {
    m.link.up = false;
    m.confirm.cancel(); // plan D13: a confirm belongs to the connection it was armed on
    m.session.onDisconnect(); // the old lease is not used again; the next welcome reclaims it
    m.queue.onDisconnect(); // mission queue plan D6: stopped, a running mission goes on
    m.linkWelcome = null; // until the next welcome, STOP is queued and resent (C1 rule)
    m.welcomeAt = null;
    if (control === ws) control = null;
    if (m.sessionEnding === "session_end") {
      // plan D13: the vehicle program ended on request and does not come back -- no reconnecting
      m.sessionNote = "세션 종료됨 · 차량 프로그램이 끝났고 다시 열리지 않습니다";
      return;
    }
    if (m.sessionEnding === "sim_reset") m.sessionNote = "장면 초기화 중 · 새 세션을 기다립니다";
    askStandby();
    const wait = Math.max(1000, m.dropUntil - performance.now());
    setTimeout(connectControl, wait);
  });
}

let media = null;
function subscribeMedia() {
  // only once the control connection has told us the session epoch; every subscribe
  // makes the vehicle send its current frames again
  if (media && media.readyState === WebSocket.OPEN && m.session.epoch !== null) {
    media.send(JSON.stringify({ t: "media_subscribe", client_id: clientId, camera_rgb: { hz: 15 }, scan: { hz: 10 }, grid: { hz: 1 } }));
  }
}

function connectMedia() {
  const ws = new WebSocket(`ws://${location.host}/api/v1/media`);
  media = ws;
  ws.binaryType = "arraybuffer";
  ws.addEventListener("open", () => {
    m.link.mediaUp = true;
    subscribeMedia();
  });
  ws.addEventListener("message", async (ev) => {
    if (!(ev.data instanceof ArrayBuffer)) return;
    const { head, body } = parseFrame(ev.data);
    const epoch = m.session.epoch;
    if (head.session_epoch !== epoch) return; // another session's frame
    if (head.t === "camera") {
      const bitmap = await createImageBitmap(new Blob([body.slice(0, head.jpeg_bytes ?? body.byteLength)], { type: "image/jpeg" }));
      // decoded: draw only if still the current session and newer than what is shown
      if (!frames.shouldDraw(head.session_epoch, head.frame_id)) {
        bitmap.close?.();
        return;
      }
      m.camera.bitmap?.close?.();
      m.camera = { bitmap, frameId: head.frame_id, lastAt: performance.now() };
      drawCamera($("cam"), { ...m.camera, state: m.session.state, cameraModel: m.cameraModel, chassis: m.chassis }, tok);
      frames.drawn(head.session_epoch, head.frame_id);
    } else if (head.t === "scan") {
      // age from the vehicle's own count: a scan resent on resubscribe is not fresh
      const at = performance.now() - (Number.isFinite(head.age_s) ? head.age_s * 1000 : 0);
      m.scan = { frame: head.frame ?? null, points: decodeScan(head, body), at, approx: head.pose_at === "latest",
                 receivedAt: performance.now() };
    } else if (head.t === "grid") {
      m.grid = head.withdrawn ? null : { head, cells: new Uint8Array(body) }; // withdrawn: drop the old map
      m.gridWithdrawn = head.withdrawn ? String(head.reason ?? "사유 없음") : null;
    }
  });
  ws.addEventListener("close", () => {
    if (media === ws) {
      media = null;
      m.link.mediaUp = false;
    }
    if (m.sessionEnding === "session_end") return; // plan D13: the vehicle program ended; nothing comes back
    setTimeout(connectMedia, 1000);
  });
}

// ------------------------------------------------------------------ STOP
// Monitor-only connection (capabilities.control === false, gateway C2a): this page cannot
// stop the truck. Say so loudly instead of sending a request that does nothing. Until a
// welcome says so, STOP is sent as before (it is kept and resent until answered).
// A disarm this page asked for (or caused: STOP, leaving TELEOP) is not logged as the vehicle's.
function noteDisarmAsked() {
  m.askedDisarm = { at: performance.now(), armGen: m.session.state?.arm_gen };
}
function askDisarm() {
  noteDisarmAsked();
  return m.session.request("disarm").then(report);
}

function pressStop(source) {
  m.queue.operatorAction("stop"); // mission queue plan D4: before anything else
  noteDisarmAsked();
  if (stopIsRefused(m.linkWelcome)) {
    $("nostop").hidden = false;
    log(`정지 불가 · 관찰 전용 연결 (${source})`, "crit");
    return;
  }
  m.session.stop(source).then(report);
}
$("nostop").onclick = () => {
  $("nostop").hidden = true;
};

// ------------------------------------------------------------------ inputs
const inputs = createInputs({
  onRelease(reason) {
    send({ t: "input_state", visible: reason !== "hidden", focused: reason !== "blur", gamepad_lost: reason === "gamepad_lost" });
    if (m.session.armed) {
      askDisarm();
      log(`조종 준비 해제 요청 · ${reason}`, "warn");
    }
  },
});

function report(r) {
  if (r && r.t === "reject") log(`거부 · ${rejectText(r.reason, r.detail)}`, "warn");
  return r;
}

function bindHold(el, onDown, onUp) {
  el.addEventListener("pointerdown", (e) => {
    if (el.disabled || el.getAttribute("aria-disabled") === "true") return;
    el.setPointerCapture(e.pointerId);
    onDown(e);
    e.preventDefault();
  });
  for (const t of ["pointerup", "pointercancel", "lostpointercapture"]) el.addEventListener(t, onUp);
  el.addEventListener("contextmenu", (e) => e.preventDefault());
}
bindHold($("deadman"), () => inputs.setTouch({ deadman: true }), () => inputs.setTouch({ deadman: false }));
const joy = $("joy");
const knob = $("knob");
function joyMove(e) {
  const rc = joy.getBoundingClientRect();
  const r = rc.width * 0.36;
  let dx = (e.clientX - (rc.left + rc.width / 2)) / r;
  let dy = (e.clientY - (rc.top + rc.height / 2)) / r;
  const n = Math.hypot(dx, dy);
  if (n > 1) {
    dx /= n;
    dy /= n;
  }
  inputs.setTouch({ x: dx, y: -dy });
  knob.style.transform = `translate(${dx * r}px, ${dy * r}px)`;
}
bindHold(joy, joyMove, () => {
  inputs.setTouch({ x: 0, y: 0 });
  knob.style.transform = "";
});
joy.addEventListener("pointermove", (e) => {
  if (joy.hasPointerCapture(e.pointerId)) joyMove(e);
});
bindHold($("lift-up"), () => inputs.setTouch({ lift: 1 }), () => inputs.setTouch({ lift: 0 }));
bindHold($("lift-dn"), () => inputs.setTouch({ lift: -1 }), () => inputs.setTouch({ lift: 0 }));

const KEYMAP = { " ": "deadman", w: "w", a: "a", s: "s", d: "d", r: "r", f: "f" };
const typing = (e) => /^(INPUT|SELECT|TEXTAREA)$/.test(e.target.tagName);
addEventListener("keydown", (e) => {
  if (e.key === "Escape") {
    pressStop("key"); // also from a text field: STOP never waits for focus
    return;
  }
  if (typing(e)) return;
  const k = KEYMAP[e.key.toLowerCase()];
  if (k) {
    inputs.setKeys({ [k]: true });
    if (k === "deadman") e.preventDefault();
  }
});
addEventListener("keyup", (e) => {
  const k = KEYMAP[e.key.toLowerCase()];
  if (k) {
    inputs.setKeys({ [k]: false });
    if (k === "deadman") e.preventDefault();
  }
});
document.addEventListener("visibilitychange", () => inputs.visibility(document.hidden));
addEventListener("gamepaddisconnected", (e) => inputs.padDisconnected(e.gamepad.index));
addEventListener("blur", () => inputs.focus(false));
addEventListener("focus", () => inputs.focus(true));

function readPad() {
  let pads = [];
  try {
    pads = navigator.getGamepads ? Array.from(navigator.getGamepads()) : [];
  } catch {
    pads = [];
  }
  // stay with the pad in use; another pad is picked only when it is gone (inputs releases)
  const same = pads.find((g) => g && g.connected && g.index === m.padIndex);
  const raw = same ?? pads.find((g) => g && g.connected);
  m.padIndex = raw ? raw.index : null;
  const snap = padSnapshot(raw ?? null);
  $("gp").textContent = raw ? (snap ? `${snap.id.slice(0, 28)} · RB 누르고 있기` : "표준 매핑이 아닌 게임패드 · 사용 안 함") : "버튼을 한 번 누르세요";
  if (snap?.buttons.stop && !m.padStopHeld) pressStop("gamepad");
  m.padStopHeld = Boolean(snap?.buttons.stop);
  return snap;
}

// ------------------------------------------------------------------ buttons
const req = (type, fields) => m.session.request(type, fields).then(report);
const missionId = () => m.session.state?.mission?.mission_id;
$("lease").onclick = () => (m.session.hasLease ? req("lease_release") : req("lease_request"));
$("stop").onclick = $("stop2").onclick = () => pressStop("console");
$("rearm").onclick = () => req("rearm");
// plan D13: the session requests, each behind a two-step confirm bound to its epoch, lease and control generation
m.confirmKey = (kind) => {
  const g = m.session.generations;
  return { kind, epoch: g.session_epoch, leaseId: g.lease_id, controlGen: g.control_gen };
};
function sessionRequest(kind) {
  const step = m.confirm.click(m.confirmKey(kind), performance.now());
  if (step !== "send") return;
  req(kind).then((r) => {
    if (r && r.t === "ack") {
      m.sessionEnding = kind;
      m.endingEpoch = m.session.generations.session_epoch;
      m.sessionNote = kind === "session_end" ? "세션을 끝내는 중" : "장면 초기화 중 · 새 세션을 기다립니다";
      log(kind === "session_end" ? "세션 종료 요청 수락" : "장면 초기화 요청 수락", "warn");
    }
  });
}
// B-P2c: the operator's pick of a start candidate (two clicks on the same candidate of the same set) and the retry
function pickCandidate(reloc, index) {
  const key = pickKey(reloc, index, m.confirmKey("relocalize_pick"));
  const step = m.confirm.click(key, performance.now());
  if (step === "armed") log(`출발 위치 후보 ${index + 1} · 한 번 더 누르면 확정합니다`);
  if (step !== "send") return;
  // the armed key's generation and candidate: the set the operator looked at (Codex B-P2c design P2)
  req("relocalize_pick", { generation: key.generation, candidate: key.candidate }).then((r) => {
    if (r && r.t === "ack") log(`출발 위치 후보 ${index + 1} 로 정했습니다 · SLAM 을 그 위치에서 시작합니다`);
  });
}
$("reloc-cands").onclick = (e) => {
  const b = e.target.closest("button[data-candidate]");
  const reloc = relocOf(m.session.state);
  if (b && reloc && relocActions(reloc, m.session.hasLease).pick) pickCandidate(reloc, Number(b.dataset.candidate));
};
$("reloc-retry").onclick = () => {
  const reloc = relocOf(m.session.state);
  if (reloc && relocActions(reloc, m.session.hasLease).retry.enabled) req("relocalize_retry", { generation: reloc.generation });
};
$("s-end").onclick = () => sessionRequest("session_end");
$("s-reset").onclick = $("scene-reset").onclick = () => sessionRequest("sim_reset");
$("m-start").onclick = () => {
  const pallet = document.querySelector('input[name="pal"]:checked')?.value;
  const targets = m.session.targets;
  if (!pallet || !targets) return;
  m.queue.operatorAction("manual_submit"); // mission queue plan D4: a manual mission stops the queue first
  // not req(): this handler logs its own refusal (one line, not two)
  m.session.request("mission_submit", { pallet_id: pallet, destination_id: $("dest").value, targets_revision: targets.targets_revision }).then((r) => {
    if (r && r.t === "reject") {
      m.wz.note = `거부 · ${rejectText(r.reason, r.detail)}`;
      log(`작업 지시 거부 · ${rejectText(r.reason, r.detail)}`, "warn");
      if (r.reason === "stale_targets") wizardGo(2); // the list changed: pick again
      else refreshWizard();
    } else if (r) closeWizard();
  });
};
$("m-pause").onclick = () => {
  m.queue.operatorAction("pause"); // mission queue plan D4: the queue stops first
  req("mission_pause", { mission_id: missionId() });
};
$("m-resume").onclick = () => req("mission_resume", { mission_id: missionId() });
$("m-cancel").onclick = () => {
  m.queue.operatorAction("cancel");
  req("mission_cancel", { mission_id: missionId() });
};

// ------------------------------------------------------------------ mission queue (plan D8, D9)
function queueConfirm(kind, then) {
  if (m.confirm.click(m.confirmKey(kind), performance.now()) === "send") then();
}
$("q-resume").onclick = () => queueConfirm("queue_resume", () => {
  if (m.queue.resume()) log("대기열 재개");
});
$("q-stop").onclick = () => m.queue.stop(); // logged by the queue timer below
$("q-clear").onclick = () => queueConfirm("queue_clear", () => m.queue.clear());
$("q-items").addEventListener("click", (e) => {
  const b = e.target.closest("button[data-act]");
  if (!b || b.disabled) return;
  const id = b.dataset.id;
  const act = b.dataset.act;
  if (act === "up" || act === "down") m.queue.move(id, act === "up" ? -1 : 1);
  else if (act === "del") m.queue.remove(id);
  else m.queue.resolveUncertain(id, act); // drop | resubmit | clear (an auto item's one submit)
});
$("q-presets").addEventListener("click", (e) => {
  const b = e.target.closest("button");
  if (!b || b.disabled) return;
  if (b.dataset.preset) {
    if (m.queue.queuePreset(b.dataset.preset)) log(`대기열에 넣음 · ${b.textContent}`);
  } else if (b.dataset.unpreset) m.queue.removePreset(b.dataset.unpreset);
});
$("q-auto").onclick = () => {
  if (m.queue.addAuto()) log("대기열에 넣음 · 남은 팔레트 전부");
};
$("q-add").onclick = () => {
  const sel = wizardSelection();
  if (!sel.pallet || !sel.destination) return;
  m.queue.add({ pallet_id: sel.pallet, destination_id: sel.destination });
  log(`대기열에 넣음 · ${sel.pallet} → ${sel.destination}`);
  closeWizard();
};
$("q-save-preset").onclick = () => {
  const sel = wizardSelection();
  if (!sel.pallet || !sel.destination || !m.session.hasLease) return;
  const p = m.queue.savePreset({ pallet_id: sel.pallet, destination_id: sel.destination });
  if (p) {
    m.wz.note = `운반 저장됨 · ${p.label}`;
    refreshWizard();
  }
};
$("recover").onclick = () => req("recovery", { action: "withdraw_straight" });
$("t-enter").onclick = () => req("mode_request", { to: "TELEOP" });
$("t-exit").onclick = () => {
  noteDisarmAsked();
  req("mode_request", { to: "PAUSED" });
};
$("t-arm").onclick = () => (m.session.armed ? askDisarm() : req("arm"));
$("m-direct").onclick = () => m.teleop.gate.setMode("direct");
$("m-video").onclick = () => {
  m.teleop.gate.setMode("video");
  storeFlag("console.video_used", true); // a reload before the disarm arrives still asks
};
$("direct-ok").onclick = () => {
  m.teleop.gate.confirmDirect();
  storeFlag("console.need_direct_confirm", false);
  storeFlag("console.video_used", m.teleop.gate.videoUsed);
  log("육안 조종 전환 · 지게차가 직접 보인다고 확인함", "warn");
};
$("l-drop").onclick = () => {
  m.dropUntil = performance.now() + 3000;
  log("끊김 시험 · 제어 연결을 3초 닫습니다", "warn");
  control?.close();
};
const local = ["localhost", "127.0.0.1", "[::1]"].includes(location.hostname);
$("gp-note").hidden = local;

// ------------------------------------------------------------------ layout: views, tabs, layers, camera window
function showView(name) {
  for (const b of document.querySelectorAll(".rail-item")) {
    if (b.dataset.view === name) b.setAttribute("aria-current", "page");
    else b.removeAttribute("aria-current");
  }
  for (const p of document.querySelectorAll("[data-view-panel]")) p.hidden = p.dataset.viewPanel !== name;
}
for (const b of document.querySelectorAll(".rail-item")) b.onclick = () => showView(b.dataset.view);

function showTab(name) {
  const tele = name === "tele";
  $("tab-detail").setAttribute("aria-selected", String(!tele));
  $("tab-tele").setAttribute("aria-selected", String(tele));
  $("pane-detail").hidden = tele;
  $("pane-tele").hidden = !tele;
}
$("tab-detail").onclick = () => showTab("detail");
$("tab-tele").onclick = () => showTab("tele");
$("go-tele").onclick = () => showTab("tele");

function setLayer(name, on) {
  m.layers[name] = on;
  const b = document.querySelector(`.layers [data-layer="${name}"]`);
  if (b) b.setAttribute("aria-pressed", String(on));
  if (name === "lidar") {
    $("lidar-on").setAttribute("aria-pressed", String(on));
    $("lidar-off").setAttribute("aria-pressed", String(!on));
  }
}
for (const b of document.querySelectorAll(".layers [data-layer]")) b.onclick = () => setLayer(b.dataset.layer, !m.layers[b.dataset.layer]);
$("lidar-on").onclick = () => setLayer("lidar", true);
$("lidar-off").onclick = () => setLayer("lidar", false);
$("cam-toggle").onclick = () => {
  const pip = $("p-cam");
  const collapsed = pip.classList.toggle("collapsed");
  $("cam-toggle").setAttribute("aria-expanded", String(!collapsed));
  $("cam-toggle").textContent = collapsed ? "+" : "–";
};

// ------------------------------------------------------------------ map: drag to pan, wheel / pinch to zoom, click a pallet
const mapCv = $("map");
const pointers = new Map();
let pinch = null;
let press = null;
const mapSize = () => {
  const rc = mapCv.getBoundingClientRect();
  return { rc, w: rc.width, h: rc.height };
};
mapCv.addEventListener("pointerdown", (e) => {
  try {
    mapCv.setPointerCapture(e.pointerId);
  } catch {
    /* pointer already gone */
  }
  pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
  press = pointers.size === 1 ? { x: e.clientX, y: e.clientY, moved: false } : null;
  if (pointers.size === 2) {
    const [a, b] = [...pointers.values()];
    pinch = { d: Math.hypot(a.x - b.x, a.y - b.y) };
  }
});
mapCv.addEventListener("pointermove", (e) => {
  const prev = pointers.get(e.pointerId);
  if (!prev || !m.view.fitted) return;
  const cur = { x: e.clientX, y: e.clientY };
  pointers.set(e.pointerId, cur);
  if (pointers.size === 1) {
    panBy(m.view, cur.x - prev.x, cur.y - prev.y);
    if (press && Math.hypot(cur.x - press.x, cur.y - press.y) > 4) press.moved = true;
  } else if (pointers.size === 2 && pinch) {
    const [a, b] = [...pointers.values()];
    const d = Math.hypot(a.x - b.x, a.y - b.y);
    const { rc, w, h } = mapSize();
    m.view.follow = false; // a zoom anchored at the fingers cannot also follow the truck
    zoomAt(m.view, w, h, (a.x + b.x) / 2 - rc.left, (a.y + b.y) / 2 - rc.top, d / Math.max(1, pinch.d));
    pinch.d = d;
  }
  $("map-follow").setAttribute("aria-pressed", String(m.view.follow));
});
function endPointer(e) {
  if (!pointers.has(e.pointerId)) return;
  pointers.delete(e.pointerId);
  if (pointers.size < 2) pinch = null;
  if (e.type === "pointerup" && press && !press.moved && pointers.size === 0) {
    const { rc, w, h } = mapSize();
    const reloc = relocOf(m.session.state);
    if (reloc) {
      // B-P2c: while the start is unknown a click picks a candidate -- twice, bound to the set it was shown in
      const k = candidateAt(m.view, w, h, reloc, e.clientX - rc.left, e.clientY - rc.top);
      if (k !== null && relocActions(reloc, m.session.hasLease).pick) pickCandidate(reloc, k);
    } else {
      const p = palletAt(m.view, w, h, m.session.targets, e.clientX - rc.left, e.clientY - rc.top);
      if (selectable(p, m.session.state?.load)) openWizard({ pallet: p.pallet_id });
    }
  }
  if (pointers.size === 0) press = null;
}
for (const t of ["pointerup", "pointercancel"]) mapCv.addEventListener(t, endPointer);
mapCv.addEventListener(
  "wheel",
  (e) => {
    if (!m.view.fitted) return;
    e.preventDefault();
    const { rc, w, h } = mapSize();
    m.view.follow = false;
    $("map-follow").setAttribute("aria-pressed", "false");
    zoomAt(m.view, w, h, e.clientX - rc.left, e.clientY - rc.top, Math.exp(-e.deltaY * 0.0015));
  },
  { passive: false },
);
$("map-fit").onclick = () => {
  m.view.fitted = false; // the next draw fits the world again
  m.view.follow = false;
  $("map-follow").setAttribute("aria-pressed", "false");
};
$("map-follow").onclick = () => {
  m.view.follow = !m.view.follow;
  if (m.view.follow) m.view.auto = false; // following sets the centre; the zoom stays where it is
  $("map-follow").setAttribute("aria-pressed", String(m.view.follow));
};

// ------------------------------------------------------------------ task assignment wizard
function wizardSelection() {
  return {
    template: m.wz.template,
    pallet: document.querySelector('input[name="pal"]:checked')?.value ?? null,
    destination: $("dest").value || null,
    revision: m.session.targets?.targets_revision ?? null,
  };
}
function blockCtx() {
  const st = m.session.state;
  return {
    state: st,
    canMission: controlProfile(m.session.welcome?.capabilities).mission,
    hasLease: m.session.hasLease,
    mode: st?.mode,
    mission: st?.mission,
  };
}
function refreshWizard() {
  if (!m.wz.open) return;
  const caps = m.session.welcome?.capabilities;
  const all = templatesFor(caps);
  if (m.wz.template && !all.some((t) => t.code === m.wz.template)) m.wz.template = null;
  const sel = wizardSelection();
  const c = m.wz.confirmed;
  if (m.wz.step === 3 && c && (c.pallet !== sel.pallet || c.destination !== sel.destination)) {
    // the reviewed targets changed under the dialog (list update): never submit a substitute
    m.wz.note = "대상 목록이 바뀌어 고른 대상이 달라졌습니다 · 다시 고르세요";
    m.wz.step = 2;
    m.wz.confirmed = null;
  }
  renderWizard(m.wz, {
    note: m.wz.note,
    nextOk: nextAllowed(m.wz.step, sel),
    selection: sel,
    template: all.find((t) => t.code === m.wz.template) ?? null,
    block: blockCtx(),
    hasLease: m.session.hasLease,
    queueRunning: m.queue.queue.run === "running",
  });
}
function drawTemplates() {
  const list = templatesFor(m.session.welcome?.capabilities, m.wz.query);
  renderTemplates(list, m.wz.template, (code) => {
    m.wz.template = code;
    drawTemplates();
    refreshWizard();
  });
}
function wizardGo(step) {
  m.wz.step = step;
  if (step === 3) {
    const sel = wizardSelection();
    m.wz.confirmed = { pallet: sel.pallet, destination: sel.destination };
  } else m.wz.confirmed = null;
  if (step === 1) drawTemplates();
  refreshWizard();
}
function openWizard({ pallet } = {}) {
  const why = blockReason(blockCtx());
  if (why) {
    log(`작업 지정 불가 · ${why}`, "warn");
    return;
  }
  const all = templatesFor(m.session.welcome?.capabilities);
  if (!m.wz.template && all.length === 1) m.wz.template = all[0].code;
  m.wz.open = true;
  m.wz.query = "";
  m.wz.note = "";
  $("wz-search").value = "";
  $("wz-why").textContent = "";
  $("wizard").hidden = false;
  for (const id of ["rail", "subbar", "body"]) document.querySelector(`.${id}`).inert = true; // STOP bar stays live
  if (pallet) {
    const input = document.querySelector(`input[name="pal"][value="${CSS.escape(pallet)}"]`);
    if (input && !input.disabled) input.checked = true;
  }
  wizardGo(pallet && m.wz.template ? 2 : 1);
}
function closeWizard() {
  m.wz.open = false;
  m.wz.confirmed = null;
  $("wizard").hidden = true;
  for (const id of ["rail", "subbar", "body"]) document.querySelector(`.${id}`).inert = false;
}
$("m-open").onclick = () => openWizard();
$("wz-close").onclick = $("wz-cancel").onclick = closeWizard;
$("wizard").addEventListener("pointerdown", (e) => {
  if (e.target === $("wizard")) closeWizard();
});
$("wz-next").onclick = () => {
  if (nextAllowed(m.wz.step, wizardSelection())) wizardGo(m.wz.step + 1);
};
$("wz-back").onclick = () => wizardGo(Math.max(1, m.wz.step - 1));
$("wz-search").addEventListener("input", (e) => {
  m.wz.query = e.target.value;
  drawTemplates();
});
$("pallet-list").addEventListener("change", () => {
  m.wz.note = "";
  refreshWizard();
});
$("dest").addEventListener("change", () => {
  m.wz.note = "";
  refreshWizard();
});

// ------------------------------------------------------------------ loops
// The teleop controls and their feedback must be on screen to drive: a tab switch, a view
// or a width change that hides them releases every input (C1 rule), whatever caused it.
let teleShown = null; // unknown until the first frame: opening on another tab hides nothing
let hiddenDisarmSent = false;
function checkTeleVisible() {
  const shown = $("pane-tele").getClientRects().length > 0;
  if (hiddenEdge(teleShown, shown)) inputs.controlsHidden();
  teleShown = shown;
  // an arm answered after the controls were hidden (a late ack) is asked back at once
  if (!shown && m.session.armed && !hiddenDisarmSent) {
    hiddenDisarmSent = true;
    askDisarm();
    log("조종 준비 해제 요청 · 조종판이 화면에 없음", "warn");
  }
  if (!m.session.armed) hiddenDisarmSent = false;
}

function frame(now) {
  checkTeleVisible();
  const st = m.session.state;
  const snap = readPad();
  inputs.setPad(snap);
  const r = inputs.read();
  m.teleop.lastRead = r;
  const allowed = m.teleop.gate.canSend() && m.session.hasLease && teleShown; // hidden controls never drive
  if (st && st.mode === "TELEOP" && m.session.armed && allowed && now - m.teleop.lastSentAt >= TELEOP_MIN_INTERVAL_MS) {
    const cmd = m.session.teleopCommand({
      mode: m.teleop.gate.mode,
      deadman: r.deadman,
      speedRatio: r.speedRatio,
      curvatureRatio: r.curvatureRatio,
      liftDir: r.liftDir,
      frameId: frames.displayedFrameId,
    });
    if (cmd) {
      send(cmd);
      m.teleop.lastSentAt = now;
    }
  }
  const agree = frameAgreement({ gridFrame: m.grid?.head.frame ?? null, poseFrame: st?.pose?.frame ?? null, scanFrame: m.scan?.frame ?? null });
  m.frameNote = agree.note;
  const scanOk = agree.scan && scanDrawable(m.scan ? { ageS: (now - m.scan.at) / 1000 } : null);
  // B-P2c: the candidate a first click armed (drawn filled, its button says once more) until sent or timed out
  const armedKey = m.confirm.armed?.key;
  m.armedCandidate = armedKey?.kind === "relocalize_pick" && m.confirm.pending(armedKey, performance.now())
    ? armedKey.candidate : null;
  drawMap(
    $("map"),
    {
      grid: m.grid,
      scanPoints: scanOk ? m.scan.points : [],
      scanApprox: Boolean(m.scan?.approx),
      state: st && !agree.pose ? { ...st, pose: null } : st,
      targets: m.session.targets,
      chassis: m.chassis,
      vehicleId: m.session.welcome?.vehicle_id,
      stale: m.stale,
      armedCandidate: m.armedCandidate,
    },
    tok,
    m.view,
    m.layers,
  );
  const zp = zoomPercent(m.view);
  $("zoom-pct").textContent = zp === null ? "—" : `${Math.round(zp)}%`;
  if (!m.camera.bitmap) drawCamera($("cam"), { state: st, cameraModel: m.cameraModel, chassis: m.chassis }, tok);
  requestAnimationFrame(frame);
}

readTokens();
setInterval(readTokens, 1000);
setInterval(() => m.session.pump(), 100);

// ------------------------------------------------------------------ status and alarms (status panel plan)
function healthTick() {
  const q = m.queue.queue;
  m.health.rows = healthRows({
    nowMs: performance.now(), welcomeAt: m.welcomeAt, link: { up: m.link.up, mediaUp: m.link.mediaUp },
    sessionEnding: m.sessionEnding, caps: m.session.welcome?.capabilities ?? null,
    monitor: stopIsRefused(m.linkWelcome), hasLease: m.session.hasLease, clientId, state: m.session.state, stateAt: m.stateAt,
    cameraAt: m.camera.lastAt, scanAt: m.scan?.at ?? null, scanReceivedAt: m.scan?.receivedAt ?? null,
    grid: { present: Boolean(m.grid), withdrawnReason: m.gridWithdrawn }, standby: m.standby,
    queue: { run: q.run, stopReason: q.stopReason, items: q.items.length, blocked: q.lastPickBlocked },
  });
  const viewing = !$("v-health").hidden && !document.hidden;
  m.alarms.update(m.health.rows, performance.now(), Date.now() / 1000, { epoch: m.session.epoch, viewing });
  if (viewing) m.alarms.markSeen();
  if (m.health.key && m.alarms.version !== m.health.saved) {
    m.health.saved = m.alarms.version;
    try {
      sessionStorage.setItem(m.health.key, JSON.stringify(m.alarms.toStorage()));
    } catch {
      /* storage unavailable: the list lives for this page only */
    }
  }
}
setInterval(healthTick, 100);
// mission queue plan D3: its own 250 ms timer (animation frames stop in a hidden tab); a stop is logged once
let queueRun = m.queue.queue.run;
setInterval(() => {
  m.queue.tick();
  const q = m.queue.queue;
  if (queueRun === "running" && q.run === "stopped") log(`대기열 멈춤 · ${queueStopText(q.stopReason)}`, q.stopReason === "완료" ? "" : "warn");
  queueRun = q.run;
}, 250);
setInterval(() => {
  render(m);
  if (m.wz.open && m.session.state?.mode && ["AUTO", "TELEOP", "STOPPED"].includes(m.session.state.mode)) closeWizard();
  refreshWizard();
  const stale = staleness({
    stateAgeS: m.stateAt === null ? null : (performance.now() - m.stateAt) / 1000,
    poseAgeS: m.session.state?.pose?.age_s ?? null,
  });
  m.stale = m.session.state ? stale.stale : false;
  $("stale-banner").hidden = !m.stale;
  $("stale-banner").textContent = stale.text;
}, 100);
connectControl();
connectMedia();
requestAnimationFrame(frame);
