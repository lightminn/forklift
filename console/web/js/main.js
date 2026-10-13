// Wiring: connections, inputs, the per-animation-frame teleop loop and rendering.
// The vehicle judges every command; this page only reports what it read and shows state.

import { drawCamera } from "./camera.js";
import { createFrameTracker } from "./frames.js";
import { chooseClientId, createTeleopGate } from "./gate.js";
import { createInputs, padSnapshot } from "./inputs.js";
import { drawMap } from "./map.js";
import { decodeScan, parseFrame } from "./media.js";
import { API, createSession } from "./protocol.js";
import { log, render, renderPallets } from "./ui.js";

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

const m = {
  session: null,
  link: { up: false, tickGapMs: null, lastTickAt: null },
  grid: null,
  scanPoints: [],
  chassis: null,
  cameraModel: null,
  camera: { bitmap: null, frameId: null, lastAt: null },
  teleop: { gate: createTeleopGate(), lastSentAt: 0, lastRead: null },
  padStopHeld: false,
  cache: { phaseKey: "", targetsRev: null },
  dropUntil: 0,
};
const tok = {};
function readTokens() {
  const cs = getComputedStyle(document.documentElement);
  for (const k of ["map-bg", "map-grid", "map-obst", "lidar", "accent", "warn", "robot", "robot-ink", "dest", "teleop", "fg", "muted", "panel"]) {
    tok[k] = cs.getPropertyValue(`--${k}`).trim();
  }
  tok.font = getComputedStyle(document.body).fontFamily;
}

let control = null;
function send(msg) {
  if (control && control.readyState === WebSocket.OPEN) control.send(JSON.stringify(msg));
}

const frames = createFrameTracker({ raf: (cb) => requestAnimationFrame(cb) });
const clientId = tabClientId();
m.session = createSession({ clientId, send, now: () => performance.now(), newId: randomId });
try {
  if (sessionStorage.getItem("console.held_lease") === "1") m.session.expectReclaim();
} catch {
  /* storage unavailable */
}

m.session.subscribe((msg) => {
  if (msg.t === "welcome") {
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
      renderPallets(msg, () => {});
    }
  } else if (msg.t === "event") {
    log(`${msg.kind}${msg.reason ? ` · ${msg.reason}` : ""}${msg.phase ? ` · ${msg.phase}` : ""}${msg.status ? ` · ${msg.status}` : ""}`, msg.kind.includes("stop") ? "crit" : "");
  } else if (msg.t === "ack" && typeof msg.lease_id === "string") {
    try {
      sessionStorage.setItem("console.held_lease", "1"); // a reload right after this reclaims
    } catch {
      /* storage unavailable */
    }
  } else if (msg.t === "state") {
    m.teleop.gate.onArmed(Boolean(msg.armed) && m.session.hasLease);
    try {
      // remembered across a reload so the page reclaims its lease (contract 2)
      if (m.session.hasLease) sessionStorage.setItem("console.held_lease", "1");
      else if (msg.lease && msg.lease.holder !== clientId) sessionStorage.removeItem("console.held_lease");
    } catch {
      /* storage unavailable */
    }
  }
});

function connectControl() {
  const ws = new WebSocket(`ws://${location.host}/api/v1/control`);
  control = ws;
  ws.addEventListener("open", () => {
    m.link.up = true;
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
    m.session.onDisconnect(); // the old lease is not used again; the next welcome reclaims it
    if (control === ws) control = null;
    const wait = Math.max(1000, m.dropUntil - performance.now());
    setTimeout(connectControl, wait);
  });
}

function connectMedia() {
  const ws = new WebSocket(`ws://${location.host}/api/v1/media`);
  ws.binaryType = "arraybuffer";
  ws.addEventListener("open", () => {
    ws.send(JSON.stringify({ t: "media_subscribe", client_id: clientId, camera_rgb: { hz: 15 }, scan: { hz: 10 }, grid: { hz: 1 } }));
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
      m.scanPoints = decodeScan(head, body);
    } else if (head.t === "grid") {
      m.grid = { head, cells: new Uint8Array(body) };
    }
  });
  ws.addEventListener("close", () => setTimeout(connectMedia, 1000));
}

// ------------------------------------------------------------------ inputs
const inputs = createInputs({
  onRelease(reason) {
    send({ t: "input_state", visible: reason !== "hidden", focused: reason !== "blur", gamepad_lost: reason === "gamepad_lost" });
    if (m.session.armed) m.session.request("disarm").then(report);
    log(`조종 준비 해제 요청 · ${reason}`, "warn");
  },
});

function report(r) {
  if (r && r.t === "reject") log(`거부 · ${r.reason}`, "warn");
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
  if (typing(e)) return;
  if (e.key === "Escape") {
    m.session.stop("key").then(report);
    return;
  }
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
addEventListener("blur", () => inputs.focus(false));
addEventListener("focus", () => inputs.focus(true));

function readPad() {
  let pads = [];
  try {
    pads = navigator.getGamepads ? Array.from(navigator.getGamepads()) : [];
  } catch {
    pads = [];
  }
  const raw = pads.find((g) => g && g.connected);
  const snap = padSnapshot(raw ?? null);
  $("gp").textContent = raw ? (snap ? `${snap.id.slice(0, 28)} · RB 누르고 있기` : "표준 매핑이 아닌 게임패드 · 사용 안 함") : "버튼을 한 번 누르세요";
  if (snap?.buttons.stop && !m.padStopHeld) m.session.stop("gamepad").then(report);
  m.padStopHeld = Boolean(snap?.buttons.stop);
  return snap;
}

// ------------------------------------------------------------------ buttons
const req = (type, fields) => m.session.request(type, fields).then(report);
const missionId = () => m.session.state?.mission?.mission_id;
$("lease").onclick = () => (m.session.hasLease ? req("lease_release") : req("lease_request"));
$("stop").onclick = $("stop2").onclick = () => m.session.stop("console").then(report);
$("rearm").onclick = () => req("rearm");
$("m-start").onclick = () => {
  const pallet = document.querySelector('input[name="pal"]:checked')?.value;
  const targets = m.session.targets;
  if (!pallet || !targets) return;
  req("mission_submit", { pallet_id: pallet, destination_id: $("dest").value, targets_revision: targets.targets_revision });
};
$("m-pause").onclick = () => req("mission_pause", { mission_id: missionId() });
$("m-resume").onclick = () => req("mission_resume", { mission_id: missionId() });
$("m-cancel").onclick = () => req("mission_cancel", { mission_id: missionId() });
$("recover").onclick = () => req("recovery", { action: "withdraw_straight" });
$("t-enter").onclick = () => req("mode_request", { to: "TELEOP" });
$("t-exit").onclick = () => req("mode_request", { to: "PAUSED" });
$("t-arm").onclick = () => req(m.session.armed ? "disarm" : "arm");
$("m-direct").onclick = () => m.teleop.gate.setMode("direct");
$("m-video").onclick = () => m.teleop.gate.setMode("video");
$("direct-ok").onclick = () => {
  m.teleop.gate.confirmDirect();
  log("육안 조종 전환 · 지게차가 직접 보인다고 확인함", "warn");
};
$("l-drop").onclick = () => {
  m.dropUntil = performance.now() + 3000;
  log("끊김 시험 · 제어 연결을 3초 닫습니다", "warn");
  control?.close();
};
const local = ["localhost", "127.0.0.1", "[::1]"].includes(location.hostname);
$("gp-note").hidden = local;

// ------------------------------------------------------------------ loops
function frame(now) {
  const st = m.session.state;
  const snap = readPad();
  inputs.setPad(snap);
  const r = inputs.read();
  m.teleop.lastRead = r;
  const allowed = m.teleop.gate.canSend() && m.session.hasLease;
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
  drawMap($("map"), { grid: m.grid, scanPoints: m.scanPoints, state: st, targets: m.session.targets, chassis: m.chassis }, tok);
  if (!m.camera.bitmap) drawCamera($("cam"), { state: st, cameraModel: m.cameraModel, chassis: m.chassis }, tok);
  requestAnimationFrame(frame);
}

readTokens();
setInterval(readTokens, 1000);
setInterval(() => m.session.pump(), 100);
setInterval(() => render(m), 100);
connectControl();
connectMedia();
requestAnimationFrame(frame);
