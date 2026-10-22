// Wiring between the mission queue, the control session and the browser storage (plan D6, D7). No DOM: main.js feeds
// it the connection's events and calls tick() on a timer; it sends mission_submit only when the queue says so.

import { MissionQueue, newItem, presetStatus } from "./queue.js";

export const queueKey = (vehicleId, clientId) => `console.queue.${vehicleId}.${clientId}`;
export const presetsKey = (vehicleId) => `console.presets.${vehicleId}`;

function readJson(store, key) {
  try {
    const raw = store?.getItem(key);
    return raw ? JSON.parse(raw) : null;
  } catch {
    return null;
  }
}

// session: createSession() (request(type, fields, {requestId}), forget, state, targets, welcome, hasLease, epoch)
// tabStore: sessionStorage (the queue, per tab); sharedStore: localStorage (saved transports)
export function createQueueWire({ session, tabStore, sharedStore, clientId, nowMs, nowUnix, newId }) {
  let queue = new MissionQueue();
  let presets = [];
  let vehicleId = null;
  let connected = false;
  let welcomed = false; // this connection's welcome (the session keeps the last one across a reconnect)
  let hadLease = false;
  let saveNote = null; // the screen's warning when the queue could not be stored
  let version = 0;

  const bump = () => { version += 1; };

  function save() {
    if (!vehicleId) return true;
    try {
      tabStore.setItem(queueKey(vehicleId, clientId), JSON.stringify(queue.toStorage()));
      saveNote = null;
      return true;
    } catch {
      saveNote = "저장 안 됨 — 새로고침하면 대기열을 잃습니다";
      try {
        tabStore.removeItem(queueKey(vehicleId, clientId)); // an older copy must not come back as the queue
      } catch {
        saveNote += " (이전 저장본이 남았을 수 있습니다 — 다시 열면 항목을 확인하세요)";
      }
      return false;
    }
  }

  function savePresets() {
    try {
      sharedStore.setItem(presetsKey(vehicleId), JSON.stringify(presets));
    } catch {
      /* saved transports are a convenience: keep them for this page */
    }
  }

  // every queue call that can stop it may hand back a submit to forget (plan D6)
  function apply(out) {
    if (out && out.forget !== undefined) session.forget(out.forget);
    save();
    bump();
    return out;
  }

  function ctx() {
    return { connected, welcomed: connected && welcomed, hasLease: session.hasLease,
             state: session.state, targets: session.targets };
  }

  function submit(s) {
    const epoch = queue.epoch;
    const key = { epoch, item_id: s.item_id, request_id: s.request_id };
    save(); // the submit is recorded before it goes out (plan D6)
    session.request("mission_submit", { pallet_id: s.pallet_id, destination_id: s.destination_id,
                                        targets_revision: s.targets_revision }, { requestId: s.request_id })
      .then((answer) => apply(queue.onAnswer(key, answer)));
  }

  return {
    // ------------------------------------------------------------ connection events
    onOpen() {
      connected = true;
      welcomed = false;
    },
    onWelcome(msg, results = []) {
      if (vehicleId === null && typeof msg.vehicle_id === "string") {
        vehicleId = msg.vehicle_id;
        queue = MissionQueue.fromStorage(readJson(tabStore, queueKey(vehicleId, clientId)),
                                         { epoch: msg.session_epoch, results });
        const stored = readJson(sharedStore, presetsKey(vehicleId));
        presets = Array.isArray(stored) ? stored.filter((p) => p && typeof p.preset_id === "string") : [];
      }
      connected = true;
      welcomed = true;
      apply(queue.onWelcome(msg.session_epoch));
    },
    onDisconnect() {
      connected = false;
      welcomed = false;
      hadLease = false;
      apply(queue.onDisconnect());
    },
    onState(state) {
      const out = queue.onState(state, nowMs());
      const has = session.hasLease;
      if (hadLease && !has) apply(queue.onLeaseLost());
      hadLease = has;
      if (state?.last_result) queue.onResult(state.last_result, nowMs()); // the results book carries it too
      apply(out);
    },
    // the results book's rows (contract 5.2), applied oldest first
    onResults(rows) {
      for (const r of [...rows].sort((a, b) => (a.seq ?? 0) - (b.seq ?? 0))) queue.onResult(r, nowMs());
      apply({});
    },
    // ------------------------------------------------------------ operator
    operatorAction(kind) {
      return apply(queue.operatorAction(kind));
    },
    resume() {
      const ok = queue.resume(nowMs());
      apply({});
      return ok;
    },
    stop() {
      return apply(queue.stop("운용자 멈춤"));
    },
    clear() {
      queue.clear();
      apply({});
    },
    add(fields) {
      const ok = queue.add(newItem({ kind: "fixed", ...fields }, nowUnix(), newId));
      apply({});
      return ok;
    },
    remove(itemId) {
      const ok = queue.remove(itemId);
      apply({});
      return ok;
    },
    move(itemId, delta) {
      const ok = queue.move(itemId, delta);
      apply({});
      return ok;
    },
    resolveUncertain(itemId, action) {
      const ok = queue.resolveUncertain(itemId, action);
      apply({});
      return ok;
    },
    savePreset({ label, pallet_id, destination_id }) {
      if (!vehicleId) return null;
      const preset = { preset_id: newId(), label: label || `${pallet_id} → ${destination_id}`, pallet_id, destination_id };
      presets = [...presets, preset];
      savePresets();
      bump();
      return preset;
    },
    removePreset(presetId) {
      presets = presets.filter((p) => p.preset_id !== presetId);
      savePresets();
      bump();
    },
    queuePreset(presetId) {
      const p = presets.find((x) => x.preset_id === presetId);
      if (!p || !presetStatus(p, session.targets, session.state).usable) return false;
      return this.add({ pallet_id: p.pallet_id, destination_id: p.destination_id, label: p.label });
    },
    // ------------------------------------------------------------ the timer (plan D3: 250 ms, not animation frames)
    tick() {
      apply(queue.tick(nowMs()));
      const n = queue.next(ctx(), nowMs(), newId);
      if (n) submit(n.submit);
      bump();
    },
    // ------------------------------------------------------------ for the screen
    get queue() { return queue; },
    get presets() { return presets; },
    get saveNote() { return saveNote; },
    get version() { return version + queue.version; },
    presetStatus(p) { return presetStatus(p, session.targets, session.state); },
  };
}
