// console/1 control-connection session (docs/interfaces/2026-10-13-operator-console-contract-draft.md).
// Pure module: no DOM, no sockets. The caller supplies send(), now() and newId().
// Freshness is judged by the vehicle; this module only echoes what it last received.

export const API = "console/1";
const STALE = new Set(["stale_epoch", "stale_lease", "stale_control", "stale_arm", "stale_seq"]);

export function createSession({ clientId, send, now, newId, retryMs = 500 }) {
  let epoch = null;
  let leaseId = null;
  let controlGen = null;
  let armGen = null;
  let armed = false;
  let lastTick = null;
  let seq = 0;
  let seqArmGen = null;
  let state = null;
  let welcome = null;
  let reclaim = false; // held the lease when the connection dropped
  let targets = null;
  const pending = new Map(); // request_id -> {msg, resolve, lastSent}
  const listeners = new Set();

  function emit(event) {
    for (const fn of listeners) fn(event);
  }

  function settle(id, result) {
    const p = pending.get(id);
    if (!p) return;
    pending.delete(id);
    p.resolve(result);
  }

  function dropSession(reason) {
    // every request of the old session ends, except STOP: it keeps being sent until a
    // vehicle answers (contract 1, rule 0: STOP runs whatever the generations)
    for (const [id, p] of [...pending]) {
      if (p.msg.t !== "stop") settle(id, { t: "reject", request_id: id, reason });
    }
    leaseId = null;
    lastTick = null;
    armed = false;
    controlGen = null;
    armGen = null;
    seq = 0;
    seqArmGen = null;
  }

  function onMessage(msg) {
    if (!msg || typeof msg.t !== "string") return;
    switch (msg.t) {
      case "welcome":
        if (epoch !== null && msg.session_epoch !== epoch) {
          dropSession("stale_epoch");
          reclaim = false; // a restarted vehicle has no lease to give back
        }
        epoch = msg.session_epoch;
        welcome = msg;
        if (reclaim) {
          reclaim = false;
          // a lease request sent on the old connection may be answered from the vehicle's
          // result cache with a lease that no longer exists: drop it and ask again
          for (const [id, p] of [...pending]) {
            if (p.msg.t === "lease_request") settle(id, { t: "reject", request_id: id, reason: "superseded" });
          }
          request("lease_request"); // within the vehicle's grace period (contract 2)
        }
        break;
      case "tick":
        if (msg.session_epoch === epoch) lastTick = { state_seq: msg.state_seq, token: msg.token };
        break;
      case "state":
        if (msg.session_epoch !== epoch) return;
        state = msg;
        controlGen = msg.control_gen;
        armGen = msg.arm_gen;
        armed = Boolean(msg.armed);
        if (msg.lease && msg.lease.holder !== clientId) leaseId = null;
        break;
      case "ack":
      case "reject":
        if (typeof msg.lease_id === "string") leaseId = msg.lease_id;
        if (typeof msg.arm_gen === "number") armGen = msg.arm_gen;
        settle(msg.request_id, msg);
        break;
      case "targets":
        targets = msg;
        break;
      default:
        break;
    }
    emit(msg);
  }

  function request(type, fields = {}) {
    // generations the page does not know yet are left out, never sent as null
    const msg = { t: type, request_id: newId() };
    if (epoch !== null) msg.session_epoch = epoch;
    if (leaseId !== null) msg.lease_id = leaseId;
    if (controlGen !== null) msg.control_gen = controlGen;
    Object.assign(msg, fields);
    return new Promise((resolve) => {
      pending.set(msg.request_id, { msg, resolve, lastSent: now() });
      send(msg);
    });
  }

  // Unanswered requests are resent with the same request_id; a stale reject ends the
  // request and is returned as is (contract 1: never auto-resend with new generations).
  function pump() {
    const t = now();
    for (const p of pending.values()) {
      if (t - p.lastSent >= retryMs) {
        p.lastSent = t;
        send(p.msg);
      }
    }
  }

  // STOP needs no lease; like other reliable requests it is resent with the same
  // request_id until answered (the vehicle executes every copy, which is safe).
  function stop(source) {
    const msg = { t: "stop", request_id: newId(), source };
    if (epoch !== null) msg.session_epoch = epoch; // never null: the vehicle runs STOP without it
    return new Promise((resolve) => {
      pending.set(msg.request_id, { msg, resolve, lastSent: now() });
      send(msg);
    });
  }

  // The control connection closed: the old lease may not be used any more.
  function onDisconnect() {
    const asking = [...pending.values()].some((p) => p.msg.t === "lease_request");
    if (leaseId !== null || asking) reclaim = true;
    leaseId = null;
    lastTick = null;
    armed = false;
  }

  function teleopCommand({ mode, deadman, speedRatio, curvatureRatio, liftDir, frameId }) {
    if (!armed || lastTick === null || armGen === null || leaseId === null) return null;
    if (seqArmGen !== armGen) {
      seqArmGen = armGen;
      seq = 0;
    }
    seq += 1;
    const msg = {
      t: "teleop",
      session_epoch: epoch,
      lease_id: leaseId,
      control_gen: controlGen,
      arm_gen: armGen,
      seq,
      echo: { ...lastTick },
      mode,
      deadman: Boolean(deadman),
      speed_ratio: clampRatio(speedRatio),
      curvature_ratio: clampRatio(curvatureRatio),
      lift_dir: liftDir > 0 ? 1 : liftDir < 0 ? -1 : 0,
    };
    if (mode === "video" && Number.isInteger(frameId)) msg.frame_echo = { frame_id: frameId };
    return msg;
  }

  function echoMessage() {
    if (lastTick === null || leaseId === null) return null;
    return { t: "echo", session_epoch: epoch, lease_id: leaseId, echo: { ...lastTick } };
  }

  return {
    onMessage,
    onDisconnect,
    expectReclaim() {
      reclaim = true; // a reloaded page that held the lease
    },
    request,
    pump,
    stop,
    teleopCommand,
    echoMessage,
    subscribe(fn) {
      listeners.add(fn);
      return () => listeners.delete(fn);
    },
    isStale: (reason) => STALE.has(reason),
    get state() {
      return state;
    },
    get welcome() {
      return welcome;
    },
    get targets() {
      return targets;
    },
    get generations() {
      return { session_epoch: epoch, lease_id: leaseId, control_gen: controlGen, arm_gen: armGen };
    },
    get hasLease() {
      return leaseId !== null;
    },
    get armed() {
      return armed;
    },
    get epoch() {
      return epoch;
    },
  };
}

function clampRatio(v) {
  const x = Number.isFinite(v) ? v : 0;
  return Math.max(-1, Math.min(1, x));
}
