// The real truck's operator panel (docs/design/2026-11-19-real-safety-wiring.md 5.1, 5c-1b): the decisions, pure
// (no DOM, no sockets) -- whether the panel shows, the revision a submit names, the operator answer bound to what was
// drawn, the bypass buttons and why one is closed. The wiring lives in main.js.

import { CHECK_LABELS } from "./realstatus.js";

// the checks the operator may bypass (the vehicle's BYPASSABLE, real_loop.py)
export const BYPASSABLE = ["measurement", "odometry", "slam", "layer_timing", "operator_start", "snapshot", "imu",
  "watch_current", "watch_wheel"];
const ACTIVE = ["accepted", "preparing", "running", "paused", "resuming", "held", "cancelling", "recovering"];
const OFF = ["OFF", "LATCHED"];

// Shown once a real_status came (an autonomy executor answers; the teleop bridge, also "real", sends none).
export function panelShown(realStatus) {
  return Boolean(realStatus && realStatus.t === "real_status");
}

// The revision a submit names: the one this session's registration got, unless the vehicle's list has moved past it
// (then register again -- 5c-1b r1 P2-4). null: none usable.
export function submitRevision(registered, status, targets = null) {
  if (!registered || !status || registered.session_epoch !== status.session_epoch) return null;
  // the vehicle's newest revision from either message (r2 P2-4)
  for (const rev of [status.targets_revision, targets && targets.targets_revision]) {
    if (Number.isInteger(rev) && rev > registered.revision) return null;
  }
  return registered.revision;
}

// Whether this console may act from this status: the status is this session's and names this client as the holder
// (5c-1b r1 P2-2: an old session's or connection's status does not open the buttons).
export function statusIsMine(status, generations, clientId) {
  return Boolean(status && generations && status.session_epoch === generations.session_epoch
    && generations.lease_id !== null && status.lease && status.lease.holder === clientId);
}

// Why the operator may not toggle a bypass now (null: may): the lease, the permit off or latched, no mission.
export function closedWhy(status, hasLease) {
  if (!hasLease) return "조종 권한이 없습니다";
  if (!status) return "실차 상태를 아직 받지 못했습니다";
  if (!OFF.includes(status.permit?.state)) return "무장 중에는 바꿀 수 없습니다";
  if (ACTIVE.includes(status.mission?.status)) return "진행 중인 미션이 있습니다";
  return null;
}

// Why a submit (and the start check) may not go now: as above, and not latched or stopped (a rearm first, r1 P2-6),
// and no bypass toggle still waiting to show in real_status (r1 P2-3).
export function submitClosedWhy(status, hasLease, pendingToggle) {
  const why = closedWhy(status, hasLease);
  if (why) return why;
  if (status.permit?.state === "LATCHED" || status.mode === "STOPPED") return "재무장이 필요합니다";
  if (pendingToggle) return "우회 변경이 적용되기를 기다립니다";
  return null;
}

// Whether a toggle sent (check, on) shows in the status's bypassed set.
export function toggleShown(toggle, status) {
  if (!toggle || !status) return false;
  const on = Array.isArray(status.bypassed) && status.bypassed.includes(toggle.check);
  return on === toggle.on;
}

// One button per bypassable check: its label, whether it is bypassed now (real_status only), enabled, why not.
export function bypassButtons(status, hasLease) {
  const on = new Set(status && Array.isArray(status.bypassed) ? status.bypassed : []);
  const why = closedWhy(status, hasLease);
  return BYPASSABLE.map((check) => ({
    check, label: CHECK_LABELS[check] ?? check, bypassed: on.has(check), enabled: why === null, why,
  }));
}

// The operator answer's key, taken when the request is drawn: (session_epoch, kind, serial), or null.
export function answerKey(status) {
  const req = status && status.operator_request;
  if (!req || !Number.isInteger(req.serial)) return null;
  return { session_epoch: status.session_epoch, kind: req.kind, serial: req.serial };
}

// Whether a drawn key still names the open request in the newest status (else the click is dropped).
export function answerStillOpen(key, status) {
  const now = answerKey(status);
  return Boolean(key && now && key.session_epoch === now.session_epoch && key.kind === now.kind
    && key.serial === now.serial);
}

// The pose fields (x m, y m, yaw deg) as the request's pose [x, y, yaw rad], or null when not three finite numbers.
export function poseOf(x, y, yawDeg) {
  const v = [x, y, yawDeg].map((s) => (typeof s === "number" ? s : Number.parseFloat(s)));
  if (!v.every(Number.isFinite)) return null;
  return [v[0], v[1], (v[2] * Math.PI) / 180];
}

// What a click depends on in a status: the session, the holder, the permit and mission, the open operator request,
// the open recovery and the bypassed set (5c-1b r3 P2-1: a fresh status with the same content does not void a click).
export function statusSig(status) {
  if (!status) return null;
  const req = status.operator_request;
  // the control generation too: a second STOP moves it with the same mode (5c-1b r4 P2-1)
  return JSON.stringify([status.session_epoch, status.control_gen ?? null, status.lease ? status.lease.holder : null,
    status.permit?.state,
    status.mission?.status, status.mode, req ? [req.kind, req.serial] : null, status.field_recovery ?? null,
    status.bypassed ?? [], status.targets_revision ?? null]);
}
