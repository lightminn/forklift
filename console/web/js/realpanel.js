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

// The revision a submit names: the newest of the registration's ack and the target list (null: none known).
export function submitRevision(registered, targets) {
  const a = Number.isInteger(registered) ? registered : null;
  const b = targets && Number.isInteger(targets.targets_revision) ? targets.targets_revision : null;
  if (a === null) return b;
  if (b === null) return a;
  return Math.max(a, b);
}

// Why the operator may not act now (null: may): the lease, the permit off, no mission under way.
export function closedWhy(status, hasLease) {
  if (!hasLease) return "조종 권한이 없습니다";
  if (!status) return "실차 상태를 아직 받지 못했습니다";
  if (!OFF.includes(status.permit?.state)) return "무장 중에는 바꿀 수 없습니다";
  if (ACTIVE.includes(status.mission?.status)) return "진행 중인 미션이 있습니다";
  return null;
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
