// The real truck's operator panel (docs/design/2026-11-19-real-safety-wiring.md 5.1, 5c-1b): the decisions, pure
// (no DOM, no sockets) -- whether the panel shows, the revision a submit names, the operator answer bound to what was
// drawn, the bypass buttons and why one is closed. The wiring lives in main.js.

import { CHECK_LABELS } from "./realstatus.js";

// the checks the operator may bypass (the vehicle's BYPASSABLE, real_loop.py)
export const BYPASSABLE = ["measurement", "odometry", "slam", "layer_timing", "operator_start", "snapshot", "imu",
  "watch_current", "watch_wheel", "start_pose", "d5_pocket", "f1"];
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
  // 6a: a mission collection failed on the vehicle -- no new mission until the executor restarts
  if (status.restart_required) return "임무 정리에 실패했습니다 · 실행기를 다시 시작하세요";
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
  return BYPASSABLE.map((check) => {
    // U3c (Codex round 1 P3-3): what the vehicle refuses on this run is closed here too (real_loop
    // OPERATING_NOT_BYPASSABLE; the per-pallet confirmation without depth, plan 2.6)
    const never = (status && status.operating && check === "layer_timing")
      || (check === "operator_start" && status && Number.isInteger(status.register_serial));
    const w = never ? "이 실행에서는 우회할 수 없습니다" : why;
    return { check, label: CHECK_LABELS[check] ?? check, bypassed: on.has(check), enabled: w === null, why: w };
  });
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
    status.bypassed ?? [], status.targets_revision ?? null,
    status.relocalize_generation ?? null,  // 5c-2: a press is bound to the generation drawn (design 6, 9)
    status.register_serial ?? null]);  // U3c: a pallet confirmation is bound to the registration drawn
}

// 5c-2 (design 6, 8-9): the relocalization row. Shown while a generation is open or a last attempt is to be told.
export function relocalizeView(status) {
  const gen = status && Number.isInteger(status.relocalize_generation) ? status.relocalize_generation : null;
  const r = (status && status.relocalize) || {};
  return {
    open: gen !== null, generation: gen, standing: Boolean(status && status.standing),
    state: r.state ?? "idle", reason: r.reason ?? null, attempt: r.attempt ?? null, kind: r.kind ?? null,
    lastGeneration: r.generation ?? null,
    // the last attempt's gap belongs to its own generation: not shown under a newer one (impl round 1 P2-4)
    gap: Number.isFinite(r.start_pose_m) && (gen === null || r.generation === gen)
      ? `${(r.start_pose_m * 100).toFixed(1)} cm · ${r.start_pose_deg.toFixed(1)}°` : "",
  };
}

// PR 7 design 6.1, 2: the session's manual-only latch (a declaration ended on a failed match). Read from the latch and
// the slam bypass, never from the SLAM state now: a later ready seed (after an MCU reboot) does not clear it.
export function manualOnlyNotice(status) {
  if (!status || !status.manual_only) return "";
  if (status.manual_only_blocking) {
    return "위치 추정 실패 — 자율은 세션을 다시 시작해야 함(실제 SLAM 응답 없이는 slam 우회로도 출발 불가)";
  }
  return "우회 중: slam(not_localized)";
}

// Why a relocalization button may not be pressed now (null: may): the lease, disarmed, no mission, standing.
export function relocalizeClosedWhy(status, hasLease) {
  const why = closedWhy(status, hasLease);
  if (why) return why;
  if (!Number.isInteger(status.relocalize_generation)) return "다시 확인할 위치가 없습니다";
  if (!status.standing) return "차가 서 있지 않습니다(정지를 기다립니다)";
  return null;
}

// The attempt an ack named (generation, attempt) as real_status tells it: its state, or null while not told yet.
export function relocalizeResult(ack, status) {
  const r = status && status.relocalize;
  // the session too: a reconnect's new session may reuse the generation and attempt numbers (impl round 1 P2-4)
  if (!ack || !r || ack.session_epoch !== status.session_epoch || r.generation !== ack.generation
    || r.attempt !== ack.attempt) return null;
  const gen = Number.isInteger(status.relocalize_generation) ? status.relocalize_generation : null;
  if (gen !== null && gen !== ack.generation) return null; // an older generation's verdict under a newer one
  return { state: r.state, reason: r.reason ?? null };
}

// U3c (docs/plans/2026-11-26-real-operating-assembly.md 2.9): the operating run's precondition rows -- the assembly's
// passed items as one grey line, then C7 IMU, C8 D5 and the pallet confirmation, and the scan source's newest refusal.
// Each row: {key, cls: "pass" | "blocked" | "bypassed" | "note", text}. Empty when the status has no preconditions
// (HIL, replay). rejectText(reason) turns a refusal into the operator's words (profile.js).
const RUNTIME_LABELS = { imu: "IMU", d5_pocket: "D5 포켓", pallet_confirm: "팔레트 확인", scan: "스캔" };
const BYPASS_NOTE = { imu: " (기울기 감시 꺼짐)" };
export function preconditionRows(status, rejectText = (r) => r) {
  const pre = status && status.preconditions;
  if (!pre || typeof pre !== "object") return [];
  const out = [];
  const passed = Array.isArray(pre.passed) ? pre.passed : [];
  if (passed.length) out.push({ key: "passed", cls: "pass", text: `조립 통과: ${passed.join(" · ")}` });
  for (const r of Array.isArray(pre.runtime) ? pre.runtime : []) {
    const label = `${r.id} ${RUNTIME_LABELS[r.item] ?? r.item}`;
    const key = `${r.id}:${r.item}`;
    if (r.state === "note") {
      out.push({ key, cls: "note", text: r.why ? `${label} · 최근 거부: ${rejectText(r.why)}` : `${label} · 거부 없음` });
    } else if (r.state === "bypassed") {
      out.push({ key, cls: "bypassed", text: `우회 중: ${label}${BYPASS_NOTE[r.item] ?? ""}` });
    } else if (r.state === "pass") {
      out.push({ key, cls: "pass", text: `${label} 통과` });
    } else {
      // anything else (blocked, or a state this console does not know) reads as blocked
      out.push({ key, cls: "blocked", text: `${label} 막힘${r.why ? ` · ${rejectText(r.why)}` : ""}` });
    }
  }
  return out;
}

// U3c (plan 2.6): the six lines the operator checks before the pallet confirmation, or null when this run needs no
// per-pallet confirmation (register_serial is set only on the operating run without depth). The fourth shows the lift
// target and the sixth the C11 test pallet and mass from the record the assembly judged ("—" where it gave none).
export function palletConfirmTexts(status) {
  if (!status || !Number.isInteger(status.register_serial)) return null;
  const load = (status.preconditions && status.preconditions.load) || {};
  // the approved values exactly as the record gives them -- a rounded mass or height is another limit (round 1 P2-1)
  const num = (v) => (Number.isFinite(v) ? String(v) : "—");
  return [
    "포크가 포켓 밖에 있음",
    "포켓 둘이 비어 있음",
    "팔레트 윗면과 등록 사각형 안에 다른 물체 없음",
    `승강 목표 ${num(load.lift_target_m)} m + 팔레트 높이까지 위 공간 비어 있음`,
    "빈 포크·승강 내림",
    `C11 에서 승인한 시험 팔레트 ${load.test_pallet ? String(load.test_pallet) : "—"}, 질량 ${num(load.pallet_mass_kg)} kg 이하`,
  ];
}

// U3c (plan 2.6): the operator_start request's fields. On a run without the per-pallet confirmation: {} (as before).
// Otherwise the registration this console made, still the vehicle's current one (same session, revision and
// register serial), as {pallet_id, targets_revision, register_serial}; null when there is none to confirm.
export function startFields(registered, status, targets = null) {
  if (!status) return null;
  if (!Number.isInteger(status.register_serial)) return {};
  if (!registered || registered.session_epoch !== status.session_epoch || !Number.isInteger(registered.serial)
    || registered.serial !== status.register_serial || registered.revision !== status.targets_revision) return null;
  // a newer list from either message (as submitRevision, Codex round 1 P3-1)
  if (targets && Number.isInteger(targets.targets_revision) && targets.targets_revision > registered.revision) return null;
  return { pallet_id: registered.pallet, targets_revision: registered.revision, register_serial: registered.serial };
}

// U3c: the start check's state line -- what the vehicle bound (pallet, register serial, the bypass region), or the
// plain "확인됨" on a run without the per-pallet confirmation.
export function startStateText(status) {
  if (!status || !status.operator_start) return "";
  const b = status.operator_start_binding;
  if (!b || !Array.isArray(b.region) || b.region.length !== 5) return "확인됨";
  const [x, y, len, wid, yaw] = b.region.map(Number);
  return `확인됨 · ${b.pallet_id} · 등록 #${b.register_serial} · 영역 중심 (${x.toFixed(2)}, ${y.toFixed(2)}) m`
    + ` · ${len.toFixed(2)}×${wid.toFixed(2)} m · ${((yaw * 180) / Math.PI).toFixed(1)}°`;
}

// U3c (Codex round 1 P1-2): the region the confirmation would bind, shown before the six lines are ticked -- the
// vehicle's own d5_region of this console's registration (real_status.register_region), or null when the status's
// region is not of that registration (another serial, revision or pallet).
export function confirmRegion(registered, status) {
  const r = status && status.register_region;
  if (!r || !registered || !Array.isArray(r.region) || r.region.length !== 5) return null;
  if (r.register_serial !== registered.serial || r.targets_revision !== registered.revision
    || r.pallet_id !== registered.pallet) return null;
  return r.region.map(Number);
}

export function regionText(region) {
  if (!region) return "";
  const [x, y, len, wid, yaw] = region;
  return `확인할 영역(팔레트 + 띠, 지도에 주황 점선) · 중심 (${x.toFixed(2)}, ${y.toFixed(2)}) m · `
    + `${len.toFixed(2)}×${wid.toFixed(2)} m · ${((yaw * 180) / Math.PI).toFixed(1)}°`;
}

// What the six ticks are bound to (Codex round 1 P1-1): the session, the lease, the registration (pallet, pose,
// revision, serial), the relocalization generation and the six lines -- any change unticks them.
export function checksKey(status, generations, registered, texts) {
  if (!status || !texts || !registered) return "";
  return JSON.stringify([status.session_epoch, generations ? generations.lease_id : null, registered.pallet,
    registered.pose, registered.revision, registered.serial, status.register_serial,
    status.relocalize_generation ?? null, texts]);
}
