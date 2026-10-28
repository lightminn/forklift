// What this connection can do (welcome.capabilities, contract 3) and whether the
// picture on screen is still current. Display rules only; the vehicle decides.

// control: false is the monitor-only profile (gateway C2a): nothing on this page acts,
// STOP included.
export function controlProfile(capabilities) {
  if (!capabilities) return { control: false, mission: false, teleop: false, lift: false };
  const control = capabilities.control !== false;
  return {
    control,
    mission: control && capabilities.mission === true,
    teleop: control && (capabilities.teleop_direct === true || capabilities.teleop_video === true),
    lift: control && capabilities.lift === true,
  };
}

export const STALE_S = 1.0;

// stateAgeS: since the browser last received a state; poseAgeS: how old the pose already
// was when the vehicle sent that state (null when it does not say). The pose on screen is
// as old as both together.
export function staleness({ stateAgeS, poseAgeS }) {
  if (stateAgeS === null || stateAgeS === undefined) return { stale: true, text: "상태를 받지 못했습니다" };
  const age = stateAgeS + (poseAgeS ?? 0);
  if (age <= STALE_S) return { stale: false, text: "" };
  return { stale: true, text: `위치 ${age.toFixed(1)} s 전 · 지금 위치가 아닐 수 있습니다` };
}

// linkWelcome is the welcome of the control connection that is open now (null while
// reconnecting). Only that connection's own word turns STOP into the "cannot stop" notice;
// an old monitor-only welcome must never swallow a STOP meant for a vehicle that can stop.
export function stopIsRefused(linkWelcome) {
  return linkWelcome?.capabilities?.control === false;
}

// A pose or scan in another frame than the grid would be drawn misaligned (e.g. odom
// before map -> odom is known). Draw each only in the grid's frame; with no grid, draw
// the pose alone and a scan only in the pose's frame.
export function frameAgreement({ gridFrame, poseFrame, scanFrame }) {
  const base = gridFrame ?? poseFrame ?? null;
  const pose = !poseFrame || !gridFrame || poseFrame === gridFrame;
  const scan = Boolean(scanFrame) && (base === null || scanFrame === base);
  const note = gridFrame && poseFrame && poseFrame !== gridFrame ? `위치가 ${poseFrame} 좌표라 ${gridFrame} 지도 위에 그리지 않습니다` : null;
  return { pose, scan, note };
}

export function scanDrawable(scan) {
  return Boolean(scan) && scan.ageS <= STALE_S;
}

// What to do instead of STOP on a monitor-only link: the vehicle says (gateway
// --stop-instruction); without it, a neutral line rather than a device that may not exist.
export function stopInstruction(linkWelcome) {
  const text = linkWelcome?.capabilities?.stop_instruction;
  return typeof text === "string" && text.trim() ? text.trim() : "차량에서 직접 멈추는 수단을 쓰세요";
}

// Resume steps as [{name, result, reason}]: the fake vehicle sends that list, the live
// runner a map {name: "ok" | "pending" | "failed(reason)"} (both read contract 6).
export function resumeSteps(steps) {
  if (!steps) return null;
  if (Array.isArray(steps)) return steps;
  return Object.entries(steps).map(([name, value]) => {
    const m = /^failed\((.*)\)$/.exec(String(value));
    return { name, result: m ? "failed" : String(value), reason: m ? m[1] : null };
  });
}

// Rearm, entering TELEOP, a mission submit and a resume are refused (not_stopped) until the
// vehicle says the truck stands; a vehicle that does not report it judges the request.
// waivable: rearm and TELEOP entry, which the vehicle takes unconfirmed while it waives the stand
// (stand_waiver "lower_only": only lowering is allowed and the truck is still, plan D14).
export function standWhy(st, { waivable = false } = {}) {
  if (waivable && st?.stand_waiver === "lower_only") return null;
  return st?.stopped_confirmed === false ? "정지 확인을 기다리는 중" : null;
}

// The rearm button: shown while STOPPED, usable with the lease once the truck stands.
export function rearmState(st, hasLease) {
  const shown = st?.mode === "STOPPED";
  if (!shown) return { shown, enabled: false, why: "" };
  if (!hasLease) return { shown, enabled: false, why: "조종 권한이 필요합니다" };
  const stand = standWhy(st, { waivable: true });
  if (stand) return { shown, enabled: false, why: stand };
  return { shown, enabled: true, why: "" };
}

// Resume: a paused or held mission, outside STOPPED and TELEOP, once the truck stands.
// hint: a cancel that became a recovery (forks in a pocket, plan D4) is paused by the PAUSED
// mode at once and moves only on a resume, which then finishes the cancel (U30 pc15).
export function resumeState({ state, mission, hasLease }) {
  const mode = state?.mode;
  if (!hasLease || !mission || !["paused", "held"].includes(mission.status) || mode === "STOPPED" || mode === "TELEOP") {
    return { enabled: false, why: "", hint: "" };
  }
  const phase = String(mission.phase || "");
  const hint = phase.startsWith("recover_") || phase === "clear_out" ? "재개하면 복구를 이어 가 취소를 마칩니다" : "";
  const stand = standWhy(state);
  return stand ? { enabled: false, why: stand, hint } : { enabled: true, why: "", hint };
}

// The fake vehicle names stops controlled_decel / safety_preempt, the live arbiter controlled / preempt.
const STOP_KINDS = {
  none: { label: "정지 종류 없음", level: "" },
  controlled: { label: "제어 감속", level: "warn" },
  controlled_decel: { label: "제어 감속", level: "warn" },
  preempt: { label: "안전 선점", level: "crit" },
  safety_preempt: { label: "안전 선점", level: "crit" },
};
export function stopKind(kind) {
  if (kind === null || kind === undefined) return { label: "정지 종류 —", level: "" };
  return STOP_KINDS[kind] ?? { label: String(kind), level: "" };
}

// The live runner names a pallet kind after its geometry file (pallet_geometry_epal6).
const PALLET_KINDS = { epal6: "EPAL 6", t11_06: "T11 ×0.6" };
export function palletKindLabel(kind) {
  if (!kind) return "종류 모름";
  const k = String(kind).replace(/^pallet_geometry_/, "");
  return PALLET_KINDS[k] ?? k;
}

// The drive permit: its limit may be unknown (null) while the reason is known.
// forbidReason: the state's teleop_forbidden_reason, which says why the arbiter forbade driving
export function permitText(permit, forbidReason = null) {
  if (!permit) return "판정 없음";
  const lim = Number.isFinite(permit.limit_mps) ? `${permit.limit_mps.toFixed(2)} m/s` : "모름";
  const why = permit.reason === "drive_forbidden" ? forbidText(forbidReason) || "수동 주행 금지" : permit.reason;
  return `${why} · 상한 ${lim}`;
}

// Why manual driving is refused: the fake vehicle's reasons and the live arbiter's policy (teleop_policy.py).
const FORBID = {
  state_forbids_teleop: "근접·삽입·승강 중이거나 포크가 포켓 안에 있어 수동 조종을 받지 않습니다",
  stopped_locked: "정지 상태입니다. 재무장해야 합니다",
  fork_in_pocket: "포크가 포켓 안에 있어 수동 조종을 받지 않습니다",
  near_section: "팔레트 근접 구간이라 수동 조종을 받지 않습니다",
  near_section_unknown: "근접 구간인지 판단할 수 없어 수동 조종을 받지 않습니다",
  state_unknown: "적재·포크 상태를 알 수 없어 수동 조종을 받지 않습니다",
  pacing: "시뮬레이션이 실시간을 따라가지 못해 수동 조종을 받지 않습니다",
  // persistent console plan D14: the recovery rows
  "recovery:lower_first": "포크가 들려 있습니다. 먼저 내려 주세요(내리기만 됩니다)",
  "recovery:ground_wait": "팔레트가 바닥에 놓였는지 확인하는 중입니다",
  "recovery:evidence": "삽입 기록을 믿을 수 없어 손으로 빼기를 열지 않습니다",
  "recovery:misaligned": "포켓 축에서 벗어났습니다. 이대로 빼면 팔레트가 끌립니다",
  "recovery:unknown": "팔레트 상태를 알 수 없어 내리기만 됩니다",
  "recovery:no_lift_measure": "승강 높이를 잴 수 없어 손 복구를 열지 않습니다",
  "recovery:no_steering": "조향각을 잴 수 없어 손으로 빼기를 열지 않습니다",
  "recovery:steering": "조향이 아직 0 으로 돌아오지 않아 멈춰 있습니다",
  "recovery:stage_unknown": "복구 단계를 알 수 없어 수동 조종을 받지 않습니다",
};

// Plan D14: which lift buttons work (the policy's directions) and the travel shown beside the height.
export function liftControls(st) {
  const dirs = Array.isArray(st?.lift_dirs) ? st.lift_dirs : st?.lift_allowed ? [-1, 1] : [];
  const limit = Number.isFinite(st?.lift_limit_m) ? st.lift_limit_m : 0.28;
  return { up: dirs.includes(1), down: dirs.includes(-1), limitMm: Math.round(limit * 1000) };
}

// Plan D14: the straight reverse of a manual recovery is said wherever the drive is shown.
export function recoveryNote(st) {
  if (st?.drive_mode !== "reverse_straight") return "";
  return st.recovery?.stage === "manual_clear"
    ? "복구 후진(곧게) · 팔레트 구역을 벗어날 때까지"
    : "복구 후진(곧게, 인증 없음) · 전진·조향 없음";
}
export function forbidText(reason) {
  if (!reason) return "";
  if (FORBID[reason]) return FORBID[reason];
  const fb = /^no_feedback:(.+)$/.exec(reason);
  return fb ? `차량 피드백(${fb[1]})이 끊겨 수동 조종을 받지 않습니다` : reason;
}

// The page's "video stalled" notice in wall seconds: a slowed simulation (pacing target f < 1)
// sends its sim-rate camera 1/f times slower and the vehicle scales its own video limit by 1/f.
export function videoStaleAfterS(st) {
  const f = st?.pacing?.target_factor;
  return Number.isFinite(f) && f > 0 && f < 1 ? 1.0 / f : 1.0;
}

// What changed for this page between two states of one session that the operator should
// read in the log: the lease left this page, or the vehicle disarmed without being asked.
export function sessionChanges(prev, cur, { clientId, askedDisarm = false } = {}) {
  if (!prev || !cur || prev.session_epoch !== cur.session_epoch) return [];
  const out = [];
  if (prev.lease?.holder === clientId && cur.lease?.holder !== clientId) {
    out.push({ text: "조종 권한이 이 기기에서 풀렸습니다 (만료·해제·다른 기기)", level: "warn" });
  }
  if (prev.armed && !cur.armed && !askedDisarm) {
    out.push({ text: "차량이 조종 준비를 풀었습니다", level: "warn" });
  }
  return out;
}

// The camera's height for the display overlay: a carriage camera (rides_lift) moves with the
// lift, so its lift-0 placement plus the measured lift; null (draw nothing) while that is unknown.
export function cameraHeightM(model, st) {
  const z = model?.base_from_camera?.xyz_m?.[2];
  if (!Number.isFinite(z)) return null;
  if (!model.rides_lift) return z;
  return Number.isFinite(st?.lift_m) ? Math.round((z + st.lift_m) * 1e9) / 1e9 : null;
}

// A disarm this page asked for is excused only in the arm period it was asked in, and
// only for a few seconds; a later vehicle disarm after a re-arm is logged.
export function disarmAsked(asked, prev, nowMs) {
  return Boolean(asked) && prev?.arm_gen === asked.armGen && nowMs - asked.at < 3000;
}

// A refused request's reason as the operator reads it; an unknown code is shown as it is. Every reason the vehicle can
// send is here (tests/unit/console/test_reject_reasons.py reads them from the arbiter and the mission commands); a
// reason ending in ":" is a head (no_feedback:rear_axle_speed).
export const REJECT = {
  busy: "차량이 요청을 너무 많이 받고 있습니다 · 잠시 뒤 다시 보내세요",
  invalid: "요청이 올바르지 않습니다(없는 대상이거나 형식 오류)",
  lease_reserved: "끊긴 기기가 조종 권한을 되찾을 수 있는 시간입니다",
  mission_active: "진행 중인 미션이 있습니다",
  near_tracking_active: "근접 추적 중에는 일시정지할 수 없습니다",
  "no_feedback:": "차량 피드백(측정 속도)이 없습니다",
  no_lease: "조종 권한이 없습니다",
  not_stopped: "차가 멈췄는지 아직 확인되지 않았습니다",
  session_ending: "세션을 끝내는 중입니다",
  stale: "오래된 요청입니다(연결이 바뀌었거나 그 기록이 지워졌습니다)",
  stale_control: "그사이 제어 상태가 바뀌었습니다 · 화면을 보고 다시 보내세요",
  stale_epoch: "차량 세션이 바뀌었습니다",
  stale_lease: "조종 권한이 바뀌었습니다",
  stale_targets: "대상 목록이 바뀌었습니다 · 다시 고르세요",
  state_forbids_teleop: "지금 상태에서는 수동 조종을 할 수 없습니다",
  stopped_locked: "정지됨 상태입니다 · 재무장을 먼저 하세요",
  unknown_mission: "그 미션은 지금 미션이 아닙니다",
  unsupported: "이 차량이 지원하지 않는 요청입니다",
  stopping: "차가 멈추는 중입니다",
  scene_damaged: "장면이 손상되어 자동 작업을 받지 않습니다 · 장면 초기화가 필요합니다",
  recovery_required: "포크가 팔레트 안에 있거나 위치 고정 중이라 받을 수 없습니다 · 수동 복구를 먼저 하세요",
  cargo_held: "포크에 팔레트가 실려 있습니다",
  localization_lost: "위치 추정 준비 중입니다",
  already_there: "그 팔레트는 이미 그 목적지에 있습니다",
  destination_occupied: "목적지를 다른 팔레트가 차지하고 있습니다",
  blocked: "다른 팔레트나 소품이 집거나 내려놓는 길을 막습니다", // a manual re-pick (2026-10-28), the blockers in detail
  held: "수동 조종으로 보류 중입니다",
  not_running: "실행 중인 미션이 없습니다",
  not_paused: "일시정지된 미션이 없습니다",
  recovering: "복구 중에는 취소 대신 일시정지하세요",
  not_held: "보류 중이 아닙니다",
  "permission_denied:": "주행 허가가 거절했습니다",
  // the fake vehicle's own (console/mock_vehicle)
  fork_in_pocket: "포크가 팔레트 포켓 안에 있습니다",
  load_present: "포크에 짐이 실려 있습니다",
  resume_failed: "재개 확인에 실패했습니다",
};
// a blocked submit's detail, "pickup:P-2,delivery:P-1,props" (the vehicle's next_pick.pair_blockers)
const BLOCKER = { pickup: "집는 길", delivery: "내려놓는 길" };
function blockersText(detail) {
  return String(detail).split(",").filter(Boolean).map((b) => {
    if (b === "props") return "고정 소품";
    const [kind, id] = b.split(":");
    return BLOCKER[kind] ? `${BLOCKER[kind]}: ${id}` : b;
  }).join(" · ");
}
export function rejectText(reason, detail = null) {
  if (!reason) return reason;
  let text;
  if (REJECT[reason]) text = REJECT[reason];
  else {
    const head = Object.keys(REJECT).find((k) => k.endsWith(":") && reason.startsWith(k));
    text = head ? `${REJECT[head]} (${reason.slice(head.length)})` : reason;
  }
  return reason === "blocked" && detail ? `${text} (${blockersText(detail)})` : text;
}
