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
  if (state?.start_block) {  // the vehicle-side F1 block (plan 2026-11-08 2.4)
    return { enabled: false, why: "출발 위치가 확인되지 않았습니다 · 자동 운행과 미션은 받지 않습니다", hint };
  }
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
  real_loaded: "실차에서는 팔레트를 실은 채 수동 조종하지 않습니다",
  near_section_unknown: "근접 구간인지 판단할 수 없어 수동 조종을 받지 않습니다",
  state_unknown: "적재·포크 상태를 알 수 없어 수동 조종을 받지 않습니다",
  // docs/validation/2026-11-12-teleop-smooth.md: only the tested setting is named (the registration's section 3)
  pacing: "시뮬레이션이 실시간을 따라가지 못해 수동 조종을 받지 않습니다 · 콘솔 기본 배속(0.4, catch-up 0)에서 직접 조작이 끊기지 않음을 확인했습니다(line_m 장면)",
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
  // the unconfirmed position check (docs/plans/2026-11-02-unconfirmed-runner.md 2.3)
  pose_unconfirmed: "출발 위치가 확인되지 않았습니다 · 곧은 후진만 됩니다",
  "pose_unconfirmed:stroke": "한 번 누름의 후진 거리를 다 썼습니다 · 차가 선 뒤 손을 뗐다가 다시 누르세요",
  "pose_unconfirmed:total": "확인 전 후진 거리를 다 썼습니다",
  pose_checking: "출발 위치를 확인하는 중입니다",
  pose_ended: "출발 위치 확인이 끝나 세션을 끝냅니다",
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
  if (st.pose_check && st.pose_check.status !== "confirmed" && !st.recovery) {
    return "확인 전 위치 · 곧은 후진만(한 번 누름 1 m) · 전진·조향·포크 없음";
  }
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
  real_rearm_refused: "실차 재무장 조건이 아직 맞지 않습니다 · 정지·DISARM·이전 명령 해소를 확인하세요",
  "not_ready:": "실차 준비 조건이 맞지 않습니다", // the real truck's submit refusal, its reason after (5c-1b r1 P3)
  // the real truck's console requests (5c-1, real safety wiring design 5)
  already_answered: "이 확인 요청에는 이미 답했습니다",
  armed: "무장 중에는 바꿀 수 없습니다 · 정지·DISARM 뒤에 하세요",
  commands_outstanding: "아직 처리되지 않은 명령이 남아 있습니다",
  lease_changed: "조종 권한이 바뀌어 우회 변경을 적용하지 않았습니다",
  no_open_recovery: "열린 현장 복구가 없습니다",
  emergency_pending: "정지·DISARM 이 처리되는 중입니다 · 잠시 뒤 다시 보내세요",
  // 6a (real HIL design 5): a second unsettled request with the same id; a request refused by a failed vehicle tick
  duplicate_request: "같은 요청 번호가 아직 처리 중입니다 · 새 요청으로 다시 보내세요",
  tick_failed: "차 쪽 처리 중 오류로 이 요청을 받지 않았습니다 · 정지 상태를 확인하고 다시 보내세요",
  no_relocalization: "다시 확인할 위치가 없습니다(재위치가 열려 있지 않음)",
  no_start_pose: "시작 자세가 설정되지 않아 재위치를 받을 수 없습니다",
  slam_not_ready: "SLAM 이 아직 위치를 잡지 못해 재위치 확인을 받을 수 없습니다",
  seed_in_progress: "앞선 시작 자세 선언을 SLAM 이 처리하는 중입니다 · 끝난 뒤 다시 누르세요",
  // PR 7 7r-2: the HIL run's check of the board is lost (or the board restarted) -- nothing completes on it
  hil_check_lost: "HIL 보드 확인이 끊겼습니다 · 보드가 다시 확인될 때까지 재위치를 받을 수 없습니다",
  slam_stale: "차는 서 있지만 SLAM 응답이 낡았습니다 · 새 응답을 기다리세요",
  stale_relocalization: "그 재위치 세대는 지금 것이 아닙니다 · 화면을 다시 보세요",
  no_operator_request: "지금 열린 확인 요청이 없습니다",
  not_bypassable: "이 검사는 우회할 수 없습니다(멈추는 쪽 검사)",
  // U3b-2: the operating run's D5 confirmation (docs/plans/2026-11-26-real-operating-assembly.md 2.6)
  operator_start_needs_registration: "시작 확인에 화면의 팔레트와 등록 번호가 없습니다 · 팔레트를 고르고 다시 확인하세요",
  targets_changed: "팔레트 등록이 바뀌었습니다 · 화면을 다시 보고 확인하세요",
  pallet_not_registered: "운전자가 등록한 팔레트 위치가 없습니다",
  "d5_pocket:no_depth_source": "깊이 카메라가 없어 D5 포켓 우회를 켜야 제출할 수 있습니다",
  "operator_start:unconfirmed": "이 팔레트·등록 번호의 시작 확인이 없습니다 · 화면을 보고 다시 확인하세요",
  "not_ready:operator_start:unconfirmed": "시작 확인이 없습니다 · 화면을 보고 확인한 뒤 제출하세요",
  // U3c (operating plan 2.7, 2.9): the precondition row of an IMU that did not qualify
  // U3c (Codex round 1 P3-2): the scan source's refusals the precondition row tells (real_sources.py), reason after
  "feed:": "스캔 수신을 거부했습니다",
  "frame:": "스캔 프레임이 자격을 잃었습니다",
  // U3c (Codex round 2): the truck moved after the six lines were ticked
  still_changed: "확인한 뒤 차가 움직였습니다 · 서 있는 지금 여섯 항목을 다시 확인하세요",
  "imu:unqualified": "IMU 가 검증되지 않았습니다 · imu 우회를 켜야 제출·무장할 수 있습니다(우회 중에는 기울기 감시 꺼짐)",
  // U3b-1: the operating run's case A and run time
  run_time_short: "남은 실행 시간이 모자랍니다",
  "case_a:": "승인된 배치·경로(사례 A)가 아닙니다",
  not_disarmed: "DISARM 이 아직 확인되지 않았습니다",
  not_still: "차가 멈춰 있지 않습니다",
  stale_operator_request: "그 확인 요청은 이미 바뀌었습니다 · 화면을 다시 보세요",
  stale_recovery: "그 현장 복구 번호는 지금 것이 아닙니다",
  state_unknown: "차량 상태를 알 수 없습니다",
  token_required: "이 요청은 연결 토큰이 있어야 합니다 · 콘솔을 다시 연결하세요",
  unknown_check: "모르는 검사 이름입니다",
  unknown_pallet: "등록 목록에 없는 팔레트입니다",
  unknown_mission: "그 미션은 지금 미션이 아닙니다",
  unsupported: "이 차량이 지원하지 않는 요청입니다",
  stopping: "차가 멈추는 중입니다",
  scene_damaged: "장면이 손상되어 자동 작업을 받지 않습니다 · 장면 초기화가 필요합니다",
  recovery_required: "포크가 팔레트 안에 있거나 위치 고정 중이라 받을 수 없습니다 · 수동 복구를 먼저 하세요",
  cargo_held: "포크에 팔레트가 실려 있습니다",
  relocalize_required: "현장 복구 뒤 위치를 다시 확인해야 받을 수 있습니다",
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
  // B-P2c: the start relocalization (docs/plans/2026-10-29-start-relocalization.md 1.3)
  relocalizing: "출발 위치를 찾는 중입니다 · 위치가 정해진 뒤에 보내세요",
  not_relocalizing: "출발 위치를 찾는 중이 아닙니다",
  reloc_stale_pick: "후보 목록이 바뀌었습니다 · 새 후보에서 다시 고르세요",
  reloc_pick_refused: "그 후보는 고를 수 없습니다",
  reloc_not_awaiting: "지금은 후보를 고르거나 다시 시도할 수 없습니다",
  reloc_operator_timeout: "고르기·다시 시도 시간이 지났습니다",
  // F1: the operator's placed pose (docs/plans/2026-10-31-operator-placed-start.md 1.2)
  reloc_place_invalid: "놓은 자세를 읽을 수 없습니다",
  reloc_place_limit: "놓기를 세 번 다 썼습니다 · 다시 시도하거나 세션을 끝내세요",
  // the unconfirmed position check (docs/plans/2026-11-02-unconfirmed-runner.md 2.3)
  pose_unconfirmed: "출발 위치가 확인되기 전입니다 · 곧은 후진으로 확인한 뒤에 보내세요",
  pose_ended: "출발 위치 확인이 끝났습니다 · 세션을 다시 시작하세요",
  // the vehicle-side F1 block (docs/plans/2026-11-08-f1-vehicle-block.md 2.4)
  operator_placed_start: "운전자가 놓은 출발 위치입니다 · 자동 운행과 미션은 받지 않습니다(수동 운전만)",
  // the bench console (docs/plans/2026-11-27-bench-console.md 3.1, B1): bench_run / clear_faults refusals
  bench_refused: "벤치 실행 요청이 거부됐습니다 · 함께 온 사유를 보세요",
  board_not_ready: "보드가 아직 준비되지 않았습니다(연결 직후·초기화 중이거나 벤치 이미지가 아님)",
  busy_running: "이미 벤치 실행 중입니다 · 끝난 뒤 다시 보내세요",
  busy_clearing: "고장 해제 중입니다 · 끝난 뒤 다시 보내세요",
  stopped: "정지 확인을 기다리는 중입니다 · 보드가 멈춘 것이 확인되면 다시 보내세요",
  record_unavailable: "실행 기록을 만들 수 없습니다(디스크·경로 확인) · 기록 없이는 실행하지 않습니다",
  not_latched: "고장·비상정지 잠금 상태가 아닙니다 · 해제할 것이 없습니다",
  // the wall escape (standing-direction-rule design 4.2)
  escape_no_offer: "탈출 제안이 바뀌었습니다 · 지금 보이는 버튼을 다시 누르세요",
  escape_stale: "탈출 제안이 이전 세대입니다 · 다시 확인하세요",
  escape_active: "이미 탈출 중입니다",
  stale_arm: "무장 세대가 바뀌었습니다 · 다시 확인하세요",
  not_teleop: "수동 조종이 아닙니다",
  not_armed: "무장하지 않았습니다",
  latched: "정지 잠금 중입니다",
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

// The S6 bridge's wall escape (docs/design/2026-11-12-standing-direction-rule.md 4.1-4.3): a button only while the
// vehicle offers one (Isaac and the mock never do), its press approving exactly that offer; while an escape runs,
// its progress instead.
const DIR_KO = { 1: "앞으로", "-1": "뒤로" };
const SIDE_KO = { 1: "앞", "-1": "뒤" };
export function escapeState(st, hasLease) {
  const offer = st?.escape_offer;
  const run = st?.escape;
  const running = run
    ? `벽 앞 탈출 중 · ${DIR_KO[run.direction] ?? ""} ${Math.max(...(run.travel_m ?? [0])).toFixed(2)} / ${run.budget_m} m · ${Number(run.elapsed_s ?? 0).toFixed(1)} / ${run.time_s} s`
    : "";
  if (!offer || run) return { shown: false, enabled: false, label: "", hint: "", running };
  const label = `벽 앞 탈출 · ${DIR_KO[offer.direction] ?? ""} 0.20 m`;
  const hint = `${SIDE_KO[offer.blocked] ?? ""} 쪽이 막혀 정지했습니다. 막힌 쪽을 직접 보고 누르세요 · 직진 저속, 한 번만`;
  return { shown: true, enabled: Boolean(hasLease), label, hint: hasLease ? hint : `조종 권한이 필요합니다 · ${hint}`, running };
}

// The approval's fields: the offer shown and the arm generation it stands under (null: nothing to approve).
export function escapeRequest(st) {
  const offer = st?.escape_offer;
  if (!offer || !Number.isInteger(offer.offer_id) || !Number.isInteger(st?.arm_gen)) return null;
  return { arm_gen: st.arm_gen, offer_id: offer.offer_id };
}

// The bench console's screen (docs/plans/2026-11-27-bench-console.md 4, 4.1): its words, kept here with the others.
export const BENCH_TEXT = {
  menu: "벤치",
  title: "모터 벤치",
  hint: "벤치 이미지(bench7) 전용 · 한 축 구동·위치 유지·바퀴 수 정지 · 판정은 보드의 기록으로",
  axis: { L: "L 왼쪽 바퀴", R: "R 오른쪽 바퀴", M: "M 조향", H: "H 승강" },
  source: { kth: "KTH(절대각)", none: "측정 없음", mt: "MT(모터축)" },
  mode: { duty: "듀티 구동", hold: "위치 유지" },
  waitB2: "A1b 뒤에 열림",
  hold: {
    title: "위치 유지",
    target: "목표",
    targets: { "": "목표 선택(필수)", zero: "영점 고정(센서 원시 0)", here: "지금 자리 고정",
      abs: "정한 각도에 고정(원시 0–4095)", rel: "지금 자리에서 옮겨 고정(카운트, 4096 = 한 바퀴)" },
    value: "값",
    cap: "출력 상한",
    kp: "세기(비례)",
    ki: "누적 보정(적분)",
    kd: "감쇠",
    kdNote: "감쇠는 실물 미시험 — 0 과 0.02 를 비교해 보세요",
    deadband: "허용 오차(카운트)",
    minDuty: "최소 출력",
    defaults: "비워 두면 기본값: 출력 상한 0.15 · 세기 0.5 · 누적 보정 0 · 감쇠 0 · 허용 오차 12 · 최소 출력 0.12",
  },
  stopTurns: "N 바퀴에서 정지(센서축 회전 수)",
  measureOnly: "회전 수로 멈추지 않고 정한 시간 동안 이동량만 재기",
  countSign: { "": "센서 방향(필수)", "1": "+1: 양의 출력에 카운트가 늘어남", "-1": "−1: 양의 출력에 카운트가 줄어듦" },
  countSignHint: "축이나 원천을 바꾸면 방향·목표·바퀴 수 칸은 비워집니다",
  sameSetup: "실보드 묶음(5.1)과 같은 구성(모터 단독)",
  rangeMark: { unchecked: "실물 확인 범위: 아직 없음(5.1 결과 전)", inside: "실물 확인 범위 안",
    outside: "실물 확인 범위 밖", not_applicable: "실물 확인 범위: 해당 없음(위치 유지는 따로 확인)" },
  waitA1c: "아직 지원하지 않음(다음 단계)",
  waitA2: "아직 지원하지 않음(다음 단계)",
  bypass: { steer_range: "조향 범위", lift_range: "승강 범위", kth_invalid: "KTH 무효" },
  bypassSession: "이 실행만 우회",
  advice: { rangeOff: "권장: 기구에서 모터를 분리하세요(부분 구성 절차 3 절) · 막지는 않습니다",
    highDutyDistance: "출력 0.4 를 넘는 바퀴 수 정지는 실물에서 확인되지 않았습니다 · 막지는 않습니다" },
  clearAdvice: "권장: 고장 해제 전에 VM 을 끄세요(주말 인계 6.5 절) · 막지는 않습니다",
  confirm: { power_switch_in_hand: "주 전원 스위치를 손에 쥠", wheels_lifted: "바퀴를 띄우고 고정함",
    vcc_watched: "VCC 를 지켜봄(듀티 0.3 초과)" },
  largeConfirm: "큰 실행 확인 문구",
  physicalSetup: { "": "물리 구성(선택)", coupled: "기구 결합", decoupled: "모터 분리", lifted: "바퀴 들어 올림" },
  start: "시작",
  stop: "STOP",
  disarm: "DISARM",
  clear: "고장 해제",
  cause: "원인(필수)",
  chipsMcu: "MCU 확인 우회",
  chipsHost: "호스트 우회",
  noBypass: "우회 없음",
  notReady: "보드 준비 안 됨",
  ready: "준비됨",
  phase: { recover: "직전 결과 확인", set: "설정 전송", arm: "무장", prep: "축 준비", first: "첫 명령 확인",
    drive: "구동", stop: "정지 중", rest: "멈춘 자리 관측", closed: "닫힘" },
  result: "결과",
  verdictRun: "실행·정리",
  verdictHold: "위치 유지 판정",
  verdictDistance: "바퀴 수 정지 판정",
  verdictMeasure: "이동량",
  verdictNone: "해당 없음",
  holdResult: { met: "목표에 머묾", unmet: "목표에 머물지 못함", failed: "실패", undecidable: "판정 불가",
    held: "판정 보류" },
  distanceResult: { reached: "도달", held: "잠정 도달 뒤 확인 못 함", waiting: "확인 대기",
    failed_loss: "확인 중 센서 끊김", missed: "도달 전에 끝남", failed_continuity: "카운트 연속성 깨짐" },
  measureWithheld: "보류(믿을 기준 없음)",
  resultMissing: "판정 불가(보드 결과를 받지 못함)",
  startWords: { accepted: "받아들임", confirmed_by_result: "보드 결과로 확인", unconfirmed: "확인 못 함",
    no_drive_confirmed: "구동 안 함" },
  problems: { count_sign: "센서 방향(+1 / −1)을 고르세요", hold_target: "위치 유지 목표를 고르세요",
    hold_value: "목표 값(정수 카운트)을 넣으세요", hold_cap: "출력 상한은 숫자로", hold_deadband: "허용 오차는 정수 카운트로",
    stop_turns: "바퀴 수는 0 보다 큰 숫자로", stop_turns_too_small: "바퀴 수가 한 카운트(1/4096 바퀴)보다 작습니다",
    duty: "출력(듀티)을 숫자로 넣으세요", duty_too_small: "출력이 너무 작습니다(0.0001 이상)", seconds: "시간을 넣으세요",
    hold_kp: "세기(비례)는 숫자로", hold_ki: "누적 보정(적분)은 숫자로", hold_kd: "감쇠는 숫자로",
    hold_min_duty: "최소 출력은 숫자로", bench_duty_max: "보드 출력 한도는 숫자로",
    bench_time_max: "보드 연속 출력 한도는 숫자로",
    window: "창 아래·위를 넣으세요", busy: "실행 중입니다" },
  runBypass: "실행 당시 우회",
  previousRun: "직전 결과(회수)",
  copyError: "결과 사본 저장 실패(판정은 그대로)",
  rest: "멈춘 자리",
  restSettled: "100 Hz 관측에서 0.5 초 동안 움직이지 않음",
  restReopened: "관측 중 보드 연결을 다시 열어 확인 못 함",
  restNote: { reopened: "관측 중 보드 연결을 다시 엶", link_reopened: "관측 중 보드 연결을 다시 엶",
    link_lost: "관측 중 보드 연결 끊김", rebooted: "관측 중 보드 재부팅", disarm: "관측 중 무장 해제 요청(기록만)",
    tool_exit: "관측 중 도구 종료", stop: "관측 중 정지 요청(기록만)",
    lease_lost: "관측 중 조종권 잃음(기록만)", lease_expired: "관측 중 조종권 만료(기록만)",
    disconnected: "관측 중 화면 연결 끊김(기록만)", control_boundary: "관측 중 조종 경계(기록만)" },
  restWhy: { no_sample: "표본 없음", continuity: "카운트 연속성 깨짐", evidence_age: "센서 증거가 오래됨",
    too_short: "관측이 짧음" },
  boardCause: "보드가 끝낸 이유",
  recordComplete: "기록 완료",
  firstCommand: "첫 명령",
  labels: "참고",
  notes: { brake_end_unseen: "제동 끝을 1 초 동안 보지 못함", result_pair_absent: "결과 표본이 0.15 초 늦음" },
  ok: "정상",
  notOk: "실패",
  noResult: "아직 결과 없음",
  graph: "최근 10 s",
  replyRefused: "거부",
  replyOk: "받음",
};

// The bench back end's detail words (B1: bench_run refusals the server explains in ``detail``).
export function benchDetailText(detail) {
  const d = String(detail ?? "");
  if (d.startsWith("count_sign")) return "센서 방향(+1 / −1)을 고르세요";
  if (d.startsWith("not_yet_a1c")) return "MT 원천은 A1c 뒤에 열립니다";
  if (d.startsWith("not_yet_a2")) return "KTH 무효 우회는 A2 뒤에 열립니다";
  return d;
}
