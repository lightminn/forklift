// Persistent console plan D13 (docs/plans/2026-10-18-persistent-live-console.md, 3rd edition): what the screen makes
// of the results book, the scene, the recovery and the session requests. Display rules only; the vehicle decides.

// The results explanation table: reason head -> what it means. It is not the exception classification
// (failures.classify): a preparation, a recovery or a delivery verdict ends one mission and the session goes on.
// tests/unit/console/test_results_reasons.py checks that every reason a result can carry has a head here.
export const REASONS = {
  // the truth checks (simulation only): the scene is damaged, latched until it is reset
  "Excessive body tilt": "차체가 지나치게 기울었습니다(장면 손상)",
  "Actual truck/load footprint overlap": "차나 짐이 장애물과 겹쳤습니다(장면 손상)",
  "Measured pallet footprint overlap": "팔레트가 다른 물체와 겹쳤습니다(장면 손상)",
  "Measured pallet solid overlap": "팔레트가 다른 물체와 겹쳤습니다(장면 손상)",
  "Pallet dropped during transport": "운반 중 팔레트가 떨어졌습니다(장면 손상)",
  "Pallet slipped off forks": "팔레트가 포크에서 미끄러졌습니다(장면 손상)",
  "Pallet displaced before pickup": "집기 전에 팔레트가 밀렸습니다(장면 손상)",
  "Forbidden fork/pallet contact": "포크가 팔레트의 닿으면 안 되는 곳에 닿았습니다(장면 손상)",
  // the session's own (contracts, localisation, sensors, the body, the setup)
  "near_field_contract": "근접 추적 계약을 어겼습니다",
  "near_field_without_capture": "근접 추적이 촬영 없이 시작됐습니다",
  "near_field_without_hold": "근접 추적이 위치 고정 없이 시작됐습니다",
  "near_field_frame_released": "근접 추적 중 위치 고정이 풀렸습니다",
  "pocket_check_without_capture": "포켓 확인이 촬영 없이 시작됐습니다",
  "perception_capture_failed:invalid_capture_pose": "촬영 자세가 잘못됐습니다(센서·자세 계약)",
  "perception_capture_failed:sensor_metadata_unavailable": "센서 정보를 받지 못했습니다",
  "plan_worker_dead": "계획 작업자를 다시 띄우지 못했습니다",
  "localization_stale": "위치 추정이 오래됐습니다",
  "slam_link_failed": "SLAM 연결이 끊겼습니다",
  "slam_prewarm_timeout": "SLAM 준비 시간이 지났습니다",
  "slam_map_mismatch": "지도 묶음이 세션의 지도와 다릅니다",
  "slam_map_not_loaded": "조사 지도를 불러오지 못했습니다",
  "slam_not_localized": "지도에서 위치를 잡지 못했습니다",
  "slam_gyro_fault": "자이로 값을 읽지 못했습니다",
  "sensor_silence_stopped": "센서가 멈춰 정지했습니다",
  "Camera did not produce RGB": "카메라 영상이 오지 않았습니다",
  "Perception camera did not produce depth": "깊이 영상이 오지 않았습니다",
  "Nonfinite body state": "차체 상태 값이 비정상입니다",
  "Mission exceeded simulation time budget": "미션이 시뮬레이션 시간 한도를 넘었습니다",
  "mission_cancelled": "미션이 취소됐습니다",
  "Cannot open base scene": "기본 장면을 열지 못했습니다",
  "Invalid pallet spawn after settling": "팔레트 배치가 잘못됐습니다",
  "Truck spawn displaced": "차 시작 자리가 밀렸습니다",
  "This adapter requires 120Hz physics": "물리 주기가 120 Hz 가 아닙니다",
  "Settings must be positive finite values": "설정 값이 잘못됐습니다",
  "Camera render product prim is missing": "카메라 렌더 대상이 없습니다",
  "Perception camera intrinsics": "인식 카메라 내부 값이 맞지 않습니다",
  "base scene built from another forklift URDF": "기본 장면이 다른 차체 모형으로 만들어졌습니다",
  "pocket camera intrinsics differ": "포켓 카메라 내부 값이 다릅니다",
  "live video camera intrinsics differ": "영상 카메라 내부 값이 다릅니다",
  "Perception display clipping range": "인식 카메라 표시 범위가 잘못됐습니다",
  "Perception camera local pose differs": "인식 카메라 장착 자세가 다릅니다",
  "--odometry-wheel-radius-m": "주행기록 바퀴 반지름 설정이 잘못됐습니다",
  "recognition label target": "장면에 인식 라벨 대상이 없습니다",
  // the delivery verdict (simulation truth): the mission fails, the session goes on
  "Pallet missed the green destination center": "팔레트가 목적지 중심을 벗어났습니다",
  "Delivered pallet is not grounded": "내려놓은 팔레트가 바닥에 닿지 않았습니다",
  "Fork still inside delivered pallet": "내려놓은 뒤에도 포크가 팔레트 안에 남았습니다",
  // the mission's own work
  "Tracking timeout": "경로 추종 시간이 지났습니다",
  "Tracking failed": "경로 추종에 실패했습니다",
  "obstacle_": "장애물 때문에 멈췄습니다",
  "perception_capture_failed": "팔레트 인식 촬영에 실패했습니다",
  "observation_discarded": "관찰 결과를 버렸습니다",
  "near_capture_": "근접 촬영에 실패했습니다",
  "pocket_check_refused": "포켓 확인에서 삽입을 거절했습니다",
  "near_field_": "근접 추적에 실패했습니다",
  "observe:": "관찰 경로를 계획하지 못했습니다",
  "observe_replan_failed": "관찰 경로를 다시 계획하지 못했습니다",
  "docking_": "도킹에 실패했습니다",
  "slam_release_": "SLAM 고정을 풀지 못했습니다",
  "async_plan_failed": "비동기 계획에 실패했습니다",
  "async_plan_discarded": "비동기 계획을 버렸습니다",
  "async_near_no_fresh_depth": "근접 계획에 쓸 새 깊이 영상이 없습니다",
  "planning_wait_moving": "계획을 기다리는 동안 차가 움직였습니다",
  "Pallet was not lifted": "팔레트를 들지 못했습니다",
  "Pallet did not settle onto destination floor": "팔레트가 목적지 바닥에 놓이지 않았습니다",
  "Fork did not lower": "포크가 내려가지 않았습니다",
  "withdraw_not_certified": "포크를 빼도 되는지 확인하지 못했습니다",
  "withdraw_tracking_exceeded": "포크를 빼는 동안 추적 한계를 넘었습니다",
  "pocket_stop": "포켓 안에서 멈췄습니다",
  "transport_recovery_after_docking": "도킹 뒤 운반 복구에 실패했습니다",
  "stall_replan_failed": "막힌 뒤 다시 계획하지 못했습니다",
  "Mission planning failed": "미션을 계획하지 못했습니다",
  "approach:": "접근 경로를 계획하지 못했습니다",
  "insert:": "삽입 경로를 계획하지 못했습니다",
  "extract:": "빼기 경로를 계획하지 못했습니다",
  "transport:": "운반 경로를 계획하지 못했습니다",
  "withdraw:": "철수 경로를 계획하지 못했습니다",
  "return_home:": "복귀 경로를 계획하지 못했습니다",
  "cusp_position_stall": "방향을 바꾸는 자리에서 멈췄습니다",
  // the preparation (the first leg, planned when the mission is submitted)
  "prepare:all_candidates_failed": "첫 구간을 계획하지 못했습니다",
  "prepare:timeout": "첫 구간 계획 시간이 지났습니다",
  "prepare:error": "첫 구간 계획 중 오류가 났습니다",
  "prepare:reference": "목적지 기준 스캔을 만들지 못했습니다",
  "prepare:targets": "계획하는 동안 대상 목록이 바뀌었습니다",
  "prepare:correction": "계획하는 동안 위치 보정이 바뀌었습니다",
  "prepare:stopped": "계획하는 동안 STOP 으로 멈췄습니다",
  "prepare:paused": "계획하는 동안 일시정지됐습니다",
  "prepare:teleop": "계획하는 동안 수동 조종으로 바뀌었습니다",
  // a loaded submit (plan D5 6c): the pallet already on the forks, delivered from where the truck stands
  "prepare:loaded:cargo_changed": "준비하는 동안 실린 팔레트의 상태가 바뀌었습니다",
  "prepare:loaded:transport:no_final_straight": "배달 직선이 남지 않았습니다 — 목적지에서 곧게 더 물러난 뒤 다시 내십시오",
  "prepare:loaded:no_final_straight": "배달 직선이 남지 않았습니다 — 목적지에서 곧게 더 물러난 뒤 다시 내십시오",
  "prepare:loaded:start_wait": "출발 준비(승강·첫 구간·도킹)가 60 초 안에 끝나지 않았습니다",
  "prepare:loaded:": "실린 팔레트의 운반을 계획하지 못했습니다",
  "prepare:": "첫 구간을 준비하지 못했습니다",
  // the recovery of a mission that ended with the forks in a pocket
  "recovery:clear_out:budget": "물러나기 거리(1.0 m)를 다 썼습니다 — 수동 복구가 필요합니다",
  "recovery:clear_out:no_start_within_budget": "1.0 m 안에 다음 출발 자리가 없습니다 — 수동 복구가 필요합니다",
  "recovery:clear_out:hold_released": "물러나는 중 위치 고정이 풀렸습니다 — 수동 복구가 필요합니다",
  "recovery:trace_invalid": "포켓 관계의 증거가 끊겼습니다 — 수동 복구가 필요합니다",
  "recovery:no_pose": "자세를 알 수 없습니다 — 수동 복구가 필요합니다",
  "recovery:no_tracker_config": "추종기 설정이 없습니다 — 수동 복구가 필요합니다",
  "recovery:blocked": "복구 동작이 막혔습니다 — 수동 복구가 필요합니다",
  "recovery:": "스스로 복구하지 못했습니다 — 수동 복구가 필요합니다",
  // the ends that are not failures of the work
  "operator": "운용자가 취소했습니다",
  "session_end": "세션이 끝나 미션도 끝났습니다",
};

const CANDIDATE = { invalid_start: "출발 자세가 막힘", invalid_goal: "도착 자세가 막힘", no_path: "경로 없음",
                    timeout: "시간 초과", expansion_limit: "탐색 한도" };

const HEADS = Object.keys(REASONS).sort((a, b) => b.length - a.length);

export function reasonText(code) {
  if (!code) return { text: "", code: code ?? null };
  const all = "prepare:all_candidates_failed:";
  if (code.startsWith(all)) {
    const counts = new Map();
    const statuses = code.slice(all.length).split(";").filter(Boolean);
    for (const s of statuses) counts.set(s, (counts.get(s) ?? 0) + 1);
    const parts = [...counts].map(([s, n]) => `${CANDIDATE[s] ?? s} ${n}`).join(" · ");
    return { text: `${REASONS["prepare:all_candidates_failed"]} — ${parts} (${statuses.length} 후보)`, code };
  }
  const head = HEADS.find((h) => code.startsWith(h));
  return { text: head ? REASONS[head] : code, code };
}

// The results book of this session (contract 5.2): the seq counts within a session_epoch only. Every message carries
// the vehicle's whole kept list (its newest 100) and how many are older; the page shows that list as it is -- a long
// session does not grow it (Codex D13 impl round 2) -- and never goes back to an older message arriving late.
export class ResultsBook {
  constructor() { this.epoch = null; this.list = []; this.older = 0; this.recordErrorsOlder = 0; this.seq = 0; this.version = 0; }

  welcome(epoch) {
    if (epoch !== this.epoch) {
      this.epoch = epoch; this.list = []; this.older = 0; this.recordErrorsOlder = 0; this.seq = 0; this.version += 1;
    }
  }

  update(msg) {
    const seq = msg.missions_seq ?? 0;
    if (seq < this.seq) return;
    this.seq = seq;
    this.list = [...(msg.results ?? [])].sort((a, b) => b.seq - a.seq);
    this.older = msg.older ?? 0;
    this.recordErrorsOlder = msg.record_errors_older ?? 0; // U29: record errors of results out of the list
    this.version += 1; // the page redraws the list when this changes
  }

  rows() { return this.list; }
}

const PERMIT = { "recovery:steering": "조향이 곧게 돌아오기를 기다리는 중", ok: "허용", occupied: "앞이나 뒤가 막힘" };

// The manual recovery's guidance (D13 item 4): from the state's recovery and its policy and permit only.
export function recoveryGuide(st) {
  const rec = st?.recovery;
  if (!rec) return null;
  const dirs = st.lift_dirs ?? [];
  let next;
  if (st.drive_mode === "reverse_straight") next = "곧게 후진하세요(0.10 m/s 이하, 조향 0)";
  else if (dirs.includes(-1)) next = "포크를 내리세요(내리기만 허용)";
  else if (st.teleop_forbidden_reason) next = `막힘: ${st.teleop_forbidden_reason}`;
  else next = "다음 동작을 기다리는 중";
  const notes = [];
  notes.push(`단계 ${rec.stage === "manual_clear" ? "빠져나옴 확인 중" : "포크가 포켓 안"}`);
  notes.push(`접지 ${rec.grounded ? "됨" : "안 됨"} · 증거 ${rec.evidence ? "있음" : "없음"} · 정렬 ${rec.aligned ? "맞음" : "어긋남"}`);
  if (Number.isFinite(st.lift_m)) notes.push(`승강 ${Math.round(st.lift_m * 1000)} mm`);
  if (st.stand_waiver === "lower_only") notes.push("서 있음 확인 없이 들어갈 수 있습니다(내리기만)");
  const reason = st.permit?.reason;
  const permit = reason ? `현재 구동 허가: ${PERMIT[reason] ?? reason}` : null;
  return { stage: rec.stage, next, notes, permit };
}

// the arbiter's ACTIVE (src/forklift_core/mission/commands.py)
export const ACTIVE = ["accepted", "preparing", "running", "paused", "resuming", "held", "cancelling", "recovering"];

// The session buttons (D13 item 5), enabled in the arbiter's order; the stand waiver does not apply to them.
export function sessionButtons(st, { hasLease, caps }) {
  let why = "";
  if (!hasLease) why = "조종 권한이 필요합니다";
  else if (st?.feedback_ok === false) why = "차량 피드백이 없습니다";
  else if (st?.stopped_confirmed === false) why = "정지 확인을 기다리는 중";
  else if (st?.mission && ACTIVE.includes(st.mission.status)) why = "진행 중인 미션이 있습니다";
  const enabled = why === "";
  return { end: { shown: true, enabled, why }, reset: { shown: caps?.sim_reset === true, enabled, why } };
}

// The two-step confirm (D13 item 5): a second click from 0.5 s to 5 s after the first sends; it belongs to the
// request kind, session epoch, lease and control generation it was armed under.
export class ConfirmGate {
  constructor(minMs = 500, maxMs = 5000) { this.minMs = minMs; this.maxMs = maxMs; this.armed = null; }

  click(key, nowMs) {
    const a = this.armed;
    if (a && sameKey(a.key, key) && nowMs - a.at <= this.maxMs) {
      if (nowMs - a.at < this.minMs) return "too_soon";
      this.armed = null;
      return "send";
    }
    this.armed = { key: { ...key }, at: nowMs };
    return "armed";
  }

  pending(key, nowMs) {
    const a = this.armed;
    return Boolean(a && sameKey(a.key, key) && nowMs - a.at <= this.maxMs);
  }

  cancel() { this.armed = null; }
}

function sameKey(a, b) {
  return a.kind === b.kind && a.epoch === b.epoch && a.leaseId === b.leaseId && a.controlGen === b.controlGen;
}

// The back-off advice (contract 5.2): a guide from the pose at the failure, not a permit.
export function adviceText(advice) {
  if (!advice) return null;
  if (advice.skipped === "deadline") return "물러날 거리를 계산할 시간이 모자랐습니다";
  if (advice.backoff_m === null || advice.backoff_m === undefined) {
    if (Number.isFinite(advice.blocked_at_m)) return `곧게 물러나는 길이 ${advice.blocked_at_m.toFixed(2)} m 에서 막힙니다`;
    return `곧게 ${(advice.within_m ?? 1.0).toFixed(1)} m 안에는 다음 출발 자리가 없습니다`;
  }
  return `곧게 ${advice.backoff_m.toFixed(2)}–${advice.backoff_max_m.toFixed(2)} m 물러난 뒤 다시 지시하세요(실패 때 자세 기준, 허가 아님)`;
}

const DESTINATIONS = { "D-0": "원래 자리(D-0)", "D-1": "목적지(D-1)" };

export function destinationLabel(id) {
  return DESTINATIONS[id] ?? id;
}

// The supervisor's standby (contract 6, /api/v1/session): what the session is doing while no runner serves the port.
// A start check that failed before the runner's server opened is a "세션 시작 실패" (SLAM localization plan L4).
const STANDBY_END = { map_mismatch: "지도가 맞지 않음", failing: "같은 원인으로 거듭 실패", epoch_limit: "실행 횟수 한도",
                      ended: "끝냄", terminated: "작업이 끝남" };

export function standbyText(s) {
  if (!s || s.standby !== true) return null;
  const last = s.last_exit;
  const parts = [];
  if (last && last.exit_code !== 0 && last.exit_code !== 3) {
    const why = reasonText(last.reason ?? `exit ${last.exit_code}`);
    const shown = why.code && why.text !== why.code ? `${why.text} (${why.code})` : why.text;
    parts.push(`${last.stage === "run" ? "세션이 끊겼습니다" : "세션 시작 실패"}: ${shown}`);
  }
  if (s.phase === "starting") parts.push(`세션을 시작하는 중 (${s.epoch}번째)`);
  else if (s.phase === "cleanup") parts.push("정리 중");
  else if (s.phase === "backoff" && s.retry) parts.push(`${s.next_epoch}번째 세션을 곧 시작합니다`);
  else if (s.phase === "stopped") parts.push(`세션이 멈췄습니다(${STANDBY_END[s.end] ?? s.end})`);
  return parts.length ? parts.join(" · ") : null;
}

// The state's optional slam field (contract 6): present only for a session localizing on a survey map; absent is
// online mapping or not known. map_loaded is the session's start check, not the localization's health now.
export function slamLabel(st) {
  if (!st) return { text: "—", title: "" };
  const slam = st.slam;
  if (!slam) return { text: "온라인 작성 또는 모름", title: "상태에 slam 필드가 없음" };
  const id = String(slam.map_id ?? "");
  return { text: `지도 ${slam.mode === "localization" ? "위치추정" : slam.mode} · ${id.slice(0, 8)}`,
           title: `지도 ${id}${slam.map_loaded ? " · 세션 시작 검사 통과" : ""}` };
}
