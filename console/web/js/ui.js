// Panel rendering from the latest vehicle state. Buttons are only enabled when the
// request could be accepted; the vehicle still decides and may reject.

const MODE_KO = { AUTO: "자동 주행", TELEOP: "수동 조종", PAUSED: "대기", STOPPED: "정지됨" };
const STOP_KO = { none: "정지 종류 없음", controlled_decel: "제어 감속", safety_preempt: "안전 선점" };
const STATUS = {
  accepted: ["acc", "수락"],
  running: ["acc", "실행 중"],
  paused: ["warn", "일시정지"],
  held: ["warn", "보류"],
  resuming: ["acc", "재개 중"],
  done: ["ok", "완료"],
  failed: ["crit", "실패"],
  cancelled: ["", "취소됨"],
};
const PHASES = [
  ["observe", "관찰"],
  ["approach", "접근"],
  ["insert", "삽입"],
  ["lift", "들기"],
  ["extract", "빼기"],
  ["transport", "운반"],
  ["lower", "내리기"],
  ["withdraw", "후퇴"],
  ["return", "복귀"],
];
const STEP_KO = {
  stop_confirmed: "정지 확인",
  state_verified: "실제 상태 확인",
  frame_reexpressed: "보정 반영",
  path_rechecked: "경로 재검사",
};
const FORBID_KO = {
  state_forbids_teleop: "근접·삽입·승강 중이거나 포크가 포켓 안에 있어 수동 조종을 받지 않습니다",
  stopped_locked: "정지 상태입니다. 재무장해야 합니다",
};

const $ = (id) => document.getElementById(id);
const fmt = (v, d = 2) => (v === null || v === undefined || !Number.isFinite(v) ? "측정 안 됨" : v.toFixed(d));

function chip(el, cls, text) {
  el.className = `chip ${cls}`.trim();
  el.innerHTML = cls ? `<i></i>${text}` : text;
}

export function renderPallets(targets, onPick) {
  const box = $("pallet-list");
  const checked = document.querySelector('input[name="pal"]:checked')?.value;
  box.innerHTML = "";
  for (const p of targets.pallets) {
    const lab = document.createElement("label");
    lab.className = "pal";
    const avail = p.state === "available";
    lab.innerHTML = `<input type="radio" name="pal" value="${p.pallet_id}"><span><b>${p.pallet_id} · ${p.kind}</b><small>${Math.round(p.depth_m * 1000)} × ${Math.round(p.width_m * 1000)} mm · ${p.pose ? `(${p.pose.x_m.toFixed(1)}, ${p.pose.y_m.toFixed(1)}) m` : "포크 위"}</small></span><span class="chip ${avail ? "" : "ok"}">${{ available: "대기", carried: "적재 중", delivered: "하역 완료" }[p.state] ?? p.state}</span>`;
    const input = lab.querySelector("input");
    input.disabled = !avail;
    input.checked = avail && p.pallet_id === checked;
    input.addEventListener("change", onPick);
    box.appendChild(lab);
  }
  if (!document.querySelector('input[name="pal"]:checked')) {
    const first = box.querySelector("input:not(:disabled)");
    if (first) first.checked = true;
  }
  const dest = $("dest");
  const keep = dest.value;
  dest.innerHTML = targets.destinations
    .map((d) => `<option value="${d.destination_id}">${d.destination_id} · (${d.pose.x_m.toFixed(1)}, ${d.pose.y_m.toFixed(1)}) m</option>`)
    .join("");
  if (keep) dest.value = keep;
  $("targets-rev").textContent = `rev ${targets.targets_revision}`;
}

export function log(text, level = "") {
  const li = document.createElement("li");
  if (level) li.className = level;
  li.innerHTML = `<time>${new Date().toTimeString().slice(0, 8)}</time><span></span>`;
  li.querySelector("span").textContent = text;
  const ol = $("log");
  ol.prepend(li);
  while (ol.children.length > 80) ol.lastChild.remove();
}

export function render(m) {
  const st = m.session.state;
  const gens = m.session.generations;
  const lease = m.session.hasLease;
  $("gens").textContent = `epoch ${gens.session_epoch ?? "—"} · control ${gens.control_gen ?? "—"} · arm ${gens.arm_gen ?? "—"}`;
  const w = m.session.welcome;
  $("vehicle-id").textContent = w?.vehicle_id ?? "—";
  $("backend-label").textContent = w ? { mock: "가짜 차량", isaac: "Isaac Sim", real: "실차" }[w.backend] ?? w.backend : "연결 중";
  $("l-ep").textContent = `ws://${location.host}/api/v1/control`;
  chip($("link-chip"), m.link.up ? "ok" : "crit", m.link.up ? "연결됨" : "끊김");
  $("l-rtt").textContent = m.link.up && m.link.tickGapMs !== null ? `tick 간격 ${Math.round(m.link.tickGapMs)} ms` : "—";
  $("lease").textContent = lease ? "조종 권한: 이 기기" : "조종 권한 받기";
  $("lease").setAttribute("aria-pressed", String(lease));
  if (!st) return;

  const mode = st.mode;
  $("mode").dataset.m = mode;
  $("mode-code").textContent = mode;
  $("mode-ko").textContent = MODE_KO[mode] ?? mode;
  chip($("stop-kind"), st.stop_kind === "safety_preempt" ? "crit" : st.stop_kind === "controlled_decel" ? "warn" : "", STOP_KO[st.stop_kind] ?? st.stop_kind);
  $("rearm").hidden = mode !== "STOPPED";
  $("rearm").disabled = !lease || !st.stopped_confirmed;
  $("l-peers").textContent = `${st.lease?.holder ? (lease ? "이 기기(조종)" : "다른 기기(조종)") : "조종 기기 없음"} · 관찰 ${st.lease?.observers ?? 0}대${st.lease?.reserved_until_s ? ` · 재연결 대기 ${st.lease.reserved_until_s.toFixed(1)} s` : ""}`;

  // mission
  const mi = st.mission;
  const active = mi && ["running", "paused", "held", "resuming"].includes(mi.status);
  $("mission-gate").textContent = lease ? "" : "조종 권한이 있어야 지시할 수 있습니다";
  $("m-start").disabled = !lease || active || mode === "STOPPED" || mode === "TELEOP";
  $("m-pause").disabled = !lease || !mi || mi.status !== "running";
  $("m-resume").disabled = !lease || !mi || !["paused", "held"].includes(mi.status) || mode === "STOPPED" || mode === "TELEOP";
  $("m-cancel").disabled = !lease || !active;
  if (mi) {
    $("req-id").textContent = `${mi.mission_id} · ${mi.pallet_id} → ${mi.destination_id}`;
    const [cls, ko] = STATUS[mi.status] ?? ["", mi.status];
    chip($("req-status"), cls, mi.reason ? `${ko} · ${mi.reason}` : ko);
  } else {
    $("req-id").textContent = "요청 없음";
    chip($("req-status"), "", "—");
  }
  const key = mi ? `${mi.mission_id}:${mi.phase}:${mi.status}` : "none";
  if (key !== m.cache.phaseKey) {
    m.cache.phaseKey = key;
    const idx = mi ? PHASES.findIndex(([en]) => en === mi.phase) : -1;
    $("phases").innerHTML = PHASES.map(([en, ko], i) => {
      let cls = "";
      if (mi && (i < idx || mi.status === "done")) cls = "done";
      else if (mi && i === idx) cls = `cur${["paused", "held"].includes(mi.status) ? " held" : ""}`;
      return `<li class="${cls}"><i class="tick"></i><span>${ko}</span><span class="en">${en}</span></li>`;
    }).join("");
  }
  const steps = mi?.resume_steps;
  $("resume-steps").hidden = !steps;
  if (steps) {
    $("resume-steps").innerHTML = steps
      .map((s) => `<li class="${s.result}"><span class="mark">${s.result === "ok" ? "✓" : s.result === "failed" ? "✕" : "…"}</span><span>${STEP_KO[s.name] ?? s.name}${s.reason ? `<small>${s.reason}</small>` : ""}</span></li>`)
      .join("");
  }
  const forbidden = st.teleop_forbidden_reason;
  $("forbid").hidden = !forbidden || forbidden === "stopped_locked";
  $("forbid-reason").textContent = FORBID_KO[forbidden] ?? forbidden ?? "";
  $("recover").disabled = !lease || mode === "STOPPED";

  // state
  $("r-v").textContent = fmt(st.rear_axle_speed_mps);
  $("r-st").textContent = st.steering_rad ? ((st.steering_rad[0] * 180) / Math.PI).toFixed(1) : "—";
  $("r-lift").textContent = Number.isFinite(st.lift_m) ? Math.round(st.lift_m * 1000) : "—";
  $("r-pose").textContent = `x ${st.pose.x_m.toFixed(2)} · y ${st.pose.y_m.toFixed(2)} · ψ ${((st.pose.yaw_rad * 180) / Math.PI).toFixed(0)}°`;
  $("r-load").textContent = st.load?.state === "loaded" ? `적재 · ${st.load.pallet_id}` : st.load?.state === "empty" ? "없음" : "측정 안 됨";
  $("r-permit").textContent = `${st.permit.reason} · 상한 ${st.permit.limit_mps.toFixed(2)} m/s`;
  $("r-cmd").textContent = `${st.command.speed_mps.toFixed(2)} → ${st.applied.speed_mps.toFixed(2)} m/s`;
  if (m.chassis) $("r-radius").textContent = m.chassis.model_turning_radius_m.toFixed(2);
  $("r-phase-t").textContent = mi ? `${mi.phase_elapsed_s.toFixed(1)} s (일시정지 동안 멈춤)` : "—";
  const fr = st.freshness || {};
  const parts = [["격자", fr.grid_free_age_s], ["SLAM 보정", fr.slam_correction_age_s], ["근접 관측", fr.near_obs_age_s]];
  $("r-fresh").textContent = parts.map(([k, v]) => `${k} ${v === null || v === undefined ? "측정 안 됨" : `${v.toFixed(1)} s`}`).join(" · ");
  $("r-still").textContent = st.stopped_confirmed ? "정지 확인됨" : "확인 안 됨";
  chip($("loc-chip"), st.pose.source === "mock_truth" ? "" : "ok", st.pose.source === "mock_truth" ? "가짜 차량 정답 위치" : st.pose.source);

  // teleop
  const inTele = mode === "TELEOP";
  $("t-enter").disabled = !lease || inTele || mode === "STOPPED" || Boolean(forbidden);
  $("t-exit").disabled = !lease || !inTele;
  const gate = m.teleop.gate;
  $("t-arm").disabled = !lease || !inTele || !gate.canSend();
  $("t-arm").textContent = st.armed ? "조종 준비 풀기" : "조종 준비";
  const arm = $("arm-chip");
  if (!inTele) chip(arm, "", "해제됨");
  else if (!st.armed) chip(arm, "warn", "준비 안 됨");
  else if (m.teleop.lastRead?.deadman) chip(arm, "tel", "조종 중");
  else chip(arm, "acc", "준비됨 · 누르고 있기 대기");
  const live = inTele && st.armed && lease; // an observer never drives
  $("deadman").disabled = !live;
  $("deadman").dataset.on = String(Boolean(live && m.teleop.lastRead?.deadman));
  $("joy").setAttribute("aria-disabled", String(!live));
  $("lift-up").disabled = $("lift-dn").disabled = !live || st.lift_allowed === false;
  $("lift-val").textContent = `${Number.isFinite(st.lift_m) ? Math.round(st.lift_m * 1000) : "—"} / 280 mm`;
  $("m-direct").setAttribute("aria-pressed", String(gate.mode === "direct"));
  $("m-video").setAttribute("aria-pressed", String(gate.mode === "video"));
  $("direct-confirm").hidden = gate.canSend();
  const lim = st.limits;
  const cmd = st.command;
  const vn = lim.forward_mps ? cmd.speed_mps / lim.forward_mps : 0;
  const sn = lim.curvature_inv_m ? cmd.curvature_inv_m / lim.curvature_inv_m : 0;
  const bar = (el, n) => {
    const x = Math.max(-1, Math.min(1, n));
    el.style.left = x >= 0 ? "50%" : `${50 + x * 50}%`;
    el.style.width = `${Math.abs(x) * 50}%`;
  };
  bar($("b-v"), vn);
  bar($("b-s"), -sn);
  $("b-v-t").textContent = `${cmd.speed_mps.toFixed(2)} m/s`;
  $("b-s-t").textContent = `${cmd.curvature_inv_m.toFixed(2)} 1/m`;
  $("limits").textContent = `상한 전진 ${lim.forward_mps.toFixed(2)} · 후진 ${lim.reverse_mps.toFixed(2)} m/s${lim.provisional ? " (잠정)" : ""}${st.drive_allowed === false ? " · 주행 금지" : ""}`;
  const pr = st.permit.reason;
  chip($("permit-chip"), pr === "ok" ? "ok" : pr.startsWith("occupied") ? "crit" : "warn", pr === "ok" ? "주행 허가 ok" : pr);
  $("input-src").textContent = m.teleop.lastRead ? m.teleop.lastRead.source : "—";

  // camera age (browser-side, for reference only)
  const age = m.camera.lastAt ? (performance.now() - m.camera.lastAt) / 1000 : null;
  $("cam-age").textContent = age === null ? "마지막 프레임 이후 —" : `마지막 프레임 이후 ${age.toFixed(2)} s (브라우저 기준, 참고용)`;
  const banner = $("cam-banner");
  banner.hidden = !(age !== null && age > 1.0);
  banner.textContent = "영상이 멈췄습니다 · 영상 조종은 차량이 거부합니다";
  $("cam-meta").textContent = m.camera.frameId ? `가짜 렌더 · frame ${m.camera.frameId}` : "영상 대기 중";
  $("map-meta").textContent = m.grid ? `map · ${m.grid.head.resolution_m} m 격자 · 스캔 ${m.scanPoints.length / 2}점` : "격자 대기 중";
}
