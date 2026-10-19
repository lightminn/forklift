// Panel rendering from the latest vehicle state. Buttons are only enabled when the
// request could be accepted; the vehicle still decides and may reject.

import { controlProfile, forbidText, liftControls, recoveryNote, videoStaleAfterS, palletKindLabel, permitText, rearmState, resumeState, resumeSteps, standWhy, stopInstruction, stopKind } from "./profile.js";
import { blockReason, currentDestination, missionProgress, PHASES, selectable, subtaskOf } from "./wizard.js";
import { adviceText, destinationLabel, reasonText, recoveryGuide, sessionButtons } from "./results.js";

const MODE_KO = { AUTO: "자동 주행", TELEOP: "수동 조종", PAUSED: "대기", STOPPED: "정지됨" };
const MODE_CLS = { AUTO: "acc", TELEOP: "tel", PAUSED: "ok", STOPPED: "crit" };
const STATUS = {
  accepted: ["acc", "수락"],
  preparing: ["acc", "준비 중"],
  running: ["acc", "실행 중"],
  paused: ["warn", "일시정지"],
  held: ["warn", "보류"],
  resuming: ["acc", "재개 중"],
  done: ["ok", "완료"],
  failed: ["crit", "실패"],
  cancelled: ["", "취소됨"],
  recovering: ["acc", "복구하며 물러나는 중"],
  recovery_required: ["crit", "복구 필요"],
};
const STEP_KO = {
  stop_confirmed: "정지 확인",
  state_verified: "실제 상태 확인",
  frame_reexpressed: "보정 반영",
  path_rechecked: "경로 재검사",
};
const PALLET_STATE = { available: "대기", carried: "적재 중", delivered: "하역 완료" };

const $ = (id) => document.getElementById(id);
const deg = (rad) => {
  const d = (rad * 180) / Math.PI;
  return (Math.abs(d) < 0.05 ? 0 : d).toFixed(1); // no "-0.0"
};
const esc = (s) => String(s).replace(/[&<>"']/g, (ch) => `&#${ch.charCodeAt(0)};`);

function chip(el, cls, text) {
  el.className = `chip ${cls}`.trim();
  el.innerHTML = cls ? `<i></i>${esc(text)}` : esc(text);
}

function badge(el, cls, text) {
  el.className = `badge ${cls}`.trim();
  el.textContent = text;
}

// "P-1 → D-1 · " when the vehicle names the mission's targets, else nothing
const route = (mi) => (mi.pallet_id ? `${mi.pallet_id} → ${mi.destination_id ?? "?"} · ` : "");

export function renderPallets(targets, onPick) {
  const box = $("pallet-list");
  const checked = document.querySelector('input[name="pal"]:checked')?.value;
  const firstList = box.children.length === 0;
  box.innerHTML = "";
  for (const p of targets.pallets) {
    const lab = document.createElement("label");
    lab.className = "pal";
    const avail = selectable(p);
    lab.innerHTML = `<input type="radio" name="pal" value="${esc(p.pallet_id)}"><span><b>${esc(p.pallet_id)} · ${esc(palletKindLabel(p.kind))}</b><small>${Number.isFinite(p.depth_m) && Number.isFinite(p.width_m) ? `${Math.round(p.depth_m * 1000)} × ${Math.round(p.width_m * 1000)} mm` : "크기 모름"} · ${p.pose ? `(${p.pose.x_m.toFixed(1)}, ${p.pose.y_m.toFixed(1)}) m` : "포크 위"}</small></span><span class="chip ${avail ? "" : "ok"}">${esc(PALLET_STATE[p.state] ?? p.state)}</span>`;
    const input = lab.querySelector("input");
    input.disabled = !avail;
    input.checked = avail && p.pallet_id === checked;
    input.addEventListener("change", onPick);
    box.appendChild(lab);
  }
  // a first list picks the first available pallet; later lists never swap in another one
  if (firstList && !document.querySelector('input[name="pal"]:checked')) {
    const first = box.querySelector("input:not(:disabled)");
    if (first) first.checked = true;
  }
  const dest = $("dest");
  const firstDest = dest.options.length === 0;
  const keep = dest.value;
  dest.innerHTML =
    '<option value="">하역 위치를 고르세요</option>' +
    targets.destinations
      .map((d) => `<option value="${esc(d.destination_id)}">${esc(destinationLabel(d.destination_id))} · (${d.pose.x_m.toFixed(1)}, ${d.pose.y_m.toFixed(1)}) m</option>`)
      .join("");
  if (keep && targets.destinations.some((d) => d.destination_id === keep)) dest.value = keep;
  else if (firstDest && targets.destinations.length) dest.value = targets.destinations[0].destination_id;
  else dest.value = "";
  $("targets-rev").textContent = `rev ${targets.targets_revision}`;
}

const PLAY = '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M8 5v14l11-7z"/></svg>';

function subtaskList(t) {
  return t.subtasks.map((s, i) => `<li class="${s.kind}"><b>${String(i + 1).padStart(2, "0")}</b><span>${esc(s.name)}</span></li>`).join("");
}

export function renderTemplates(list, selected, onPick) {
  const box = $("wz-templates");
  if (!list.length) {
    box.innerHTML = '<div class="notice">이 차량이 지원하는 작업 종류가 없거나 검색과 맞는 것이 없습니다.</div>';
    return;
  }
  box.innerHTML = list
    .map(
      (t) => `<button type="button" class="tpl" data-code="${esc(t.code)}" aria-pressed="${t.code === selected}">
        <span class="tpl-ico">${PLAY}</span>
        <span class="tpl-name">${esc(t.name)}<span class="tpl-tag">console/1</span></span>
        <span class="chev" aria-hidden="true">›</span>
        <span class="tpl-code">${esc(t.code)} · ${t.subtasks.length}단계</span>
        <span class="tpl-sub">${esc(t.summary)}</span>
        <ol class="subtasks">${subtaskList(t)}</ol>
      </button>`,
    )
    .join("");
  for (const el of box.querySelectorAll(".tpl")) el.addEventListener("click", () => onPick(el.dataset.code));
}

export function renderWizard(wz, ctx) {
  for (const li of document.querySelectorAll(".wz-steps li")) {
    const s = Number(li.dataset.step);
    if (s === wz.step) li.setAttribute("aria-current", "step");
    else li.removeAttribute("aria-current");
    li.classList.toggle("done", s < wz.step);
  }
  for (const p of document.querySelectorAll("[data-step-panel]")) p.hidden = Number(p.dataset.stepPanel) !== wz.step;
  $("wz-back").hidden = wz.step === 1;
  $("wz-next").hidden = wz.step === 3;
  $("m-start").hidden = wz.step !== 3;
  $("wz-next").disabled = !ctx.nextOk;
  const why = blockReason(ctx.block);
  $("m-start").disabled = Boolean(why) || !ctx.selection.pallet || !ctx.selection.destination;
  $("wz-why").textContent = ctx.note || (wz.step === 3 ? why ?? "" : "");
  if (wz.step === 3 && ctx.template) {
    const sel = ctx.selection;
    $("wz-review").innerHTML = `<dt>작업 종류</dt><dd>${esc(ctx.template.name)} <span class="mono hint">${esc(ctx.template.code)}</span></dd>
      <dt>팔레트</dt><dd class="mono">${esc(sel.pallet ?? "—")}</dd>
      <dt>하역 위치</dt><dd class="mono">${esc(sel.destination ?? "—")}</dd>
      <dt>대상 목록</dt><dd class="mono">rev ${esc(sel.revision ?? "—")}</dd>`;
    $("wz-review-steps").innerHTML = subtaskList(ctx.template);
  }
}

export function log(text, level = "") {
  const li = document.createElement("li");
  if (level) li.className = level;
  li.innerHTML = `<time>${new Date().toTimeString().slice(0, 8)}</time><span></span>`;
  li.querySelector("span").textContent = text;
  const ol = $("log");
  ol.prepend(li);
  while (ol.children.length > 200) ol.lastChild.remove();
}

export function render(m) {
  const st = m.session.state;
  const gens = m.session.generations;
  const lease = m.session.hasLease;
  $("gens").textContent = `epoch ${gens.session_epoch ?? "—"} · control ${gens.control_gen ?? "—"} · arm ${gens.arm_gen ?? "—"}`;
  const w = m.session.welcome;
  const backend = w ? { mock: "가짜 차량", isaac: "Isaac Sim", real: "실차" }[w.backend] ?? w.backend : "연결 중";
  $("vehicle-id").textContent = w?.vehicle_id ?? "—";
  $("backend-label").textContent = backend;
  $("d-id").textContent = w?.vehicle_id ?? "—";
  $("d-sub").textContent = w ? `${backend} · session ${w.session_epoch}${w.capabilities?.provisional_limits && w.capabilities?.control !== false ? " · 잠정 상한" : ""}` : "연결 중";
  $("l-ep").textContent = `ws://${location.host}/api/v1/control`;
  chip($("link-chip"), m.link.up ? "ok" : "crit", m.link.up ? "연결됨" : "끊김");
  $("l-rtt").textContent = m.link.up && m.link.tickGapMs !== null ? `${Math.round(m.link.tickGapMs)} ms` : "—";
  $("lease").textContent = lease ? "조종 권한: 이 기기" : "조종 권한 받기";
  $("lease").setAttribute("aria-pressed", String(lease));
  const prof = controlProfile(w?.capabilities);
  const monitor = w?.capabilities?.control === false; // explicit only: before a welcome STOP still sends
  $("monitor-alert").hidden = !monitor;
  if (monitor) {
    const how = stopInstruction(w);
    $("monitor-alert").textContent = `관찰 전용 연결 · 이 화면으로 차량을 멈출 수 없습니다 · ${how}`;
    $("nostop-how").textContent = how;
  }
  $("draft").hidden = w?.backend !== "mock";
  for (const id of ["stop", "stop2"]) {
    const b = $(id);
    b.classList.toggle("nostop-btn", monitor);
    b.querySelector("b").textContent = monitor ? "정지 불가" : "STOP";
    b.querySelector("small").textContent = monitor ? "이 연결은 관찰 전용" : "편의 정지 · 비상정지 아님";
  }
  $("lease").disabled = monitor;
  $("lease").title = monitor ? "관찰 전용 연결입니다" : "";
  $("tab-tele").disabled = !prof.teleop; // monitor only: nothing to steer with
  for (const [id, be] of [["be-real", "real"], ["be-isaac", "isaac"], ["be-mock", "mock"]]) $(id).setAttribute("aria-pressed", String(w?.backend === be));
  if (!st) {
    $("m-open").disabled = true;
    return;
  }

  const mode = st.mode; // null: no arbiter behind this connection (monitor only)
  $("mode").dataset.m = mode ?? "NONE";
  $("mode-code").textContent = mode ?? "—";
  $("mode-ko").textContent = mode ? MODE_KO[mode] ?? mode : "판정 없음";
  const sk = stopKind(st.stop_kind);
  chip($("stop-kind"), sk.level, sk.label);
  const rearm = rearmState(st, lease);
  $("rearm").hidden = !rearm.shown;
  $("rearm").disabled = !rearm.enabled;
  $("rearm").title = rearm.why;
  $("rearm").textContent = rearm.why && lease ? `재무장 · ${rearm.why}` : "재무장";
  $("l-peers").textContent = monitor
    ? "알 수 없음 · 이 연결은 조종 권한을 다루지 않습니다"
    : `${st.lease?.holder ? (lease ? "이 기기(조종)" : "다른 기기(조종)") : "조종 기기 없음"} · 관찰 ${st.lease?.observers ?? 0}대${st.lease?.reserved_until_s ? ` · 재연결 대기 ${st.lease.reserved_until_s.toFixed(1)} s` : ""}`;
  $("l-ttl").textContent = monitor ? "해당 없음 · 이 연결은 명령을 보내지 않습니다" : "0.30 s · 차량 시계 (잠정)";
  $("l-policy").textContent = monitor ? "이 연결과 무관 · 차량 쪽 동작은 이 화면이 알 수 없습니다" : "자동 주행은 계속 · 수동 조종은 만료로 정지";

  // badges
  badge($("b-lease"), lease ? "ok" : st.lease?.holder ? "warn" : "", monitor ? "모름" : lease ? "이 기기" : st.lease?.holder ? "다른 기기" : "없음");
  badge($("b-mode"), MODE_CLS[mode] ?? "", mode ? MODE_KO[mode] ?? mode : "판정 없음");
  const mi = st.mission;
  const [mcls, mko] = mi ? STATUS[mi.status] ?? ["", mi.status] : ["", monitor ? "모름" : "작업 없음"];
  badge($("b-work"), mcls, mko);

  // progress strip and destination
  const prog = $("d-progress");
  const active = mi && ["accepted", "preparing", "running", "paused", "held", "resuming", "recovering"].includes(mi.status);
  if (mi) {
    const ph = PHASES.find(([en]) => en === mi.phase);
    $("req-status-line").textContent = `${mko}${ph && mi.status !== "done" ? ` · ${ph[1]}` : ""}${mi.reason ? ` · ${mi.reason}` : ""}`;
    prog.dataset.tone = mcls;
    const pct = missionProgress(mi);
    $("d-prog-fill").style.width = `${(pct * 100).toFixed(1)}%`;
    $("d-prog-pct").textContent = `${route(mi)}${Math.round(pct * 100)}%`;
  } else {
    $("req-status-line").textContent = monitor ? "작업 정보 없음 · 관찰 전용 연결" : "진행 중인 작업 없음";
    prog.dataset.tone = "";
    $("d-prog-fill").style.width = "0";
    $("d-prog-pct").textContent = "";
  }
  $("d-dest").textContent = currentDestination(mi) ?? (monitor ? "알 수 없음 (관찰 전용)" : "지정된 목적지 없음");
  renderSession(m, st, lease, prof, monitor);

  // mission
  const why = blockReason({ state: st, canMission: prof.mission, hasLease: lease, mode, mission: mi });
  $("mission-gate").textContent = "";
  const ready = $("ready-line");
  ready.className = `ready ${why ? "warn" : "ok"}`;
  ready.querySelector("span").textContent = why ?? "작업을 받을 수 있습니다";
  $("m-open").disabled = Boolean(why);
  // a recovery is paused, not cancelled (persistent console plan D4)
  $("m-pause").disabled = !lease || !mi || !["running", "recovering"].includes(mi.status);
  // the mission now is a recovery: paused or resuming too, never a new mission still in its old phase (D4)
  const recovery = Boolean(mi && (String(mi.phase || "").startsWith("recover_") || mi.phase === "clear_out")
    && ["recovering", "paused", "resuming", "held"].includes(mi.status));
  $("m-cancel").title = recovery ? "복구 중에는 취소 대신 일시정지합니다" : "";
  const resume = resumeState({ state: st, mission: mi, hasLease: lease });
  $("m-resume").disabled = !resume.enabled;
  $("m-resume").title = resume.why;
  $("m-cancel").disabled = !lease || !active || recovery;
  if (mi) {
    $("req-id").textContent = `${mi.mission_id}${mi.pallet_id ? ` · ${mi.pallet_id} → ${mi.destination_id ?? "?"}` : ""}`;
    chip($("req-status"), mcls, mi.reason ? `${mko} · ${mi.reason}` : mko);
    const sub = subtaskOf(mi.phase);
    const ph = PHASES.find(([en]) => en === mi.phase);
    $("cur-step").textContent = mi.status === "done" ? "완료" : `${sub >= 0 && sub < 4 ? `${String(sub + 1).padStart(2, "0")} · ` : ""}${ph ? ph[1] : mi.phase}`;
  } else {
    $("req-id").textContent = "요청 없음";
    chip($("req-status"), "", "—");
    $("cur-step").textContent = "—";
  }
  const key = mi ? `${mi.mission_id}:${mi.phase}:${mi.status}` : "none";
  if (key !== m.cache.phaseKey) {
    m.cache.phaseKey = key;
    const idx = mi ? PHASES.findIndex(([en]) => en === mi.phase) : -1;
    $("phases").innerHTML = PHASES.map(([en, ko], i) => {
      let cls = "";
      if (mi && (i < idx || mi.status === "done")) cls = "done";
      else if (mi && i === idx) cls = `cur${["paused", "held"].includes(mi.status) ? " held" : ""}`;
      return `<li class="${cls}" title="${en}"><i class="tick"></i><span>${ko}</span></li>`;
    }).join("");
  }
  const steps = resumeSteps(mi?.resume_steps);
  $("resume-steps").hidden = !steps;
  if (steps) {
    $("resume-steps").innerHTML = steps
      .map((s) => `<li class="${esc(s.result)}"><span class="mark">${s.result === "ok" ? "✓" : s.result === "failed" ? "✕" : "…"}</span><span>${esc(STEP_KO[s.name] ?? s.name)}${s.reason ? `<small>${esc(s.reason)}</small>` : ""}</span></li>`)
      .join("");
  }
  const forbidden = st.teleop_forbidden_reason;
  $("forbid").hidden = !forbidden || forbidden === "stopped_locked";
  $("forbid-reason").textContent = forbidText(forbidden);
  $("recover").disabled = !lease || mode === "STOPPED";

  // position and readouts
  const pose = st.pose;
  $("r-x").textContent = pose ? pose.x_m.toFixed(3) : "—";
  $("r-y").textContent = pose ? pose.y_m.toFixed(3) : "—";
  $("r-yaw").textContent = pose ? deg(pose.yaw_rad) : "—";
  $("d-frame").textContent = pose ? `${pose.frame ?? "map"} · 보정 ${pose.correction_version ?? "—"}` : "위치 없음";
  $("r-v").textContent = Number.isFinite(st.rear_axle_speed_mps) ? st.rear_axle_speed_mps.toFixed(2) : "—";
  $("r-st").textContent = st.steering_rad ? deg(st.steering_rad[0]) : "—";
  $("r-lift").textContent = Number.isFinite(st.lift_m) ? Math.round(st.lift_m * 1000) : "—";
  $("r-load").textContent = st.load?.state === "loaded" ? `적재 · ${st.load.pallet_id ?? "팔레트 모름"}` : st.load?.state === "empty" ? "없음" : "측정 안 됨";
  $("r-permit").textContent = permitText(st.permit, st.teleop_forbidden_reason);
  $("r-cmd").textContent = st.command && st.applied ? `${st.command.speed_mps.toFixed(2)} → ${st.applied.speed_mps.toFixed(2)} m/s` : "—";
  if (m.chassis) $("r-radius").textContent = m.chassis.model_turning_radius_m.toFixed(2);
  $("r-phase-t").textContent = mi && Number.isFinite(mi.phase_elapsed_s) ? `${mi.phase_elapsed_s.toFixed(1)} s (일시정지 동안 멈춤)` : "—";
  const fr = st.freshness || {};
  const parts = [["격자", fr.grid_free_age_s], ["스캔", fr.scan_age_s], ["SLAM 보정", fr.slam_correction_age_s], ["근접 관측", fr.near_obs_age_s]];
  $("r-fresh").textContent = parts.map(([k, v]) => `${k} ${v === null || v === undefined ? "측정 안 됨" : `${v.toFixed(1)} s`}`).join(" · ");
  $("r-still").textContent = st.stopped_confirmed ? "정지 확인됨" : "확인 안 됨";
  if (!pose) chip($("loc-chip"), "warn", "위치 없음");
  else chip($("loc-chip"), pose.source === "mock_truth" ? "" : "ok", pose.source === "mock_truth" ? "정답 위치(가짜 차량)" : pose.source);

  // teleop
  const inTele = mode === "TELEOP";
  const enterStand = inTele ? null : standWhy(st, { waivable: true }); // refused until the truck stands (or waived)
  // plan D14: a state that only lowers may be entered (teleop_allowed), though driving is forbidden there
  const teleopOk = st.teleop_allowed === undefined ? !forbidden : st.teleop_allowed !== false;
  $("t-enter").disabled = !prof.teleop || !lease || inTele || mode === "STOPPED" || !teleopOk || Boolean(enterStand);
  $("t-enter").title = enterStand ?? "";
  $("t-exit").disabled = !lease || !inTele;
  const gate = m.teleop.gate;
  $("t-arm").disabled = !lease || !inTele || !gate.canSend();
  $("t-arm").textContent = st.armed ? "조종 준비 풀기" : "조종 준비";
  const arm = $("arm-chip");
  if (!inTele) chip(arm, "", "해제됨");
  else if (!st.armed) chip(arm, "warn", "준비 안 됨");
  else if (m.teleop.lastRead?.deadman) chip(arm, "tel", "조종 중");
  else chip(arm, "acc", "준비됨");
  const live = inTele && st.armed && lease; // an observer never drives
  $("deadman").disabled = !live;
  $("deadman").dataset.on = String(Boolean(live && m.teleop.lastRead?.deadman));
  $("joy").setAttribute("aria-disabled", String(!live));
  const lc = liftControls(st); // plan D14: each direction on its own
  $("lift-up").disabled = !live || !prof.lift || !lc.up;
  $("lift-dn").disabled = !live || !prof.lift || !lc.down;
  $("lift-val").textContent = `${Number.isFinite(st.lift_m) ? Math.round(st.lift_m * 1000) : "—"} / ${lc.limitMm} mm`;
  $("m-direct").setAttribute("aria-pressed", String(gate.mode === "direct"));
  $("m-video").setAttribute("aria-pressed", String(gate.mode === "video"));
  $("direct-confirm").hidden = gate.canSend();
  const lim = st.limits ?? { forward_mps: 0, reverse_mps: 0, curvature_inv_m: 0, provisional: false };
  const cmd = st.command ?? { speed_mps: 0, curvature_inv_m: 0 };
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
  const note = recoveryNote(st);
  $("limits").textContent = st.limits ? `상한 전진 ${lim.forward_mps.toFixed(2)} · 후진 ${lim.reverse_mps.toFixed(2)} m/s${lim.provisional ? " (잠정)" : ""}${st.drive_allowed === false ? " · 주행 금지" : ""}${note ? ` · ${note}` : ""}` : "상한 —";
  const pr = st.permit?.reason;
  if (!pr) chip($("permit-chip"), "", "허가 판정 없음");
  else chip($("permit-chip"), pr === "ok" ? "ok" : pr.startsWith("occupied") ? "crit" : "warn", pr === "ok" ? "주행 허가 ok" : pr);
  $("input-src").textContent = m.teleop.lastRead ? m.teleop.lastRead.source : "—";

  // camera age (browser-side, for reference only)
  const age = m.camera.lastAt ? (performance.now() - m.camera.lastAt) / 1000 : null;
  $("cam-age").textContent = age === null ? "마지막 프레임 이후 —" : `마지막 프레임 ${age.toFixed(2)} s 전 (브라우저 기준)`;
  const banner = $("cam-banner");
  banner.hidden = !(age !== null && age > videoStaleAfterS(st));
  banner.textContent = "영상이 멈췄습니다 · 영상 조종은 차량이 거부합니다";
  $("cam-meta").textContent = m.camera.frameId ? `frame ${m.camera.frameId}` : "영상 대기 중";
  $("map-meta").textContent = m.grid ? `${Number(m.grid.head.resolution_m.toPrecision(4))} m 격자 · 스캔 ${m.scan ? m.scan.points.length / 2 : 0}점` : "격자 대기 중";
  const notes = [...(st.notes ?? []), ...(m.frameNote ? [m.frameNote] : [])];
  $("notes").hidden = notes.length === 0;
  $("notes").textContent = notes.join(" · ");
}

const OUTCOME = { done: ["ok", "완료"], failed: ["crit", "실패"], cancelled: ["warn", "취소"], recovery_required: ["crit", "복구 필요"] };

function wallTime(unix) {
  if (!Number.isFinite(unix)) return "";
  const d = new Date(unix * 1000);
  return [d.getHours(), d.getMinutes(), d.getSeconds()].map((v) => String(v).padStart(2, "0")).join(":");
}

// Persistent console plan D13: the scene banner, the results book, the recovery guidance, the advice, the session
// buttons. The vehicle decides; these show what it said and enable a button only when it could be taken.
function renderSession(m, stIn, lease, prof, monitor) {
  const st = stIn ?? {};
  const damaged = st.scene?.damaged ?? null;
  $("scene-banner").hidden = !damaged;
  if (damaged) $("scene-text").textContent = `${reasonText(damaged.reason).text}${damaged.unix ? ` · ${wallTime(damaged.unix)}` : ""}`;
  const note = $("session-note");
  note.hidden = !m.sessionNote;
  note.textContent = m.sessionNote ?? "";

  const book = m.results;
  if (book && book.version !== m.cache.resultsVersion) {
    m.cache.resultsVersion = book.version;
    const rows = book.rows();
    $("results-list").innerHTML = rows.length ? rows.map((r) => {
      const [cls, ko] = OUTCOME[r.outcome] ?? ["", r.outcome];
      const why = r.reason ? reasonText(r.reason) : null;
      const advice = adviceText(r.advice);
      const span = Number.isFinite(r.started_s) && Number.isFinite(r.ended_s) ? ` · ${(r.ended_s - r.started_s).toFixed(0)} s` : "";
      return `<li><span class="chip ${cls}">${esc(ko)}</span><span class="what">#${r.seq} ${esc(r.pallet_id ?? "?")} → ${esc(destinationLabel(r.destination_id ?? "?"))}</span><span class="when">${esc(wallTime(r.ended_unix))}${span}</span>${why ? `<span class="why">${esc(why.text)}${why.text !== why.code ? ` <span class="mono">(${esc(why.code)})</span>` : ""}</span>` : ""}${advice ? `<span class="advice">${esc(advice)}</span>` : ""}${r.record_error ? `<span class="why">기록 파일을 쓰지 못함 · ${esc(r.record_error)}</span>` : ""}</li>`;
    }).join("") : '<li class="hint">아직 끝난 미션이 없습니다</li>';
    $("results-older").textContent = book.older ? `앞선 ${book.older}건은 기록 파일(missions/<id>/result.json)에만` : "";
  }

  const guide = recoveryGuide(st);
  const g = $("recovery-guide");
  g.hidden = !guide;
  if (guide) g.innerHTML = `<b>수동 복구 · ${esc(guide.next)}</b><br>${guide.notes.map(esc).join(" · ")}${guide.permit ? `<br>${esc(guide.permit)}` : ""}`;
  // the advice of the last result, while no newer mission runs (it is from the pose at that failure)
  const last = st.last_result;
  const active = st.mission && ["accepted", "preparing", "running", "paused", "held", "resuming", "recovering"].includes(st.mission.status);
  const advice = !active && last?.advice ? adviceText(last.advice) : null;
  const a = $("advice-line");
  a.hidden = !advice;
  if (advice) a.textContent = `${advice}${st.permit?.reason ? ` · 현재 구동 허가: ${permitText(st.permit, st.teleop_forbidden_reason)}` : ""}`;

  const caps = m.session.welcome?.capabilities ?? {};
  const btn = sessionButtons(st, { hasLease: lease && !monitor && prof.control, caps });
  for (const [id, kind, b, whyId] of [["s-end", "session_end", btn.end, "s-end-why"], ["s-reset", "sim_reset", btn.reset, "s-reset-why"], ["scene-reset", "sim_reset", btn.reset, null]]) {
    const el = $(id);
    el.hidden = !b.shown;
    const armed = m.confirm?.pending(m.confirmKey?.(kind), performance.now());
    el.disabled = !b.enabled;
    el.dataset.confirm = armed ? "armed" : "";
    el.textContent = armed ? (kind === "sim_reset" ? "정말 초기화? (5 s)" : "정말 종료? (5 s)") : (kind === "sim_reset" ? "장면 초기화" : "세션 종료");
    el.title = b.why;
    if (whyId) $(whyId).textContent = b.why;
    if (!b.enabled && armed) m.confirm.cancel(); // the condition broke: the confirm is gone
  }
}
