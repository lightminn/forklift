// The real truck's operator panel on the page (docs/design/2026-11-19-real-safety-wiring.md 5.1, 5c-1b): register a
// pallet, confirm the start, submit with the registered revision, answer the operator check bound to what was
// pressed, finish a field recovery, bypass checks, rearm. The decisions are realpanel.js's; this file draws and sends.

import { answerKey, answerStillOpen, bypassButtons, panelShown, poseOf, statusIsMine, submitClosedWhy,
  submitRevision, toggleShown } from "./realpanel.js";

const $ = (id) => document.getElementById(id);
const TOGGLE_WAIT_MS = 2000; // a toggle not seen in real_status by then: told it did not apply

export function bindRealPanel(m, rejectText, clientId) {
  const panel = { registered: null, opKey: null, pressKey: null, toggle: null, targetsAsked: null, note: "" };
  m.realPanel = panel;
  const say = (text) => {
    panel.note = text;
    const el = $("rp-msg");
    if (el) el.textContent = text;
  };
  const gens = () => m.session.generations;
  // what the buttons were drawn for: the session and the lease (5c-1b r1 P2-2) -- a send under another is dropped
  const drawnFor = () => ({ epoch: gens().session_epoch, lease: gens().lease_id });
  const sameAsDrawn = (d) => d && d.epoch === gens().session_epoch && d.lease === gens().lease_id
    && statusIsMine(m.session.realStatus, gens(), clientId);
  const send = (type, fields, okText) => m.session.request(type, fields).then((r) => {
    if (r && r.t === "reject") say(`거부 · ${rejectText(r.reason, r.detail)}`);
    else if (r && okText) say(okText);
    return r;
  });
  $("rp-register").onclick = () => {
    const d = drawnFor();
    const pose = poseOf($("rp-x").value, $("rp-y").value, $("rp-yaw").value);
    const pallet = $("rp-pallet").value;
    if (!pallet || !pose) return say("팔레트와 자세(x, y, yaw)를 넣으세요");
    if (!sameAsDrawn(d)) return say("세션이나 권한이 바뀌었습니다 · 다시 보세요");
    send("pallet_register", { pallet_id: pallet, pose }, "등록했습니다").then((r) => {
      if (r && r.t === "ack" && Number.isInteger(r.targets_revision)) {
        panel.registered = { session_epoch: d.epoch, revision: r.targets_revision };
        m.session.request("targets_request", {}); // the list with the registered pose
      }
    });
  };
  $("rp-start").onclick = () => {
    if (!sameAsDrawn(drawnFor())) return say("세션이나 권한이 바뀌었습니다");
    send("operator_start", {}, "시작 확인을 보냈습니다");
  };
  $("rp-submit").onclick = () => {
    const rev = submitRevision(panel.registered, m.session.realStatus);
    if (rev === null) return say("팔레트를 (다시) 등록하세요");
    send("mission_submit", { pallet_id: $("rp-pallet").value, destination_id: $("rp-dest").value,
      targets_revision: rev }, "작업 지시를 보냈습니다").then((r) => {
      if (r && r.t === "reject" && r.reason === "stale_targets") {
        panel.registered = null; // register again with the new list (r1 P2-4)
        m.session.request("targets_request", {});
      }
    });
  };
  // the operator answer: the key of the request on screen when the press began (r1 P2-1)
  const press = () => { panel.pressKey = panel.opKey ? { ...panel.opKey } : null; };
  for (const id of ["rp-confirm", "rp-reject"]) {
    $(id).onpointerdown = press;
    $(id).onkeydown = (ev) => { if (ev.key === "Enter" || ev.key === " ") press(); };
  }
  const answer = (verdict) => {
    const key = panel.pressKey ?? panel.opKey;
    panel.pressKey = null;
    if (!answerStillOpen(key, m.session.realStatus)) return say("확인 요청이 바뀌었습니다 · 화면을 다시 보세요");
    send("operator_answer", { kind: key.kind, serial: key.serial, verdict }, verdict === "confirm" ? "확인했습니다" : "거부했습니다");
  };
  $("rp-confirm").onclick = () => answer("confirm");
  $("rp-reject").onclick = () => answer("reject");
  $("rp-recovery").onclick = () => {
    const st = m.session.realStatus;
    const drawn = panel.recoveryId;
    // the newest number, the same as drawn, under the session it was drawn for (r1 P2-2)
    if (!Number.isInteger(drawn) || !st || st.field_recovery !== drawn || !sameAsDrawn(panel.recoveryFor)) {
      return say("현장 복구 번호가 바뀌었습니다 · 다시 보세요");
    }
    send("recovery", { recovery_id: drawn, action: "done" }, "현장 복구 완료를 보냈습니다");
  };
  $("rp-rearm").onclick = () => send("rearm", {}, "재무장을 보냈습니다");
  $("rp-bypass").onclick = (ev) => {
    const btn = ev.target.closest("button[data-check]");
    if (!btn || btn.disabled) return;
    const toggle = { check: btn.dataset.check, on: btn.dataset.bypassed !== "true", at: performance.now() };
    send("check_bypass", { check: toggle.check, on: toggle.on }, "다음 틱에 적용합니다").then((r) => {
      if (r && r.t === "ack") panel.toggle = toggle; // the submit waits until real_status shows it (r1 P2-3)
    });
  };
}

export function renderRealPanel(m, clientId) {
  const el = $("real-panel");
  if (!el) return;
  const st = m.session.realStatus;
  el.hidden = !panelShown(st);
  if (el.hidden) return;
  const panel = m.realPanel;
  const gens = m.session.generations;
  const mine = statusIsMine(st, gens, clientId);
  if (panel.registered && panel.registered.session_epoch !== gens.session_epoch) panel.registered = null;
  // the target list once per session when none came with the welcome (r1 P2-5)
  if (!m.session.targets && panel.targetsAsked !== gens.session_epoch && gens.session_epoch !== null) {
    panel.targetsAsked = gens.session_epoch;
    m.session.request("targets_request", {});
  }
  // a toggle shown, or not by its deadline (then told; r1 P2-3)
  if (panel.toggle && (toggleShown(panel.toggle, st) || performance.now() - panel.toggle.at > TOGGLE_WAIT_MS)) {
    if (!toggleShown(panel.toggle, st)) {
      panel.note = "우회 변경이 적용되지 않았습니다 · 실차 상태의 우회 목록을 보세요";
      $("rp-msg").textContent = panel.note;
    }
    panel.toggle = null;
  }
  $("rp-status").textContent = st ? `허가 ${st.permit?.state ?? "—"} · 미션 ${st.mission?.status ?? "—"}` : "실차 상태 대기 중";
  const fill = (sel, items, idKey) => {
    const want = items.map((x) => x[idKey]).join("|");
    if (sel.dataset.items === want) return;
    const keep = sel.value;
    sel.innerHTML = items.map((x) => `<option value="${x[idKey]}">${x[idKey]}</option>`).join("");
    sel.dataset.items = want;
    if (items.some((x) => x[idKey] === keep)) sel.value = keep;
  };
  fill($("rp-pallet"), m.session.targets?.pallets ?? [], "pallet_id");
  fill($("rp-dest"), m.session.targets?.destinations ?? [], "destination_id");
  const why = mine ? submitClosedWhy(st, true, Boolean(panel.toggle)) : "이 콘솔의 실차 상태가 아닙니다(권한·세션)";
  for (const id of ["rp-register", "rp-start", "rp-submit"]) $(id).disabled = why !== null;
  $("rp-why").textContent = why ?? "";
  $("rp-rearm").hidden = !(mine && (st.permit?.state === "LATCHED" || st.mode === "STOPPED"));
  $("rp-start-state").textContent = st?.operator_start ? "확인됨" : "";
  const key = answerKey(st);
  panel.opKey = key;
  $("rp-op").textContent = key ? `확인 요청 #${key.serial} · ${key.kind}` : "열린 확인 요청 없음";
  $("rp-confirm").disabled = $("rp-reject").disabled = !key || !mine;
  panel.recoveryId = st && Number.isInteger(st.field_recovery) ? st.field_recovery : null;
  panel.recoveryFor = { epoch: gens.session_epoch, lease: gens.lease_id };
  $("rp-recovery").hidden = panel.recoveryId === null;
  $("rp-recovery").disabled = !mine;
  $("rp-recovery").textContent = panel.recoveryId === null ? "" : `현장 복구 #${panel.recoveryId} 완료`;
  const buttons = bypassButtons(st, mine);
  const sig = buttons.map((b) => `${b.check}:${b.bypassed}:${b.enabled}:${b.why}`).join("|"); // the reason too (r1 P3)
  const box = $("rp-bypass");
  if (box.dataset.sig !== sig) {
    box.innerHTML = buttons.map((b) => `<button class="btn${b.bypassed ? " warn" : ""}" type="button" data-check="${b.check}" data-bypassed="${b.bypassed}"${b.enabled ? "" : " disabled"} title="${b.why ?? ""}">${b.label} · ${b.bypassed ? "우회 중" : "켬"}</button>`).join("");
    box.dataset.sig = sig;
  }
}
