// The real truck's operator panel on the page (docs/design/2026-11-19-real-safety-wiring.md 5.1, 5c-1b): register a
// pallet, confirm the start, submit with the registered revision, answer the operator check bound to what was
// drawn, finish a field recovery, bypass checks. The decisions are realpanel.js's; this file only draws and sends.

import { answerKey, answerStillOpen, bypassButtons, closedWhy, panelShown, poseOf, submitRevision } from "./realpanel.js";

const $ = (id) => document.getElementById(id);

export function bindRealPanel(m, rejectText) {
  const panel = { registered: null, opKey: null, recoveryId: null, note: "" };
  m.realPanel = panel;
  const say = (text) => {
    panel.note = text;
    const el = $("rp-msg");
    if (el) el.textContent = text;
  };
  const send = (type, fields, okText) => m.session.request(type, fields).then((r) => {
    if (r && r.t === "reject") say(`거부 · ${rejectText(r.reason, r.detail)}`);
    else if (r && okText) say(okText);
    return r;
  });
  $("rp-register").onclick = () => {
    const pose = poseOf($("rp-x").value, $("rp-y").value, $("rp-yaw").value);
    const pallet = $("rp-pallet").value;
    if (!pallet || !pose) return say("팔레트와 자세(x, y, yaw)를 넣으세요");
    send("pallet_register", { pallet_id: pallet, pose }, "등록했습니다").then((r) => {
      if (r && r.t === "ack" && Number.isInteger(r.targets_revision)) {
        panel.registered = r.targets_revision;
        m.session.request("targets_request", {}); // the list with the registered pose
      }
    });
  };
  $("rp-start").onclick = () => send("operator_start", {}, "시작 확인을 보냈습니다");
  $("rp-submit").onclick = () => {
    const rev = submitRevision(panel.registered, m.session.targets);
    if (rev === null) return say("먼저 팔레트를 등록하세요");
    send("mission_submit", { pallet_id: $("rp-pallet").value, destination_id: $("rp-dest").value,
      targets_revision: rev }, "작업 지시를 보냈습니다").then((r) => {
      if (r && r.t === "reject" && r.reason === "stale_targets") panel.registered = null;
    });
  };
  const answer = (verdict) => {
    const key = panel.opKey;
    if (!answerStillOpen(key, m.session.realStatus)) return say("확인 요청이 바뀌었습니다 · 화면을 다시 보세요");
    send("operator_answer", { kind: key.kind, serial: key.serial, verdict }, verdict === "confirm" ? "확인했습니다" : "거부했습니다");
  };
  $("rp-confirm").onclick = () => answer("confirm");
  $("rp-reject").onclick = () => answer("reject");
  $("rp-recovery").onclick = () => {
    if (Number.isInteger(panel.recoveryId)) send("recovery", { recovery_id: panel.recoveryId, action: "done" }, "현장 복구 완료를 보냈습니다");
  };
  $("rp-bypass").onclick = (ev) => {
    const btn = ev.target.closest("button[data-check]");
    if (!btn || btn.disabled) return;
    send("check_bypass", { check: btn.dataset.check, on: btn.dataset.bypassed !== "true" }, "다음 틱에 적용합니다");
  };
}

export function renderRealPanel(m) {
  const el = $("real-panel");
  if (!el) return;
  el.hidden = !panelShown(m.session.realStatus);
  if (el.hidden) return;
  const st = m.session.realStatus;
  const lease = m.session.hasLease;
  const panel = m.realPanel;
  $("rp-status").textContent = st ? `허가 ${st.permit?.state ?? "—"} · 미션 ${st.mission?.status ?? "—"}` : "실차 상태 대기 중";
  // the target list into the two pickers (kept selections)
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
  const why = closedWhy(st, lease);
  for (const id of ["rp-register", "rp-start", "rp-submit"]) $(id).disabled = why !== null;
  $("rp-why").textContent = why ?? "";
  $("rp-start-state").textContent = st?.operator_start ? "확인됨" : "";
  // the operator check: the key bound to what is drawn now
  const key = answerKey(st);
  panel.opKey = key;
  $("rp-op").textContent = key ? `확인 요청 #${key.serial} · ${key.kind}` : "열린 확인 요청 없음";
  $("rp-confirm").disabled = $("rp-reject").disabled = !key || !lease;
  panel.recoveryId = st && Number.isInteger(st.field_recovery) ? st.field_recovery : null;
  $("rp-recovery").hidden = panel.recoveryId === null;
  $("rp-recovery").textContent = panel.recoveryId === null ? "" : `현장 복구 #${panel.recoveryId} 완료`;
  const buttons = bypassButtons(st, lease);
  const sig = buttons.map((b) => `${b.check}:${b.bypassed}:${b.enabled}`).join("|");
  const box = $("rp-bypass");
  if (box.dataset.sig !== sig) {
    box.innerHTML = buttons.map((b) => `<button class="btn${b.bypassed ? " warn" : ""}" type="button" data-check="${b.check}" data-bypassed="${b.bypassed}"${b.enabled ? "" : " disabled"} title="${b.why ?? ""}">${b.label} · ${b.bypassed ? "우회 중" : "켬"}</button>`).join("");
    box.dataset.sig = sig;
  }
}
