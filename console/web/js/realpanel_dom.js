// The real truck's operator panel on the page (docs/design/2026-11-19-real-safety-wiring.md 5.1, 5c-1b): register a
// pallet, confirm the start, submit with the registered revision, answer the operator check bound to what was
// pressed, finish a field recovery, bypass checks, rearm. The decisions are realpanel.js's; this file draws and sends.

import { answerKey, answerStillOpen, bypassButtons, panelShown, poseOf, statusIsMine, statusSig, submitClosedWhy,
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
  // every send: the session, the lease and the status object the buttons were last drawn for, still current and this
  // console's (5c-1b r1 P2-2, r2 P2-2) -- else dropped
  const ctxNow = () => ({ epoch: gens().session_epoch, lease: gens().lease_id, sig: statusSig(m.session.realStatus) });
  const same = (a, b) => Boolean(a && b && a.epoch === b.epoch && a.lease === b.lease && a.sig === b.sig);
  const inputOf = (ev) => (ev && Number.isInteger(ev.pointerId) && ev.pointerId >= 0 ? `p${ev.pointerId}` : "kbd");
  // every panel button: the context (session, lease, the status a click depends on) taken when its press began,
  // per button and per input; the click compares that one (else what was drawn) with now (r5 P2: a press begun
  // before a second STOP does not rearm after the redraw). A cancelled pointer drops its record (r5 P3).
  panel.press = {};
  panel.lastPointer = {}; // button -> its latest pointer input (r7 P2-2)
  const panelEl = $("real-panel");
  const begin = (ev) => {
    const btn = ev.target.closest("button");
    if (!btn || !panelEl.contains(btn)) return;
    const key = btn.id || `check:${btn.dataset.check}`;
    // what was drawn, not the newest state (r6 P2-1: a press on a stale drawing can never pass)
    const rec = panel.drawn ? { ...panel.drawn } : null;
    (panel.press[key] ??= {})[inputOf(ev)] = rec;
    if (inputOf(ev) !== "kbd") panel.lastPointer[key] = inputOf(ev); // a click without a pointer id (older Safari)
  };
  panelEl.addEventListener("pointerdown", begin, true);
  panelEl.addEventListener("keydown", (ev) => {
    if (ev.key !== "Enter") return;
    if (ev.repeat) { ev.preventDefault(); return; } // a held Enter clicks no panel button again (r6 P2-2)
    begin(ev);
  }, true);
  // a pointer released off a button: no click follows there -- its press on that button is over (r9 P2)
  document.addEventListener("pointerup", (ev) => {
    const input = inputOf(ev);
    for (const [key, rec] of Object.entries(panel.press)) {
      if (!(input in rec)) continue;
      const btn = key.startsWith("check:") ? panelEl.querySelector(`button[data-check="${key.slice(6)}"]`) : $(key);
      if (!btn || !btn.contains(ev.target)) {
        delete rec[input];
        if (panel.pressKey[key]) delete panel.pressKey[key][input];
        if (panel.lastPointer[key] === input) delete panel.lastPointer[key];
      }
    }
  }, true);
  panelEl.addEventListener("pointercancel", (ev) => {
    for (const rec of Object.values(panel.press)) delete rec[inputOf(ev)];
    for (const rec of Object.values(panel.pressKey)) delete rec[inputOf(ev)];
  }, true);
  // a click's input: its pointer id; none (a MouseEvent click, older Safari) after a pointer press on this button: that
  // press's input -- not the keyboard's, never the newest drawing (r7 P2-2)
  const clickInput = (key, ev) => {
    const input = inputOf(ev);
    if (input !== "kbd") return input;
    const rec = panel.press[key] || {};
    if ("kbd" in rec) return "kbd";
    const pointers = Object.keys(rec).filter((k) => k !== "kbd");
    if (pointers.length === 1) return pointers[0]; // the one pointer press this click can be
    // two pointer presses, or a pointer press since cancelled: which one clicked is not known -- refused (r8 P2),
    // and the button's records start over so the next tap is judged afresh (r9 P2)
    if (pointers.length > 1 || panel.lastPointer[key]) {
      for (const k of pointers) {
        delete rec[k];
        if (panel.pressKey[key]) delete panel.pressKey[key][k];
      }
      delete panel.lastPointer[key];
      return null;
    }
    return "kbd";
  };
  const pressCtx = (key, ev) => {
    const rec = panel.press[key];
    const input = clickInput(key, ev);
    if (input === null) return { ambiguous: true };
    const ctx = rec && input in rec ? rec[input] : null;
    if (rec) delete rec[input];
    return ctx;
  };
  const clickOk = (key, ev) => {
    const ctx = pressCtx(key, ev) ?? panel.drawn;
    if (ctx && ctx.ambiguous) return false;
    return same(ctx, ctxNow()) && same(panel.drawn, ctxNow()) && panel.mine;
  };
  const guard = (fn, key) => (ev) => {
    const input = clickInput(key, ev);
    if (!clickOk(key, ev)) {
      if (panel.pressKey && panel.pressKey[key]) delete panel.pressKey[key][input]; // consumed too (r6 P3)
      delete panel.lastPointer[key];
      return say("세션·권한·상태가 바뀌었습니다 · 화면을 다시 보세요");
    }
    delete panel.lastPointer[key];
    return fn(ev, input);
  };
  const send = (type, fields, okText) => m.session.request(type, fields).then((r) => {
    if (r && r.t === "reject") say(`거부 · ${rejectText(r.reason, r.detail)}`);
    else if (r && okText) say(okText);
    return r;
  });
  $("rp-register").onclick = guard(() => {
    const d = { epoch: gens().session_epoch };
    const pose = poseOf($("rp-x").value, $("rp-y").value, $("rp-yaw").value);
    const pallet = $("rp-pallet").value;
    if (!pallet || !pose) return say("팔레트와 자세(x, y, yaw)를 넣으세요");
    send("pallet_register", { pallet_id: pallet, pose }, "등록했습니다").then((r) => {
      if (r && r.t === "ack" && Number.isInteger(r.targets_revision)) {
        panel.registered = { session_epoch: d.epoch, revision: r.targets_revision, pallet, pose }; // what was sent
        const now = poseOf($("rp-x").value, $("rp-y").value, $("rp-yaw").value);
        if ($("rp-pallet").value !== pallet || !now || now.some((v, i) => Math.abs(v - pose[i]) > 1e-9)) {
          say("등록했습니다 · 그 사이 입력을 바꿨으니 지금 입력으로 다시 등록하세요"); // (r8 P3)
        }
        m.session.request("targets_request", {}); // the list with the registered pose
      }
    });
  }, "rp-register");
  $("rp-start").onclick = guard(() => {
    if (panel.toggle) return say("우회 변경이 적용되기를 기다립니다");
    send("operator_start", {}, "시작 확인을 보냈습니다");
  }, "rp-start");
  $("rp-submit").onclick = guard(() => {
    if (panel.toggle) return say("우회 변경이 적용되기를 기다립니다"); // from the toggle's send on (r2 P2-3)
    const rev = submitRevision(panel.registered, m.session.realStatus, m.session.targets);
    if (rev === null) return say("팔레트를 (다시) 등록하세요");
    // the inputs still name what was registered (r7 P2-1: an edited pose after the registration is not sent under it)
    const reg = panel.registered;
    const pose = poseOf($("rp-x").value, $("rp-y").value, $("rp-yaw").value);
    if ($("rp-pallet").value !== reg.pallet || !pose || pose.some((v, i) => Math.abs(v - reg.pose[i]) > 1e-9)) {
      return say("등록한 뒤 팔레트나 자세를 바꿨습니다 · 다시 등록하세요");
    }
    send("mission_submit", { pallet_id: $("rp-pallet").value, destination_id: $("rp-dest").value,
      targets_revision: rev }, "작업 지시를 보냈습니다").then((r) => {
      if (r && r.t === "reject" && r.reason === "stale_targets") {
        panel.registered = null; // register again with the new list (r1 P2-4)
        m.session.request("targets_request", {});
      }
    });
  }, "rp-submit");
  // the operator answer: the key of the request on screen when the press began (r1 P2-1)
  // a press record per button and per input (pointer id, or the keyboard) -- a click uses only its own input's
  // record (r3 P2-2, r4 P2-2: a second pointer, on either button, does not change the first one's)
  panel.pressKey = { "rp-confirm": {}, "rp-reject": {} };
  const press = (id, input) => { panel.pressKey[id][input] = panel.opKey ? { ...panel.opKey } : null; };
  for (const id of ["rp-confirm", "rp-reject"]) {
    $(id).onpointerdown = (ev) => press(id, inputOf(ev));
    $(id).onkeydown = (ev) => {
      if (ev.key !== "Enter") return; // Space is the teleop deadman key (r2 P3): Enter only
      if (ev.repeat) { ev.preventDefault(); return; } // a held key does not take a newer request (r2 P2-1)
      press(id, "kbd");
    };
  }
  const answer = (verdict, ev, input) => {
    const id = verdict === "confirm" ? "rp-confirm" : "rp-reject";
    const records = panel.pressKey[id];
    // its own input's record; none at all (a click with no press seen): the request drawn now
    const key = input in records ? records[input] : (Object.keys(records).length ? null : panel.opKey);
    delete records[input];
    if (!answerStillOpen(key, m.session.realStatus)) return say("확인 요청이 바뀌었습니다 · 화면을 다시 보세요");
    send("operator_answer", { kind: key.kind, serial: key.serial, verdict }, verdict === "confirm" ? "확인했습니다" : "거부했습니다");
  };
  $("rp-confirm").onclick = guard((ev, input) => answer("confirm", ev, input), "rp-confirm");
  $("rp-reject").onclick = guard((ev, input) => answer("reject", ev, input), "rp-reject");
  $("rp-recovery").onclick = guard(() => {
    const st = m.session.realStatus;
    const drawn = panel.recoveryId;
    // the newest number, the same as drawn, under the session it was drawn for (r1 P2-2)
    if (!Number.isInteger(drawn) || !st || st.field_recovery !== drawn) {
      return say("현장 복구 번호가 바뀌었습니다 · 다시 보세요");
    }
    send("recovery", { recovery_id: drawn, action: "done" }, "현장 복구 완료를 보냈습니다");
  }, "rp-recovery");
  $("rp-rearm").onclick = guard(() => send("rearm", {}, "재무장을 보냈습니다"), "rp-rearm");
  $("rp-bypass").onclick = (ev) => {
    const btn = ev.target.closest("button[data-check]");
    if (!btn || btn.disabled) return;
    const bkey = `check:${btn.dataset.check}`;
    const ok = clickOk(bkey, ev);
    delete panel.lastPointer[bkey];
    if (!ok) return say("세션·권한·상태가 바뀌었습니다 · 화면을 다시 보세요");
    if (panel.toggle) return say("앞선 우회 변경이 적용되기를 기다립니다"); // one at a time (r2 P2-3)
    // tracked from the send: the submit and the start check are closed at once; the 2 s run from the ack
    const toggle = { check: btn.dataset.check, on: btn.dataset.bypassed !== "true", at: null,
      epoch: gens().session_epoch, lease: gens().lease_id }; // bound to the session and lease (r3 P3)
    panel.toggle = toggle;
    say("우회 변경을 보냈습니다");
    m.session.request("check_bypass", { check: toggle.check, on: toggle.on }).then((r) => {
      // dropped, or another session or lease by now: its answer tells nothing (r5-r6 P3)
      if (panel.toggle !== toggle || toggle.epoch !== gens().session_epoch || toggle.lease !== gens().lease_id) {
        if (panel.toggle === toggle) {
          panel.toggle = null;
          say(""); // its note goes with it (r7 P3)
        }
        return;
      }
      if (r && r.t === "ack") {
        toggle.at = performance.now();
        say("다음 틱에 적용합니다");
      } else {
        panel.toggle = null; // refused or lost: nothing to wait for
        if (r && r.t === "reject") say(`거부 · ${rejectText(r.reason, r.detail)}`);
      }
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
  // only a status received while this lease was held counts (r2 P2-2; protocol.js records it at receipt)
  const mine = statusIsMine(st, gens, clientId) && m.session.realStatusLease === gens.lease_id;
  panel.mine = mine;
  panel.drawn = { epoch: gens.session_epoch, lease: gens.lease_id, sig: statusSig(st) };
  if (panel.registered && panel.registered.session_epoch !== gens.session_epoch) panel.registered = null;
  // the target list once per session when none came with the welcome (r1 P2-5)
  if (!m.session.targets && panel.targetsAsked !== gens.session_epoch && gens.session_epoch !== null) {
    const epoch = gens.session_epoch;
    panel.targetsAsked = epoch; // one in flight; a refusal clears it to ask again (r2 P2-5)
    m.session.request("targets_request", {}).then((r) => {
      if (r && r.t === "reject") {
        if (panel.targetsAsked === epoch) panel.targetsAsked = null;
        panel.note = `대상 목록을 받지 못했습니다 · ${r.reason} · 다시 요청합니다`;
        $("rp-msg").textContent = panel.note;
      }
    });
  }
  // a toggle shown, or not by its deadline (then told; r1 P2-3)
  let t = panel.toggle;
  if (t && (t.epoch !== gens.session_epoch || t.lease !== gens.lease_id)) {
    panel.toggle = t = null; // another session or lease: that toggle tells nothing of this one (r3 P3)
    panel.note = ""; // nor its waiting note (r4 P3)
    $("rp-msg").textContent = "";
  }
  if (t && t.at !== null && (toggleShown(t, st) || performance.now() - t.at > TOGGLE_WAIT_MS)) {
    panel.note = toggleShown(t, st) ? "우회 변경이 적용됐습니다" // (r2 P3)
      : "우회 변경이 적용되지 않았습니다 · 실차 상태의 우회 목록을 보세요";
    $("rp-msg").textContent = panel.note;
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
