// The bench console's screen on the page (docs/plans/2026-11-27-bench-console.md 4, 4.1). The decisions are
// bench.js's; this file makes the '벤치' menu entry and its page only while the welcome is a bench session, removes
// both (and their handlers) on any other welcome or a disconnect, draws, and sends through the controller. The big
// STOP is the shared one (pressStop): the bench screen adds only its own requests.

import { AXES, BYPASS, DEFAULT_SOURCE, advice, bypassChips, clearView, fieldState, largePhrase,
  resultView } from "./bench.js";
import { BENCH_TEXT as T, benchDetailText } from "./profile.js";

const VIEW = "bench";
let mounted = null; // {rail, page, abort}

const el = (tag, attrs = {}, ...kids) => {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === "class") e.className = v;
    else if (k === "text") e.textContent = v;
    else if (v !== false && v !== null && v !== undefined) e.setAttribute(k, v === true ? "" : String(v));
  }
  for (const kid of kids) if (kid) e.append(kid);
  return e;
};

function buildPage() {
  const axisSel = el("select", { id: "bn-axis", "aria-label": "축" },
    ...AXES.map((a) => el("option", { value: a, text: T.axis[a] })));
  const sourceSel = el("select", { id: "bn-source", "aria-label": "원천" });
  const modeSel = el("select", { id: "bn-mode", "aria-label": "모드" },
    el("option", { value: "duty", text: T.mode.duty }),
    el("option", { value: "hold", text: `${T.mode.hold} · ${T.waitB2}`, disabled: true }));
  const num = (id, label, step) => el("label", { class: "bn-field" }, el("span", { text: label }),
    el("input", { id, type: "number", step, "aria-label": label }));
  const check = (id, label) => el("label", { class: "bn-check" }, el("input", { id, type: "checkbox" }),
    el("span", { text: label }));
  const bypassRow = el("div", { class: "bn-row", id: "bn-bypass", "aria-label": T.bypassSession },
    el("span", { class: "hint", text: T.bypassSession }),
    ...Object.keys(BYPASS).map((n) => check(`bn-bp-${n}`, T.bypass[n])),
    el("label", { class: "bn-check", title: T.waitA2 }, el("input", { type: "checkbox", disabled: true }),
      el("span", { text: `${T.bypass.kth_invalid} · ${T.waitA2}` })));
  const setup = el("select", { id: "bn-setup", "aria-label": T.physicalSetup[""] },
    ...Object.entries(T.physicalSetup).map(([v, t]) => el("option", { value: v, text: t })));
  return el("section", { class: "page bench", id: "v-bench", "data-view-panel": VIEW, hidden: true },
    el("header", {}, el("h2", { text: T.title }), el("span", { class: "hint", text: T.hint })),
    el("div", { class: "bn-head" },
      el("span", { class: "mono", id: "bn-product", text: "—" }),
      el("span", { class: "chip", id: "bn-ready", text: T.notReady }),
      el("span", { class: "chip", id: "bn-mode-chip", text: "—" }),
      el("span", { class: "mono", id: "bn-driver", title: "드라이버 상태 네 바이트", text: "—" }),
      el("span", { class: "chip warn", id: "bn-chips", hidden: true })),
    el("div", { class: "bn-grid" },
      el("div", { class: "bn-col" },
        el("div", { class: "bn-row" }, axisSel, sourceSel, modeSel),
        el("div", { class: "bn-row" }, num("bn-duty", "듀티", "0.01"), num("bn-seconds", "시간 s", "0.1"),
          num("bn-duty-max", "듀티 상한(선택)", "0.01"), num("bn-time-max", "시간 상한 s(선택)", "0.1")),
        el("div", { class: "bn-row", id: "bn-window-row" }, num("bn-win-lo", "창 아래", "0.01"),
          num("bn-win-hi", "창 위", "0.01"), el("span", { class: "hint", id: "bn-win-unit" })),
        el("div", { class: "bn-row hint" }, el("span", { text: `정지: N 회전 / N 카운트 · ${T.waitB2}` })),
        bypassRow,
        el("div", { class: "bn-advice warn", id: "bn-advice", hidden: true }),
        el("div", { class: "bn-row" }, check("bn-power", T.confirm.power_switch_in_hand),
          check("bn-lifted", T.confirm.wheels_lifted), check("bn-vcc", T.confirm.vcc_watched)),
        el("div", { class: "bn-row" }, el("input", { id: "bn-large", type: "text", placeholder: T.largeConfirm,
          "aria-label": T.largeConfirm }), setup),
        el("div", { class: "hint", id: "bn-large-hint", hidden: true }),
        el("div", { class: "bn-row" },
          el("button", { class: "btn primary", id: "bn-start", type: "button", text: T.start }),
          el("button", { class: "stop bn-stop", id: "bn-stop", type: "button" }, el("b", { text: T.stop })),
          el("button", { class: "btn", id: "bn-disarm", type: "button", text: T.disarm })),
        el("div", { class: "bn-row" },
          el("input", { id: "bn-cause", type: "text", placeholder: T.cause, "aria-label": T.cause }),
          el("button", { class: "btn", id: "bn-clear", type: "button", text: T.clear })),
        el("div", { class: "hint", text: T.clearAdvice }),
        el("div", { class: "hint", id: "bn-clear-result", role: "status" }),
        el("div", { class: "hint", id: "bn-msg", role: "status" })),
      el("div", { class: "bn-col" },
        el("div", { class: "hint", text: T.graph }),
        el("canvas", { id: "bn-graph", width: 640, height: 300, "aria-label": "벤치 그래프" }),
        el("div", { class: "bn-result", id: "bn-result" }, el("span", { class: "hint", text: T.noResult })))));
}

function buildRail() {
  // an icon as the other entries have: the narrow layout hides their text (app.css, max-width 1180px)
  const ns = "http://www.w3.org/2000/svg";
  const svg = document.createElementNS(ns, "svg");
  svg.setAttribute("viewBox", "0 0 24 24");
  svg.setAttribute("aria-hidden", "true");
  for (const d of ["M4 18h16", "M7 18V9l5-4 5 4v9", "M12 11v4"]) {
    const p = document.createElementNS(ns, "path");
    p.setAttribute("d", d);
    svg.append(p);
  }
  const b = el("button", { class: "rail-item", type: "button", "data-view": VIEW, id: "bn-rail", title: T.menu }, svg);
  b.append(T.menu);
  return b;
}

function form() {
  const v = (id) => document.getElementById(id)?.value ?? "";
  const c = (id) => document.getElementById(id)?.checked === true;
  return {
    axis: v("bn-axis"), source: v("bn-source"), duty: v("bn-duty"), seconds: v("bn-seconds"),
    bench_duty_max: v("bn-duty-max"), bench_time_max: v("bn-time-max"), window_lo: v("bn-win-lo"),
    window_hi: v("bn-win-hi"), bypass: Object.keys(BYPASS).filter((n) => c(`bn-bp-${n}`)),
    power_switch_in_hand: c("bn-power"), wheels_lifted: c("bn-lifted"), vcc_watched: c("bn-vcc"),
    large_confirm: v("bn-large"), physical_setup: v("bn-setup"),
  };
}

// Make or remove the bench menu and page so they exist exactly while the controller is visible.
export function syncBench(m, { showView, pressStop, rejectText }) {
  const want = Boolean(m.bench?.visible);
  if (want && !mounted) {
    const abort = new AbortController();
    const rail = buildRail();
    const page = buildPage();
    document.querySelector(".rail")?.insertBefore(rail, document.querySelector(".rail-foot"));
    // the pages sit in the body grid's first column, before the side panel (as index.html lays out its own pages)
    const body = document.querySelector(".body");
    body?.insertBefore(page, body.querySelector(":scope > aside.detail"));
    const opt = { signal: abort.signal };
    rail.addEventListener("click", () => showView(VIEW), opt);
    const say = (text) => {
      const e = document.getElementById("bn-msg");
      if (e) e.textContent = text;
    };
    page.addEventListener("click", (ev) => {
      const id = ev.target.closest("button")?.id;
      if (id === "bn-start") {
        const r = m.bench.run(form());
        say(r.problems.length ? `입력 확인: ${r.problems.join(", ")}` : r.sent ? "보냄" : "보낼 수 없음");
        // a large run's retyped confirmation is used once (round 2 P1-1): the next run retypes it
        if (r.sent) document.getElementById("bn-large").value = "";
      } else if (id === "bn-stop") {
        pressStop("bench"); // the shared STOP: the same request the top STOP and Escape send
      } else if (id === "bn-disarm") {
        say(m.bench.disarm() ? "보냄" : "보낼 수 없음");
      } else if (id === "bn-clear") {
        const r = m.bench.clearFaults(document.getElementById("bn-cause")?.value);
        say(r.problems.includes("busy") ? "진행 중인 고장 해제가 있습니다"
          : r.problems.length ? "원인을 적으세요" : r.sent ? "보냄" : "보낼 수 없음");
      }
    }, opt);
    page.addEventListener("change", (ev) => {
      const id = ev.target?.id;
      if (["bn-axis", "bn-duty", "bn-seconds"].includes(id)) {
        document.getElementById("bn-large").value = ""; // the confirmation was for another run (round 2 P1-1)
      }
      if (id === "bn-axis") {
        // the window belongs to its axis (rad for steering, m for lift: round 2 P1-2), the source follows the axis
        document.getElementById("bn-win-lo").value = "";
        document.getElementById("bn-win-hi").value = "";
        const srcSel = document.getElementById("bn-source");
        srcSel.dataset.list = "";
        srcSel.dataset.axis = "";
      }
      renderBench(m, { rejectText });
    }, opt);
    page.addEventListener("input", (ev) => {
      if (["bn-duty", "bn-seconds"].includes(ev.target?.id)) document.getElementById("bn-large").value = "";
    }, opt);
    mounted = { rail, page, abort, replies: 0 };
    renderBench(m, { rejectText });
  } else if (!want && mounted) {
    const wasShown = !mounted.page.hidden;
    mounted.abort.abort(); // every handler of the bench page goes with it
    mounted.rail.remove();
    mounted.page.remove();
    mounted = null;
    if (wasShown) showView("monitor");
  }
}

function setText(id, text) {
  const e = document.getElementById(id);
  if (e && e.textContent !== text) e.textContent = text;
}

export function renderBench(m, { rejectText }) {
  if (!mounted || !m.bench?.visible) return;
  const st = m.bench.state;
  const f = form();
  const fs = fieldState(f);
  // the source list follows the axis
  const srcSel = document.getElementById("bn-source");
  const want = fs.sources.join(",");
  if (srcSel.dataset.list !== want || srcSel.dataset.axis !== f.axis) {
    srcSel.replaceChildren(...fs.sources.map((s) => el("option", { value: s, text: T.source[s] })),
      el("option", { value: "mt", text: `${T.source.mt} · ${T.waitA1c}`, disabled: true }));
    srcSel.dataset.list = want;
    srcSel.dataset.axis = f.axis;
    srcSel.value = DEFAULT_SOURCE[f.axis] ?? fs.source; // a new axis starts on its own default (P3)
  }
  setText("bn-win-unit", f.axis === "M" ? "rad(조향)" : f.axis === "H" ? "m(승강)" : "");
  for (const [n, open] of Object.entries(fs.bypassOpen)) {
    const box = document.getElementById(`bn-bp-${n}`);
    if (box) {
      box.disabled = !open;
      if (!open) box.checked = false;
    }
  }
  document.getElementById("bn-window-row").hidden = !fs.windowOpen;
  const adv = advice(f);
  const advEl = document.getElementById("bn-advice");
  advEl.hidden = adv.length === 0;
  advEl.textContent = adv.map((a) => T.advice[a]).join(" · ");
  // the head line
  setText("bn-product", st?.product ?? m.session.welcome?.product ?? "—");
  setText("bn-ready", st?.ready ? T.ready : `${T.notReady}${st?.not_ready ? ` · ${st.not_ready}` : ""}`);
  document.getElementById("bn-ready").className = `chip ${st?.ready ? "ok" : "warn"}`;
  const run = st?.run;
  setText("bn-mode-chip", `${st?.board?.mode ?? "—"}${run ? ` · ${T.phase[run.phase] ?? run.phase}` : ""}`);
  setText("bn-driver", st?.board ? `drv ${st.board.driver_status.join(" ")}` : "drv —");
  const chips = bypassChips(st);
  const chipEl = document.getElementById("bn-chips");
  chipEl.hidden = chips.mcu.length === 0 && chips.host.length === 0;
  chipEl.textContent = `우회 중: ${T.chipsMcu} ${chips.mcu.map((n) => T.bypass[n] ?? n).join(", ") || "—"} · `
    + `${T.chipsHost} ${chips.host.join(", ") || "—"}`;
  // the buttons: the start needs the lease and a ready board; DISARM and the clear need the lease
  const lease = m.session.hasLease;
  document.getElementById("bn-start").disabled = !(lease && st?.ready && !run && !m.bench.runPending);
  document.getElementById("bn-disarm").disabled = !lease;
  const latched = ["FAULT", "ESTOP"].includes(st?.board?.mode);
  // only a latched board is cleared, one attempt at a time (its own result closes it: round 3 P2-2)
  document.getElementById("bn-clear").disabled = !(lease && latched && !run && !m.bench.clearPending && !st?.clear);
  // the last reply, with the large run's exact phrase when the server named it
  const replies = m.bench.replies;
  const last = replies.at(-1);
  const phrase = last ? largePhrase(last.detail) : null;
  const hint = document.getElementById("bn-large-hint");
  hint.hidden = !phrase;
  if (phrase) hint.textContent = `그대로 입력: ${phrase}`;
  if (last && m.bench.replySeq !== mounted.replies) {
    // only a new answer is written over the line (a local input message stays until then; counted, not the list's
    // length, which stops growing at 20: round 3 P2-3)
    mounted.replies = m.bench.replySeq;
    const text = last.t === "ack" ? T.replyOk
      : `${T.replyRefused} · ${rejectText(last.reason)}${last.detail ? ` (${benchDetailText(last.detail)})` : ""}`;
    setText("bn-msg", `${last.kind}: ${text}`);
  }
  const cv = clearView(m.bench.clear);
  setText("bn-clear-result", cv ? `고장 해제: ${cv.ok ? "해제됨" : cv.result}${cv.why ? ` · ${cv.why}` : ""} · 전 ${cv.before.mode} `
    + `(${cv.before.faults}, drv ${cv.before.driver}) → 후 ${cv.after.mode ?? "—"} (${cv.after.faults}, drv ${cv.after.driver})` : "");
  renderResult(m);
  drawGraph(m, run?.axis ?? f.axis); // the axis being driven while a run goes, the picked one otherwise (P2-4)
}

function renderResult(m) {
  const own = m.bench.result;
  const v = resultView(own ?? m.bench.previous);
  const box = document.getElementById("bn-result");
  const key = JSON.stringify([Boolean(own), v]); // whose result it is belongs to the key (P2-2)
  if (box.dataset.key === key) return;
  box.dataset.key = key;
  if (!v) {
    box.replaceChildren(el("span", { class: "hint", text: T.noResult }));
    return;
  }
  const row = (k, val) => el("div", { class: "bn-kv" }, el("span", { class: "hint", text: k }), el("span", { text: val }));
  box.replaceChildren(
    el("h3", { text: own ? T.result : `${T.result} · 직전 실행(이 탭이 시작한 실행이 아닐 수 있음)` }),
    row(T.verdictRun, `${v.ok ? T.ok : T.notOk}${v.why.length ? ` · ${v.why.join(", ")}` : ""}`),
    row(T.verdictHold, T.verdictLater),
    row(T.verdictDistance, T.verdictLater),
    row("닫힘", v.closed ?? "—"),
    row("정지 원인", v.stopReason ?? "—"),
    row("중단", v.aborted ?? "—"),
    row("축", v.axis ?? "—"),
    row("최고 전류 mA", v.peakCurrentMa ? v.peakCurrentMa.join(" / ") : "—"),
    row("기록", v.record ?? "—"));
}

function drawGraph(m, axis) {
  const cv = document.getElementById("bn-graph");
  const ctx = cv?.getContext?.("2d");
  if (!ctx) return;
  const cs = getComputedStyle(document.documentElement);
  const col = (k, d) => cs.getPropertyValue(`--${k}`).trim() || d;
  const W = cv.width, H = cv.height, half = H / 2;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = col("panel", "#111");
  ctx.fillRect(0, 0, W, H);
  m.bench.graph.expire(performance.now()); // the window by the clock, not only by arriving samples (P2-4)
  const ser = m.bench.graph.series(axis);
  if (ser.length < 2) return;
  const t0 = ser[0].t, t1 = ser.at(-1).t;
  const x = (t) => ((t - t0) / Math.max(1, t1 - t0)) * (W - 10) + 5;
  const line = (pts, y0, h, lo, hi, color) => {
    ctx.strokeStyle = color;
    ctx.beginPath();
    let started = false;
    for (const [t, v] of pts) {
      if (v === null || !Number.isFinite(v)) { started = false; continue; }
      const y = y0 + h - ((v - lo) / Math.max(1e-9, hi - lo)) * h;
      if (started) ctx.lineTo(x(t), y); else { ctx.moveTo(x(t), y); started = true; }
    }
    ctx.stroke();
  };
  const vals = ser.map((s) => s.value).filter((v) => v !== null && Number.isFinite(v));
  const lo = vals.length ? Math.min(...vals) : 0, hi = vals.length ? Math.max(...vals) : 1;
  line(ser.map((s) => [s.t, s.value]), 4, half - 8, lo, hi === lo ? lo + 1 : hi, col("accent", "#4af"));
  line(ser.map((s) => [s.t, s.duty]), 4, half - 8, -1, 1, col("warn", "#fa4"));
  const cur = ser.flatMap((s) => s.current).filter((c) => c !== null);
  const cmax = Math.max(1, ...cur.map((c) => Math.abs(c)));
  ["ok", "lidar", "teleop", "crit"].forEach((k, i) =>
    line(ser.map((s) => [s.t, s.current[i] === null ? null : Math.abs(s.current[i])]), half + 4, half - 8, 0, cmax,
      col(k, "#8f8"))); // a board gap stays a gap, never a measured 0 mA (round 4 P2-3)
  ctx.fillStyle = col("muted", "#999");
  ctx.font = "11px sans-serif";
  const what = axis === "M" ? "조향 rad" : axis === "H" ? "승강 m" : "바퀴 카운트 변화(측정 원천 아님)";
  ctx.fillText(`${what} ${lo.toFixed(3)}…${hi.toFixed(3)} · 듀티 ±1`, 8, 14);
  ctx.fillText(`전류 L R M H · 최대 ${cmax} mA`, 8, half + 16);
}

export function benchMounted() {
  return mounted !== null;
}
