// The bench console's screen model (docs/plans/2026-11-27-bench-console.md 4, 4.1). Pure module: no DOM, no sockets.
// The back end (tools/bench_console.py, B1) judges every request; this page shapes what the operator typed, keeps the
// bench requests it sent (dropped on a disconnect, never resent on the next connection) and the last 10 s for the
// graph. Hold and the distance stop come with B2 (A1b/A1c): their fields stay disabled here.

export const AXES = ["L", "R", "M", "H"];
export const DEFAULT_SOURCE = { L: "none", R: "none", M: "kth", H: "kth" };
// the MCU bypass bits this image has (A1a); kth_invalid comes with A2
export const BYPASS = { steer_range: "M", lift_range: "H" };
export const GRAPH_WINDOW_MS = 10000;
const BENCH_REQUESTS = new Set(["bench_run", "bench_stop", "bench_disarm", "clear_faults"]);

export function isBenchWelcome(welcome) {
  return Boolean(welcome && welcome.session === "bench");
}

// Which inputs are open for this form (plan 4: fields that do not fit the source are disabled).
export function fieldState(form) {
  const axis = AXES.includes(form.axis) ? form.axis : "L";
  const sources = axis === "M" || axis === "H" ? ["kth", "none"] : ["none"];
  const rangeName = axis === "M" ? "steer_range" : axis === "H" ? "lift_range" : null;
  const rangeOff = rangeName !== null && (form.bypass ?? []).includes(rangeName);
  return {
    sources,
    source: sources.includes(form.source) ? form.source : DEFAULT_SOURCE[axis],
    holdOpen: false, // B2 (A1b)
    stopOpen: false, // B2 (A1b/A1c)
    mtOpen: false, // A1c
    kthInvalidOpen: false, // A2
    bypassOpen: Object.fromEntries(Object.entries(BYPASS).map(([name, ax]) => [name, ax === axis])),
    windowOpen: (axis === "M" || axis === "H") && !rangeOff,
    rangeOff,
    wheel: axis === "L" || axis === "R",
  };
}

function num(v) {
  if (v === "" || v === null || v === undefined) return null;
  const x = typeof v === "number" ? v : Number(v);
  return Number.isFinite(x) ? x : NaN;
}

// The bench_run fields from the form; problems are the page's own shape checks (the server judges the limits).
export function runFields(form) {
  const fs = fieldState(form);
  const problems = [];
  const axis = AXES.includes(form.axis) ? form.axis : null;
  if (!axis) problems.push("axis");
  const duty = num(form.duty);
  const seconds = num(form.seconds);
  if (duty === null || Number.isNaN(duty)) problems.push("duty");
  if (seconds === null || Number.isNaN(seconds)) problems.push("seconds");
  const fields = { axis, source: fs.source, mode: "duty", duty, seconds,
    bypass: (form.bypass ?? []).filter((n) => fs.bypassOpen[n]),
    power_switch_in_hand: form.power_switch_in_hand === true,
    wheels_lifted: form.wheels_lifted === true,
    vcc_watched: form.vcc_watched === true };
  for (const k of ["bench_duty_max", "bench_time_max"]) {
    const v = num(form[k]);
    if (Number.isNaN(v)) problems.push(k);
    else if (v !== null) fields[k] = v;
  }
  if (fs.windowOpen) {
    const lo = num(form.window_lo);
    const hi = num(form.window_hi);
    if (lo === null || hi === null || Number.isNaN(lo) || Number.isNaN(hi)) problems.push("window");
    else fields.window = [lo, hi];
  }
  if (typeof form.large_confirm === "string" && form.large_confirm.trim()) fields.large_confirm = form.large_confirm.trim();
  if (typeof form.physical_setup === "string" && form.physical_setup) fields.physical_setup = form.physical_setup;
  return { fields, problems };
}

// The exact phrase a large run must be retyped with, when the server's refusal names it.
export function largePhrase(detail) {
  const m = /large_confirm must be exactly '([^']+)'/.exec(String(detail ?? ""));
  return m ? m[1] : null;
}

// Plan 0.2: shown and recorded, never enforced.
export function advice(form) {
  const out = [];
  if (fieldState(form).rangeOff) out.push("rangeOff");
  return out;
}

// The bypass chips: what the MCU confirmed (its BENCH_STATE) apart from what the host applies.
export function bypassChips(state) {
  const run = state?.run ?? null;
  const mcu = state?.bench?.mcu_bypass ?? [];
  return { mcu: [...mcu], host: [...(run?.bypass_host ?? [])], asked: [...(run?.bypass_asked ?? [])] };
}

// The result panel's rows from a bench_result (verdict 1 in B1; 2 and 3 with B2).
export function resultView(result) {
  if (!result) return null;
  const v = result.verdict?.run_cleanup ?? { ok: false, why: ["no_verdict"] };
  return {
    ok: Boolean(v.ok),
    why: [...(v.why ?? [])],
    closed: result.closed ?? null,
    stopReason: result.stop_reason ?? null,
    aborted: result.aborted ?? null,
    peakCurrentMa: Array.isArray(result.peak_current_ma) ? [...result.peak_current_ma] : null,
    record: result.record ?? null,
    axis: result.axis ?? null,
    holdVerdict: null, // B2
    distanceVerdict: null, // B2
  };
}

// The firmware's 32-bit modular count step (forklift_core.mcu_link.messages.wheel_count_delta).
export function wheelDelta(newer, older) {
  const d = (newer - older + 2 ** 31) % 2 ** 32;
  return (d < 0 ? d + 2 ** 32 : d) - 2 ** 31;
}

// A clear_result's rows (6.5: what to check before VM goes back on).
export function clearView(c) {
  if (!c) return null;
  const faults = (v) => (v?.fault_flags?.length ? v.fault_flags.join(", ") : "없음");
  return { result: c.result ?? null, why: c.why ?? null, cause: c.cause ?? null,
    before: { mode: c.before?.mode ?? null, faults: faults(c.before), driver: (c.before?.driver_status ?? []).join(" ") },
    after: { mode: c.after?.mode ?? null, faults: faults(c.after), driver: (c.after?.driver_status ?? []).join(" ") },
    ok: c.result === "cleared" };
}

// The last GRAPH_WINDOW_MS of bench_state samples (the screen stream; verdicts come from the board's own STATEs).
export class Graph {
  constructor(windowMs = GRAPH_WINDOW_MS) {
    this.windowMs = windowMs;
    this.samples = [];
  }

  push(atMs, state) {
    const b = state?.board;
    if (!b) return;
    this.samples.push({ t: atMs, boot: b.boot_id ?? null, duty: [...(b.duty_e4 ?? [0, 0, 0, 0])],
      current: [...(b.current_ma ?? [0, 0, 0, 0])], steer: b.steering_rad ?? null, lift: b.lift_m ?? null,
      wheel: [...(b.wheel_count ?? [0, 0])] });
    const cut = atMs - this.windowMs;
    while (this.samples.length && this.samples[0].t < cut) this.samples.shift();
  }

  clear() {
    this.samples = [];
  }

  // the measurement for an axis: steering rad (M), lift m (H), or the wheel count since the window's first sample
  series(axis) {
    const k = axis === "R" ? 1 : 0;
    // the count since the window's first sample of the same boot, by the firmware's 32-bit modular step
    // (messages.wheel_count_delta); a boot change breaks the line (null) and the count starts again from there
    let prev = null;
    let boot;
    let total = 0;
    const counts = this.samples.map((s) => {
      const c = s.wheel[k];
      if (prev === null || s.boot !== boot) {
        const first = prev === null;
        prev = c;
        boot = s.boot;
        total = 0;
        return first ? 0 : null;
      }
      total += wheelDelta(c, prev);
      prev = c;
      return total;
    });
    return this.samples.map((s, i) => ({
      t: s.t,
      value: axis === "M" ? s.steer : axis === "H" ? s.lift : counts[i],
      duty: s.duty[AXES.indexOf(axis)] / 1e4,
      current: s.current,
    }));
  }
}

// The bench session on one console connection: what is shown, the bench requests in flight, the latest stream.
export function createBenchController({ session, now = () => Date.now() }) {
  let visible = false;
  let state = null;
  let result = null; // the bench_result of a run this tab started (matched by its request id)
  let previous = null; // the board's last result the stream carries (a run before this tab's, never attached to one)
  let clear = null;
  let runId = null; // the request id of this tab's latest bench_run
  let clearId = null; // the request id of this tab's latest clear_faults
  let stateAt = null; // when the latest bench_state of this connection came (the health row's stream age)
  const pending = new Map(); // request_id -> kind
  const replies = []; // {kind, t, reason, detail, at}
  const graph = new Graph();

  function track(kind, promise, requestId) {
    pending.set(requestId, kind);
    promise.then((r) => {
      if (!pending.has(requestId)) return; // forgotten at a disconnect: its answer is not this connection's
      pending.delete(requestId);
      // accepted: its own result is the next one -- unless that result already came (a late ACK of a resend)
      if (kind === "bench_run" && r?.t === "ack" && result?.request_id !== requestId) result = null;
      replies.push({ kind, t: r?.t ?? "none", reason: r?.reason ?? null, detail: r?.detail ?? null, at: now() });
      while (replies.length > 20) replies.shift();
    });
  }

  function send(kind, fields) {
    if (!visible || !BENCH_REQUESTS.has(kind)) return null;
    const id = `bench-${kind}-${Math.random().toString(16).slice(2)}-${now()}`;
    if (kind === "bench_run") runId = id;
    if (kind === "clear_faults") clearId = id;
    track(kind, session.request(kind, fields, { requestId: id }), id);
    return id;
  }

  return {
    onWelcome(msg) {
      const was = visible;
      visible = isBenchWelcome(msg);
      if (!visible || !was) {
        state = null;
        result = null;
        previous = null;
        clear = null;
        runId = null;
        clearId = null;
        stateAt = null;
        replies.length = 0; // another connection's answers are not this screen's
        graph.clear();
      }
    },
    // the connection closed: the bench requests in flight are dropped, never resent on the next one (plan 4 (a''))
    onDisconnect() {
      for (const id of pending.keys()) session.forget(id);
      pending.clear();
      visible = false;
      state = null;
      stateAt = null;
      replies.length = 0;
      graph.clear();
    },
    onMessage(msg) {
      if (!visible || !msg) return;
      if (msg.t === "bench_state" && msg.session_epoch === session.epoch) {
        state = msg;
        stateAt = now();
        graph.push(now(), msg);
        if (msg.last_result) previous = msg.last_result;
      } else if (msg.t === "bench_result") {
        if (msg.request_id === runId) result = msg; // the run this tab asked for: ACK is acceptance, this is its end
        previous = msg;
      } else if (msg.t === "clear_result") {
        if (msg.request_id === clearId) clear = msg; // this tab's clear: its end (ACK was only acceptance)
      }
    },
    run(form) {
      // one start at a time: not while one is on its way or a run is going (round 2 P2-2)
      if ([...pending.values()].includes("bench_run") || state?.run) return { sent: null, problems: ["busy"] };
      const { fields, problems } = runFields(form);
      if (problems.length) return { sent: null, problems };
      return { sent: send("bench_run", fields), problems };
    },
    benchStop: () => send("bench_stop", {}),
    disarm: () => send("bench_disarm", {}),
    clearFaults(cause) {
      const c = String(cause ?? "").trim();
      if (!c) return { sent: null, problems: ["cause"] };
      return { sent: send("clear_faults", { cause: c }), problems: [] };
    },
    get visible() {
      return visible;
    },
    get state() {
      return state;
    },
    get result() {
      return result;
    },
    get previous() {
      return previous;
    },
    get stateAt() {
      return stateAt;
    },
    get runPending() {
      return [...pending.values()].includes("bench_run");
    },
    get clear() {
      return clear;
    },
    get pending() {
      return [...pending.entries()];
    },
    get replies() {
      return [...replies];
    },
    graph,
  };
}
