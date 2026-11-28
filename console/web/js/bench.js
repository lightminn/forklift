// The bench console's screen model (docs/plans/2026-11-27-bench-console.md 4, 4.1, 4.4 B2). Pure module: no DOM, no
// sockets. The back end (tools/bench_console.py) judges every request; this page shapes what the operator typed, keeps
// the bench requests it sent (dropped on a disconnect, never resent on the next connection) and the last 10 s for the
// graph. B2: a hold (position kept by the KTH count), a stop after N turns, a measuring run -- steering and lift on
// the KTH source only; the MT source (A1c) and the second bypass bits (A2) stay closed.

export const AXES = ["L", "R", "M", "H"];
export const DEFAULT_SOURCE = { L: "none", R: "none", M: "kth", H: "kth" };
// the MCU bypass bits this image has (A1a); kth_invalid comes with A2
export const BYPASS = { steer_range: "M", lift_range: "H" };
export const GRAPH_WINDOW_MS = 10000;
export const HOLD_TARGETS = ["zero", "here", "abs", "rel"];
export const HIGH_DUTY_DISTANCE = 0.4; // a distance stop above this duty is shown as unverified, never refused
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
  const source = sources.includes(form.source) ? form.source : DEFAULT_SOURCE[axis];
  const kth = source === "kth";
  const mode = kth && form.mode === "hold" ? "hold" : "duty";
  return {
    sources,
    source,
    mode,
    holdOpen: kth, // a hold needs the KTH count (steering, lift)
    stopOpen: kth && mode === "duty", // a stop after N turns: a duty run on the KTH count
    measureOpen: kth && mode === "duty" && !(num(form.stop_turns) > 0 || num(form.stop_turns) < 0),
    signNeeded: kth && (mode === "hold" || num(form.stop_turns) !== null || form.measure === true),
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

// The run's kind as the back end reads it (plan 4.4 decision 2): a hold, a stop after N turns, a measuring run, or a
// plain duty run (a B1-style request with the KTH source stays a plain duty run).
export function runKind(form) {
  const fs = fieldState(form);
  if (fs.mode === "hold") return "hold";
  if (fs.stopOpen && num(form.stop_turns) !== null) return "distance";
  if (fs.measureOpen && form.measure === true) return "measure";
  return "duty";
}

// What a change of axis, source or mode leaves behind (plan 4.4 decision 6): the hold, the stop, the sign and the
// measuring box belong to the run they were typed for -- an axis or a source change clears them, so a sign typed for M
// never goes out with H. Returns the form keys to empty.
export function clearedOnChange(changed) {
  if (changed === "axis" || changed === "source") {
    return ["hold_target", "hold_value", "hold_kp", "hold_ki", "hold_kd", "hold_deadband", "hold_min_duty",
      "count_sign", "stop_turns", "measure", "same_setup"];
  }
  if (changed === "mode") return ["stop_turns", "measure"];
  return [];
}

// The bench_run fields from the form; problems are the page's own shape checks (the server judges the limits).
export function runFields(form) {
  const fs = fieldState(form);
  const problems = [];
  const axis = AXES.includes(form.axis) ? form.axis : null;
  if (!axis) problems.push("axis");
  const kind = runKind(form);
  const duty = num(form.duty);
  const seconds = num(form.seconds);
  // a hold's duty is its cap and may be left out (the back end then takes 0.15, as the CLI does)
  if ((duty === null && kind !== "hold") || Number.isNaN(duty)) problems.push("duty");
  if (seconds === null || Number.isNaN(seconds)) problems.push("seconds");
  const fields = { axis, source: fs.source, mode: kind === "hold" ? "hold" : "duty", seconds,
    bypass: (form.bypass ?? []).filter((n) => fs.bypassOpen[n]),
    power_switch_in_hand: form.power_switch_in_hand === true,
    wheels_lifted: form.wheels_lifted === true,
    vcc_watched: form.vcc_watched === true };
  if (duty !== null) fields.duty = duty;
  if (kind !== "duty") {
    // the sensor's sign is the operator's, never a default (plan 4.4, independent review P1-2)
    const cs = num(form.count_sign);
    if (cs !== 1 && cs !== -1) problems.push("count_sign");
    else fields.count_sign = cs;
    if (form.same_setup === true) fields.same_setup_as_bundle = true;
  }
  if (kind === "hold") {
    const t = HOLD_TARGETS.includes(form.hold_target) ? form.hold_target : null;
    if (!t) problems.push("hold_target");
    const hold = {};
    if (t === "zero" || t === "here") hold.target = t;
    else if (t) {
      const v = num(form.hold_value);
      if (v === null || Number.isNaN(v) || !Number.isInteger(v)) problems.push("hold_value");
      else hold.target = `${t}:${v}`;
    }
    for (const [k, key] of [["hold_kp", "kp"], ["hold_ki", "ki"], ["hold_kd", "kd"], ["hold_min_duty", "min_duty"]]) {
      const v = num(form[k]);
      if (Number.isNaN(v)) problems.push(k);
      else if (v !== null) hold[key] = v;
    }
    const db = num(form.hold_deadband);
    if (Number.isNaN(db) || (db !== null && !Number.isInteger(db))) problems.push("hold_deadband");
    else if (db !== null) hold.deadband = db;
    fields.hold = hold;
  } else if (kind === "distance") {
    const turns = num(form.stop_turns);
    if (Number.isNaN(turns) || turns === null || turns <= 0) problems.push("stop_turns");
    else fields.stop = { turns };
  } else if (kind === "measure") {
    fields.measure = true;
  }
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

// Plan 0.2 and 4.4: shown and recorded, never enforced.
export function advice(form) {
  const out = [];
  if (fieldState(form).rangeOff) out.push("rangeOff");
  const duty = num(form.duty);
  if (runKind(form) === "distance" && duty !== null && Math.abs(duty) > HIGH_DUTY_DISTANCE) out.push("highDutyDistance");
  return out;
}

// The checked-range mark before the start (plan 4.4 decision 2): the 5.1 bundle's range is not in yet, so a stop after
// N turns or a measuring run is 'unchecked'; a hold's gains and settling are checked apart. The result carries the back
// end's own mark.
export function rangeMarkBefore(form) {
  const kind = runKind(form);
  if (kind === "hold") return "not_applicable";
  if (kind === "distance" || kind === "measure") return "unchecked";
  return null;
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
    kind: result.kind ?? "duty",
    start: result.start ?? null,
    labels: [...(v.labels ?? [])],
    notes: [...(result.notes ?? [])],
    lateAborts: [...(result.late_aborts ?? [])],
    boardCause: result.board_cause ?? null,
    cleanup: result.cleanup ?? null,
    recordComplete: result.record_complete ?? null,
    rangeMark: result.range_mark ?? null,
    advisories: [...(result.advisories ?? [])],
    holdVerdict: result.hold ? {
      result: result.hold.verdict?.result ?? null,
      why: [...(result.hold.verdict?.why ?? [])],
      endError: result.hold.verdict?.end_error ?? null,
      insideShare: result.hold.verdict?.settle?.inside_share ?? null,
      entries: result.hold.entries?.label ?? null,
      target: result.hold.settings?.target ?? null,
    } : null,
    distanceVerdict: result.distance ? {
      result: result.distance.result ?? null,
      stopCounts: result.distance.stop_counts ?? null,
      movedAtCut: result.distance.moved_at_cut ?? null,
    } : null,
    measure: result.measure ? { moved: result.measure.moved ?? null, status: result.measure.status ?? null } : null,
    rest: result.rest ? { result: result.rest.result ?? null, position: result.rest.position ?? null } : null,
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
  // no after (the board was lost): unknown, never "none" (round 3 P2-2)
  const faults = (v) => (!v ? "확인 불가" : v.fault_flags?.length ? v.fault_flags.join(", ") : "없음");
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
    this.expire(atMs);
    if (!b) {
      // the board lost while the socket lives: a gap that breaks the lines (round 3 P2-4)
      if (this.samples.length && !this.samples.at(-1).gap) this.samples.push({ t: atMs, gap: true });
      return;
    }
    const run = state?.run ?? null;
    this.samples.push({ t: atMs, boot: b.boot_id ?? null, duty: [...(b.duty_e4 ?? [0, 0, 0, 0])],
      current: [...(b.current_ma ?? [0, 0, 0, 0])], steer: b.steering_rad ?? null, lift: b.lift_m ?? null,
      wheel: [...(b.wheel_count ?? [0, 0])],
      // the run's own count values (null until they are this run's, and across a pair gap: plan 4.4 decision 5)
      holdError: Number.isFinite(run?.error_counts) ? run.error_counts : null,
      deadband: Number.isFinite(run?.deadband) ? run.deadband : null,
      moved: Number.isFinite(run?.moved_counts) ? run.moved_counts : null });
  }

  // drop what is older than the window (called with each sample and by the screen with its clock)
  expire(atMs) {
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
      if (s.gap) {
        prev = null; // a board gap: the next sample starts again
        return null;
      }
      const c = s.wheel[k];
      if (prev === null || s.boot !== boot) {
        const reboot = prev !== null; // a boot change without a gap: this sample breaks the line
        prev = c;
        boot = s.boot;
        total = 0;
        return reboot ? null : 0;
      }
      total += wheelDelta(c, prev);
      prev = c;
      return total;
    });
    return this.samples.map((s, i) => (s.gap ? { t: s.t, value: null, duty: null, current: [null, null, null, null] } : {
      t: s.t,
      value: axis === "M" ? s.steer : axis === "H" ? s.lift : counts[i],
      duty: s.duty[AXES.indexOf(axis)] / 1e4,
      current: s.current,
      holdError: s.holdError ?? null,
      deadband: s.deadband ?? null,
      moved: s.moved ?? null,
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
  let replySeq = 0; // every answer counted (the screen redraws on a new one even when the list is full)
  let runOpen = false; // this tab's start, from its send to its result, a refusal or a disconnect
  let clearOpen = false; // this tab's clear, likewise
  const graph = new Graph();

  function track(kind, promise, requestId) {
    pending.set(requestId, kind);
    promise.then((r) => {
      if (!pending.has(requestId)) return; // forgotten at a disconnect: its answer is not this connection's
      pending.delete(requestId);
      // accepted: its own result is the next one -- unless that result already came (a late ACK of a resend)
      // only this tab's latest start's ACK clears the result: an earlier run's late ACK never does (round 4 P2-1)
      if (kind === "bench_run" && r?.t === "ack" && requestId === runId && result?.request_id !== requestId) {
        result = null;
      }
      if (r?.t !== "ack" && kind === "bench_run" && requestId === runId) runOpen = false; // refused: no run
      if (r?.t !== "ack" && kind === "clear_faults" && requestId === clearId) clearOpen = false;
      replySeq += 1;
      replies.push({ kind, t: r?.t ?? "none", reason: r?.reason ?? null, detail: r?.detail ?? null, at: now() });
      while (replies.length > 20) replies.shift();
    });
  }

  function send(kind, fields) {
    if (!visible || !BENCH_REQUESTS.has(kind)) return null;
    const id = `bench-${kind}-${Math.random().toString(16).slice(2)}-${now()}`;
    if (kind === "bench_run") {
      runId = id;
      runOpen = true;
    }
    if (kind === "clear_faults") {
      clearId = id;
      clearOpen = true;
      clear = null; // a new attempt: an earlier clear's end is not shown as this one's
    }
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
        runOpen = false;
        clearOpen = false;
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
      runOpen = false;
      clearOpen = false;
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
        if (msg.request_id === runId) {
          result = msg; // the run this tab asked for: ACK is acceptance, this is its end
          runOpen = false;
        }
        previous = msg;
      } else if (msg.t === "clear_result") {
        if (msg.request_id === clearId) {
          clear = msg; // this tab's clear: its end (ACK was only acceptance)
          clearOpen = false;
        }
      }
    },
    run(form) {
      // one start at a time: not while one is on its way or a run is going (round 2 P2-2)
      if (runOpen || state?.run) return { sent: null, problems: ["busy"] }; // until its own result (round 3 P2-1)
      const { fields, problems } = runFields(form);
      if (problems.length) return { sent: null, problems };
      return { sent: send("bench_run", fields), problems };
    },
    benchStop: () => send("bench_stop", {}),
    disarm: () => send("bench_disarm", {}),
    clearFaults(cause) {
      const c = String(cause ?? "").trim();
      if (!c) return { sent: null, problems: ["cause"] };
      if (clearOpen || state?.clear) return { sent: null, problems: ["busy"] }; // one clear at a time (P2-2)
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
      return runOpen;
    },
    get clearPending() {
      return clearOpen;
    },
    get replySeq() {
      return replySeq;
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
