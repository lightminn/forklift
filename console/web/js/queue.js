// The mission queue (docs/plans/2026-10-22-console-mission-queue.md): saved transports run one after another, and
// any anomaly stops the queue until the operator resumes it -- the page never submits the next mission on its own
// after one (user decision 2026-10-22). Pure module: no DOM, no sockets, no storage; times come in as arguments.

import { ACTIVE } from "./results.js";
import { selectable } from "./wizard.js";

export const SILENCE_MS = 2000; // no state for this long: the link is treated as gone (plan D3, D6)
const SETTLE_MS = 1000; // after a result, before the next submit (plan D3; the cycle probe's 1 s)
// A vehicle feedback loss is one that lasts: the live runner's 0.1 s feedback age limit shows a control tick longer
// than that as one state with feedback_ok false in a normal run (Isaac live run 2026-10-22); the status panel opens
// its alarm after the same 0.5 s. A submit still needs the feedback valid at that moment (next()).
const FEEDBACK_LOST_MS = 500;
const FINISHED_KEEP = 50;
const PREEMPT = new Set(["preempt", "safety_preempt"]); // live arbiter, fake vehicle
const UNSURE_REJECTS = new Set(["busy", "stale"]); // the request may still run (plan D5)
const KINDS = new Set(["fixed", "auto"]);

export function newItem({ kind = "fixed", pallet_id, destination_id, label }, nowUnix, newId) {
  return { item_id: newId(), kind, pallet_id: pallet_id ?? null, destination_id: destination_id ?? null,
           label: label ?? `${pallet_id} → ${destination_id}`, added_unix: nowUnix };
}

function check(palletId, destinationId, targets, state) {
  if (!targets) return "대상 목록을 아직 받지 못했습니다";
  const pallet = (targets.pallets ?? []).find((p) => p.pallet_id === palletId);
  if (!pallet) return `팔레트 ${palletId} 가 목록에 없습니다`;
  if (!selectable(pallet, state?.load ?? null)) return `팔레트 ${palletId} 를 지금 고를 수 없습니다(${pallet.state})`;
  if (!(targets.destinations ?? []).some((d) => d.destination_id === destinationId)) {
    return `하역 칸 ${destinationId} 가 목록에 없습니다`;
  }
  return null;
}

// Only "fixed" runs now; "auto" (decide at submit time) is reserved for the continuous-transport work (plan D1).
export function resolveItem(item, targets, state) {
  if (item?.kind !== "fixed") return { ok: false, reason: "unsupported_kind" };
  const why = check(item.pallet_id, item.destination_id, targets, state);
  return why ? { ok: false, reason: why } : { ok: true, pallet_id: item.pallet_id, destination_id: item.destination_id };
}

export function presetStatus(preset, targets, state) {
  const why = check(preset?.pallet_id, preset?.destination_id, targets, state);
  return { usable: why === null, reason: why };
}

const OPERATOR = { cancel: "운용자 취소", pause: "운용자 일시정지", stop: "STOP", manual_submit: "수동 지시" };
// Stop reasons that are the operator's own doing or the queue at rest -- not an anomaly to raise (status panel plan
// D2). A STOP seen in the state ("정지됨(STOP)": another device or the vehicle) is not one of them.
export const CALM_REASONS = ["시작 전", "완료", "콘솔 다시 열림", "운용자 멈춤", ...Object.values(OPERATOR)];

export class MissionQueue {
  constructor() {
    this.items = [];
    this.run = "stopped";
    this.stopReason = "시작 전";
    this.phase = "waiting"; // of the front item: waiting | submitting | in_mission
    this.current = null; // {item_id, request_id, mission_id, epoch, pallet_id, destination_id}
    this.finished = []; // newest first: {item, outcome, reason, mission_id}
    this.counts = { done: 0, failed: 0, cancelled: 0 };
    this.epoch = null;
    this.lastStateAt = null; // when the last state was received -- only a received state sets it (Codex r5)
    this.waitSince = null; // resumed with no state since the welcome: the first one is due by SILENCE_MS
    this.feedbackLostSince = null; // the first of the states with feedback_ok false that came in a row
    this.lastResultAt = null;
    this.applied = new Set(); // mission ids whose result was applied (last_result and missions both carry it)
    this.early = new Map(); // mission id -> result seen before the ack named it
    this.version = 0; // bumped on every change the screen or the storage should see
  }

  // ---------------------------------------------------------------- operator
  add(item) {
    if (!item || !KINDS.has(item.kind)) return false;
    this.items.push({ ...item });
    return this.#changed();
  }

  remove(itemId) {
    if (this.#isCurrent(itemId)) return false;
    const n = this.items.length;
    this.items = this.items.filter((it) => it.item_id !== itemId);
    return this.items.length !== n && this.#changed();
  }

  move(itemId, delta) {
    const i = this.items.findIndex((it) => it.item_id === itemId);
    const j = i + delta;
    if (i < 0 || j < 0 || j >= this.items.length || this.#isCurrent(itemId)) return false;
    if (this.current && this.items[j].item_id === this.current.item_id) return false; // nothing in front of it
    if (this.items[i].uncertain || this.items[j].uncertain) return false; // nor across an uncertain one (Codex r4)
    [this.items[i], this.items[j]] = [this.items[j], this.items[i]];
    return this.#changed();
  }

  clear() {
    this.items = this.items.filter((it) => this.#isCurrent(it.item_id));
    return this.#changed();
  }

  resume(nowMs) {
    if (!this.items.length || this.items.some((it) => it.uncertain)) return false; // the operator decides first
    this.run = "running";
    this.stopReason = null;
    this.waitSince = nowMs;
    return this.#changed();
  }

  stop(reason) {
    const forget = this.phase === "submitting" ? this.current?.request_id : undefined;
    if (forget !== undefined) this.#dropCurrentAsUncertain();
    if (this.run === "running") {
      this.run = "stopped";
      this.stopReason = reason;
    }
    this.#changed();
    return forget === undefined ? {} : { forget };
  }

  operatorAction(kind) {
    return this.stop(OPERATOR[kind] ?? kind);
  }

  resolveUncertain(itemId, action) {
    const it = this.items.find((x) => x.item_id === itemId);
    if (!it || !it.uncertain) return false;
    if (action === "drop") this.items = this.items.filter((x) => x.item_id !== itemId);
    else if (action === "resubmit") {
      it.uncertain = false;
      delete it.maybe; // a later result of that mission is no longer this item's
    }
    else return false;
    return this.#changed();
  }

  // ---------------------------------------------------------------- inputs
  onWelcome(epoch) {
    let out = {};
    if (this.epoch !== null && epoch !== this.epoch) {
      out = this.stop("새 세션");
      if (this.current) this.#dropCurrentAsUncertain(); // its result never comes in the new session
      this.applied.clear();
      this.early.clear();
      this.lastResultAt = null;
    }
    this.epoch = epoch;
    this.lastStateAt = null; // a new connection: silence is counted from its first state
    this.#changed();
    return out;
  }

  onDisconnect() {
    return this.stop("연결 끊김");
  }

  onLeaseLost() {
    return this.stop("조종 권한 상실");
  }

  onState(state, nowMs) {
    let out = {};
    const ref = this.lastStateAt ?? this.waitSince;
    if (this.run === "running" && ref !== null && nowMs - ref > SILENCE_MS) {
      out = this.stop("상태 수신 끊김"); // decided before the late state refreshes the clock (plan D3)
    }
    this.lastStateAt = nowMs;
    // a loss that already lasted 0.5 s counts even if this state says recovered: it was not seen to end sooner
    // (Codex ops round 1)
    const lost = this.#feedbackLost(nowMs);
    if (state?.feedback_ok === false) this.feedbackLostSince ??= nowMs;
    else this.feedbackLostSince = null;
    if (this.run === "running" && lost) return this.stop("차량 피드백 없음");
    if (this.run !== "running" || !state) return out;
    if (state.mode === "STOPPED") return this.stop("정지됨(STOP)");
    if (state.mode === "TELEOP") return this.stop("수동 조종");
    if (PREEMPT.has(state.stop_kind)) return this.stop("안전 선점 정지");
    // the arbiter does not preempt a lost feedback in PAUSED: waiting would submit once it recovers (plan D4) --
    // a loss that lasts, not one late state
    if (this.#feedbackLost(nowMs)) return this.stop("차량 피드백 없음");
    const mi = state.mission;
    if (mi && ACTIVE.includes(mi.status)) {
      if (!this.#isOurs(mi)) return this.stop("대기열이 내지 않은 미션");
      if (mi.status === "paused" || mi.status === "cancelling") return this.stop(mi.status === "paused" ? "미션 일시정지" : "미션 취소 중");
    }
    return out;
  }

  onResult(result, nowMs) {
    const id = result?.mission_id;
    if (!id || this.applied.has(id)) return;
    const maybe = this.items.find((x) => x.uncertain && x.maybe?.mission_id === id && x.maybe.epoch === this.epoch);
    if (maybe) {
      this.#record(maybe.item_id, result); // recorded and out of the queue; the queue stays as it is
      return;
    }
    if (this.current && this.current.mission_id === id) this.#finish(result, nowMs);
    else if (this.current && this.phase === "submitting") this.early.set(id, { result, nowMs });
  }

  onAnswer(key, answer) {
    const c = this.current;
    if (!c || this.phase !== "submitting" || key.epoch !== c.epoch || key.item_id !== c.item_id
        || key.request_id !== c.request_id) {
      return {};
    }
    if (answer?.t === "ack" && typeof answer.mission_id === "string") {
      c.mission_id = answer.mission_id;
      this.phase = "in_mission";
      const early = this.early.get(answer.mission_id);
      this.early.clear();
      // a mission taken for ours before the ack, and the ack names another: that one was not ours (Codex r4)
      const foreign = c.seen !== undefined && c.seen !== answer.mission_id;
      delete c.seen;
      if (early) this.#finish(early.result, early.nowMs);
      if (foreign) return this.stop("대기열이 내지 않은 미션");
      this.#changed();
      return {};
    }
    const reason = answer?.reason ?? "unknown";
    const item = this.items.find((x) => x.item_id === c.item_id);
    if (item && UNSURE_REJECTS.has(reason)) item.uncertain = true;
    this.current = null;
    this.phase = "waiting";
    this.early.clear();
    return this.stop(`거부: ${reason}`);
  }

  tick(nowMs) {
    const ref = this.lastStateAt ?? this.waitSince;
    if (this.run === "running" && ref !== null && nowMs - ref > SILENCE_MS) return this.stop("상태 수신 끊김");
    if (this.run === "running" && this.#feedbackLost(nowMs)) return this.stop("차량 피드백 없음");
    return {};
  }

  next(ctx, nowMs, newId) {
    if (this.run !== "running" || this.current) return null;
    const front = this.items[0];
    if (!front || front.uncertain) return null;
    const st = ctx.state;
    if (!ctx.connected || !ctx.welcomed || !ctx.hasLease || !st || !ctx.targets) return null;
    if (this.lastStateAt === null || nowMs - this.lastStateAt > SILENCE_MS) return null;
    if (st.mode !== "PAUSED" || st.stopped_confirmed !== true || st.feedback_ok === false) return null;
    if (st.mission && ACTIVE.includes(st.mission.status)) return null;
    if (this.lastResultAt !== null && nowMs - this.lastResultAt < SETTLE_MS) return null;
    const r = resolveItem(front, ctx.targets, st);
    if (!r.ok) {
      this.stop(r.reason === "unsupported_kind" ? "아직 실행할 수 없는 항목(자동 선택)" : `대상 확인 실패: ${r.reason}`);
      return null;
    }
    const submit = { item_id: front.item_id, request_id: newId(), pallet_id: r.pallet_id,
                     destination_id: r.destination_id, targets_revision: ctx.targets.targets_revision };
    this.current = { item_id: front.item_id, request_id: submit.request_id, mission_id: null, epoch: this.epoch,
                     pallet_id: r.pallet_id, destination_id: r.destination_id };
    this.phase = "submitting";
    this.early.clear();
    this.#changed();
    return { submit };
  }

  // ---------------------------------------------------------------- storage (plan D7)
  toStorage() {
    return { v: 1, epoch: this.epoch, items: this.items, current: this.current, phase: this.phase,
             finished: this.finished, counts: this.counts };
  }

  static fromStorage(obj, { epoch, results = [] } = {}) {
    const q = new MissionQueue();
    q.epoch = epoch ?? null;
    if (!obj || obj.v !== 1 || !Array.isArray(obj.items)) return q; // nothing stored: a new queue
    q.stopReason = "콘솔 다시 열림";
    q.items = obj.items.filter((it) => it && typeof it.item_id === "string" && KINDS.has(it.kind)).map((it) => ({ ...it }));
    if (Array.isArray(obj.finished)) q.finished = obj.finished.slice(0, FINISHED_KEEP);
    if (obj.counts && typeof obj.counts === "object") q.counts = { done: 0, failed: 0, cancelled: 0, ...obj.counts };
    const c = obj.current;
    const it = c && q.items.find((x) => x.item_id === c.item_id);
    if (it) {
      const found = obj.phase === "in_mission" && obj.epoch === epoch
        ? results.find((r) => r.mission_id === c.mission_id) : null;
      if (found) {
        q.current = { ...c };
        q.phase = "in_mission";
        q.#finish(found, 0);
      } else {
        it.uncertain = true; // submitted, result unknown: the operator decides (plan D6, D7)
        if (c.mission_id) it.maybe = { mission_id: c.mission_id, epoch: obj.epoch };
      }
    }
    q.current = null;
    q.phase = "waiting";
    return q;
  }

  // ---------------------------------------------------------------- internals
  #feedbackLost(nowMs) {
    return this.feedbackLostSince !== null && nowMs - this.feedbackLostSince >= FEEDBACK_LOST_MS;
  }

  #isCurrent(itemId) {
    return Boolean(this.current && this.current.item_id === itemId);
  }

  #isOurs(mission) {
    const c = this.current;
    if (!c) return false;
    if (c.mission_id) return mission.mission_id === c.mission_id;
    // submitted, no ack yet: the state may show the new mission first -- taken for ours, and checked against the ack
    if (this.phase !== "submitting" || mission.pallet_id !== c.pallet_id || mission.destination_id !== c.destination_id) {
      return false;
    }
    c.seen ??= mission.mission_id;
    return mission.mission_id === c.seen;
  }

  #dropCurrentAsUncertain() {
    const item = this.current && this.items.find((x) => x.item_id === this.current.item_id);
    if (item) {
      item.uncertain = true;
      if (this.current.mission_id) item.maybe = { mission_id: this.current.mission_id, epoch: this.current.epoch };
    }
    this.current = null;
    this.phase = "waiting";
    this.early.clear();
  }

  // a result into the finished list and the counts, its item out of the queue
  #record(itemId, result) {
    this.applied.add(result.mission_id);
    const item = this.items.find((x) => x.item_id === itemId) ?? null;
    this.items = this.items.filter((x) => x.item_id !== itemId);
    const outcome = result.outcome;
    this.finished.unshift({ item, outcome, reason: result.reason ?? null, mission_id: result.mission_id });
    this.finished = this.finished.slice(0, FINISHED_KEEP);
    if (outcome === "done") this.counts.done += 1;
    else if (outcome === "cancelled") this.counts.cancelled += 1;
    else this.counts.failed += 1;
    this.#changed();
  }

  // the current mission's result: recorded, and the queue goes on or stops by its outcome
  #finish(result, nowMs) {
    this.#record(this.current.item_id, result);
    const outcome = result.outcome;
    this.current = null;
    this.phase = "waiting";
    this.lastResultAt = nowMs;
    if (this.run === "running") {
      if (outcome === "done") {
        if (!this.items.length) this.stop("완료");
      } else if (outcome === "failed" || outcome === "recovery_required") {
        this.stop(`실패${result.reason ? `: ${result.reason}` : ""}`);
      } else if (outcome === "cancelled") {
        this.stop("취소");
      } else {
        this.stop(String(outcome));
      }
    }
    this.#changed();
  }

  #changed() {
    this.version += 1;
    return true;
  }
}
