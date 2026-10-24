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
// CT-2 (docs/plans/2026-10-24-ct2-console-auto-item.md): the vehicle's bound from a pick_request to its pick, wall
// clock; the vehicle announces it in welcome capabilities.next_pick_deadline_ms (5000 when it does not)
export const PICK_DEADLINE_MS = 5000;
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

// A fixed item's pallet and destination, checked against the newest targets; an auto item is resolved by the
// vehicle's pick instead (CT-2), so here it is unsupported.
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
    this.phase = "waiting"; // of the front item: waiting | picking | picked (an auto item) | submitting | in_mission
    this.current = null; // {item_id, request_id, mission_id, epoch, pallet_id, destination_id}
    this.finished = []; // newest first: {item, outcome, reason, mission_id, pallet_id, destination_id}
    // CT-2: the auto item's pick_request out ({item_id, request_id, epoch, sentAt}) and the pick held ({pallet_id,
    // destination_id, targets_revision, plan, missions}); asks counts the requests of one pick (one re-ask allowed)
    this.pick = null;
    this.held = null;
    this.pickAsks = 0;
    this.pickDeadlineMs = PICK_DEADLINE_MS;
    this.lastPick = null; // the last pick taken (the screen shows its plan)
    this.lastPickBlocked = null; // what blocked the last failed pick (the screen explains it)
    this.toForget = []; // request ids the session should stop resending (drainForget)
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
    if (item.kind === "auto" && this.items.some((it) => it.kind === "auto")) return false; // one is all of them
    this.items.push({ ...item });
    return this.#changed();
  }

  remove(itemId) {
    if (this.#isCurrent(itemId)) return false;
    if (this.pick?.item_id === itemId || (this.held && this.items[0]?.item_id === itemId)) this.#dropPick();
    const n = this.items.length;
    this.items = this.items.filter((it) => it.item_id !== itemId);
    return this.items.length !== n && this.#changed();
  }

  move(itemId, delta) {
    const i = this.items.findIndex((it) => it.item_id === itemId);
    const j = i + delta;
    if (i < 0 || j < 0 || j >= this.items.length || this.#isCurrent(itemId)) return false;
    if (this.current && this.items[j].item_id === this.current.item_id) return false; // nothing in front of it
    if ((this.phase === "picking" || this.phase === "picked") && (i === 0 || j === 0)) return false; // nor of a pick
    if (this.items[i].uncertain || this.items[j].uncertain) return false; // nor across an uncertain one (Codex r4)
    [this.items[i], this.items[j]] = [this.items[j], this.items[i]];
    return this.#changed();
  }

  clear() {
    this.#dropPick();
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
    // a pick asked or held is let go -- nothing was submitted, so nothing is uncertain (CT-2)
    let forget = this.phase === "picking" ? this.pick?.request_id : undefined;
    if (this.phase === "picking" || this.phase === "picked") this.#dropPick(false);
    if (this.phase === "submitting") {
      forget = this.current?.request_id;
      this.#dropCurrentAsUncertain();
    }
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
    else if (action === "resubmit" || action === "clear") {
      // fixed: submit it again; auto ("clear"): the uncertain mark was on one submit, the item goes on -- its next
      // pick is worked out from the vehicle's own list, so the same pallet is not moved twice (CT-2 C4)
      this.#clearUncertain(it);
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
      this.lastPick = null; // another session's pick and its blockers say nothing about this one (CT-2)
      this.lastPickBlocked = null;
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
      this.#record(maybe.item_id, result); // recorded (a fixed item out of the queue); the queue stays as it is
      return;
    }
    if (this.current && this.current.mission_id === id) this.#finish(result, nowMs);
    else if (this.current && this.phase === "submitting") this.early.set(id, { result, nowMs });
  }

  onAnswer(key, answer) {
    const pk = this.pick;
    if (pk && key.epoch === pk.epoch && key.item_id === pk.item_id && key.request_id === pk.request_id) {
      if (answer?.t === "ack") return {}; // the pick comes as its own message
      return this.stop(`자동 선택 거부: ${answer?.reason ?? "unknown"}`);
    }
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

  // CT-2: the vehicle's answer to a pick_request (contract D3), matched by its key; targetsRevision is the list the
  // page holds now
  onPick(key, msg, nowMs, targetsRevision) {
    const pk = this.pick;
    if (!pk || this.phase !== "picking" || key.epoch !== pk.epoch || key.item_id !== pk.item_id
        || key.request_id !== pk.request_id || msg?.request_id !== pk.request_id) {
      return {};
    }
    const reason = msg.reason ?? "unknown";
    if (reason === "ok" && msg.pick) {
      if (msg.targets_revision !== targetsRevision) return this.#reAsk();
      this.held = { pallet_id: msg.pick.pallet_id, destination_id: msg.pick.destination_id,
                    targets_revision: msg.targets_revision, plan: msg.plan ?? null, missions: msg.missions ?? null };
      this.lastPick = { ...this.held };
      this.lastPickBlocked = null;
      this.pick = null;
      this.phase = "picked";
      this.#changed();
      return {};
    }
    if (reason === "none_remaining") {
      const autoId = pk.item_id;
      this.#dropPick(false);
      this.items = this.items.filter((x) => x.item_id !== autoId);
      this.lastPickBlocked = null;
      this.lastPick = null;
      if (!this.items.length && this.run === "running") return this.stop("완료");
      this.#changed();
      return {};
    }
    this.lastPickBlocked = msg.blocked ?? null;
    this.lastPick = null; // its plan no longer stands
    return this.stop(`자동 선택 실패: ${reason}`);
  }

  // request ids the session should stop resending (a pick let go by a removal or a clear)
  drainForget() {
    const out = this.toForget;
    this.toForget = [];
    return out;
  }

  tick(nowMs) {
    if (this.run === "running" && this.phase === "picking" && nowMs - this.pick.sentAt > this.pickDeadlineMs) {
      return this.stop("자동 선택 응답 없음");
    }
    const ref = this.lastStateAt ?? this.waitSince;
    if (this.run === "running" && ref !== null && nowMs - ref > SILENCE_MS) return this.stop("상태 수신 끊김");
    if (this.run === "running" && this.#feedbackLost(nowMs)) return this.stop("차량 피드백 없음");
    return {};
  }

  next(ctx, nowMs, newId) {
    if (this.run !== "running" || this.current || this.phase === "picking") return null;
    const front = this.items[0];
    if (!front || front.uncertain) return null;
    const st = ctx.state;
    if (!ctx.connected || !ctx.welcomed || !ctx.hasLease || !st || !ctx.targets) return null;
    if (this.lastStateAt === null || nowMs - this.lastStateAt > SILENCE_MS) return null;
    if (st.mode !== "PAUSED" || st.stopped_confirmed !== true || st.feedback_ok === false) return null;
    if (st.mission && ACTIVE.includes(st.mission.status)) return null;
    if (this.lastResultAt !== null && nowMs - this.lastResultAt < SETTLE_MS) return null;
    if (front.kind === "auto") return this.#nextAuto(front, ctx, nowMs, newId);
    const r = resolveItem(front, ctx.targets, st);
    if (!r.ok) {
      this.stop(r.reason === "unsupported_kind" ? "아직 실행할 수 없는 항목(자동 선택)" : `대상 확인 실패: ${r.reason}`);
      return null;
    }
    return this.#submit(front, r.pallet_id, r.destination_id, ctx, newId);
  }

  #submit(front, palletId, destinationId, ctx, newId) {
    const submit = { item_id: front.item_id, request_id: newId(), pallet_id: palletId,
                     destination_id: destinationId, targets_revision: ctx.targets.targets_revision };
    this.current = { item_id: front.item_id, request_id: submit.request_id, mission_id: null, epoch: this.epoch,
                     pallet_id: palletId, destination_id: destinationId };
    this.phase = "submitting";
    this.early.clear();
    this.#changed();
    return { submit };
  }

  // the auto item (CT-2 C2): ask for a pick, or submit the pick held -- the submit conditions were just checked again
  #nextAuto(front, ctx, nowMs, newId) {
    if (ctx.nextPick !== true) {
      this.stop("차량이 자동 선택을 지원하지 않습니다");
      return null;
    }
    if (this.phase === "picked") {
      const h = this.held;
      if (h.targets_revision === ctx.targets.targets_revision) {
        const r = resolveItem({ kind: "fixed", pallet_id: h.pallet_id, destination_id: h.destination_id }, ctx.targets, ctx.state);
        if (!r.ok) {
          this.stop(`대상 확인 실패: ${r.reason}`);
          return null;
        }
        this.held = null;
        this.pickAsks = 0;
        return this.#submit(front, r.pallet_id, r.destination_id, ctx, newId);
      }
      if (this.pickAsks >= 2) {
        this.stop("대상 목록이 계속 바뀜");
        return null;
      }
      this.held = null; // the list changed under the pick: the one re-ask
    }
    const request_id = newId();
    this.pick = { item_id: front.item_id, request_id, epoch: this.epoch, sentAt: nowMs };
    this.pickAsks += 1;
    this.phase = "picking";
    this.#changed();
    return { pick_request: { item_id: front.item_id, request_id } };
  }

  #reAsk() {
    if (this.pickAsks >= 2) return this.stop("대상 목록이 계속 바뀜");
    this.pick = null;
    this.held = null;
    this.phase = "waiting";
    this.#changed();
    return {};
  }

  // a pick asked or held let go; a request out goes to the list to forget unless the caller returns it itself
  #dropPick(queueForget = true) {
    if (queueForget && this.phase === "picking" && this.pick) this.toForget.push(this.pick.request_id);
    if (this.phase === "picking" || this.phase === "picked") this.phase = "waiting";
    this.pick = null;
    this.held = null;
    this.pickAsks = 0;
  }

  // ---------------------------------------------------------------- storage (plan D7)
  toStorage() {
    // a pick asked or held is not kept: a reopened page asks again (CT-2 C5)
    const phase = this.phase === "picking" || this.phase === "picked" ? "waiting" : this.phase;
    return { v: 1, epoch: this.epoch, items: this.items, current: this.current, phase,
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
        if (it.kind === "auto") {
          it.uncertain_pallet_id = c.pallet_id ?? null;
          it.uncertain_destination_id = c.destination_id ?? null;
        }
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
      if (item.kind === "auto") { // the mark is on this one submit (CT-2 C4)
        item.uncertain_pallet_id = this.current.pallet_id;
        item.uncertain_destination_id = this.current.destination_id;
      }
    }
    this.current = null;
    this.phase = "waiting";
    this.early.clear();
  }

  #clearUncertain(it) {
    it.uncertain = false;
    delete it.maybe; // a later result of that mission is no longer this item's
    delete it.uncertain_pallet_id;
    delete it.uncertain_destination_id;
  }

  // a result into the finished list and the counts with the pallet and destination it moved; a fixed item leaves the
  // queue, an auto item stays for the next pick (CT-2 C3) with any uncertain mark on that submit cleared
  #record(itemId, result) {
    this.applied.add(result.mission_id);
    const item = this.items.find((x) => x.item_id === itemId) ?? null;
    const cur = this.current?.item_id === itemId ? this.current : null;
    const palletId = cur?.pallet_id ?? item?.uncertain_pallet_id ?? item?.pallet_id ?? result.pallet_id ?? null;
    const destinationId = cur?.destination_id ?? item?.uncertain_destination_id ?? item?.destination_id
      ?? result.destination_id ?? null;
    if (item?.kind === "auto") this.#clearUncertain(item);
    else this.items = this.items.filter((x) => x.item_id !== itemId);
    const outcome = result.outcome;
    this.finished.unshift({ item: item ? { ...item } : null, outcome, reason: result.reason ?? null,
                            mission_id: result.mission_id, pallet_id: palletId, destination_id: destinationId });
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
