"""The console's mission queue (console/web/js/queue.js, docs/plans/2026-10-22-console-mission-queue.md D1-D6) in
Python, for the live probe (CT-1 PR 4: continuous runs under the queue's own policy -- ws1 has no Node). A line for
line port of the decisions the probe needs: fixed items and the CT-2 auto item (its pick_request / pick, picking and
picked), resume/stop, the welcome, disconnect and lease events, the
states (silence, feedback loss, STOPPED / TELEOP, a safety preempt, a mission not ours, paused / cancelling), the
results, the submit answers, the timer and the next submit. Plan D5's one exception is ported too (the session's first submit refused not_stopped goes out once more after a
1 s stand: user decision 2026-10-25, feat/queue-start-resubmit 55b3ab81). Times are milliseconds, as in the JS; the stop reasons
are the JS strings, so tests/unit/tools/test_mission_queue_port.py can run both on the same events."""

from __future__ import annotations

SILENCE_MS = 2000
SETTLE_MS = 1000
FEEDBACK_LOST_MS = 500
FINISHED_KEEP = 50
# CT-2 (console/web/js/queue.js PICK_DEADLINE_MS): the vehicle's bound from a pick_request to its pick, wall clock;
# the vehicle announces it in welcome capabilities.next_pick_deadline_ms (5000 when it does not)
PICK_DEADLINE_MS = 5000
PREEMPT = {"preempt", "safety_preempt"}
UNSURE_REJECTS = {"busy", "stale"}
# Plan D5's one exception (queue.js START_RETRY_STAND_MS): the session's first queue submit refused not_stopped is
# submitted again once, the same item, after the truck has stood this long in a row (PAUSED, stop confirmed, feedback
# valid, on the page's clock like the other waits)
START_RETRY_STAND_MS = 1000
KINDS = {"fixed", "auto"}
# console/web/js/results.js ACTIVE
ACTIVE = ["accepted", "preparing", "running", "paused", "resuming", "held", "cancelling", "recovering"]
OPERATOR = {"cancel": "운용자 취소", "pause": "운용자 일시정지", "stop": "STOP", "manual_submit": "수동 지시"}


def new_item(pallet_id, destination_id, item_id, *, kind="fixed", label=None, added_unix=0.0) -> dict:
    return {"item_id": item_id, "kind": kind, "pallet_id": pallet_id, "destination_id": destination_id,
            "label": label or f"{pallet_id} → {destination_id}", "added_unix": added_unix}


def _carried_label(pallet, load):  # console/web/js/wizard.js
    on_forks = (pallet or {}).get("state") == "carried" and (load or {}).get("state") == "loaded" \
        and (load or {}).get("pallet_id") == (pallet or {}).get("pallet_id")
    return "실린 팔레트 — 바로 운반" if on_forks else None


def _selectable(pallet, load=None) -> bool:
    if not pallet:
        return False
    return pallet.get("state") in ("available", "delivered") or _carried_label(pallet, load) is not None


def _check(pallet_id, destination_id, targets, state):
    if not targets:
        return "대상 목록을 아직 받지 못했습니다"
    pallet = next((p for p in targets.get("pallets") or [] if p.get("pallet_id") == pallet_id), None)
    if not pallet:
        return f"팔레트 {pallet_id} 가 목록에 없습니다"
    if not _selectable(pallet, (state or {}).get("load")):
        return f"팔레트 {pallet_id} 를 지금 고를 수 없습니다({pallet.get('state')})"
    if not any(d.get("destination_id") == destination_id for d in targets.get("destinations") or []):
        return f"하역 칸 {destination_id} 가 목록에 없습니다"
    return None


def resolve_item(item, targets, state) -> dict:
    if (item or {}).get("kind") != "fixed":
        return {"ok": False, "reason": "unsupported_kind"}
    why = _check(item["pallet_id"], item["destination_id"], targets, state)
    return {"ok": False, "reason": why} if why else {"ok": True, "pallet_id": item["pallet_id"],
                                                      "destination_id": item["destination_id"]}


class MissionQueue:
    def __init__(self):
        self.items: list = []
        self.run = "stopped"
        self.stop_reason = "시작 전"
        self.phase = "waiting"
        self.current: dict | None = None
        self.finished: list = []
        self.counts = {"done": 0, "failed": 0, "cancelled": 0}
        self.epoch = None
        self.submits_this_epoch = 0  # the queue's own submits in this session (the first one may be resubmitted once)
        self.start_retry: dict | None = None  # {item_id, stand_since}: the one resubmit waiting for its stand
        self.start_retry_used = False  # spent for this session
        self.last_state_at = None
        self.wait_since = None
        self.feedback_lost_since = None
        self.last_result_at = None
        self.applied: set = set()
        self.early: dict = {}
        self.stops: list = []  # (reason, now_ms): every stop, for the probe's report
        # CT-2: the auto item's pick_request (kept until acked or let go, through a pick held) and the pick held
        # (queue.js pick / held / pickAsks, feat/ct2-console 2a4c64c3)
        self.pick: dict | None = None
        self.held: dict | None = None
        self.pick_asks = 0
        self.pick_deadline_ms = PICK_DEADLINE_MS
        self.last_pick: dict | None = None
        self.last_pick_blocked = None
        self.to_forget: list = []
        self.picks: list = []  # every pick answer taken (the probe's report)

    # --- operator ---------------------------------------------------------------------------------------------
    def add(self, item) -> bool:
        if not item or item.get("kind") not in KINDS:
            return False
        if item.get("kind") == "auto" and any(it.get("kind") == "auto" for it in self.items):
            return False  # one is all of them
        if item.get("kind") == "auto":
            self._forget_last_pick()  # a new auto item has had no pick yet
        self.items.append(dict(item))
        return True

    def resume(self, now_ms) -> bool:
        if not self.items or any(it.get("uncertain") for it in self.items):
            return False
        self.run, self.stop_reason, self.wait_since = "running", None, now_ms
        return True

    def stop(self, reason, now_ms=None) -> dict:
        # a pick asked or held is let go -- nothing was submitted, so nothing is uncertain (CT-2)
        forget = None
        if self.phase == "picking":
            forget = (self.pick or {}).get("request_id")
            self._drop_pick(False)
        elif self.phase == "picked":
            self._drop_pick(True)  # an unacked request still resending goes to the forget list
        if self.phase == "submitting":
            forget = (self.current or {}).get("request_id")
            self._drop_current_as_uncertain()
        if self.phase == "start_retry":  # the resubmit is not made: nothing went out
            self.phase = "waiting"
            self.start_retry = None
        if self.run == "running":
            self.run, self.stop_reason = "stopped", reason
            self.stops.append((reason, now_ms))
        return {} if forget is None else {"forget": forget}

    # --- inputs -----------------------------------------------------------------------------------------------
    def on_welcome(self, epoch) -> dict:
        out = {}
        if self.epoch is not None and epoch != self.epoch:
            out = self.stop("새 세션")
            if self.current:
                self._drop_current_as_uncertain()
            self.applied.clear()
            self.early.clear()
            self.last_result_at = None
            self.last_pick = None
            self.last_pick_blocked = None
        if epoch != self.epoch:  # a session of its own: its first submit may be resubmitted once again
            self.submits_this_epoch = 0
            self.start_retry_used = False
        self.epoch = epoch
        self.last_state_at = None
        return out

    def on_disconnect(self) -> dict:
        return self.stop("연결 끊김")

    def on_lease_lost(self) -> dict:
        return self.stop("조종 권한 상실")

    def on_state(self, state, now_ms) -> dict:
        out = {}
        ref = self.last_state_at if self.last_state_at is not None else self.wait_since
        if self.run == "running" and ref is not None and now_ms - ref > SILENCE_MS:
            out = self.stop("상태 수신 끊김", now_ms)
        self.last_state_at = now_ms
        lost = self._feedback_lost(now_ms)
        if state is not None and state.get("feedback_ok") is False:
            if self.feedback_lost_since is None:
                self.feedback_lost_since = now_ms
        else:
            self.feedback_lost_since = None
        if self.run == "running" and lost:
            return self.stop("차량 피드백 없음", now_ms)
        if self.run != "running" or not state:
            return out
        if state.get("mode") == "STOPPED":
            return self.stop("정지됨(STOP)", now_ms)
        if state.get("mode") == "TELEOP":
            return self.stop("수동 조종", now_ms)
        if state.get("stop_kind") in PREEMPT:
            return self.stop("안전 선점 정지", now_ms)
        if self._feedback_lost(now_ms):
            return self.stop("차량 피드백 없음", now_ms)
        if self.phase == "start_retry":  # the stand counted in a row; a break starts it over
            standing = (state.get("mode") == "PAUSED" and state.get("stopped_confirmed") is True
                        and state.get("feedback_ok") is not False)
            since = self.start_retry["stand_since"] if self.start_retry["stand_since"] is not None else now_ms
            self.start_retry["stand_since"] = since if standing else None
        mi = state.get("mission")
        if mi and mi.get("status") in ACTIVE:
            if not self._is_ours(mi):
                return self.stop("대기열이 내지 않은 미션", now_ms)
            if mi.get("status") in ("paused", "cancelling"):
                return self.stop("미션 일시정지" if mi["status"] == "paused" else "미션 취소 중", now_ms)
        return out

    def on_result(self, result, now_ms) -> None:
        mid = (result or {}).get("mission_id")
        if not mid or mid in self.applied:
            return
        maybe = next((x for x in self.items if x.get("uncertain") and (x.get("maybe") or {}).get("mission_id") == mid
                      and x["maybe"].get("epoch") == self.epoch), None)
        if maybe:
            self._record(maybe["item_id"], result)
            return
        if self.current and self.current.get("mission_id") == mid:
            self._finish(result, now_ms)
        elif self.current and self.phase == "submitting":
            self.early[mid] = (result, now_ms)

    def on_answer(self, key, answer, now_ms=None) -> dict:
        pk = self.pick
        if pk and key["epoch"] == pk["epoch"] and key["item_id"] == pk["item_id"] \
                and key["request_id"] == pk["request_id"]:
            if (answer or {}).get("t") == "ack":  # the pick comes as its own message; a held pick may now go
                # an ack is judged against the deadline too when its time is given (Codex CT-2 r2)
                if now_ms is not None and now_ms - pk["sent_at"] > self.pick_deadline_ms:
                    return self.stop("자동 선택 응답 없음", now_ms)
                pk["acked"] = True
                if pk.get("none"):
                    return self._end_auto(pk["item_id"], now_ms)  # none remaining came first, waiting for this ack
                return {}
            return self.stop(f"자동 선택 거부: {(answer or {}).get('reason') or 'unknown'}", now_ms)
        c = self.current
        if (not c or self.phase != "submitting" or key["epoch"] != c["epoch"] or key["item_id"] != c["item_id"]
                or key["request_id"] != c["request_id"]):
            return {}
        if (answer or {}).get("t") == "ack" and isinstance(answer.get("mission_id"), str):
            c["mission_id"] = answer["mission_id"]
            self.phase = "in_mission"
            early = self.early.get(answer["mission_id"])
            self.early.clear()
            foreign = "seen" in c and c["seen"] != answer["mission_id"]
            c.pop("seen", None)
            if early:
                self._finish(*early)
            if foreign:
                return self.stop("대기열이 내지 않은 미션", now_ms)
            return {}
        reason = (answer or {}).get("reason") or "unknown"
        item = next((x for x in self.items if x["item_id"] == c["item_id"]), None)
        if reason == "not_stopped" and self.submits_this_epoch == 1 and not self.start_retry_used and item:
            # Plan D5's one exception: the truck was still settling after the start; the same item once more after a
            # stand
            self.start_retry_used = True
            self.start_retry = {"item_id": c["item_id"], "stand_since": None}
            self.current = None
            self.phase = "start_retry"
            self.early.clear()
            return {}
        if item and reason in UNSURE_REJECTS:
            item["uncertain"] = True
            if item.get("kind") == "auto":  # what may have gone out, kept on the mark (Codex CT-2 r1)
                item["uncertain_pallet_id"] = c.get("pallet_id")
                item["uncertain_destination_id"] = c.get("destination_id")
        self.current, self.phase = None, "waiting"
        self.early.clear()
        return self.stop(f"거부: {reason}", now_ms)

    def on_pick(self, key, msg, now_ms, targets_revision) -> dict:
        """CT-2: the vehicle's answer to a pick_request, matched by its key (queue.js onPick)."""
        pk = self.pick
        if (not pk or self.phase != "picking" or key["epoch"] != pk["epoch"] or key["item_id"] != pk["item_id"]
                or key["request_id"] != pk["request_id"] or (msg or {}).get("request_id") != pk["request_id"]):
            return {}
        # the deadline counts from the request, whether the timer or the pick comes first (Codex CT-2 r1)
        if now_ms - pk["sent_at"] > self.pick_deadline_ms:
            return self.stop("자동 선택 응답 없음", now_ms)
        reason = msg.get("reason") or "unknown"
        self.picks.append({"now_ms": now_ms, "request_id": pk["request_id"], "reason": reason, "pick": msg.get("pick"),
                           "targets_revision": msg.get("targets_revision"), "plan": msg.get("plan"),
                           "asked_ms": now_ms - pk["sent_at"]})
        if reason == "ok" and msg.get("pick"):
            if msg.get("targets_revision") != targets_revision:
                return self._re_ask(now_ms)
            self.held = {"pallet_id": msg["pick"]["pallet_id"], "destination_id": msg["pick"]["destination_id"],
                         "targets_revision": msg["targets_revision"], "plan": msg.get("plan"),
                         "missions": msg.get("missions")}
            self.last_pick, self.last_pick_blocked = dict(self.held), None
            pk["none"] = False  # a pick computed again for a resend supersedes an earlier one
            self.phase = "picked"  # the request stays tracked until its ack
            return {}
        if reason == "none_remaining":
            if pk.get("acked"):
                return self._end_auto(pk["item_id"], now_ms)
            pk["none"] = True  # a refusal may still come: the auto item ends with the ack (Codex CT-2 r2)
            return {}
        self.last_pick_blocked, self.last_pick = msg.get("blocked"), None
        return self.stop(f"자동 선택 실패: {reason}", now_ms)

    def drain_forget(self) -> list:
        out, self.to_forget = self.to_forget, []
        return out

    def tick(self, now_ms) -> dict:
        waiting_ack = self.phase == "picking" or (self.phase == "picked" and not (self.pick or {}).get("acked"))
        if self.run == "running" and waiting_ack and now_ms - self.pick["sent_at"] > self.pick_deadline_ms:
            return self.stop("자동 선택 응답 없음", now_ms)
        ref = self.last_state_at if self.last_state_at is not None else self.wait_since
        if self.run == "running" and ref is not None and now_ms - ref > SILENCE_MS:
            return self.stop("상태 수신 끊김", now_ms)
        if self.run == "running" and self._feedback_lost(now_ms):
            return self.stop("차량 피드백 없음", now_ms)
        return {}

    def next(self, ctx, now_ms, new_id):
        if self.run != "running" or self.current or self.phase == "picking":
            return None
        front = self.items[0] if self.items else None
        if not front or front.get("uncertain"):
            return None
        st = ctx.get("state")
        if not ctx.get("connected") or not ctx.get("welcomed") or not ctx.get("has_lease") or not st \
                or not ctx.get("targets"):
            return None
        if self.last_state_at is None or now_ms - self.last_state_at > SILENCE_MS:
            return None
        if st.get("mode") != "PAUSED" or st.get("stopped_confirmed") is not True or st.get("feedback_ok") is False:
            return None
        if st.get("mission") and st["mission"].get("status") in ACTIVE:
            return None
        if self.last_result_at is not None and now_ms - self.last_result_at < SETTLE_MS:
            return None
        if self.phase == "start_retry":
            if front["item_id"] != self.start_retry["item_id"]:
                self.stop("시작 직후 재제출 취소: 맨 앞 항목이 바뀜", now_ms)
                return None
            since = self.start_retry["stand_since"]
            if since is None or now_ms - since < START_RETRY_STAND_MS:
                return None
            self.phase = "waiting"
            self.start_retry = None
        if front.get("kind") == "auto":
            return self._next_auto(front, ctx, now_ms, new_id)
        r = resolve_item(front, ctx["targets"], st)
        if not r["ok"]:
            self.stop("아직 실행할 수 없는 항목(자동 선택)" if r["reason"] == "unsupported_kind"
                      else f"대상 확인 실패: {r['reason']}", now_ms)
            return None
        return self._submit(front, r["pallet_id"], r["destination_id"], ctx, new_id)

    def _submit(self, front, pallet_id, destination_id, ctx, new_id):
        submit = {"item_id": front["item_id"], "request_id": new_id(), "pallet_id": pallet_id,
                  "destination_id": destination_id, "targets_revision": ctx["targets"]["targets_revision"]}
        self.current = {"item_id": front["item_id"], "request_id": submit["request_id"], "mission_id": None,
                        "epoch": self.epoch, "pallet_id": pallet_id, "destination_id": destination_id}
        self.phase = "submitting"
        self.early.clear()
        self.submits_this_epoch += 1
        return {"submit": submit}

    def _next_auto(self, front, ctx, now_ms, new_id):
        """The auto item (CT-2): ask for a pick, or submit the pick held -- the submit conditions were just checked."""
        if ctx.get("next_pick") is not True:
            self.stop("차량이 자동 선택을 지원하지 않습니다", now_ms)
            return None
        if self.phase == "picked":
            if not (self.pick or {}).get("acked"):
                return None  # a refusal may still come for this request (Codex CT-2 r1)
            h = self.held
            if h["targets_revision"] == ctx["targets"]["targets_revision"]:
                r = resolve_item({"kind": "fixed", "pallet_id": h["pallet_id"], "destination_id": h["destination_id"]},
                                 ctx["targets"], ctx.get("state"))
                if not r["ok"]:
                    self.stop(f"대상 확인 실패: {r['reason']}", now_ms)
                    return None
                self.held, self.pick, self.pick_asks = None, None, 0
                return self._submit(front, r["pallet_id"], r["destination_id"], ctx, new_id)
            if self.pick_asks >= 2:
                self.stop("대상 목록이 계속 바뀜", now_ms)
                return None
            self.held = None  # the list changed under the pick: the one re-ask
            self.pick = None  # acked: nothing to forget
        request_id = new_id()
        self.pick = {"item_id": front["item_id"], "request_id": request_id, "epoch": self.epoch, "sent_at": now_ms}
        self.pick_asks += 1
        self.phase = "picking"
        return {"pick_request": {"item_id": front["item_id"], "request_id": request_id}}

    def _end_auto(self, auto_id, now_ms=None) -> dict:
        """None remaining, acked: the auto item is done; the next item goes on, or the queue is done."""
        self._drop_pick(False)
        self.items = [x for x in self.items if x["item_id"] != auto_id]
        self.last_pick_blocked, self.last_pick = None, None
        if not self.items and self.run == "running":
            return self.stop("완료", now_ms)
        return {}

    def _re_ask(self, now_ms=None) -> dict:
        if self.pick_asks >= 2:
            return self.stop("대상 목록이 계속 바뀜", now_ms)
        if self.pick and not self.pick.get("acked"):
            self.to_forget.append(self.pick["request_id"])  # the old one stops resending
        self.pick, self.held, self.phase = None, None, "waiting"
        return {}

    def _drop_pick(self, queue_forget=True) -> None:
        if queue_forget and self.pick and not self.pick.get("acked"):
            self.to_forget.append(self.pick["request_id"])
        if self.phase in ("picking", "picked"):
            self.phase = "waiting"
        self.pick, self.held, self.pick_asks = None, None, 0

    # --- internals --------------------------------------------------------------------------------------------
    def _feedback_lost(self, now_ms) -> bool:
        return self.feedback_lost_since is not None and now_ms - self.feedback_lost_since >= FEEDBACK_LOST_MS

    def _is_ours(self, mission) -> bool:
        c = self.current
        if not c:
            return False
        if c.get("mission_id"):
            return mission.get("mission_id") == c["mission_id"]
        if self.phase != "submitting" or mission.get("pallet_id") != c["pallet_id"] \
                or mission.get("destination_id") != c["destination_id"]:
            return False
        c.setdefault("seen", mission.get("mission_id"))
        return mission.get("mission_id") == c["seen"]

    def _drop_current_as_uncertain(self) -> None:
        item = self.current and next((x for x in self.items if x["item_id"] == self.current["item_id"]), None)
        if item:
            item["uncertain"] = True
            if self.current.get("mission_id"):
                item["maybe"] = {"mission_id": self.current["mission_id"], "epoch": self.current["epoch"]}
            if item.get("kind") == "auto":  # the mark is on this one submit (CT-2 C4)
                item["uncertain_pallet_id"] = self.current.get("pallet_id")
                item["uncertain_destination_id"] = self.current.get("destination_id")
        self.current, self.phase = None, "waiting"
        self.early.clear()

    def _forget_last_pick(self) -> None:
        self.last_pick, self.last_pick_blocked = None, None

    @staticmethod
    def _clear_uncertain(it) -> None:
        it["uncertain"] = False
        for key in ("maybe", "uncertain_pallet_id", "uncertain_destination_id"):
            it.pop(key, None)

    def _record(self, item_id, result) -> None:
        """A result into the finished list with the pallet and destination it moved; a fixed item leaves the queue,
        an auto item stays for the next pick with any uncertain mark on that submit cleared (CT-2 C3)."""
        self.applied.add(result["mission_id"])
        item = next((x for x in self.items if x["item_id"] == item_id), None)
        cur = self.current if (self.current or {}).get("item_id") == item_id else None

        def first(*values):
            return next((v for v in values if v is not None), None)

        pallet_id = first((cur or {}).get("pallet_id"), (item or {}).get("uncertain_pallet_id"),
                          (item or {}).get("pallet_id"), result.get("pallet_id"))
        destination_id = first((cur or {}).get("destination_id"), (item or {}).get("uncertain_destination_id"),
                               (item or {}).get("destination_id"), result.get("destination_id"))
        outcome = result.get("outcome")
        if item is not None and item.get("kind") == "auto":
            self._clear_uncertain(item)
            if outcome == "done":
                item["moved"] = item.get("moved", 0) + 1  # the progress shown on the row
        else:
            self.items = [x for x in self.items if x["item_id"] != item_id]
        self.finished.insert(0, {"item": None if item is None else dict(item), "outcome": outcome,
                                 "reason": result.get("reason"), "mission_id": result["mission_id"],
                                 "pallet_id": pallet_id, "destination_id": destination_id})
        self.finished = self.finished[:FINISHED_KEEP]
        if outcome == "done":
            self.counts["done"] += 1
        elif outcome == "cancelled":
            self.counts["cancelled"] += 1
        else:
            self.counts["failed"] += 1

    def _finish(self, result, now_ms) -> None:
        self._record(self.current["item_id"], result)
        outcome = result.get("outcome")
        self.current, self.phase = None, "waiting"
        self.last_result_at = now_ms
        if self.run == "running":
            if outcome == "done":
                if not self.items:
                    self.stop("완료", now_ms)
            elif outcome in ("failed", "recovery_required"):
                self.stop("실패" + (f": {result['reason']}" if result.get("reason") else ""), now_ms)
            elif outcome == "cancelled":
                self.stop("취소", now_ms)
            else:
                self.stop(str(outcome), now_ms)


__all__ = ["MissionQueue", "PICK_DEADLINE_MS", "START_RETRY_STAND_MS", "new_item", "resolve_item"]
