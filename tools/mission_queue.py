"""The console's mission queue (console/web/js/queue.js, docs/plans/2026-10-22-console-mission-queue.md D1-D6) in
Python, for the live probe (CT-1 PR 4: continuous runs under the queue's own policy -- ws1 has no Node). A line for
line port of the decisions the probe needs: fixed items, resume/stop, the welcome, disconnect and lease events, the
states (silence, feedback loss, STOPPED / TELEOP, a safety preempt, a mission not ours, paused / cancelling), the
results, the submit answers, the timer and the next submit. Times are milliseconds, as in the JS; the stop reasons
are the JS strings, so tests/unit/tools/test_mission_queue_port.py can run both on the same events."""

from __future__ import annotations

SILENCE_MS = 2000
SETTLE_MS = 1000
FEEDBACK_LOST_MS = 500
FINISHED_KEEP = 50
PREEMPT = {"preempt", "safety_preempt"}
UNSURE_REJECTS = {"busy", "stale"}
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
        self.last_state_at = None
        self.wait_since = None
        self.feedback_lost_since = None
        self.last_result_at = None
        self.applied: set = set()
        self.early: dict = {}
        self.stops: list = []  # (reason, now_ms): every stop, for the probe's report

    # --- operator ---------------------------------------------------------------------------------------------
    def add(self, item) -> bool:
        if not item or item.get("kind") not in KINDS:
            return False
        self.items.append(dict(item))
        return True

    def resume(self, now_ms) -> bool:
        if not self.items or any(it.get("uncertain") for it in self.items):
            return False
        self.run, self.stop_reason, self.wait_since = "running", None, now_ms
        return True

    def stop(self, reason, now_ms=None) -> dict:
        forget = self.current.get("request_id") if self.phase == "submitting" and self.current else None
        if forget is not None:
            self._drop_current_as_uncertain()
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
        if item and reason in UNSURE_REJECTS:
            item["uncertain"] = True
        self.current, self.phase = None, "waiting"
        self.early.clear()
        return self.stop(f"거부: {reason}", now_ms)

    def tick(self, now_ms) -> dict:
        ref = self.last_state_at if self.last_state_at is not None else self.wait_since
        if self.run == "running" and ref is not None and now_ms - ref > SILENCE_MS:
            return self.stop("상태 수신 끊김", now_ms)
        if self.run == "running" and self._feedback_lost(now_ms):
            return self.stop("차량 피드백 없음", now_ms)
        return {}

    def next(self, ctx, now_ms, new_id):
        if self.run != "running" or self.current:
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
        r = resolve_item(front, ctx["targets"], st)
        if not r["ok"]:
            self.stop("아직 실행할 수 없는 항목(자동 선택)" if r["reason"] == "unsupported_kind"
                      else f"대상 확인 실패: {r['reason']}", now_ms)
            return None
        submit = {"item_id": front["item_id"], "request_id": new_id(), "pallet_id": r["pallet_id"],
                  "destination_id": r["destination_id"], "targets_revision": ctx["targets"]["targets_revision"]}
        self.current = {"item_id": front["item_id"], "request_id": submit["request_id"], "mission_id": None,
                        "epoch": self.epoch, "pallet_id": r["pallet_id"], "destination_id": r["destination_id"]}
        self.phase = "submitting"
        self.early.clear()
        return {"submit": submit}

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
        self.current, self.phase = None, "waiting"
        self.early.clear()

    def _record(self, item_id, result) -> None:
        self.applied.add(result["mission_id"])
        item = next((x for x in self.items if x["item_id"] == item_id), None)
        self.items = [x for x in self.items if x["item_id"] != item_id]
        outcome = result.get("outcome")
        self.finished.insert(0, {"item": item, "outcome": outcome, "reason": result.get("reason"),
                                 "mission_id": result["mission_id"]})
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


__all__ = ["MissionQueue", "new_item", "resolve_item"]
