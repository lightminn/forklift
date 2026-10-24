# CT-2 콘솔 — 대기열의 자동 항목 "남은 팔레트 전부" 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

작성일: 2026-10-24 · 상태: 초안(차량 쪽 인터페이스 확정 대기) · 근거: [CT-2 다음 팔레트·하역 칸 계획](2026-10-24-ct2-next-pick.md)(브랜치 `feat/ct2-ordering`, 연속 이송 세션) D3·D4, [미션 대기열 계획](2026-10-22-console-mission-queue.md) D1(`auto` 예약)·D3–D7, 총괄 지시 2026-10-24

**Goal:** 대기열에 "남은 팔레트 전부" 항목 하나를 넣으면, 차량이 추천한 다음 팔레트·하역 칸을 확정 ID 로 하나씩 내고, 남은 팔레트가 없을 때 끝난다. 이상 시 멈춤(사용자 결정 2026-10-22)은 그대로다.

**Architecture:** 판정은 차량 쪽(`next_pick.recommend`, 계약 `pick_request`/`pick`). 콘솔은 추천을 받아 지금과 같은 `mission_submit` 으로 낸다. 대기열 순수 모듈(`queue.js`)에 추천 대기 단계를 더하고, 연결부(`queuewire.js`)가 요청·수신을 잇는다. 화면은 대기열 칸에 버튼 하나와 단계 표시를 더한다.

## 1. 설계

**C1 항목.** `{kind: "auto", scope: "remaining", label: "남은 팔레트 전부"}`. 확정 ID 는 비워 둔다. 대기열에 하나만 둔다(이미 있으면 버튼이 꺼진다). 남은 팔레트가 없다는 추천(`none_remaining`)이 올 때까지 맨 앞에 남는다.

**C2 단계.** 맨 앞 항목의 단계에 둘을 더한다: `waiting` → **`picking`**(요청이 나감) → **`picked`**(추천을 받음) → `submitting` → `in_mission`.
- `next()` 는 맨 앞이 `auto` 이고 지금의 제출 조건 1–5 가 참이면 `{pick_request: {item_id, request_id}}` 를 돌려주고 `picking` 으로 간다. 열쇠는 `{epoch, item_id, request_id}`.
- 요청을 보낸 뒤 기한(`PICK_DEADLINE_MS`, 기본 5 s, 벽시계 — 차량 쪽 V1·V3 측정으로 다시 정함) 안에 열쇠가 맞는 `pick` 이 없으면 멈춘다("자동 선택 응답 없음").
- `ack` 는 재전송만 끝낸다(단계는 그대로). `reject` 는 멈춘다("자동 선택 거부: 사유"). `pick` 이 `ack` 보다 먼저 와도 받는다.
- `pick` 의 `reason` 이 `ok` 이고 `targets_revision` 이 지금 목록과 같으면 `picked` 로 둔다. 제출은 다음 `next()` 에서 **제출 조건 1–5 를 다시 확인한 뒤** 낸다(조건이 거짓이면 고정 항목처럼 기다린다). `picked` 동안 목록 revision 이 바뀌거나, 받은 revision 이 처음부터 다르면 한 번 다시 묻는다. 두 번째도 다르면 멈춘다("대상 목록이 계속 바뀜").
- `none_remaining`: 자동 항목을 '완료'로 빼고 다음 항목으로. 대기열이 비면 멈춤 사유 '완료'.
- `no_free_destination`·`all_blocked`·`error:<예외 종류>`(`pick` 은 null): 멈춘다("자동 선택 실패: 사유"), 항목은 맨 앞. 사유 어휘는 계약이 고정한 다섯(`ok`·`none_remaining`·`no_free_destination`·`all_blocked`·`error:*`)이고, 모르는 사유도 멈춤이다.
- 멈춤(STOP·끊김·권한 상실·새 epoch·운용자 멈춤·항목 삭제·피드백 상실 등)은 `picking`·`picked` 에서 요청을 잊고(`session.forget`) `waiting` 으로 돌아간다. **불확실 표시를 만들지 않는다**(아직 낸 것이 없다).
- `submitting` 부터는 고정 항목과 같다: `current` 에 추천받은 팔레트·칸, `#isOurs`, 결과 처리, 불확실 규칙.

**C3 결과.** `done`: 끝낸 목록·수에 실제로 낸 팔레트·칸으로 적고, 자동 항목은 맨 앞에 남아 다음 추천으로. `failed`·`recovery_required`·`cancelled`: 지금처럼 멈춤, 자동 항목은 맨 앞. 미션 결과는 결과 기록부에 있으므로 실적 요약은 바뀌지 않는다.

**C4 불확실(제출 뒤).** 자동 항목의 불확실은 **그 한 번의 제출**에 붙인다(`uncertain_submit: {request_id, pallet_id, destination_id, maybe}`). 재개는 불확실이 풀릴 때까지 막힌다. 화면의 선택지는 '확인하고 계속'(불확실만 지움 — 다음 추천은 차량의 실제 목록으로 계산되므로 같은 팔레트를 두 번 옮기지 않는다)과 '자동 항목 지우기'. 같은 세션에서 그 미션의 결과가 오면 끝낸 목록에 적고 불확실을 지운다(고정 항목의 `maybe` 와 같다).

**C5 저장(D7).** `picking`·`picked` 는 저장하지 않는다(다시 열면 추천부터). 진행 중 미션은 고정 항목처럼 C4 의 불확실로 되살린다.

**C6 화면.** 대기열 칸에 '남은 팔레트 전부' 버튼(welcome `capabilities.next_pick === true` 일 때만, 하나만). 항목 줄은 라벨 + 지금 추천(`P-2 → D-1`)·남은 계획 수(`plan` 길이). 단계 문구 '자동 선택 중'·'자동 선택: P-2 → D-1'. 멈춘 이유는 `queueStopText` 에 자동 선택 사유 문구표를 더해 한국어로, `blocked`(`{팔레트·칸 ID: ["pickup:<id>"·"delivery:<id>"·"occupied:<id>"·"props"·"no_pose"·"no_size"…]}`)는 한국어로 풀어 상태 행 설명란과 대기열 칸에 요약.

**C7 가짜 차량.** 화면 시험용 `pick_request`: 첫 `available` 팔레트와 확인된 자리가 아닌 첫 칸을 고르는 **장난감 규칙**(실제 규칙이 아님), 주입 `/mock/inject/pick_<사유>` 로 다음 한 번의 추천 사유를 바꿀 수 있다.

**C8 파이썬 판 맞춤.** `tools/mission_queue.py` 의 자동 동작은 연속 이송 세션이 구현한다. JS 구현 뒤 `tests/unit/tools/test_mission_queue_port.py` 에 같은 사건열(자동 항목·추천·거부·기한·revision)을 더한다.

## 2. 열린 점(인터페이스 확정 때 정함)

- 연속 이송 세션이 제안 1–6 을 반영(2026-10-24): 제출 전 조건 재확인(`picked`), 계산 실패도 `pick`(`error:*`), 사유 어휘·`blocked` 형식 고정, 기한 `PICK_DEADLINE_MS`(기본 5 s), `capabilities.next_pick`, `pick` 이 `ack` 보다 먼저 와도 받음. 그 계획의 Codex 재검토 수렴 뒤 최종 인터페이스를 받아 맞춘다.

## 3. Task (인터페이스 확정 뒤)

1. `queue.js` — C1–C5, 시험 먼저(`queue.test.mjs`).
2. `queuewire.js`·`protocol.js` 수신 — 요청 수 세는 시험(`queuewire.test.mjs`).
3. 화면 C6, 가짜 차량 C7 + pytest, Playwright.
4. 맞춤 시험 C8, ws1 pytest, Codex 수렴, 차량 쪽 머지 뒤 Isaac live 확인, PR.

## 4. 자체 검토

(인터페이스 확정 뒤 독립 검토 3 회를 여기에 적는다.)
