# 운용 콘솔 — 저장된 운반 버튼과 미션 대기열 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

작성일: 2026-10-22 · 상태: 구현됨(계획 4판, PR feat/console-mission-queue; 2026-10-25 D5 예외 개정, feat/queue-start-resubmit) · 근거: [상용 FMS 사례 조사](../design/2026-10-22-fms-case-survey.md) 4절 1순위(최소안), 사용자 결정 2026-10-22(총괄 경유)

**Goal:** 자주 하는 운반을 버튼 하나로 대기열에 넣고, 대기열의 운반을 차례로 내되 **어떤 이상이 있어도 다음 미션을 자동으로 내지 않는** 콘솔 기능을 만든다.

**Architecture:** 차량 쪽(실행기·중재기·console/1 계약)은 바꾸지 않는다. 대기열은 브라우저 콘솔이 들고, 다음 항목을 기존 `mission_submit {pallet_id, destination_id, targets_revision}` 으로 하나씩 낸다. 판단 규칙은 DOM 없는 순수 모듈(`console/web/js/queue.js`)에 두고, 화면 연결부(`main.js`)는 사건을 넘기고 대기열이 내라고 할 때만 보낸다. 대기열의 기본은 **멈춤**이고, "실행 중"은 운용자가 재개를 누른 뒤 이상이 하나도 없을 때뿐이다.

**Tech Stack:** 빌드 없는 ES 모듈(`console/web/js/`), `node --test "console/web/test/*.test.mjs"`(Node ≥ 18), 가짜 차량(`console/mock_vehicle`)과 Playwright 화면 확인.

## 1. 사용자 결정(2026-10-22)과 원칙

- 미션 실패 → 대기열 멈춤, 운용자가 확인 후 재개.
- 운용자 취소 → 대기열 멈춤.
- 조종 권한(lease) 상실·콘솔 연결 끊김 → 진행 중 미션은 계속(기존 결정 "AUTO 중 끊김 계속"), 다음 미션은 제출하지 않고 대기열 멈춤.
- 재접속 → 대기열은 멈춘 채, 운용자가 직접 재개.
- **원칙: 어떤 이상이 있어도 다음 미션을 자동으로 내지 않는다.** 모든 "멈춤"은 운용자의 재개로만 풀린다. 차량 미션의 재개(`mission_resume`)와 대기열의 재개는 별개다.

## 2. 연속 이송 세션과의 조율(2026-10-22, sw-연속이송-재관측)

- 차량은 지금처럼 미션 하나(`pallet_id`, `destination_id`)를 실행하고, **다음 팔레트·하역 칸은 콘솔 대기열이 정한다**. 하역 칸은 지금의 destinations(`D-0`, `D-1` …) 그대로다. 칸이 찼는지는 targets 에 없고 차량이 제출 때 `destination_occupied` 로 거부한다.
- 대기열 항목은 확정 ID 외에 "제출 순간에 정하기"(`auto`)를 담을 수 있는 모양으로 둔다. 이번 구현은 `fixed` 만 실행하고 `auto` 는 형식만 예약한다(D1). 선택 규칙과 콘솔이 그것을 부르는 방식(JS 판 또는 차량이 추천을 내는 계약 추가), 빈 칸을 찾지 못했을 때의 처리는 연속 이송 계획서에서 정한다 — 계약을 바꾸면 그때 합의한다.
- 완료·실패는 미션 하나 단위(결과 기록부 `missions`, 계약 5.2)로 충분하다. 묶음 결과는 콘솔이 자기 항목으로 계산한다.

## 3. 설계

**D1 항목.** `{item_id, kind, pallet_id, destination_id, label, added_unix}`. `kind` 는 `"fixed"`(확정 ID)와 예약된 `"auto"`. 콘솔은 kind 별 **해석기**(`resolve(item, targets, state) → {ok, pallet_id, destination_id} | {ok:false, reason}`)를 거쳐서만 낸다. `fixed` 해석기는 제출 직전 **최신 targets** 로 확인한다: 팔레트가 있고 고를 수 있는가(`wizard.selectable` 규칙 — available·delivered, 또는 `carried` 이면서 상태의 `load` 가 `loaded` 로 그 팔레트를 지목; 고를 수 있다는 것이 차량 수락을 보장하지는 않는다), 하역 칸이 있는가. **해석 실패와 모르는 kind 는 대기열을 멈춘다**(그 항목은 맨 앞에 그대로).

**D2 상태.** 실행 상태는 `stopped`(이유 포함) 또는 `running`. `running` 안에서 맨 앞 항목은 `waiting` → `submitting` → `in_mission` 으로 간다. 처음 상태와 페이지를 다시 연 뒤의 상태는 언제나 `stopped` 다. 항목에는 `uncertain`(제출했는지·결과가 무엇인지 모름) 표시가 붙을 수 있고, `uncertain` 항목이 맨 앞에 있으면 재개되지 않는다 — 운용자가 그 항목을 지우거나 "다시 내기"로 고른 뒤에만.

**D3 제출 조건.** `running` 이고 `waiting` 일 때, 아래가 모두 참인 순간에만 낸다. 거짓이면 기다린다(멈춤으로 바꾸는 것은 D4–D6 의 사건뿐).
1. 제어 연결이 열려 있고 이 연결의 welcome 을 받았다. 최근 **2 s 안에** 상태를 받았다(열린 채 끊긴 연결 대비 — 2 s 를 넘기면 D6 의 멈춤). 공백 판정은 두 곳에서 한다: 타이머(새 상태가 아예 오지 않을 때)와 **상태를 받는 순간**(직전 상태와의 간격이 2 s 를 넘었으면 멈춤을 먼저 확정한 뒤에 수신 시각을 갱신) — 그래야 공백 뒤에 온 상태가 타이머보다 먼저 공백을 지우지 못한다.
2. 이 콘솔이 조종 권한을 쥐었다.
3. 상태의 미션이 없거나 끝난 상태다(`results.ACTIVE` 밖 — `cancelling` 포함 목록. `wizard`·가짜 차량의 목록은 쓰지 않는다).
4. 모드가 `PAUSED`(대기)이고 `stopped_confirmed` 가 참이며 `feedback_ok` 가 거짓이 아니다. (`feedback_ok` 가 거짓인 상태를 실행 중에 보면 그것은 기다림이 아니라 D4 의 멈춤 사건이다 — 중재기는 `PAUSED` 에서 피드백 상실을 선점하지 않으므로, 기다리기만 하면 저절로 회복된 뒤 다음 항목이 나간다.)
5. 직전 항목의 결과를 받은 뒤 **1.0 s** 가 지났다(cycle 프로브의 정지 확인 뒤 1 s 와 같은 안정 대기).
6. 해석기가 성공했다(실패는 멈춤, D1).
판단은 애니메이션 프레임이 아니라 **250 ms 타이머와 사건(상태·결과·응답)** 으로 돈다 — 탭이 뒤로 가면 애니메이션 프레임은 멈춘다. 브라우저가 뒤의 탭 타이머를 늦춰도 늦게 낼 뿐 이상을 지나치지 않는다(멈춤 사건은 메시지 처리에서 곧바로 반영).

**D4 결과와 운용자 행동.**
- `in_mission` 항목은 그 `mission_id` 의 결과가 결과 기록부(`missions` 또는 상태의 `last_result`; 둘 다 오면 처음 본 것을 쓰고 다음은 무시)에 나올 때 끝난다.
  - `done` → 끝낸 항목에 남기고 다음 항목(`waiting`). 비면 `stopped("완료")`.
  - `failed`·`recovery_required` → `stopped("실패")`, 항목은 끝낸 항목에 실패로.
  - `cancelled` → `stopped("취소")`.
  - 그 밖 → `stopped(값)`.
- **멈춤 사건(결과를 기다리지 않고 그 순간 멈춘다):**
  - 운용자가 이 화면에서 취소·일시정지·STOP 을 누름(대기열이 낸 미션이든 아니든). 완료 직후의 취소가 차량에서 `not_running` 으로 끝나도 대기열은 이미 멈춰 있다.
  - 상태의 모드가 `STOPPED`·`TELEOP` 이 되거나, 정지 종류(`stop_kind`)가 안전 선점(live `preempt`, 가짜 차량 `safety_preempt`)임 — 항목 사이의 STOP → 재무장이 대기열을 다시 돌리지 않게. 정상 대기(`PAUSED`)의 `controlled`(live)·`controlled_decel`(가짜 차량)은 미션이 끝난 뒤의 평상 상태라 멈춤 사건이 아니다.
  - `in_mission` 인 대기열 미션이 상태에서 `paused`·`cancelling` 이 됨(다른 콘솔이 일시정지·취소한 경우 포함).
  - 상태의 `feedback_ok` 가 **0.5 초 이상 이어서** 거짓(차량 피드백 상실, Codex 3회차; 2026-10-22 Isaac 실시간 운용에서 개정 — 실행기는 0.1 초마다 상태를 보내고 피드백 나이 한계도 0.1 초라, 정상 AUTO 중에도 틱이 길어진 순간 거짓 상태가 한 번씩 섞여 와 대기열이 미션마다 멈췄다. 상태·경보 패널이 경보를 여는 0.5 초와 같은 값. 제출은 여전히 그 순간 `feedback_ok` 가 거짓이 아니어야 한다 — D3).
  - 운용자가 작업 지정 창으로 직접 미션을 냄 — 그 전에 대기열을 멈춘다.
  - 상태에 대기열이 내지 않은 진행 중 미션이 보임(`mission_id` 가 대기열의 것이 아님) — 다른 콘솔이나 수동 지시.
- 대기열이 낸 미션이 아닌 결과(과거 미션·다른 콘솔의 미션)는 끝낸 항목에 넣지 않는다. 그러나 위 사건이 이미 대기열을 멈춘다.

**D5 제출 응답.**
- `ack` → `in_mission`(받은 `mission_id`).
- **모든 `reject` → `stopped("거부: <설명>")`**, 항목은 맨 앞에 그대로. 거부가 "이 요청은 실행되지 않았다"를 보장하는 경우만 `uncertain` 을 붙이지 않는다. live 의 채널 포화 `busy` 는 요청 원장에 닿기 전에 돌아오므로 먼저 들어간 같은 요청이 뒤에 받아들여질 수 있고(Codex 가 `Channels`·`Link`·중재기로 재현), 원장이 요청을 잊은 `stale` 도 실행 여부를 모른다 — 이 둘(가짜 차량의 `busy` 포함)은 **`uncertain`** 으로 남긴다. 자동으로 다시 내지 않는다(계약 1절: 거부 뒤 새 요청을 자동으로 만들지 않고 운용자에게 다시 묻는다). 일시적일 수 있는 `not_stopped`·`stale_targets` 는 D3 의 4·5 조건(정지 확인, 결과 뒤 1 s)으로 대부분 피한다. 남은 경우는 운용자가 한 번 더 재개한다.
- **유일한 예외(사용자 결정 2026-10-25).** 이번 세션(`session_epoch`)에서 대기열이 낸 **첫 제출**이 `not_stopped` 로 거부되면 멈추지 않고 `start_retry` 로 둔다. 시작 직후 아직 흔들리며 서는 차 때문이다.
  - 그 뒤 상태가 PAUSED·`stopped_confirmed`·피드백 정상으로 이 화면의 시계 기준 1 s 이상 끊김 없이 이어지면, **같은 항목을 한 번만** 새 `request_id` 로 다시 낸다.
  - 화면에는 "시작 직후 서 있음 확인 뒤 1회 재제출"을 보인다.
  - 다음은 모두 지금처럼 멈춤이다: 두 번째 `not_stopped`, 다른 거부 사유, 세션의 두 번째 이후 제출의 거부, 기다리는 동안의 STOP·텔레옵·선점·피드백 상실·상태 끊김, 맨 앞 항목이 바뀜(`시작 직후 재제출 취소: 맨 앞 항목이 바뀜`).
  - 재제출은 세션마다 한 번이다. 멈췄다 재개해도 다시 생기지 않고, 새 세션에서만 다시 생긴다. 화면을 다시 열면 기다림은 남지 않는다(D7).
  - 차량 쪽 파이썬 포트(`tools/mission_queue.py`)도 같은 규칙을 따른다. 같은 사건을 queue.js 와 포트에 넣어 사건마다 상태를 맞추는 시험(`tests/unit/tools/test_mission_queue_port.py` 의 `start_retry_*` 8 경우)으로 고정한다. 맨 앞 항목이 바뀌는 경우만 포트 단독 시험이다(대기열 조작으로는 순서를 바꿀 수 없다).
  - 실시간 탐침(`tools/live_console_probe.py`)의 `--queue-resume-not-stopped`(not_stopped 뒤 운용자 재개 1회)는 이 규칙으로 대체했다. 같은 거부 뒤 두 번 다시 낼 수 있어 이제 그 옵션은 거부한다. 그 옵션으로 돈 기록(CT-2 V3 seed 2 재실행, 남은 실패 V4)은 그 앞 스냅숏의 탐침으로 돌았다.

**D6 끊김·권한·세션·요청 정리.**
- 제어 연결이 닫히거나 2 s 동안 상태가 오지 않으면 → `stopped("연결 끊김")`. 차량의 진행 중 미션은 그대로 둔다.
- 조종 권한을 잃으면 → `stopped("조종 권한 상실")`.
- 새 `session_epoch` 의 welcome → `stopped("새 세션")`. 그때 `submitting`·`in_mission` 이던 항목은 **`uncertain`** 으로 맨 앞에 남긴다(새 세션에서 그 결과는 오지 않는다 — 결과 기록부도 epoch 마다 빈다, 계약 5.2).
- 같은 epoch 로 다시 붙으면 → 멈춘 채로 둔다. `in_mission` 의 `mission_id` 결과가 오면 기록한다(다음은 내지 않음).
- **요청 정리.** 대기열은 제출할 때 `request_id` 를 먼저 정하고(`protocol.request` 에 받은 ID 를 쓰게 하는 선택 인자를 더함), 저장(D7)한 **다음에** 보낸다. `submitting` 중에 **어떤 멈춤이든** 생기면 그 요청을 재전송 목록에서 뺀다(`session.forget(request_id)`, 이 계획에서 추가 — 그 뒤 온 응답은 버린다). 이미 나간 요청은 되돌릴 수 없으므로 그 항목은 `uncertain` 이 된다. `forget` 은 "멈춘 뒤 이 화면이 더 보내지 않게" 하는 장치다 — 실제로는 재접속 뒤 옛 요청이 lease·연결 토큰이 낡아 거부되지만(가짜 차량 실측, Codex 확인), 그 거부에 기대지 않는다.
- 응답은 `{epoch, item_id, request_id}` 가 지금 `submitting` 과 같을 때만 반영한다.

**D7 다시 열 때(새로고침)와 여러 탭.**
- 대기열(항목·실행 정보)은 **탭마다 따로** `sessionStorage` 의 `console.queue.<vehicle_id>.<client_id>` 에 둔다(`client_id` 는 탭마다 다른 기존 값 — 이 탭에서 연 새 창은 `sessionStorage` 를 물려받을 수 있지만 `gate.chooseClientId` 가 새 id 를 주므로 옛 탭의 대기열 키를 읽지 않는다). 같은 탭의 새로고침에는 살아남고, 다른 탭과는 공유하지 않으며, 탭이나 브라우저를 닫으면 사라진다. 그래서 두 탭이 같은 대기열을 덮어쓰는 일이 없다(권한 인계 중 두 탭이 잠깐 모두 권한을 가진 것으로 보이는 경합 포함 — Codex 2회차). 두 탭이 각자 대기열을 돌려도 차량은 유효한 lease 의 제출만 받고 진행 중 미션이 있으면 거부하므로, 낡은 쪽은 거부로 멈춘다(D5).
- 저장된 운반(D8)은 실행 정보가 아니므로 `localStorage` 의 `console.presets.<vehicle_id>` 에 두어 탭·브라우저를 넘어 남긴다(여러 탭 편집은 마지막 저장이 이긴다 — 실행에는 영향 없음).
- 다시 열면 항목은 되살리되 실행 상태는 언제나 `stopped("콘솔 다시 열림")`. 저장 당시 `submitting` 이던 항목은 **`uncertain`**. `in_mission` 이던 항목은 저장된 epoch 와 지금 epoch 가 같고 지금 결과 기록부에서 그 `mission_id` 를 찾으면 그 결과로 정리하고, 아니면 `uncertain`("결과 확인 불가" — 결과가 최근 100건 밖으로 밀렸거나 새 세션).
- 저장에 실패하면 대기열 칸에 "저장 안 됨 — 새로고침하면 대기열을 잃습니다"를 보이고 이전 저장본을 지우려 시도한다(`removeItem`). 지우기마저 실패하면 이전 저장본이 남을 수 있으나, 되살린 대기열은 멈춘 채 열리고 운용자가 항목을 보고 재개한다 — 이 한계를 화면 문구로 밝힌다.
- 탭을 닫으면 대기열을 잃는 것은 받아들인다: 잃어도 자동으로 내는 일은 없고(원칙), 저장된 운반 버튼으로 다시 넣으면 된다.

**D8 저장된 운반.** `{preset_id, label, pallet_id, destination_id}`. 작업 지정 창 3단계(확인)의 "운반 저장"으로 만든다. 버튼을 누르면 대기열 끝에 `fixed` 항목으로 들어간다(바로 내지 않는다). 지금 targets 에 없는 팔레트·칸을 가리키는 버튼은 흐리게 보이고 이유를 붙인다. 편집은 조종 권한을 쥔 탭에서만.

**D9 화면.** 차량 상세 패널의 "작업 지정" 아래 "대기열" 칸.
- 머리: 상태(실행 중 / 멈춤 — 이유), "재개"(이유를 보인 2단계 확인 — 기존 `ConfirmGate`), "멈춤", "비우기"(2단계 확인).
- 저장된 운반 버튼 줄.
- 항목: 순서, `팔레트 → 하역 칸`, 상태(대기·제출 중·진행 중 `M-0007`·불확실 — 재접속 뒤라면 상태의 진행 중 미션을 옆에 보임), 위/아래/지우기. 제출 중·진행 중 항목은 옮기거나 지울 수 없고, 불확실 항목은 "지우기"·"다시 내기"만 된다.
- 끝낸 항목: 이번 묶음의 완료/실패/취소 수와 최근 몇 건(`reasonText`).
- 작업 지정 창 3단계: "작업 시작" 옆에 "대기열에 넣기"·"운반 저장". 대기열이 실행 중일 때 "작업 시작"을 누르면 대기열을 먼저 멈춘다는 문구를 보인다.
- 대기열이 멈췄을 때는 칸 머리를 경고색으로 강조한다(소리는 내지 않는다 — 브라우저 자동 재생 제한).

**D10 하지 않는 것.** 차량 쪽 대기열, `auto` 항목의 선택, 빈 칸 자동 탐색, 일정(시각 지정), 여러 콘솔 사이 대기열 공유, 권한을 넘긴 다른 기기로 대기열 넘기기(같은 브라우저의 탭 사이만 D7). 실차·Isaac 연속 운용 시험은 이 PR 범위가 아니다(가짜 차량과 브라우저로 확인). 가짜 차량은 실린 팔레트 제출을 거부하고 진행 중 충돌에 `busy` 를 쓰므로, live 의 `mission_active`·적재 제출 경로는 이 PR 의 화면 시험으로 확인한 것이 아니다.

## 4. 파일

- Create: `console/web/js/queue.js` — 순수 모듈: 항목·상태기·해석기·저장 직렬화
- Create: `console/web/test/queue.test.mjs`
- Create: `console/web/js/queuewire.js` — 대기열과 세션을 잇는 작은 연결부(순수, DOM 없음): 사건 → 대기열 입력, 대기열의 제출 지시 → `session.request` + `forget`, 저장 호출. `main.js` 는 이것만 부른다
- Create: `console/web/test/queuewire.test.mjs` — 가짜 세션·가짜 저장소로 **보낸 요청 수**를 센다
- Modify: `console/web/js/protocol.js` — `request(type, fields, {requestId})`, `forget(request_id)`; `console/web/test/protocol.test.mjs` 에 시험
- Modify: `console/web/js/main.js`, `ui.js`, `index.html`, `app.css` — 화면
- Modify: `console/mock_vehicle/core.py` — `inject("fail_running")`(진행 중 미션을 `failed` 로 끝냄)과 그 시험
- Modify: `console/README.md` — 사용법 한 단락

## 5. Task

### Task 1: 대기열 순수 모듈

**Interfaces (Produces):**
- `newItem({kind, pallet_id, destination_id, label}, nowUnix, newId) → item`
- `resolveItem(item, targets, state) → {ok:true, pallet_id, destination_id} | {ok:false, reason}`
- `class MissionQueue` — `items`, `run`, `stopReason`, `phase`, `current` (`{item_id, request_id, mission_id, epoch}`), `finished`(최근 50건), `counts`.
  - 운용자: `add`, `remove`, `move`, `clear`, `resume(nowMs) → bool`(맨 앞이 불확실이면 false), `stop(reason) → {forget?: request_id}`, `resolveUncertain(item_id, "drop"|"resubmit")`, `operatorAction(kind)`(cancel·pause·stop·manual_submit → 멈춤)
  - 입력: `onWelcome(epoch)`, `onDisconnect()`, `onLeaseLost()`, `onState(state, nowMs)`(STOPPED·stop_kind·남의 진행 미션 → 멈춤), `onResult(result, nowMs)`, `onAnswer({epoch, item_id, request_id}, answer)`, `tick(nowMs)`(상태 2 s 공백 → 멈춤)
  - 멈춤을 일으키는 입력은 모두 `{forget?: request_id}` 를 돌려준다(제출 중이었다면)
  - `next(ctx, nowMs, newId) → null | {submit: {item_id, request_id, pallet_id, destination_id, targets_revision}}`
  - `toStorage()`, `static fromStorage(obj, {epoch, results, stateMission})`
- `presetStatus(preset, targets, state) → {usable, reason}`

- [ ] **Step 1: 실패하는 시험** — 아래 각각이 `test`:
  - 새 대기열·`fromStorage` 결과는 `stopped`, `next` 는 `null`
  - `resume` 뒤 D3 조건이 모두 참이면 맨 앞을 내라고 함; 조건 1–5 중 하나만 거짓이면 `null` 이고 실행 상태 유지; 해석 실패(없는 팔레트·고를 수 없음·없는 칸·`auto`)는 `stopped`
  - 결과 뒤 1.0 s 전에는 내지 않음
  - `submitting` 중 다시 내라고 하지 않음
  - 결과 `done` → 다음, 마지막이면 `stopped("완료")`; `failed`·`recovery_required`·`cancelled` → 각 멈춤; 같은 결과를 `last_result` 와 `missions` 로 두 번 받아도 한 번만 반영
  - 모든 거부 → `stopped`, 항목은 맨 앞, 다시 내라고 하지 않음(`busy`·`stale` 은 불확실, 그 밖은 불확실 아님)
  - `operatorAction` 네 가지와 `onState` 의 STOPPED·`stop_kind`·남의 진행 미션·`feedback_ok` 거짓 → 즉시 멈춤(피드백이 저절로 회복돼도 다음을 내지 않음); 이후 `done` 결과가 와도 다음을 내지 않음(완료 직후 취소 반례)
  - 멈춤 입력이 `submitting` 중이면 `{forget: request_id}` 를 돌려주고 그 항목은 불확실; 그 뒤 온 그 요청의 `ack` 는 무시
  - 새 epoch welcome → 멈춤, 진행 중·제출 중 항목은 불확실; 같은 epoch welcome → 멈춘 채, 그 미션 결과는 기록되지만 다음을 내지 않음
  - `tick` 이 상태 2 s 공백을 보면 멈춤; 공백 뒤 상태가 타이머보다 먼저 와도(`onState` 가 간격을 먼저 봄) 멈춤
  - 정상 완료 뒤 `PAUSED`·`controlled`(live)·`controlled_decel`(가짜 차량) 상태로 세 항목이 끊기지 않고 이어짐; `preempt`·`safety_preempt`·`STOPPED`·`TELEOP` 은 멈춤; 대기열 미션이 상태에서 `paused`·`cancelling` 이면 멈춤
  - 거부 `busy`·`stale` 은 멈춤 + 불확실, 그 밖의 거부는 멈춤만
  - 맨 앞이 불확실이면 `resume` 이 false; `resolveUncertain(drop)` 뒤 재개 가능; `resubmit` 은 불확실 표시만 지움
  - `toStorage`/`fromStorage`: 실행 상태는 멈춤; `submitting` 이던 항목은 불확실; `in_mission` 은 같은 epoch·결과 있으면 정리, 아니면 불확실
  - 제출 중·진행 중 항목은 `remove`·`move` 거절, 불확실 항목은 `remove` 허용
- [ ] **Step 2: 실패 확인** — `node --test "console/web/test/queue.test.mjs"`.
- [ ] **Step 3: 구현.**
- [ ] **Step 4: 통과 확인** — 같은 명령과 전체 `node --test "console/web/test/*.test.mjs"`.
- [ ] **Step 5: 커밋** — `console: mission queue core -- stop on any anomaly (result, operator action, STOP, foreign mission, reject, disconnect, lease, epoch, silent link), never auto-resubmit, uncertain items block resume`

### Task 2: 프로토콜 보강과 연결부

- [ ] **Step 1: 실패하는 시험** — `protocol.test.mjs`: `request(type, fields, {requestId})` 가 그 ID 로 보냄; `forget(id)` 뒤 `pump` 가 그 요청을 다시 보내지 않고 그 응답은 아무 Promise 도 풀지 않음(다른 요청은 그대로). `queuewire.test.mjs`(가짜 세션·저장소·시계): 세 항목 정상 흐름에서 `mission_submit` 이 정확히 3번; 끊김·권한 상실·새 epoch·STOP·취소·수동 제출·거부·상태 공백 각각에서 그 뒤 `mission_submit` 0번(재접속·재무장 뒤에도); 제출 중 끊김에서 `forget` 1번; ACK 전에 결과가 먼저 와도 1번만; 저장이 보내기보다 먼저; 대기열은 `sessionStorage`, 저장된 운반은 `localStorage` 키에만 씀; 저장 실패 시 문구 상태와 `removeItem` 시도.
- [ ] **Step 2–4:** 실패 확인 → 구현 → 통과.
- [ ] **Step 5: 커밋** — `console: protocol request ids and forget; queue wiring that sends only when the queue says so`

### Task 3: 화면

- [ ] **Step 1:** `main.js` 가 사건을 `queuewire` 에 넘기고(취소·일시정지·STOP·작업 시작 버튼은 대기열 `operatorAction` 을 먼저), 250 ms 타이머로 `tick`/`next`. `ui.js`·`index.html`·`app.css` 에 D9.
- [ ] **Step 2:** 가짜 차량 `inject("fail_running")` 과 그 단위 시험.
- [ ] **Step 3:** 화면 확인(Playwright, 가짜 차량): ① 저장된 운반 두 개 → 대기열 셋 → 재개(2단계) → 둘이 `done` 되고 셋째 진행 ② `fail_running` → 멈춤, 다음 안 냄 ③ 진행 중 취소 → 멈춤 ④ 항목 사이 STOP → 재무장 → 대기열은 멈춘 채 ⑤ 끊김 시험(3 s) → 멈춤, 재접속 뒤에도 멈춤, 진행 미션은 끝까지 ⑥ 새로고침 → 항목 남고 멈춤 ⑦ 두 번째 탭은 빈 대기열로 열리고 저장된 운반은 보임 ⑧ 1600·400 px 에서 가로 넘침 없음. 스크린숏은 저장소 밖.
- [ ] **Step 4:** `console/README.md`, 커밋 `console: mission queue on screen -- saved transports, queue panel, wizard buttons`.

### Task 4: 검증과 PR

- [ ] `node --test "console/web/test/*.test.mjs"`, 콘솔 Python 시험(ws1 `forklift-ws1-pytest tests/unit/console tests/integration/console`), Codex 리뷰 수렴, PR·머지, 총괄과 연속 이송 세션에 보고.

## 6. 자체 검토

계획 단계 독립 검토 3회.

**① 코드 사실 재확인(Claude).** `protocol.js` 를 읽어 답 없는 요청이 재접속 뒤 같은 `request_id` 로 다시 나감을 확인 → `forget` 단계 추가. `state.last_result`·`missions` 의 `outcome`·`mission_id`, `results.ACTIVE`(cancelling 포함)와 `wizard` 목록의 차이, 가짜 차량의 `busy`·적재 제출 거부, cycle 프로브의 재시도 사유를 원문에서 확인했다.

**② Codex 적대 검토 1회차.** P1 2·P2 5·P3 2, 모두 반영:
- P1 취소·STOP 을 결과로만 기다림(완료 직후 취소는 `not_running` 으로 끝나고 늦은 `done` 으로 다음이 나감) → 운용자 행동·STOPPED·`stop_kind`·남의 미션을 즉시 멈춤 사건으로(D4).
- P1 거부 뒤 자동 재제출(특히 `no_feedback:*`)은 원칙과 계약 1절에 어긋남 → 모든 거부는 멈춤, 대신 제출 전 조건(정지 확인·`feedback_ok`·결과 뒤 1 s)으로 피함(D3·D5).
- P2 수동 미션 끼어듦 → 작업 지정 창 제출은 대기열을 먼저 멈추고, 남의 진행 미션을 보면 멈춤(D4).
- P2 `forget` 에 요청 ID 경로 없음 → `request(…, {requestId})`, 저장 뒤 송신, 모든 멈춤에서 `forget`, 응답은 `{epoch, item_id, request_id}` 로 대조(D6).
- P2 제출 중 새로고침·epoch 모순 → 제출 중·진행 중 항목은 새 epoch·다시 열기에서 언제나 불확실(D6·D7).
- P2 여러 탭 덮어쓰기 → 권한 쥔 탭만 대기열 키를 쓰고, 저장된 운반은 다른 키(D7) — 2회차에서 탭별 `sessionStorage` 로 바꿈(아래).
- P2 결과가 100건 밖 → 다시 열 때 찾지 못하면 불확실, 불확실 항목은 지울 수 있음(D7·D9).
- P3 "같은 lease 를 되찾음" 원인 설명 정정(D6), 해석 실패 처리와 시험 기대값 통일(D1·Task 1).
- 시험에 실제 연결부를 묶어 보낸 요청 수를 세는 시험 추가(`queuewire.test.mjs`).

**③ agy 독립 검토(Gemini 3.8 Flash — `agy-quota-ok --pick`; ①·② 결론 미공개, "적히지 않은 전제").** 반영: 렌더 루프 대신 250 ms 타이머·사건 구동(뒤 탭에서 애니메이션 프레임이 멈춤), 결과 뒤 1.0 s 안정 대기, 권한 쥔 탭만 대기열 편집·다른 탭은 읽기 미러, 재개 2단계 확인과 멈춤 강조, 열린 채 끊긴 연결 대비 상태 2 s 공백 멈춤. 반영 안 함: 멈춤 소리(브라우저 자동 재생 제한, 강조로 대신), `destination_occupied` 때 다른 칸 자동 탐색(`auto` 항목·연속 이송 계획의 몫 — 이번엔 멈춤).

**② Codex 2회차.** 새 P1 없음, P2 4건 반영:
- 정상 대기의 `stop_kind` 는 `controlled`(live)·`controlled_decel`(가짜 차량)이라 "`none` 이 아니면 멈춤"은 정상 연속 실행을 끊음 → 안전 선점·`STOPPED`·`TELEOP` 과 대기열 미션의 `paused`·`cancelling` 만 멈춤 사건(D4).
- 2 s 공백을 타이머에서만 보면 늦게 온 상태가 공백을 지움 → 상태를 받는 순간 직전 간격을 먼저 판정(D3).
- 채널 포화 `busy`·원장이 잊은 `stale` 거부는 미실행을 보장하지 않음 → 멈춤 + 불확실(D5).
- 권한 인계 중 두 탭이 모두 `hasLease` 일 수 있어 "권한 쥔 탭만 저장"으로는 덮어쓰기를 못 막음 → 대기열을 탭별 `sessionStorage` 로(D7).

**② Codex 3회차.** 새 P1 없음, P2 1·P3 2 반영: 실행 중 `feedback_ok` 거짓은 기다림이 아니라 멈춤(중재기는 `PAUSED` 에서 피드백 상실을 선점하지 않아 회복 뒤 자동 제출이 됨, D3·D4) / 거부 시험 기대값을 D5 에 맞춤 / 대기열 키에 탭 `client_id`(복제 창의 `sessionStorage` 상속, D7).

**② Codex 4회차.** 계획 문구에는 새 P1/P2 없음(계획 수렴). Task 1 구현에서 P2 2건을 재현해 반영: ack 전 같은 팔레트·칸의 미션을 자기 것으로 본 뒤 ack 가 다른 ID 를 대면 멈춤(처음 본 ID 를 `current.seen` 에 두고 대조) / 불확실 항목이 하나라도 있으면 재개 안 됨, 불확실 항목을 옮기거나 그 앞뒤로 가로지르는 이동 금지, 불확실 항목의 늦은 결과 정리는 진행 중 제출 추적(`current`)을 건드리지 않음(`#record` 분리), "다시 내기"는 `maybe` 를 지움. 연결부(`queuewire`)에는 연결마다의 `welcomed`(재접속 소켓이 열린 뒤 새 welcome 전에는 내지 않음)를 더함.

**② Codex 5회차(구현).** P2 2건 반영: 상태 없이 ack 로 받은 조종 권한을 잃는 경우를 놓침 → 연결부가 상태·재개·타이머마다 권한 보유를 대조(`checkLease`) / 같은 세션 재접속 뒤 재개가 옛 상태를 새것으로 만듦 → `lastStateAt` 은 받은 상태만 갱신하고, 재개 뒤 첫 상태는 `waitSince` 기준 2초 안에 와야 함.

**② Codex 6회차(구현·화면).** P2 1건 반영: 바뀐 것이 없어도 타이머·상태마다 대기열 목록 DOM 을 다시 그려 버튼 포커스·클릭을 잃을 수 있음 → 연결부 버전은 실제 변경에만 오르고, 저장도 대기열이 바뀔 때만 하며, 목록 HTML 이 같으면 다시 넣지 않는다.

**② Codex 7회차.** 새 P1/P2/P3 없음 — 수렴(웹 시험 152개, DOM 대역으로 정지 2.5초·진행 2초 동안 불필요한 교체·저장 없음 확인).

**Isaac 실시간 운용(2026-10-22) 뒤 개정.** D4 의 피드백 상실을 '0.5 초 이상 이어진 거짓'으로 좁혔다 — [운용 확인](../validation/2026-10-22-console-queue-isaac-live.md).
