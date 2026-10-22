# 운용 콘솔 — 연결·장치 상태 패널과 경보 목록 계획(최소안)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

작성일: 2026-10-22 · 상태: 계획 3판 · 근거: [상용 FMS 사례 조사](../design/2026-10-22-fms-case-survey.md) 4절 2순위(**최소안**), 사용자 결정 2026-10-22(총괄 경유: 후보 1→3 최소안을 PR 하나씩, 확장안은 하지 않음)

**Goal:** 실차·Isaac 을 붙였을 때 "무엇이 안 오는지, 왜 멈췄는지"를 한 화면에서 보이고, 그 이상이 언제 생기고 풀렸는지를 경보 목록으로 남긴다.

**Architecture:** 차량 쪽(실행기·중재기·게이트웨이·console/1 계약)은 바꾸지 않는다. 콘솔이 **이미 받는** 것 — 제어·미디어 연결, 상태·위치·영상·스캔·지도의 나이, 상태의 `feedback_ok`·`mode`·`stop_kind`·`permit`·`scene`·`lease`·`slam`·`freshness`·`notes`, 대기 서버(standby) 응답과 세션 문구, 대기열의 멈춘 이유 — 를 순수 모듈(`console/web/js/health.js`)이 항목별 판정으로 바꾸고, 같은 모듈의 경보 기록부가 판정 변화를 경보로 남긴다. 화면은 왼쪽 메뉴의 새 '상태·경보' 페이지와 메뉴 배지다.

**Tech Stack:** 빌드 없는 ES 모듈, `node --test "console/web/test/*.test.mjs"`, 가짜 차량 + Playwright 화면 확인.

## 1. 범위(최소안)와 하지 않는 것

- 하는 것: 콘솔이 지금 받는 값만 모아 판정·표시, 판정이 나빠지고 풀린 것을 **이 화면이 본 시각**으로 기록.
- 하지 않는 것(조사 문서의 확장안): 차량이 내는 경보 원장(식별자·발생 시각·심각도·확인), 실행기의 `freshness` 채우기와 그 한계값, MCU 결함 비트(`mcu-link/1 STATE.fault_flags`, 보드 연결 뒤), 경보 확인(ack)·소리·알림, 탭·기기를 넘는 경보 공유, 여러 대 비교, 지금 SLAM 위치추정 건강 판정.
- 시각의 한계를 화면에 적는다: "시각은 이 화면이 처음 본 시각(브라우저 시계)입니다 — 실제 발생 시각과 다를 수 있고, 화면이 닫혀 있거나 잠든 동안의 일은 없습니다."

## 2. 설계

**D1 판정 수준.** 각 항목은 `{key, label, level, value, detail}`, `level` 은 다섯 가지다.
- `ok` — 받아야 할 것을 받고 이상이 없다. 열린 경보를 '복구'로 닫는 근거.
- `warn`·`crit` — 이상. 경보를 여는 근거.
- `unknown` — **알아야 하는데 모른다**(계약상 `null` = '모름', 아직 기다리는 중). 경보를 열지도 닫지도 않고, 진행 중인 열기·닫기 연속 시간을 끊는다.
- `off` — **지금은 해당이 없다**(대기열이 비었음, 의도한 세션 종료·장면 초기화·기동 중, 차량이 그 능력을 선언하지 않음, 정보만 보이는 행). 열린 경보를 '해당 없어짐'으로 닫고(복구와 구별해 표시), 새로 열지 않는다.

**D2 항목.** 먼저 **연결을 기대하지 않는 때**를 정한다(제어 연결이 닫혀 있을 때만): 운용자의 세션 종료 → "세션 종료됨", 대기 서버가 실패를 알리지 **않는** 동안의 장면 초기화 → "장면 초기화 중", 대기 서버가 다음 세션을 기동 중(실패 아님) → "세션 기동 중". 이때 `control`·`media`·`state`·`pose`·`camera`·`scan` 은 모두 off 다(대기 서버는 두 웹소켓을 정상적으로 503 으로 거절한다, `tools/live_supervisor.py`). **대기 서버가 알린 실패(아래 `session` 의 warn 조건)는 초기화 요청·기동 단계보다 우선한다** — 초기화 뒤 시작에 실패하면 welcome 이 오지 않아 `sessionEnding` 이 남지만, 그때 연결 행은 아래 일반 규칙(끊김 crit)을 따른다.

한계값은 이미 콘솔에 있는 값만 쓴다. "받은 뒤"의 시각은 모두 **현재 연결의 welcome 이후** 받은 것만 친다(이전 연결·이전 에폭의 수신은 새 세션의 증거가 아니다). `W` = 현재 제어 연결의 welcome 을 받은 시각.

| key | 판정 |
|---|---|
| `control` | 열림+welcome → ok / 열림·welcome 전 → warn "연결 중" / 닫힘 → crit "끊김 · 재접속 중" / 운용자가 세션 종료를 요청해 닫힘(`m.sessionEnding === "session_end"`) → off "세션 종료됨" / 장면 초기화 중 닫힘 → off "장면 초기화 중" |
| `state` | `W` 이후 받은 상태가 있으면 그 나이: ≤ `STALE_S`(1.0 s) ok / ≤ `SILENCE_MS`(2.0 s) warn / 그 위 crit. `W` 이후 아직 없음: `W` 뒤 2.0 s 까지 unknown "첫 상태 기다리는 중", 그 뒤 crit "welcome 뒤 상태가 오지 않음". 연결 전·세션 종료 → off |
| `pose` | 상태 나이 + `pose.age_s`(profile `staleness` 식): ≤ 1.0 s ok / 넘음 warn / `pose` 가 `null` → warn "위치 없음" / 상태 없음 → unknown |
| `camera` | **최근 프레임 수신**(센서 건강이 아님 — 재접속 때 같은 촬영 번호가 다시 올 수 있다). `capabilities.media.camera_rgb` 없음 → off. `W` 이후 받은 프레임의 나이 ≤ `videoStaleAfterS(state)` ok / 넘음 warn. 아직 없음: `W` 뒤 max(2.0 s, `videoStaleAfterS`) 까지 unknown, 그 뒤 warn "프레임이 오지 않음" |
| `scan` | 능력 없음 → off. 스캔 나이(차량 `age_s` 포함, main.js 기준) ≤ `STALE_S × 배속 보정` ok / 넘음 warn. 배속 보정은 `videoStaleAfterS` 와 같은 식(`pacing.target_factor` 가 0–1 이면 그 역수) — live 스캔은 120 Hz 물리 12 스텝마다라 배속 0.05 에서 2 s 간격이 정상이다. 아직 없음은 `camera` 와 같은 규칙 |
| `grid` | 능력 없음 → off / 있음 ok / 철회됨 → warn "지도 철회됨(사유)" / 아직 없음 → unknown(경보 없음 — 지도는 1 Hz 이하로 오고, 첫 지도 전 상태는 정상일 수 있다) |
| `media` | 미디어 웹소켓 열림 ok / 닫힘 warn "재접속 중" / 세션 종료 → off |
| `feedback` | `feedback_ok` true ok / false crit "차량 피드백 없음" / `null`·필드 없음 unknown |
| `stop` | `mode` 가 `null`(관찰 전용 게이트웨이 등) → unknown "모름". `STOPPED` → crit "정지됨 · 재무장 필요" / `stop_kind` 가 `preempt`·`safety_preempt` → crit "안전 선점" / 그 밖 ok. 관찰 전용 연결이면 값에 "이 화면으로 멈출 수 없음"을 덧붙인다. `stopped_confirmed` 는 `PAUSED`·`STOPPED` 에서만 설명에 쓴다("정지 확인 기다리는 중") — 주행 중에는 늘 거짓이라 쓰지 않는다 |
| `permit` | `limit_mps === 0` → warn(`permitText`) / 숫자 > 0 → ok(사유) / `permit` 이 `null` 이거나 `limit_mps` 가 숫자가 아님(live 상태 생성기의 `{limit_mps: null, reason}`) → unknown |
| `scene` | `scene.damaged` 있음 crit(사유·시각) / 손상 없음 ok / 필드 없음 off |
| `lease` | 경보 없음(늘 off, 값만): 이 기기 / 다른 기기 / 상태의 `lease` 가 객체이고 `holder` 가 `null` → "아무도 쥐지 않음" / `lease` 가 `null` → "모름"(계약상 `null` 은 모름) |
| `slam` | 경보 없음(off): "SLAM 시작 검사 통과(지도 …)"(`results.js` `slamLabel`) 또는 "온라인 작성 또는 모름" — `map_loaded` 는 세션 시작 검사 기록이지 지금 위치추정의 건강이 아니다 |
| `freshness` | 경보 없음(off): 수치가 있으면 그대로, 모두 없으면 "차량이 주지 않음" — 한계값이 계약에 없다 |
| `session` | 대기 서버 응답(`/api/v1/session`)을 분류한다. 우선순위: 마지막 종료가 실패(`last_exit.exit_code` 가 0·3 이 아님) 또는 `phase: "stopped"` 이고 `end` 가 `ended`·`terminated` 가 아님 → warn(`standbyText` 문구; `noteText` 가 대기 서버 문구를 페이지 문구보다 앞세우는 것과 같은 이유) / 운용자가 연 세션 종료·장면 초기화 → off / 그 밖의 대기 서버 응답(기동 중 `starting`·`cleanup`·재시도 `backoff`) → off "세션 기동 중" / 응답 없음 → ok. 상태의 `notes` 는 값에 붙여 보이되 경보로 만들지 않는다(게이트웨이가 늘 싣는 안내가 있을 수 있다) |
| `queue` | 실행 중 ok / 멈춤이고 항목이 남았으며 이유가 `CALM_REASONS`(아래)에 없음 → warn "대기열 멈춤: 이유" / 그 밖(항목 없음 포함) off |

`CALM_REASONS` 는 `queue.js` 에 **새로 정의해 내보내는** 평상 이유 목록이다: '시작 전'·'완료'·'콘솔 다시 열림'·'운용자 멈춤'과 운용자 동작 '운용자 취소'·'운용자 일시정지'·'STOP'·'수동 지시'(`OPERATOR` 표의 값). 상태에서 본 '정지됨(STOP)'(다른 기기·차량의 정지)은 이상이다. `SILENCE_MS` 도 `queue.js` 에서 내보낸다.

**D3 경보 기록부.** `AlarmBook.update(rows, nowMs, nowUnix)` 는 화면 갱신(100 ms)과 **상태 수신 때마다** 부른다(배경 탭에서 타이머가 1 s 로 느려져도 웹소켓 메시지는 온다).
- 행마다 연속 시간을 센다. `warn`·`crit` 가 **0.5 s 이상 이어지면** 경보를 연다: `{id, key, label, level, text, peak_text, opened_unix, cleared_unix: null, cleared_note: null}`. `opened_unix` 는 그 연속의 **첫 관측 시각**이다(0.5 s 뒤 확정한 시각이 아니다).
- 열린 동안: 더 나빠지면(`warn`→`crit`) `level` 을 올리고 `peak_text` 를 그때 문구로 바꾼다. 수준은 같고 문구만 바뀌면 `text` 만 바꾼다. 내려가도 `level` 은 최고값을 유지한다(`text` 는 지금 문구, `peak_text` 는 최고 수준 때 문구).
- `ok` 가 **1.0 s 이상 이어지면** 닫는다(`cleared_note: "복구"`). `off` 가 되면 곧바로 닫는다(`"해당 없어짐"` — 세션 종료로 off 가 되면 `"관측 종료"`). `unknown` 은 열린 경보를 그대로 두고 진행 중인 연속 시간을 끊는다.
- **관측 공백:** 앞 호출과 2.0 s 넘게 떨어진 호출(노트북 잠자기, 탭 정지)이면 모든 연속 시간을 처음부터 센다 — 보지 못한 구간을 이어진 것으로 치지 않는다. 열린 경보는 그대로 두고, 그 뒤 판정으로 닫힌다.
- 새 세션(`session_epoch` 가 바뀜): 열린 경보를 모두 닫고(`"새 세션"`) 연속 시간을 지운다.
- 최근 100 건만 둔다(열린 것은 지우지 않는다).
- **보존:** 탭의 `sessionStorage`(`console.alarms.<vehicle_id>.<client_id>`)에 기록부를 저장해, 문제 뒤에 새로고침해도 목록이 남는다. 되살릴 때 열려 있던 경보는 `"화면 다시 열림 — 그 사이 모름"` 으로 닫고, 지금도 이상이면 새 판정으로 다시 열린다. 저장은 기록부가 바뀔 때만 한다. 다른 탭·기기와는 공유하지 않는다(화면에 적는다).
- **보지 못한 위험 경보:** 이 페이지를 보지 않는 동안 열렸다가 닫힌 `crit` 경보 수를 센다(`unseen_crit`). 페이지를 열면 0 이 된다. 확인(ack) 기능이 아니라 "봤는지" 표시다.

**D4 화면.**
- 왼쪽 메뉴에 '상태·경보'(`data-view="health"`)를 '통신' 위에 더하고 배지를 단다: 열린 경보 수(위험이 하나라도 있으면 위험색, 아니면 주의색). `unseen_crit > 0` 이면 숫자가 있든 없든 그 옆에 위험색 점을 단다(열린 주의 경보가 보지 못한 위험 경보를 가리지 않게). 둘 다 없으면 숨김.
- 페이지 위: 장치 상태 표 — 색 점, 이름, 값, 설명. 행의 HTML 이 바뀐 행만 다시 그린다(같으면 DOM 을 건드리지 않는다 — FMS-1 Codex 6회차 교훈).
- 페이지 아래: 경보 목록 — 열린 것 먼저(최근 순), 그 아래 닫힌 것(최근 순): 수준, 이름, 문구(최고 수준 때 문구가 다르면 함께), 연 시각–닫은 시각(지속 시간), 닫힌 까닭(복구 / 해당 없어짐 / 관측 종료 / 새 세션 / 화면 다시 열림), 1절의 시각 한계 문구.
- 모니터링 화면은 바꾸지 않는다.

**D5 하지 않는 것.** 1절의 확장안 전부, 기존 '통신' 페이지 재배치, 가짜 차량·게이트웨이·실행기 수정.

## 3. 파일

- Create: `console/web/js/health.js` — `healthRows(input) → rows`, `class AlarmBook`(`update`, `markSeen`, `toStorage`, `static fromStorage`), `badge(book) → {count, level, dot}`. DOM·시계 없음.
- Create: `console/web/test/health.test.mjs`
- Modify: `console/web/js/queue.js` — `SILENCE_MS` 내보내기, `CALM_REASONS` 새로 정의해 내보내기(기존 동작은 바꾸지 않음)
- Modify: `console/web/js/main.js` — 입력 모으기(`m.welcomeAt`, 미디어 웹소켓 열림 `m.link.mediaUp`, 지도 철회 사유 `m.gridWithdrawn`, 대기 서버 원 응답 `m.standby`, `W` 이후 첫 상태·프레임·스캔 시각), 상태 수신과 100 ms 갱신에서 `healthRows` → `AlarmBook.update`, 저장
- Modify: `console/web/js/ui.js`(`renderHealth`), `index.html`(메뉴·페이지), `app.css`
- Modify: `console/README.md` — 사용법 한 단락, 가짜 차량 항목에 "하역된 팔레트를 다시 집는 미션(예: P-01→D-A 뒤 P-01→D-B)은 접근 중 `occupied` 로 멈춰 끝나지 않는다(가짜 차량의 장면 인공물, 막힘 시간 제한 없음 — 끊김과 무관, 2026-10-22 분류)" 한 줄
- import 출처: `STALE_S`·`videoStaleAfterS`·`staleness`·`permitText`·`stopKind` 는 `profile.js`, `slamLabel`·`standbyText`·`noteText` 는 `results.js`, `SILENCE_MS`·`CALM_REASONS` 는 `queue.js`.

## 4. Task

### Task 1: 판정과 경보 기록부(순수)

- [ ] **Step 1:** `health.test.mjs` 를 먼저 쓴다. 시험 이름과 단언:
  - `"rows follow the existing limits"` — `state`: `W` 이후 나이 0.9 ok, 1.5 warn, 2.5 crit; `pose` 0.8+0.3 warn; `camera` 나이 > `videoStaleAfterS` warn; `feedback_ok` false crit, 없음 unknown; `STOPPED` crit, `controlled` 의 `PAUSED` ok, `safety_preempt` crit; `permit.limit_mps` 0 warn; `scene.damaged` crit; `lease`·`slam`·`freshness` 는 늘 off.
  - `"a state from before this welcome does not count; no first state within 2 s is critical"` — `stateAt < W` 면 `W`+1.9 s unknown, `W`+2.1 s crit.
  - `"scan and camera never received: unknown, then warn; not declared: off"`.
  - `"slow pacing widens the scan limit like the video one"` — `pacing.target_factor` 0.05 에서 스캔 나이 2.0 s ok, 21 s warn.
  - `"a monitor-only state is unknown, not ok"` — `mode` null → `stop` unknown, `lease` null → 값 "모름", `{holder:null}` → "아무도 쥐지 않음"; 관찰 전용이면 `stop` 값에 "멈출 수 없음".
  - `"an intended session end or reset turns the link rows off"` — `sessionEnding` `session_end` + 닫힘 → `control`·`state`·`media`·`camera`·`scan` off; standby `phase: starting` → `session` off; `last_exit.exit_code` 1 → warn.
  - `"the queue row warns only for an anomaly stop with items left"` — '실패: x'·'연결 끊김'·'정지됨(STOP)' + 항목 1 warn; `CALM_REASONS` 의 모든 이유나 항목 0 → off.
  - `"an alarm opens after 0.5 s of warn at its first sighting and closes after 1.0 s of ok"` — 0.4 s 없음, 0.6 s 열림이고 `opened_unix` 는 첫 관측; ok 0.9 s 열린 채, 1.1 s 닫힘 `"복구"`.
  - `"off closes at once as not applicable; unknown breaks a streak and closes nothing"`.
  - `"an escalation raises the level and keeps the peak text; a drop keeps the level"`.
  - `"a gap over 2 s restarts every streak"` — warn 0.3 s, 3 s 공백, warn 0.3 s → 열리지 않음.
  - `"a new session closes open alarms with a note"`, `"only the last 100 alarms are kept; open ones stay"`.
  - `"restored alarms that were open are closed as unseen"` — `fromStorage` 뒤 열린 것 0, `cleared_note` "화면 다시 열림 — 그 사이 모름".
  - `"the badge counts open alarms, takes the worst level, and shows a dot for an unseen critical one"` — 닫힌 crit 1 + 미열람 → `{count: 0, dot: true}`; `markSeen()` 뒤 dot false; 열린 warn 1 + 미열람 crit 1 → `{count: 1, level: "warn", dot: true}`.
  - `"while a standby starts the session or a reset runs, the links are not expected"` — standby `starting` 또는 `sim_reset` + 두 소켓 닫힘 → 연결 행 6개 off.
  - `"a failed start shows through a reset in progress"` — `sim_reset` + standby 실패 → `session` warn, `control` crit.
  - `"a permit with no limit is unknown"`.
- [ ] **Step 2:** 돌려 실패를 확인한다.
- [ ] **Step 3:** `health.js` 를 구현하고 `queue.js` 에 내보내기를 더한다(3절 import 출처, 수치 복사 금지).
- [ ] **Step 4:** 통과 확인, 커밋 `console: device status rows and an alarm book from what the console already receives`.

### Task 2: 화면

- [ ] **Step 1:** `main.js` 입력 모으기·상태 수신과 100 ms 갱신 연결·저장, `ui.js`·`index.html`·`app.css` 에 D4.
- [ ] **Step 2:** 화면 확인(가짜 차량, Playwright): ① 처음 연결 → 경보 0, `lease`·`slam`·`freshness` 는 값만 ② 끊김 시험(3 s) → `control` crit 경보, 재접속 1 s 뒤 '복구'로 닫힘(지속 시간) ③ STOP → `stop` crit, 재무장 뒤 닫힘 ④ 장면 손상 주입(`/mock/inject/scene_damage`) → `scene` crit, 배지 위험색 ⑤ 진행 중 실패 주입 + 대기열 항목 남김 → `queue` warn; 대기열 비우기 → '해당 없어짐'으로 닫힘 ⑥ 다른 페이지를 보는 동안 crit 가 열리고 닫힘 → 배지 점, 페이지를 열면 사라짐 ⑦ 새로고침 → 목록 남고 열린 것은 '화면 다시 열림'으로 닫힘 ⑧ 바뀐 것 없을 때 표의 행 노드가 그대로 ⑨ 1600·400 px 가로 넘침 없음. 스크린숏은 저장소 밖.
- [ ] **Step 3:** README, 커밋 `console: status and alarms page`.

### Task 3: 검증과 PR

- [ ] 웹 시험 전체, ws1 `forklift-ws1-pytest tests/unit/console tests/integration/console`, Codex 리뷰 수렴, PR(merge commit), 총괄 보고.

## 5. 자체 검토

**① 원자료 재대조(Claude).** `STALE_S = 1.0`·`scanDrawable`(`profile.js`), `videoStaleAfterS`(배속 목표의 역수, 기본 1.0 s), 대기열 `SILENCE_MS = 2000`, `STOP_KINDS`(`none`·`controlled`·`controlled_decel`·`preempt`·`safety_preempt`), 상태의 `freshness`(가짜 차량은 셋 다 `null`, 게이트웨이는 `scan_age_s` 까지 채움, 실행기는 `freshness` 를 넘기지 않음 — `safety/state.py` 는 받은 것을 실을 뿐), 지도 철회 메시지(`withdrawn_grid` 의 `reason`), 가짜 차량 `inject("scene_damage")` 를 소스에서 확인. 고친 것 2: `stopped_confirmed` 거짓을 경보 대상에서 뺌 / 대기열의 운용자 동작 이유를 평상 이유에 넣고 목록을 `queue.js` 에서 내보내 씀.

**② Codex 적대적 검토 1회차(새 스레드).** P1 없음, P2 5·P3 5 — 모두 소스로 확인하고 2판에 반영: `info` 로 바뀐 행의 열린 경보가 영원히 남음 → 수준을 `unknown`(모름, 닫지 않음)과 `off`(해당 없음, 곧바로 닫음)로 나눔(D1) / 정상 세션 종료·기동이 여러 장애 경보를 만듦(`main.js` 종료 경로는 재접속을 멈추고 문구를 남김, standby `starting`·`cleanup` 도 문구가 있음) → 의도한 종료·초기화·기동은 off, 닫힌 까닭에 '관측 종료' / 관찰 전용의 `null` 을 정상·'아무도 없음'으로 읽음(게이트웨이 `mode`·`stop_kind`·`lease` 가 `null`, 계약 6절 "null 은 모름") → `stop` unknown, `lease` 는 `null` 과 `{holder:null}` 구별 / welcome 뒤 첫 상태가 영원히 안 오는 장애에 경보가 없음 → `W` 이후 수신만 치고 2 s 뒤 crit(카메라·스캔도 같은 규칙) / `slamLabel` 은 `profile.js` 가 아니라 `results.js`, `CALM_REASONS` 는 새로 정의해야 함 → import 출처 표와 파일 목록에 `queue.js`. P3: SLAM 행을 '시작 검사'로 한정 / `stopped_confirmed` 는 주행 내내 거짓이라 정지 모드에서만 설명 / 저배속에서 스캔 간격 2 s 가 정상(live 12 스텝마다) → 스캔 한계에 영상과 같은 배속 보정 / 카메라 행은 '최근 프레임 수신'(같은 에폭 재접속 때 같은 촬영 번호가 다시 옴) / 기록부 규칙 명시(`unknown` 이 연속 시간을 끊음, `opened_unix` 는 첫 관측, `peak_text`, 닫힌 까닭 표시, 2 s 넘는 관측 공백은 연속으로 치지 않음).

**③ agy 독립 검토(Gemini 3.8 Flash — `agy-quota-ok --pick` 이 Argon 을 보류해 Flash 선택; 1·2 의 결론은 보여주지 않고 "적히지 않은 전제"를 물음).** 5건 중 3건 반영, 2건 부분 반영: 문제 뒤 새로고침하면 탭 메모리의 경보가 사라짐 → 탭 `sessionStorage` 보존(탭 간 공유는 하지 않음, 화면에 적음) / 배경 탭 타이머가 느려지면 판정을 놓치거나 잠자기 뒤 가짜 지속시간 → 상태 수신 때도 기록부를 부르고 2 s 넘는 공백은 연속으로 치지 않음 / 잠깐 열렸다 닫힌 위험 경보를 운용자가 모름 → 확인(ack)은 최소안 밖이라 대신 '보지 못한 위험 경보' 점 / 시뮬레이션 저배속에서 오경보(부분) → 영상·스캔은 배속 보정, 상태·위치 나이는 연결의 벽시계 지연이라 보정하지 않음(기존 정지 표시 배너와 같은 기준) / 관찰 전용에서 빈 안전 필드가 사각지대(부분) → 마스크 대신 `unknown` 을 '모름'으로 드러내 보임(Codex P2 와 같은 처리).

**② Codex 2회차.** P1 없음, P2 3·P3 1 — 시험으로 재현해 3판에 반영: 정상 대기 서버 기동·장면 초기화 중에도 `control` crit·`media` warn 이 남(대기 서버는 두 소켓을 503 으로 거절) → '연결을 기대하지 않는 때'를 연결 행 6개에 일관되게 적용 / 초기화 뒤 시작 실패를 `sim_reset` 의 off 가 가림(`sessionEnding` 은 새 에폭 welcome 이 와야 지워짐) → 대기 서버의 실패가 초기화·기동보다 우선 / 열린 주의 경보가 미열람 위험 점을 숨김 → 점은 숫자와 함께도 단다. P3: `permit.limit_mps` 가 `null` 인 live 형태도 unknown 으로 명시.
