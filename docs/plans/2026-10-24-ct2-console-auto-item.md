# CT-2 콘솔 — 대기열의 자동 항목 "남은 팔레트 전부" 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

작성일: 2026-10-24 · 상태: 1판(인터페이스 확정 — CT-2 계획 3판 D3·D4, 구현 중) · 근거: [CT-2 다음 팔레트·하역 칸 계획](2026-10-24-ct2-next-pick.md)(브랜치 `feat/ct2-ordering`, 연속 이송 세션) D3·D4, [미션 대기열 계획](2026-10-22-console-mission-queue.md) D1(`auto` 예약)·D3–D7, 총괄 지시 2026-10-24

**Goal:** 대기열에 "남은 팔레트 전부" 항목 하나를 넣으면, 차량이 추천한 다음 팔레트·하역 칸을 확정 ID 로 하나씩 내고, 남은 팔레트가 없을 때 끝난다. 이상 시 멈춤(사용자 결정 2026-10-22)은 그대로다.

**Architecture:** 판정은 차량 쪽(`next_pick.recommend`, 계약 `pick_request`/`pick`). 콘솔은 추천을 받아 지금과 같은 `mission_submit` 으로 낸다. 대기열 순수 모듈(`queue.js`)에 추천 대기 단계를 더하고, 연결부(`queuewire.js`)가 요청·수신을 잇는다. 화면은 대기열 칸에 버튼 하나와 단계 표시를 더한다.

## 1. 설계

**C1 항목.** `{kind: "auto", scope: "remaining", label: "남은 팔레트 전부"}`. 확정 ID 는 비워 둔다. 대기열에 하나만 둔다(이미 있으면 버튼이 꺼진다). 남은 팔레트가 없다는 추천(`none_remaining`)이 올 때까지 맨 앞에 남는다.

**C2 단계.** 맨 앞 항목의 단계에 둘을 더한다: `waiting` → **`picking`**(요청이 나감) → **`picked`**(추천을 받음) → `submitting` → `in_mission`.
- `next()` 는 맨 앞이 `auto` 이고 지금의 제출 조건 1–5 가 참이면 `{pick_request: {item_id, request_id}}` 를 돌려주고 `picking` 으로 간다. 열쇠는 `{epoch, item_id, request_id}`.
- 요청을 보낸 뒤 기한(`PICK_DEADLINE_MS`, 기본 5 s, 벽시계 — 차량 쪽 V1·V3 측정으로 다시 정함) 안에 열쇠가 맞는 `pick` 이 없으면 멈춘다("자동 선택 응답 없음").
- 요청 기록(`pick`: 열쇠·보낸 시각·`acked`)은 `ack` 를 받거나 놓을 때까지 남는다. 추천을 쥐어도(`picked`) **`ack` 전에는 제출하지 않고**, 그 사이 온 `reject` 는 멈춘다. 멈춤·다시 묻기·삭제로 놓을 때 `ack` 전이면 재전송 목록에서 뺀다(`forget`). 기한은 보낸 시각부터이고 `pick` 이나 `ack` 가 타이머보다 먼저 와도 따진다(기한 지난 `ack` 도 멈춤).
- `ack` 는 재전송만 끝낸다(단계는 그대로). `reject` 는 멈춘다("자동 선택 거부: 사유" — 거부 사유표로 풀어 보인다). `pick` 이 `ack` 보다 먼저 와도 받는다. 같은 요청의 재전송이 저장된 `ack` 를 받으면 차량은 `pick` 을 다시 계산해 보내지만, 열쇠가 맞는 첫 `pick` 만 쓴다. 예외 하나: `ack` 전에 온 `none_remaining` 은 아직 확정이 아니어서, 그 뒤 다시 계산된 `ok` 가 오면 그것으로 바꾼다.
- `pick` 의 `reason` 이 `ok` 이고 `targets_revision` 이 지금 목록과 같으면 `picked` 로 둔다. 제출은 다음 `next()` 에서 **제출 조건 1–5 를 다시 확인한 뒤** 낸다(조건이 거짓이면 고정 항목처럼 기다린다). `picked` 동안 목록 revision 이 바뀌거나, 받은 revision 이 처음부터 다르면 한 번 다시 묻는다. 두 번째도 다르면 멈춘다("대상 목록이 계속 바뀜").
- `none_remaining`: 자동 항목을 '완료'로 빼고 다음 항목으로. 대기열이 비면 멈춤 사유 '완료'. `ack` 전에 왔으면 `picking` 에 둔 채 `ack` 를 기다렸다가 뺀다 — 그 사이 `reject` 는 멈추고 뒤의 항목을 내지 않는다. 같은 요청에 다시 계산된 `ok` 가 오면 그것을 쓴다.
- `no_free_destination`·`all_blocked`·`error:<예외 종류>`(`pick` 은 null): 멈춘다("자동 선택 실패: 사유"), 항목은 맨 앞. 사유 어휘는 계약이 고정한 다섯(`ok`·`none_remaining`·`no_free_destination`·`all_blocked`·`error:*`)이고, 모르는 사유도 멈춤이다.
- 멈춤(STOP·끊김·권한 상실·새 epoch·운용자 멈춤·항목 삭제·피드백 상실 등)은 `picking`·`picked` 에서 요청을 잊고(`session.forget`) `waiting` 으로 돌아간다. **불확실 표시를 만들지 않는다**(아직 낸 것이 없다).
- `submitting` 부터는 고정 항목과 같다: `current` 에 추천받은 팔레트·칸, `#isOurs`, 결과 처리, 불확실 규칙.

**C3 결과.** `done`: 끝낸 목록·수에 실제로 낸 팔레트·칸으로 적고, 자동 항목은 맨 앞에 남아 다음 추천으로. `failed`·`recovery_required`·`cancelled`: 지금처럼 멈춤, 자동 항목은 맨 앞. 미션 결과는 결과 기록부에 있으므로 실적 요약은 바뀌지 않는다.

**C4 불확실(제출 뒤).** 자동 항목의 불확실은 **그 한 번의 제출**에 붙인다(항목의 `uncertain` + `uncertain_pallet_id`·`uncertain_destination_id` + `maybe` — 끊김·새 세션·`busy`/`stale` 거부 모두 같은 정보를 남긴다). 재개는 불확실이 풀릴 때까지 막힌다. 화면의 선택지는 '확인하고 계속'(불확실만 지움 — 다음 추천은 차량의 실제 목록으로 계산되므로 같은 팔레트를 두 번 옮기지 않는다)과 '자동 항목 지우기'. 같은 세션에서 그 미션의 결과가 오면 끝낸 목록에 적고 불확실을 지운다(고정 항목의 `maybe` 와 같다).

**C5 저장(D7).** `picking`·`picked` 는 저장하지 않는다(다시 열면 추천부터). 진행 중 미션은 고정 항목처럼 C4 의 불확실로 되살린다.

**C6 화면.** 자동 항목 줄에 '옮긴 n건'과 마지막 추천의 남은 계획 수(자동 항목을 지우거나 새로 넣거나 추천이 실패하거나 새 세션이면 지운다). '멈춤' 버튼 설명: 대기열만 멈추고 진행 중 미션은 끝까지 간다(= '이번 미션까지만'). 대기열 칸에 '남은 팔레트 전부' 버튼(welcome `capabilities.next_pick === true` 일 때만, 하나만). 항목 줄은 라벨 + 지금 추천(`P-2 → D-1`)·남은 계획 수(`plan` 길이). 단계 문구 '자동 선택 중'·'자동 선택: P-2 → D-1'. 멈춘 이유는 `queueStopText` 에 자동 선택 사유 문구표를 더해 한국어로, `blocked`(`{팔레트·칸 ID: ["pickup:<id>"·"delivery:<id>"·"occupied:<id>"·"props"·"no_pose"·"no_size"…]}`)는 한국어로 풀어 상태 행 설명란과 대기열 칸에 요약.

**C7 가짜 차량.** 화면 시험용 `pick_request`: 첫 `available` 팔레트와 확인된 자리가 아닌 첫 칸을 고르는 **장난감 규칙**(실제 규칙이 아님), 주입 `/mock/inject/pick_<사유>` 로 다음 한 번의 추천 사유를 바꿀 수 있다.

**C8 파이썬 판 맞춤.** `tools/mission_queue.py` 의 자동 동작은 연속 이송 세션이 구현한다. `tests/unit/tools/test_mission_queue_port_auto.py` 가 같은 사건열(자동 항목·추천·거부·기한·revision·`ack` 순서) 19 개를 Node 의 `queue.js` 와 파이썬 판에 넣어 사건마다 상태를 대조한다. 파이썬 판에 자동 항목이 없으면 건너뛴다.

## 2. 인터페이스(확정, CT-2 계획 3판)

- welcome `capabilities.next_pick === true` 면 지원, `next_pick_deadline_ms`(정수, 없으면 5000).
- `pick_request {t, request_id}` + 원장 필드: 관찰 권한, 신뢰 요청, `ack`/`reject`(`stale`·`busy`·`unsupported`·`mission_active`).
- `pick {request_id, targets_revision, pick|null, reason, plan, missions, estimate_total_m, home_assumed, blocked, pose}`; `reason` ∈ `ok`·`none_remaining`·`no_free_destination`·`all_blocked`·`error:<예외 종류>`; `blocked` 사유 ∈ `pickup:<id>`·`delivery:<id>`·`occupied:<id>`·`props`·`no_pose`·`no_size`.
- 콘솔 의견 1–6(제출 전 조건 재확인, 계산 실패도 `pick`, 어휘·형식 고정, 기한 5 s 로 늘리고 실측으로 다시 정함, 기능 표시, `pick`-먼저 수용)은 그 계획에 반영됐다. 멈춤 문구와 `next()` 반환 형식은 이 JS 가 정본이고, 파이썬 판(`tools/mission_queue.py`)은 이 JS 를 줄 단위로 옮긴다(연속 이송 세션).

## 3. Task

1. [x] `queue.js` — C1–C5, 시험 `console/web/test/queue_auto.test.mjs`(20).
2. [x] `queuewire.js` — 요청·수신·잊기·기한, 시험 `queuewire_auto.test.mjs`(5, 보낸 요청 수를 셈). `protocol.js` 는 그대로(`pick` 은 구독자에게 그대로 전달된다).
3. [x] 화면 C6(`index.html`·`ui.js`·`main.js`·`app.css`, 문구표 `health.js` `queueStopText`·`blockedText`), 가짜 차량 C7 + `tests/unit/console/test_mock_vehicle_pick.py`(5), Playwright(가짜 차량: 정상 2 건 → 완료, 모두 막힘, 응답 없음, 추천 중 끊김).
4. [x] 맞춤 시험 C8(두 브랜치를 합친 트리에서 33/33 — 자동 19·기존 14, 포트 f0997ca9), ws1 pytest(`tests/unit/console tests/integration/console` + C8: 163 통과·20 건너뜀 — C8 19 는 이 브랜치에 포트가 없어 건너뜀), Codex 수렴(3회차), Isaac live 확인(5 절).
5. [ ] PR.

## 4. 자체 검토

**① 원자료 재대조(Claude).** 확정 인터페이스(CT-2 계획 3판 D3·D4, 계약 5.3)와 구현을 대조: 기능 표시·기한 이름, 사유 어휘 다섯·`blocked` 사유 여섯, 열쇠 `{epoch,item_id,request_id}`, `picking` 이 불확실을 만들지 않음, revision 다시 묻기 1 회, `none_remaining` → 완료. 차이 하나 — 계획서는 `pick_request` 의 `reject` 도 "자동 선택 실패"라 했으나 콘솔은 거부 사유표로 풀려고 "자동 선택 거부"로 가름 → 그 계획 D4 에 반영됨. 가짜 차량에서 정상 흐름(추천 2 건 → 완료)·모두 막힘·응답 없음·추천 중 끊김을 화면으로 확인.

**② Codex 적대적 검토 1회차(새 스레드, 계획 + 구현).** P1 없음, P2 4·P3 1 — 모두 재현 확인 뒤 반영: 추천 뒤 도착한 거부를 무시하고 제출(요청 기록을 추천과 함께 지움) → 요청 기록을 `ack` 까지 유지, `ack` 전 제출 금지, 놓을 때 `ack` 전이면 잊음 / 기한 지난 추천이 타이머보다 먼저 오면 제출 → 수신 때도 기한 판정 / 자동 제출의 `busy`·`stale` 거부에서 팔레트·칸이 불확실 표시에 안 남음 → 남김 / 가짜 차량이 저장된 `ack` 재전송에 새 `pick` 을 안 보냄(계약 5.3) → 모든 `ack` 뒤 다시 계산 / 지웠다 다시 넣은 자동 항목에 이전 추천 표시 → 지우거나 넣을 때 비움.

**② Codex 2회차(같은 스레드).** 1회차 반례는 모두 해결 확인. 새 P2 2 — 재현 확인 뒤 반영: `ack` 전에 온 `none_remaining` 이 요청을 지워 뒤이은 `reject` 를 놓치고 다음 고정 항목을 냄 → `ack` 를 기다렸다가 자동 항목을 뺌 / 기한 지난 `ack` 가 타이머보다 먼저 오면 `ack` 대기 기한이 사라져 제출 → `ack` 수신 때도 기한 판정(`onAnswer` 가 시각을 받음).

**② Codex 3회차(같은 스레드).** 수렴 — 새 P1/P2 없음. 2회차 반례 재실행에서 거부·늦은 `ack` 뒤 제출 0 건, `ack` 전 멈춤·삭제·비우기·끊김·권한 상실·새 epoch 에서 재전송 중단·늦은 응답 무시·불확실 없음 확인. P3 1(문구): '첫 `pick` 만 쓴다'에 `ack` 전 `none_remaining` 예외를 적음 → C2 에 반영, 공유 계약 문구는 차량 쪽 세션에 전달.

**③ agy 독립 검토(Gemini 3.8 Flash — `agy-quota-ok --pick`, Argon 한도 보류; ①·② 결론은 보여주지 않고 "적히지 않은 전제"를 물음).** 5 건 중 2 반영: '이번 미션까지만 하고 멈춤'이 안 보임 → '멈춤' 버튼에 그 뜻을 적음(지금 동작이 그렇다) / 진척이 안 보임 → 자동 항목 줄에 '옮긴 n건'. 반영 안 함: 여러 탭 경합(조종 권한을 쥔 탭만 제출하므로 이미 막힘), 추천과 제출 사이 환경 변화(차량이 제출을 다시 검사하고 revision 이 바뀌면 다시 물음), 적재 상태 기반 수습 안내(최소안 밖), 제출 전 카운트다운(정책상 이상이 없으면 내는 것이 대기열의 뜻이고, 운용자는 언제든 멈춤·취소 가능).

## 5. Isaac live 확인 (2026-10-24)

ws1 잡 10499(`console_ops.sbatch`, `shard:2`), 장면 `scenes_ct1_a770031/line_m_session.json`(팔레트 P-1–P-3 한 줄, 하역 칸 D-1–D-3), 트리 = 콘솔 6ba22d94 + 차량 32d12a03 병합(1e4f5eb3, 스냅숏의 `SNAPSHOT_COMMIT`). 터널 + Playwright(1600 px).

- welcome 의 `next_pick` 으로 '남은 팔레트 전부 옮기기' 버튼이 보였고, 넣은 뒤 비활성. 재개(2단계) 직후 추천 P-1 → D-1, 0.21 s 뒤 제출.
- m1 `done` → 1.2 s 뒤 추천 P-2 → D-2 → 제출(m2). m3: P-3 → D-3. 네 번째 추천 `none_remaining` → 자동 항목이 빠지고 '완료'. 끝낸 목록 3 건 모두 실제 팔레트·칸으로 적힘, 줄의 '옮긴 n건' 1→2→3.
- 차량 기록(`live_log.jsonl`): `pick_request` 4 건 모두 `ack`(재전송 없음), `mission_submit` 3 건.
- 둘째 미션 중 15:59:14.4–15:59:26.7 에 콘솔→차량 메시지가 12.3 s 끊기고(echo 간격) 그동안 화면도 상태를 받지 못했다. 제어 틱은 1 s 넘는 틈이 없었고 연결은 하나로 유지됐다(재접속 없음). ws1 부하 28–39(20 CPU). 원인이 실행기의 연결 처리인지 터널·네트워크인지는 이 기록으로 가를 수 없다. 대기열은 '상태 수신 끊김'으로 멈췄고, 조종 권한은 3 초 뒤 만료, m2 는 끝까지 가서 완료로 기록됐다 — 고정 항목과 같은 멈춤 정책대로. 권한을 다시 받아 재개하자 m3 를 이어 냈다.
- 증거: ws1 `test_runs/live/console_ops/ct2_line_m/`(실행 기록·화면 `screens/`), 요약 스크립트 `cycle_gap.py`·`gap_scan.py`.
