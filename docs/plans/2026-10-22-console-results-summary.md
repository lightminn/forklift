# 운용 콘솔 — 실적 요약 계획(최소안)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

작성일: 2026-10-22 · 상태: 구현됨(계획 2판, PR feat/console-results-summary) · 근거: [상용 FMS 사례 조사](../design/2026-10-22-fms-case-survey.md) 4절 3순위(**최소안**), 사용자 결정 2026-10-22(총괄 경유: 후보 1→3 최소안을 PR 하나씩, 확장안은 하지 않음)

**Goal:** 연속 운용 결과를 발표·보고에 쓸 수 있게, 지금 세션의 결과 기록부(계약 5.2)를 완료·실패·취소 수, 사유별 수, 미션 소요 시간으로 요약해 화면에 보인다 — 무엇을 셀 수 있고 무엇을 셀 수 없는지와 함께.

**Architecture:** 차량 쪽(실행기·`ResultsBook`·console/1 계약)은 바꾸지 않는다. 콘솔이 이미 받는 `missions` 메시지(`missions_seq`, `older`, 최근 100 건 `results`)만 순수 모듈(`console/web/js/summary.js`)이 집계하고, 차량 상세의 '미션 결과' 칸 위에 요약을 그린다.

**Tech Stack:** 빌드 없는 ES 모듈, `node --test "console/web/test/*.test.mjs"`, 가짜 차량 + Playwright 화면 확인.

## 1. 범위(최소안)와 하지 않는 것

- 하는 것: 받은 결과 기록부의 결과별 수, 완료율, 사유별 수, 소요 시간(중앙값·최대), 집계 범위 문구.
- 하지 않는 것(조사 문서의 확장안): 세션·하루 누적(세션을 넘는 보관), 실패한 단계·단계별 소요·주행 거리(결과 메시지에 없는 필드 — 계약 추가와 미션 기록 가공 필요), 시간당 처리량, 내보내기(CSV), 여러 대 비교, 차트.

## 2. 설계

**D1 입력.** 콘솔 `ResultsBook`(results.js)이 들고 있는 그 세션의 기록부: `epoch`, `seq`(=`missions_seq`, 세션 안 전체 결과 수), `older`(목록 밖 건수), `recordErrorsOlder`, `list`(최근 100 건, 새것 먼저). 늦게 붙은 화면도 차량이 저장한 목록 전체(최근 100 건)를 받는다 — 접속 뒤의 결과만 세는 것이 아니다. 같은 `missions_seq` 의 재발행(기록 실패 덧붙임)은 목록을 통째로 바꾸므로 두 번 세지 않는다. 결과 항목: `outcome`(`done`·`failed`·`cancelled`·`recovery_required`), `reason`, `started_s`·`ended_s`(**시뮬 시각** — 계약 5.2; 가짜 차량은 자기 시계), `ended_unix`, `pallet_id`, `destination_id`, `map_id`, `record_error`.

**D2 집계(`summarize(book) → summary`).**
- `listed` = 목록 건수, `total` = `seq`, `older`. 결과별 수 `counts`(네 가지, 모르는 값은 `other`).
- 완료율 = `done / listed` — **표시된 종료 결과 중 완료 비율**(취소·복구 필요·준비 중 일시정지로 끝난 `prepare:paused` 실패 포함, 진행 중 미션·거부된 요청 제외). 함께 **취소 제외 완료율** = `done / (listed − cancelled)`. 둘 다 분모 건수를 붙여 보인다(`p % (n 건)`). 분모가 0 이면 없음.
- 사유별 수(**설명별 건수** — 원인 분석이 아니다): `done` 이 아닌 결과를 묶는다. 묶음 키는 `results.js` 에 새로 내보내는 `reasonHead(code)` 의 머리(`REASONS` 의 가장 긴 접두어)가 가리키는 **설명**이고(설명이 같은 두 머리 — 예: `Measured pallet footprint overlap` 과 `… solid overlap` — 는 한 줄), 후보 전부 실패(`prepare:all_candidates_failed:…`)는 후보 상태별 빈도를 **정렬한** 문자열로 묶는다(후보 순서가 달라도 같은 묶음). 머리가 없는 코드는 코드 그대로. 사유 없는 실패·취소는 "사유 없음". 각 줄에 코드별 건수를 함께 둔다(`approach:no_path` 와 `approach:timeout` 처럼 한 설명에 묶인 코드도 각각 몇 건인지 남는다). 많은 순, 같으면 최근 순.
- 팔레트별: `pallet_id` 별 완료/전체(대상 목록의 `kind` 를 알면 함께 — EPAL 6 과 T11 이 한 세션에 섞이면 갈라 볼 수 있게). 지도(`map_id`)별 분할은 하지 않는다(결과 행에 지도가 이미 보인다).
- 소요 시간: **제출 수락부터 종료까지의 경과**(`ended_s − started_s`; 준비·계획 대기·일시정지 포함 — 주행 시간이 아니다), 단위는 "차량 시계 s"(계약 5.2 는 시뮬 시각이라 하고, 가짜 차량은 자기 시계다; Isaac 이 배속 1 미만이면 벽시계로는 더 길다 — 결과에 벽시계 시작 시각이 없어 벽시계 경과는 셀 수 없다). `started_s`·`ended_s` 가 둘 다 유한하고 `ended_s ≥ started_s` 인 결과만, 완료 / 그 밖 두 묶음으로 건수·중앙값·최대. 유효 표본이 0 이면 "—"와 계산 불가 건수(`no_duration`). 가짜 차량의 보통 미션은 `started_s` 가 `null` 이라 화면 확인으로는 시간 표시를 검증할 수 없다 — 단위 시험으로 확인한다.
- 기록 실패: `record_error` 가 있는 결과 수와 `book.recordErrorsOlder` — 수만.

**D3 화면.** '미션 결과' 칸 머리 아래 한 줄 요약과 접이식 상세(`<details>`):
- 한 줄: "완료 a · 실패 b · 취소 c · 복구 필요 d — 완료율 p % (n 건) · 취소 제외 q % (n′ 건) · 완료 경과 중앙값 m s". 결과가 없으면 숨김.
- 상세: 완료율의 분모 설명, 설명별 건수 표(코드별 건수), 팔레트별 완료/전체, 경과 시간(완료 / 그 밖), 기록 실패 수, 집계 범위 문구.
- **집계 범위 문구(늘 보임):** "이 세션(epoch E)만 · 결과 n 건 중 최근 k 건 집계"(older 가 0 이면 "결과 n 건 집계"), "세션이 바뀌면 처음부터 셉니다(장면 초기화 포함) — 세션을 넘는 누적이 아닙니다", older > 0 이면 "앞선 m 건은 현재 목록에 없어 집계에서 제외됩니다"(파일에 저장됐다고 보장하지 않는다 — 기록 쓰기는 비동기이고 큐가 차면 버려질 수 있다, 계약 5.2).
- 기록부가 바뀔 때만 다시 그린다(`book.version`).

## 3. 파일

- Create: `console/web/js/summary.js` — `summarize(book, targets) → summary`; DOM 없음.
- Modify: `console/web/js/results.js` — `reasonHead(code) → {head, text}` 내보내기(기존 `reasonText` 동작은 그대로).
- Create: `console/web/test/summary.test.mjs`
- Modify: `console/web/js/ui.js`(요약 그리기, 기존 결과 목록 그리기 옆), `index.html`(요약 자리), `app.css`, `console/README.md`.

## 4. Task

### Task 1: 집계(순수)

- [ ] **Step 1:** `summary.test.mjs` 를 먼저 쓴다:
  - `"counts each outcome and the done rate over the listed results"` — done 3·failed 1·cancelled 1·recovery_required 1 → 완료율 50 %; 빈 목록 → 완료율 없음.
  - `"groups reasons by their meaning, most first"` — 머리가 같은 두 코드가 한 줄이고 코드별 건수가 남음; 사유 없는 취소는 "사유 없음"; done 의 사유는 세지 않음.
  - `"candidate failures group by their sorted status counts"` — `…:invalid_start;timeout` 과 `…:timeout;invalid_start` 가 한 줄.
  - `"the done rate also without cancellations, with its denominators"` — done 1·cancelled 1 → 50 %(2 건)·취소 제외 100 %(1 건); 전부 취소 → 취소 제외 없음.
  - `"per pallet done over total, with the kind when the targets know it"`.
  - `"durations use only finite start and end, done and the rest"` — `started_s` null·`ended_s < started_s` 는 `no_duration`; 완료 [10, 20, 40] → 중앙값 20·최대 40; 짝수 개 중앙값은 가운데 둘의 평균; 유효 0 건 → `null`.
  - `"the coverage says what is left out"` — seq 130·older 30·list 100 → total 130·listed 100·older 30, `recordErrorsOlder` 를 그대로; older 0 → 전부; epoch 를 싣는다.
  - `"an unknown outcome is counted as other"`.
- [ ] **Step 2:** 실패 확인. **Step 3:** 구현. **Step 4:** 통과 확인, 커밋 `console: a summary of the session's results book`.

### Task 2: 화면

- [ ] **Step 1:** ui.js·index.html·app.css 에 D3.
- [ ] **Step 2:** 화면 확인(가짜 차량, Playwright): ① 결과 없음 → 요약 숨김 ② 운반 하나 완료 → "완료 1 … 완료율 100 % (1 건)", 경과는 "—"(가짜 차량은 `started_s` 가 없음) ③ 진행 중 실패 주입·취소 → 수와 설명별 표, 취소 제외 완료율 ④ 장면 초기화(새 세션) → 요약 비움 ⑤ 1600·400 px 가로 넘침 없음.
- [ ] **Step 3:** README, 커밋 `console: results summary above the results list`.

### Task 3: 검증과 PR

- [ ] 웹 시험 전체, ws1 `forklift-ws1-pytest tests/unit/console tests/integration/console`, Codex 리뷰 수렴, PR(merge commit), 총괄 보고.

## 5. 자체 검토

**① 원자료 재대조(Claude).** 계약 5.2: `missions_seq`·`seq` 는 세션 안에서만 증가, 새 실행(장면 초기화 포함)은 1 부터; `results` 최근 100 건, 넘친 수는 `older`; `started_s`·`ended_s` 는 시뮬 시각, `ended_unix` 는 벽시계; `record_error`·`record_errors_older`. `src/forklift_core/mission/results.py`: `OUTCOMES` 네 가지, `KEEP = 100`, `message()` 는 `seq > len` 일 때만 `older`, `started_s` 는 `None` 일 수 있음. 콘솔 `ResultsBook`: `seq`=`missions_seq`, `list` 는 `seq` 내림차순, 에폭이 바뀌면 비움. 가짜 차량도 같은 `ResultsBook` 형식(`_book`)을 쓴다. 고친 것: 소요 시간 단위를 "시뮬 s"로(계약이 시뮬 시각이라고 적음).

**② Codex 적대적 검토 1회차(새 스레드).** P1 없음, P2 2·P3 2 — 소스로 확인해 2판에 반영: 목록 밖 결과가 파일에 있다고 단정함(계약 5.2 — 쓰기는 비동기, 큐가 차면 버려짐) → "현재 목록에 없어 제외"로만, 입력은 `book.recordErrorsOlder` / 설명 문자열로 묶으면 후보 순서만 다른 같은 실패가 갈리고 한 설명에 묶인 코드의 건수가 사라짐 → 머리(`reasonHead`)로 묶고 후보 실패는 정렬한 상태 빈도로, 코드별 건수 유지, "설명별 건수"로 표기. P3: 경과 시간은 제출 수락부터(대기·일시정지 포함), 가짜 차량 보통 미션은 `started_s` 가 `null` → 유효 0 건이면 "—", 시간 표시는 단위 시험으로 / 완료율의 분모(취소·복구 필요 포함, 진행 중·거부 제외)를 화면에 설명. 확인된 경계: 같은 `missions_seq` 재발행은 두 번 세지 않음, 늦게 붙은 화면도 저장된 목록 전체를 받음, Isaac 시작·종료는 같은 시뮬 시계.

**③ agy 독립 검토(Gemini 3.8 Flash — `agy-quota-ok --pick` 이 Argon 을 보류해 Flash; 1·2 의 결론은 보여주지 않음).** 5건 중 4건 반영, 1건 반영 안 함: 한 세션에 시험 조건(팔레트 규격)이 섞이면 완료율이 결함을 가림 → 팔레트별 완료/전체(대상 목록의 `kind`) / 운용자 취소가 분모에 들어가 완료율을 깎음 → 취소 제외 완료율 병기 / 표본이 작으면 백분율이 오해를 부름 → 분모 건수를 늘 붙임 / 세션 한정 휘발성 집계를 누적으로 오독 → "이 세션(epoch E)만 · 세션을 넘는 누적이 아님" / 시뮬 시간을 현실 처리량으로 오독(반영 안 함) → 결과에 벽시계 시작 시각이 없어 미션별 벽시계 경과를 셀 수 없으므로, 단위 문구에 "배속 1 미만이면 벽시계로는 더 김"만 적는다.

**② Codex 2회차(계획 2판 + Task 1).** 새 P1/P2 없음(수렴). P3 1건 반영: 설명이 같은 서로 다른 머리가 두 줄로 나옴 → 묶음 키를 머리의 설명으로(시험 `two table heads with one description make one line`).

**② Codex 3회차(화면).** P1 없음, P2 2·P3 1 반영: 늘 보여야 할 집계 범위 문구가 접힌 상세 안에 있음 → 상세 밖 `sum-scope` 로 / 기존 결과 머리의 "앞선 N건은 기록 파일에만" 문구가 파일 보존을 단정 → "현재 목록에 없습니다" / 계산 불가 원인을 "시작 시각 없음"으로 단정 → "시각 없음 또는 유효하지 않은 구간".

**② Codex 4회차.** 새 P1/P2/P3 없음 — 수렴.
