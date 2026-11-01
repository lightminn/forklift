# 복귀 출발 후진을 막힌 시작까지 넓힘 — F2′′ (2026-11-02, 2 판)

지시: 총괄(2026-11-02). 진단 `docs/validation/2026-11-02-release-replan-failures.md`(PR #245)의 대안 ② 로 간다. F2′ 는 지금 등록하지 않는다.
정책(사용자 2026-10-24): 복구 로직·런처 기본값은 사용자 결정 없이 정하고 문서·보고로 남긴다.
바탕: F2′ 계획 `docs/plans/2026-10-27-return-start-withdraw-extension.md`(3 판, R2–R6·C1–C5). 이 계획은 그 위에 두 가지만 더한다.
branch `plan/return-start-backoff-boxed`.

## 1. 문제

복귀 cusp 배치 `rc_95e61a74` 의 하역 뒤 해제 재계획 실패 10 건은 같은 자리의 두 단계다(진단 문서).
- 시작의 정확한 여유가 0.10 m 아래면 `invalid_start` 다(7 건). F2′ 가 발동한다.
- 그보다 조금 크면 시작 검사는 통과하지만, 모든 시도가 노드 1 개만 펼친 `no_path` 다(3 건, "막힌 시작"). F2′ 는 발동하지 않는다.
- 발동 조건만 넓혀도 F2′ 의 후진 길이(시작 검사가 통과하는 첫 거리 + 0.03 m)는 막힌 시작에서 0.04 m 이고, 거기서도 막힌다.

## 2. 범위

넓히는 것은 둘뿐이다.
1. 발동 조건: `invalid_start` 에 더해, 모든 탐색 시도가 노드 1 개만 펼친 `no_path` 에서도 발동한다.
2. 후진 길이: 후보 거리의 끝 자세를 "탐색이 거기서 첫걸음을 뗄 수 있는가"로 검사한다. 검사는 탐색 자신의 함수와 값을 그대로 호출한다.

그대로 두는 것: 후진 상한 0.30 m, 간격 0.01 m, 여분 0.03 m, 직선 검사(슬롯 소품 대비 `straight_clearance_m`), 채택 규칙(`prefix`·`unmet:*`), 기록 형식, F2′ 의 시험·C5. 하역 가로 정밀도(진단 ③)는 범위 밖이다.

## 3. 설계

### D1 옵션과 범위

- 러너 `--return-start-backoff on|boxed|off`(기본 off). `on` 은 F2′ 그대로이고 `boxed` 가 F2′′ 다. `boxed` 는 `on` 의 발동(`invalid_start`)에 D2 의 발동을 더하고, 두 발동 모두에 D3 의 끝 검사와 D6 의 실패 처리를 쓴다.
- 계획기 키워드는 둘이다. `return_start_backoff_boxed: bool = False` 와, 끝 검사가 쓸 이동 config(아래 D3)다. 이것을 `plan_return_leg`·`plan_transport`·`plan_loaded_delivery` 와 그 아래 `_return_search`·`_backed_off`·`return_start_backoff` 에 넣는다.
  - 지금 `_backed_off`·`return_start_backoff` 는 여유 스칼라만 받는다. 그래서 `boxed` 에서는 그 호출이 실제로 탐색에 넘기는 이동 config 객체(`travel_config`, `_return_search` 의 `config`)를 그대로 넘긴다. 기본 `PlannerConfig()` 를 새로 만들지 않는다(agy ①: 기본값은 곡률 0.5·동작 0.5 m 라 실제 탐색 0.29·0.25 와 다르다).
- 미션 코어 옵션 `return_start_backoff_boxed` 가 참일 때만 `_mission_backoff_kwargs`·`_return_backoff_kwargs` 가 키를 넣는다. `off`·`on` 의 계획 호출 kwargs 는 지금과 같다(경계 기록·재생 호환).
- 적용되는 호출(F2′ R2 와 같다, Codex 1 차 확인): `plan_transport`·`plan_loaded_delivery` 의 빈 차 `return_home`, 하역 해제 재계획(동기·작업자), 첫 live 복귀. 정지·멈춤·cusp·장면 재계획에는 넣지 않는다. 호출별로 키가 들어가고 빠지는지 시험한다.
- 작업자(`plan_worker`)는 kwargs 를 그대로 넘긴다. 피클 왕복 시험을 더한다.

### D2 막힌 시작의 판정

- 모두 참일 때만 막힌 시작이다(Codex 1 차 P2-1):
  - 최종 `direct.status == "no_path"`.
  - `direct.search_attempts` 가 비어 있지 않다.
  - 모든 시도의 상태가 정확히 `"no_path"` 이고 펼친 노드 수(튜플 세 번째 값)가 정확히 1 이다. 꼬리의 `timeout`·`no_path:late` 가 하나라도 섞이면 아니다(`_search` 는 실패·지각한 꼬리 시도도 `search_attempts` 에 남기고 이전 결과를 돌려준다).
- 그래서 사다리의 어느 시도(기본 동작 0.10 m 시도 포함)든 노드를 2 개 이상 펼쳤으면 발동하지 않는다(agy ②-1). 그 경우는 시작 둘레에 첫걸음 자리가 있는데 그 너머가 막힌 것이라 이 기능의 범위 밖으로 둔다. 기록된 3 건은 8 시도 모두 노드 1 이다.
- 마감(deadline)·`SearchBudget` 이 있는 호출에서 사다리가 마감에 걸리면 `timeout` 이라 발동하지 않는다. 막힌 시작은 사다리를 끝까지 돈 뒤에야 판정되므로, 해제 재계획처럼 마감이 없는 호출에서 주로 작동한다(agy ②-2). 후진 끝 탐색의 예산은 지금 F2′ 와 같이 그 호출 자신의 것이다.
- `forward_turnaround` 를 켠 호출도 실패한 direct 를 그대로 돌려주므로(`_return_turnaround`) 같은 판정을 쓴다. 이 결합을 시험한다(agy 시험 ⑥).

### D3 후진 끝 검사 — 탐색 자신의 첫걸음

- 지금(F2′): 거리 d 의 끝 자세가 복귀의 시작 검사(`checker.free(end, clearance)`)를 통과하고, 그 직선이 슬롯 소품에서 여유를 지키면 통과다.
- `boxed`: 시작 검사에 더해, 탐색이 거기서 충돌 검사를 통과하는 기본 동작 자식을 하나라도 만들 수 있어야 통과다.
  - hybrid_astar 에 `first_step_free(pose, checker, config) -> bool` 을 둔다. 방위를 `_wrap` 한 뒤(`plan_hybrid_astar` 와 같다, agy ⑤), 시작 검사 `checker.free(pose, config.clearance_m)` 와, 펼침 고리가 도는 여섯 기본 동작(`PRIMITIVES`, 전진·후진 × 좌·직·우)을 같은 `_sample_arc` 로 돈다.
  - `PRIMITIVES` 는 펼침 고리와 이 함수가 같이 쓰는 모듈 상수로 뺀다.
  - 중간점 부풀림 `clearance + ds/2·(1 + R·|κ|)` 는 `sweep_margin_m(config, radius_m, curvature, ds)` 로 빼고 `_sample_arc` 가 그것을 쓴다. 상수를 베끼지 않는다.
  - config 는 사다리의 막힌 시작용 대안 시도 config 다. `boxed_fallback_config(config)` 로 빼서 `_search` 와 같이 쓴다. 기존 단계가 `FALLBACK_BOXED_COLLISION_STEP_M` 보다 작거나 같으면 config 를 그대로 둔다(`_search` 의 조건 그대로, Codex 1 차 P2-2·agy ①). 시작 config 로는 막혔는데 대안 config 로만 풀리는 끝도 받는다. 기본 시도가 노드 1 개면 사다리가 그 대안을 실제로 돌리기 때문이다(Codex 확인).
- 계약은 "충돌 검사를 통과하는 기본 동작이 있다"이다. 펼친 노드 수와의 동등성은 유효한 목표, root shot 이 막힘, 충분한 예산, `turn_cycles` 꺼짐일 때만 성립한다(Codex P3-4·agy ⑤: 열린 자세에서는 root shot 으로 노드 1 개 성공이 난다). 기록된 6 건은 모두 `turn_cycles` 꺼짐이다. 켜진 config 에서 기본 동작 자식이 없고 turn cycle 만 있으면 막힌 것으로 보고 더 후진한다(보수적).
- 길이: 조언(첫 통과 거리)은 F2′ 보다 같거나 크다. 최종 길이 `min(조언 + 0.03, last, 0.30)` 은 연속 통과 구간의 끝(`last`)에 묶이므로, 엄격한 검사에서는 **F2′ 보다 짧거나 `unmet:clipped` 가 될 수 있다**(Codex P2-3 반례: `on` 0.20 m, `boxed` 0.18 m; agy ③). 1 판의 "길이 ≥ on" 은 틀렸다.
  - F2′ 의 끝이 첫걸음 검사에서 막히면, 위 동등성 아래서 그 끝의 탐색도 노드 1 개로 끝난다. 그래서 `boxed` 가 `on` 이 회복하던 경우를 잃는지는 F2′ 의 기존 회복 사례(F2′ 계획 C2 네 건과 그 시험 고정 장면)를 `boxed` 로 다시 돌려 확인한다(§4).
  - 채택 규칙(`prefix`·`unmet:clipped`·`unmet:no_advice`)과 간격·여분·상한은 바꾸지 않는다.

### D4 진짜 갇힘은 그대로 거부

- 뒤가 막혀 있으면(후진 직선이 슬롯 소품·경계·격자에 걸림) 어느 d 도 통과하지 않는다. 기록은 `unmet:no_advice` 이고 결과는 direct 다. 지금 F2′ 와 같다.
- 뒤는 열렸지만 0.30 m 안에서 첫걸음을 뗄 자리가 없어도 `unmet:no_advice` 다.
- 첫걸음 자리가 한 칸만 열렸다 닫히면 `unmet:clipped` 다(agy ③-2). 거부지만 더 위험한 동작은 아니다.
- Codex 가 두 합성 장면(뒤가 막힘, 뒤는 열렸지만 자리 없음)에서 실제 사다리 `no_path/1`·D3 통과 후보 없음을 확인했다. 시험으로 고정한다.

### D5 기록

- `start_backoff` 기록에 `trigger`(`invalid_start`·`boxed`)와 `check`(`start`·`first_step`)를 더한다. `boxed` 일 때만 넣어 `on` 의 기록은 지금과 같다.
- 미션 상태 `return_start_backoffs`(R6)를 만드는 `note_return_backoff` 는 지금 다섯 필드만 고른다. `boxed` 의 두 필드가 있으면 그것도 옮긴다(Codex P3-5·agy ⑥-2). 없으면 지금과 같다.

### D6 실패한 후진은 호출자의 대안을 지우지 않는다

- 지금 F2′ 는 후진 끝 탐색이 실패하면 그 실패(`rest`)를 돌려준다. 그러면 원래 `invalid_start` 가 `no_path` 로 바뀐다. 해제 재계획의 격자 모드 여유 0 재시도는 `invalid_start` 일 때만 돌므로 그 대안이 사라진다(agy ⑥-1, `core.py` 해제 재계획).
- `boxed` 에서는 후진 끝 탐색이 실패하면 direct 의 상태를 유지하고, 기록에 `unmet:search_<status>` 를 남긴다. `on` 은 바꾸지 않는다.

### D7 짧은 후진 뒤의 추종

- 후진 직선(0.07–0.08 m)과 복귀 경로의 첫 구간이 같은 기어(후진)면 그 이음에서 서지 않는다. 그러면 추종기(lookahead 0.28 m)가 L 보다 먼저 조향을 시작한다(agy ④). F2′ 의 시험(`test_the_recorded_backed_off_return_is_followed_home_clear_of_the_slot_props`)이 같은 문제를 이상 자전거 추종으로 쟀다.
- 기록 6 건의 F2′′ 경로를 같은 방법으로 추종해, 그 궤적의 외곽 여유를 잰다(§5). 막는 것이 실제 기둥이라(F2′ 사례는 부풀린 추정) 슬롯 소품과 복귀 소품 모두에 대해 잰다.
- 판정(구현 전 고정): 추종 궤적의 L + 0.30 m 안에서 슬롯 소품 여유 ≥ 계획 여유 0.10 m, 복귀 소품 여유 > 0. 못 미치면 구현 전에 설계를 바꾼다. 후보는 이음에서 서게 하는 것(같은 기어여도 정지점)이나 L 을 늘리는 것이다.

## 4. 시험 (구현 전 고정)

- 단위(계획기):
  - `sweep_margin_m` 이 `_sample_arc` 가 실제로 `checker.free` 에 넘기는 값과 같다. 기록하는 가짜 검사기로 직진·최대 곡률을 보고, 기존 단계 0.005·0.01·0.06 m 세 config 에서 확인한다(Codex P2-2).
  - `boxed_fallback_config` 가 `_search` 의 대안 시도 config 와 같다(같은 세 단계, 사다리 시도 기록의 collision step). 0.005 m 는 그대로 둔다.
  - `first_step_free` 와 대안 config `plan_hybrid_astar` 의 펼친 노드 수 > 1 의 동등성. 유효 목표, root shot 이 막힌 목표, 충분한 예산, `turn_cycles` 꺼짐을 전제로 막힌 시작·무효 시작·첫걸음만 열린 장면에서 본다. 열린 자세의 root shot 성공(노드 1)은 동등성의 예외로 따로 고정한다.
  - 방위가 ±π 를 넘는 자세에서도 같은 판정(`_wrap`).
  - 발동(D2): 막힌 `no_path` 는 발동한다. 다음은 발동하지 않는다.
    - 노드 2 개 이상인 `no_path`(0.10 m 동작 시도만 펼친 경우 포함)
    - 최종 `timeout`
    - `no_path/1` 시도에 `timeout/0` 이나 `no_path:late/1` 이 섞인 이력(Codex 재현)
    - `invalid_goal`
    - `on` 에서의 막힌 `no_path`(F2′ 그대로)
  - 진짜 갇힘 반례 둘(뒤가 막힘, 뒤는 열렸지만 0.30 m 안에 자리 없음)에서 `unmet:no_advice`, 결과는 direct 와 같다. 한 칸만 열린 장면은 `unmet:clipped` 다.
  - 길이: 조언은 `boxed` ≥ `on` 이다. Codex 반례(뒤쪽 막대 하나)에서 `boxed` 의 최종 길이가 `on` 보다 짧다.
  - F2′ 의 기존 회복 사례(F2′ 계획 C2 네 건, 기존 시험 고정 장면)를 `boxed` 로 다시 돌려 모두 회복하는지 본다. 못 하면 D3 을 고친다.
  - D6: `boxed` 에서 후진 끝 탐색 실패가 direct 의 `invalid_start` 를 유지한다. `on` 은 지금과 같다.
  - D7: 기록 6 건과 같은 기하의 고정 장면에서 추종 궤적 여유(위 판정).
  - `forward_turnaround=True` + 막힌 시작에서 후진 끝의 turnaround 평가·이음.
  - `off`·`on` 의 결과·기록이 바뀌지 않는다(기존 F2′ 시험 그대로 통과).
- 단위(코어·러너):
  - `boxed` 일 때만 계획 kwargs 에 키가 있고, 적용 호출에만 들어간다(D1 목록). `off`·`on` 은 지금과 같다.
  - `note_return_backoff` 가 `boxed` 의 두 필드를 옮긴다.
  - 작업자 피클 왕복. 러너 인자를 근접 계약 목록(`near_tracking.py`)에 넣는다.
- 재호출: 기록 있는 6 건(`rc_95e61a74` G s1·s8, GN s3·s10·s11·s13)의 실패한 `plan_return_leg` 를 `boxed` 로 다시 불러 모두 성공하는지, 후진 길이가 §5 와 같은지 본다. A 3 건은 Codex 의 `result.json` 재구성 결과로 둔다(경계 기록이 없다).
- 재생 게이트 runs29 29/29(옵션 기본 off). ws1 전체 시험.

## 5. 재계산 (원자료, 계획 ①)

기록 있는 6 건의 실패한 `plan_return_leg` 요청을 경계 기록에서 그대로 꺼내, 스냅숏 `rc_95e61a74` 의 계획기 함수로 다시 쟀다(ws1 `tss_diag/probe_release4.py`·`probe_release5.py`, 잡 14018·14024, 로컬 사본 `tmp/rx/rel/all4.jsonl`·`all5.jsonl`).

- 검사기 상수: 발자국 반지름 R = 1.33929 m(`checker.radius_m`), 여유 0.10 m, 막힌 시작용 단계 0.01 m(`FALLBACK_BOXED_COLLISION_STEP_M`), 기본 동작 0.25 m, 곡률 0.29 1/m. 첫 중간점 부풀림은 직진 0.105 m, 최대 곡률 0.10694 m 다. 6 건 모두 `turn_cycles` 꺼짐, `forward_turnaround` 없음, 격자 없음.
- 시작에서 검사기가 받아들이는 최대 팽창(이분법, `checker.free`):

| 실행 | 유형 | 정확한 여유 | 최대 팽창: 0 | 0.005 m 뒤 | 0.04 m 뒤 | 0.05 m 뒤 |
|---|---|---|---|---|---|---|
| G s1 | no_path | 0.1137 | 0.10130 | 0.10134 | 0.10160 | 0.10168 |
| G s8 | no_path | 0.1157 | 0.10049 | 0.10052 | 0.10070 | 0.10697 |
| GN s3 | invalid_start | 0.1070 | 0.09001 | 0.09001 | 0.09791 | 0.10788 |
| GN s10 | invalid_start | 0.0934 | 0.07840 | 0.07842 | 0.09052 | 0.10052 |
| GN s11 | invalid_start | 0.1026 | 0.08371 | 0.08374 | 0.09909 | 0.10909 |
| GN s13 | invalid_start | 0.1025 | 0.08056 | 0.08061 | 0.10320 | 0.11320 |

  - no_path 두 건은 시작이 0.10 은 넘지만 0.105 에 못 미친다(Codex 진단 검토의 0.1013 / 0.1005 와 같다).
  - G s1 은 곧게 후진해도 허용 팽창이 0.05 m 에서 0.1017 로만 는다. 기둥이 옆에 있어서다. 그래도 0.05 m 에서 첫걸음이 생기는 것은 곡선 동작이다(아래).
- `first_step_free` 시제품(여섯 기본 동작을 `_sample_arc` 로, 대안 config)과 대안 시도 `plan_hybrid_astar` 의 펼친 노드 수를 d = 0, 0.01, …, 0.06, 0.08, 0.10 m 의 54 칸에서 견줬다. 54 칸 모두 "참 ⇔ 펼친 노드 > 1" 이다.
- 넓힌 조언(첫 통과 거리)·후진 길이(+0.03)·그 끝의 복귀:

| 실행 | 지금 F2′ 조언 → 길이 | 넓힌 조언 → 길이 | 끝에서 복귀 |
|---|---|---|---|
| G s1 | (발동 안 함; 넓히면 0.01 → 0.04, 막힘) | 0.05 → 0.08 | 성공 17.10 m |
| G s8 | (발동 안 함; 넓히면 0.01 → 0.04, 막힘) | 0.05 → 0.08 | 성공 25.08 m |
| GN s3 | 0.05 → 0.08 | 0.05 → 0.08 | 성공 17.57 m |
| GN s10 | 0.05 → 0.08 | 0.05 → 0.08 | 성공 16.62 m |
| GN s11 | 0.05 → 0.08 | 0.05 → 0.08 | 성공 15.98 m |
| GN s13 | 0.04 → 0.07 | 0.04 → 0.07 | 성공 15.98 m |

  - `invalid_start` 4 건은 넓힌 검사에서도 길이가 같다. 0.10 을 처음 넘는 거리에서 이미 첫걸음이 생기기 때문이다.
  - 후진 끝 복귀의 성공은 거리에 대해 단조가 아니다. G s1 은 0.06 m 끝에서, G s8 은 0.10 m 끝에서 노드 1 439–1 441 개를 펼친 `no_path` 다(막힌 것은 아니다). F2′′ 는 첫 통과 거리 + 0.03 하나만 시도하므로 이런 거리를 고르면 지금처럼 `unmet:search_no_path` 로 끝난다.

## 6. 비교 배치

- 구현·검증 뒤 따로 등록한다. 팔은 `off` 대 `boxed` 다(Codex P2-3). 노출은 aisle 10/48 을 바탕으로 다시 센다(진단 문서).
- F2′ C5(끔보다 완주가 줄지 않음, 네 단계 노출: 시도·채택·후진 완료·복귀 완료)를 넓힌 경로에도 그대로 쓴다. 이것은 `off` 대비 비감소이고, `on` 이 회복하던 경우를 `boxed` 가 지키는지의 증명은 아니다. 그것은 §4 의 F2′ 회복 사례 재실행이 맡는다.

## 7. 자체 검토

- ① 원자료 재계산(나): §5. 기록 6 건의 실패 요청을 그대로 다시 불러, 최대 허용 팽창(0.1013 / 0.1005 등), 부풀림 0.105 / 0.10694, 조언·길이·끝 복귀, 54 칸 판정 일치를 쟀다. Codex 진단 검토의 수치와 같다. 추종 궤적 여유(D7)는 잡 14049 로 잰다.
- ② Codex 적대 검토 1 차(e7bfde4d, `tmp/rx/codex_f2pp1.out`): P1 없음, P2 3·P3 2.
  - P2 꼬리 `timeout`·`:late` 가 섞인 이력도 D2 에 걸림 → 모든 시도의 상태 `no_path`·노드 정확히 1.
  - P2 대안 config 를 무조건 0.01 로 바꾸면 탐색보다 엄격해짐 → `_search` 의 조건 그대로, 0.005·0.01·0.06 시험.
  - P2 "길이 ≥ on" 은 틀림(반례 0.20 / 0.18) → 조언만 비감소, F2′ 회복 사례 재실행, 비교 팔 `off` 대 `boxed`.
  - P3 첫걸음 ⇔ 노드 수 > 1 은 일반 계약이 아님 → 전제를 적고 root shot 예외를 따로.
  - P3 `note_return_backoff` 가 새 필드를 버림 → 옮긴다.
  - 확인: 적용 호출 범위, D4 두 합성 장면, §5 수치·54 칸 재현.
- ③ 독립 검토(agy, Gemini 4 Argon, 결론을 보여 주지 않음 — 배경 사실과 설계 절만 넘김, `tmp/rx/agy_f2pp1.out`):
  - ① 이동 config 를 넘겨야 함 → D1.
  - ② 0.10 m 동작 시도·마감 → D2 에 범위로 적음.
  - ③ `last`·`unmet:clipped` 역전 → D3 길이, D4.
  - ④ 짧은 후진 뒤 같은 기어 이음에서 추종기가 먼저 조향 → D7(가장 큰 새 지적, 구현 전 판정).
  - ⑤ root shot·방위 `_wrap` → D3·§4.
  - ⑥ 실패한 후진이 `invalid_start` 를 지워 여유 0 재시도가 사라짐 → D6. 새 필드 유실 → D5.
  - 시험 빈틈 ①–⑥ 은 §4 에 넣었다.
