# 복귀 출발 후진을 막힌 시작까지 넓힘 — F2′′ (2026-11-02, 1 판)

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

### D1 옵션

- 러너 `--return-start-backoff on|boxed|off`(기본 off). `on` 은 F2′ 그대로이고, `boxed` 가 F2′′ 다. `boxed` 는 `on` 의 모든 동작을 하고 아래 D2·D3 을 더한다.
- 계획기 키워드 `return_start_backoff_boxed: bool = False` 를 `plan_return_leg`·`plan_transport`·`plan_loaded_delivery`(와 그 아래 `_return_search`·`_backed_off`·`return_start_backoff`)에 넣는다. 미션 코어 옵션 `return_start_backoff_boxed` 가 참일 때만 `_mission_backoff_kwargs`·`_return_backoff_kwargs` 가 이 키를 넣는다. 그래서 `off`·`on` 의 계획 호출 kwargs 는 지금과 같다(경계 기록·재생 호환).
- 작업자(`plan_worker`)는 kwargs 를 그대로 넘기므로 따로 바꿀 것이 없다. 피클 왕복 시험만 더한다.

### D2 막힌 시작의 판정

- `direct.status == "no_path"` 이고, `direct.search_attempts` 가 비어 있지 않으며, 모든 시도의 펼친 노드 수(튜플 세 번째 값)가 1 이하다.
- `timeout`·`:late`·`invalid_goal`·노드를 2 개 이상 펼친 `no_path` 는 발동하지 않는다. 그런 실패는 공간이나 예산의 문제라 후진이 고칠 자리가 아니다.
- `forward_turnaround` 를 켠 호출도 실패한 direct 를 그대로 돌려주므로(`_return_turnaround`), 같은 판정을 쓴다.

### D3 후진 끝 검사 — 탐색 자신의 첫걸음

- 지금(F2′): 거리 d 의 끝 자세가 복귀의 시작 검사(`checker.free(end, clearance)`)를 통과하고, 그 직선이 슬롯 소품에서 여유를 지키면 통과다.
- `boxed`: 시작 검사에 더해, **탐색이 거기서 자식을 하나라도 만들 수 있어야** 통과다.
  - hybrid_astar 에 `first_step_free(pose, checker, config) -> bool` 을 둔다. `plan_hybrid_astar` 가 시작에서 쓰는 검사(`checker.free(pose, config.clearance_m)`)와, 펼침 고리가 도는 여섯 기본 동작(전진·후진 × 좌·직·우, `PRIMITIVES`)을 같은 `_sample_arc` 로 돈다.
  - `PRIMITIVES` 는 펼침 고리와 이 함수가 같이 쓰는 모듈 상수로 뺀다(지금 고리 안의 `(1, -1)` × `(0, -1, 1)`).
  - 중간점 부풀림 `clearance + ds/2·(1 + R·|κ|)` 는 `_sample_arc` 안에서 `sweep_margin_m(config, radius_m, curvature, ds)` 로 빼고, `_sample_arc` 가 그것을 쓴다. 상수를 베끼지 않는다.
  - config 는 사다리의 막힌 시작용 대안 시도 config 다. 그 자체를 `_search` 와 이 검사가 같이 쓰는 함수 `boxed_fallback_config(config)` 로 뺀다(지금은 `_search` 안의 `replace(config, collision_step_m=FALLBACK_BOXED_COLLISION_STEP_M)`).
  - 이 config 에서 자식이 생기면 그 시도는 노드를 2 개 이상 펼치므로, 그 끝에서는 사다리가 막힌 채 끝나지 않는다. 경로가 있다는 보장은 아니다.
- `turn_cycles` 는 기록된 이동 config 에서 꺼져 있다(이 6 건). 켜진 config 에서도 기본 동작 자식이 있으면 펼침이 일어나므로 이 검사는 보수적이다(통과면 펼친다). 반대(기본 동작은 없고 turn cycle 만 있음)는 막힌 시작으로 보고 더 후진한다.
- 이 검사는 `invalid_start` 와 막힌 시작 두 발동 모두에 쓴다. 그래서 `boxed` 에서 `invalid_start` 의 후진 길이는 F2′ 와 같거나 길다(더 엄격한 검사의 첫 통과 거리 + 0.03).

### D4 진짜 갇힘은 그대로 거부

- 뒤가 막혀 있으면(후진 직선이 슬롯 소품·경계·격자에 걸림) 어느 d 도 통과하지 않는다. 기록은 `unmet:no_advice` 이고 결과는 direct(`no_path`) 그대로다. 지금 F2′ 와 같다.
- 뒤는 열렸지만 0.30 m 안에서 첫걸음을 뗄 자리가 없으면 역시 `unmet:no_advice` 다.
- 후진 끝에서 탐색이 실패하면 지금처럼 그 실패와 `unmet:search_<status>` 기록이다.

### D5 기록

- `start_backoff` 기록에 `trigger`(`invalid_start`·`boxed`)와 `check`(`start`·`first_step`)를 더한다. `boxed` 일 때만 넣어 `on` 의 기록은 지금과 같다.
- 미션 상태 `return_start_backoffs`(R6)는 그대로 그 기록을 덧붙인다.

## 4. 시험 (구현 전 고정)

- 단위(계획기):
  - `sweep_margin_m` 이 `_sample_arc` 가 실제로 `checker.free` 에 넘기는 값과 같다(기록하는 가짜 검사기로 직진·최대 곡률 두 경우, 대안 config 와 기본 config).
  - `first_step_free` 가 `plan_hybrid_astar`(대안 config) 의 펼친 노드 수 > 1 과 같다. 열린 자세, 막힌 시작, 무효 시작 세 고정 장면에서 본다.
  - `boxed_fallback_config` 가 `_search` 의 대안 시도 config 와 같다(사다리 시도 기록의 collision step).
  - 발동: 막힌 `no_path` 는 발동, 노드 2 개 이상 `no_path`·`timeout`·`invalid_goal` 은 발동 안 함, `on` 에서는 막힌 `no_path` 에 발동 안 함(F2′ 그대로).
  - 진짜 갇힘 반례 둘(뒤가 막힘 → `unmet:no_advice`, 뒤는 열렸지만 0.30 m 안에 첫걸음 자리 없음 → `unmet:no_advice`)에서 결과가 direct 와 같다.
  - `boxed` 의 `invalid_start` 후진 길이 ≥ `on` 의 것.
  - `off`·`on` 의 결과·기록이 바뀌지 않는다(기존 F2′ 시험 그대로 통과).
- 단위(코어·러너): `boxed` 일 때만 계획 kwargs 에 키가 있고 `off`·`on` 은 지금과 같다. 작업자 피클 왕복. 러너 인자를 근접 계약 목록(`near_tracking.py`)에 넣는다.
- 재호출: 기록 있는 6 건(`rc_95e61a74` G s1·s8, GN s3·s10·s11·s13)의 실패한 `plan_return_leg` 를 `boxed` 로 다시 불러 모두 성공하는지, 후진 길이가 아래 §5 와 같은지 본다. A 3 건은 Codex 의 `result.json` 재구성 결과로 둔다(경계 기록이 없다).
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

- 구현·검증 뒤 따로 등록한다. 노출은 aisle 10/48 을 바탕으로 다시 센다(진단 문서). F2′ C5(끔보다 완주가 줄지 않음, 네 단계 노출: 시도·채택·후진 완료·복귀 완료)를 넓힌 경로에도 그대로 쓴다.

## 7. 자체 검토

(채운다.)
