# 복귀 cusp 재계획 계획 (2026-10-31, 1 판 초안)

지시: 총괄(2026-10-31) 후보 1 — 진단 → 계획(검토 3 회) → 수정 → 게이트·비교 배치. 총괄 보탬: 곡률 여유(다)를 대조 후보로 넣고 오프라인 재호출로 잰다.
진단: `docs/validation/2026-10-31-return-cusp-diagnosis.md`.
정책(사용자 2026-10-24): 복구 로직·런처 기본값은 사용자 결정 없이 정하고 문서·보고로 남긴다.

## 1. 무엇을 바꾸는가

- (가) 복귀 cusp 재계획: 복귀 단계 cusp 의 `endpoint_heading` 실패를, 운반 cusp 재계획처럼 서서 지금 자세에서 복귀 경로를 다시 계획해 이어 간다. 지금은 실행이 끝난다.
- (나) 운반 cusp 재계획 예산: 실행당 2 회를 3 회로 늘릴지 비교 배치의 별도 팔로 본다.
- (다) 복귀 계획 곡률 한계: 0.29 → 0.26 1/m 를 오프라인 재호출로 잰다. 이 계획에서 채택하지는 않는다. 측정 결과로 (가)만으로 충분한지 판단할 근거를 남긴다.

## 2. 근거 (진단 요약)

- riib 배치 2 의 cusp 방위 실패 6 건 = 복귀 4(복귀에는 cusp 재계획이 없음) + 운반 2(예산 2 회 소진 뒤).
- 운반 cusp 재계획: 33 회 모두 계획 성공, 그것을 쓴 23 실행 중 21 이 운반 단계를 넘겼다. 예산 2 회를 다 쓴 실행은 10 건(모두 aisle)이고, 그중 2 건이 세 번째 cusp 에서 실패했다.
- 계획된 복귀 경로 자세의 70 % 가 계획 곡률 한계 0.29 에, 추종 명령은 복귀 표본의 30 % 가 조향 한계 0.36 에 붙는다. 보정 여유가 0.07 1/m 다.

## 3. 설계

### D1 (가) 복귀 cusp 재계획

- 발동 조건(운반과 같음, 단계만 다름): 추종 상태 `failed`, 단계 `return_home`, 실패 `endpoint_heading`, `at_cusp`, `off_path` 아님, 이 복귀 다리의 재계획 횟수 < 2.
  - 보류(fix A′, 0.05–0.08 rad)가 `judged_out`·`window` 등으로 실패를 확정하면(`fail_pending`), 그 틱의 추종 결과가 곧 `failed` 가 된다. 지금은 같은 틱의 `command` 에서 `Tracking failed in return_home` 으로 끝난다(`MissionCore.command` 의 `require`).
  - 그래서 복귀 재계획 블록은 `MissionCore.follow` 에서 **보류 블록 뒤**에 둔다. 운반 재계획 블록은 보류 블록 앞에 있지만, 운반은 보류 단계가 아니라 문제가 없다. 이 위치면 보류가 실패를 확정한 틱에 바로 상태를 `braking` 으로 바꿔 끝나지 않게 한다.
  - 보류 규칙은 바꾸지 않는다.
  - 0.08 rad 이상이라 보류 없이 바로 실패한 경우도 같다(BF aisle s6).
- 정지 확인: 운반과 같은 `cusp_stop_needed`(12 틱) 동안 정지(SLAM 이면 `slam["stop_now"]`). 그동안 명령은 0(실패한 추종기가 이미 0 을 낸다).
- 계획: 장애물 재계획의 복귀 다리 재계획과 같은 호출을 쓴다(`hooks.plan_return_leg(grid_world(return_scenario), 지금 자세, return_to_pose, planner_config, geometry, travel_config, grid_kwargs("return", destination), turnaround, backoff)`). `invalid_start`·`no_path` 면 여유 0 으로 한 번 더 한다(같은 규칙).
  - `return_scenario` 는 SLAM 이면 `slam["return_scenario"]`(도킹으로 옮겨진 목적지), 없으면 운반 장면이다(장애물 재계획과 같은 선택).
- 성공: 경로·`state["paths"]["return_home"]`(`path_record`)·`paths.json`·표시·새 추종기를 설치하고, 장애물 진행 기록을 비우고, 단계 시작 시각을 바꾼다(운반 cusp 재계획과 같은 처리).
- 실패: `return_cusp_replan_failed: <failed_plan_reason>` 로 실행을 끝낸다. 지금의 `Tracking failed in return_home` 과 구별된다.
- 기록: `state["return_cusp_replans"]` 에 항목마다 시각, 자세, 위치·방위 오차, 상태, 계획 벽시계, `search_attempts`, 성공이면 새 경로 기록, 바뀐 경로 기록.
- 예산: 복귀 다리마다 2 회. 운반의 `state["cusp_replans"]`(실행당 2 회)와 따로 센다. 다리가 바뀌는 다른 재계획(장애물·장면·해제)이 새 추종기를 설치해도 이 예산은 그 복귀 다리 전체에 대해 센다(실행당 복귀는 하나).
- 바꾸지 않는 것: `cusp_end_recovered`(짧은 다음 다리 판정)는 그대로 복귀 cusp 를 복구로 보지 않는다. 보류 판정의 결과는 그대로이고, 판정이 실패를 확정한 뒤에 재계획이 붙는다.
- 옵션: 러너 `--return-cusp-replan on|off`(기본 off). 기본 off 라 기존 기록의 재생이 바뀌지 않는다. riib 런처 기본값은 비교 배치 결과로 정한다.

### D2 (나) 운반 cusp 재계획 예산

- 러너 `--max-cusp-replans N`(기본 2, 지금은 러너 지역 상수 `max_cusp_replans = 2` 이고 재생 설정의 `_bound` 로 덮어쓴다). 비교 배치의 한 팔에서 3 을 쓴다. 재생은 기록된 설정값을 그대로 쓰므로 기존 기록에는 영향이 없다.
- 운반의 예산은 지금처럼 실행 전체 횟수다(riib 은 실행당 운반 다리 하나).

### D3 (다) 복귀 곡률 오프라인 재호출

- 입력: ws1 `r2_abe5e07b` 에 남은 경계 기록 19 건의 `Planner.plan_return_leg` 요청 전부. 표본이 적으면 gdrive 보관본에서 회귀 장면 기록 일부(line_2·random_1)를 되살려 더한다.
- 같은 요청을 곡률 한계 0.29(원래)와 0.26 으로 다시 부른다. 다른 설정은 그대로다.
- 보고: 장면별 성공 수, 실패 상태, 길이·후진 길이·기어 전환, 곡률 한계에 붙은 자세 비율, 계획 벽시계.
- 이 측정은 판정 항목이 아니다. 0.26 에서 실패가 늘지 않으면 다음 등록에서 (다)를 팔로 넣을지 총괄과 정한다.

## 4. 시험 (구현 전 고정)

- 단위:
  - 복귀 cusp 실패 → 정지 대기 → `plan_return_leg` 호출(인자: 지금 자세, `return_scenario`, 목적지 grid, turnaround·backoff 인자) → 성공 설치·기록.
  - 여유 0 재시도(`invalid_start`·`no_path`).
  - 계획 실패 → `return_cusp_replan_failed:` 로 끝남, 기록 남음.
  - 예산 2 회 소진 → 지금처럼 `Tracking failed in return_home`.
  - 옵션 off → 지금과 같음(재계획 호출 없음).
  - 운반 cusp 재계획과 예산을 나눠 씀(운반 2 회를 써도 복귀 2 회가 남음).
  - `--max-cusp-replans` 가 운반 예산을 바꾸고 기본은 2.
- 재생 게이트 runs29 29/29(옵션 기본 off).
- ws1 전체 시험.

## 5. 비교 배치 (등록은 이 계획 수렴 뒤 별도 문서)

- 팔: A = riib 기본(면제 켬), G = A + `--return-cusp-replan on`, GN = G + `--max-cusp-replans 3`.
- 장면: 대상 aisle_rack 0–7·trailer 0–4, 회귀 line_2·random_1 0–9 → 33 seed × 3 팔 = 99 실행.
- 판정 초안:
  - G 대 A: 복귀 단계 cusp 방위 실패(`Tracking failed in return_home` 의 `endpoint_heading`) 감소, 완주 합 G ≥ A, 짝 부호 검정(두 방향 불일치 모두)으로 G 쪽 악화가 유의하지 않음, 안전 키(접촉·허가 없는 진입·면제 키) 0, G 에만 나온 새 실패 유형은 원인을 댄다.
  - GN 대 G: 운반 cusp 방위 실패 감소, 완주 비감소, 같은 안전 조건.
  - 각 판정은 따로 둔다. G 가 통과해야 GN 을 본다.
- 유효성·결측·디스크·재실행 규칙은 riib 배치 2 등록(§3–4)을 그대로 따른다.

## 6. 남는 것

- BF aisle s0 형(복귀 시작 때 경로에서 1 m, cusp 근처 0.23 m 벗어나 `occupied` 정지 시간 초과)은 다른 계열이라 이 계획 밖이다.
- 실시간(작업자) 경로에서 cusp 재계획은 운반과 같이 틱 안 동기 호출이다. 이번 배치(riib, 배치 경로)는 이 점에 영향받지 않는다. 실시간 콘솔에 켜는 것은 따로 정한다.

## 7. 자체 검토

(검토 뒤 채운다: ① 원자료 재계산, ② Codex 적대적 검토, ③ agy 독립 검토.)
