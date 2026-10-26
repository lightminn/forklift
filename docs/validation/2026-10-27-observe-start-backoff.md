# 첫 원거리 관측 재계획의 시작 무효 — 곧은 물러남, 검증 결과 (2026-10-27)

계획: `docs/plans/2026-10-26-observe-start-backoff.md`(4 판, 계획·구현 Codex 수렴, agy 독립 검토). 지시: 총괄(2026-10-26, 인계 sw-연속이송-재관측) — 첫 관측 다리 `observe_replan_failed:invalid_start`. 정책(사용자 2026-10-24): 복구 로직·안전 판정 규칙·런처 기본값은 사용자 결정 없이 정하고 문서·보고로 남긴다.
구현: branch `fix/observe-start-backoff`. Isaac 비교에 쓴 코드는 `a9d4c4df`(스냅숏 `osb_a9d4c4df`) 이다. 그 뒤로 바뀐 것은 origin/main 병합뿐이다.

## 결론

- **기능만 넣고 기본은 끈다.** 러너 `--observe-start-backoff` 는 off 다. 런처(CT-1 `isaac_live.sbatch`, 리보틱스 `riib_scene.sbatch`)에는 기본값을 넣지 않았다. 켜려면 CT-1 래퍼는 `MORE`, 콘솔 런처를 바로 부를 때는 `EXTRA_ARGS` 에 `--observe-start-backoff on` 을 준다.
  - 계획서 3 장 4 의 기준이다. 켬 팔의 노출이 2 미만이면 기능 확인으로만 읽는다. 기본값을 켜려면 물러남이 실제로 설치되어 끝나고, 그 뒤 재계획이 설치되어 관측까지 간 사례가 있어야 한다.
  - CT-1 random_m 16 실행에서 노출은 0 이었다. 두 팔 모두 관측 재계획의 `invalid_start` 가 한 번도 없었고, 켬 팔의 물러남 기록도 0 이다.
  - 그래서 이 비교로는 효과도 해도 판정할 수 없다. 기록의 사례 12 건은 reported 오도메트리(10 건)와 survey 세션(2 건, team-spec)에서 났다. 이번 비교는 team-spec 의 CT-1 세션이다.
- 예방책(기어 전환점에 추종 오차만큼 계획 여유를 더 두기)은 계획서 6 장대로 따로 계획한다.

## 무엇을 넣었나

계획서 2 장(4 판) 그대로다.
- **B1 스위치**: `--observe-start-backoff on|off`(기본 off, 근접 계약 off). 끔이면 새로 읽는 값도 새 기록도 없다.
- **B2 거는 때**: 다음을 모두 만족할 때만 건다.
  - observe_miss 의 동기 재계획이 `invalid_start` 를 냈다.
  - 첫 원거리 관측(`scene_far_observe()`)이다. 장면 배치·관측 단계·인식 전이고, 근접·재정렬·밑판 다리가 아니다.
  - 그 관측 다리에서 처음이다(다리 serial 마다 한 번).
  - 지금 자세가 같은 시작 검사로 여유 0 에서는 통과한다.
- **B3 거리**: 실패한 틱의 입력으로 같은 시작 검사를 다시 만든다. 방향마다 `straight_start_advice`(0.01 m 간격, 최대 0.30 m)를 구한다. 뒤를 먼저 보고, 뒤가 어떤 이유로든 거절되면 앞을 본다. L = min(조언 + 0.03, 막히지 않고 이어지는 최대 거리, 0.30) 이다. 조언 + 0.01 보다 짧게 잘리면 물러나지 않는다. 사전 검사는 셋이다: 쓸림(관측 다리 사각형·계획 영역), 허가의 후보 검사, 출발 허가.
- **B4 주행**: 독립 상태 `core.observe_backoff` 를 둔다. D4 물러남의 소유권은 쓰지 않는다. 물러남 추종기(끝 허용 오차 0.01 m·0.02 rad)를 관측 다리 추종기로 둔다.
  - 러너는 `follow` 다음에 물러남 틱을 부르고, 그동안 D7·정지 재계획과 관측 복구를 건너뛴다. 설치한 틱도 물러남 틱을 거친다.
  - 매 틱 감시: 지금 자세에서 정지 길이만큼의 직선과 잡은 조향의 정지 원호를 조밀 표본으로 본다. 사각형에 닿으면 그 틱을 hold 로 세우고 다음 틱에 `observe_start_backoff_stopped` 로 미션을 끝낸다.
  - 재개 검사는 물러남 중 `free_leg` 를 거짓으로 둔다.
- **B5 끝**: 서면 SLAM 보류를 한 번 풀고 같은 시작 검사를 다시 한다.
  - 통과하면 원래 관측 목표로 한 번 계획해 떠난 다리의 설정으로 설치한다. 설치된 경로는 관측 재계획에 `after_backoff` 로 남는다.
  - 못 넘으면 `observe_start_backoff_short`, 계획이 실패하면 `observe_replan_failed:<status>` 로 끝낸다.
- **B6·B8 기록과 수명**: `state["observe_start_backoffs"]`, 미션 기준값 등록(`observe_start_backoffs`·`observe_backoff`·`observe_backoff_used`), 새 종료 사유의 미션 종료 분류, 콘솔 결과 화면의 사유 설명.

## CPU 확인

- 원자료 재계산(계획 1 장): 기록의 12 미션 모두 정지 자세가 여유 0 에서는 시작 검사를 통과하고 0.10 에서는 막힌다.
  - 10 건은 넓힌 팔레트 자리 때문이다. 검사기 여유 0.060–0.098 m 이고, 곧게 2–6 cm 물러나면 통과한다.
  - survey 2 건은 홀 계획 영역 경계 때문이다. 여유 0.0994 m 이고, 뒤로는 0.15 m 에서 막히고 앞쪽은 열려 있다.
- 단위 시험: `tests/unit/mission/test_observe_start_backoff.py` 22 개(매개변수 포함 23 건), `test_failures.py`·`test_results_reasons.py`·`test_isaac_truth_audit.py` 의 등록 시험.
- 재생 게이트 runs29 29/29: `replay_gate/osb_d6fd3c27`(첫 구현, 끔). 그 뒤 이 브랜치 자체 수정은 켬 경로뿐이다. 앞쪽 방향 추가, 설치 틱 경유, `clipped`, 콘솔 사유, 감사 이름이 그렇다. main 을 합친 HEAD 는 게이트를 다시 돌린다.
- ws1 전체 pytest(`a9d4c4df`): 6957 passed, 알려진 ws1 전용 실패 10 건만.

## Isaac 비교 (osbb)

조건: `ct1_queue.sbatch`, random_m 세션, SCENARIO=queue, CYCLE=P-4:D-1,P-5:D-2,P-3:D-3, seed 0·1·2·3·5·7·8·9, 두 팔이 같은 seed 를 번갈아 돈다. 팔 차이는 `MORE=--observe-start-backoff on` 하나다. 스냅숏 `osb_a9d4c4df` 는 PR #137 이후·PR #155 이전 main 이다. 그래서 두 팔 모두 team-spec 오도메트리이고 D7 재접근 보류는 꺼져 있다. 2 슬롯 묶음 4 잡(11463–11466)으로 냈고, 시간이 모자라 남은 끔 s9 하나는 11574 로 다시 돌렸다.

| 팔 | 기록된 미션 완료 | 관측 재계획 `invalid_start` | 물러남 기록 |
|---|---|---|---|
| 끔 | 20/22 | 0 | — |
| 켬 | 23/24 | 0 | 0 |

- 노출: 두 팔 모두 0 이다. 끝난 미션 기록과 세션 루트 `result.json` 을 모두 셌다.
- 실패 미션:
  - 끔: s0 m1 `scene_replan_refused:cusp_pose:blocked:occupied`, s7 m3 `transport_recovery_after_docking`.
  - 켬: s5 m3 `docking_retry_match_refused:correction_out_of_bounds`.
  - 모두 관측 다리 밖의 실패이고, 이 기능과 무관하다.
- 세션 실패 0(이 스냅숏은 SLAM 링크 완화 PR #145 이후다).
- 묶음 기록: 16 실행 모두 돌았다. 기반 실패·멈춤 0 이다. `time_left` 는 늦게 들어온 잡에 넘긴 실행 수이고, 잘린 실행은 없다.

## 한계와 남은 일

- 효과를 판정할 실행이 없다. 이 실패는 seed 와 실행 조건에 따라 드물게 난다. 기록의 12 건 가운데 이번 조건(team-spec CT-1 세션)에서 난 것은 없다. 다시 모이면 켬의 효과를 본다.
- 물러남 뒤 재계획은 같은 기어 전환점에서 다시 어긋날 수 있다. 다리마다 한 번이라 두 번째는 지금처럼 끝난다. 추종 오차 자체는 이 작업이 줄이지 않는다(계획 4 장).
- B4 감시와 B5 끝 처리는 Isaac 에서 실행된 적이 없다. 단위 시험과 Codex 의 CPU 반례로만 확인했다.
- 실패 뒤 다음 미션 준비의 시작 무효(survey 의 m2·m3)는 콘솔 범위다(계획 6 장).
- 관측 중 `return_home:invalid_start` 로 끝난 4 미션은 다른 지점이고, 조사하지 않았다(계획 6 장). 같은 방식으로 볼지 총괄에 제안한다.

## 실행 위치

- ws1 `test_runs/live/osbb_a9d4c4df_*`, `osbb_bundle`(요약 `osbb_bundle/osbb_sum.py`), 스냅숏 `snapshots/osb_a9d4c4df`, 재생 게이트 `test_runs/replay_gate/osb_d6fd3c27`.

## 자체 검토

- (결과 뒤 적는다)
