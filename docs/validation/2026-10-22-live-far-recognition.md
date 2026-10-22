# live 실행기의 원거리 인식 — 검증 (2026-10-22)

계획: `docs/plans/2026-10-22-live-far-recognition.md`(3판). 상위 계획: `docs/plans/2026-10-22-continuous-transport-ct1.md` PR ①. 브랜치 `feat/continuous-transport`.

## CPU 시험

- 배치 경로 불변: `tests/unit/test_far_batch_golden.py` 가 고정 입력 6 시나리오를 두 번 돌려 비교한다(oracle, 다리 손실, 학습, 학습→기하 대체, 구역 변경, 포크 불일치). 한 번은 이 브랜치 코드, 한 번은 분리 전 `ec4bece` 의 두 파일(`tests/fixtures/far_ec4bece/`)이다. 관측·기록·요약·저장 배열 해시가 모두 같다.
  - 처음에는 노트북에서 만든 JSON 기준값과 비교했다. ws1 에서는 CPU 리그 렌더가 달라 맞지 않았다. 그래서 같은 기계에서 두 코드를 함께 돌리는 방식으로 바꿨다.
- 작업 스레드·틱 적용: `tests/unit/test_far_live_worker.py`(28).
  - 배치와 같은 판정이 나온다.
  - 취득 자세는 제출할 때 계산한다.
  - 늦은 결과를 꼬리표로 버린다(세대·쉬는 세션·다리·이전 미션).
  - 큐 상한 4 를 넘으면 가장 오래된 대기 프레임을 버리고, 정지 일은 버리지 않는다.
  - 30 s 기한: 대기 중일 때와, 기한 뒤에 적용됐을 때 모두 `recognition_timeout` 이다.
  - 다리를 잃으면 정지가 무효가 된다.
  - 작업 스레드 예외와 주행 프레임 읽기 예외는 `recognition_error:<형식>` 이다.
  - 미션별 초기화·디렉터리·통계, 종료 때 모든 일의 회계(폐기·진행 중)를 시험한다.
  - 쓰기 오류(거절·실패)는 미션 기록에 남는다. 재계산 도구는 적용된 프레임만 재생한다. 캡처 보호의 짝, live 게이트도 시험한다.
- 미션 코어: `tests/unit/mission/test_core_async_plan.py` 끝 10 개.
  - 정지 판단 대기 중 작업 시계가 멈춘다(`begin_wait/end_wait`).
  - 대기 중 보정이 바뀌면 관측을 버린다.
  - 다리를 잃으면 재캡처 1 회, 두 번째는 관측 실패다.
  - 기한 초과 뒤 미션 끝, 일시정지 폐기를 시험한다.
  - live 관측 실패 뒤 남은 후보 계획은 작업자 `observe_ladder`(가까운 사다리만)로 간다. 끝까지 없으면 `observe:all_candidates_exhausted` 이고, 폐기되면 다시 요청한다.
- parser·분류·미션 키: `tests/unit/test_live_far_parser.py`, `tests/unit/planning/test_plan_worker.py` 마지막 시험.
- ws1 전체(`~/bin/forklift-ws1-pytest`, `5b40c79`): 5818 통과, 실패 10 건. 10 건은 모두 알려진 ws1 전용이다(mujoco 없음 6, evaluate_cli·remote_model_jobs 2, diagnose 게이트 2).
- 리플레이 게이트(M-a1 29 건, 스냅숏 `ct1_1cfd50e`): 29/29 가 갈라짐 없이 재생됐다(`finish ok`, `failure None`).

## Isaac live (ws1)

- 조건:
  - `deploy/console/isaac_live.sbatch` 에 `SCENE_FILE`(line_1, `--layout scene`)을 주었다.
  - `--recognition far --recognition-proposals learned` 는 YOLO11s, tau 0.3, CUDA 다.
  - 근접 재관측 인자는 PR #88 의 것이다. 배속 0.4, 픽셀 지연 12 스텝(아래 재측정값)이다.
  - 탐침은 `tools/live_console_probe.py` 다.
- 실행 스크립트: ws1 `test_runs/live/ct1_far.sbatch`. 결과는 `test_runs/live/ct1_{cycle_1cfd50e,main_419efe6,cancel_419efe6,reject_419efe6}/` 에 있다.

| 세션 | 미션 | 원거리 판정 | 미션 결과 |
|---|---|---|---|
| cycle | P-1:D-1 | fused(주행 12 프레임), 오차 측면 2.7·앞뒤 1.4 mm | 하역 중심 0.10 m 벗어남(실패) |
| | P-1:D-0 | 주행 중 표적 손실로 새 다리 → fused(43 프레임), 측면 2.2 mm | 완료, 1.8 cm |
| | P-1:D-1 | fused(51 프레임) | 운반 중 추종 시간 초과(실패) |
| main | P-1:D-1 | 첫 캡처 중 STOP(`live_stop`) → 재무장·재개 → fused | 완료, 5.1 cm |
| cancel | P-1:D-1 | fused 뒤 관측 중 취소 | 취소 |
| | P-1:D-1(새 제출) | fused(138 프레임) | 하역 중심 9 cm 벗어남(실패) |
| reject | P-1:D-1, 구역 +3 m(시험 인자) | 여섯 관측점 모두 `no_candidate_in_radius`(한 번 `unresolved_in_radius`) | `observe:all_candidates_exhausted`(미션 실패, 세션 생존) |
| | P-1:D-1(새 제출) | fused(69 프레임) | 완료, 6.4 cm |

- 미션마다 구역과 기록이 따로다.
  - 미션별 디렉터리 `recognition_missions/<id>/` 에 프레임과 `result.json` 이 있다.
  - 구역: 처음 자리의 사전 팔레트는 장면 구역(0.786, 0)이다. D-1 에 내린 팔레트는 그 추정 위치(3.0, −8.946) 둘레, D-0 에 내린 팔레트와 취소 뒤 재제출은 추정 위치 둘레다. 반경은 모두 0.354 m 다.
  - 앞 미션의 트랙·선택이 다음 미션에 남지 않았다.
  - 모든 미션의 기록 오류는 0 건이다.
- 픽셀 지연: live 의 원거리 다리마다 `lag_fit` 은 12 스텝이고 받아들여졌으며, 쓴 값과 같다. 배치의 24 스텝과 다른 이유는 live 콘솔 카메라가 12 스텝마다 렌더하기 때문이다.
- 학습 제안 왕복(cycle 세션): p50 57–59 ms, 최대 166 ms. 기하 대체는 0 건이다.
- 제출부터 틱 적용까지의 원거리 일 시간은 최대 1.23 s 다(큐 대기 포함). 틱은 그동안 돈다.
- 틱 간격(cycle 세션 62,317 틱):
  - 0.5 s 를 넘은 간격은 4 번이다. 두 번은 세션 시작(첫 미션 전, 2.89·1.02 s), 두 번은 m2 의 근접 캡처 뒤 접근·삽입 구간(1.15·1.10 s)이다.
  - 원거리 관측 다리 안에서는 0.5 s 를 넘은 간격이 없었다.
  - 상태 발행 간격 p99 는 1.06 s 다.
- 관측 실패 뒤 다음 후보 계획이 작업자에서 돌았다(reject m1 의 다섯 번, 각 `success`).

### 이번 PR 밖의 실패

- 하역 중심 0.09–0.10 m 벗어남(cycle m1, cancel m2)은 열린 하역 구역 정밀도 문제다. 별도 세션의 계획(`docs/plans` 열린 구역 도킹)이 다룬다.
- 운반 추종 시간 초과(cycle m3)는 다리 끝 크리프·교착 계열이다. 다른 세션이 담당한다.
- reject 세션의 첫 epoch 는 시작할 때 학습 작업자의 warm-up 이 GPU 메모리 할당에 실패했다(`bfc_arena`).
  - 그 시각 GPU 0 에는 다른 잡들이 12.0 GB 를 쓰고 있었다.
  - 감독기가 epoch 2 로 다시 띄웠다. epoch 1 은 설정 중에 끝나 live 기록도 미션도 없다. 탐침의 세션(`session_epoch`)은 epoch 2 의 것이다. 재시작은 판정에 섞이지 않았다.
- VRAM 실측(`ct1_vram_419efe6`, 빈 GPU 1 에 `shard:4`, main 시나리오, 잡의 프로세스 트리를 5 s 마다 표본):
  - live 원거리 세션 하나의 최대는 3,886 MiB 다(Isaac 3,372 + 학습 작업자 514).
  - shard 하나는 4,076 MiB(16,303/4)라 여유가 5 % 다. 작업자 warm-up 의 순간 최대는 표본 사이에 있어 재지 못했다.
  - 그래서 학습 제안을 쓰는 live 세션은 `--gres=shard:2` 로 잡는다(`isaac_live.sbatch` 머리말, ws1 `ct1_far.sbatch`).
- main 탐침의 텔레옵 구간은 배속 미달(`state_forbids_teleop:pacing`)로 거절됐다. 원인은 같은 GPU 부하다. 미션은 재개 뒤 끝까지 갔다.

## 교차검증

- Codex 구현 검토 4 차에서 수렴했다("새 P1/P2 없음").
- 1차 P2 6:
  - STOP 이 걸린 틱의 결과 적용
  - 준비 중 끝난 미션의 기록 소유
  - 종료 때 폐기 회계
  - 종료 뒤 쓰기 오류
  - 기한 뒤 적용
  - 메타데이터 분류
- 2차 P2 3:
  - 종료 스냅숏 경합
  - 세션 종료 순서
  - 이전 미션의 늦은 결과
- 3차 P2 1: 고정본의 저장소 경로.
- 지적은 모두 반영했고 시험으로 고정했다.
