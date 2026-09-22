# 임무·관측 계획 primitive 0.25 m 통일

기준 revision: `2671053` + 이 검증의 미커밋 변경. 검증 종류는 **호스트 CPU의
합성 기하 계획·설정 전달·직렬화**다. Isaac 및 원격 실행은 수행하지 않았다.

## 적용 범위

- `pallet_mission.make_transport_planner_config()`가 임무·관측 기본값
  `primitive_length_m=0.25`, `clearance_m=0.10`을 생성한다. 일반
  `PlannerConfig`의 기본 primitive 0.5 m와 확장 한도 12,000은 유지한다.
- `plan_transport`와 `plan_observation_leg`는 `config=None`일 때만 이 함수를
  사용한다. 명시적 객체는 그대로 전달하며, approach의 기존 clearance 상한
  `min(config.clearance_m, geometry.approach_gap_m / 2)`만 복사본에 적용한다.
- 실행기는 설정을 한 번 생성해 관측 2곳·임무 2곳에 같은 객체를 전달한다.
  실행기와 벤치마크의 확장 한도는 모두 기존 **30,000**이다.
- 실행기 `result.json`과 벤치마크 `results.json`에 실제 전달 객체의 전체
  `planner_config`와 `approach_clearance_m`을 기록한다. 기본 approach 값은
  0.05 m이며, `settings_synthetic`만으로 설정을 재구성하지 않는다.

## 두 회귀의 계약 보존

### 관측 장애물 회피

`test_observation_leg_avoids_props`의 상자 중심 y를 0에서 0.45 m로 옮겼다.
상자 하단은 y=0.30 m이고 차체 반폭은 0.36 m이므로 **직진은 여전히 충돌**한다.
시험은 직진 충돌, 우회 계획 성공, 반환 경로 전체의 장애물·팔레트 clearance를
모두 검사한다. 기본 설정과 명시적 0.25/0.5 m 설정을 각각 실행한다.

| 설정 | 결과 | 확장 수 |
|---|---|---:|
| 기본 0.25 m, clearance 0.10 m | 성공 | 272 |
| 명시적 0.25 m, clearance 0.15 m | 성공 | 272 |
| 명시적 0.5 m, clearance 0.15 m | 성공 | 200 |

원래 중앙 장애물 fixture에서 0.25 m가 `no_path`인 현상은 해결했다고 주장하지
않는다. 이번 변경은 탐색 알고리즘을 수정하지 않으며, 도달 가능한 입력으로
원래의 장애물 회피 계약을 검사한다.

### 배송 최종 직선 clearance

픽업 `(0, 0, 0)`과 배송 `(3.4, 0, 0)`을 같은 축에 배치하고, 포스트 중심을
`(3.2, 0.475)`로 옮겼다. 적재 반폭 0.40 m와 포스트 반폭 0.025 m 사이의
표면 간격은 0.05 m다. predelivery는 0.10 m 여유로 통과하고, 최종 직선은
0.00 m에서 통과·0.10 m에서 충돌한다는 기하 전제도 직접 검사한다.

새 기본값으로 **정확히 `transport:straight_collision`**을 요구하고, 같은
0.25 m primitive에 clearance만 0.00으로 명시하면 전체 계획이 성공해야 한다.
따라서 탐색 한도 소진이나 다른 단계의 실패로 이 시험을 통과할 수 없다.

## 시험과 변이

전체 명령: `python -m pytest tests --ignore=tests/simulation -q -p no:cacheprovider -W error`.

- 변경 전 기준선: **1,126 passed**.
- 시험 먼저 추가한 RED: **8 failed, 39 passed**. 기존 기본 경로·벤치마크의
  primitive 0.5와 공통 생성 함수·실행 설정 기록 부재로 실패했다.
- 구현 후 관련 시험: **47 passed**. 변경한 Python 6개 파일의 Ruff lint 및
  format check도 통과했다.
- 변경 후 전체 시험: **1,135 passed** (175.04초). 기존 1,126개에서 관측
  fixture 파라미터 2개·코어 설정 시험 4개·실행기 설정 시험 3개가 추가됐다.

변이는 별도 Python 프로세스의 메모리에만 적용하여 작업트리 소스는 유지했다.

| 변이 | 검출 결과 |
|---|---|
| 임무 기본 primitive 0.25 → 0.5 | 기본 경로·생성 함수·실행기 기록 시험 **6개 실패** |
| 임무 기본 clearance 0.10 → 0.00 | 배송 직선 시험 **1개 실패**: 계획이 성공해 버림 |
| 배송 최종 직선의 clearance만 0.00 | 같은 시험 **1개 실패**: `withdraw:straight_collision`로 바뀌어 정확한 단계 assertion이 검출 |
| 관측 계획에서 props 누락 | 장애물 회피 시험 **3개 모두 실패** |

실행기 시험은 AST에서 설정/기록 대입문과 네 계획 호출식을 추출하여 **실제
코어 계획기**로 실행한다. 임의의 추가 설정 값도 직렬화되는지, 기록 객체와
실제 전달 객체가 같은지, 세 가지 gap/clearance 조합에서 실제 approach 설정과
기록이 맞는지 확인한다. 이는 SDK·장면·분기 진입을 포함한 Isaac 실행 증거가 아니다.

로컬 상세 로그와 소스 해시는
`artifacts/20260921T002636Z_transport_primitive_025/`에 보관한다.

## 결과 해석의 경계

기존 G2 5/9와 정답 대조군 3/9는 당시 구성의 유효한 기록으로 남는다. 이를 새
구성의 성적으로 사용하거나, 추가 계획 성공 4건을 더해 정답 임무 성적을 7/9로
계산하지 않는다. 추종 제한시간은 경로 길이에 따라 계산되므로 경로 변경 시
제한시간도 달라진다. 새 구성의 전체 임무 성적은 사용자의 후속 ws1 실행으로
확정해야 한다. 기존 실행 기록·Isaac 상태·`omni`는 변경하지 않았고 커밋하지 않았다.
