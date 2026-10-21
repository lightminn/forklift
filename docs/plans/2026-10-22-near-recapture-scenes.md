# 리보틱스 장면의 삽입 직전 재관측 안정화 (3판)

브랜치 `feat/near-recapture`(origin/main `d2367b8` + `feat/riibotics-scenarios` `b35fcc9` 병합). 8주차 발표가 정한 다음 목표(일자·랜덤 배치 연속 이송을 인식·SLAM 포함 완주)의 1단계다. 연속 이송(다음 팔레트 선택·하역 칸 배정·옮긴 팔레트를 장애물로)은 이 계획 뒤 별도 계획으로 한다.

## 1. 사실 (원자료에서 다시 계산)

원자료: ws1 `test_runs/riibotics/r4_*`·`r5_aisle_out`(스냅숏 `riib_0ccec64`·`riib_827e757`, `riib.sbatch` 설정 — `--recognition far` 학습 제안, 밑판 검사 끔, 근접 추적 끔, `--near-capture-redraw` 기본 `on`). 근접 캡처 depth 7 장을 이 브랜치의 검출기로 재생했고 모두 기록과 같은 결과를 냈다(`detect_pockets` 상태·선택 일치).

8주차 실패 중 근접 캡처에서 끝난 것은 5 건이다.

| 실행 | 결과 | 원인 |
|---|---|---|
| line_1 s0 | `near_capture_failed:perception_no_pallet:no_opening_pattern` | 진짜 팔레트 패턴의 lower 4 < 15 |
| line_2 s0 | 같음 | lower 14 < 15 |
| random_1 s0 | `near_capture_misaligned` (측면 −6.7 cm, yaw 0.066) | 근접 지점 느슨한 도착 + 새 추정의 진입선 이동 |
| random_1 s1 | `near_capture_misaligned` (측면 −9.8 cm, yaw 0.105) | 같음 |
| aisle_out s0 | `approach:invalid_goal` | 근접 캡처가 옆 칸 팔레트(0.936 m)를 골랐다 |

### line — lower 띠

- 같은 팔레트의 원거리 캡처 lower 는 34–54 였다. 근접 캡처는 4·14(실패), 27·37·48(성공)이다.
- 근접 지점은 최종 직선(2.1 m) 시작점이라 원거리 지점보다 0.3 m 더 멀다(line_1: 원거리 base x −2.30, 근접 −2.61).
- lower 를 판정에서 빼고 재생하면 두 실패 모두 `valid` 이고, truth base 기준 팔레트 중심 오차 2 mm 이하, yaw 0.4 mrad 이하다.
- lower 가 판별력이 거의 없다는 사실은 `docs/plans/2026-10-21-near-margins.md` 1절 C 가 CPU 리그로 보였다. 그 계획의 사용자 결정은 "검사 켬일 때만 lower 문턱 끔"이고, 검사 끔 근거리 실패(`f_real_off_s3`)는 그대로 실패해야 한다고 적었다.

### random_1 — 근접 지점 도착

- 근접 관측 자체는 맞았다. 재생한 관측의 상대 yaw(팔레트 − 차)는 −0.0661 이고 truth(팔레트 0.1745 − 차 0.2406 = −0.0661)와 같다. truth base 로 옮긴 중심 오차는 2 mm 다.
- 8주차 원고의 "뒤 팔레트가 겹쳐 방향이 틀어졌다"는 가설은 틀렸다. 뒤쪽 팔레트 평면은 별도 후보(점수 2394·2700, 중심 1.8 m 밖)였고 선택되지 않았다.
- 두 seed 모두 `observe_loose_arrivals` 가 있다. 근접 다리도 `observe` 추종기를 쓰므로, 다리 끝에서 정지 1 초 + 위치 0.10 m·헤딩 0.10 rad 안이면 도착으로 치는 규칙(`OBSERVE_ARRIVAL_M`·`OBSERVE_ARRIVAL_YAW_RAD`, `core.py` 2283행)이 적용됐다. 헤딩 오차는 −0.047·−0.086 rad 이다.
- 도착은 원래 경로 기준이고 근접 수락은 새 추정으로 다시 그은 진입선 기준이다(`core.py` 3332행). 수락 offset yaw 0.0661·0.1050 은 도착 헤딩 오차 0.047·0.086 에 새 추정의 진입선 회전 0.0193·0.0191 을 더한 값이다. 측면 offset 은 −6.7·−9.8 cm 다.
- 근접 수락 상자는 측면 0.05 m·헤딩 0.05 rad 이다. 넘으면 근접 재작도(`near_capture_redraw`)를 부르는데, 이 실행은 근접 추적이 없고 `--near-capture-redraw` 가 `on` 이라 재작도 없이 끝났다.
- **PR #79 의 `always` 만으로는 고쳐지지 않는다.** 재작도(`redraw_docking`, `pallet_mission.py` 977행)는 헤딩 상자만 0.12 rad(`CORRECTION_MAX_YAW_RAD`)로 넓히고 측면 상자는 기본 0.05 m 그대로다. 두 seed 모두 `box` 로 거부된다(독립 검토 ③ 지적, CPU 재계산으로 확인).
- 측면 상자만 0.10 m 로 넓히고 기록된 정지 자세(`capture_control.control_rear`)·새 추정·접근 추종기 설정(`tracker_configs.approach`)으로 `straight_from_pose` + `bicycle_rollout` 을 돌리면 두 seed 모두 `arrived` 다. 끝은 목표를 약 2.5 mm 지나고 측면 약 0.1 mm·yaw 약 0.6 mrad(s1: 0.112 mm·0.607 mrad)이며, 궤적의 최대 측면 편차는 시작 offset(0.067·0.098 m)을 넘지 않는다. 시운전 판정의 6 mm 는 목표를 지나친 거리의 상한이고, 못 미친 거리는 추종기의 `arrived` 허용에 맡긴다. 같은 계산에서 random_2·line_1 s1 은 0.05 m 상자 그대로 통과한다.
- 성공한 random_2·line_1 s1/s2·line_3·random_3 에는 느슨한 도착이 없다.

### aisle_out — 이웃 선택

- 근접 프레임의 후보 패턴은 8 개이고, `upper_ok` 와 최종 검증(`_build_observation`)을 모두 통과하는 것은 4 개다(lower 문턱 유지 기준). 점수 순으로 진짜 팔레트(중심이 truth 에서 0.003 m)는 전체 6위, 최종 유효 후보 중 3위다. 1위는 0.951 m 옆 칸이다. 0.353·0.598 m 걸침 패턴은 upper 증거(오른쪽·왼쪽)가 부족하다. 검출기는 후보 전체에서 최고점을 고른 뒤 검증한다.
- 검출기는 점수 최대 후보를 고른다(`pocket_detector.py` `max(candidates, ...)`). 예상 위치를 보지 않는다.
- 0.353·0.598 m 후보는 두 팔레트의 블록에 걸친 가짜 패턴이다(칸 간격 0.95·1.05 m, 팔레트 폭 0.8 m).
- 원거리 캡처는 `--recognition far` 의 구역(pickup_prior)으로 진짜 팔레트를 골랐다(오차 2.4 cm). 근접 캡처는 원거리 인식을 거치지 않는다(`far_capture` 조건, `core.py` 3748행).
- 밑판 검사 켬의 채택 전 대조(`insert_refusal`, 0.15 m·15°)는 고른 뒤 거부만 하므로, 같은 장면에서 켬이어도 옆 칸을 고르고 거부로 끝났을 것이다.

### 원거리→근접 추정 이동

- riib 성공 실행에서 근접 추정과 원거리 추정(`near_capture.far_estimate_m`/`near_estimate_m`)의 차이는 0.009–0.059 m, yaw 0.0001–0.0193 rad 다(aisle_out 이웃 선택은 0.936 m).
- ws1 `test_runs` 의 모든 `result.json`(488 개)에서 근접 추정이 있는 156 건: 거리 중앙값 0.037, p90 0.081, p99 0.115, 최대 0.121 m(`bd_isaac/c_on_s4`, 밑판 검사 켬의 긴 옆면 우회), yaw 최대 0.033 rad.
- riib 장면에서 목표와 가장 가까운 이웃 팔레트 중심 간격: aisle·aisle_out 0.95, line_1 1.014, line_2 1.121, random_1 1.564, random_2 1.631, random_3 1.881, trailer 0.841 m(장면 파일 `scenes_riib_b35fcc9`; Codex 가 장면 생성 코드에서 같은 값을 다시 계산).

## 2. 범위

- **A — 근접 캡처의 예상 위치 제한 선택**: `--near-capture-expected on`(기본 `off`). 근접 캡처 검출은 현재 계획 목표(`planning_pickup`)를 캡처 완료 뒤의 제어 base 로 옮긴 예상 위치에서 중심 **0.20 m**·진입 yaw 15° 안의 후보만 본다. 그 안에서 기존 규칙(점수 최대, 같으면 가까운 평면)대로 고른다. 안에 후보가 없으면 `no_pallet:no_expected_candidate`.
  - 반경 근거: 관측된 최대 원거리→근접 이동 0.121 m 를 한 가지 기준으로 두 쪽에 적용한다. 진짜 팔레트는 0.121 < 0.20 이라 남고(여유 0.079 m), 가장 가까운 걸침 패턴은 0.353 − 0.121 = 0.232 > 0.20 이라 빠진다(여유 0.032 m). 2판의 0.25 m 는 이 기준에서 걸침 패턴이 들어왔다(Codex 2차 P3). 검증된 오차 상한이 아니라 riib 시험 시작값이다.
  - 이동이 0.20 m 를 넘으면 진짜 팔레트가 빠진다. 그때 결과는 `no_expected_candidate` 가 될 수도, 범위 안의 다른 패턴이 될 수도 있다(보장이 아니라 가능한 결과).
  - A 가 주는 것은 원거리에서 고른 개체를 근접에서도 이어 보는 **선택 연속성**이지 올바른 개체라는 보증이 아니다. 원거리 인식이 이웃을 골랐으면 A 는 그 오선택을 고정한다.
- **B — riib 실행에 `--near-capture-redraw always`**: 코드 변경 없음. 연구용 실행 설정으로만 쓰고, 장면 기본 설정 채택은 4절의 강화된 검증 뒤로 둔다(Codex).
- **D — 비추적 근접 재작도의 측면 상자**: `--near-capture-redraw-lateral-m`(기본 0.05, 0.05–0.10). 근접 추적 없이 `always` 일 때만 근접 재작도의 `max_lateral_m` 을 넓힌다. 헤딩 0.12 rad·시운전 도착(6 mm·6 mm·0.015 rad)·접근 직선과 시운전 궤적의 충돌 검사는 그대로다. riib 실행은 0.10.
- **C — 근접 캡처 lower 면제(검사 끔)**: **결정 대기**(총괄 세션에 요청함). 결정 전에는 구현하지 않는다. 결정하면 "원거리에서 lower 를 통과한 목표"의 연결 조건부터 정해야 한다 — 원거리에서는 기본 검출 결과가 `detector_reference` 로 남고 채택 관측은 학습 인식 결과로 바뀌므로, 기본 검출기의 `selected_lower` 만으로 채택 목표의 lower 를 말할 수 없다(Codex P3).
- **범위 밖**: 근접 다리의 느슨한 도착 규칙 자체(이번에는 D 로 받는다), 하역 구역 정밀도(2순위), 연속 이송(다음 계획), 트레일러·좁은 통로 경로, 속도.

## 3. 작업

### 작업 A — 예상 위치 제한 선택

- 검출기(`pocket_detector.py`):
  - `ExpectedSite`(base_link 의 팔레트 중심 x·y, 진입 yaw, 반경, 각 허용; 유한·양수 검사)와 `ExpectedCandidate`(후보별 평면 번호, 함의 중심, yaw, 거리, 각 오차, 점수, 포함 여부).
  - `detect_pockets_expected(scene, prior, params, expected, *, lower_evidence=True) -> (DetectionResult, candidates)`. `detect_pockets` 는 시그니처·결과가 그대로이고 같은 내부 본문을 `expected=None` 으로 부른다.
  - 후보 중심은 패턴 두 구멍의 앞면 중점 + 사전 깊이/2 를 법선 yaw 방향으로(`_build_observation` 과 어댑터 `estimate_pallet_center_m` 의 식과 같다). 범위 밖 후보는 선택 전에 뺀다. 모든 패턴이 범위 밖인 평면의 거절 이유는 `outside_expected`(기존 `lower_pattern_score` 와 구별).
  - `DetectorParams`·`DetectionDiagnostics`·`DetectionResult` 의 필드는 바꾸지 않는다.
- 미션(`core.py` `capture_and_detect`):
  - 옵션이 `on` 이고 `near_capture.status == "pending"` 이면, 캡처 완료 뒤 SLAM 추정으로 정한 제어 base(`rear`·`base`·`yaw`, 3672행 이후)로 `planning_pickup` 을 옮겨 `ExpectedSite` 를 만들고 `hooks.detect_pockets_expected` 로 부른다. 재시도·재정렬 stand 재관측은 `near_capture` 를 지운 뒤라 대상이 아니다(Codex 확인).
  - 시도 기록 `attempt["expected"]`: 예상 위치, 반경·각, 후보 목록. `off` 에서는 기존 호출(kwargs 포함)과 시도 기록 모양이 그대로다.
  - 밑판 검사 켬 면제와 조합하면 `lower_evidence=False` 를 같이 넘긴다.
- 러너(`run_transport.py`):
  - `--near-capture-expected {off,on}`. `on` 은 `--use-perception` 과 `args.planning_target == "perception"` 일 때만(그 밖은 parser 오류). 상태의 출처 문자열은 `camera_depth_detection` 이므로 상태로 검사하지 않는다(Codex P1).
  - `detect_pockets_expected` 를 `detect_pockets` 처럼 기록 포트(`recorded_function(..., "Perception", "detect_pockets_expected")`)로 감싸고 `replay_setup.PORT_FUNCTIONS` 에 등록, 훅으로 연결한다. 반환값(후보 dataclass 튜플)이 경계 기록 코덱을 통과하는지 확인한다.
  - `near_tracking.POST_MEASUREMENT_ARGUMENTS` 에 `"near_capture_expected": "off"`. `on` 은 근접 추적 측정 조건에서 거부된다(근접 추적과의 조합은 지원으로 해석하지 않는다).
- 진단 도구(`tools/diagnose_detection.py`):
  - `attempt_expected(attempt)` 로 기록의 예상 위치를 복원해 `detect_pockets_expected` 로 재생한다.
  - 자체 후보 재구성(427행~)과 선택(476행 `max`)에도 같은 포함 판정(검출기와 공유하는 기하 함수)을 적용한다. 다른 후보 선택·전부 탈락·기존 invalid 유지 세 경우를 시험한다(Codex P2).
- 시험(먼저 작성):
  - 검출기: `scene_rig` 두 팔레트 배치 스윕(x 2.4–3.6 m, 이웃 ±0.95/1.0 m, 목표 y −0.1–0.1)에서 찾은 경우 — 제한 없이 이웃, 제한으로 목표(x 3.2); 목표 패턴이 증거 부족으로 없으면 이웃 대신 `no_expected_candidate`(x 3.0); 각 허용; 후보 기록; `expected=None` 은 `detect_pockets` 와 같음; lower 면제와 조합; `ExpectedSite` 값 검사.
  - 미션: 근접 캡처에서만 예상 위치가 넘어감(원거리·재관측·stand 는 아님), 좌표 변환은 캡처 뒤 제어 base 기준, `off` 의 호출·기록 불변, 실제 `G2.planning_target` 를 거친 출처로도 동작.
  - 러너: 옵션 조합 거부(`--planning-target` truth·oracle, perception 없음).
  - 진단 재생: 위 세 경우 + aisle_out 저장 프레임에서 제한 선택 결과가 truth 3 mm 후보.
  - 경계 재생: 기존 boundary 기록 재생이 그대로 통과(옛 setup 의 인자에는 새 옵션이 없으므로 `off` 로 복원), 새 `on` 기록의 재생은 Isaac 실행 뒤 따로 확인. 러너의 옵션 조립은 재생이 복원한 옛 Namespace(새 속성 없음)에서도 기본값으로 읽는다(구현 리뷰 1차 P2). M-a2 지도는 동결된 M-a1 단언에 대응하는 구조라 새 속성을 넣지 않는다 — 새 동작은 새 단위 시험이 지키고, 기존 변이의 치환 위치가 사라지지 않았는지와 `--complete` 만 확인한다(Codex 구현 리뷰 1차 동의).

### 작업 D — 비추적 근접 재작도의 측면 상자

- 러너: `--near-capture-redraw-lateral-m`(기본 0.05). 0.05 가 아니면 `--near-capture-redraw always` 이고 `--near-tracking` 이 없을 때만, 값은 (0.05, 0.10].
- 미션: `near_capture_redraw` 가 비추적(`self.near is None`)일 때 그 값을 `redraw_docking` 의 `max_lateral_m` 으로 넘긴다. 기본값(0.05, 옵션 없음 포함)에서는 호출 kwargs 와 재작도 기록 항목이 모두 그대로이고, 비기본값에서만 호출에 `max_lateral_m` 을, 기록에 `max_lateral_m` 을 더한다(경계 재생의 최종 상태 정확 비교).
- `POST_MEASUREMENT_ARGUMENTS` 에 `"near_capture_redraw_lateral_m": 0.05`.
- 시험: random_1 s0·s1 의 기록 자세·추정으로 0.05 는 `box`, 0.10 은 수락(시운전 도착); 기본값의 호출 불변; 옵션 조합 거부.

### 작업 B — riib 설정

- ws1 `test_runs/riibotics/riib2.sbatch`: `riib.sbatch` 에 `--near-capture-redraw always --near-capture-redraw-lateral-m 0.10 --near-capture-expected on` 을 더한 것(원본은 재현용으로 그대로).

## 4. 검증

- ws1 pytest 전체(`~/bin/forklift-ws1-pytest`), 알려진 ws1 전용 실패 외 0.
- Isaac(ws1, 스냅숏 = 이 브랜치 머리, `riib2.sbatch`):
  - 재현 대상: random_1 s0·s1, aisle_out s0, line_1 s0, line_2 s0.
  - 회귀: line_1 s1, random_2 s0.
  - 판정(근접 캡처): 수락 여부, 이웃 선택 없음(선택 후보와 truth 거리), 근접 추정의 truth 상대 오차.
  - 판정(재작도가 쓰인 실행, Codex P2): 재작도 `accepted` 와 경로 설치, 접근·삽입 동안 SLAM hold, 실제 궤적의 장애물·팔레트 최소 여유, truth 기준 삽입 종점과 좌우 `beyond_centre`, 금지 접촉 0. 비추적 실행은 S4 판정기가 거부하므로 이 항목들을 기록에서 직접 계산한다. 기본 속도에서만 확인한다.
  - 사이클 완주 여부와 하역 오차는 기록만 한다(하역 8 cm 는 2순위 범위).
  - C 결정 전 line 두 건은 "A·B·D 로는 실패가 남는다"(`no_expected_candidate` 또는 `no_opening_pattern`)는 확인으로 센다.
- 잡은 shard 1·CPU 4·메모리 8G(riib 기존 설정, `sacct` MaxRSS 로 재확인). V-G2 잡(8026–8031)과 다른 사용자 잡은 건드리지 않고, 제출 전 `AllocTRES` 를 본다.

## 5. 위험

- **제한 반경:** 0.25 m 보다 큰 원거리→근접 이동(긴 우회 뒤 SLAM 보정 등)에서는 진짜 팔레트가 밖으로 밀려 `no_expected_candidate` 가 난다 — 이웃을 고르는 것보다 낫지만 실패다.
- **좁은 칸:** 걸침 패턴 위치는 장면마다 다르다(aisle 은 0.353·0.598 m). 이웃 중심 간격이 riib 최소 0.841 m 보다 촘촘한 배치에서는 걸침 패턴이 반경 안에 들어올 수 있다. 일반 법칙이 아니라 장면별로 본다.
- **D 의 안전 근거:** 비추적 재작도는 보정점 검사와 근접 추적의 보호가 없고 hold 오도메트리로 달린다(PR #79 위험 절). 측면 상자를 넓히면 선회가 늘어 오도메트리 과대계상이 커질 수 있다. bicycle 모형 수렴은 Isaac 수렴을 보장하지 않는다 — 4절 재작도 판정 항목을 필수로 잰다.
- **C 의 위험(결정 시):** 검사 끔에서 lower 는 약하나마 grounded 를 거르는 항이었다(`docs/plans/2026-10-14-near-capture-lower-evidence.md` 의 Codex 9판 지적). 원거리 학습 인식의 음성 오검출은 grounded 13/65 였다.
- **사전 깊이:** 후보 중심은 `prior.overall_depth_m` 으로, 미션 추정은 `geometry.pallet_depth_m` 으로 계산한다. riib(EPAL 6)는 둘 다 0.6 m 다. T11 등 다른 형상에서 두 값이 다르면 그 차이의 절반만큼 비교가 치우친다(독립 검토 ③).
- **시뮬 일반화:** 근접 depth 는 잡음 없는 렌더(1 mm 양자화)다.

## 6. 자체 검토 — 직접 재계산

- 표의 실패 원인·lower 값(4·14 / 27·37·48 / 원거리 51·34·40·54·41)은 각 실행 `result.json` 의 `detection_diagnostics.rejected_patterns`·`selected_lower` 에서 다시 읽었다.
- 근접 depth 7 장(line_1 s0 #1·#2, line_2 #1·#2, aisle_out #1·#2, random_1 #1·#2)을 `tools/diagnose_detection.attempt_scene` + 이 브랜치 검출기로 재생해 기록 상태와 일치를 확인했다. lower 면제 재생: line_1 s0 #2 중심 오차 0.002 m·yaw −0.0003, line_2 #2 0.000 m·−0.0004.
- aisle_out 후보 수: 처음 "유효 8"로 적었다가 재계산해 "후보 8·유효 7"로 고쳤다. 진짜 팔레트 순위는 유효 후보 중 5위.
- random_1 상대 yaw: 처음 "차 − 팔레트 = 0.0661"로 적었다가 관측 정의(팔레트 − 차)에 맞춰 −0.0661 로 고쳤다(Codex P3).
- 이동 분포: ws1 Slurm 잡 8041(`scan_shift_nr.py`)이 488 개 `result.json` 에서 156 건을 읽었다.
- 이웃 간격: 장면 파일 10 개에서 `riibotics:*pallet*` 소품 중심과 목표 중심 거리.
- D 수렴: `redraw_probe.py`(로컬, 기록 자세·추정·추종기 설정) — 0.05 m 에서 random_1 두 seed `box`, 0.10·0.15 m 에서 `arrived`.
- 리그 스윕 1(`rig_scan.py`, 반경 0.15 m·예상 위치 오차 0, 132 배치 완료분): 제한 없이 목표를 맞게 고르던 77 배치는 제한 뒤에도 모두 목표, 이웃을 고르던 35 배치 중 5 배치가 목표로 바뀌고 30 배치는 목표 패턴 자체가 증거 부족이라 `no_expected_candidate`.
- 리그 스윕 2(`rig_scan2.py`, 반경 0.20 m, 예상 위치 측면 오차 0·±0.12 m): 결과는 8절.

## 7. 검토 기록

### 2판 → 3판

- **② Codex 재검토(2차): 새 P1/P2 없음(수렴), P3 4.** 모두 받아들였다.
  - aisle_out 후보 수: 재계산이 `upper_ok` 검사를 건너뛰었다 → "후보 8·최종 유효 4·진짜 전체 6위·유효 3위".
  - 반경: 포함·배제에 다른 이동값(0.121·0.059)을 썼다 → 0.121 하나로 두 쪽을 보고 0.25 → 0.20 m. 진짜가 빠질 때의 결과는 보장이 아니라 가능성으로 고쳤다. 0.15 m·오차 0 스윕을 0.20 m 근거로 쓰지 않고 스윕 2 를 새로 돌렸다.
  - D 기록: 기본값에서 kwargs·기록 항목 모두 불변, 비기본값에서만 추가(구현은 이미 그렇게 함)로 문구를 맞췄다.
  - 시운전 문구: 6 mm 는 지나친 거리 상한, 수치는 근삿값으로.
  - Codex 확인(반박 없음): D 의 기본값 호출·기록이 HEAD 와 같음, 10 cm 출발점에서도 시운전 궤적 충돌 검사가 실제 출발 자세부터 다시 검사함(작은 장애물 반례에서 `dry_run_collision`), `s4_judge` 는 비추적 실행을 입구에서 거부하므로 판정기 수정 불필요.

### 1판 → 2판

- **① 직접 재계산(6절):** 후보 수, 상대 yaw 부호, 이동 분포(반경 0.15 → 0.25 m 근거)를 고쳤다.
- **② Codex 적대적 검토(1차): P1 1·P2 3·P3 2.** 모두 받아들였다.
  - P1 출처 문자열: 상태의 `planning_target_source` 는 `camera_depth_detection` 이다 → 러너 인자 `args.planning_target` 로 검사, 시험은 실제 `G2.planning_target` 경유.
  - P2 진단 재생: 자체 후보 재구성·선택에도 같은 판정, 거절 이유 `outside_expected` 분리.
  - P2 B 완료 조건: 재작도 실행의 판정 항목을 4절에 추가, 기본 설정 채택은 그 뒤.
  - P2 경계 재생: 새 기능을 기록 포트로, `off` 의 호출·기록 불변, 옛 기록·새 기록 재생을 따로 확인, M-a2 속성.
  - P3 random_1 원인 표현과 상대 yaw 부호, C 의 연결 조건 — 반영.
  - Codex 확인 사항(반박 없음): 기준은 `planning_pickup`(`obstacle["pickup_estimate"]`·`far_estimate_m` 은 대체물이 아님), stand 재관측은 `pending` 조건으로 빠짐, 이웃 간격 수치 일치.
- **③ 독립 검토(agy, Gemini 3.8 Flash Medium — Argon 은 일할 한도 보류).** ①·② 결론을 보이지 않고 1판만 주고 적히지 않은 전제를 물었다. 10 개 중 받아들인 것:
  - 재작도 측면 상자 0.05 m 라 `always` 만으로 random_1 이 안 고쳐진다 → CPU 로 확인하고 작업 D 를 새로 넣었다(가장 큰 변경).
  - A 만으로는 line 이 안 고쳐진다 → 4절에 명시.
  - 사전 깊이와 미션 기하 깊이의 일치 → 5절.
  - 원거리 오선택 고정 → 2절 A(Codex 도 같은 반례).
  - 반경 대비 이동 → 분포로 확인해 반경 조정.
  - 경계 재생 포트 계약 → Codex P2 와 같은 항목.
  - 받아들이지 않은 것: 법선 yaw 의 15° 초과 흔들림(기록 156 건의 yaw 이동 최대 0.033 rad), 재작도 안전성 일반론(4절 판정으로 대체).

## 8. 리그 스윕 2 (반경 0.20 m, 예상 위치 오차)

`scene_rig` 로 EPAL 6 두 개(목표 x 2.4–3.6 m, 이웃 ±0.95·±1.0 m, 목표 y 0·±0.1 m) 156 배치를 그리고, 예상 위치를 truth 에 두거나 측면으로 ±0.12 m(관측 최대 이동) 옮겨 넣었다. 세 경우의 결과는 156 배치 모두 서로 같았다.

| 제한 없음 | 제한(0, +0.12, −0.12 모두) | 배치 |
|---|---|---|
| 목표 | 목표 | 83 |
| 이웃 | 목표 | 5 |
| 이웃 | `no_expected_candidate` | 48 |
| invalid(걸침 패턴 등) | `no_expected_candidate` | 19 |
| `no_opening_pattern` | `no_opening_pattern` | 1 |

- 제한으로 이웃이나 걸침 패턴을 고른 배치는 0 이다. 목표를 맞게 고르던 배치를 잃은 경우도 0 이다.
- `no_expected_candidate` 67 배치는 목표 자신의 패턴이 후보에 없다(대부분 lower 증거 부족 — 리그는 1 mm 양자화 렌더). 이 경우 A 는 오선택을 실패로 바꿀 뿐 성공으로 바꾸지 못한다. line 실패와 같은 원인이고, 고치려면 C(결정 대기)가 필요하다.
