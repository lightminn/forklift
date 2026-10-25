# live 기본을 사전 지도 위치 추정으로 — 구현 계획 (S3, 초안 v0, 2026-10-26)

상위: [SLAM 그래프 크기 묶기](2026-10-25-slam-bounded-graph.md)(후보 A 확정, 6.2–6.3) · [상시 운용 세션의 SLAM](2026-10-20-slam-localization-session.md)(PR #62: 위치 추정 경로 L1–L7, 이미 main 에 있음) · [slam_link_failed](2026-10-25-slam-link-failed.md)(지금 기본값인 임시 완화)

사용자 결정(2026-10-26, 총괄 전달): A — 장면마다 미리 만든 지도에서 운행 중 위치만 추정 — 를 채택 후보로 확정했다. live 기본 전환은 이 계획이 수렴한 뒤 총괄에 요약 보고하고 그때 한다.

## 1. 목표

- 콘솔 live 세션이 장면의 조사 지도 묶음 위에서 위치 추정으로 돈다. sim 1 h 연속 운용에서 keyframe 처리 시간이 모두 1 s 미만이다(상위 계획의 목표).
- 묶음이 없는 장면은 지금처럼 mapping(임시 완화 설정)으로 돈다. 어느 쪽으로 도는지 화면·기록에 보인다.
- 정확도·임무 결과가 지금 기본값보다 나빠지지 않는다.

## 2. 이미 있는 것과 없는 것

있음(PR #62 S3·L3–L7, main):
- 런처 `MAP_BUNDLE`·`MAP_START_POSE`(`deploy/console/isaac_live.sbatch` 34–62), 감독자의 세션 단위 묶음 해시(`tools/live_supervisor.py` 25–28), 브리지의 묶음 적재·L4 시작 검사(`slam_map_mismatch`·`slam_map_not_loaded`·`slam_not_localized`), runner 의 묶음 대조(`run_transport.py` 3427–3444)와 조사 격자의 정적 층 적재(4453–4456), 결과 기록·화면의 `map_id`(`mission/results.py`, `console/web/js/ui.js`).
- 조사 에폭 런처(`deploy/console/slam_survey.sbatch`, SCENE_FILE·CYCLE 지원)와 묶음 작성기·검사(`tools/slam_survey_bundle.py`: finalize·received·nodes·world(V2′)·start·coverage·moving).

없음:
- 장면별 묶음과 그것을 고르는 등록부. 지금은 운용자가 `MAP_BUNDLE` 을 손으로 준다.
- seed 가 다른 세션의 묶음 재사용. `run_mismatches` 는 seed·`slam_noise_seed` 가 조사와 같아야 통과시킨다(`slam_map.py` 140–160). CT-2 시험은 seed 0–4 를 쓴다.
- CT-2 장면의 team-spec 조사 묶음(잡 11393·11394 진행 중).

## 3. 결정 (잠정)

- **D1 묶음 등록부.** `config/slam_map_bundles.yaml` 에 장면 파일(내용 해시)·오도메트리 모형별로 묶음 폴더·`map_id`·출발 자세를 적는다. 묶음 파일은 ws1 `artifacts/slam_bundles/<장면>_<모형>_<map_id 앞 12>/`. 저장소에는 등록부만 커밋한다(묶음은 큰 이진 파일).
- **D2 런처 기본.** `isaac_live.sbatch` 가 `SCENE_FILE` 과 `ENCODER_MODEL` 로 등록부를 찾아, 있으면 `MAP_BUNDLE`·`MAP_START_POSE` 를 채운다. 없으면 mapping. `MAP_BUNDLE=none` 은 mapping 을 강제한다(대조 실험·조사 에폭용). 고른 결과를 세션 기록(`supervisor.jsonl`)과 상태 메시지의 `slam.mode` 에 남긴다.
- **D3 seed 재사용.** 장면 파일·소품·충돌체가 같으면 seed 가 달라도 LiDAR 평면의 세계는 같다. 그래서 묶음 대조에서 seed·`slam_noise_seed` 일치 요구를 "장면 내용 해시(장면 파일 + 생성된 장면 USD + 소품)" 일치로 바꾼다. 잡음 seed 가 다른 위치 추정은 S2 재생에서 이미 했다(지도 실행과 입력 실행의 잡음이 달랐다). 바뀌면 안 되는 것(URDF·LiDAR 설정·장애물 층·충돌체)은 그대로 대조한다.
- **D4 위치 추정 설정.** 변형 J(`tools/slam_localization_params.py`, 버퍼·chain 3/3, 루프 켬, link 3.0 m, 루프 창 3.0 m·거리 2.0 m). S2 에서 J 두 번이 판정을 채웠다.
- **D5 묶음 합격.** 조사 묶음은 `slam_survey_bundle.py` 의 검사를 모두 통과해야 등록부에 오른다. world(V2′) 기준은 우리 잠정값(위치 최대 ≤ 0.10 m·방향 최대 ≤ 0.03 rad, PR #62 계획 413 행)이다. 미통과면 원인과 숫자를 총괄에 올리고, 기준 조정은 총괄이 판단한다.
- **D6 재조사 조건.** 장면 파일·URDF·LiDAR 설정·충돌체·오도메트리 모형이 바뀌면 묶음은 대조에서 거절되고(`slam_map_mismatch`, 감독자가 즉시 멈춤), 등록부를 새 조사로 바꿔야 한다. 이 절차를 운용 문서에 적는다.
- **D7 운용 중 위치 상실.** slam_toolbox 위치 추정에는 전역 재위치가 없다. 조사 덮개 밖으로 나가거나 크게 어긋나면 돌아오지 못할 수 있다. 이 계획은 막지 않고 잰다(S4 의 정확도·완주). 문제가 보이면 별도 계획.
- **D8 지금 기본값 유지.** mapping 경로의 임시 완화 설정은 그대로 둔다(묶음 없는 장면용).

## 4. 단계

1. **조사(진행 중):** line_m·random_m team-spec 조사 에폭(잡 11393·11394) → 묶음과 검사 결과를 5 장에 적는다.
2. **구현(시험 먼저):** D1 등록부와 읽기, D2 런처 선택(쉘 시험), D3 대조 변경(`run_mismatches` 시험: seed 만 다른 경우 통과, 장면 내용이 다르면 거절), 상태의 `slam.mode`.
3. **Isaac 확인(확인 전용):** random_m·line_m 의 CT-2 순환을 묶음 위에서 sim 1 h 이어서 각 두 번(seed 는 조사와 다른 것 포함). 판정: keyframe 모두 < 1 s, slam_link_failed·무응답 없음, 위치 오차(응답 자세 대 참값) 중앙·최대가 지금 기본값의 같은 장면 Isaac 결과 이하, 임무 완료 수가 지금 기본값의 같은 구간보다 적지 않음. 결과로 설정을 고치면 새 확인 자료가 필요하다.
4. 총괄에 요약 보고 → 기본 전환(D2 적용) → PR·ws1 정리 인계.

## 5. 조사 결과

(잡 11393·11394 뒤에 적는다.)

## 6. 남는 위험

- 조사 지도의 world 오차는 오도메트리 방향 오차가 지배한다(PR #62 S2 진단). team-spec 오도메트리에서도 같을 수 있다.
- 시뮬은 정적 세계다(P4). 실차 창고의 바뀌는 배치는 이 계획이 다루지 않는다.
- 위치 추정의 `size()` 상한 함정(상위 계획 3 장)은 재생에서 해롭게 드러나지 않았지만 일반 보증은 아니다.

## 7. 자체 검토

(① 재계산, ② Codex 적대 검토, ③ 독립 검토를 아래에 덧붙인다.)
