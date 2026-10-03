# 임무 렌더링 — Isaac Sim (실행 기록)

상위: [운반 단계 수정 계획](../plans/2026-10-03-transport-stage-fixes.md) "렌더링" 절, [개발 실행 기록](2026-10-03-transport-stage-dev-run.md).
사용자(2026-10-03): "최종적으로 isaac sim으로 시뮬레이션 렌더링 단계까지 진행해", 렌더링은 채택 결정(채택)을 따른다.

**렌더링 실행은 평가가 아니다.** 성패는 각 실행의 `result.json` 으로만 판정하고(README 규칙), 영상은 그 실행을 보여 줄 뿐이다.

## 실행

- 코드: main `c295a1a`(PR #14 머지), ws1 snapshot `snapshots/render_c295a1a`. `run_transport.py` `53f6ef00…`(개발 실행과 같음).
- 인자: 평가와 같은 인자(`--obstacles 4 --max-sim-seconds 150 --use-perception --planning-target perception`, 잠정 차체 `dls08_provisional`, EPAL 6, bay)에
  `--video --camera-inset --robot-camera --extra-views chase` 를 더함. ws1 Slurm 작업 720(17 분 34 초), 합성 작업 722.
- 출력: `artifacts/20261003_render/`(로컬·ws1).

| seed | 보여 주는 것 | `result.json` | 시뮬 시간 | 관측 후보(번호) | cusp 재계획 | 삽입 위치 오차 |
|---|---|---|---|---|---|---|
| 2001 | 고정 관측점에서 한 번에 인식 → 접근·삽입·들어올림·운반·하역 | 성공 | 62.9 s | 1 | 0 | 7.70 mm |
| 3015 | 첫 관측(고정 후보 0) 기각 → 실행 중 관측점(후보 8)에서 재관측 → 완주 | 성공 | 120.1 s | 0, 8 | 0 | 7.74 mm |
| 3003 | 운반 중 cusp 에서 방향 어긋남 → 정지 후 재계획 → 완주 | 성공 | 83.5 s | 1 | 1(t 64.29 s, yaw 67.7 mrad, 새 경로 1.73 m) | 7.74 mm |

3003 의 재계획은 개발 실행(t 65.3 s, yaw 57.5 mrad, 새 경로 3.24 m)과 시각·오차·경로가 다르다 — 영상 실행과 평가 실행은 같은 seed 라도 물리 변동으로 갈린다.
개요 영상에서 운반 계획선은 재계획 직후 새 경로로 바뀐다(재계획 1 s 전과 3 s 뒤 프레임으로 확인).

## 산출물

seed 마다 `transport.mp4`(개요 1280×720·60 fps, 왼쪽 위에 인식 카메라 화면과 검출 추정·정렬 각), `view_chase.mp4`(추적 시점 1280×720), `camera_rgb.mp4`·
`camera_depth.mp4`(로봇 RGB-D 640×480). 합성: `side_<seed>.mp4`(개요 + 추적 나란히, 1920×620·30 fps, 위에 설명 자막), `reel.mp4`(세 장면 이어 붙임, 266.2 s).

| 파일 | sha256 앞 8 자리 |
|---|---|
| seed_2001 transport / view_chase / camera_rgb / camera_depth | `06ddd082` / `f7138050` / `c99ca351` / `2b6b88ca` |
| seed_3015 transport / view_chase / camera_rgb / camera_depth | `0bafb940` / `ae49758e` / `f0ffdfd9` / `f7710c05` |
| seed_3003 transport / view_chase / camera_rgb / camera_depth | `586d1afe` / `1d3992b7` / `97d08dcd` / `e42c4dfd` |
| side_2001 / side_3015 / side_3003 / reel | `5042d577` / `a51bd24d` / `799f733b` / `6047c369` |

## 이 기록이 말하지 않는 것

- 잠정 차체·시뮬레이터 정답 자세 피드백·합성 bay·EPAL 6 의 장면이다. 실측 차체·실물 카메라·T11 이 아니다.
- 세 장면은 성공한 seed 를 골라 보여 준 것이다. 성공률은 개발 실행(125/129)과 동결 평가 기록을 본다.
