# 계획 경로 표시선의 인식 렌더 누출 수정 (2026-11-14)

- 발견: ① 적재 곡률 규칙 비교(#509 3 절). 계획 경로 표시선(`sim/isaac/scene.py` `add_path_display`, 렌더되는 `UsdGeom.BasisCurves`)이 인식 카메라 RGB·depth 에 찍힌다. aisle_rack s0·s1 에서 두 팔의 근접 캡처 depth 차이가 이것으로 설명되고(표시 끔 잡 19043 에서 차이 0 px), B0 의 근접 depth 도 표시를 끄면 바뀌었다(s0 `6a6b54af` → `9bb0db71`).
- 배정(총괄, 2026-11-14): 이 세션이 수정한다. 캡처 순간만 숨기는 방식은 쓰지 않는다(근접 추적이 매 틱 프레임을 쓴다). 인식 카메라에서 영구 제외하고 영상 카메라에는 남기는 방식을 먼저 조사하고, 어려우면 "인식이 켜진 실행에서는 표시를 끄고 영상에는 사후 합성" 을 검토한다. 게이트 기준을 어떻게 다룰지는 계획에 적어 보고하고, 재기록 여부는 총괄이 정한다.
- 영향 조사(최근 결론들)는 이 계획과 나란히, 기록만으로 한다. 이 문서의 범위가 아니다.

## 1. 카메라별 제외 조사 결과

ws1 의 Isaac Sim 5.1.0(`/opt/isaacsim-env`, extscache: omni.replicator.core 1.12.27, Kit 107.3)에서 확장 소스와 RTX·hydra 플러그인 문자열을 찾았다(2026-11-14).

| 후보 | 찾은 것 | 쓸 수 있나 |
|---|---|---|
| prim primvar `hideForCamera`(속성 창 "Invisible to Primary Ray"), `invisibleToSecondaryRays` | `omni.kit.property.geometry` 의 속성 위젯 | 아니오. prim 단위라 모든 카메라에서 함께 사라진다. 영상에서도 빠진다 |
| render purpose `guide` | 설정 `/persistent/app/hydra/displayPurpose/guide`. 뷰포트 메뉴도 이 전역 설정을 바꾼다 | 아니오. 렌더 프로덕트마다 다르게 줄 수 없다 |
| 렌더 프로덕트별 설정 덮어쓰기 `omni:rtx:<설정>` | Replicator 가 depthSensor·tvNoise 후처리에 쓴다 | 아니오. RTX 설정 문자열에 가시성·purpose 항목이 없다. `/rtx/matteObject/enabled` 를 프로덕트별로 켜고 표시선을 matte 로 두는 우회는 문서가 없고, matte 물체가 depth 에 어떻게 찍히는지도 모른다 |
| 별도 렌더 레이어, 두 번째 UsdContext | Camera·Replicator 경로가 기본 컨텍스트 하나를 쓴다 | 아니오. 장면을 두 벌 유지해야 해서 범위가 크다 |
| `isaacsim.util.debug_draw` 오버레이 | 뷰포트 오버레이다 | 아니오. 렌더 프로덕트에 안 찍혀서 영상(이것도 렌더 프로덕트)에서도 빠진다 |
| 캡처 순간만 숨김 | 프레임마다 가시성을 바꾸고 렌더를 두 번 해야 한다 | 총괄이 제외했다 |

- 결론: Isaac 5.1 에는 prim 을 특정 렌더 프로덕트에서만 빼는 지원 경로가 없다. 차선책으로 간다.
- 웹 검색에서도 카메라별 prim 제외를 직접 다루는 문서는 찾지 못했다.

## 2. 수정: 장면에 그리지 않고 영상 프레임에 합성

- **러너의 표시 어댑터를 바꾼다.** `run_transport.py` 의 `isaac.add_path_display`(코어의 `show_path` 와 `add_path_display` 훅, 러너 6557–6560 이 모두 여기로 온다)가 장면에 prim 을 만들지 않는다. 대신 러너 안의 표시 목록(이름 → 월드 다각선, 색)에 넣는다. 같은 이름이면 덮어쓴다(지금 prim 경로 `/World/PlannedPaths/<이름>` 을 다시 Define 하던 것과 같은 뜻).
- **영상 싱크가 그 목록을 합성한다.** RGB 영상 싱크 네 곳이다.
  - 개관 영상(`write_video_frame`의 `camera`, 1280×720)
  - 추가 보기(quarter·chase·perception)
  - 로봇 카메라 영상(`record_robot_camera_frame` 의 RGB. depth 색 영상에는 그리지 않는다)
  - 라이브 콘솔 영상(`live_camera`)
- 합성 방법:
  - 그 프레임의 카메라 월드 자세와 내부 파라미터로 다각선 점(z 0.012 m)을 투영한다. 카메라 뒤의 점은 선분 단위로 잘라 낸다.
  - 지금 색으로 2–3 px 선을 그린다.
  - 가림 처리는 하지 않는다. 지게차·팔레트 위로도 선이 보인다. 이것만 지금 영상과 다르다.
  - 기존 투영 도구(`mission_views.project_world_opening`, `camera_calibration`)를 다시 쓴다.
  - **새 Isaac 호출 없이 합성한다.** 재생에서 인식 카메라 같은 객체는 녹화 프록시이고, 기록에 없는 호출을 거부한다. 개관 카메라는 `_NoPicture` 라 모든 호출이 None 이다. 그래서 카메라 자세는 러너가 이미 가진 값에서 얻는다.
    - 개관·quarter: 카메라를 둘 때 쓴 eye·target
    - chase: `place_chase_camera` 에 넘긴 eye·target
    - 로봇 카메라·라이브: 이미 부르는 `annotation_pose()` 와 마운트 변환
    - 내부 파라미터는 설정 때 읽어 둔 값을 쓴다.
    - 목록이 비었거나 프레임이 None 이면 아무것도 하지 않는다. 재생에서는 어댑터가 no-op 이라 목록이 늘 비어 있다.
- **옵션 `--path-display {overlay,stage,off}`, 기본 overlay.**
  - `stage` 는 지금 동작(장면 prim)이다. 수정 전 기록을 그대로 재현하는 진단용으로만 쓴다.
  - `off` 는 표시를 아예 하지 않는다.
  - 새 러너 인자라서 `near_tracking.py` 의 RUN_SPECIFIC 에 등록한다.
- 범위 밖:
  - `scene.add_path_display` 함수 자체는 그대로 둔다(`stage` 모드가 쓴다).
  - `run_slam_drive.py`(조사 경로 표시): 인식 센서가 없다. 카메라는 영상용이고 LiDAR 는 PhysX 레이캐스트라 영향이 없어 그대로 둔다.
  - `run_perception_approach.py`·`run_pocket_insertion.py`: 지금 런처·도구가 부르지 않는 옛 러너라 그대로 두고, 문서에 누출이 남아 있다고 적는다.
- 영향이 없는 것: LiDAR(`planar_lidar.py` PhysX 레이캐스트, 표시선은 충돌체가 없다), SLAM, 장애물 층.

## 3. 게이트 기준의 처리(총괄 결정 사항)

- 재생 게이트(`replay_gate.sbatch` → `tools/boundary_replay.py`)는 기록된 입력으로 코어를 CPU 에서 다시 돌린다. 렌더하지 않는다.
  - 재생의 Isaac 어댑터는 `add_path_display=lambda *a, **k: None` 이다(298 행). `show_path` 는 러너에서 이 어댑터를 부르는 람다다.
  - 이번 수정은 코어 호출(`hooks.show_path`·`add_path_display`)을 바꾸지 않는다. 바뀌는 것은 러너 쪽 어댑터 구현과 영상 싱크뿐이다. 틱 안에서 새 포트를 부르지 않는다.
  - 그래서 **runs29 기준 그대로 29/29 를 기대하고, 재기록은 필요 없다고 본다(권고).** 머지 전에 게이트를 돌려 확인한다.
- 다만 게이트 기록의 인식 입력은 표시선이 찍힌 렌더에서 나왔다. 게이트는 "코어가 기록을 그대로 재생하는가" 만 본다. 그래서 이 사실이 판정을 무효로 만들지는 않는다. 그러나 그 기록을 새 Isaac 실행과 결과 수준에서 비교하는 데는 쓸 수 없다.
- **비교 규칙(제안):**
  - 수정 뒤의 Isaac 결과를 수정 전 기준선과 섞지 않는다. 두 팔은 같은 코드와 같은 `--path-display` 로 돌린다.
  - 수정 전 결과를 재현해야 할 때만 `--path-display stage` 를 쓴다.
- 기록을 다시 만들지는 총괄이 정한다. 위 권고는 "게이트 기준 재기록 불필요, 비교 기준선은 수정 뒤 새로" 다.

## 4. 검증

- 단위 시험:
  - 표시 목록의 덮어쓰기
  - 투영(합성 카메라에서 알려진 점이 알려진 픽셀로 가는지, 카메라 뒤 선분 자르기)
  - 모드별 동작: overlay 는 prim 이 없고 목록에 있음, stage 는 prim, off 는 둘 다 없음
  - 옵션 파싱
  - near_tracking 등록
- 재생 게이트 runs29: 29/29(`SETUP_INPUTS=recorded`).
- ws1 전체 pytest.
- Isaac smoke(제출 전 총괄에 묻는다, 다른 Isaac 잡과 겹치지 않게):
  - S1 동치: aisle_rack s0·s1 B0, `--deterministic`, VIDEO=off. 원·근접 캡처 depth 해시가 표시 끔 진단(잡 19043, `9bb0db71`·`6642789c`)과 같아야 한다. 장면에 표시선이 없다는 점은 두 경우가 같다. `/World/PlannedPaths` prim 이 없는지도 기록에서 본다.
  - S2 결정성: S1 을 한 번 더 돌려 결과·depth 해시가 같은지 본다.
  - S3 영상: aisle_rack s0 B0, VIDEO=on, 개관·로봇 카메라. 합성된 선이 경로를 따라 그려지는지 프레임 몇 장을 눈으로 본다. 그리고 인식 RGB 에 선이 없는지 본다.
  - S4 라이브(필요하면 콘솔 세션과 맞춘다): live 영상에 선이 있는지만 확인한다.
  - 표시 끔 진단 실행(`test_runs/lcr/pdoff_27daf0b0`)은 S1 비교가 끝날 때까지 지우지 않는다. 해시는 #509 에 남아 있다.
- 기본 경로가 바뀌므로 머지 전 Isaac smoke 1 회는 필수다(재생 게이트 규칙).

## 5. 자체 검토

(검토 뒤 채운다: ① 원자료 재확인 ② Codex 적대적 검토 ③ 새 스레드 또는 agy "적히지 않은 전제")
