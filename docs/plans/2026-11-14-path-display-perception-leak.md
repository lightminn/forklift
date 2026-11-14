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
| USD 렌더 패스 컬렉션(`UsdRenderPass` 의 `collection:renderVisibility`·`cameraVisibility`) | `renderVisibility` 문자열은 USD 스키마 라이브러리(`libusd_usdRender.so`·`libusd_usdGeom.so`)에만 있고 RTX·hydra 렌더러 플러그인에는 없다. 렌더러 쪽 컬렉션은 light/shadow/filter link 뿐이다 | 아니오. RTX 가 렌더 패스 가시성 컬렉션을 읽는 흔적이 없다 |
| 별도 렌더 레이어, 두 번째 UsdContext, 프로덕트별 render var 구성 | Camera·Replicator 경로가 기본 컨텍스트 하나를 쓴다. render var 는 출력(AOV) 선택이지 prim 선택이 아니다 | 아니오. 장면을 두 벌 유지해야 해서 범위가 크다 |
| `isaacsim.util.debug_draw` 오버레이 | 뷰포트 오버레이다 | 아니오. 렌더 프로덕트에 안 찍혀서 영상(이것도 렌더 프로덕트)에서도 빠진다 |
| 캡처 순간만 숨김 | 프레임마다 가시성을 바꾸고 렌더를 두 번 해야 한다 | 총괄이 제외했다 |

- 조사 방법: 확장 소스(`*.py`·`*.toml`·`*.usda`)에서 가시성 primvar·`omni:rtx:` 속성을 찾았다. RTX·hydra·replicator·usd 플러그인 `*.so` 의 문자열에서 설정 경로·컬렉션 이름을 찾았다.
- 결론(범위 한정): **지금 쓰는 Camera·Replicator 구성과 이 설치의 설정·스키마 경로에서는 prim 을 특정 렌더 프로덕트에서만 빼는 방법을 찾지 못했다.** 문서화되지 않은 경로가 없다는 증명은 아니다. matte 우회는 depth 의미가 바뀔 수 있어 미검증 대안으로만 남긴다. 그래서 차선책으로 간다.
- 웹 검색에서도 카메라별 prim 제외를 직접 다루는 문서는 찾지 못했다.

## 2. 수정: 장면에 그리지 않고 영상 프레임에 합성

- **러너의 표시 어댑터를 바꾼다.** `run_transport.py` 의 `isaac.add_path_display`(코어의 `show_path` 와 `add_path_display` 훅, 러너 6557–6560 이 모두 여기로 온다)가 장면에 prim 을 만들지 않는다. 대신 러너 안의 표시 목록(이름 → 월드 다각선, 색)에 넣는다. 같은 이름이면 덮어쓴다(지금 prim 경로 `/World/PlannedPaths/<이름>` 을 다시 Define 하던 것과 같은 뜻).
- **영상 싱크가 그 목록을 합성한다.** 영상 산출물과 정책은 다음과 같다.

| 산출물 | 입력 이미지 | 카메라 | 합성 |
|---|---|---|---|
| 개관 영상(`write_video_frame`) | 일반 영상 | `/World/GlobalCamera`, 고정 | 한다 |
| 추가 보기 quarter·chase | 일반 영상 | 고정(quarter), 매 프레임 이동(chase) | 한다 |
| 추가 보기 perception | RGB 절반 + depth 색 절반 | `PerceptionDisplayCamera`(표시 전용, 센서 아님) | RGB 절반에만 한다 |
| 로봇 카메라 영상(`record_robot_camera_frame`) | RGB, depth 색 | 같은 표시 카메라, 없으면 인식 카메라 | RGB 에만 한다 |
| 라이브 콘솔 영상(`live_camera()` → LiveRuntime) | RGB(640 폭으로 줄임) | `LiveVideoCamera`, 없으면 인식 카메라 | `live_camera()` 안에서 줄이기 전에 한다 |
| 카메라 인셋(`camera_inset`) | **인식 카메라 센서 RGB** | 인식 카메라 | 하지 않는다. 인셋은 센서가 본 것을 보여 주므로 선이 없어야 맞다 |

- 합성 방법:
  - 그 프레임의 카메라 월드 자세와 내부 파라미터로 다각선 점을 투영한다. 점의 높이는 지금처럼 z 0.012 m 다. 카메라 앞 0.05 m 평면에서 선분을 잘라 낸다.
  - 지금 색으로 그린다. 선 폭은 영상 폭 1280 px 기준 3 px 이고, 폭에 비례해 줄이되 최소 1 px 이다. 지금의 폭 0.018 m 리본(원근에 따라 굵기가 변함)과 같은 표현은 아니다. 표시 전용이라 받아들인다.
  - 가림 처리는 하지 않는다. 지게차·팔레트 위로도 선이 보인다. 이것만 지금 영상과 다르다. 그래서 `result.json` 에 `path_display: {"mode": "overlay", "occlusion": false}` 를 남긴다.
  - 기존 투영 도구(`mission_views.project_world_opening`, `camera_calibration`)를 다시 쓴다.
- **재생 계약.**
  - 재생에서는 러너의 Isaac 어댑터가 `add_path_display=lambda *a, **k: None` 이다(`tools/boundary_replay.py` 298 행). 그래서 목록이 늘 비어 있다.
  - 합성은 목록이 비었거나 프레임이 None 이면 아무것도 읽지 않고 돌아온다.
  - 카메라 자세는 그 카메라 객체(`isaacsim.sensors.camera.Camera.get_world_pose()`)에서만 읽는다. 녹화 프록시(`Vehicle`·`SimTruth`·`Clock` 등, `run_transport.py` 6343–6369·9023–9035)는 부르지 않는다. 카메라 객체는 프록시로 감싸지 않으므로, 새 기록에도 합성 때문에 생기는 포트 호출이 없다. 그래서 목록이 빈 재생과 새 기록이 어긋나지 않는다.
  - 카메라 객체가 주는 자세가 그 프레임의 렌더 자세와 같은지(로봇에 붙은 카메라의 USD 자세가 물리 뒤에 갱신되는지)는 확인되지 않았다. S3 에서 확인한다.
  - 내부 파라미터는 설정 때 읽어 둔 값(`state["camera"]`·`state["extra_views"]`·`state["live_video_camera"]` 등)을 쓴다.
- **옵션 `--path-display {overlay,stage,off}`, 기본 overlay.**
  - `stage` 는 지금 동작(장면 prim)이다. 수정 전 기록을 그대로 재현하는 진단용으로만 쓴다.
  - `off` 는 표시를 아예 하지 않는다.
  - 새 러너 인자라서 `near_tracking.py` 의 RUN_SPECIFIC_ARGUMENTS 에 등록한다. 표시 모드는 측정 조건이 아닌 출력 설정이라 조건 비교에서 뺀다.
  - 다만 `stage` 는 인식 입력을 바꾸므로 측정 조건이다. 그래서 비교 도구가 쓰는 실행 기록(`result.json` 의 `path_display.mode`)에 모드를 남긴다. 판정 도구는 두 팔의 모드가 같은지 확인한다.
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
- 재생 게이트 runs29: 29/29(`SETUP_INPUTS=recorded`). 옛 기록에는 `--path-display` 키가 없다. 비교는 현재 제공된 키만 보므로 충돌하지 않을 것으로 본다(`contract_arguments`). 기대일 뿐이라 게이트로 확인한다.
- ws1 전체 pytest.
- Isaac smoke(제출 전 총괄에 묻는다, 다른 Isaac 잡과 겹치지 않게):
  - S1 표시 없음과 결정성: aisle_rack s0·s1 B0, `--deterministic`, VIDEO=off, 같은 조건 2 회.
    - 통과 조건 1: 두 번의 결과와 원·근접 캡처 depth 해시가 같다.
    - 통과 조건 2: `/World/PlannedPaths` prim 이 없다(`scene.usda` 내보내기에서 확인).
    - 관측: 해시가 표시 끔 진단(잡 19043, B0 근접 `9bb0db71`·`6642789c`)과 같은지 본다. 진단은 `scene.add_path_display` 안에서, 이번 수정은 러너 어댑터에서 막는다. 둘 다 USD 연산을 하나도 하지 않으므로 같을 것으로 기대한다. 다만 스냅숏의 다른 코드가 달라 동치가 보장되지는 않는다. 다르면 머지 전에 원인을 찾는다.
  - S1r 새 기록의 재생: S1 실행 하나의 boundary 기록을 수정 스냅숏으로 재생해 `REPLAY_EXIT 0` 인지 본다. 합성이 새 포트 호출을 만들지 않는다는 확인이다.
  - S3 영상: aisle_rack s0 B0, VIDEO=on, extra views quarter·chase·perception, 로봇 카메라.
    - 합성된 선이 경로를 따라 그려지는지 프레임 몇 장을 눈으로 본다. 개관·chase(움직이는 카메라)·로봇 카메라를 다 본다.
    - 인셋(센서 RGB)에는 선이 없는지 본다.
    - 로봇 카메라의 `get_world_pose()` 가 그 시각의 참값 base 자세에 마운트를 합성한 값과 같은지 프레임마다 기록해 비교한다. 이 비교는 진단 전용 필드로, smoke 에서만 켠다.
    - 기준은 1 mm·0.1° 이다. 어긋나면 로봇 카메라의 합성 자세를 바꾼다. 다른 비프록시 출처를 찾는다.
  - S4 라이브(콘솔 세션과 맞춘다): live 영상에 선이 있는지만 확인한다.
  - 표시 끔 진단 실행(`test_runs/lcr/pdoff_27daf0b0`)은 S1 비교가 끝날 때까지 지우지 않는다. 해시는 #509 에 남아 있다.
- 기본 경로가 바뀌므로 머지 전 Isaac smoke 1 회는 필수다(재생 게이트 규칙).

## 5. 자체 검토

(검토 뒤 채운다: ① 원자료 재확인 ② Codex 적대적 검토 ③ 새 스레드 또는 agy "적히지 않은 전제")
