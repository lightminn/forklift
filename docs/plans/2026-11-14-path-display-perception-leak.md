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
  - **좌표는 지금 prim 과 같다.** 경로 점(계획기 좌표, SLAM 이 켜지면 지도 좌표)을 그대로 월드 좌표로 그린다. 지금 prim 도 그렇게 그렸다. 그래서 SLAM 드리프트가 있으면 선이 바닥에서 어긋나 보이는 것도 지금과 같다.
  - **카메라 자세는 그 카메라 객체에서 읽는다(`isaacsim.sensors.camera.Camera.get_world_pose()`).** `annotation_pose()` 는 쓰지 않는다. 그것은 SLAM 이 켜지면 추정 자세를 주고, 포크 리프트도 빠져 있다(인식·표시 카메라는 `perception_mount` 가 legacy 가 아니면 `fork_carriage` 아래에 붙는다).
  - **내부 파라미터도 카메라 객체에서 읽는다.** 목록이 처음 비지 않았을 때 그 카메라에서 한 번 읽어(`perception_adapter.read_isaac_intrinsics`) 합성 모듈 안에 둔다. 개관·quarter·chase 는 지금 설정 코드가 내부 파라미터를 읽지 않으므로 이렇게 새로 얻는다. 라이브의 인식 카메라 대체 경로처럼 이미지를 줄인 경우에는, 이미지 크기와 카메라 해상도의 비로 내부 파라미터를 줄인다.
  - **픽셀과 자세의 시각을 맞춘다.**
    - 결정 모드는 `frame_sync` 가 그 프레임 자신의 RGB 를 주므로, 쓰는 순간의 카메라 자세가 픽셀의 자세다.
    - 비결정(라이브 등)에서는 렌더 픽셀이 몇 스텝 늦다(`live_runtime.py` 의 `camera_pixel_lag_s`). 움직이는 카메라(chase·로봇·라이브)는 스텝마다 카메라 자세를 짧은 고리 버퍼에 쌓고, 프레임의 `rendering_time` 에 가장 가까운 자세로 투영한다. 버퍼는 목록이 비지 않았고 움직이는 카메라 싱크가 켜졌을 때만 쌓는다.
    - 어긋남의 크기는 S3·S4 에서 잰다.
  - **원본 버퍼에 그리지 않는다.** 늘 복사본(`np.array(..., copy=True)`)에 그린다. `rgba[:, :, :3]` 은 뷰이고, 인식 카메라로 대체되는 싱크(로봇 카메라·라이브)에서는 그 버퍼가 인식 입력이다.
  - 점의 높이는 지금처럼 z 0.012 m 다. 카메라 앞 0.05 m 평면에서 선분을 자른 뒤 투영한다(0 에 가까운 깊이에서 좌표가 발산하지 않게).
  - 다각선은 200 점 이하로 줄인다(`live_runtime.PlanFeed._thin` 과 같은 상한). 투영은 numpy 로 한 번에 하고, 그리기는 cv2 polyline 이다. `--live` 의 스텝 CPU 시간(`live_timers`)을 S4 에서 본다.
  - 지금 색으로 그린다. 선 폭은 영상 폭 1280 px 기준 3 px 이고, 폭에 비례해 줄이되 최소 1 px 이다. 지금의 폭 0.018 m 리본(원근에 따라 굵기가 변함)과 같은 표현은 아니다. 표시 전용이라 받아들인다.
  - 가림 처리는 하지 않는다. 지게차·팔레트 위로도 선이 보인다. 이것이 지금 영상과 다르다. 그래서 `result.json` 에 `path_display: {"mode": "overlay", "occlusion": false}` 를 남긴다(남기는 곳은 아래 재생 계약).
  - **순서:**
    - 개관 영상은 선을 먼저 그린 뒤 인셋을 붙인다(`compose_frame`). 그래서 인셋 패널은 선에 덮이지 않는다. 인셋 안 센서 영상에는 선이 없다. `stage` 에서는 인셋 안에도 선이 보였다.
    - 로봇 카메라는 선을 먼저, 검출 표시(`annotate_detection`)를 그 위에 그린다.
    - perception 추가 보기도 같은 순서로 그린다.
  - 투영은 새 함수로 쓴다. `mission_views.project_world_opening` 은 base_link 와 ROS optical 마운트를 전제해 월드 카메라에 바로 쓸 수 없다. 핀홀 투영(`forklift_core.perception.overlay.project_point` 의 규약)만 다시 쓴다.
- **재생 계약.**
  - 재생에서는 러너의 Isaac 어댑터가 `add_path_display=lambda *a, **k: None` 이다(`tools/boundary_replay.py` 298 행). 그래서 목록이 늘 비어 있다.
  - 목록과 내부 파라미터 저장소는 새 모듈 `sim/isaac/path_overlay.py` 의 모듈 상태로 둔다. `run()` 이 새 이름을 묶어 `drive()` 가 `_bound` 에서 읽는 일이 없다. 그러면 옛 setup 기록에 없는 이름 때문에 재생이 실패하지 않고, `SETUP_NAMES_ADDED` 도 필요 없다.
  - `drive()` 안에서 `state` 나 비교되는 컨테이너에 새 키를 쓰지 않는다. `final_projection` 해시가 바뀌지 않게 하려는 것이다. 모드는 `args` 에 있고 `recorded_arguments` 로 이미 기록된다. `path_display.occlusion` 같은 표기는 결과를 쓰는 마지막 단계에서 `result.json` 에만 넣는다. 그 위치가 투영 비교 밖인지 구현 때 `PROJECTION_CONTAINERS` 와 대조한다.
  - `drive()` 가 모드를 읽어야 하면 `getattr(args, "path_display", "stage")` 로 읽는다. 옛 기록의 Namespace 에는 이 키가 없다.
  - 합성은 목록이 비었거나 프레임이 None 이면 아무것도 읽지 않고 돌아온다.
  - 카메라 객체는 녹화 프록시가 아니다(프록시는 `Vehicle`·`SimTruth`·`Clock`·`Capture` 등, `run_transport.py` 6343–6369·9023–9035). 그래서 새 기록에도 합성 때문에 생기는 포트 호출이 없다. 목록이 빈 재생과 새 기록이 어긋나지 않는다(S1r 로 확인).
  - 카메라 객체가 주는 자세가 그 프레임의 렌더 자세와 같은지는 확인되지 않았다. 로봇에 붙은 카메라의 USD 자세가 물리 뒤에 갱신되는지가 문제다. S3 에서 확인한다.
- **옵션 `--path-display {overlay,stage,off}`, 기본 overlay.**
  - `stage` 는 지금 동작(장면 prim)이다. 수정 전 기록을 재현하는 **진단용**으로만 쓴다(총괄 결정).
    - 운영·평가 런처는 `stage` 를 받으면 시작을 거부한다. 대상은 `isaac_live.sbatch`·`riib_scene.sbatch`·`slam_survey.sbatch`, 그리고 `isaac_bundle.py check` 다(실행 목록에 `--path-display stage` 가 있으면 거부).
    - 러너는 `stage` 일 때 결과에 진단 표시를 남긴다.
  - `off` 는 표시를 아예 하지 않는다.
  - **조건 비교: `near_tracking.POST_MEASUREMENT_ARGUMENTS` 에 `"path_display": "stage"` 로 넣는다(권고).**
    - 옛 측정 기록(키 없음)은 수정 전, 곧 stage 로 읽힌다.
    - 그래서 overlay 실행을 옛 측정 기록과 비교하면 조건이 다르다고 거부된다. 수정 전·후를 섞지 않는다는 규칙을 도구가 지킨다.
    - stage 로 낸 진단 실행은 옛 기록과 맞는다.
    - RUN_SPECIFIC 에 넣으면 비교에서 빠져 섞임을 막지 못하므로 그렇게 하지 않는다.
    - 대가가 있다. 근접 추적 측정 기록(S4 계열)과 비교해야 하는 새 실행은 측정을 다시 할 때까지 거부된다. S4 는 지금 사용자 보류다.
  - 비교 판정 도구(k1_judge·lcr_judge 같은 것)는 두 팔의 `arguments.path_display` 가 같은지 확인한다.
- **목적지 고리(`scene.add_destination`, `/World/Destination`)도 같은 종류의 표시 전용 prim 이다.** z 0.006 m 의 녹색 발광 고리이고 충돌체가 없다. 인식 카메라에 찍힐 수 있다(하역 칸 근처의 다음 팔레트 원거리 인식 등). **이번 수정에 넣을지는 총괄 결정 사항이다.** 권고는 같은 모드로 함께 처리하는 것이다. overlay 면 영상에만 고리를 그린다. 바닥 패치(`add_scene_boxes` 의 2 mm floor)는 장면의 바닥 표지라 실물에도 있을 수 있어 그대로 둔다.
- 범위 밖:
  - `scene.add_path_display` 함수 자체는 그대로 둔다(`stage` 모드가 쓴다).
  - `run_slam_drive.py`(조사 경로 표시): 인식 센서가 없다. 로봇 카메라 RGB·depth 는 영상 출력일 뿐이고, LiDAR 는 PhysX 레이캐스트라 영향이 없어 그대로 둔다.
  - `run_perception_approach.py`·`run_pocket_insertion.py`: 지금 런처·도구가 부르지 않는 옛 러너라 그대로 두고, 문서에 누출이 남아 있다고 적는다.
  - `scene.usda` 내보내기에서 `/World/PlannedPaths` 가 빠진다. 이것을 읽는 도구가 없는지 구현 때 확인한다(`tools/list_rigid_bodies.py`·`tools/slam_survey_bundle.py`·`sim/isaac/lidar_bench.py`·`tools/p0b_sensor_study.py`).
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
  - 투영:
    - 합성 카메라에서 알려진 점이 알려진 픽셀로 가는지 본다(USD 축 카메라와 ROS 축 카메라 둘 다).
    - 근평면 0.05 m 자르기와 0 근처 깊이에서 좌표가 발산하지 않는지 본다.
    - 줄인 이미지에서 내부 파라미터를 비례로 줄이는지 본다.
  - 복사본에만 그리는지: 원본 배열이 바뀌지 않아야 한다.
  - 모드별 동작: overlay 는 prim 이 없고 목록에 있음, stage 는 prim, off 는 둘 다 없음
  - 옵션 파싱
  - `POST_MEASUREMENT_ARGUMENTS` 등록. 키 없는 옛 기록은 stage 와 같고, overlay 와는 다르게 판정되는지 본다.
  - 운영 런처의 stage 거부
  - `recorded_arguments` 관련 기존 시험(`tests/unit/test_pallet_mass.py` 등)의 갱신
- 재생 게이트 runs29: 29/29(`SETUP_INPUTS=recorded`). 기대이지 검증이 아니다. 29/29 가 안 나오면 원인을 먼저 총괄에 보고한다(총괄 지시). 재기록은 총괄이 정한다.
- ws1 전체 pytest.
- Isaac smoke(제출 전 총괄에 묻는다, 다른 Isaac 잡과 겹치지 않게). 모두 수정 스냅숏 하나에서 돈다.
  - **S1 센서 입력 불변과 결정성:** aisle_rack s0·s1 B0, `--deterministic`, VIDEO=on(영상 싱크가 모두 돌게), extra views quarter·chase·perception, 로봇 카메라.
    - 팔 셋을 돌린다: overlay ×2, off ×1.
    - 통과 조건 1: overlay 두 번과 off 의 결과, 원·근접 캡처 depth 해시, 인식 RGB 해시가 모두 같다. 합성이 센서 입력을 건드리지 않는다는 직접 확인이다(버퍼 aliasing 포함).
    - 통과 조건 2: `/World/PlannedPaths` prim 이 없다(`scene.usda` 내보내기에서 확인).
    - 관측: 근접 depth 해시가 표시 끔 진단(잡 19043, VIDEO=off, B0 근접 `9bb0db71`·`6642789c`)과 같은지 본다. VIDEO·스냅숏이 달라 동치는 보장되지 않는다. 다르면 머지 전에 원인을 찾는다.
  - **S1r 새 기록의 재생:** S1 실행 하나의 boundary 기록을 수정 스냅숏으로 재생해 `REPLAY_EXIT 0` 인지 본다. 합성이 새 포트 호출을 만들지 않는다는 확인이다.
  - **S3 정렬:** aisle_rack s0 B0, VIDEO=on. 진단 전용 환경 변수로 stage prim 과 overlay 를 함께 그린다. overlay 는 대비색이고, smoke 에서만 쓰며 운영 런처는 거부한다.
    - 영상 카메라마다 렌더된 선(주황 픽셀)과 합성 선 사이의 픽셀 거리를 잰다. 대상은 개관·quarter·chase·perception 보기·로봇 카메라다.
    - 정지 구간과 선회 구간을 따로 잰다. 기준은 정지 2 px 이하다. 선회는 결정 모드에서 2 px 이하, 비결정은 값을 기록한다.
    - 같은 실행에서 로봇 카메라의 `get_world_pose()` 를 그 시각의 참값 base 자세 + 리프트 + 마운트와 비교한다(1 mm·0.1°). 진단 필드로, smoke 에서만 켠다.
    - 기준을 못 넘는 싱크는 그 싱크의 합성을 끄고 문서에 적는다. 다른 싱크 결과는 유지한다.
    - S3 는 센서 입력이 바뀌는 실행이라 결과는 정렬 측정에만 쓴다.
  - **S4 라이브(콘솔 세션과 맞춘다, 총괄이 콘솔에 확인 요청함):** live 영상에 선이 있는지, 비결정 어긋남, `live_timers` 의 스텝 CPU 시간 변화를 본다. PR 리뷰에 콘솔 라이브 경로를 포함한다(총괄 지시).
  - 표시 끔 진단 실행(`test_runs/lcr/pdoff_27daf0b0`)은 S1 비교가 끝날 때까지 지우지 않는다(총괄이 보류함). 해시는 #509 에 남아 있다.
- 기본 경로가 바뀌므로 머지 전 Isaac smoke 1 회는 필수다(재생 게이트 규칙).
- **수정 전·후 Isaac 결과를 섞지 않는다(총괄 결정).** PR 과 결과 문서에 이 규칙을 적는다. 수정 뒤 첫 비교부터 새 기준선이다.

## 5. 자체 검토

- ① 원자료 재확인(Claude): 인용한 코드 줄을 main c84473f2 에서 다시 확인했다.
  - 러너 6557–6560·7268·7285·5313
  - `boundary_replay.py` 298·176
  - near_tracking 의 조건 비교 1395–1418
  - 확장·플러그인 문자열 조사(1 절)
  - 이 과정에서 재생 프록시에 대한 제약을 찾아 2 절에 넣었다. 개관 카메라는 재생에서 `_NoPicture` 다.
- ② Codex 적대적 검토(스레드 01a1d7d7):
  - 1 회차 P1 2 건과 P2 4 건을 처리했다:
    - S1 동치를 관측으로 낮춤
    - 라이브 싱크
    - 게이트 조건 명시
    - perception 보기·인셋 정책
    - 카메라 자세 출처
    - 조사 결론의 범위
  - P3 2 건(가림 표기, 선 폭·z·근평면)도 반영했다.
  - 2 회차 P2 2 건을 처리했다:
    - 모드 일치 검사의 위치: `POST_MEASUREMENT_ARGUMENTS` 로 옮기고 판정 도구가 확인
    - 라이브 카메라 모델과 렌더 시각
- ③ agy(Gemini 4 Hydrogen High) "적히지 않은 전제", ①②를 보지 않은 독립 검토. 처리:
  - 리프트·SLAM: `annotation_pose` 대신 카메라 객체 자세를 쓴다. 경로 좌표는 지금 prim 과 같다.
  - 재생: 새 setup 이름을 만들지 않는다. `drive()` 안에서 `state` 에 새 키를 쓰지 않는다. `getattr` 기본값을 쓴다.
  - 월드 카메라 내부 파라미터와 투영 도구: 카메라 객체에서 읽고, 새 투영 함수를 쓴다.
  - 픽셀 지연: 고리 버퍼를 쓰고 S3·S4 에서 잰다.
  - 근평면 자르기와 성능.
  - 버퍼 aliasing: 복사본에만 그리고, S1 에서 overlay·off 를 같은 스냅숏으로 비교한다.
  - 목적지 고리: 총괄 결정 사항으로 올렸다.
  - 그리는 순서.
  - matte·`UsdRender.Product` 실측: 하지 않았다. overlay 방식을 총괄이 승인했다(2026-11-14). matte 는 미검증 대안으로만 남긴다.
- 총괄 결정(2026-11-14):
  - overlay 방식 승인
  - 게이트는 재기록 없이 29/29 를 기대하고, 안 나오면 원인을 먼저 보고
  - 수정 전·후 Isaac 결과를 섞지 않음
  - stage 는 진단 전용이고 운영·평가 런처는 거부
  - 콘솔 라이브 경로를 리뷰에 포함
