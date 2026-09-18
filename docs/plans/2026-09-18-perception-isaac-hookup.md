# 3순위: 기존 팔레트 인식기를 Isaac 운반에 연결 — 설계

작성일: 2026-09-18. 범위: [`docs/plans/2026-09-17-project-status-and-next-steps.md`](2026-09-17-project-status-and-next-steps.md) 3순위. 로드맵상 위치는
[`docs/plans/2026-09-11-development-roadmap.md`](2026-09-11-development-roadmap.md)의 M5 인근(개발 5–6주차)이지만, 차체가
아직 입고되지 않아 H0/H1이 막혀 있는 동안의 **병행 트랙**(§7 위험표의 "차체 입고 지연 → 센서 데이터·오프라인 추적 개발
지속" 대응)으로 지금 진행한다. 완료돼도 M4/M5 마일스톤으로 세지 않는다.

**개정 이력:** v1(최초 초안) → Codex 적대적 검토 1라운드에서 P1 결함 2건(포켓 좌표 변환 누락, 장애물 지도 오염)과
관측 시점 계산 오류를 지적받아 v2로 전면 수정. v1의 "시작점이 사거리 밖(5.6m)"이라는 판단은 카메라 위치를
후축으로 잘못 계산한 내 오류였음을 직접 재계산으로 확인했다(후축(-2.34,0)이 아니라 카메라(-1.25,0) 기준
4.20m — `range_max_m=5.0` 안).

## 범위와 경계

- **바꾸는 것:** 팔레트 픽업 위치·자세만. 지금까지 `scenario.pickup`(ground truth)을 그대로 계획 목표로 썼던 것을,
  Isaac에서 렌더링한 RGB-D 프레임에 기존 `detect_pockets()`를 실제로 돌려 얻은 추정치로 바꾼다.
- **바꾸지 않는 것(문서 78줄, 3순위 항목 5):** 로봇 위치·장애물 지도는 여전히 시뮬레이터 정답으로 유지한다.
  **[P1 수정, v2]** 이는 `plan_transport()`의 목표 웨이포인트(`site_poses(scenario.pickup, ...)`)만 추정치로 바꾸고,
  충돌 회피에 쓰는 팔레트 장애물 사각형(`Rectangle(scenario.pickup.x_m, ...)`, `pallet_mission.py:404`)은 정답을 그대로
  유지해야 한다는 뜻이다. 지금의 `plan_transport(scenario, ...)`는 목표와 장애물을 **같은** `scenario.pickup`에서
  만들므로, 이 둘을 분리하려면 `plan_transport`에 목표용 픽업과 장애물용 픽업을 별도로 받는 매개변수가 필요하다
  (아래 "필요한 코드 변경" 참조 — 이 변경 자체도 구현 전 별도 검토 대상).
- **1회 관측, 폐루프 추적 아님:** 지정된 관측 지점에 도달해 한 번만 촬영·인식한다. 접근 중 연속 재관측·재계획은
  4순위 범위이며 여기서는 하지 않는다. 인식 실패 시 정지·실패로 끝난다(재시도 없음, 첫 버전 범위 밖).

## 관측 지점: 별도의 고정 웨이포인트, `scenario.pickup` 중간 지점이 아니다

**[v2, Codex 반대 의견 수용]** v1은 "접근 시작 직전"이라고만 적었는데, 이는 두 가지로 잘못 해석될 수 있었다:
(a) 진짜 시작점(후축 `scenario.start_rear`) — 계산해보면 이것도 사거리 안(카메라 기준 4.20m, seed 0)이라 문제
없다. (b) `scenario.pickup`으로 만든 접근 경로 중간의 한 지점 — **이건 안 된다.** 정답 목표로 이미 경로를 만든
뒤 그 경로 위에서 처음 인식하면, 인식이 실패해도 이미 정답 쪽으로 절반쯤 다가간 상태라 3순위가 증명하려는
"정답을 계획 입력으로 안 썼다"는 증거가 오염된다.

**설계: 관측 전용 고정 웨이포인트를 하나 둔다.** 이 웨이포인트는 시나리오의 seed(즉 실제 `pickup` 좌표)와
무관하게 미리 고정한 좌표다 — `make_scenario()`가 `pickup.x_m ∈ [2.6, 3.6]`, `pickup.y_m ∈ [-0.2, 1.5]`로
난수 배치하지만 그 분포의 대략적 중심·방향은 시나리오 설계 자체가 이미 고정하고 있으므로(실제 창고에서
"몇 번 랙에 팔레트가 있다"는 사전 지식과 같은 위상), 이 범위의 대략적 중앙을 향해 미리 정한 한 지점으로
이동하는 것은 개별 seed의 정답 좌표를 쓰는 것과 다르다.

- 후보(Codex가 seed 0 CPU 합성으로 직접 확인, `valid`): 후축 `(-0.10, 0.90, yaw=0)`. 카메라-전면 거리 약 2.04m.
- **[정정, v3] 실제 CPU 결과:** 원래 시작점(후축 `(-2.34, 0)`)에 실제 props 근사 상자를 추가하면
  `no_pallet: no_opening_pattern`이었고, 관측 웨이포인트 후보 `(-0.10, 0.90, 0)`에 같은 props를 둔 조건은
  `valid`였다(v2에서 이 둘을 반대로 적었던 것을 바로잡음 — Codex가 실행하지 않은 결과를 실패로 잘못 옮겼다).
- **미해결로 남기는 것:** 이 지점이 다른 seed·다른 랜덤 장애물 배치에서도 유효한지는 seed 0 하나로 확정할 수
  없다. 장애물의 랜덤 배치가 인식 성패에 실제로 영향을 준다는 것은 이미 확인됐으므로, **모든 seed에 통하는
  단일 고정 웨이포인트는 없을 수 있다.** 구현 단계에서는 여러 seed에 대해 후보 웨이포인트를 CPU 합성으로
  먼저 스윕해 검출 성공률을 확인하고, 실패하는 seed는 "관측 실패"로 정직하게 기록한다(재시도·다중 웨이포인트
  탐색은 4순위로 유보).
- **[P1 추가, v3] 관측 웨이포인트로 가는 이동 자체의 목표 좌표는 `scenario.pickup`을 참조하지 않지만, 그
  이동의 충돌 회피 지도에는 진짜 팔레트(`scenario.pickup` 기반 `Rectangle`)를 반드시 포함해야 한다** — "정답을
  전혀 참조하지 않는다"는 v2 표현은 틀렸다. 관측 지점으로 이동하다 실제 팔레트에 부딪히면 안 되기 때문이다.
  `run_transport.py`는 현재 `phase == "approach"`일 때만 팔레트를 장애물로 검사하므로(469행), 새로 추가하는
  "관측 이동" phase에도 이 검사를 똑같이 적용해야 한다.
- 실행 순서: **관측 웨이포인트로 이동(목표 좌표는 정답 무관, 충돌 지도는 정답 팔레트 포함) → 정지·정착 →
  명시적 강제 렌더 → RGB-D 1회 캡처 → 인식 → 유효하면 픽업 추정치로 최종 접근·삽입 계획 → 무효하면 종료.**

## 카메라 설계

기존 Gazebo 경로가 이미 세 후보를 문서화해뒀다(`sim/gazebo/build_scene_world.py:53` `APPROVED_CAMERAS`):
`baseline_0p50`(0.75, 0, 0.50m, 모든 2026-09 측정의 기준), `low_0p27`(근거리용, 원거리 손실),
`mast_0p90`(브리핑이 실제로 요구하는 마스트 높이, 근거리 1.90m 미만 전부 미검출로 측정됨).

**선택: `baseline_0p50`을 그대로 쓴다.** 이유는 v1과 동일 — 지금까지의 모든 EPAL6 검출률·오차 수치가 이
마운트 기준이라, 마운트를 동시에 바꾸면 "인식 결과로 계획이 움직이는가"라는 이번 시험의 변수에 "새 마운트의
검출 성능"이라는 두 번째 미지수가 섞인다. **[v2 경고, Codex 지적]** 다만 이 마운트에서 seed 0 팔레트 전면을
투영하면(**원래 시작점 `(-2.34,0)` 기준, 관측 웨이포인트 기준이 아님**) 화면에서 약 u=243–329, v=279–297,
**포켓 높이가 약 8.7픽셀**밖에 안 된다 — 기하학적으로는 화각 안이지만 점 개수·경계 샘플링에 민감한 크기다.
"기존 기준선이므로 가시성도 확보됐다"고 결론 내리지 않는다. 관측 웨이포인트(카메라-전면 약 2.04m)에서는
거리가 더 가까워 픽셀 크기가 커지지만 이 값도 아직 재계산하지 않았다. 실제 Isaac 렌더(자기 차체·포크 가림,
실제 깊이 잡음 포함)로 확인하기 전까지 이 항목은 열어 둔다.

| 파라미터 | 값 | 근거 |
|---|---|---|
| 장착 위치 (base_link 기준) | (0.75, 0.0, 0.50) m | `APPROVED_CAMERAS["baseline_0p50"]` |
| 광학 자세 | `_LEVEL = [-0.5, 0.5, -0.5, 0.5]` (xyzw) | 동일 소스 — base +x(전방)를 광학 +z로 매핑하는 REP-103 광학 프레임. **[v2] Isaac의 로컬 pose 설정 API는 쿼터니언을 wxyz 순서로 받는다** — 어댑터에서 xyzw→wxyz 순서 변환을 명시적으로 하고, 단위시험으로 변환 함수를 검증한다. **[v3, Codex 지적: 순서 변환만으로는 안 끝난다]** Isaac `Camera`는 `camera_axes` 인자(`"world"`/`"usd"`/`"ros"`)로 카메라 로컬 좌표축 자체의 정의가 다르다 — `_LEVEL`이 인코딩하는 "광학 +z가 base +x"라는 관계는 어떤 `camera_axes` 모드를 쓰느냐에 따라 실제로 적용해야 할 회전이 달라진다. 설계는 어떤 `camera_axes` 모드를 쓸지 명시하고, "base 전방의 알려진 점이 영상 중심 부근에 투영된다"는 최소 시험으로 최종 조합(쿼터니언 순서 변환 + `camera_axes` 선택)이 맞는지 확인해야 한다. **[v4, 2026-09-19 ws1 실측으로 확정] `camera_axes="ros"` + `xyzw_to_wxyz(OPTICAL_QUATERNION_XYZW)`가 정답이다.** `sim/isaac/verify_perception_camera.py`로 base_link 전방 2.0m·높이 0.3m에 발광 마커를 두고 렌더한 결과, 마커가 영상 중심 근처(analytic 예측 (320, 314.5)px 대비 실측 centroid (346.0, 289.2)px, 거리 55.6px)에 선명하게 나타났다 — 육안 확인도 완료(첨부 이미지, 마커가 중앙 살짝 아래·오른쪽). `"world"`/`"usd"` 모드는 시험하지 않았다(첫 후보가 통과해 추가 확인 생략, 필요하면 재현 가능). |
| 해상도 | 640×480 | 동일 소스 `_SENSOR` |
| fx = fy | **465.741156 px** (Isaac 기본 FOV가 아니라 `tools/scene_rig.py`의 `FOCAL_PX`를 강제) | 아래 "카메라 → SceneInput 어댑터" 참조 |
| 주기 | 1회(웨이포인트 도달·정착 후), 스트리밍 아님 | 위 절 |
| 장착 링크 | **[v2, Codex 채택] `/World/Forklift/base_link`의 실제 USD 자식 prim으로 심는다.** 매 프레임 `robot.get_world_pose()`로 수동 재배치하지 않는다 — 부모 변환을 상속하면 로봇이 움직여도 자동으로 따라오고, "정지 상태에서 pose 설정 → 렌더 → 그 프레임의 시각·pose를 묶는" 수동 동기화 문제가 줄어든다. base_link 하위 scale이 1인지, 변환 상속이 끊기지 않는지 확인한다. | Codex 검토, [Isaac 5.1 Camera API] |

## Isaac 카메라 → `SceneInput` 어댑터

새 함수 `sim/isaac/perception_adapter.py::capture_scene_input(camera, base_from_optical, stamp_ns) -> SceneInput`.

1. **강제 렌더와 캡처 준비 확인 (v3, Codex 추가 지적 — "표준편차>2"만으로는 부족하다):** 지금
   `run_transport.py:295`의 정착 대기는 `world.step(render=args.video)`다 — `--video` 없이 실행하면 정착
   120프레임 동안 카메라가 전혀 렌더되지 않는다. 인식용 캡처는 **`--video` 여부와 무관하게** 캡처 직전
   `world.step(render=True)`를 강제 실행해 카메라를 워밍업한다. 준비 완료 조건은 다음 네 가지를 **모두**
   만족해야 한다 (RGB 표준편차 조건 하나만으로는 부족하다는 지적을 반영):
   - RGB가 `(H,W,4)`이고 `np.std(rgb[...,:3]) > 2` (기존 `probe_final.py` 관례)
   - depth 배열(`distance_to_image_plane`)이 실제로 채워져 있음(annotator 준비 완료)
   - 매 반복이 **새 프레임**임을 확인(같은 프레임을 두 번 읽는 것을 방지 — 프레임 카운터나 렌더 타임스탬프 비교)
   - **유한한 대기 상한**을 두고, 상한 초과 시 "capture_failed"로 종료(정지 상태에서도 `world.step()`은 물리
     시간을 계속 진행시키므로 무한 대기하지 않는다)

   RGB·depth·base pose는 **같은 시점**이어야 한다 — 카메라 pose를 옮기거나 로봇이 막 정지한 직후 곧바로
   `get_current_frame()`을 읽는 순서는 금지한다(Codex 지적).
2. **RGB:** `camera.get_rgba()[:, :, :3].astype(np.uint8)`.
3. **Depth와 무효값 정규화 (v2, Codex 지적 — 확정하지 않고 방어적으로 처리):**
   `camera.get_current_frame()["distance_to_image_plane"]`. NVIDIA Replicator 문서는 무관측 픽셀을 **0**으로
   설명하지만, 이 프로젝트의 `determinism_probe.py`는 `isfinite` 개수만 기록해 실제 값이 0/NaN/+inf 중 무엇인지
   구분한 적이 없다. **정규화 전에 `nan/posinf/neginf/0/negative/유한 양수` 각각의 개수와 유한 양수 최솟값·최댓값을
   `frame_diagnostics`로 남긴 뒤**, `~np.isfinite(depth) | (depth <= 0)`를 모두 `np.nan`으로 바꿔
   `SceneInput.depth_m` 계약(NaN=미관측)에 맞춘다. 배열이 비어 있거나 이 통계 자체가 안 나오면(annotator 미준비)
   "무관측 픽셀"이 아니라 별도의 **capture_failed**로 분류한다(§ "실패 분류" 참조).
4. **Intrinsics: Isaac 기본값을 쓰지 않고 `tools/scene_rig.py`의 `FOCAL_PX = 465.741156`(640×480, fx=fy, cx=320,
   cy=240)를 강제한다.** 계산(Codex 검증 완료): `focal_length=1.0`일 때 `horizontal_aperture ≈ 1.374154`,
   Isaac이 `maintain_square_pixels=True`로 `vertical_aperture ≈ 1.030615`를 자동 보정(공식 API 문서 근거,
   수직 FOV ≈ 0.9516 rad). **[v2] 설정값만 믿지 않고, 설정 후 실제 K를 읽어 fx·fy·cx·cy를 허용오차 안에서
   재확인하는 런타임 자체 점검을 추가한다.** 렌즈 왜곡 없음·주점 오프셋 0도 함께 확인한다. 같은 K를 재현해도
   렌더러·깊이 양자화·잡음이 CPU `scene_rig`와 다르므로, **기존 검출률이 그대로 유지된다는 보장은 아니다** — 이는
   실제 Isaac 실행으로만 확인 가능하고 이번 설계 문서로 닫히지 않는다.
5. **`base_from_optical`:** 고정 `RigidTransform(source_frame="camera_optical_frame", target_frame="base_link",
   rotation=quat_to_matrix(_LEVEL), translation_m=(0.75, 0.0, 0.50))`(`"camera_optical_frame"`/`"base_link"`는
   `tools/scene_rig.py:48`의 기존 관례 문자열). USD 자식 prim으로 심으면 이 상수 자체가 마운트 정의이자 로컬
   pose 설정값이 된다.
6. **`clock_domain`:** `"synthetic"`. **`source_provenance`:** `"synthetic"` (둘 다 `pocket_observation.py`의
   허용 집합에 있는 값).

## 검출기 호출과 좌표 변환

1. `prior`는 새 필수 인자 `--pallet-prior`(예: `config/pallet_prior_epal6.yaml`)로 받는다. `params`는
   `DetectorParams.derived_for(prior)`를 기본으로 쓴다 **[P2, v2]** — override를 허용할지, 최종 파라미터와
   prior의 해시를 `result.json`에 남길지 결정해야 한다(임의로 기본 `DetectorParams()`를 쓰면 EPAL6에 맞지 않는
   임계값이 적용된다).
2. `detect_pockets(scene_input, prior, params)` 호출.
3. **[P1 수정, v2] 포켓 전면 중점 → 팔레트 중심 변환.** `PocketObservation`의 `left`/`right` 중점은 팔레트
   **전면**(fitted vertical plane) 위에 있다(`pocket_detector.py:_build_observation`, `centre = plane.point + ...`),
   `PalletSite`(`pallet_mission.py:49`)가 뜻하는 팔레트 **중심**이 아니다. 올바른 변환은:

   ```
   front_mid = (left.center_m[:2] + right.center_m[:2]) / 2   # base_link, xy
   insertion_dir = (cos(insertion_yaw_rad), sin(insertion_yaw_rad))  # 전면에서 안쪽으로
   centre_estimate_base = front_mid + (pallet_depth_m / 2) * insertion_dir
   ```

   EPAL6은 `pallet_depth_m = 0.6`이므로 0.30m를 더해야 한다. 빠뜨리면 완벽한 검출도 위치오차 30cm로
   기록되고, 목표 삽입 깊이 0.36m가 명목상 약 0.06m로 줄어드는 심각한 결함이 된다(Codex 발견, 소스로 직접
   확인 완료 — `pocket_detector.py:544`). **[v3, Codex 재확인]** `plane.point`가 두 포켓의 평균이라는 뜻은
   아니다 — 각 포켓을 평면 위에서 좌우로 재구성한 뒤(`plane.left_axis` 방향 이동, 법선 방향으로는 이동하지
   않음) 그 둘의 평균이 `front_mid`다. 법선은 카메라를 향하고 `insertion_yaw_rad`는 그 반대이므로 위 식의
   부호도 맞다. 단, 이 복원은 **검출이 올바른 입구·형상을 찾았다는 조건에서만** 유효하고, 오검출까지 이
   공식으로 교정해주지는 않는다.
4. 캡처 시점의 `robot.get_world_pose()`(ground truth 허용 범위)로 `centre_estimate_base`와 `insertion_yaw_rad`를
   월드 좌표로 변환해 추정 `PalletSite(x, y, yaw)`를 만든다.

## 실행기 연결과 필요한 코드 변경 (`run_transport.py`, `pallet_mission.py`)

1. **[P1, v3 — Codex가 메모리 내 실제 실행으로 인터페이스 적합성 확인] `plan_transport`에 `target_pickup`과
   `start_rear`를 별도로 받는 선택 인자 두 개가 필요하다.**

   ```python
   def plan_transport(
       scenario,
       config=None,
       *,
       geometry=None,
       target_pickup: PalletSite | None = None,
       start_rear: Pose2D | None = None,
   ) -> MissionPlan:
   ```

   `target_pickup`은 `site_poses(...)`에만 쓰고, 팔레트 `Rectangle(scenario.pickup.x_m, ...)`(장애물)은 항상
   원본 `scenario.pickup`(정답) 그대로 만든다. `start_rear`는 관측 지점에 실제로 정지·정착한 후륜축 pose로
   최종 계획의 시작점을 대체한다 — **관측 후 계획을 안 바꾸면 여전히 `scenario.start_rear=(-2.34,0,0)`에서
   계획하게 되는 결함**이라 반드시 필요하다(Codex 지적). 둘 다 `None`이면 기존 동작과 완전히 같다.

   Codex가 파일을 바꾸지 않고 메모리 안에서 이 선택 인자를 임시로 추가해 실제 planner를 돌려본 결과: 인자를
   생략하면 5개 단계의 pose 배열이 기존과 완전히 동일했고, `target_pickup`을 1cm 바꾸면 approach/insert/extract
   목표가 갱신되며 transport 출발점까지 자연스럽게 전파되고(목적지는 유지) 팔레트 장애물 사각형은 원본 그대로
   유지됐다 — 단계 경계도 끊기지 않았다. **추정치가 approach부터 extract까지 전파되는 것은 버그가 아니라
   필요한 동작이다**(추출·운반 출발점만 정답 기준으로 남으면 단계 경계가 끊긴다). 별도의 대규모 request 객체나
   planner 재설계는 필요 없다는 결론이다. **이 변경 자체는 구현 단계에서 TDD로 별도 검증**(선택 인자 생략 시
   기존 회귀 그대로 통과 확인 포함)한다.
2. 카메라(USD 자식 prim)는 `configure_drives()` 직후, `world.reset()` 이전에 생성한다.
3. 관측 웨이포인트까지의 이동은 정답과 무관한 고정 목표를 향한 **별도의 계획**(위 §"관측 지점" 참조)이다.
   **목표 좌표에는 `scenario.pickup`을 쓰지 않지만, 이 이동의 계획·실행 충돌 지도에는 정답 팔레트 사각형을
   그대로 포함한다** — 관측 지점으로 가다 실제 팔레트에 부딪히면 안 되기 때문이다. `run_transport.py`의
   `phase == "approach"` 전용 팔레트 장애물 검사(469행)를 이 관측 이동 phase에도 동일하게 적용한다.
4. 도달·정착 후 §"어댑터"의 강제 렌더 → 캡처 → §"검출기 호출"의 변환을 거쳐 추정 `PalletSite`를 얻는다.
5. 유효하면 `target_pickup=추정치`, `start_rear=관측 지점에 실제로 정착한 후륜축 pose`로 최종 접근·삽입
   `plan_transport` 호출(장애물은 정답 유지, 위 1번). 무효/실패하면 아래 "실패 분류"에 따라 종료하며,
   **`target_pickup`을 생략해 정답 목표로 돌아가는 fallback은 금지한다.**
6. `result.json`에 새 필드 추가: `observation_waypoint`(고정 좌표), `pocket_observation`(base_link 원본 관측
   전체 직렬화, `status`/`reason` 포함), `frame_diagnostics`(§3의 깊이 무효값 통계, 캡처 프레임 수),
   `perception_pickup_estimate_m`(월드 좌표 추정), `perception_error`(추정값과 `scenario.pickup` 정답의
   위치·yaw 차이 — 오차 평가는 여기서만 정답을 쓴다).

## 실패 분류 (v2, Codex 지적 — 단일 문자열로 뭉개지 않는다)

| 상황 | `failure_reason` | 비고 |
|---|---|---|
| 카메라 워밍업 실패(RGBA 안 채워짐, annotator 미준비) | `perception_capture_failed` | 센서 문제, 인식 문제와 구분 |
| `detect_pockets` status가 `no_pallet`/`invalid` | `perception_{status}:{detector_reason}` | detector 고유 사유를 보존(`opening_width_mismatch`, `pocket_occluded:left` 등) |
| 검출 valid, 좌표 변환·`plan_transport` 자체가 실패 | 기존 계획 실패 사유 체계 그대로 | 인식 성공 후의 통상적 계획 실패와 동일하게 취급 |

## 남은 시험 계획 (구현 전 확정할 것)

1. 여러 seed에 대해 관측 웨이포인트 후보를 CPU `scene_rig` + `DetectorParams.derived_for(prior)`로 먼저
   스윕해 검출 성공률과 실패 사유 분포를 낸다 — 이번 세션에서는 seed 0 하나만 확인했다.
2. Isaac 실측: 무효 depth 센티널이 실제로 0/NaN/inf 중 무엇인지, 강제 렌더 후 K 재확인이 설정값과 일치하는지,
   자식 prim 카메라가 로봇 이동을 따라가는지 — 코드 구현과 함께 최소 시험으로 직접 확인한다.
3. `plan_transport`의 `target_pickup` 분리 매개변수는 별도 TDD 대상: 목표만 바뀌고 장애물 사각형은 그대로인
   것을 회귀 시험으로 고정한다.
