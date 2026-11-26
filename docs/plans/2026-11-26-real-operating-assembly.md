# 운영 REAL 조립 해제 — S10 선행 조건 확인으로 바꾸기

작성일: 2026-11-26 · 상태: **계획 8판 — 7판 승인(총괄 2026-11-26, #643) 뒤 총괄 결정·조건 반영 축소 개정(감시 토글 사전 목록 삭제, 벤치를 새 손작업만으로, C11 학교 수준), Codex 1 회 수렴 대상** · 관련: [PR 7 설계](../design/2026-11-25-real-sources-pr7.md) 2·5·6 절, [실차 조립 설계](../design/2026-11-19-real-assembly.md), [실차 안전 배선](../design/2026-11-19-real-safety-wiring.md) 1.3·5·7 절, [실차 시운전 계획](2026-11-11-sim-to-real-bringup.md) 3.4·3.5·5 절·R4, [7e 계획](2026-11-26-7e-depth-pocket-check.md) 6.5, [7a-3 계획](2026-11-26-7a3-sdk-arrival-stamp.md), [7r 계획](2026-11-26-7r-replay-run.md)

총괄 지시(2026-11-26, #642 머지 뒤): 운영 REAL 조립 해제의 계획부터 쓴다.
- `client_real:pr7_preconditions` 거부를 S10 선행 조건 확인으로 바꾼다.
- D5 는 `d5_pocket` 우회가 없으면 거부한다.
- 등록 행(`start_relocalizable`)·지도 묶음 정합이 코드로 막히는지 이 계획에서 확인한다.
- 7a-3 기입이 없으면 실 스캔은 진단 전용이라 층이 빈다. 그 상태로 REAL 조립이 열리면 안 된다. 선행 조건에 '스캔 상한 기입' 또는 '층 우회 명시'를 넣는다.
- 선행 조건마다 '우회 가능?' 칸을 둔다. 원칙은 멈추는 쪽만 우회 불가다.
- 벤치·사람 손작업은 항목마다 한 줄, 학교 수준이다.
- Codex 수렴과 독립 검토 뒤 승인을 요청한다.

## 0. 범위와 결론

- **결론(질문 셋)**
  1. **질문 1(선행 조건)**: 한 줄 거부를 판정 함수 하나로 바꾸는 것으로는 부족하다. 지금 코드에는 실차를 움직일 배관 자체가 없다. 없는 것은 다음 다섯이다.
     - 정상 보드로의 ARM·DRIVE 송신 권한(`motion_allowed`)
     - 잠금과 USB 신원을 갖춘 실 TTY 전송기
     - 실 스캔 원천 어댑터
     - D435i IMU 장치 래퍼
     - 실차 층의 LiDAR 보정 배선

     그래서 해제는 세 PR 이다(6 절). 마지막 PR 전에는 실차 조립이 계속 거부된다.
  2. **질문 2(지도 정합)**: 지금은 **코드로 막히지 않는다.** 실행기가 등록부를 읽지 않기 때문이다. 이 계획이 그 대조를 만든다(2.8). `start_relocalizable` 은 실차 행에도 `false` 로 저장되지만, 실차는 전역 시작 정합을 쓰지 않으므로 판정에 쓰지 않는다.
  3. **질문 3(스캔 상한 없을 때)**: **우회 불가로 권고한다**(C5). 층이 비면 장애물 정지가 없다. 또 REAL 에서는 지금 있는 `layer_timing` 우회도 막는다(2.5). 그 우회는 층의 지연 감시까지 끄기 때문이다.
- **하는 것**
  - 송신 권한과 실 전송기, 실 원천, 한 보정에서 나온 층·SLAM 배선을 만든다.
  - 선행 조건 판정과 기록 파일, 실차 전용 준비 판정(IMU·D5·층 시간)을 만든다.
  - `--client real` 실행 경로와 콘솔 표시를 만든다.
- **하지 않는 것**
  - 벤치 측정과 그 값의 기입은 하지 않는다. 판정은 그 결과를 읽는다.
  - 새 안전 감시, 실 D5(7e-2b 보류), 적재 운반은 하지 않는다. S10 은 들고 멈춤까지다.
  - 펌웨어 계약 변경(보정 식별자를 `STATE` 로 올리기)도 하지 않는다. 7 절의 남는 위험에 둔다.
- **코드 머지는 S10 승인이 아니다.** 모든 코드가 들어가도 지금 설정으로는 조립이 거부된다. 거부 사유는 C2–C14 의 기록·측정이 비어 있다는 것이다. S10 시작은 총괄 결정이다(PR 7 설계 2 절).

## 1. 지금 상태(확인한 사실, main 93c37dba)

- **조립과 실행기**
  - `assembly_refusal` 은 `Client.REAL` 이면 무조건 `client_real:pr7_preconditions` 를 낸다(`src/forklift_core/live/real_run.py` 663).
  - 실행기는 HIL 만 만든다(`tools/real_autonomy.py` 348 `real_host(..., client=Client.HIL)`). 원천은 합성 또는 재생이다(`real_run.py` 494).
  - 실행 시간 `--duration-s` 가 필수다(최대 3600 s). 기동 예산은 `HIL_STARTUP_MAX_S` 5 s 다(`tools/real_loop_transport.py` 45). 정리는 pty 기준이다(`real_autonomy.py` 439–474).
- **송신 권한**: `motion_allowed` 는 가짜 MCU(`fake_mcu`) 또는 검증된 HIL 보드(`hil_plant`)일 때만 참이다(`real_loop.py` 76–82). 정상 보드는 둘 다 아니다. 그래서 조립이 열려도 ARM·DRIVE 는 모두 `device_block` 으로 막힌다.
- **전송기**
  - 루프의 송신 계약(`gate`·`partial_frame`·`resync`)을 갖춘 것은 `LoopTtyTransport` 뿐이다. 여기에는 실행기 잠금과 USB 제품 문자열 읽기가 없다.
  - `ReconnectingLink`(`tools/real_mcu_link.py` 249–365)에는 잠금과 `link_identity` 가 있지만 루프 계약이 없다.
  - USB 제품 문자열이 없으면 `estop_waived` 는 거짓이다. 그러면 PCB V1.0 면제 이미지도 `drive_refusal` 의 `estop_unwired` 로 막힌다(`mcu_link/build_policy.py` 36–66).
- **보드 판정**(이미 있음): `connect_refusal` 은 실차가 시운전·HIL 보드에 붙는 것을 막고, `drive_refusal` 은 `outputs_locked`·비면제 `estop_unwired` 를 막는다.
- **층의 LiDAR 보정**
  - 실차 경로는 층의 장착·주기를 `config/real/obstacle_layer_real.yaml` 에서 읽는다. 지금은 자리표시자 장착(원점)과 `scan_period_s` 0.1 이다.
  - `lidar.yaml` 의 장착과 0.12 s 를 넣는 분기는 재생에만 있다(`real_run.py` 388–390, `replay_assembly.py` 166–173).
  - `_layer_sensors` 는 `measured: False` 를 고정한다(`real_run.py` 698–704).
- **스캔 원천**
  - `RunSources.scans` 계약(`scans`·`scan_source_events`·`sensor`)을 갖춘 실 어댑터가 없다. 합성과 재생만 있다.
  - `ScanFeed` 는 프레임 해석기다(`sensors/scan_feed.py`). 소켓, 수신 공백(`lost`), 층 변환(`to_layer_scan`), SLAM 변환(`from_feed`)의 묶음은 관측 실행 안에만 있다.
  - `ScanFeed` 는 합성 프레임에 표식을 붙이고, `to_layer_scan` 은 그 표식을 지킨다.
- **스캔 자격**: `lidar_cfg.frame_bound` 가 `first_beam_err_s`·`timing.basis`·`applies_to` 를 요구한다(`lidar_cfg.py` 174). 지금은 셋 다 null 이라 모든 스캔이 진단 전용이다. `mount.measured` 는 SLAM 의 `from_feed` 가 본다.
- **SLAM**
  - `RealSlamSource(mode="operating")` 은 자격 없는 스캔을 보내지 않는다(`real_slam_source.py` 503–512).
  - 브리지는 `hello` 의 `map_id` 만 확인한다. LiDAR 장착 변환은 ROS 파라미터 `laser_xyz_yaw`(기본 0)로 따로 받는다(`ros2/src/forklift_ros/forklift_ros/real_slam_bridge.py` 87–96, `localization/real_bridge_core.py` 76–86).
  - HIL 의 SLAM 입력 `from_layer` 는 실 스캔의 오차 구간과 생산 세대를 지운다(`localization/slam_real_scan.py` 74).
- **IMU**
  - `ImuSource(operating=True)` 는 `imu.yaml` 의 `measured: false` 면 만들어지지 않는다(`imu_source.py` 145–147). 장치를 여는 쪽(`tools/imu_record.py` `RealSenseSource`)에는 분리 때 `lost()` 를 부르는 길과 `fits_watch` 관문이 없다.
  - 루프 생성자는 `imu_policy: required` 인데 원천이 없으면 거부한다(`real_loop.py` 309). 감시와 원천이 짝이 아니어도 거부한다(311). 조립도 `imu_required` 로 거부한다(`real_run.py` 684).
  - 제출 전 IMU 자격 판정은 없다(`real_autonomy.py` 634–693). 무장 관문(`_imu_arm_gate`)이 처음 보고, 무장 뒤에는 틱과 송신 관문이 감시한다(`real_loop.py` 1677–1775, 2544).
- **우회**
  - `real_loop.BYPASSABLE` 은 준비 판정 6 개에 `imu`·`watch_current`·`watch_wheel`·`start_pose`·`d5_pocket`·`f1` 을 더한 것이다(`real_loop.py` 186).
  - `layer_timing` 우회는 다음을 모두 건너뛴다(`real_loop.py` 458, `real_layer.py` 481): 준비 판정의 `snapshot:scan_stale`, `RealLayer.limit()` 의 나이 검사, `ok()` 의 신선도, watchdog 마감, `layer_latency_exceeded`.
  - `imu` 우회는 시작 수평 판정과 무장 중 기울기·표본 나이·공백 비상정지를 함께 끈다(`real_loop.py` 1595).
- **D5**
  - 실 깊이 원천은 없다(7e-2a, 미승인). `no_depth_no_bypass` 는 `d5_pocket` 의 목록 소속만 본다.
  - 세션에서 우회하지 않으면 접근 끝 첫 정지에서 프레임 대기 시한 뒤 끝난다(`mission/core.py` 1555).
  - 우회 중에는 등록 자세 주위 사각형을 통째로 FREE 로 덮는다(`real_layer.py` 75). 지령 상한은 후륜축 0.06 m/s(`DECLARED_SPEED_MPS`)다.
  - `operator_start` 확인은 콘솔에 '시작 확인' 버튼 하나뿐이다(`console/web/index.html` 40, `realpanel_dom.js` 127–130). '빈 포크·포켓 밖·승강 내림' 세 항목은 설계 문서(`real_assembly.py` 문서 문자열)에만 있다. 이 확인도 우회 목록에 있다.
- **지도 등록**
  - 실차 행(`tools/slam_bundle_registry.py add-site`)에는 `site_record_sha256`·`map_id`·시작 자세·`survey_report`(경로·sha256)·`start_relocalizable: false` 가 있다.
  - `lidar.yaml`·URDF 해시와 차량 보정 식별자는 **보고서 JSON 안**에 있고 행에는 없다(166–224, 328–344).
  - `add-site` 는 S10-A `map_id`·`start_rear` 를 요구한다. 번들 자체는 S10-A `map_id` 와 대조된다(`real_run.py` 137).
- **설정의 잠정값**: 다음이 모두 `measured: false` 또는 시뮬 잠정값이다.
  - `vehicle.yaml`: G 80 추정, `wheel_radius: null`, 조향표, 한계
  - `obstacle_layer.yaml`: 지연 예산
  - `obstacle_layer_real.yaml`: `latency_s` 0.30, `decel_mps2` 0.5
  - `safety_watch.yaml`: 전류·바퀴, `imu.level_ref: [0,0,1]`

  펌웨어 보정 식별자(`FW_VEHICLE_CALIBRATION_ID`)는 헤더로만 만들어지고 `STATE` 로 올라오지 않는다(`tools/real_vehicle_header.py` 54–64).
- **링크 신선도**: 실행기 호스트 설정은 `gap_clock="host"`, `state_max_age_s` 0.05 다(`real_run.py` 44).
- **승강**: S10 은 `lift_target_m` 0.20 까지 든다(`autonomy.yaml` 4). 승강 위치 측정 안(3.5 A/B)은 미정이다. 결정 전에는 빈 포크·한 바퀴 미만만 시험한다. 승강 자잠김(R4)도 미확인이다.

## 2. 설계

### 2.1 송신 권한과 실 전송기

- **실 전송기 `RealTtyTransport`**(`tools/real_loop_transport.py`): 루프 계약(`gate`·`partial_frame`·`resync`)은 `LoopTtyTransport` 와 같고, 다른 점은 셋이다.
  - 같은 fd 를 설정 전에 잠근다(`open_raw(..., before_config=lock_fd)`, 실행기 잠금 #618). 잠겨 있으면 `DeviceBusy` 다.
  - 그 fd 에서 USB 제품 문자열을 읽어 `host.link_identity()` 에 넘긴다. 면제 이미지면 경고를 기록한다.
  - 끊기면 다시 붙지 않는다. `DeviceLost` 로 송신 권한을 버리고 실행을 끝낸다.
- **권한 `motion_allowed`**: 세 번째 조건 `real_operating is True` 를 더한다. 이 값은 실 전송기만 가지며, 다음을 모두 만족할 때만 참이다.
  - 잠금 보유
  - 첫 `STATE` 의 `connect_refusal` 없음
  - 조립 때 선행 조건 판정 빈 목록(그때 받은 토큰)
  - 장치 상실 없음

  송신마다 지금 있는 허가·신선도·빌드 검사(`drive_refusal`)는 그대로 돈다.
- **끝낼 때**: 정상 종료·신호에서는 DISARM 확인 경로가 끝날 때까지 fd 와 잠금을 쥔다. 장치 상실이면 권한만 버린다.
- **잠금 인계**: 텔레옵 다리·관측 실행도 같은 장치를 잠근다. 자율을 띄우려면 그쪽을 먼저 끝낸다. 실행기는 잠금 실패를 사유와 함께 낸다. 자율 콘솔은 텔레옵을 하지 않는다(지금 그대로).

### 2.2 실 원천

- **스캔 `RealScanSource`**(`live/real_sources.py`, 새로)
  - 스캔 컨테이너의 소켓을 읽어 `ScanFeed` 로 해석한다.
  - 수신 공백(`SCAN_GAP_NS`)과 소켓 오류는 `("lost", sensor)` 이다. 소켓 오류는 실행을 끝낸다(`scan_socket_error`).
  - 공백이 한 번의 읽기 안에서 끝나면 그 읽기에서 공백 앞의 스캔은 버리고 센다(`before_gap_dropped`). 그래서 층은 `lost` → `connected` 뒤 스캔만 보고, SLAM 은 공백의 세대 뒤에 옛 세대를 듣지 않는다.
  - 각 프레임은 먼저 생산 세대를 SLAM 에 알린다. 그다음 `frame_bound` 로 자격을 본다(7a-3 규칙).
  - 자격 있는 프레임은 층에 `to_layer_scan` 으로 넘긴다. SLAM 에는 **층이 받아들인 스캔만** `from_feed` 로 넣는다(실행의 SLAM 분기 `slam_feed` = `offer_for`, 층 거부면 둘 다 받지 않는다). 오차 구간과 세대를 보존하고, HIL 의 `from_layer` 는 쓰지 않는다.
  - 합성 표식이 붙은 프레임은 판정과 상관없이 실행을 끝낸다(`scan_synthetic`, `ScanFeed.synthetic_frames` 로 센다).
  - 층과 스캔 원천은 `prepare_run` 이 한 번 읽은 같은 `LidarConfig` 를 쓴다(`RunPrep.lidar`).
- **IMU 장치 래퍼**: D435i 를 열고 실제 열린 프로필의 `fits_watch` 를 확인한다(열린 가속도 주기를 말하지 않는 프로필은 맞지 않음). 분리되면 `imu.lost()` 를 부르고(`start(..., on_lost=)`), 닫을 때 `stop()` 한다(`closers`, 실패한 `stop` 은 남겨 다시 시도). 원천 판정은 `ImuSource(operating=True)` 다. 시작은 C7 의 세 조건(`imu.yaml` 측정·지연 상한, 감시의 `level_ref` 측정)을 prep 이 해시한 바이트로 본 뒤에만 한다.
- **SLAM**: `RealSlamSource(mode="operating")`, `client = Client.REAL` 이다. HIL 확인 대신 실 전송기의 부팅 확인을 쓴다.
- **깊이**: 없다(2.6).
- **한 호스트**: 스캔 컨테이너·SLAM 브리지·실행기는 같은 Jetson 에서 돈다(같은 `CLOCK_MONOTONIC`). 변환의 `after_receipt` 는 프레임 안의 어댑터 수신 시각과 비교하므로, 다른 호스트여도 거부가 보장되지는 않는다. 그래서 같은 시계 도메인은 런처의 운용 조건이고 C14 기록에 들어간다.
- **실행 CLI**: `tools/real_autonomy.py --client real --device <USB CDC> --scan-socket <스캔> --slam-bridge <SLAM> --site-record <조사 기록>`.
  - pty·재생·합성 입력은 거부한다(`SyntheticRefused`, `synthetic.py` 42).
  - **실행 시간은 유한하게 둔다**(`--duration-s`, 최대 3600 s, 지금 `RealRun.run()` 그대로). 무기한 실행과 콘솔 종료를 새로 다루지 않아 범위가 줄어든다.
    - 남은 시간이 `mission_time_budget_s`(새 설정, 잠정 300 s)보다 적으면 제출을 거부한다(`run_time_short`). 여유를 두려는 검사이고 보장은 아니다. 실행 시한은 벽시계로 흐르고 미션 작업 시계는 대기·일시정지를 빼기 때문이다(Codex 3 회차 P2).
    - 그래서 삽입 진입과 재개 직전에도 남은 시간(잠정 120 s)을 다시 본다. 모자라면 그 단계로 들어가지 않고 대기 프레임이다.
    - 시한이 실제로 닿으면 지금의 정지와 현장 복구 절차가 우선이다.
    - 구현(U3b-1): `autonomy.yaml` `mission_time_budget_s`·`insert_time_min_s`·`real_startup_max_s`. 실행 시한은 `RealRun.run` 이 루프에 넘기는 하나다. 삽입 전환 직전에 모자라면 `arrive()` 를 하지 않고 대기 프레임(`run_time_short:insert`), 삽입 중 재개(0 프레임 뒤 첫 비영 프레임)는 비영 프레임이 실제로 쓰일 때까지 `run_time_short:resume` 으로 기다린다. 시한과 종료 신호는 모든 비 DISARM 프레임의 첫 바이트·이어쓰기 관문에서도 본다.
  - 기동 시한은 새 설정값(`autonomy.yaml` `real_startup_max_s`, 잠정 30 s)이다.
  - **수신량**: `ScanFeed.poll()` 은 큐를 비울 때까지 읽는다. REAL 어댑터는 한 틱에 읽는 원시 데이터그램 수를 제한한다(잠정 8, `read_capped` 로 센다). 남은 것은 버리지 않고 다음 틱에 읽는다(밀린 수신은 공백이 아니다). 한 틱에 층으로 가는 스캔은 마지막 4 개이고 나머지는 센다(`cut`). 원천 사건(`lost`·세대 변경)은 같은 틱의 송신 판단 전에 루프에 전한다.

### 2.3 한 보정에서 층과 SLAM

- `prepare_run` 은 REAL 에서도 재생과 같은 함수(`replay_layer_cfg`)로 `lidar.yaml` 의 장착과 `scan_period_s` 를 층 설정에 넣는다. `_layer_sensors` 의 `measured` 는 `lidar.yaml` `mount.measured` 다. 그래서 층의 투영·정지 지연·오도메트리 여유가 같은 보정을 쓴다.
- 실행기는 `base_from_laser(lidar.mount, rear_offset_m)` 를 `hello` 에 실어 보낸다. 브리지는 자기 `laser_xyz_yaw` 와 다르면(1 mm·0.1° 초과) `fault` 로 거부한다.
- 같으면 브리지의 `ready` 가 대조한 값의 해시(`laser_checked`)를 돌려준다. REAL 의 SLAM 원천은 그 해시가 자기 값과 같은 `ready` 뒤의 응답만 받는다. 새 칸을 모르는 구버전 브리지는 `laser_checked` 가 없으니, 그 세션의 `processed` 를 하나도 받지 않는다(Codex 2 회차 P1-1).
  - 브리지 코드(`real_bridge_core`)와 `hello`·`ready` 계약을 바꾼다. 브리지 시험이 함께 간다. 새 실행기 + 구버전 브리지 조합 거부와, HIL·관측 도구 조합의 회귀를 본다.
  - 런처는 같은 `lidar.yaml` 에서 그 파라미터를 계산해 띄운다(`tools/real_bridge_laser.py`, 실행기와 같은 `real_run.bridge_laser_xyz_yaw`).
  - 구현(U2b): `hello.base_from_laser` 와 `state.laser_checked`(hello 값 그대로의 sha256)는 선택 칸이다. 칸 없는 hello(HIL·관측)는 대조도 `laser_checked` 도 없고, 그 실행기도 요구하지 않는다. 운영 실행기는 시작과 씨앗의 모든 `ready` 에 자기 값의 digest 를 요구하고, 없거나 다르면 실행을 끝낸다(`slam_laser_unchecked`).

### 2.4 선행 조건 판정 `operating_preconditions`

- 위치는 `live/real_preconditions.py`(새로)다. 조립과 콘솔이 같은 함수를 쓴다. 순수 함수라 사유 전체 목록을 단위 시험으로 본다.
- **입력**
  - 설정 파일을 직접 읽는다. 같은 값을 기록 파일에 다시 적지 않는다.
  - 첫 `STATE` 의 보드 사실과 USB 제품 문자열.
  - 사람이 확인하는 항목의 기록 `config/real/s10_preconditions.yaml`.
- **기록 형식**(`schema: s10_preconditions/1`): 항목마다 다음을 둔다.
  - `status`: `pass` | `approved_limited` | `null`
  - `evidence`: 저장소 안 문서 경로, 필수
  - `date`
  - `decision_id`: `approved_limited` 이면 필수
  - `binds`: 그 확인이 성립하는 대상

  `binds` 의 대상과 조립 때 대조하는 것은 다음과 같다. 어긋나면 그 항목이 무효다(Codex 2 회차 P1-4·P1-5).
  - 보드: 같은 fd 의 USB 일련번호(96-bit UID)
  - 이미지: USB 제품 문자열(빌드 종류·`git describe`·기록·면제 표식). 기록에는 그 빌드가 만든 차량 헤더의 보정 식별자(`firmware_vehicle_calibration_id`)를 함께 둔다. 이것이 지금 `vehicle.yaml` 보정 식별자와 같아야 한다.
    - 제품 문자열의 `git describe` 에 `-dirty` 가 있으면 거부한다.
    - 운영 빌드에서 `FW_VEHICLE_HEADER` 덮어쓰기를 금지한다(펌웨어 CMake 의 빌드 종류 검사, 한 줄). 그러면 헤더는 그 커밋의 `vehicle.yaml` 에서만 만들어진다. 지금 펌웨어 교차 시험(`tests/integration/test_firmware_cross.py` 298·499)이 정상 빌드에 덮어쓰기를 넘기므로, 그 fixture 가 헤더 생성 경로를 타도록 `vehicle.yaml`·생성 도구를 복사본에 넣고, 덮어쓰기 거부 시험을 따로 둔다(Codex 4 회차 P2).
    - 굽은 이미지를 판독해 빌드 산출물과 sha256 을 대조한 결과도 기록에 둔다(벤치 5 분).
    - **운영 관문**: 최종 판독 뒤 다시 굽으면 S10 승인 기록(C1·C2·C3·C11·C12)이 무효다. 다시 판독·확인한다. 같은 커밋·같은 표식으로 다시 구우면 실행기는 구별할 입력이 없다. 그래서 이것은 사람이 지키는 관문이다(7 절, Codex 3 회차 P1-2).
  - 실제로 읽는 정지·시간 설정 파일 sha256: `obstacle_layer_real.yaml`, `obstacle_layer.yaml`(지연 예산), 오도메트리 나이 표(`odometry_age` 가 가리키는 파일), `vehicle.yaml`, `safety_watch.yaml`
  - LiDAR(C5): `scan_load` 는 `lidar.yaml` sha256 과 Jetson 운영 구성(스캔 컨테이너 이미지 식별자)에 묶는다. `scan_coverage` 는 장착값·가림 마스크·승인 통로에 묶는다.
  - 승강: `autonomy.yaml` 의 `lift_target_m`·`lift_rate_mps`, 승인 하중(시험 팔레트 식별과 질량). `lift_rate_mps` 는 보드의 승강 한계 이하여야 한다. 지금 셋이 서로 다르다: `autonomy.yaml` 0.06 m/s, 보드 `lift_rate_max` 0.02 m/s(`board_config.c` 64, S4 실측 대기), HIL 상한 0.10 m/s. 판정은 `lift_rate_mps ≤` 보드 값을 본다. 코어는 승강 시작 약 4.5 s 뒤부터 10 s 의 도달 기한을 둔다(`mission/core.py` 745·6593). 0.01 m/s 면 0.20 m 의 5 mm 안까지 19.5 s 라 기한을 넘는다(Codex 4 회차 P2). 8판에서 도달 시간은 선행 조건에서 뺐다. 런타임 코어 기한이 강제하고, 넘으면 미션이 실패해 포크가 그 자리에 서며 그때를 C11 의 하중 유지가 덮는다(3 절 C11 주, 총괄 조건).
  - 시험 조건: 시험한 속도. 승인 속도가 그보다 크면 무효다.
  - 승인 값: `approved_max_speed_mps`, 배치·경로 한도

  이렇게 하면 실측 뒤 설정·펌웨어·보드를 바꾸면 그 확인이 무효가 된다.
- **승인 값의 강제**
  - `s10a.max_speed_mps ≤ approved_max_speed_mps` 를 코드로 본다.
  - 사례 A 는 승인된 배치와 경로로 강제한다. 새 사례 분류기는 만들지 않는다(Codex 2 회차 P1-3, 3 회차 P1-1).
    - **배치**: 제출 때 등록 팔레트 자세가 S10-A `check_pallet_pose` 에서 0.10 m·5° 안이어야 한다.
    - **시작점**: 지금 자세가 `start_rear` 에서 0.10 m·5° 안이어야 한다. 제출 때와 **무장 전 검사 때 둘 다** 본다. 계획 대기 중 위치 추정이 갱신될 수 있기 때문이다(Codex 4 회차 P1-2). 이 검사는 `start_pose` 세션 우회와 **무관하게** REAL 에서 늘 돈다. 지금의 `_off_start()` 는 그 우회로 건너뛰므로(`real_loop.py` 978–984, 1861), REAL 에서는 그 함수에 같은 규칙을 넣는다.
    - **경로**: 실제로 실행하는 단계(`approach`·`insert`)만 본다. 승강 뒤에만 쓰는 `extract` 는 늘 후진으로 계획되지만 S10 은 실행하지 않으므로 대상이 아니다(`pallet_mission.py` 2029).
      - 그 두 단계에 후진 구간이 있으면 채택을 거부한다.
      - 승인 통로는 승인 배치(`start_rear`, `check_pallet_pose`)에서 정한 선분 하나다. 시작점에서 **삽입 끝**(접근 직선 끝 + 접근 간격 + 목표 삽입 깊이, `pallet_mission.py` 399·2014)까지 잇고, 좌우 0.10 m 다. `approach`·`insert` 의 자세가 하나라도 그 밖이면 거부한다(Codex 4 회차 P1-1). 방위 변화 한도만으로는 15° 안의 S자 측면 우회를 못 막는다.
  - C4 의 `approved_limited` 는 같은 값(`approved_max_speed_mps`, 배치·경로 한도)으로 실행기에 전해진다.
  - 구현(U3b-1, `live/real_case_a.py`): 배치·시작점·실행 시간은 제출마다 판정한다 — `RealSession.tick_start` 가 `link.begin` 뒤 이번 묶음의 첫 제출을 읽어 그 대상으로 `submit_refusal` 을 정한다(`commands.py` 무변경). 경로는 채택 훅(`live_mission_start` 의 `started`)에서 보고, 어기면 ARM 전에 미션이 끝난다. 시작점은 운영판에서 `_off_start` 가 `start_pose` 우회를 보지 않아, 제출·무장 준비·첫 비영 DRIVE 와 모든 ARM 송신(재시도 포함)에서 본다.
- **출력**: `precondition:<항목>:<이유>` 목록이다.
  - 우회 불가 항목이 하나라도 있으면 조립을 거부한다. 콘솔은 뜨지 않으므로 종료 코드 2, 표준 출력, 기록의 끝 행에 사유를 남긴다.
  - 우회 가능 항목(C7)은 조립을 막지 않는다. 런타임 준비 판정에서 막히고 세션 우회로 연다.
- **마지막 PR 전 잠금**: 6 절의 U1·U2 동안 판정은 `precondition:unlock_incomplete` 를 늘 낸다. U3 이 그것을 지운다. 중간 머지에서 송신이 열리지 않게 한다.

### 2.5 층 시간 — REAL 에서 우회 불가

- REAL 에서 `layer_timing` 을 `NOT_BYPASSABLE` 로 옮긴다. 콘솔 토글 요청은 다른 우회 불가 항목과 같은 거부 사유 `not_bypassable` 로 거부된다(콘솔 문구가 이미 있음, U3a). HIL 은 지금 그대로다.
- **스캔 생산 세대가 바뀌거나 자격을 잃으면 래치한다.** 장치·이미지·USB 가 바뀌어 `applies_to` 가 어긋나는 경우, 어댑터가 재시작해 세대가 바뀌는 경우다.
  - 수신 공백보다 짧게 재시작하면 새 세대 스캔이 watchdog 마감 안에 들어온다. 그래서 공백에만 기대지 않는다(Codex 2 회차 P1-2).
  - REAL 루프는 그 틱에 `scan_generation_changed` 로 래치한다(정지·DISARM, 자동 재개 없음). 층의 설치 결과와 대기 작업을 버리고, 이전 작업이 늦게 끝나도 설치하지 않는다.
  - 다시 시작은 운용자가 한다. 래치 해제 뒤 새 세대의 층 설치와 이번 세대의 `processed` 가 있어야 무장할 수 있다(지금 규칙).
  - 학교 수준에서 가장 단순한 선택이고, 주행 중 세대 전환을 이어 붙이는 일은 하지 않는다.
  - `real_status` 에 최근 스캔의 거부 사유(`frame_bound` 의 이유)를 싣는다.
  - 구현(U3a): 경계는 스캔을 하나라도 받은 뒤의 입력 세대 단절 전부다 — 피드 세대가 어떤 이유로든 오름(생산자 변경·id 역행·시계 재확립·수신 공백·소켓 오류), `clock_jump` 거부, 자격 상실(같은 생산 세대 안에서도, 이때 피드 세대를 새로 열어 SLAM 이 대기·처리 중인 것을 버림). 운영 루프는 그 틱의 층 설치보다 먼저 원천 사건을 처리하고, 층의 격자·설치 결과·계획 격자·대기 작업(작업자에 아직 시작 안 한 요청 포함)·`_fresh_ns`·진행 중 재위치 시도를 버린다. 경계 이전에 시작한 증거의 늦은 `processed` 응답은 신선도를 채우지 못한다(`before_scan_boundary`).

### 2.6 D5 — 우회 필수와 팔레트별 확인

- REAL 에서 `sources.depth is None` 이면 **제출**을 거부한다: `d5_pocket:no_depth_source`. 세션 우회에 `d5_pocket` 이 있을 때만 받는다. HIL 은 그대로다.
- **팔레트별 현장 확인**: 버튼을 새로 두지 않고 지금 `OperatorStart` 확인을 넓힌다(`real_assembly.py` 441, Codex 2 회차 P1-6).
  - REAL 에서 깊이 원천이 없으면 확인에 대상 팔레트와 그 등록 revision 이 묶인다. 그 확인은 세션·lease·재위치 세대·revision 이 모두 같을 때만 유효하다. 승인된 제출 한 번에 소비된다(지금의 무효화 경로를 그대로 쓴다).
  - 화면 문구는 여섯이다.
    1. 포크가 포켓 밖에 있음
    2. 포켓 둘이 비어 있음
    3. 팔레트 윗면과 등록 사각형 안에 다른 물체 없음
    4. `lift_target_m` + 팔레트 높이까지 위 공간 비어 있음
    5. 빈 포크·승강 내림
  - 여섯째 문구는 하중이다: 'C11 에서 승인한 시험 팔레트 <식별>, 질량 <값> kg 이하'. 하중 센서가 없으니 사람이 확인한다(Codex 3 회차 P1-3).
  - D5 우회가 등록 사각형을 FREE 로 덮으므로, 사람 눈이 그 자리를 대신한다.
- REAL 에서 깊이 원천이 없으면 `operator_start` 우회를 거부한다(`not_bypassable`). 그래서 이 확인은 **우회할 수 없다.**
- 구현(U3b-2): 운영판이고 깊이 원천이 없을 때만 적용한다(HIL·재생·깊이 원천이 있는 실행은 그대로).
  - `operator_start` 요청은 콘솔이 보이는 `pallet_id`·`targets_revision`·`register_serial`(이 실행의 등록마다 하나씩 오르는 번호, 등록 응답과 `real_status` 에 있다)을 싣는다. 서버가 최신 revision 으로 대신 채우지 않는다. 지금 등록(revision 이나 등록 번호)과 다르면 `targets_changed`, 운전자 등록이 없으면 `pallet_not_registered`, 빠지면 `operator_start_needs_registration`.
  - 확인은 우회 영역 전체(`RealHooks.bypass_region`: 등록 자세 둘레로 팔레트 + 띠, 미션의 우회 검사와 같은 기하 하나)를 싣고, 응답과 기록(`operator_start`)에 남긴다.
  - 제출 판정: `d5_pocket` 우회가 없으면 `d5_pocket:no_depth_source`, 이 제출의 (팔레트, revision)·lease·재위치 세대와 같은 확인이 없으면 `operator_start:unconfirmed`. 이 실행에서는 한 묶음의 제출을 하나씩 따로 판정하고 하나라도 거부면 묶음 전체를 거부한다(깊이 원천이 있는 운영 실행은 묶음의 첫 제출만, 지금처럼). 승인된 제출이 확인을 소비하며 미션 id 와 함께 기록한다(`operator_start_used`).
  - 팔레트를 다시 등록하면(같은 자세·같은 revision 이라도) 확인은 무효다. 확인을 쓴 뒤 그 미션의 첫 ARM 이 실제로 쓰이기 전에(대기열에 있거나 EAGAIN 으로 못 나간 동안 포함) 차가 움직이면(그 사이 STATE 하나라도 오도메트리 1 cm·0.5°, 승강이 포트의 정지 허용치를 넘거나 오도메트리 연속이 끊기면 — 첫 ARM 까지 래치) 미션이 끝나고 ARM 은 쓰기 관문에서 거부된다. 사유는 `operator_start_moved` 이고, 먼저 걸리는 기존 비상 경로가 있으면 그 사유다(예: ARM 대기 중 오도메트리 단절은 `real:odometry_break`). 어느 쪽이든 그 미션의 ARM 은 쓰이지 않는다.
  - 미션이 만드는 우회 검사의 영역은 그 미션이 쓴 확인의 영역이어야 한다(중심·크기 같고 축 5° 안). 없으면 `d5_confirmation_missing`, 다르면 `d5_region_mismatch` 로 채택 중 끝난다. 맞으면 면제되는 영역은 확인한 사각형 그대로다(축 허용각만큼 돌린 사각형이 아니다).
  - `real_status.operator_start` 는 지금 등록 revision·재위치 세대와 같은 확인일 때만 참이고, `operator_start_binding` 이 묶인 내용을 보인다.
  - 화면 문구 여섯은 U3c(콘솔)에서 넣는다. 그 전까지 지금 콘솔의 빈 `operator_start` 는 운영판에서 `operator_start_needs_registration` 으로 거부된다.
- 우회 중 지령 상한은 0.06 m/s 그대로다. 실측 과속 감시는 따로 돈다.
- 시운전 계획 5 절의 '포켓 육안 확인' 한 줄은 이 확인으로 실제 화면과 기록에 들어간다.

### 2.7 IMU — 미검증은 준비 판정으로

- **자격**: 셋을 모두 만족해야 운영 IMU 다.
  - `imu.yaml` 측정됨(지연 상한)
  - 실제 열린 프로필의 `fits_watch`
  - `safety_watch.yaml` `imu.level_ref` 측정됨
- **자격이 없으면** REAL 조립은 IMU 감시를 만들되 원천 없이 둔다. 루프 생성자의 두 검사(309·311)를 REAL 에서만 바꾼다. 감시는 `imu:unqualified` 를 낸다.
- `auto_ready_refusal` 이 제출과 무장 두 단계에서 `imu:unqualified` 로 막는다. 지금 무장 관문만 보는 것을 넓힌다.
- 열 때는 기존 `imu` 세션 우회를 쓴다. 다른 우회와 같이 세션 한정·기록·콘솔 '우회 중' 표시이고, 사전 승인 목록은 두지 않는다(총괄 결정 2026-11-26, 8 절 2). `imu` 우회 중이면 IMU 가 미검증이어도 제출·무장이 열린다. REAL 에서 `autonomy.yaml imu: none` 은 미검증과 같게 다룬다. 설정으로 감시를 없애지 못한다.
- `watch_current`·`watch_wheel` 도 S10 에서 같은 방식으로 세션 우회할 수 있다. 펌웨어 과전류·STOP·DISARM·링크 단절은 그대로 우회 불가다.
- **우회의 범위**: 지금 `imu` 우회는 시작 수평 판정과 무장 중 기울기 비상정지를 함께 끈다.
  - 사용자 원칙의 예('IMU 체크를 잠깐 내려 두고 구동')는 이 토글을 가리킨다. 그래서 그대로 쓴다.
  - 콘솔은 '우회 중: IMU(기울기 감시 꺼짐)'으로 보인다.
  - 주행 중 감시가 끊기는 경우(분리)는 우회가 아니다. 지금 감시 규칙(`watch_imu_*`)대로 비상정지다.

### 2.8 지도 묶음 정합

- 조립은 `lookup_site(registry, sha256(site_record), encoder_model)` 로 행을 찾는다. `encoder_model` 은 자유 문자열이 아니라, 실행기가 고정으로 만드는 `RealOdometry` 에 대응하는 단일 식별자 상수다(`real_run.py` 753). 조사 보고서의 값과 같아야 한다.
- 행 찾기와 번들 경로 선택은 `prepare_run()` 의 지도 로드보다 앞이다(`real_run.py` 368).
- 행의 `survey_report.path` 파일 sha256 을 행의 값과 먼저 대조한다. 그다음 보고서의 다음 값을 지금 파일과 대조한다.
  - `config_sha256.lidar`·`config_sha256.chassis`: 지금 `lidar.yaml`·차체 URDF
  - `vehicle_calibration_id`: 지금 차량 보정 식별자(`load_real_vehicle(...).calibration_id`)
  - `map_id`
- 행의 시작 자세(`base_link`)를 후륜축으로 바꿔 S10-A `start_rear` 와 비교한다(0.10 m·5°). 번들 폴더는 행의 `SLAM_BUNDLE_ROOT / folder` 하나다(따로 받는 인자 없음).
- 하나라도 다르면 `precondition:site_bundle:<무엇>` 이고 **우회 불가**다. 고치는 길은 다시 조사·등록하는 것이다.
- 구현(U2b, `live/real_site.py`): `encoder_model` 상수는 `live/real_site.REAL_ENCODER_MODEL`(`"real_wheel_odometry/1"`)이다(시뮬레이션 미션 코어가 import 하는 `real_vehicle` 은 건드리지 않음). 운영 실행의 경로는 `RunPaths.site_record`·`registry`·`bundle_root` 이고, 손으로 준 `map_bundle` 은 거부한다(`map_bundle_by_hand`). 행의 `folder` 는 경로 구분자 없는 이름 하나, 보고서는 `bundle_root` 안에서만 읽는다. 대조에 쓴 site record·registry·보고서의 해시는 `prep.hashes` 에 남는다.
- **순서 주의**: 보고서가 `lidar.yaml`·`vehicle.yaml` 해시를 고정한다. 그래서 순서는 보정·장착 확정(S2·S7·S8·7a-3 기입) → 조사 주행 → `map_id`·시작점 기입 → 등록이다(4 절).

### 2.9 콘솔

- 조립을 통과하면 `real_status.preconditions` 에 통과 요약과 런타임 항목을 싣는다. 런타임 항목은 C7 IMU, C8 팔레트 확인, 최근 스캔 거부 사유다.
- 실차 패널에 줄을 둔다. 통과는 회색, 막힘은 빨강, 우회 중은 주황 '우회 중: …'이다.
- 팔레트 확인 버튼(2.6)을 두고, 문구는 `profile.js` 거부 문구 표에 더한다.

### 2.10 기록 형식 — C11 승강(학교 수준)

```yaml
lift:
  status: pass            # 처짐이 한도 안이면 pass
  evidence: docs/validation/<날짜>-s10-lift-hold.md   # 사진 한 장·줄자 값·방법
  date: 2026-12-..
  binds:
    board_serial: "<USB 일련번호>"
    lift_target_m: 0.20   # autonomy.yaml 과 같아야 함
    lift_rate_mps: 0.02   # autonomy.yaml 과 같아야 함, 보드 승강 한계 이하
    test_pallet: "P-S10"  # S10-A pallet.id
    pallet_mass_kg: 4.5   # 저울 또는 표기값
    lift_mechanism: "<승강 기구 설명·사진 경로>"  # 기구를 바꾸면 다시 잰다(사람 확인)
  origin:                 # 원점·환산 확인(기존 승강 실측을 재사용해도 됨)
    state_lift_m_at_floor: 0.000   # 포크 바닥일 때 STATE lift_m
    tape_cm_at_target: 20.0        # 목표 높이에서 줄자로 잰 포크 높이
    state_lift_m_at_target: 0.200  # 같은 때 STATE lift_m
  hold:                   # 받침 없이 주 전원 차단 60 s 뒤 처짐(줄자), 높이마다 한 줄
    - {raised_cm: 8.0, sag_cm_after_60s: 0.0, worst: true,     # 무전원 유지가 가장 불리한 높이
       worst_reason: "<기구 근거: 예) 나사 리드각이 가장 큰 구간·체인 처짐이 큰 높이>"}
    - {raised_cm: 20.0, sag_cm_after_60s: 0.0}   # 목표 높이(lift_target_m)
  sag_limit_cm: 1.0       # 한도: 총괄이 정한 값
  sag_limit_decision_id: "<총괄 결정>"
  reach_s: null           # 첫 S10 실행 뒤 STATE 로그(lift_m)나 스톱워치로 적는다(선행 조건 아님)
```

- 판정이 코드로 보는 것(Codex 8판 검토 P1 둘)
  - 형식, `status`, 근거 문서 파일.
  - `binds` 의 보드 일련번호·`lift_target_m`·`lift_rate_mps`·`test_pallet` 이 지금 값과 같다.
  - `lift_rate_mps ≤` 보드 승강 한계다.
  - **유지 높이**: 코어는 승강 도중에도 시간 초과·고장이면 DISARM 한다(`real_loop.py` 1800). 그래서 목표 높이만으로는 부족하다(Codex 8판 검토 2·3 회차 P1).
    - `hold` 에 목표 높이 한 줄(`raised_cm ≥ lift_target_m × 100 − 0.5`)이 있어야 한다.
    - 그리고 **무전원 유지가 가장 불리한 높이** 한 줄(`worst: true`)이 있어야 한다. 그 높이는 기구에 근거해 정하고 이유(`worst_reason`)를 적는다. 임의의 중간 한 점으로는 그 사이 높이에서의 역구동을 못 막는다.
    - 대신 기구의 자잠김·기계식 브레이크가 전 구간에서 동작한다는 근거 문서(`self_locking_evidence`, 부품 사양서 등)가 있으면 가장 불리한 높이 줄은 없어도 된다.
  - **처짐**: 모든 줄의 `sag_cm_after_60s ≤ sag_limit_cm` 이고, 한도에는 `sag_limit_decision_id`(총괄 결정)가 있어야 한다.
  - **원점·환산**: `|state_lift_m_at_floor| ≤ 0.005`, `|tape_cm_at_target / 100 − state_lift_m_at_target| ≤ 0.01`, `state_lift_m_at_target` 가 `lift_target_m` 의 0.5 cm 안이다. 호스트의 `lift_origin_ok` 는 센서값이 유한하고 범위 오류가 없다는 뜻일 뿐이다. 영점·환산이 틀리면 보고값이 기한 안에 맞아도 실제 포크 높이가 다를 수 있어 S10 전에 줄자로 본다.
  - 승강 기구를 바꾸면(`lift_mechanism`) 다시 잰다. 이것은 사람이 지키는 조건이다(MCU 일련번호로는 기구 교체를 구별하지 못한다).
- 출발 확인(2.6)의 여섯째 문구가 `test_pallet` 과 `pallet_mass_kg` 를 보인다.

## 3. 선행 조건 표

| ID | 조건 | 확인 방법 | 우회 가능? | 손작업(기존 단계 측정 = 추가 0 분 / 새 손작업) |
|---|---|---|---|---|
| C1 보드 | 운영 빌드(시운전·HIL 아님), `outputs_locked` 해제, 비상정지 배선 또는 면제 이미지 | 첫 `STATE` + 같은 fd 의 USB 제품 문자열·일련번호 | 불가 | 굽기는 기존 단계(0 분). 새: 판독 sha 대조 5 분 |
| C2 S5 안전 실측 | 최대 유효시간 DRIVE 직후 링크 단절, DISARM, Jetson 정지, 주 전원 차단, 각 경우의 포크 거동은 **최종 운영 이미지에서 직접** 잰다. IWDG 리셋 → 출력 차단은 운영 이미지로는 일으킬 수 없다(`FW_TEST_IWDG_STALL` 은 출력 잠금 빌드만, `CMakeLists.txt` 30). 그래서 같은 커밋의 잠금 `fw-stall` 빌드 워치독 동작과 받침대 리셋 → 출력 차단을 합친 근거로 둔다(도착일 절차 349) | 기록 `s5_safety`(직접 실측: `binds` 보드·이미지·정지 설정 해시; IWDG 근거: `binds` 커밋 + 리셋 → 출력 차단을 잰 보드 일련번호) | 불가(멈추는 쪽) | S5·IWDG 는 기존 단계(0 분). 새: 최종 이미지에서 축약 재확인(링크 단절 정지·DISARM 정지) 10 분 |
| C3 고장 검출 | 엔코더 고장 검출(3.3)·입력 나이 정지(2.2)가 실기에서 동작 | 기록 `fault_detection`(`binds`: 보드) | 불가(멈추는 쪽) | S4 고장 주입은 기존 단계(0 분). 새: 최종 이미지에서 고장 주입 한 번 5 분 |
| C4 port-review | 7 항목 실측 해결, 또는 총괄 제한 운용 승인(속도·사례) | 기록 `port_review`(`approved_limited` 면 `approved_max_speed_mps`·`approved_cases`) | 결정으로만(콘솔 우회 아님) | 0 분(총괄 결정) |
| C5 스캔 자격 | `lidar.yaml` `first_beam_err_s`·`timing.basis`·`applies_to`(7a-3), `basis_doc` 파일 있음, `mount.measured`(S7). 7a-3 승인 조건의 운영 부하 재확인. S10 이동 공간을 빔 높이가 대표하는 범위와 가림 구간(S7 가림 마스크) | 설정 직접 + 기록 `scan_load`·`scan_coverage` | 불가(층이 비면 장애물 정지 없음) | 7a-3 진단 측정·SDK 빌드는 7a-3 계획, 장착·가림 마스크는 S7, 부하는 C14 와 같은 실행(0 분) |
| C6 지도 묶음 | 2.8 대조 | 등록부·보고서 대조 | 불가(다시 조사·등록) | 조사 주행·시작 자세 줄자는 PR 7 벤치 7(0 분) |
| C7 IMU | 2.7 자격 셋 | 설정 직접 + 열린 프로필 | 가능(`imu` 세션 우회 — 기울기 감시 꺼짐 표시) | IMU 지연은 7b 벤치 2, 수평 기준은 S7(0 분) |
| C8 D5 | 깊이 원천 없음 → `d5_pocket` 우회와 팔레트별 확인(2.6) | 제출 판정 | 우회가 유일한 길이고 팔레트 확인은 우회 불가 | 벤치 0 분. 운용 중 제출마다 육안 확인 약 1 분 |
| C9 S10 속도·사례 | `max_speed_mps` ≤ 승인값, 사례 A 만 | 기록 `s10_speed`(`approved_max_speed_mps`·`approved_cases`) | 결정으로만 | 0 분(총괄 결정) |
| C10 S10-A 설정 | `map_id`·`start_rear`(바닥 줄자)·팔레트 치수·`check_pallet_pose` 실값 | 건조 조립(지금 규칙) + `start_rear` 자리표시자 금지(기록 `start_taped`) | 불가 | 시작점 줄자는 PR 7 벤치 7(0 분). 새: 팔레트 치수 5 분 |
| C11 승강 | 원점·환산(바닥과 목표 높이에서 줄자 대 `STATE`), 시험 팔레트(실제 하중)를 목표 높이로 들고 받침 없이 **실제 전원 차단** 뒤 하중 유지(R4, 처짐 cm ≤ 총괄 한도). `lift_rate_mps ≤` 보드 승강 한계(코드). 도달 시간은 선행 조건이 아니다(아래 C11 주) | 기록 `lift`(형식 2.10, `binds`: 보드·`lift_target_m`·`lift_rate_mps`·시험 팔레트) | 불가(멈출 때 하중이 떨어짐) | 새: 시험 팔레트를 목표 높이로 들고 전원 차단 처짐 줄자 + 바닥·목표 높이 줄자 대 `STATE` 10 분 |
| C12 차량 보정 | `vehicle.yaml` 의 G·바퀴 반지름·조향표·한계가 측정값(S2·S8) 또는 총괄 예외 결정, 그 값으로 펌웨어 재빌드 | 설정의 `provenance` 직접 + 기록 `firmware_build`(보정 식별자 = 헤더 생성 값) | 불가(속도·조향 한계의 근거) | G 는 S2, 반지름·조향은 S8, 재빌드·굽기는 기존 단계(0 분) |
| C13 정지·감시 문턱 | `obstacle_layer_real.yaml` `latency_s`·`decel_mps2`(S5 실측), `obstacle_layer.yaml` 지연 예산, `safety_watch.yaml` 전류·바퀴의 측정값 또는 총괄 예외 결정 | 설정의 `provenance`·`measured` 직접 | 불가(멈추는 쪽) | S5 결과 기입(기록 줄, 아래 '기록 쓰기'에 포함) |
| C14 부하·호스트 | 같은 Jetson 에서 원천·브리지·실행기, 동시 부하에서 50 Hz 틱·`STATE` 50 ms 신선도·25 ms 관문 예산(7r) 확인. 관문 예산은 정상 DRIVE 후보가 관문에 닿은 표본 수(**잠정** ≥ 500, 7r 측정 뒤 다시 정함)와 함께 기록. 출력 잠금 공회전 측정과 HIL 주행 경로 측정을 나눠 적는다 | 기록 `jetson_load`(`binds`: 설정 해시) | 불가(링크 신선도는 멈추는 쪽) | 25 ms 관문 예산은 PR 7 벤치(7r, 0 분) |

- **C5 주(총괄 질문)**: 스캔 상한이 없으면 층이 비고, 장애물 정지가 없다. 멈추는 쪽 보호가 없는 상태라 우회 불가를 권고한다.
  - '층 우회 명시'는 장애물 감지 없이 구동을 허락하는 것이라 원칙과 맞지 않는다.
  - 같은 이유로 REAL 의 `layer_timing` 우회도 막는다(2.5).
  - 7a-3 기입은 이미 '진단 측정 뒤 총괄 예외 결정(`basis: engineering`)'으로 정해져 있어 15 분 측정이면 채울 수 있다.
- **C11 주(총괄 조건, 학교 수준)**: 새 승강 벤치 도구는 만들지 않는다.
  - 선행 조건은 **원점·환산**과 **전원 차단 뒤 하중 유지**다. 하중 유지는 기구(자잠김)의 성질이라 이미지가 아니라 보드·차체·기구에 묶는다.
  - 시험 팔레트를 포크에 올린다. 기존 시운전 수단으로 가장 불리한 높이(기구 근거로 정함)와 `lift_target_m` 높이까지 차례로 띄운다(승강 시운전 축 듀티, 또는 손으로 받쳐 올림). 높이마다 받침을 빼고 주 전원을 끈 뒤 60 s 후 포크 처짐을 줄자로 cm 단위로 잰다. 목표 높이와 바닥에서 줄자 값과 `STATE lift_m` 을 같이 적는다(원점·환산).
  - **도달 시간**은 선행 조건에서 뺀다. S10 실행 중 코어가 이미 승강 도달 기한(승강 시작 약 4.5 s 뒤부터 10 s, `mission/core.py` 745·6593)을 판정한다. 넘으면 미션이 실패하고 포크는 그 자리에 선다. 그때 하중을 버티는 것이 위의 하중 유지 확인이다.
  - 그래서 도달 시간은 첫 S10 실행의 `STATE` 로그(`lift_m`, 또는 스톱워치)로 기록만 한다. 기한을 넘기면 `lift_rate_mps` 를 고치고 다시 한다.
- **우회 불가가 많다.** 대부분이 멈추는 쪽(정지·층·링크)이거나, 틀리면 위치·한계가 틀어지는 근거 값(보정·지도·승강)이다.
- **C13 은 값의 근거이지 토글이 아니다.** 전류·바퀴·IMU 감시의 세션 토글(`watch_current`·`watch_wheel`·`imu`)은 사용자 결정(2026-11-19, 실차 검사 세션 우회)으로 우회 가능하다. 토글을 켜면 그 감시가 무장 중 비상정지를 판단하지 않는다(`real_loop.py` 1298·1595). IMU 우회 중에는 분리 감시도 읽지 않는다(Codex 2 회차 P2).
  - S10 에서도 이 토글은 세션 우회를 허용한다(총괄 결정 2026-11-26, 8 절 2). 사전 승인 목록은 두지 않는다.
  - 콘솔이 '우회 중: 전류 감시(꺼짐)'처럼 무엇이 꺼졌는지 보인다. 그동안 대신 막는 것은 MCU 과전류·만료 정지와 사람 감시다.

## 4. 벤치·손작업 — 새 손작업만(S10 앞)

이미 잡힌 시운전 단계(S2·S4·S5·S6·S7·S8)와 7a-3·7b·PR 7 벤치에서 재는 값은 **추가 0 분**이다. 그 단계를 할 때 기록 파일 한 줄을 같이 쓴다. 이 계획이 새로 더하는 손작업은 아래뿐이고, 하루 약 15 분 이하로 날을 묶는다(총괄 조건 2026-11-26).

| 날 | 새 손작업 | 분 | 순서 조건 |
|---|---|---|---|
| A | 최종 운영 이미지 판독 sha 대조(C1·C12) + 고장 주입 한 번(C3) | 10 | S8 값으로 최종 굽기 뒤. 이 뒤로 다시 굽지 않는다 |
| B | 최종 이미지에서 링크 단절 정지·DISARM 정지 축약 재확인(C2·C13) | 10 | 날 A 뒤. 시운전 계획의 S5 나머지(Jetson 정지·주 전원 차단 등)는 **최종 판독 뒤 최종 이미지로** 하도록 일정을 잡아 따로 세지 않는다(Codex 8판 검토 P2) |
| C | 시험 팔레트 들고 전원 차단 처짐 줄자(C11) + 팔레트 치수(C10) | 15 | 승강 측정 안(3.5) 결정 뒤 |
| D | 기록 쓰기 정리: 각 단계에서 쓴 줄의 `binds`·근거 문서 확인(C2–C14) | 10 | 조사·등록(PR 7 벤치 7) 뒤, S10 전날 |

- 새 손작업은 합 45 분, 나흘이다. 운용 중에는 제출마다 팔레트 육안 확인 약 1 분(C8)이 더해진다.
- **순서**: 시운전 단계 순서는 그대로다. 단 두 가지를 지킨다.
  - 최종 굽기는 S8 값이 들어간 뒤 한 번이고, 그 뒤로는 굽지 않는다. 시험 이미지(IWDG `fw-stall`, 승강 시운전 축)는 그 앞에 쓴다.
  - 조사 주행·등록(C6)은 `lidar.yaml`·`vehicle.yaml` 확정(7a-3·S7·S8) 뒤다.
- 날 A·B 를 다시 해야 하는 경우는 최종 굽기를 다시 했을 때, 그리고 C2 에 묶인 정지 설정 파일(`stop_files`)을 바꿨을 때다(`binds` 무효).

## 5. 시험(벤치 없이)

- **판정(단위)**
  - 지금 저장소 설정 그대로면 C2–C14 가 사유와 함께 거부된다. C1 은 보드 사실이 필요해 따로 본다. 이것으로 이 계획의 코드가 들어가도 실차가 열리지 않는다는 것을 고정한다.
  - 기록 형식 오류, 근거 문서 없음, `decision_id` 없음, `binds` 불일치를 본다. 설정 해시 한 바이트, 보정 식별자, USB 제품 문자열이 대상이다.
  - `max_speed_mps` 0.16 대 승인 0.15, 사례 B 를 본다.
  - C5: `basis` 만 있음, `basis_doc` 없는 파일, `mount.measured: false`.
  - C6: 행 없음, 보고서 sha256 다름, 보고서의 `lidar` 해시 다름, 시작 자세 0.11 m 차이, `encoder_model` 다름.
  - C12·C13: `provenance` 가 잠정이면 거부, 총괄 예외 `decision_id` 가 있으면 통과.
  - `unlock_incomplete` 는 U3 전 늘 거부된다.
- **실 원천(단위·통합)**
  - `RealScanSource`: 합성 표식 거부, 수신 공백 → `lost`, 세대 알림이 자격 판정보다 먼저, `from_feed` 가 구간·세대 보존, `applies_to` 불일치 프레임은 층에 안 들어감.
  - IMU 래퍼: `fits_watch` 실패, 분리 → `lost`.
  - 층 설정: `lidar.yaml` 의 비영 장착·0.12 s 가 `RealLayer`·정지 모형에 반영된다.
  - 브리지: `hello` 의 `base_from_laser` 가 다르면 `fault` 다.
- **끝까지 도는 시험**(통합, 가짜 장치): MCU 모형 pty 에 운영 보드 표식(`HIL_PLANT` 0, `OUTPUTS_LOCKED` 0)을 둔다. `product_reader` 를 주입하고, 가짜 SDK 소켓·가짜 RealSense·가짜 SLAM 브리지를 쓴다. 실행 CLI 는 pty 를 거부하므로 시험은 조립 함수에 내부 주입한다.
  - 정상 경로를 끝까지 본다: 자격 스캔 설치 → 이번 세대 `processed` → 재위치 확인 → 팔레트 등록 → 팔레트 확인 → `d5_pocket` 우회 → 제출 → ARM → 비영 DRIVE(모두 ≤ 0.06 m/s) → S10 정지 → DISARM 확인.
  - 면제 제품 문자열이면 통과하고 비면제면 `estop_unwired` 다.
  - **출발 전 거부**는 실행 전체에서 ARM·비영 DRIVE 0 건을 본다.
    - 팔레트 확인 없음, 확인 뒤 lease·재위치 세대·revision 변경, `operator_start` 와 `d5_pocket` 동시 우회 요청.
    - 배치 0.11 m 벗어남, `start_pose` 우회를 켠 채 시작점 밖에서 제출, 접근·삽입에 후진 구간이 있는 계획, 방위 변화 15° 안이지만 통로를 벗어나는 S자 계획, 남은 시간 부족.
    - IMU 미검증이고 `imu` 우회가 없으면 제출이 닫힌다. `imu` 우회를 켜면 열린다.
    - `-dirty` 제품 문자열.
  - **정상 A 의 통과**: 정렬된 배치의 계획이 실제 `insert` 끝점까지 같은 검사기를 통과하고, 미실행 `extract`(후진)가 있어도 받는다.
  - 제출 뒤 계획 대기 중 위치가 허용차 밖으로 바뀌면(`start_pose` 우회를 켠 상태 포함) ARM·비영 DRIVE 0 건이다.
  - **주행 중 고장**은 고장 판정 **뒤**의 새 ARM·비영 DRIVE 0 건과 부분 프레임 폐기를 본다: 주행 중 보드 표식 변경, 스캔 단절, `applies_to` 변경, 수신 공백보다 짧은 어댑터 재시작(세대 변경), 같은 읽기에서 단절과 복귀, 이전 층 작업의 늦은 완료, 종료 신호.
    - 링크가 살아 있으면 DISARM 확인까지 본다.
    - 장치 상실이면 권한 폐기를 본다. MCU 만료 정지는 S5(C2)가 실기에서 본다.
  - 구버전 브리지(`laser_checked` 없음)면 `processed` 0 건이고 무장도 안 된다.
  - `layer_timing` 토글 거부, `imu:unqualified` 가 제출·무장 모두에서 막고 `imu` 우회 뒤 열림.
  - 실행 CLI 가 pty·재생·합성 입력을 거부한다.
- **회귀**: HIL 실행(`test_real_run_hil*`, 4 CPU)의 조립·제출·우회는 그대로다. 브리지 시험, 전체 pytest.
- **게이트**: 공유 미션 코어(`core.py`)·공유 장애물 층·Isaac 러너를 건드리지 않고 실차 루프·준비 판정·조립·브리지에서 막으면 재생 게이트는 필요 없다. 그 셋 중 하나라도 손이 가면 29/29 를 돌린다. 배치·경로 검사(2.4)와 세대 래치(2.5)는 실차 루프 쪽에 둔다.

## 6. 나눔

- **U1** — 송신 권한과 실 전송기, 판정과 기록(2.1·2.4·3 절, `unlock_incomplete` 유지). 실행 CLI 골격도 포함한다. 실 원천은 아직 없어 조립이 거부된다.
- **U2** — 실 원천과 한 보정(2.2·2.3·2.8). 스캔 어댑터, IMU 래퍼, SLAM REAL, 층 설정 배선, 브리지 `hello` 대조, 등록부 대조를 담는다.
- **U3** — 실차 준비 판정과 콘솔(2.5·2.6·2.7·2.9), 끝까지 도는 시험, 운영 빌드의 `FW_VEHICLE_HEADER` 덮어쓰기 금지(2.4, 펌웨어 교차 시험 fixture 포함), 펌웨어 제품 문자열에 태그와 무관한 커밋 약어를 늘 넣기(`git describe --long` 이나 별도 칸 — 태그 이름이 긴 형식처럼 보여도 커밋을 오인하지 않게, Codex U1 4 회차), `unlock_incomplete` 제거.
- 각 PR 은 Codex 를 수렴할 때까지 돌리고, HIL 4 CPU 와 전체 pytest 를 최종 트리에서 한 번 돌린다. U3 머지가 송신을 여는 유일한 지점이다. 그래도 설정이 비어 있으면 거부된다.

## 7. 남는 위험과 실기 전용 항목

- **기록은 사람의 확인이다.** 거짓 `pass` 는 막지 못한다. 근거 문서·`binds`·git 기록이 막는 전부다.
- **같은 표식으로 다시 구운 이미지는 구별하지 못한다.** 같은 커밋·같은 표식이면 USB 일련번호·제품 문자열이 같다. 그래서 '최종 판독 뒤 재굽기면 승인 무효'는 사람이 지키는 운영 관문이다(2.4).
- **하중은 사람이 확인한다.** 하중 센서가 없다. 승인한 시험 팔레트인지는 출발 확인의 문구로만 묶인다.
- **펌웨어 보정 식별자는 `STATE` 로 오지 않는다.** 그래서 탑재 펌웨어가 `vehicle.yaml` 과 같은 값으로 빌드됐는지는 기록(C12 `firmware_build`)으로만 묶인다. `STATE` 나 USB 제품 문자열에 넣는 것은 펌웨어 계약 변경이라 별도 결정이다.
- **실기에서만 보이는 것**: USB CDC 의 실제 끊김·재열거, D435i 분리, Jetson 동시 부하, 물리 정지 거리, 승강 처짐. 가짜 장치 시험은 이것을 증명하지 못한다. C1·C2·C11·C14 의 벤치 기록이 그 자리다.
- **조사 지도와 S10 당일 현장이 다를 수 있다.** 재위치 확인이 실패하면 선언(7b)으로 시작한다. 그래도 새 ARM 에는 신선한 `processed` 가 필요하다(지금 규칙, `real_loop.py` 2580).

## 8. 총괄 결정이 필요한 것

총괄 결정(2026-11-26, #643 승인 때):
1. **C5 우회 불가**: 권고대로다. REAL 에서 `layer_timing` 우회도 막는다(층·지연 감시는 멈추는 쪽).
2. **감시 토글**: `watch_current`·`watch_wheel`·`imu` 는 S10 에서도 세션 우회를 허용한다. 사전 승인 목록은 두지 않고, 다른 우회와 같이 세션 한정·기록·콘솔 '우회 중' 표시다. `imu` 우회 중이면 IMU 미검증이어도 제출·무장이 열린다. 펌웨어 과전류·STOP·DISARM·링크 단절은 우회 불가다.
3. **스캔 세대 변경 = 래치**: 동의.
4. **유한 실행 시간**: 남은 시간 부족 시 제출 거부, 삽입 진입·재개 전 재확인에 동의.

## 9. 자체 검토

- **① 원자료 재계산**(Claude, 1판 사실 확인): 1판 1 절의 줄 번호와 값을 코드에서 다시 읽었다.
  - 틀린 것: IMU 쪽 바꿀 곳을 둘로 적었으나 생성자 검사가 하나 더 있었다(311).
  - 확인한 것: 시작 허용차 0.10 m·5°(`real_loop.py` 63–64), `DECLARED_SPEED_MPS` 0.06(65).
- **② Codex 적대적 검토 1 회차**(스레드 01a215f5): P1 5, P2 4, P3 2.

  | 지적 | 처리 |
  |---|---|
  | P1-1 층·SLAM 이 `lidar.yaml` 보정을 안 씀, `from_layer` 가 구간·세대를 지움, 브리지 `laser_xyz_yaw` 따로 | 2.3, 2.2(`from_feed`), 브리지 `hello` 대조 |
  | P1-2 `layer_timing` 우회가 층 지연 보호까지 끔 | 2.5 REAL 우회 불가, 주행 중 자격 상실 시험, C5 에 운영 부하 재확인 |
  | P1-3 D5 우회가 사각형을 FREE 로 덮는데 문구 추가만으로 부족, `operator_start` 우회로 건너뜀 | 2.6 팔레트 revision 확인(우회 불가), `operator_start` 동시 우회 거부, U3 까지 송신 잠금 |
  | P1-4 기록이 차체·펌웨어·승인 범위에 묶이지 않음 | 2.4 `binds`·승인 값 강제, C2 경우 목록, 7 절 펌웨어 식별자 |
  | P1-5 실제 팔레트 들기 조건 없음 | C11 |
  | P2 정상 보드 송신 권한·잠금·USB 신원·종료·인계 | 2.1 |
  | P2 등록 행이 아니라 보고서에 해시, `encoder_model` 출처, 순서 | 2.8 |
  | P2 IMU 미검증과 감시 상실 구분, 311·`imu: none`·`fits_watch`·`level_ref`, 우회 범위 | 2.7 |
  | P2 시험이 조립에서 끝남, 합성 표식, SDK 모드 | 5 절 끝까지 도는 시험, C5 `sdk` 모드 |
  | P3 `start_rear` 는 자리표시자·`check_pallet_pose` null·사유 목록은 순수 함수에서 | C10, 2.4 |
  | P3 게이트 생략 조건 | 5 절 |

- **② Codex 2 회차**(같은 스레드, 2판 대상): 1 회차의 송신 경로·`layer_timing`·조기 해제는 닫힘. P1 6, P2 4, P3 2.

  | 지적 | 처리 |
  |---|---|
  | P1-1 구버전 브리지가 장착 대조를 건너뜀 | 2.3 `ready` 의 `laser_checked`, 구버전 조합 거부 시험 |
  | P1-2 짧은 재시작이면 앞 세대 증거가 남음 | 2.5 세대 변경 래치, 층 결과·대기 작업 폐기, 시험 셋 |
  | P1-3 `approved_cases` 가 경로를 제한하지 않음 | 2.4 배치(0.10 m·5°)·경로(후진 없음, 방위 변화 15°) 검사 |
  | P1-4 다른 보드·빌드가 같은 승인으로 통과 | 2.4 `binds` 에 USB 일련번호·`git describe`·펌웨어 헤더 보정 식별자·판독 대조 |
  | P1-5 승강·정지 승인값과 실행값의 결속, 빔 범위 | 2.4 `binds` 에 `lift_target_m`·`lift_rate_mps`·하중·지연 예산·오도메트리 나이 표·시험 속도, C5 `scan_coverage` |
  | P1-6 D5 확인 만료(lease·재위치), 승강 높이 | 2.6 `OperatorStart` 를 넓혀 세션·lease·재위치 세대·revision 결속·한 번 소비, 문구 다섯 |
  | P2 C13 과 감시 토글 충돌, IMU 우회 중 분리 감시 | 3 절 주, 8 절 2 |
  | P2 S8 이 너무 이름, `encoder_model`, 번들 선택 시점 | 4 절 순서, 2.8 |
  | P2 수신량·실행 시간 | 2.2 데이터그램 제한·사건 순서, 유한 실행과 `run_time_short` |
  | P2 실패 시험의 전후 구분, C14 표본 수 | 5 절, C14 |
  | P3 74 행, IMU 제출 판정 문구, 다른 호스트 거부 보장 없음, 보정 식별자, 게이트 조건 | 1·2.2·2.8·5 절 |

- **② Codex 3 회차**(같은 스레드, 3판 대상): 2 회차의 브리지·세대·확인 결속은 닫힘. P1 3, P2 4.

  | 지적 | 처리 |
  |---|---|
  | P1-1 후진 검사를 전체 계획에 하면 미실행 `extract` 로 정상 A 가 막히고, 접근만 보면 `start_pose` 우회·15° 안 S자가 통과 | 2.4 실행 단계(`approach`·`insert`)만, 시작점 검사는 우회와 무관, 승인 통로 ±0.10 m, 시험 |
  | P1-2 같은 표식으로 다시 굽은 이미지를 못 잡음 | 2.4 `-dirty` 거부, 운영 빌드의 헤더 덮어쓰기 금지, 재굽기면 승인 무효(사람 관문), 7 절 |
  | P1-3 승인 하중을 실제 팔레트와 대조할 경로 없음 | 2.6 여섯째 문구(시험 팔레트·질량), `lift_rate_mps` 와 보드 한계 확인, 7 절 |
  | P2 감시 토글 결정 전 무엇이 닫혀 있는지 | 2.7 `approved_bypasses`, 비어 있으면 IMU 미검증 시 제출·무장 닫힘 |
  | P2 C5 기록 결속 | 2.4 `scan_load`·`scan_coverage` 결속 |
  | P2 IWDG 는 운영 이미지로 일으킬 수 없음 | C2 직접 실측과 `fw-stall` 근거 분리, 4 절 4 |
  | P2 `run_time_short` 는 보장 아님 | 2.2 삽입 진입·재개 직전 재확인, 시한이 닿으면 정지·현장 복구 |

- **② Codex 4 회차**(같은 스레드, 4판 대상): 3 회차 지적은 반영 확인. P1 2, P2 3.

  | 지적 | 처리 |
  |---|---|
  | P1-1 통로가 접근 끝에서 끝나 정렬된 정상 A 의 삽입이 거부됨 | 2.4 통로를 삽입 끝까지, 정상 A 가 `insert` 끝점까지 통과하는 시험 |
  | P1-2 시작점 검사가 제출에서 끝남 | 2.4 무장 전 검사(`_off_start`)에도 REAL 규칙, 제출 뒤 위치 이탈 시험 |
  | P2 IWDG 차단 증거를 보드에 묶지 않음 | C2 `binds` 에 보드 일련번호, 4 절 재확인 조건에 하드웨어 변경 |
  | P2 헤더 덮어쓰기 금지가 펌웨어 교차 시험을 깸 | 2.4 fixture 에 헤더 생성 입력, 거부 시험 따로 |
  | P2 승강 상한만으로 S10 완료를 보장 못 함(도달 기한) | 2.4·C11 실제 도달 시간 |

- **② Codex 5 회차**(같은 스레드): 새 P1 없음, P2 2.

  | 지적 | 처리 |
  |---|---|
  | P2 IWDG 재시험이 최종 판독 뒤 재굽기 금지와 충돌 | 4 절 순서: `fw-stall`·승강 시험 이미지 먼저 → 최종 굽기·판독 → 그 이미지에서 S4·S5 직접 실측 |
  | P2 C11 을 잴 도구·절차가 없음, DISARM 은 전원 차단과 다름 | 4 절 4 승강 벤치 절차를 C11 선행 산출물로, 실제 전원 차단 뒤 하중 유지 |

- **② Codex 6 회차**(같은 스레드): 새 P1 없음, P2 1 — C11 을 시험용 이미지로 재면 최종 이미지 결속을 못 넘음. 처리: 4 절 4 는 예비 시험, 승인용 승강 실측은 6 단계에서 최종 이미지로 재굽기 없이.

- **② Codex 7 회차**: 새 P1/P2 없음(수렴).

- **8판 개정**(총괄 승인 조건, 축소): 감시 토글 사전 목록 삭제(결정 2), 벤치를 '기존 단계 측정 0 분 / 새 손작업'으로 나누고 새 손작업만 날별 ≤ 15 분(합 45 분), C11 을 전원 차단 처짐 줄자로(도달 시간은 런타임 코어 기한이 강제, 기록만), C14 표본 수 잠정 표시. Codex 8판 검토 1 회차 P1 2(C11 유지 높이·처짐 한도 결속, 원점·환산 확인)·P2 1(S5 를 최종 이미지로, 정지 설정 변경 시 재확인) 반영.

- **③ agy 독립 검토**(Gemini 4 Neon High, ①·② 를 보지 않음, 적히지 않은 전제 12 항목). ② 와 겹친 것은 송신 권한, 층 보정, 브리지 변환, 실 원천 부재, 보고서 해시·`encoder_model`, IMU 의 311·제출 판정, 팔레트 확인, `binds`, 끝까지 도는 시험이다. ② 에 없던 것과 처리는 다음과 같다.

  | 적히지 않은 전제 | 처리 |
  |---|---|
  | `vehicle.yaml`·층·감시 문턱의 잠정값이 측정됐다는 전제, 펌웨어 보정 식별자 불일치 | C12·C13, 4 절 1–2 순서, 7 절 |
  | 실 스캔 어댑터·IMU 장치 래퍼·`--scan-socket` 이 있다는 전제 | 2.2 |
  | 콘솔에 확인 문구가 있다는 전제 | 1 절 정정, 2.6·2.9 |
  | 실행 시간·기동 예산·정리가 HIL 과 같아도 된다는 전제, 거부 시 콘솔이 안 뜸 | 2.2 CLI, 2.4 출력 |
  | 승강 측정·자잠김·Jetson 부하·한 시계 도메인 | C11·C14, 2.2 한 호스트 |
