# 실제 A2M12 스캔 → 장애물 층 → 텔레옵 허가 (bring-up ⑦, 계획 P0-5a 의 S6 몫)

작성 2026-11-12 · 상태 **초안(검토 전)** · 계획 [sim-to-real bring-up](../plans/2026-11-11-sim-to-real-bringup.md) 2.1·2.3a·2.5·P0-5a·S6·S7 · 선행 [텔레옵 다리](2026-11-12-teleop-bridge.md)(#459), [벽 앞 탈출](2026-11-12-standing-direction-rule.md)(#465)

## 0. 범위와 전제

- 목적: S6 텔레옵 다리의 바닥 허가(`floor_permit`)를 지금의 `no_scan`(언제나 0)에서 **실제 A2M12 스캔으로 만든 장애물 층**으로 바꾼다. 그래야 실차 바퀴가 처음으로 돈다.
- 계획 P0-5a 의 합격 조건 중 'SLAM 과 장애물 층에 같은 스캔'은 이 문서 밖(SLAM 실차 모드, P1)이다. 여기서는 SLAM 없이 **오도메트리 좌표**에서 층을 만든다. S6 는 SLAM 없이 텔레옵이 열린다(총괄 결정, 다리 설계).
- 사용자 결정(2026-11-12): 비상정지는 다음 PCB 판이다. 현 판에서는 S4·S5 정상 빌드·S6 가 안 된다. 그래서 **실차 텔레옵 연결 시험은 다음 판까지 못 한다.** 지금 할 수 있는 실기 시험은 **노트북에 꽂은 A2M12 단독**이다(사용자가 이번 주 연결).
- 지키는 것:
  - K2: 센서는 ROS 컨테이너 안의 어댑터가 Unix 소켓으로 넘긴다. 실행기(다리) 쪽은 rclpy·시리얼을 import 하지 않는다.
  - 2.5 시간 규칙: 판단은 단조 시계로만 한다. ROS stamp 는 바꾸지 않고, (ROS stamp, 수신 단조 시각) 짝으로 변환한다.
  - CONTRIBUTING 5절: 무효 거리(0·inf·범위 밖)는 NaN(모름)이다. 빈 공간으로 넣지 않는다.
  - 2.3a: 합성 잡음을 다시 넣지 않는다.
- 이 문서가 정하지 않는 것: A2M12 의 포트·보율·스캔 모드(노트북에 연결한 뒤 직접 확인), 장착 위치(실기 때 사용자에게 묻고 S7 에서 측정).

## 1. 경로

```
A2M12 (USB-UART) ─ sllidar_ros2 ─ /sensors/lidar/scan_raw ─ real_scan_adapter ─ Unix 소켓 ─ 다리(teleop_bridge)
                                   (ROS 컨테이너 안)                                          ├ ScanFeed (나이·형식 검사)
                                                                                              ├ 오도메트리(엔코더+조향)
                                                                                              ├ ObstacleLayer(실제 스캔 입력)
                                                                                              └ floor_permit = 층 허가
```

### 1.1 드라이버
- `sllidar_ros2` 를 커밋 고정으로 소스 빌드해 ROS 이미지에 넣는다. x86(노트북 시험)용은 `deploy/ros2/`, ARM64(Jetson)는 계획의 `deploy/ros2-real/`(P0-2)이다.
- 원시 토픽은 `/sensors/lidar/scan_raw` 로 remap 한다(2.1 토픽 소유).
- 포트는 udev 규칙으로 고정 이름(`/dev/forklift-lidar`)을 준다. 연결 뒤 VID/PID 를 확인해 #462 와 같은 방식으로 적는다.

### 1.2 어댑터 (`forklift_ros/real_scan_adapter`, ROS 컨테이너)
- `/sensors/lidar/scan_raw` 를 구독해, 받은 메시지마다 한 프레임을 Unix 소켓(SOCK_SEQPACKET, 경로는 인자)으로 보낸다. 받는 쪽이 없거나 느리면 **그 스캔을 버린다**(막고 기다리지 않음, 버린 수는 센다).
- 프레임 `scan/1`(JSON 머리 + 바이트 본문, 길이 검사):
  - `scan_id`(어댑터 단조 증가), `ros_stamp_ns`(드라이버 stamp 그대로), `recv_mono_ns`(어댑터 수신 `CLOCK_MONOTONIC`), `sys_minus_mono_ns`(그 순간 시스템 시각 − 단조 시각).
  - `angle_min`, `angle_increment`, `time_increment`, `scan_time`, `range_min`, `range_max`, `frame_id`.
  - `ranges`(f32 배열), `intensities`(있으면).
- 어댑터는 값을 고치지 않는다. 각도·유효성 해석은 다리가 한다(한 곳에서 시험).

### 1.3 다리 쪽 `ScanFeed` (`src/forklift_core/sensors/scan_feed.py`, 새로)
- 소켓에서 비차단으로 읽는다(다리의 '수신만 → 판단 → 송신만' 주기의 수신 단계).
- 형식 검사: 길이, `angle_increment > 0`, 빔 수 상한, 유한한 수. 하나라도 틀리면 그 스캔을 버리고 센다.
- **빔 각도는 스캔마다** `angle_min + i·angle_increment` 로 만든다. 고정 빔 각도(`ObstacleLayer.beam_angles`)를 쓰지 않는다(2.3a).
- 거리: `range_min ≤ r ≤ range_max` 이고 유한한 값만 쓴다. 나머지는 NaN 이다(모름, 지우지 않음).
- **스캔 시각**: 스캔 끝의 단조 시각을 `ros_stamp_ns + scan_time − sys_minus_mono_ns` 로 계산한다(드라이버 stamp 가 스캔 시작이라는 전제 — 연결 뒤 sllidar_ros2 소스와 실측으로 확인한다). 이 값과 `recv_mono_ns` 의 차이를 **드라이버 지연**으로 기록한다. 지연이 상한(잠정 0.15 s)을 넘거나 음수(−5 ms 아래)면 그 스캔을 쓰지 않는다. `sys_minus_mono` 가 직전보다 50 ms 넘게 뛰면(시스템 시각 조정) 경고하고, 다음 스캔부터 새 값으로 계산한다.
- 회전 중 이동(비뒤틀림 보정 없음): 한 바퀴(10 Hz 면 0.1 s) 동안 차가 움직인 거리는 명령 상한 0.132 m/s × 0.1 s = 13 mm 이다. 스캔 전체를 **스캔 가운데 시각의 오도메트리 자세 하나**에 놓고, 반 바퀴 이동량(≤ 7 mm)을 층의 센서 오차 폭에 더한다(3.2).

### 1.4 오도메트리 (다리, 새로)
- 지금 다리는 엔코더로 속도만 만든다. 층을 놓으려면 **뒤축 오도메트리 자세**가 필요하다. 시작점을 (0, 0, 0)으로 하는 오도메트리 좌표에서, STATE 마다 바퀴 카운트 차분과 측정 조향으로 적분한다(`team_spec_quantized` 와 같은 관절각 모형을 쓰고, 그 모형의 실차 판은 `forklift_core` 의 기존 함수를 쓴다 — 구현 때 확인).
- 같은 부팅 안에서만 잇는다. 부팅이 바뀌거나 STATE 간격이 상한(`dt_max_s`)을 넘으면 오도메트리를 **끊는다**. 끊기면 층을 비우고 허가는 0 이며, 새 스캔이 충분히 쌓일 때까지 기다린다.
- 스캔 가운데 시각의 자세는 그 앞뒤 STATE 의 자세를 보간해서 얻는다. 시각이 STATE 범위 밖이면(너무 오래된 스캔) 그 스캔을 쓰지 않는다.

### 1.5 장애물 층 (`sim/isaac/obstacle_layer.py` 의 `ObstacleLayer` 를 그대로 쓰되 실제 스캔 입력 추가)
- 새 입력 `add_real_scan(stamp_s, sensor, odom_rear, angles, ranges, own)` 을 둔다. 합성 레이캐스트용 `add_scans` 와 나란히 둔다. 합성 경로(잡음 추가)는 거치지 않는다.
- 설정은 `config/real/obstacle_layer.yaml` 이다. 같은 A2 단일 LiDAR 설정(`config/obstacle_layer_single_a2_team_spec.yaml`)을 바탕으로 하되 다음이 다르다.
  - `sensors`: 장착은 `config/real/sensors.yaml` 의 `base_link_from_laser` 에서 읽는다. S7 전에는 **추정값**이고 꼬리표를 단다.
  - `beams` 는 쓰지 않는다(스캔마다).
  - 격자 범위(`hall`): 오도메트리 원점 둘레의 정사각형(잠정 ±10 m). 차가 범위 끝에서 1 m 안에 들어오면 허가 0 이다.
  - 나이 오차표(`AgeErrorTable`): 시뮬의 `team_spec` 표를 잠정으로 쓰고, 실측(S5·S6)으로 바꾼다고 적는다.
- **자기 차체 가림**: 스캔 각도 중 마스트·차체가 가리는 구간을 `config/real/sensors.yaml` 의 `laser_self_mask`(각도 구간 목록)로 둔다. 그 빔은 `own=True`(지우지도 표시하지도 않음)이다. S7 전에는 차체 모형 윤곽에서 계산한 추정 구간에 여유(±3°)를 더한 값이다.
- 적재물 가림은 S6 범위 밖이다. 다리는 빈 포크만 연다(시작 확인 `--empty-forks`).

### 1.6 허가 (`floor_permit`)
- 실행기의 `live_permit`(`run_transport.py` 8602 행)과 같은 순서를 따른다.
  1. 층 지연 예산 감시(`watchdog`).
  2. 층 snapshot 이 없으면 0.
  3. **스캔 신선도**: 마지막으로 층에 들어간 스캔의 단조 시각이 지금보다 0.3 s(잠정) 넘게 오래되면 0. 사유는 `scan_stale`.
  4. 오도메트리가 끊겼거나 격자 범위 끝이면 0.
  5. 같은 허가 계산(`LiveDrivePermission.limit` 와 같은 함수 — 지금 `run_transport.py` 안에 있으니 `forklift_core` 로 옮긴다, 4절)에 오도메트리 뒤축 자세·곡률 묶음·방향·검사 속도·차체 윤곽을 넣는다.
- 다리의 방향 규칙(`output_direction_check`)과 벽 앞 탈출은 이 허가 위에서 그대로다.
- 노트북 단독 시험과 실차 사이의 차이는 오도메트리뿐이다(노트북: 고정 자세 (0, 0, 0), 4절).

## 2. 노트북 A2M12 단독 시험 (이번 주, 장치 연결 뒤)

목적: 실차 없이 드라이버 → 어댑터 → ScanFeed → 층 → 허가 경로를 실제 스캔으로 확인하고, 연결 사실(포트·보율·모드·빔 수·주기·지연·무효 비율)을 기록한다.

사용자에게 칠 명령(짧게):
```bash
# 1. 장치 확인 (꽂은 뒤)
ls -l /dev/serial/by-id/ ; udevadm info -q property -n /dev/ttyUSB0 | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL'
# 2. 스캔 확인 (ROS 컨테이너에서 드라이버+어댑터, 호스트에서 확인 도구)
tools/ros2_dev.sh scan-a2m12 /dev/ttyUSB0          # 드라이버와 어댑터를 띄운다
python tools/scan_check.py --socket /tmp/forklift-scan.sock --seconds 30 --out scan_check.json
```
- 포트·보율은 1 의 출력과 sllidar_ros2 의 A2M12 launch 기본값으로 정하고, 노트북에서 내가 직접 확인한다(총괄 지시).
- `scan_check.py` 가 낸다:
  - 스캔 주기·빔 수·각도 범위·`angle_increment`.
  - 무효(NaN) 비율, 드라이버 지연 분포(p50·p99·최대), `sys_minus_mono` 변화.
  - 버린 스캔 수와 사유.
  - 고정 자세 (0, 0, 0) 에 차체 윤곽을 놓았을 때의 층 허가(앞·뒤, 0.13 m/s). 장치 주위에 상자를 놓았다가 치우면 허가가 바뀌는 것을 본다.
- 합격(잠정): 30 s 동안 끊김 없음, 주기 편차 ±20 %, 지연 최대 0.15 s 안, 상자를 놓은 쪽 허가가 줄어듦. 결과는 `docs/validation/` 에 기록한다(원자료는 CONTRIBUTING 7절의 자리).

## 3. 수치 (재계산 대상)

### 3.1 전송량·빔 수
- A2M12 의 카탈로그 값(실측 아님, `docs/hardware.md`): 표본 16 kHz(최대), 회전 10 Hz 기본 → 스캔당 최대 1600 빔. f32 거리 1600 × 4 B = 6.4 kB/스캔, 10 Hz 에 64 kB/s(세기 포함 128 kB/s). Unix 소켓에 문제없다.
- 빔 수 상한은 4096 으로 둔다(형식 검사).

### 3.2 오차 폭
- 층의 센서 오차 폭(`sensor_bound_m`)에 더하는 회전 중 이동: 반 바퀴 × 명령 상한 = 0.05 s × 0.132 m/s = 6.6 mm. 경계 속도(≤ 0.2 m/s, 다리 STOP 문턱)로 잡으면 10 mm. **경계 속도로 잡는다**(10 mm).
- 스캔 끝 시각을 쓰는 신선도: 0.3 s 의 내역 = 스캔 주기 0.1 + 드라이버 지연 상한 0.15 + 다리 주기 0.02 + 여유 0.03.

### 3.3 격자 나이와 실제 지연 (재계산에서 나옴)
- 시뮬 A2 설정은 `free_max_age_s` 0.2 s, `occupied_max_age_s` 0.3 s 이다(`config/obstacle_layer_single_a2_team_spec.yaml`). 시뮬의 스캔 나이에 맞춘 값이다.
- 실제 스캔은 쓰일 때 최대 0.1(주기) + 0.15(드라이버 지연 상한, 잠정) + 0.02(다리 주기) = **0.27 s** 늙는다. 그대로 두면 빈 칸이 쓰이기 전에 만료되어 **허가가 늘 0** 이다.
- 그래서 실제 설정의 칸 나이는 시뮬 값을 옮기지 않는다. ⑦c 에서 잰 지연 분포(p99)로 정한다: `free_max_age_s ≥ 주기 + 지연 p99 + 다리 주기`, `occupied_max_age_s` 는 그보다 크게. 나이가 늘면 나이 오차표의 위치 오차도 함께 커진다(같은 표를 쓴다). ⑦c 전까지는 0.3 / 0.4 s 를 잠정으로 두고 꼬리표를 단다.
- 드라이버 지연 상한(0.15 s)도 실측 전 잠정이다. 실측 p99 가 크면 상한을 늘리기 전에 원인(USB-UART 버퍼, 드라이버 모드)을 먼저 본다.

### 3.4 격자
- ±10 m, 해상도는 기존 설정(5 cm)이다. 400 × 400 = 160,000 칸으로, 시뮬 홀보다 작다.

## 4. 바꾸는 코드와 나누기

| 조각 | 내용 | 장치 필요 |
|---|---|---|
| ⑦a | `scan/1` 형식(`forklift_core/sensors/scan_feed.py`), ScanFeed, 어댑터 노드, ROS 이미지에 sllidar_ros2(커밋 고정), `tools/scan_check.py`, udev 규칙 초안 — 합성 스캔 시험 | 아니오(시험), 연결 뒤 실측 |
| ⑦b | `add_real_scan`, `config/real/obstacle_layer.yaml`·`sensors.yaml` 의 LiDAR 장착·자기 가림(추정 꼬리표), `LiveDrivePermission` 을 `forklift_core` 로 옮김(실행기 import 만 바꿈, 동작 그대로 — 재생 게이트), 다리 오도메트리, `floor_permit` 연결, 고정 자세 모드(노트북) | 아니오 |
| ⑦c | 노트북 실측과 기록, 문턱 확정 | 예 |

- ⑦b 의 `LiveDrivePermission` 이동은 실행기를 건드리므로 재생 게이트 29/29 가 필요하다.
- 실차 연결 시험(S6)은 다음 PCB 판 뒤다.

## 5. 시험

- ScanFeed:
  - 형식 오류마다 버림·계수. 무효 거리(0, inf, 범위 밖)는 NaN.
  - 빔 각도는 스캔마다 다름(빔 수가 다른 두 스캔).
  - 시각 변환, 지연 상한·음수 지연 거부, `sys_minus_mono` 점프 경고.
- 어댑터: 받는 쪽이 없을 때 버림(막지 않음). `scan/1` 왕복.
- 층:
  - 실제 스캔 입력이 합성 경로와 같은 격자를 만듦(같은 빔·같은 거리일 때).
  - NaN 빔은 지우지 않음. 자기 가림 구간은 표시도 지움도 없음.
- 오도메트리: 직진·회전 적분, 부팅·간격 끊김, 스캔 시각 보간과 범위 밖 거부.
- 허가:
  - 스캔이 0.3 s 넘게 끊기면 0(`scan_stale`).
  - 오도메트리 끊김·격자 끝이면 0.
  - 앞에 상자 → 앞 허가 감소, 뒤에 벽 → 벽 앞 탈출 제안(#465 와 연결).
- 다리 주기(가짜 MCU + 합성 스캔 소켓): 스캔이 들어오면 바퀴가 돌고, 스캔이 끊기면 멈춤.
- 재생 게이트(⑦b): 29/29.

## 6. 자체 검토

| 검토 | 지적 | 처리 |
|---|---|---|
| (작성 중) | | |
