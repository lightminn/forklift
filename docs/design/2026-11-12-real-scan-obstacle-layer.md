# 실제 A2M12 스캔 → 장애물 층 → 텔레옵 허가 (bring-up ⑦, 계획 P0-5a 의 S6 몫)

작성 2026-11-12 · 2판(독립 검토 3 개 반영) · 상태 **⑦a 설계 확정 대기, ⑦b 는 요구사항만** · 계획 [sim-to-real bring-up](../plans/2026-11-11-sim-to-real-bringup.md) 2.1·2.3a·2.5·P0-5a·S6·S7 · 선행 [텔레옵 다리](2026-11-12-teleop-bridge.md)(#459), [벽 앞 탈출](2026-11-12-standing-direction-rule.md)(#465)

## 0. 범위

- 목적: S6 텔레옵 다리의 바닥 허가(`floor_permit`)를 지금의 `no_scan`(언제나 0)에서 **실제 A2M12 스캔으로 만든 장애물 층**으로 바꾼다.
- 사용자 결정(2026-11-12): 비상정지는 다음 PCB 판에 넣는다. 그래서 현 판에서는 S4·S5 정상 빌드·S6 가 안 되고, **실차 텔레옵 연결 시험은 다음 판까지 못 한다.** 지금 할 수 있는 실기 시험은 **노트북에 꽂은 A2M12 단독**이다(사용자가 이번 주 연결).
- 1판의 독립 검토(6절) 결과로 둘로 나눈다.
  - **⑦a(지금)**: 드라이버 → 어댑터 → 소켓 → `ScanFeed` → `scan_check` 까지 스캔 경로만 만든다. 노트북에서 스캔과 시각을 확인한다. 허가는 계속 0 이다.
  - **⑦b(⑦a 실측 뒤, 별도 설계·검토)**: 실제 스캔으로 층을 만들고 허가를 연다. 1판의 P1 다섯 개가 닫히기 전에는 **비영 허가를 열지 않는다**(3절 요구사항).
- 지키는 것:
  - K2: 센서는 ROS 컨테이너 안의 어댑터가 Unix 소켓으로 넘긴다. 다리 쪽은 rclpy·시리얼을 import 하지 않는다.
  - 2.5 시간 규칙: 판단은 단조 시계로만 한다. ROS stamp 는 바꾸지 않고 (ROS stamp, 수신 단조 시각) 짝으로 변환한다. 시각이 뛰면 **경고하고 정지**한다(경고만으로 끝내지 않음, Codex P1-1).
  - CONTRIBUTING 5절: 무효 거리(0·inf·범위 밖)는 NaN(모름)이다. 빈 공간으로 넣지 않는다. 층에서 NaN 은 UNKNOWN 으로 남고, UNKNOWN 은 허가를 막는다(Codex 지적 없음; ⑦b 시험으로 고정).
  - 2.3a: 합성 잡음을 다시 넣지 않는다.
- 이 문서가 정하지 않는 것: A2M12 의 포트·보율·스캔 모드(연결 뒤 직접 확인), 장착 위치(실기 때 사용자에게 묻고 S7 에서 측정).

## 1. ⑦a — 스캔 경로 (지금 구현)

```
A2M12 (USB-UART) ─ sllidar_ros2(커밋 고정) ─ /sensors/lidar/scan_raw ─ real_scan_adapter ─ Unix 소켓 ─ ScanFeed ─ scan_check
                    (ROS 컨테이너 안)                                                       (호스트, 다리와 같은 코어)
```

### 1.1 드라이버와 시각의 뜻 (Codex P1-1)
- `sllidar_ros2` 를 커밋 고정으로 소스 빌드해 `deploy/ros2/`(x86, 노트북 시험) 이미지에 넣는다. ARM64 는 계획의 `deploy/ros2-real/`(P0-2)에서 같은 커밋을 쓴다.
- 확인된 사실(upstream main, Codex 대조): 노드는 `grabScanDataHq()` **호출 직전**의 시각을 stamp 로 쓰고, 호출 전후 차이를 `scan_time` 으로 낸다. SDK 는 스캔마다 자기 timestamp 를 갖지만 이 호출에서 버린다. 그래서 **stamp 는 취득 시각이 아니다.** 버퍼에 있던 스캔이 빨리 돌아오면 오래된 관측이 신선해 보인다.
- ⑦a 에서 할 일:
  1. 고정할 커밋에서 위 사실을 다시 확인한다.
  2. SDK 의 스캔 timestamp 가 어느 시계인지(장치 시각인지, 호스트 수신 시각인지) 소스로 확인한다.
  3. 그 값을 메시지로 꺼낼 수 있는지 본다. 필요하면 노드를 작게 고친 사본(fork, 커밋 고정)을 둔다.
  4. **드라이버 정체·버퍼 지연 주입 시험**으로 스캔 나이 추정이 늦은 스캔을 신선하다고 하지 않는지 검증한다.
- 그 전까지 ⑦a 의 지연 수치는 **'stamp → 수신'** 으로만 이름 붙인다. 취득 나이라고 부르지 않는다.
- **고정 커밋에서 확인한 것**(2026-11-12, `34300099`, 2024-06-17 — 이미지가 이 커밋을 쓴다):
  - 노드(`src/sllidar_node.cpp` 362–365 행)는 `grabScanDataHq()` 직전의 `this->now()` 를 `header.stamp` 로, 호출 전후 차이를 `scan_time` 으로 낸다. 1.1 의 사실 그대로다.
  - SDK 에는 `grabScanDataHqWithTimeStamp` 가 있다(`sdk/src/sl_lidar_driver.cpp` 783 행). 그 값은 **스캔 첫 표본의 시각**이다(`_scan_begin_timestamp_uS`, 293 행).
  - 이 시각은 호스트 `CLOCK_MONOTONIC`(`arch/linux/timer.cpp`)으로 디코드한 시각에서 표본·필터·전송·묶음 지연 모형을 뺀 값이다(`handler_hqnode.cpp` 등). `native_timestamp_support = false` 라 장치 시각이 아니다.
  - 컨테이너는 호스트 커널의 단조 시계를 같이 쓴다. 그래서 이 값은 다리의 단조 시계에서 **취득 시작의 추정치**로 쓸 수 있다. 다만 노드가 이 값을 내지 않으므로, 쓰려면 노드를 고친 사본이 필요하다. 이 결정은 ⑦b(3절 1)에서 한다.
  - A2M12 launch 기본값: `serial_baudrate` 256000, `scan_mode` Sensitivity, `angle_compensate` true, `frame_id` laser. upstream launch 파일 이름에 공백이 있어(`sllidar_a2m12_launch .py`) 런처는 `ros2 run` 으로 같은 인자를 준다.
  - 각도: 노드가 `angle_min = π − angle_max` 로 ROS 규약(반시계)에 맞춘다. 거리 0 은 `inf` 로 내고, `range_min` 은 0.05 m 로 고정이다. `ScanFeed` 에서는 둘 다 NaN 이다.
- 원시 토픽은 `/sensors/lidar/scan_raw` 로 remap 한다(2.1 토픽 소유).
- 포트는 udev 규칙으로 고정 이름(`/dev/forklift-lidar`)을 준다. 연결 뒤 VID/PID·일련번호를 확인해서 규칙을 만든다. 흔한 USB-UART 칩(예: CP2102)은 다른 장치와 VID/PID 를 같이 쓰므로, 일련번호 없이 맞추지 않는다. `GROUP` 은 정하지 않는다(배포판마다 직렬 그룹이 다름: Ubuntu/Jetson dialout, Arch uucp). USB-UART 의 `latency_timer` 값도 함께 기록한다(agy).

### 1.2 어댑터 (`ros2/src/forklift_ros/forklift_ros/real_scan_adapter.py`, ROS 컨테이너)
- `/sensors/lidar/scan_raw` 를 구독해 메시지마다 한 프레임을 **Unix 데이터그램 소켓**(경로는 인자, 호스트가 bind)으로 보낸다. 데이터그램이라 한 프레임은 통째로 가거나 안 간다. 받는 쪽이 없거나(`ENOENT`·`ECONNREFUSED`) 가득 차면(`EAGAIN`) **그 스캔을 버리고 센다**(막고 기다리지 않음). 1판의 SEQPACKET 은 연결 수락이 필요해 데이터그램으로 바꿨다.
- 프레임 `scan/1`:
  - 머리(JSON): `scan_id`(어댑터 단조 증가), `ros_stamp_ns`(드라이버 stamp 그대로), `recv_mono_ns`(수신 `CLOCK_MONOTONIC`), `sys_minus_mono_ns`(그 순간 시스템 시각 − 단조 시각), `angle_min`, `angle_increment`, `time_increment`, `scan_time`, `range_min`, `range_max`, `frame_id`, `n`, 1.1 에서 꺼낸 SDK timestamp(있으면).
  - 본문: `ranges`(f32 × n), `intensities`(있으면 f32 × n).
- 어댑터는 값을 고치지 않는다. 해석은 `ScanFeed` 한 곳에서 한다.

### 1.3 `ScanFeed` (`src/forklift_core/sensors/scan_feed.py`, 새로)
- 소켓에서 비차단으로 읽는다(다리의 '수신만' 단계에서도 쓸 수 있게).
- 형식 검사: 길이, `n ≤ 4096`, `angle_increment > 0`, 유한한 수. 틀리면 그 스캔을 버리고 사유별로 센다.
- 빔 각도는 스캔마다 `angle_min + i·angle_increment` 로 만든다(고정 빔 각도를 쓰지 않음, 2.3a).
- 거리: `range_min ≤ r ≤ range_max` 이고 유한한 값만 쓴다. 나머지는 NaN 이다.
- 시각:
  - 'stamp 단조 시각' = `ros_stamp_ns − sys_minus_mono_ns`, 'stamp → 수신' = `recv_mono_ns − stamp 단조 시각`.
  - `sys_minus_mono` 의 기준값을 둔다. 한 스캔의 값이 기준값에서 **50 ms 넘게** 벗어나면(시스템 시각 조정) 그 스캔부터 거부하고 재수립 상태로 간다(Codex P1-1).
  - 재수립: 연속한 스캔 **10 개**의 값이 그 10 개의 중앙값에서 모두 **±1 ms** 안이면 그 중앙값이 새 기준값이 되고, 다음 스캔부터 낸다. 두 시계를 따로 읽어 생기는 정상 흔들림은 1 ms 보다 훨씬 작다(정수 ns 완전 일치를 요구하지 않음, Codex 2판 P2-1). 10 개 안에서 다시 50 ms 넘게 뛰면 처음부터 다시 센다. 기준값 안에서의 작은 흔들림(≤ 50 ms)은 거부하지 않고 기록한다.
  - 'stamp → 수신' 이 음수(−5 ms 아래)면 거부한다.
- 결과는 `RealScan(scan_id, stamp_mono_ns, recv_mono_ns, angles, ranges, intensities, meta)` 다. 층이 쓸 시각(취득 구간)은 ⑦b 가 1.1 의 결과로 정한다.

### 1.4 `tools/scan_check.py` (호스트, 스캔만)
- 소켓에서 `ScanFeed` 로 읽어 N 초 동안 다음을 낸다(⑦a 는 **스캔 검사만**이다. 층·허가는 ⑦b, Codex P2-8):
  - 스캔 주기·빔 수·각도 범위·`angle_increment`·`scan_time`.
  - NaN 비율, 'stamp → 수신' 분포(p50·p99·최대), `sys_minus_mono` 변화.
  - 버린 스캔 수와 사유, 어댑터가 버린 수(`scan_id` 틈).
  - **각도 규약 확인**: `--probe right|left|front|back` 이면 그쪽 가까이 놓은 상자의 최소 거리 빔의 각도를 보여 준다. ROS 규약(앞 0, 왼쪽 +, 반시계)과 맞는지 판정한다(agy).
- 원시 기록: `--record out.scan.jsonl` 이면 **`ScanFeed` 판정 전의 수신 프레임**을 그대로 남긴다(머리 JSON + 본문 base64 + 호스트 수신 단조 시각). 그 뒤에 `ScanFeed` 의 판정(수락, 또는 거부 사유와 재수립 상태)을 함께 적는다. 재생은 기록한 프레임을 같은 순서로 새 `ScanFeed` 에 넣는 것이고, 수락·거부 순서가 기록과 같아야 한다(Codex 2판 P2-2). ⑦b 의 재생 입력이다.

### 1.5 노트북 시험 (사용자 연결 뒤)
사용자에게 칠 명령:
```bash
# 1. 장치 정보 (꽂은 뒤)
ls -l /dev/serial/by-id/
# 2. 드라이버+어댑터 (ROS 컨테이너), 다른 창에서 확인 도구
docker build -t forklift/ros2-dev:jazzy -f deploy/ros2/Dockerfile .      # 한 번(sllidar_ros2 포함)
python tools/scan_check.py --socket /tmp/forklift-scan/scan.sock --seconds 30 --record a2m12_first.scan.jsonl &
tools/ros2_dev.sh scan-a2m12 /dev/ttyUSB0
# 3. 각도 확인: 라이다 오른쪽 30 cm 에 상자를 두고
python tools/scan_check.py --socket /tmp/forklift-scan/scan.sock --seconds 5 --probe right
```
- 포트·보율·스캔 모드는 1 의 출력과 sllidar_ros2 의 A2M12 launch 기본값으로 정한다. 나머지는 내가 직접 확인한다(총괄 지시).
- ⑦a 의 합격은 **연결과 정적 경로**다(실차 문턱을 정하지 않음, Codex P2-8):
  - 30 s 동안 끊김 없음.
  - 각도 규약 일치.
  - 형식 거부 0.
  - 'stamp → 수신' 분포와 `latency_timer` 를 기록.
  - 드라이버 정체 주입에서 늦은 스캔이 드러남(1.1 의 4).
- 결과는 `docs/validation/` 에 기록한다(원자료는 CONTRIBUTING 7절의 자리).

## 2. 노트북 시험이 보여 주지 못하는 것 (agy)
- 주행 진동·피치에 따른 바닥 맞음(가짜 장애물)과 장착 기울기.
- 선회 중 스캔 뒤틀림(⑦b 3.2).
- 실제 차체의 자기 가림 구간과 진동 중의 마스크 경계.
- MCU STATE 와 스캔의 시각 맞춤·오도메트리 단절.
- Jetson 과 노트북의 USB 지연 차이(Jetson 에서 다시 잰다, P0-2).
- 2D 평면 밖 장애물(포크 끝 높이·위 선반) — 층의 전제(3.5)로만 다룬다.
- 허가가 0 으로 바뀐 뒤 실제 정지까지의 물리 체인(S5).

## 3. ⑦b 요구사항 — 비영 허가를 열기 전에 닫을 것

⑦b 는 별도 설계 문서와 독립 검토 3 개를 거친다. 아래가 그 입력이다.

1. **취득 시각 계약**(Codex P1-1): 1.1 의 결과로 스캔의 취득 구간(시작·끝)과 그 불확실성을 정한다. stamp 를 취득 시각으로 쓰지 않는다. 시각 점프는 거부하고 변환을 다시 세운다.
2. **스캔 뒤틀림 여유**(Codex P1-2): 대표 자세 하나에 스캔을 놓을 때의 여유에 병진 `v·δt` 와 회전 `2ρ sin(|ω| δt/2)`, 센서 지레팔을 넣는다(예: δt 0.05 s, v 0.2 m/s, κ 0.367 → ρ 4 m 에서 약 24.7 mm). 실제 최대 취득 기간으로 계산한다. **이 여유를 OCCUPIED 팽창과 FREE 축소 양쪽에 적용**한다 — 지금 `a2_spec` 모형의 `sensor_bound_at()` 은 `sensor_bound_m` 을 쓰지 않고, FREE 의 횡방향 축소는 `sensor=False` 다(값을 0.02 → 0.50 으로 바꿔도 격자가 같았다). 격자 코드에 이 입력을 새로 만든다. 또는 빔마다 오도메트리로 비뒤틀림 보정을 한다.
3. **STATE 측정 시각**(Codex P1-3): `Sample.host_ns` 는 수신 시각이다. MCU `mcu_time_us` 를 호스트 단조 시각으로 옮기는 추정과 그 불확실성 구간을 정의한다(`DelayBound` 는 지연 상한일 뿐 시계 변환이 아니다). 최신 자세부터 판단 시점까지의 오차도 넣는다. 오도메트리 단절 조건에 부팅·간격 초과 외에 엔코더 gap/jump·조향 무효를 넣고, 단절 때 층과 허가 캐시를 함께 버린다.
4. **실차 상대 오차 예산**(Codex P1-4): 필요한 것은 관측부터 지금까지의 짧은 상대 오차다. 시뮬 관절각 재생에서 얻은 나이 오차표는 실차의 조향 보정·백래시·미끄럼(agy)·시각 불확실성을 담지 않는다. 노트북·HIL 용 임시 표와 실차 비영 허가용 표를 나누고, S6 첫 시험의 보수적 오차 예산을 정한다. 기준 설정의 감속 1.5 m/s² 도 그대로 쓰지 않고 S5 정지 실측에 묶는다.
5. **높이 구간과 시작 기억**(Codex P1-5): 기준 설정은 'LiDAR 평면보다 낮은 장애물이 없는 장면'을 전제한다. 실차의 장착 높이·기울기에서 유효한 높이 구간, 검사장 장애물 조건, 기울어진 빔의 `limit_m` 을 정한다. `shadow_band_m` 을 이어받으면 `ShadowMemory` 가 시작 때 관측되지 않은 띠를 비었다고 등록한다. `--forks-clear` 는 그 띠 전체의 공백 확인이 아니므로, 가림 구간과 시작 기억의 운용 전제를 따로 닫는다. 자기 가림 각도 마스크는 UNKNOWN 으로 남겨 보수적으로 한다. 진동 중의 마스크 경계는 S7 에서 확인한다(agy).
6. **시간 문턱의 한 묶음 계산**(Codex P2-6): 층 지연 예산 B(지금 실차 0.10 s), 내부 `sensor_timeout_s`(0.25 s), FREE 사용 나이, 센서 침묵 한도, 정지 지연을 1 의 취득 구간 위에서 함께 계산한다. 칸 나이만 늘리지 않는다. 비차단 계산·설치, 스캔 미도착, 계산 정체를 주입하는 시험을 넣는다.
7. **허가 재사용 대상**(Codex P2-7): `LiveDrivePermission` 은 이미 코어 `DrivePermission` 의 별칭이고, `live_perm.limit()` 객체는 코어의 `BundlePermit` 이다. 다리는 이것을 쓰고, 실제 이동량에 따른 `advance()` 와 오도메트리 단절 때의 캐시 초기화를 함께 옮긴다. 실행기를 건드리면 재생 게이트 29/29 를 거친다. 새 실차 조립·시각·스캔 경로는 별도 통합 시험으로 검증한다.
8. **허가 경로의 정적 판정**(Codex P2-8): 노트북 고정 자세에서 다음을 본다(⑦a 의 원시 기록 재생 포함).
   - 빈 공간에서 비영 허가.
   - 정지 검사 영역 안 장애물이면 허가 0, 치우면 회복.
   - 방향·좌표가 맞음.
   - 스캔 중단·지연이면 정한 기한 안에 0.
9. **격자 창**(Codex): 지금 `refresh()` 는 `hall` 대신 차 주위의 창을 만든다. 고정 범위 제한과 실제 snapshot 경계를 구분해 정한다.
10. **반사·재질**(agy): 검은 무광·투명 랩·경면은 NaN(UNKNOWN)으로 남아 허가를 막는지 시험으로 고정한다. 막지 않는 경우(오인 거리)를 S7 에서 본다.

## 4. ⑦a 의 코드와 시험

| 파일 | 내용 |
|---|---|
| `src/forklift_core/sensors/scan_feed.py` | `scan/1` 해석, `ScanFeed`, `RealScan` |
| `ros2/src/forklift_ros/forklift_ros/real_scan_adapter.py` + 진입점 | 어댑터 노드(버림·계수) |
| `deploy/ros2/Dockerfile` | sllidar_ros2 커밋 고정 빌드 |
| `tools/ros2_dev.sh` | `scan-a2m12 <장치>` 하위 명령(드라이버+어댑터) |
| `tools/scan_check.py` | 1.4 |
| `deploy/udev/99-forklift-lidar.rules` | 연결 뒤 VID/PID 로 채움(초안) |

시험:
- `ScanFeed`:
  - 형식 오류별 버림·계수, 무효 거리 → NaN, 빔 수가 다른 두 스캔.
  - 시각 변환, 음수 지연 거부, `sys_minus_mono` 점프 → 거부와 재수립(정상 흔들림 ±0.3 ms 섞인 값으로 재수립됨, 재수립 중 다시 점프하면 처음부터).
  - 기록 재생: 점프·재수립이 든 기록을 새 `ScanFeed` 에 넣으면 수락·거부 순서가 같음.
  - 비차단(읽을 것이 없을 때 즉시 돌아옴).
- 어댑터: `scan/1` 왕복(ROS 없이 직렬화 함수만), 받는 쪽이 없을 때 버림(막지 않음).
- `scan_check`: 합성 스캔 소켓으로 통계·각도 판정·기록 형식. 늦게 보내는 합성 드라이버로 'stamp → 수신' 꼬리가 드러남.

## 5. 수치 (재계산)
- 전송량: A2M12 카탈로그 표본 16 kHz, 회전 10 Hz → 스캔당 최대 1600 빔(실측 아님, `docs/hardware.md`). f32 1600 × 4 B = 6.4 kB/스캔, 10 Hz 에 64 kB/s(세기 포함 128 kB/s). 빔 수 상한 4096.
- 1판의 '칸 나이 0.2 s < 실제 0.27 s → 늘 0' 은 **다음 갱신 직전의 최대 나이**이지 새 스캔의 첫 사용 나이가 아니어서 일반적으로 성립하지 않는다(Codex P2-6). 문턱은 3절 6 에서 한 묶음으로 다시 계산한다.

## 6. 자체 검토

| 검토 | 지적 | 처리 |
|---|---|---|
| ① 원자료 재계산(Claude) | 시뮬 칸 나이(free 0.2 s)와 실제 지연 비교 | 1판 3.3 에 적음 → ② 가 '늘 0' 은 과장이라고 바로잡음 → 3절 6 의 한 묶음 계산으로 |
| ② Codex 적대 검토(1판) | P1 5: stamp 는 취득 시각 아님·점프는 정지, 회전 뒤틀림과 `sensor_bound_m` 미사용, STATE 측정 시각 없음, 시뮬 오차표·감속 승계, 높이 전제·시작 기억; P2 3: 시간 문턱 묶음, `BundlePermit`·캐시, 노트북 합격 조건 | ⑦a/⑦b 로 나눔. ⑦a 는 스캔 경로와 시각 사실 확인(1.1·1.3), ⑦b 는 3절 요구사항 1–9 로 별도 설계 |
| Codex 2판 재검토 | 새 P1 없음, P2 2: 재수립 조건에 허용폭, 원시 기록의 위치와 재생 경계 | 기준값·±1 ms 중앙값 10 개 재수립·재점프 초기화, 판정 전 수신 프레임 + 판정을 기록하고 재생 순서 일치 시험 |
| ③ agy(Gemini 3.8 Flash, 1·2 를 보지 않은 '적히지 않은 전제') | 스캔 평면 수평·바닥 맞음, 2D 평면 밖 장애물, 각도 규약, USB 지연(`latency_timer`), 미끄럼, 마스크 진동, 반사 재질, 정지 체인; 노트북 시험의 한계 | 각도 규약 확인과 `latency_timer` 기록을 ⑦a 에(1.1·1.4), 나머지는 2절(노트북 한계)·3절 4·5·10 에 |
