# ⑦b — 텔레옵 다리의 스캔 허가 (설계 메모, 학교 규모)

작성 2026-11-16 · 총괄 배정(Jetson 대기 중) · 선행 [⑦a 설계](2026-11-12-real-scan-obstacle-layer.md)(#470) 3 절 요구사항 · 상태 **Codex 검토 전**

## 목적과 범위

- 다리의 바닥 허가 `floor_permit` 을 지금의 `no_scan`(언제나 0)에서 **실제 스캔으로 만든 장애물 층의 허가**로 바꾼다. 이것이 없으면 실차 G6 일정표가 `no_scan` 으로 속도 0 이다.
- 쓰는 곳: 빈 시험장, 빈차, 다리 명령 상한 0.1321 m/s(지금 설정), M1–M3 일정표와 손 텔레옵. 그 밖(적재·자율·좁은 통로)은 이 메모의 범위가 아니다.
- 새 알고리즘을 만들지 않는다. Isaac 라이브 콘솔이 쓰는 경로를 그대로 쓴다: `sim/isaac/obstacle_layer.py` 의 `ObstacleLayer`(Isaac import 없음) → `refresh()` 의 snapshot → `teleop_drive.BundlePermit.limit()`. 러너·장애물 층 코드는 고치지 않는다(바뀌면 재생 게이트 29/29).
- A2M12 없이: ⑦a 의 `scan/1` 형식(sllidar_ros2 고정 빌드, `ScanFeed`)으로 만든 합성 스캔과 `scan_check --record` 기록 형식의 픽스처로 경로를 시험으로 고정한다. A2M12 를 연결한 날에는 `scan_check` 실측 한 번과 다리 정적 확인(아래 시험 ⑤)만 하면 된다.

## 구성

```
ScanFeed(⑦a) ─ RealScan ─┐
MCU STATE ─ RealOdometry ─┴─ ScanPermit ─ ObstacleLayer.grid.add_scan → refresh() → snapshot ─ BundlePermit.limit → floor_permit
```

- 새 파일 `tools/bridge_scan.py` 의 `ScanPermit`(다리가 쓰는 `floor_permit` 호출 가능 객체). 층 모듈은 `tools/permit_bench.py` 처럼 `sim/isaac/obstacle_layer.py` 를 경로로 읽는다.
- 다리(`tools/teleop_bridge.py`) 변경은 셋뿐이다: `--scan-socket <경로> --lidar-mount x,y,yaw_deg`(둘 다 주어야 켜짐, 장착 위치 기본값 없음 — S7 에서 잰다), STATE 마다 `ScanPermit.on_state(sample)`, 주기마다 `ScanPermit.poll(now_ns)`.
- 층 설정은 새 파일 `config/real/obstacle_layer_real.yaml`: `obstacle_layer_single_a2_team_spec.yaml` 에서 시작해 아래 값만 바꾼다. 출처와 '잠정' 표시를 줄마다 단다.

## ⑦a 3 절 요구사항을 학교 규모로 닫는 방법

| # | 요구 | 이 메모의 처리 |
|---|---|---|
| 1 | 취득 시각 | 스캔 시각 = `stamp 단조 시각 − 스캔 주기`(주기는 연속 stamp 의 중앙값, 0.05–0.2 s 로 자름). stamp 는 grab 직전 시각이라 취득은 그보다 앞일 수 있으므로 한 주기 앞으로 당겨 **더 오래된 쪽**으로 둔다. 'stamp → 수신' 0.15 s 초과 스캔은 버린다. 시각 점프는 ⑦a `ScanFeed` 가 이미 거부·재수립한다. |
| 2 | 스캔 뒤틀림 | 상한 속도에서 한 주기 동안의 이동 `v·T`(0.1321 × 0.2 = 26 mm, 회전 몫은 κ 0.36 에서 같은 크기)를 허가의 `envelope_m` 에 더한다: 0.01 → 0.04 m. 격자 코드는 고치지 않는다. |
| 3 | STATE 측정 시각 | 자세는 STATE 의 호스트 수신 시각에 붙인다. 수신 지연(DelayBound 실측 ≤ 28 ms) × 0.1321 m/s = 4 mm 로 2 의 여유 안에 든다. 스캔 자리 자세는 스캔 시각 앞뒤 두 STATE 사이의 선형 보간. 오도메트리 단절(부팅·간격·엔코더 끊김·조향 무효 — `RealOdometry` 의 사유)이면 층과 `BundlePermit` 를 새로 만들고, 새 스캔이 쌓일 때까지 허가 0. |
| 4 | 실차 오차 예산 | 나이 오차표는 `obstacle_odometry_age_team_spec.json`(시뮬)을 **잠정**으로 쓴다. 대신 상한 속도가 작아(0.13 m/s) 0.35 s 동안 46 mm 만 간다. 감속은 S5 정지 실측 전까지 1.5 → 0.5 m/s², 지연 `latency_s` 0.15 → 0.30 s 로 보수적으로 둔다. 실차 표는 S6 첫 시험 기록으로 다시 정한다. |
| 5 | 높이 구간·시작 기억 | 운용 전제: 시험장 바닥에 LiDAR 평면보다 낮은 장애물이 없다(절차서에 한 줄). `shadow_band_m` 은 **끈다**(시작 기억 없음, 엄격 규칙). 자기 가림 각도는 설정의 `mask_deg` 구간 목록으로 받아 그 빔을 NaN(UNKNOWN)으로 둔다 — S7 장착 뒤 채운다. 처음엔 빈 목록이고, 가림 때문에 허가가 0 이면 그것이 도착일에 보인다. |
| 6 | 시간 문턱 한 묶음 | 스캔 주기 T ≈ 0.1 s, 시각 당김 T, 전송 ≤ 0.05 s → 새 스캔의 첫 사용 나이 ≤ 0.15 s, 다음 스캔까지 +0.1 s → 0.25 s. `free_max_age_s`·`free_use_age_s` 0.35 s, `sensor_timeout_s` 0.35 s(> 주기 0.1 + 전송 0.05 + 당김 0.1 + 여유). 스캔이 0.35 s 끊기면 허가 0(시험 ⑥). |
| 7 | 허가 재사용 | `BundlePermit` 그대로. 주기마다 오도메트리 이동량으로 `advance()`, 단절 때 새로 만듦(3). |
| 8 | 정적 판정 | 아래 시험 ①–⑥. |
| 9 | 격자 창 | `hall` = 시작 자세 둘레 ±20 m(오도메트리 좌표), 창 6 m 그대로. 창 밖으로 나가면 허가 0(층이 그렇게 한다). |
| 10 | 반사·재질 | 무효 거리 → NaN → UNKNOWN 이 허가를 막는지 시험 ④로 고정. 오인 거리는 S7 에서 본다. |

차체 외곽은 Isaac 실행과 같은 측정 차체 값(`Footprint(1.29, 0.17, 0.36)`, `dls08_measured`)을 쓰고, 허가 모양은 `hull`(포크 사이도 막힘, 보수적)로 둔다.

## 시험(먼저 쓴다, A2M12 없이)

합성 `scan/1` 프레임(빈 공간 = 먼 벽, 장애물 = 가까운 상자)과 합성 STATE 로 `ScanPermit` 을 돌린다.

1. 빈 공간: 몇 스캔 뒤 전진 허가 > 0.
2. 진행 방향 정지 검사 영역 안 상자 → 0, 치우면 회복. 뒤쪽 상자는 전진을 막지 않는다(방향·좌표).
3. 장착 위치·각도: `--lidar-mount` 의 yaw 180° 이면 앞 상자가 뒤로 간다(각도 규약).
4. NaN 이 진행 방향을 덮으면 0(UNKNOWN 이 막음).
5. `scan_check --record` 형식의 기록 파일을 그대로 재생해 1–2 가 같은 결과(도착일에 실측 기록으로 같은 시험을 돌린다).
6. 스캔 중단 → 0.35 s 안에 0, 'stamp → 수신' 과대 스캔은 버림, 오도메트리 단절 → 0 뒤 회복.
7. 다리 통합: `--scan-socket`·`--lidar-mount` 없이는 여전히 `no_scan`; 하나만 주면 시작 거부; 리허설 `--synthetic-permit` 과 함께 주면 거부.

## 하지 않는 것

비뒤틀림 보정, 실차 나이 오차표, 적재·자율 주행의 허가, Jetson 측정, SLAM. 이 메모의 값은 모두 잠정이고, A2M12 실측(`scan_check`)과 S5·S6·S7 기록으로 다시 정한다.

## 검토

- Codex 1 회(기존 설계 범위 안이라 3 중 독립 검토 생략, 총괄 2026-11-16).
