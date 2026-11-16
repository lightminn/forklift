# ⑦b — 텔레옵 다리의 스캔 허가 (설계 메모, 학교 규모)

작성 2026-11-16 · 3판(Codex 2 회차 반영) · 총괄 배정(Jetson 대기 중) · 선행 [⑦a 설계](2026-11-12-real-scan-obstacle-layer.md)(#470) 3 절 요구사항

## 목적과 범위

- 다리의 바닥 허가 `floor_permit` 을 지금의 `no_scan`(언제나 0)에서 **실제 스캔으로 만든 장애물 층의 허가**로 바꾼다. 이것이 없으면 실차 G6 일정표가 `no_scan` 으로 속도 0 이다.
- 쓰는 곳: 빈 시험장, 빈차, 다리 명령 상한 0.1321 m/s(지금 설정), M1–M3 일정표와 손 텔레옵, 운용자가 주 전원 차단 스위치를 쥔 상태(절차서 1 절). 그 밖(적재·자율·좁은 통로)은 범위가 아니다.
- 새 알고리즘을 만들지 않는다. Isaac 라이브 콘솔의 텔레옵 경로를 그대로 쓴다: 스캔마다 `ObstacleLayer.refresh(stamp, correction, version, current_pose, path_ahead=[current], loaded=False, direction=0)`(러너가 텔레옵 때 넘기는 값 그대로) → `layer.snapshot` → `teleop_drive.BundlePermit.limit()`. 러너·장애물 층·격자 코드는 고치지 않는다(바뀌면 재생 게이트 29/29).
- A2M12 없이: ⑦a 의 `scan/1` 형식(sllidar_ros2 고정 빌드, `ScanFeed`)으로 만든 합성 스캔과 `scan_check --record` 기록 형식의 픽스처로 경로를 시험으로 고정한다. A2M12 를 연결한 날에는 `scan_check` 실측 한 번과 시험장 정적 확인(아래 운용 조건)만 하면 된다.

## 구성

```
ScanFeed(⑦a) ─ RealScan ─┐
MCU STATE ─ RealOdometry ─┴─ ScanPermit ─ grid.add_scan(ObstacleScan) → refresh() → snapshot ─ BundlePermit.limit → floor_permit
```

- 새 파일 `tools/bridge_scan.py` 의 `ScanPermit`(다리의 `floor_permit` 호출 가능 객체). 층 모듈은 `tools/permit_bench.py` 처럼 `sim/isaac/obstacle_layer.py` 를 경로로 읽는다.
- 스캔은 `add_scans()`(고정 빔 각도)가 아니라 `layer.grid.add_scan(ObstacleScan(...))` 로 넣고 `layer.last_scan_s` 를 직접 적는다. 빔 각도는 `RealScan.angles` 그대로, `self_hit` 은 자기 가림 각도 마스크(아래 5), `limit_m` 은 `beam_limits()`(아래 5). 입력 검증(길이·유한성)은 `ScanPermit` 이 한다(`add_scan` 은 길이만 본다).
- 다리(`tools/teleop_bridge.py`) 변경은 넷뿐이다: `--scan-socket <경로> --lidar-mount x,y,z,yaw_deg`(둘 다 주어야 켜짐, 장착 위치 기본값 없음 — S7 에서 잰다; 리허설 `--synthetic-permit` 과 함께면 거부), floor_permit 연결, STATE 마다 `ScanPermit.on_state(sample, delay_s)`, 주기마다 `ScanPermit.poll(now_ns)`.
- 층 설정은 새 파일 `config/real/obstacle_layer_real.yaml`(`obstacle_layer_single_a2_team_spec.yaml` 에서 시작, 바꾼 값마다 출처와 '잠정').

## ⑦a 3 절 요구사항을 학교 규모로 닫는 방법

핵심은 **시각 불확실성을 나이 오차표에 더해 격자의 OCCUPIED 팽창과 FREE 축소 양쪽에 함께 넣는 것**이다. 격자는 점마다 `e(age) + 2ρ sin(ψ(age)/2)` 를 쓰고(`ObstacleGrid.radius_m`, OCCUPIED 와 FREE 띠 모두), ρ 는 뒤축에서 점까지의 거리라 센서 지레팔이 들어 있다. 그래서 오차표의 위치·방향 열 전체에 상수 δ_e·δ_ψ 를 더하면 양쪽이 같이 보수적이 된다. 격자 코드는 고치지 않는다.

- 시각 불확실성 δt = 스캔 한 바퀴 T(빔들의 취득 시각이 퍼진 폭, 비뒤틀림 보정을 안 함) + 취득 시작의 불확실성 T(stamp 는 grab 직전 시각이라 시작은 그보다 앞) + STATE 측정 시각의 불확실성(다리의 지연 상한 `DelayBound`, 실측 ≤ 28 ms → 0.03 s). T = 0.1 s 면 δt = 0.23 s.
- 차가 δt 동안 움직이는 동안 뒤축에서 ρ 떨어진 점(센서 지레팔 포함)이 옮겨 가는 거리는 많아야 `v·δt + 2ρ sin(ω·δt/2)`, ω ≤ v·κ 다. 격자의 반경이 `e + 2ρ sin(ψ/2)` 이므로 e 에 δ_e = v_max·δt, ψ 에 δ_ψ = v_max·κ_max·δt 를 더하면 **점마다 자기 ρ 로 이 두 항을 함께** 덮는다(회전 지레팔 변위가 ψ 항에 들어 있음, Codex 2 회차 P1).
- v_max 는 다리가 overspeed 로 세우기 전의 속도 한계 `limit_mps`(지금 0.2 m/s, 명령 상한 0.1321 보다 큼)를, κ_max 는 조향 지도의 물리 최대 곡률 `max|cfg.steering.curvatures_inv_m|` 를 코드에서 읽는다(명령 상한이나 묶음 여유보다 크거나 같음, Codex 2 회차 P2). 지금 값으로 δ_e = 0.2 × 0.23 = 46 mm, δ_ψ = 0.2 × κ_max × 0.23(κ_max 0.367 이면 16.9 mrad, ρ 4 m 에서 68 mm).
- 오차표는 0.1 s 부터 10 s 까지 있고 `at()` 은 다음 행 값을 돌려주므로 0.35 s 까지의 나이는 모두 값이 있다. `ScanPermit` 은 시작 때 표의 마지막 나이가 `max(free_use_age_s, occupied_max_age_s)` 이상인지 확인한다. 표 증분은 격자 반경(OCCUPIED 와 FREE 띠)만 바꾸고, `DrivePermission`·`BundlePermit` 의 나이 제한은 바꾸지 않는다.

| # | 요구 | 처리 |
|---|---|---|
| 1 | 취득 시각 | 스캔의 층 시각 = `stamp 단조 시각 − T`(T 는 연속 stamp 간격의 중앙값, 0.05–0.2 s 로 자름). 남는 불확실성은 위 δt 로. 'stamp → 수신' 0.15 s 초과 스캔은 버린다. 시각 점프는 ⑦a `ScanFeed` 가 거부·재수립한다. |
| 2 | 스캔 뒤틀림 | 위 δt 의 T 몫으로, 오차표를 통해 OCCUPIED 와 FREE 양쪽에(Codex P1-1: `envelope_m` 은 경로 여유만 키워 대신이 못 됨 → 쓰지 않음). |
| 3 | STATE 측정 시각 | STATE 자세는 `RealOdometry`(MCU 시각 순서로 적분)의 결과를 `host_ns − DelayBound` 시각에 붙인다. 측정 시각은 [host_ns − DelayBound, host_ns] 안이고 그 폭이 δt 에 들어 있다. 스캔 자리 자세는 그 시각축에서 앞뒤 두 STATE 의 선형 보간(스캔이 가장 오래된 STATE 보다 앞이면 버림). |
| 3' | 오도메트리 단절 | `RealOdometry` 의 `continuity_broken`(부팅·간격·엔코더 끊김·조향 무효, 첫 표본의 조향 무효 포함)이면 층과 `BundlePermit` 를 새로 만들고 STATE 자세 이력을 비운다. 새 기준점 뒤의 스캔이 snapshot 을 만들 때까지 허가 0. `time_not_increasing` 은 끊김이 아니라 그 표본만 건너뛴다. 주기마다 적분 이동량으로 `BundlePermit.advance()`. |
| 4 | 실차 오차 예산 | 오차표 바탕은 `obstacle_odometry_age_team_spec.json`(시뮬) **잠정**에 위 δ 를 더한 것. 감속은 S5 정지 실측 전까지 1.5 → 0.5 m/s², `latency_s` 0.15 → 0.30 s. 실차 표는 S6 첫 기록으로 다시 정한다. |
| 5 | 높이·기울기·자기 가림 | `limit_m` = `beam_limits(장착 높이 z, 빔 방향을 최대 기울기 θ_max 로 기울인 값, band_top/bottom)` — θ_max 는 설정값(잠정 1°), `band_top_m`·`band_bottom_m` 은 장착 높이 ± 그 기울기로 계산. `may_clear` 는 아래 운용 조건 아래에서만 참이다. 자기 가림은 설정 `mask_deg` 구간의 빔을 `self_hit` 로 표시(격자가 그 빔을 쓰지 않음) — S7 장착 뒤 채우고, 처음엔 빈 목록. 가려진 구간이 차체 둘레 띠(정지호·외곽이 FREE 여야 하는 곳)에 걸리면 허가는 그 방향으로 늘 0 이다(시험 3: 뒤쪽 좁은 가림은 후진만, 옆 뒤 40° 가림은 전진까지 닫음). 그래서 장착 위치는 차체 둘레가 보이는 곳이어야 하고, 이것을 S7 에서 먼저 확인한다. `shadow_band_m` 은 끈다(시작 기억 없음). |
| 6 | 시간 문턱 | 새 스캔의 첫 사용 나이 ≤ T(당김) + 0.05(전송) ≈ 0.15 s, 다음 스캔 직전 ≈ 0.25 s. `free_max_age_s`·`free_use_age_s`·`sensor_timeout_s` 0.35 s. `sensor_timeout_s` 는 침묵이 0.35 s 를 **넘은 뒤** 0 이다(시험이 경계 양쪽을 본다). |
| 7 | 허가 재사용 | `BundlePermit` 그대로(3' 의 advance·초기화). |
| 9 | 격자 창 | `hall` = 시작 자세 둘레 ±20 m(오도메트리 좌표, 보정 없음 = 항등 `correction`, `version` 0), 창 6 m. |
| 10 | 반사·재질 | NaN → UNKNOWN 이 허가를 막는 것은 시험으로 고정. **틀린 유효 거리**(거울·유리·검은 무광의 오인)는 소프트웨어로 못 막는다 → 아래 운용 조건. |

**모양.** 차체는 Isaac 실행과 같은 측정 차체 값 `Footprint(1.29, 0.17, 0.36)`(`dls08_measured`), 날개는 `blades_rear_m = ()`(날개 목록 없음)으로 두어 `footprints()` 가 전·후진 모두 `(unloaded_shape, body)` 를 쓰게 한다 — 검사 모양은 외곽 전체(포크 사이도 막힘, 보수적), 자기 외곽(own outline)은 차체. 적재는 범위 밖이라 `loaded=False` 고정.

## 운용 조건 (비영 허가의 전제 — 총괄 확인 필요)

비영 허가는 아래가 모두 맞을 때만 쓴다. 절차서에 한 묶음으로 넣는다.

1. 빈 시험장, 빈차, 상한 0.1321 m/s, 운용자가 주 전원 차단 스위치를 쥔다.
2. 시험장에 LiDAR 평면(장착 높이 ± θ_max 의 띠)보다 낮은 장애물, 거울·유리·투명 랩·검은 무광 물체가 없다.
3. 도착일 첫 주행 전에 정적 확인: 차를 세운 채 사람이 상자를 들고 진행 방향 정지 영역에 들어가면 허가가 0 이 되고 나가면 돌아오는지 본다(`scan_check --record` 로 같이 기록, 시험 ⑤ 로 재생).

이 조건으로 남는 위험(오인 거리, 평면 밖 물체)은 낮은 속도·운용자 차단으로 받아들인다. 받아들일지는 총괄이 정한다.

## 시험(먼저 쓴다, A2M12 없이)

합성 `scan/1` 프레임(빈 공간 = 먼 벽, 장애물 = 가까운 상자)과 합성 STATE 로 `ScanPermit` 을 돌린다.

1. 빈 공간: 몇 스캔 뒤 전진 허가 > 0. 스캔별 각도가 그대로 격자에 들어감(빔 수가 다른 두 스캔).
2. 진행 방향 정지 영역 안 상자 → 0, 치우면 회복. 뒤쪽 상자는 전진을 막지 않는다.
3. 장착 yaw 180° 면 앞 상자가 뒤로 간다(각도·좌표 규약). `mask_deg` 안의 빔은 쓰지 않는다(`self_hit`).
4. NaN 이 진행 방향을 덮으면 0.
5. `scan_check --record` 형식 기록을 재생해 1–2 가 같은 결과.
6. 시각: 침묵 0.35 s 직전은 열림·넘으면 0; 'stamp → 수신' 과대 스캔 버림; 가장 오래된 STATE 보다 앞선 스캔 버림; δ 를 더한 오차표가 `limit_mps`·조향 지도 최대 곡률로 계산되고, 격자 반경이 ρ 에서 원래 반경 + δ_e + (2ρ sin((ψ+δ_ψ)/2) − 2ρ sin(ψ/2)) 만큼 커짐; 표가 0.35 s 를 못 덮으면 시작 거부.
7. 오도메트리 단절 → 즉시 0, 옛 snapshot·허가 캐시를 다시 쓰지 않음, 새 스캔 뒤 회복; `advance()` 가 이동량만큼 캐시를 진행.
8. 다리 통합: 인자 없으면 `no_scan`; 하나만 주면 시작 거부; `--synthetic-permit` 과 함께면 거부.

## 하지 않는 것

빔별 비뒤틀림 보정(δt 로 대신), 실차 나이 오차표, 적재·자율 주행의 허가, Jetson 측정, SLAM. 값은 모두 잠정이고 A2M12 실측과 S5·S6·S7 기록으로 다시 정한다.

## 검토

| 회차 | 지적 | 처리 |
|---|---|---|
| Codex 1 | P1 3: 뒤틀림을 `envelope_m` 으로 대신할 수 없음, STATE 수신 시각 ≠ 측정 시각, 높이·오인 전제 미흡; P2 4: 시간 문턱 경계, refresh 입력과 호출 정책, 모양 설명 틀림, 단절 처리·advance; 시험 빈틈 | 시각 불확실성 δt 를 오차표 양쪽 열에 더해 OCCUPIED·FREE 양쪽에(envelope 안 씀), STATE 를 `host_ns − DelayBound` 에 붙이고 그 폭을 δt 에, `beam_limits`·θ_max·마스크·운용 조건(총괄 확인), 경계 시험, refresh 를 러너 텔레옵과 같은 인자로, 모양을 `blades=()` 로 명시, 단절·advance 규칙과 시험 추가 |
| Codex 2 | P1 1: 회전 지레팔 변위가 표 증분에 들어 있는지 근거 부족; P2 2: 표 나이 범위 확인, κ_max 를 실제 곡률 상한에서 | 점 이동 상한 `v·δt + 2ρ sin(ωδt/2)` 와 격자 반경식의 대응을 적음(시험 6 에 반경 확인 추가), v_max 를 overspeed 한계 0.2 m/s 로, κ_max 를 조향 지도 물리 최대로, 표 범위 시작 검사 |

기존 설계 범위 안이라 3 중 독립 검토는 생략한다(총괄 2026-11-16).
