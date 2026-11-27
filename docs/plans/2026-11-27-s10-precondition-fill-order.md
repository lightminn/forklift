# S10 선행 조건 31 건 — 채우는 순서 (2026-11-27)

- 상태: 참고표. 총괄 요청으로 만들었고, Codex 적대적 검토 4 회차에서 '새 P1/P2 없음'. 판정 코드는 `src/forklift_core/live/real_preconditions.py`, 계획은 [운영 REAL 조립 해제](2026-11-26-real-operating-assembly.md) 3·4 절.
- 실 하드웨어 작업은 사용자 지시로 보류 중이다. 이 표는 재개 때의 순서이고, 측정값이 아니다.

기준: `feat/bench-a1a2 c3d6ea5d`. S10 판정 코드와 기본 설정은 origin/main 68bee19f 와 같다. `operating_preconditions` 를 config/real 기본 파일로 실행하면 31 건이 나온다.

분류는 셋이다.
- **측정**: 시운전 단계 S2–S8, 7a-3·7r 벤치에서 잰 값.
- **결정**: 총괄 결정 기록(`decision_id`).
- **설정**: 사람이 설정·기록 파일에 쓰는 줄.

'분'은 계획 4 절이 정한 새 손작업(날 A–D, 합 45 분)만 적었다. 나머지는 기존 단계에 포함된 작업이고, 현장 시간은 따로 추정해야 한다.

| # | 거부 사유 | 채움 | 손작업 한 줄 | 선행 |
|---|---|---|---|---|
| 1–3 | C1 board `no_state`·`no_usb_product`·`no_usb_serial` | 보드 사실(기록 아님) | 최종 운영 이미지를 굽고 USB 로 연다. 같은 fd 의 첫 STATE·제품 문자열·일련번호를 읽는다. 빌드 종류·출력 잠금·면제·`-dirty` 도 여기서 판정한다 | 최종 빌드(18–23·25–30·16 의 보드 상한 확정 뒤) |
| 4–6 | C5 `first_beam_err_s`·`basis`·`applies_to` | 측정 → 결정 → 설정 | 기존 7a-3 벤치 진단 15 분. SDK 모형으로 공학 상한을 내고, 한쪽 11 ms 안이면 총괄이 결정한다. 공학 상한은 두 진단 결과와 SDK 모형·`latency_timer`·전송 시간 가정으로 낸다. `first_beam_err_s` 수치·`basis: engineering`·`decision_id`·저장소 안의 `basis_doc` 를 쓰고, `applies_to` 여섯 키(image_id=Docker ID, usb, host, ans_type, sample_period_us, baudrate)에 그 실행의 실제 값을 쓴다 | Jetson 에 arm64 패치 스캔 이미지 반입(빌드 경로는 있음, 반입은 미확인) |
| 7 | C5 `mount_unmeasured` | 측정 | S7: 뒤축 기준 `mount.xyz`·`yaw_deg` 를 줄자로 재고 `self_mask_deg` 를 쓴 뒤 `measured: true` | 최종 차체에 LiDAR 고정. **S6 보다 먼저** |
| 8 | C5 `scan_load` | 측정 | 31 과 같은 부하 벤치에서 실 스캔 재측정. 진단값이 상한의 절반을 넘으면 4 가 무효다(31 의 HIL 표본과 따로 적음). binds: `lidar_sha256`·`scan_image` | 4–7 |
| 9 | C5 `scan_coverage` | 측정 | S10 통로(`corridor_half_width_m: 0.10`)를 빔 높이와 가림 마스크가 덮는지 본다. binds: lidar 해시·`start_rear`·`check_pallet_pose`·`map_id`·`inserted_offset_m` | 4–7·차량 보정 확정 → 조사 → `map_id`·`start_rear` 기입 → 등록(C6) → 9 |
| 10 | C2 `s5_safety` | 측정, `pass` 만 | 최종 이미지에서 S5 를 한다. Jetson 정지·주 전원 차단·포크 거동은 기존 단계, 링크 단절(**최대 유효시간 DRIVE 직후** 단절 포함)·DISARM 축약 재확인은 날 B 10 분. binds: 보드·제품 문자열·`stop_files` | 날 A. **정지 설정 파일 확정본 위에서**(아래 무효 사슬) |
| 11 | C2 `iwdg_evidence` | 측정, `pass` 만 | 두 시험을 합친 근거다. 잠금 `fw-stall` 로 제어 주기 정지 → IWDG 리셋을 보고, `fw-open` 받침대에서 외부 리셋 → 출력 차단을 오실로스코프로 본다. **53.8 ms + 1 ms + 실측 리셋→차단 ≤ 100 ms** 여야 `pass`. binds: 최종 커밋·보드 | 최종 커밋 확정, 최종 굽기 **전** |
| 12 | C3 `fault_detection` | 측정 | 최종 이미지에서 엔코더 고장 주입 1 회와 입력 나이 초과 정지를 본다(날 A 5 분). binds: 보드·제품 문자열 | 1–3 |
| 13 | C4 `port_review` | 결정 | `approved_limited` 에 `decision_id`·속도 상한·`[A]` 를 쓴다. 실측 해결은 S8 | 없음 |
| 14 | C9 `s10_speed` | 측정 + 결정 | 시험 속도는 **S5 바닥 정지 시험을 통과한 속도**를 사건 직전 STATE 로 확인한 값이다. 텔레옵 속도는 근거가 아니다. 판정식은 `s10a.max_speed_mps ≤ 승인 ≤ 시험`이고, C4 제한 승인이 있으면 `s10a.max_speed_mps` 도 그 상한 이하. `[A]` 필수 | S5(최종 이미지) |
| 15 | C10 `start_taped` | 측정 | 바닥 시작점을 줄자로 재 `start_rear` 와 맞춘다(PR 7 벤치 7). 팔레트 치수(날 C 5 분)·`check_pallet_pose` 는 S10-A 설정에 따로 쓴다 | 장소 결정 |
| 16 | C11 `lift` | 측정 + 결정 | 날 C 10 분. 시험 팔레트를 받침 없이 들고 주 전원 차단 60 s 뒤 처짐을 줄자로 잰다(가장 불리한 높이·목표 높이). 바닥·목표 높이에서 줄자 대 STATE 를 잰다. 처짐 한도는 총괄이 정한다(`sag_limit_decision_id`). 목표 높이 `hold` 행은 늘 필요하다. 가장 불리한 높이는 그 행에 `worst: true`·`worst_reason` 을 쓰거나, 저장소 안의 전 구간 자잠김 근거를 `self_locking_evidence` 로 지정한다. binds: 보드·목표 높이·승강률·팔레트 ID·질량·기구(바뀌면 다시 잼) | **결정 항목(실 하드웨어 재개 때)**: 기본값이 이미 어긋난다 — `autonomy.yaml` `lift_rate_mps` 0.06 m/s > 보드 `lift_rate_max` 0.02 m/s. 둘 중 하나를 최종 굽기 전에 정한다. 승강 측정 안(3.5 A/B). 승강 시운전 이미지를 쓰면 최종 굽기 전 |
| 17 | C12 `firmware_build` | 설정 | 최종 이미지를 판독해 sha256 을 사람이 대조한다(날 A 5 분). `product`·보정 식별자·`readback_sha256`·`board_lift_rate_max_mps` 를 쓴다 | 1–3 |
| 18–19 | C12 `wheel_counts.left/right` | 측정 | S2: ABZ 해상도를 먼저 확인한다. 손으로 N(≥ 10) 바퀴 돌려 카운트 ÷ (CPR × N) = 감속비 | L·R MT6701 장착(지금 미장착) |
| 20 | C12 `wheel_radius` | 측정 | S8: 바닥 직진 거리 줄자 대 카운트 | 18–19, S6 |
| 21 | C12 `steering_map` | 측정 | S2 에서 끝점·영점을 재고, S8 조향각–곡률 자료로 완성한다 | 조향 KTH 기구 장착·범위 확인(판독은 11-27 회복, 점퍼 간헐 끊김 남음) |
| 22 | C12 `wheel_rate_limit` | 측정 | S3: 바퀴 띄우고 무부하 최대 속도 | 모터 출력 빌드(V1.0 IPROPI 는 위험 수락 기록) |
| 23 | C12 `steering_control` | 측정 | S4 조향 PI 의 오차·지연으로 변화율과 허용치 셋을 정한다 | S2 조향 끝점·영점, S3 |
| 24 | C13 `stop_model` | 설정 | 확정된 정지 설정 해시·lidar 해시로 한 줄(날 D) | 25–30·4–7 확정 |
| 25 | C13 budget `latency_budget` | 측정 또는 결정 | S5 에서 층 지연을 잰다(29 의 정지 지연과 별개, 실행기가 둘을 더함) | Jetson 운영 스택 + LiDAR |
| 26 | C13 budget `overdue_end` | 결정 | 정책값 1.0 s. 총괄 예외가 현실적이다(`until: S8`) | 없음 |
| 27 | C13 watch `current` | 측정 | S3 전류 측정으로 문턱을 정한다 | S3 |
| 28 | C13 watch `wheel` | 측정 | S8 바퀴 추종 오차로 허용치를 정한다 | S8 |
| 29–30 | C13 layer `latency_s`·`decel_mps2` | 측정 | S5: 정지 반응 시간, 바닥 정지 거리 → 감속도 | 초기 S5(바닥) |
| 31 | C14 `jetson_load` | 측정 | 7r: 원천·브리지·실행기를 같은 Jetson 에서 돌려 50 Hz 틱·STATE 50 ms·관문 25 ms 를 본다. 표본은 **잠정** ≥ 500(분모는 관문에 닿은 정상 DRIVE 후보). 공회전과 HIL 은 나눠 적는다. 코드는 status·근거·날짜·`stop_files` 만 본다 | 자격 있는 실 관측 기록, 7b IMU 지연 상한(기본 `imu: required`), 7, 4, 지도·시작점, 정지 설정 확정. 결과는 실제로 도달한 분기까지만 |

**순서.** 세 덩어리로 간다.
1. **보정 단계**: S2(18·19·21 끝점) → S3(22·27) → 조향·승강 센서와 전류 입력이 정상 무장 조건 충족(승강 센서 연결 포함) → S4 면제 정상 이미지(23) → S7 LiDAR 장착(7) → S5 시험 구역·허용 정지 거리 결정 → 초기 S5(받침대·첫 바닥 정지, 29·30·25) → 실 스캔으로 텔레옵 허가 확인 → S6 바닥 텔레옵 → S8(20·21 완성·28).
2. **최종 이미지**: 승강 속도(16)·보드 상한 확정 → IWDG 두 시험(11) → 최종 빌드·굽기 1 회(1–3) → 날 A(17·12) → 최종 S5(날 B: 10·14, 초기 S5 에서 얻은 29–30 근거 재확인. 이때 정지 설정을 고치면 무효 사슬이 적용된다).
3. **장소와 기록**: C11(16, 날 C) → 장소·지도(15·9) → 7r 부하(31·8) → 날 D(24, C2–C14 기록 binds 점검).

**무효 사슬.** `stop_files` 는 파일 전체의 해시다. `obstacle_layer_real.yaml`·`obstacle_layer.yaml`·오도메트리 나이 표·`vehicle.yaml`·`safety_watch.yaml` 가운데 하나라도 바뀌면 **10·24·31 이 함께 무효**다. 25–30 의 provenance 를 고치는 것도 바뀜이다. 그래서 설정을 먼저 확정하고 그 확정본 위에서 10·31 을 기록한다. `lidar.yaml` 이 바뀌면 8·9·24 와 C6 지도 정합이 무효다. 차량 `calibration_id`·차체 URDF 가 바뀌어도 C6 이 무효다. 다시 구우면 C1·C2·C3·C11·C12 를 다시 확인한다.

**기록 한 줄.** 18–23·25–30 의 측정 경로는 해당 설정에 값과 `provenance.measured: true` 를 쓴다(C5 장착은 `mount.measured`, 시간 상한은 4–6 의 `timing.basis` 형식). 예외는 `{file, key, decision_id}` 다(29–30 의 file 은 `layer`). `s10_preconditions.yaml` 의 항목에는 `status`·저장소 안 `evidence`·`date`·`binds` 가 공통이다.

**결정만으로 풀리는 것.** 13 과 26 뿐이다. 4–6 은 15 분 진단 뒤 결정한다. provenance 12 건(18–23, 25–30)은 키별 총괄 예외(`config_exceptions`)로도 풀리고, 그러면 31 → 19 건이다. 다만 수치 유효성 검사는 계속 적용되고, 정지·한계의 근거라 측정을 권한다.

**하드웨어 현황**은 다음과 같다. 사용자 지시로 실 하드웨어 작업은 보류 중이다.
- 조향 KTH: 판독은 회복됐고, 점퍼와 기구 장착이 남았다.
- 승강 센서: 연결돼 있지 않다(RJ2 종단 미실장).
- L·R MT6701: 장착돼 있지 않다.
- Jetson arm64 스캔 이미지: 반입됐는지 확인하지 못했다.

## 실 하드웨어 재개 때 결정할 것

- C11 승강 속도: `autonomy.yaml` `lift_rate_mps` 0.06 m/s 와 보드 `lift_rate_max` 0.02 m/s(`firmware/boards/weact_h743/src/board_config.c`) 중 어느 쪽을 바꿀지. 최종 굽기 전에 정한다(총괄 2026-11-27).
