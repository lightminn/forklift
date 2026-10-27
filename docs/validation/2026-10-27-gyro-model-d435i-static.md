# D435i 자이로 정지 실측 모형 — T2 C1 팔 준비

- 날짜: 2026-10-27
- 계획: [`docs/plans/2026-10-27-insertion-lateral-drift.md`](../plans/2026-10-27-insertion-lateral-drift.md) T2 의 C1(탐색 비교)
- 산출물: `config/gyro_model_d435i_static_20261023.yaml`, `config/obstacle_odometry_age_team_spec_gyro_d435i_static.json`, `config/obstacle_layer_single_team_spec_gyro_d435i_static.yaml`, `tools/fit_gyro_static.py`, riib 런처 `GYRO=measured`

## 무엇을 했나

- **원자료.** 2026-10-23 책상 정지 기록 run1.
  - 장치: D435i serial 261822074539, firmware 5.15.1.55.
  - 자이로 400 Hz, motion correction off, 공장 IMU 보정 없음(내장값 단위행렬). 온도 기록은 비어 있다.
  - 보관본 `gdrive:forklift-archive/laptop-downloads-20261027/d435i_imu_static_20261023.tar.zst` 을 ws1 `/home/projects/forklift/data/d435i_imu_static/2026-10-23/` 에 풀었다.
  - 3,175,962 표본, 7,922 s, 400.8 Hz, 10 ms 넘는 틈 0.
- **시간축.** 장치 `frame_timestamp`(µs)는 32 bit 라 2³² µs(4,295 s)마다 되감긴다. 도구는 이것을 풀어서 쓴다. 처음 실행에서 이것을 놓쳐 길이 4,295 s·표본률 734 Hz 로 잘못 나온 것을 고쳤다(시험 있음).
- **맞춤 방법**(`tools/fit_gyro_static.py`, 책상 검토 5.2 의 지시대로 데움 구간과 안정 구간을 나눔). 각 값이 무엇에서 나왔는지는 모형 파일의 값마다 주석으로 적었다.
  - 안정 구간(t ≥ 1,800 s)의 겹침 Allan 편차:
    - 잡음 밀도: τ 0.5–5 s 에서 σ√τ 의 중앙값
    - 편향 불안정성: 최솟값 / 0.664
    - 편향 걷기: τ ≥ 최솟값 τ 구간에서 σ√(3/τ) 의 **최댓값**(보수 상한). 최솟값으로 잡으면 긴 τ 의 추정 산포 때문에 참값 아래로 떨어질 수 있어 최댓값을 쓴다(합성 시험으로 확인).
  - 데움: 분 평균에 b(t) = 안정값 + A·e^(−t/τ) 를 맞췄다.
  - LSB: 표본 값 사이의 가장 작은 간격.

## 숫자

| 축 | 안정 편향 °/s | 잡음 밀도 °/s/√Hz | 편향 불안정성 °/h (τ s) | 걷기 상한 °/s/√s | 데움 A °/s, τ s | 맞춤 잔차 °/s |
|---|---|---|---|---|---|---|
| x | −0.1877 | 0.00732 | 4.66 (236) | 9.7e-5 | −0.0312, 960 | 0.0014 |
| y | −0.2473 | 0.01177 | 6.57 (191) | 1.52e-4 | +0.0318, 451 | 0.0028 |
| z | +0.0949 | 0.00775 | 9.47 (291) | 1.77e-4 | +0.0031, 883 | 0.0035 |

LSB 는 세 축 모두 0.1 °/s 다(motion_xyz32f 출력). 가정 모형의 0.0305 °/s 보다 거칠다.

**모형 값**(세 축 공통 한 값이라 가장 나쁜 축):

| 항목 | 값 |
|---|---|
| `noise_dps_rthz` | 0.01177 (y) |
| `initial_bias_dps` | 0.2473 (y, 실행마다 U(±x)) |
| `temperature_dps` / `temperature_s` | 0.0318 / 451.4 (y — 차가운 시작 기울기 \|A\|/τ 7.0e-5 °/s² 가 가장 큰 축) |
| `random_walk_dps_rts` | 0.000177 (z) |
| `lsb_dps` | 0.1 |
| `scale`·`range_dps`·`mount_error_deg`·`zupt_*`·`gap_s` | 가정 모형과 같음(측정 아님: 정지 기록에는 회전이 없고, 장착·알고리즘 설정은 이 기록과 무관) |

가정 모형(`gyro_model_spec_typical`)과의 비교:
- 잡음: 0.014 → 0.0118.
- 온도 경사: 0.15 °/s / 1000 s(1.5e-4 °/s²) → 7.0e-5 °/s².
- 걷기: 0.0005 → 0.000177.
- 초기 편향 범위: ±1 → ±0.25 °/s.

모형의 선형 경사는 0 에서 시작해 끝없이 커지지만, 실측 편향은 계단처럼 시작해 가라앉는다. 그래서 미션 몇 분 안에서는 실측 변화를 위에서 막고, τ 를 넘으면 과장한다. 모양은 모형의 것이지 기록의 것이 아니다.

## 나이표

- **생성:** `tools/p0a_measure.py age --odometry-heading gyro --gyro-model config/gyro_model_d435i_static_20261023.yaml --draws 5`.
- **입력:** 가정 모형 나이표와 같은 V-G2 team-spec 조향 순회 seed 0–2.
  - 원래 경로 `test_runs/live/vg2/steer_s*` 는 ws1 정리로 지워져 있었다.
  - 보관본 `gdrive:forklift-archive/ws1-cleanup-20261022/live_vg2.tar.zst`(md5 일치)에서 ws1 `test_runs/c1_gyro_fit/inputs/` 로 그 세 실행만 풀었다. 그래서 나이표의 `runs` 경로가 이 위치를 가리킨다.
- **riib 실행은 입력으로 쓸 수 없다.** 자이로 빌드 러너는 첫 정지 편향까지 차를 붙잡아 두는데, 그 전에 움직인 기록은 재생이 거부한다(`the truck moved at 0.450 s before the first standstill bias`).
- **재현 확인:** 같은 입력·같은 명령에 가정 모형을 넣으면 기존 `config/obstacle_odometry_age_team_spec_gyro.json`(b5f3831d)과 숫자·잡음 기록이 그대로 같다.
- **결과 — 누적 상한, 후축:**

  | 나이 | 위치 | yaw |
  |---|---|---|
  | 5 s | 0.0105 m | 0.0138 rad |
  | 10 s | 0.0341 m | 0.0266 rad |

  가정 모형은 10 s 에서 0.042 m·0.033 rad 다. 정지 편향 갱신은 순회마다 10 / 254 / 259 회였다.
- **시험:** 나이표는 모형 SHA-256 `fd0a7999…` 에 묶여 있다. 실행기 대조(`odometry_table_problem`)가 통과하는 것을 `tests/unit/test_gyro_model_d435i_static.py` 가 고정한다. **모형 파일을 고치면 나이표를 다시 만들어야 한다.**

## C1 팔의 실행 인자

riib 런처: `GYRO=measured`(기본 off). 조건은 MODE=slam, ENCODER_MODEL=team_spec_quantized, NOISE=const 이고, 다른 조합은 거부한다. 붙는 인자는 다음과 같다.

```
--obstacle-layer config/obstacle_layer_single_team_spec_gyro_d435i_static.yaml
--odometry-heading gyro --gyro-model config/gyro_model_d435i_static_20261023.yaml
```

`riib_defaults.txt` 에 `GYRO=` 가 남는다.

## 이것이 덮지 않는 것

- **G4 를 채우지 않는다.** 정지·책상 조건이라 주행 진동, 마스트 승강, 온도 기록, 회전(감도·교차축), 차량 위 모터 전원·제자리 조향이 없다(`docs/design/2026-10-22-d435i-imu-desk-review.md` 5).
- **편향 불안정성(flicker)을 담을 필드가 없다.** 걷기 상한이 긴 τ 에서 그것을 대신한다.
- **결과의 쓰임.** C1 결과는 "정지 실측 모형에서의 비교"로만 읽는다. 자이로 헤딩 채택 근거가 아니다(사용자 결정 대기).
- **기록 한 개다.** 장치 한 대, 2.2 h, 반복 기록이 없다.
