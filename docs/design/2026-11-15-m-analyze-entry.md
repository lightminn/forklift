# G8 입구 — `tools/m_analyze.py` 와 `forklift_core.plant.m_entry` (D4) — 설계 (2026-11-15)

- 출처: [도착일 리허설 기록](../validation/2026-11-15-arrival-day-rehearsal.md) D4(13 행), [M1–M3 등록](../plans/2026-11-15-m1-m3-registration.md)(초안 개정 2) 1–6 절, [프로필 구동·M 분석](2026-11-14-profile-drive-and-m-analysis.md) 3 절, [리허설 설계](2026-11-15-arrival-day-rehearsal.md) 2.3·2.5.
- 상태: 초안 개정 1. Codex 적대 검토 1 회차와 agy 전제 검토를 반영했다(8 절). 같은 Codex 스레드의 재검토("새 P1/P2 없음")는 아직 받지 않았다.
- **PR 나누기:** ① 공유 데이터 모델과 입력 검증(모든 스키마, 신원 검사, void/판정 불가 틀), `m_analysis` 변경(4 절), CLI, M1, 그리고 M2·M3 가 나중에 채울 결과 계약(5 절). `m_synth` 와 그 적대적 경계 사례도 ① 에 넣는다. ② M2 ③ M3(합류·q 계단). 각 PR 은 시험부터 쓰고, 머지 전에 ws1 전체 시험을 거친다.

## 1. 목적·범위

- **D4 가 고치는 것:** `m_analysis.py` 에 계산 함수는 다 있지만, 기록을 읽어 그 함수들에 넣어 주는 입구가 없다. 리허설에서는 세션 스크립트가 기록 행 → `McuState`(`record_rows.py:22-37`), IMU 행 → `imu_stream`(`sensors/imu_record.py:92-122`), 서 있는 구간·`quiet_window`·시행 구간·참값을 손으로 이었다. 그 일을 라이브러리 `src/forklift_core/plant/m_entry.py` 와 CLI `tools/m_analyze.py` 가 맡는다. 시행마다 등록 3.3·4.3·5.4 의 값을 낸다.
- **세션 스크립트에서 드러난 빈틈**(도구가 되풀이하지 않게 적는다):
  1. `s_m` 을 시행으로 거르지 않고 max 로 골랐다.
  2. 엔코더 거리를 `distance_source="truth"` 로 넘겼다.
  3. `quiet_window` 를 5/3 s, 0.01(`m_analysis.py:21`)로 불렀다. 등록 2.2 는 60/50 s, 0.025 다.
  4. `up=(0,-1,0)` 을 코드에 박았다.
  5. `broken` 을 보고만 하고 void 로 만들지 않았고, 결함 비트도 보지 않았다.
  6. 책상 N 과 3.0 s 창을 상수로 넣었다.
  7. 참값을 시행 번호로만 짝지었다.
- **범위 밖:**
  - 문턱 L·H, 순서통계량 구간, +5 회. 이것은 여러 기록을 합치는 판정 도구가 맡는다(설계 3 절 "G8 은 문턱을 박지 않는다"). 이 도구는 시행별 값과 그 값을 판정할 수 있는지만 낸다.
  - G9 방법 선택, G4, G3 배율 계산(등록 2.6). 지금은 배율이 없다고 보고, 배율이 필요한 값은 판정 불가로 낸다.

## 2. 입력과 신원 검사

```
tools/m_analyze.py --record TELEOP.jsonl --imu IMU.jsonl --profile config/profiles/m1_m3/<name>.yaml
                   (--truth TRUTH.jsonl | --no-truth) (--mount G3.json | --up X,Y,Z)
                   [--field-log FIELD.jsonl] [--vehicle config/real/vehicle.yaml] [--trial N] [--allow-synthetic]
```

- **파일마다 머리 줄**
  - 기록·IMU·참값·현장 기록, 네 파일 모두 첫 행이 `t:"header"` 이고 아는 스키마여야 한다. 아니면 거부한다.
  - `refuse_or_mark` 를 **파일마다** 부르고(`live/synthetic.py:78-85`), 하나라도 표지가 나오면 출력 표지를 전파한다. 이 함수는 `t=="header"` 행만 본다(79 행). 그래서 머리 줄이 없는 파일은 합성 판정을 빠져나간다. 머리 줄을 필수로 둔 이유다.
  - 머리 줄에 표지가 없어도 장치가 pty 인 기록은 합성으로 본다(68-75 행).
  - 합성 참값 하나만 섞여도 `--allow-synthetic` 없이는 거부한다(시험이 있다).
- **텔레옵 기록**
  - 머리 줄(`teleop_bridge.py:402-404`), `state` 행, `profile_start`·`profile` 행(`drive_profile.py:384-394`), 꼬리 줄(446-449)을 읽는다.
  - STATE 행은 받은 STATE 마다 하나씩 쓰인다(`_on_state` 223-230). 주기는 MCU 의 `state_period_ms` 이고, 다리 주기 `CYCLE_S` 20 ms(54 행)와 상관없다. 결정이 가정한 10 ms 는 G1 기록으로 확인 필요다.
  - 측정 종류는 일정표 `name` 의 접두사로 정한다(`m1_`·`m2_`·`m3_merge_`·`m3_qstep_`). 그 밖은 거부한다.
- **IMU**(`imu_record/1`): `imu_stream(rows, stream="g"|"a")` 로 읽는다. 한 파일에 boot 나 호스트 시계가 둘이면 거기서 거부된다(116-119 행). 가속도는 자이로 시각에 선형 보간한다.
- **장착 축:** `--mount` 는 G3 출력의 두 방향 평균 û_ω 다. `--up` 은 손으로 주는 벡터이고 출력에 `up_source:"argument"` 로 남는다. 기본값은 없다.
- **참값 `truth/1`**
  - 머리 줄: `{"t":"header","schema":"truth/1","method":"A_floor_marks"|"B_camera"|"synthetic","synthetic":bool,"record":{"host_boot_id","start_mono_ns"},"body_point":"rear_axle_center","frame":"floor","sigma":{"pose_yaw_rad","pose_xy_m","source"}}`
  - 시행 행: `{"t":"truth","trial":N,"points":{"<id>":{"kind","x_m","y_m","yaw_rad",["host_ns"],["cov"]}}}`
  - 점은 정해진 id 와 그 종류로만 받는다.
    - `start`·`end` 는 `kind:"stop_pose"` 로, 시작·끝 정지 자세다. `cov`(x, y, yaw 의 3×3)가 없으면 머리 줄 σ 를 쓴다. A 의 σ 는 그날의 반복성 검사로 정한다(등록 6 절 415 행).
    - `merge_end`·`run_042` 는 `kind:"moving_point"` 다(B 카메라). 표본 시각 `host_ns` 와 점마다의 `cov` 가 **필수**다.
    - 모르는 id, 종류가 맞지 않는 id, 필수 필드가 없는 점은 거부한다.
  - 같은 시행 행이 둘이거나 기록에 없는 시행의 행이 있으면 거부한다. `record`·`body_point` 가 다를 때도 거부한다. 몸체 점은 `RealOdometry` 의 뒤축 중심이어야 하기 때문이다.
  - 현장 void 행: `{"t":"void","trial":N,"reason","closed_before_results":true}`. 등록 1.1(71 행)은 현장 void 를 결과 계산 전에 닫는다고 정했다. 그래서 결과를 계산하기 전에 이 행만 읽고, `closed_before_results` 가 참이 아니면 거부한다.
- **현장 기록 `field_log/1`**(선택)
  - 머리 줄(`synthetic`, `record:{"host_boot_id","start_mono_ns"}` — 텔레옵 기록과 대조) 뒤에, 시각 순으로 `{"t":"bias_recheck","block","exceeded_host_ns","recheck_start_ns","recheck_end_ns","slope_rad_s2","passed":bool}` 행이 온다.
  - 등록 2.3 의 10 분 재확인 결과를 블록별로 남긴다(3.3 ⑦). 각 재확인 행은 그 블록의 뒤 시행 시작보다 앞서야 한다. 신원이 다르거나 시각 순이 아니거나 이 조건을 어기면 exit 2(9 절).
- **옛 `rehearsal_truth/1`**(`rehearsal_imu_truth.py:94-99`): `--allow-synthetic` 일 때만 읽고 heading 만 있는 참값으로 다룬다. 거리는 엔코더로 표시한다.

**신원 검사**(값을 계산하기 전에 한다; 실패는 `{"result":"refused","why"}` 와 exit 2 로 낸다. `bringup.py:941-947` 과 같은 방식이다)

| 검사 | 실패 |
|---|---|
| 기록·IMU·참값·현장 기록의 `host_boot_id` 가 같다. 참값 `record.start_mono_ns` 가 기록 머리 줄과 같다 | 거부 |
| 거부되지 않은 `profile_start` 행의 `profile`(`teleop_bridge.py:178`)이 `--profile` 의 `name` 과 같다. label 집합이 같다. drive 구간 `req_mps` 의 부호·크기가 일정표 속도와 같다. label 집합만으로는 같은 측정군의 파일을 가르지 못한다(m1 4 개, m2 16 개가 각각 같은 집합) | 거부 |
| 텔레옵 꼬리 줄이 마지막 행이고 `dropped == 0`, `write_errors == 0` 이다(`record_writer.py:126-127`) | 거부(7 절) |
| 시행을 덮는 MCU `ClockFit.valid`(`clock_map.py:96-123`). 품질 지표(`drift_split_ppm`·`drift_vs_halves_ppm`·`delay_ns`·`dropped`)는 출력한다 | 거부 |
| 시행 창 안에서 MCU `boot_id` 가 한 값이다 | 시행 void `mcu_boot_changed` |
| IMU `profile` 행: `options.enable_motion_correction.read_back == 0`(`tools/imu_record.py:50,59-67`), `rate_mismatch` 거짓(269 행). 꼬리 줄: `stop` ∈ {seconds, interrupt}, `error` 없음, `dropped == 0`, `write_errors == 0`(296-301 행) | 모든 시행의 IMU 항목 `imu_record_unclean` |
| `ImuStream.domains` 가 한 값이다 | IMU 항목 `imu_domain_mixed` |
| IMU fit 이 valid 이고, 시행의 IMU 구간이 fit 하나 안에 있으며 외삽이 아니다. IMU 호스트 시각이 [앞 창 탐색 시작, 뒤 창 탐색 끝]을 덮는다 | `imu_clock_invalid`·`imu_clock_break`·`imu_coverage` |
| 차량 보정 id: 기록 머리 줄에 없다. `--vehicle` 의 `calibration_id`(`real_vehicle.py:136`)와 sha256 은 출력만 한다 | 확인 필요(7 절) |
| G3 장착 파일의 날짜·serial 을 IMU 머리 줄 `serial` 과 대조하는 것 | 확인 필요 |

## 3. 계산

### 3.1 등록 상수

`m_entry.REGISTERED` 한 곳에 모으고 등록 판 이름을 붙인다. `m_analysis` 의 기본값은 바꾸지 않고 등록값을 인자로 넘긴다.

| 상수 | 값 | 등록 |
|---|---|---|
| `quiet_window` max_s / min_s, gyro_tol / accel_tol | 60 / 50 s, 0.025 rad/s / 0.15 m/s² | 2.2(160·163·169 행) |
| 시행별 \|Δb_yaw\| 급변 | 1.5×10⁻⁴ rad/s | 2.3 (a) |
| 편향 변화율. 직전 6 개 편향 구간의 이동 맞춤 | 3.5×10⁻⁷ rad/s² | 2.3 (b)(177·180 행) |
| 책상 N | 2.054×10⁻⁴ rad/√s | 1.1 표 28 행 |
| 상대 예산 | heading 변화 1.0 mrad, 상대 위치 1.7 mm | 6 절 413-414 행, 6.1–6.3 |
| 차체 위 N 의 k 한계 | 0.9 m 합류 끝 1.48, 0.9 m + 주행 중 0.42 m 점 1.14 | 6.3 표(525-532 행) |
| 배율 없이 허용하는 회전 | 0.04 rad | 1.1(79 행) |
| 가속 램프, 평활 창, `pre_s` / `frac` | 0.125 s, 중심 25 ms, 0.5 s / 0.63 | 4.3·5.1·5.2(371 행) |

### 3.2 시계 — 호스트 단조 ns 하나

- **STATE:** `clock_fits_from_rows(rows)`(`read_ns`, `clock_map.py:167-188`)의 (host boot, MCU boot) 별 fit 으로 `mcu_time_us` 를 호스트 ns 로 옮긴다. `ClockFit` 에는 장치 → 호스트 방향만 있다(117-123 행). 그래서 호스트 시각을 MCU 시각으로 옮기는 계산은 하지 않는다.
- **IMU:** `imu_clock_fits`(125-133 행)로 dev_us 를 옮긴다. 그 전에 unwrap 한 dev_us 가 마지막으로 남긴 값 이하인 표본(늦은 표본·중복)을 `clean_points` 의 규칙(28-51 행)대로 빼고 수를 센다. `imu_stream` 은 기록 순서 그대로 돌려주는데, `imu_heading` 은 시각이 엄격히 늘지 않으면 예외를 낸다(128 행).
- `profile` 행과 참값 `host_ns` 는 이미 호스트 ns 다. `step_response_delay` 는 **ns** 입력을 받는다(`pre_s * 1e9`, 261 행).

### 3.3 모든 시행에 공통인 순서

1. 2 절 검증. `trials(rows, valid_only=False)`(`m_analysis.py:41-59`)로 void 도 사유와 함께 센다.
2. `states = [sample_from_row(r).state ...]` 를 만들고, `odometry_pose`(4 절)를 기록 전체에 한 번 돌린다. 시각은 3.2 로 호스트 ns 에 놓는다.
3. 시행 창 [t₀, t₁] 은 첫·끝 `running` 행이다. 주행 구간은 일정표에서 `kind=="drive"` 인 label 이다.
   - **움직임 시작:** 주행 구간이 시작된 뒤 바퀴 카운트가 처음 바뀐 STATE 의 시각(MCU → 호스트)이다. 주행 구간은 deadman 이 눌릴 때까지 제자리에서 기다리므로(`drive_profile.py:353-357`) 구간 시작과 다르다. 등록이 이 정의를 고정하지 않았다(확인 필요).
4. void 판정(3.4). void 시행은 값을 내지 않는다.
5. **서 있는 구간:** 두 바퀴 속도가 문턱 아래이고, 조향 |Δq| 가 문턱 아래(presteer 의 조향 슬루를 뺀다)이며, STATE 간격이 `max_gap_s`(0.05 s, 9 절) 이하인 연속 호스트 구간이다. 바퀴 속도·조향 문턱은 확인 필요다.
   - `quiet_window` 는 걸러 낸 표본의 인덱스 연속성만 보고 `sel[s]:sel[e-1]+1` 을 돌려준다(83-87·122 행). 그래서 사이에 낀 비정지 표본까지 들어갈 수 있다.
   - 이것을 고친다(4 절). 입구에서도 돌려받은 창 안에 간격이 없고 비정지 표본이 없는지 다시 확인한다.
6. **편향 창:**
   - 앞 창은 첫 주행 구간 시작에서 `side="before"`, 뒤 창은 마지막 주행 구간 끝에서 `side="after"` 로 찾는다. 등록 상수를 넘긴다(64-122 행).
   - 하나라도 None 이면 IMU 항목은 `no_bias_window` 다.
   - Δb_yaw 는 û 성분으로 낸다. `bias_between` 의 3 축 노름은 보조값이다.
   - |Δb_yaw| > 1.5×10⁻⁴ 이면 `bias_jump` 다.
7. **편향 변화율**(등록 2.3 (b), 183 행):
   - 같은 IMU 파일에서 이 시행 앞뒤 창을 포함한 직전 6 개 편향 구간의 중심 시각에 대해 Δb_yaw 를 직선 맞춤하고, 기울기를 블록·시각과 함께 낸다.
   - 기울기가 문턱 안이면 IMU 항목을 판정할 수 있다.
   - 넘으면 그 시행은 `bias_slope` 다.
   - 같은 블록에서 넘은 시각 뒤의 시행은 `field_log` 에 그 블록의 `passed:true` 재확인이 있을 때만 판정할 수 있다. 재확인이 있으면 이동 맞춤은 그 재확인 구간부터 다시 센다. 재확인 기록이 없으면 `bias_slope_unconfirmed` 다.
   - 파일 안의 구간이 3 개 미만이라 기울기를 낼 수 없을 때도 `field_log` 통과 기록이 없으면 `bias_slope_unconfirmed` 다.
8. **차체 위 N**(6.3, 531 행):
   - 두 창의 û 투영 rate(창마다 편향을 뺀 것)로 `allan_dev_tau(t_ns, rate, 1.0)` 를 **실제 시각**으로 낸다.
   - 각 창의 값을 내고, **두 창 각각**이 k 한계 안일 때만 자이로 항목을 값으로 낸다(등록 6.3, 9 절). 합친 값은 정보로만 낸다.
9. **IMU heading:** `imu_heading(t[창0 끝 : 창1 시작], g, up=û, bias=두 창 평균)`(125-137 행).
   - 하한은 `heading_sigma_lower(arw=N_body, drive_s=적분 길이, bias_window_s=짧은 창, bias_change_rad_s=Δb_yaw)`(140-152 행)다. 책상 N 으로 낸 값도 나란히 적는다.
   - `is_lower_bound` 와 `not_included` 를 그대로 옮긴다.
10. **상대 자세:**
    - 정지 자세의 오도메트리는 그 정지 구간 안의 STATE 에서 읽는다(첫 움직임 직전, 마지막 주행 뒤 `still_post`). 그 사이에 카운트 변화가 없음을 확인한다.
    - 주행 중 점은 오도메트리 x, y, yaw 를 그 `host_ns` 에 선형 보간하고, 양옆 STATE 와의 시각 차 `odo_dt_ns` 를 낸다. 간격이 `max_gap_s` 를 넘으면 `odo_interp_gap` 이다.
    - Δ = R(−θ₀)(p₁ − p₀), Δθ = atan2(sin(θ₁ − θ₀), cos(θ₁ − θ₀)) 를 참값과 오도메트리 모두에 쓴다.
    - **예산은 상대량으로 검사한다**(등록 6.1–6.3, 9 절): 두 자세의 3×3 공분산(자세끼리 독립)을 (Δ, Δθ) 의 야코비안으로 전파하고, 그 결과의 횡 성분 σ ≤ 1.7 mm, heading σ ≤ 1.0 mrad 를 본다. 등록의 √(σ_θ0² + σ_θ1²)·√(σ_y0² + σ_y1² + (|Δp|·σ_θ0)²) 는 공분산이 대각·독립일 때의 특수형이다(시험 `test_relative_budget_jacobian`). 넘으면 `truth_over_budget` 이다.

### 3.4 void 와 판정 불가

**공통 시행 void**(등록 1.1, 68-72 행; 값을 내지 않고 사유별로 센다)
- G6 결과 ≠ `done`.
- 창 안 STATE 에 결함 비트 `ENCODER_GAP|ENCODER_JUMP_L|_R|ENCODER_STUCK_L|_R|ENCODER_POLARITY_L|_R|STEER_SENSOR_INVALID` 가 있음(`messages.py:77,89-95`; 엔코더 일곱 비트는 `real_teleop._SPEED_BITS` 29-31 행과 같다).
- 창 안 오도메트리 `broken`, 또는 `time_not_increasing`. 이것은 `continuity_broken` 으로 올라오지 않는다(`real_vehicle.py:328-329`).
- `mcu_boot_changed`, 현장 void 행.
- **참값 파일은 있는데 이 시행 행이 없거나, 이 측정·방법에 필요한 점이 없음: `truth_missing`**(제안, 총괄 확인 필요).
- **`--no-truth`(참값을 쓰지 않음)는 다르다.** 시행은 유효하고, 참값에 기대는 값만 모두 `no_truth` 로 판정 불가다. 거리는 encoder 로 표시한다.

| 측정 | 값 | 판정 불가(시행은 유효) |
|---|---|---|
| M1 | d, 끝 횡 오차(보고), `radius_scale`(보고) | `no_truth`, `truth_over_budget` |
| M1 | IMU Δθ(보고) | `no_bias_window`, `bias_jump`, `bias_slope`(`_unconfirmed`), IMU 파일·시계 사유(2 절) |
| M2 | `gain_measured_q`, `gain_commanded_q`, Δg 입력 | `no_truth`, `truth_over_budget`, `rotation_too_small`, `meas_q_missing`(arc 의 meas_q 가 모두 None) |
| M2 | `under_reached`(보고) | `meas_q_missing`. None 비율은 늘 낸다 |
| M2 | IMU κ(보고) | `no_gyro_scale`(회전 ≥ 0.068 > 0.04, 2.6) + IMU 사유 |
| M3 합류 | 판정점의 e_θ, e_y | `no_truth`, `truth_over_budget`, `odo_interp_gap`(B) |
| M3 합류 | 주행 중 점의 IMU heading(A) | IMU 사유, `imu_noise_over_limit`(N_body > k·책상 N), `lower_bound_over_budget`, `no_gyro_scale`(순회전 > 0.04) |
| M3 q 계단 | 엔코더 지연 | 참값 불필요. `step_response_delay` 가 None 이면 그 사유로 **지연만 void**(5.6) |
| M3 q 계단 | IMU 지연(보고) | IMU 사유. 늘 `unknown_clock_offset:true` |

### 3.5 M1(등록 3.3)

- Δθ_odo 와 Δθ_true 는 `start`·`end` 정지 자세에서 잰다. s_true = |Δp_true| 다(직진이라 현을 쓴다).
- **`s_enc_path_m`** 은 시행 창 안에서 적분한 오도메트리 걸음 |Δp| 의 합, 곧 엔코더가 잰 **경로 길이**다. 현 |Δp_odo| 는 변위 `disp_enc_m` 으로 따로 낸다.
- `radius_scale = s_true / s_enc_path_m` 다. 그 시행의 행으로만 거른 일정표 `s_m` 은 교차 확인값일 뿐이다. `s_m` 은 구간 끝에서 멈추고(`drive_profile.py:293`) 감속 3.3 mm 를 빠뜨린다.
- `m1_drift(odo_dyaw, heading_ref_dyaw=Δθ_true, ref_source="truth", distance_m=s_true, distance_source="truth")` 를 부르고, 끝 횡 오차 Δp_odo,y − Δp_true,y 를 낸다.

### 3.6 M2(등록 4.3; PR ②)

- **원호 식:** Δθ = Δθ_true(정규화한 값), c = |Δp_true|, s = c / sinc(Δθ/2) 다(sinc x = sin x / x, |x| < 1e-4 이면 급수).
  - |Δθ| < θ_min(제안 0.01 rad, 확인 필요; 가장 작은 조건은 0.068 rad)이면 `rotation_too_small` 이다.
  - direction 은 일정표 속도의 부호다.
- `kappa_measured_q` 는 arc 구간의 `running` 행 가운데 움직임 시작 + 0.125 s 뒤, None 이 아닌 `meas_q` 의 평균을 `held_curvature` 로 바꾼 값이다. `kappa_commanded_q` 는 같은 행의 `drive_q` 에서 낸다.
- `m2_gains(heading_ref_dyaw=Δθ_true, ref_source="truth", distance_m=s, direction=±1, ...)` 를 부른다.
- **보고값:**
  - `meas_q` 의 흔들린 폭, `drive_q − meas_q` 의 중앙값, presteer 가 다가온 방향.
  - `under_reached`: 행에 없으므로 `max` 일정표에서 다시 계산한다. **arc 구간에서 움직임이 시작된 뒤의 `running` 행만** 보고, \|meas_q − q(κmax)\| ≤ 0.02 를 한 번도 만나지 않았는지를 본다. 실행기도 주행 중에만 이것을 본다(376-378 행). presteer 를 넣으면 늘 거짓이 된다. `meas_q` 가 None 인 행의 비율을 함께 낸다.

### 3.7 M3 합류(등록 5.4·5.5; PR ③)

- **판정점은 방법에 따라 갈린다.**
  - B 이면 `run_042`(moving_point)가 판정점이 될 자리다. 다만 0.42 m 를 무엇으로 재는지 정해지기 전까지는 `judgeable:false, why:"run_042_definition_pending"` 인 보고값으로만 낸다(9 절). `end` 정지 자세와 `merge_end` 도 보고값이다.
  - A 이면 `end`(cont 끝 정지 자세)를 판정점으로 하는 **별도 규칙**으로 낸다. 출력에 `rule:"A_substitute_cont_end_stop", substitute_pending_lead:true` 를 붙인다(5.1, 총괄 승인 대기).
  - e_θ = Δθ_odo − Δθ_true, e_y 는 횡 성분, e_x 는 보고만 한다.
- **IMU heading(A, 주행 중 점):** 움직임 시작부터 그 점까지 적분한다. T 는 그 **지속 시간**(점 시각 − 움직임 시작)이고, 기록에서 잰다.
  - N_body ≤ k·책상 N 이고 그 N 으로 낸 하한 ≤ 1 mrad 일 때만 값이다.
  - 2.1 m 합류 끝이 판정 불가인 것은 따로 박지 않고 이 계산에서 나와야 한다.
- B 의 `run_042` 가 어느 거리(엔코더냐 참이냐)로 0.42 m 인지는 G9 와 총괄이 정한다(확인 필요). 분석기는 같은 `host_ns` 의 자세끼리만 비교한다.

### 3.8 M3 q 계단(등록 5.2; PR ③)

- q 는 STATE `steering_rad` 다. y 는 `encoder_yaw_rate(wheel rates, radius_m, track_m)`(185-190 행)를 25 ms 중심 평활한 값이다. 둘 다 3.2 로 호스트 ns 에 놓는다.
  - STATE 가 10 ms 이면 창(±12.5 ms)에 표본이 2–3 개뿐이다. 등록의 10 표본은 400 Hz IMU 기준이다.
  - MCU `wheel_rate` 자체의 필터 지연도 모른다. 엔코더 y 의 평활 방식은 확인 필요다.
- **지연은 측정 q 의 출발(문턱 넘음)에서 잰다. 호스트 명령 시각에서 재지 않는다.**
  - `step_response_delay(q_t, q, y_t, y, ...) − step_response_delay(q_t, q, q_t, q, ...)` 가 t₆₃(y) − t₆₃(q) 다(249-290 행). 두 항이 같은 출발을 쓰므로 출발 시각은 지워진다. 단위는 ns 입력, 초 출력이다.
- `profile` 행의 호스트 시각은 사후 창을 놓는 데만 쓴다.
  - 시작은 `step` 구간에서 `req_kappa` 가 목표로 바뀐 첫 행 + `q_step_ramp_s`(175 행) + `Q_STEP_SETTLE_S` 1.0 s(31 행)다.
  - 끝은 그 구간의 마지막 `running` 행이다.
- IMU 지연도 같은 식으로 계산하되 보고만 한다(`unknown_clock_offset:true`).

## 4. `m_analysis.py` 변경(PR ①)

**(가) 참값 heading 이름**(등록 3.3 261 행, 4.3 314 행)
- `m1_drift(*, odo_dyaw, heading_ref_dyaw=None, ref_source=None, imu_dyaw=None, distance_m, distance_source)` 로 바꾸고, `m2_gains` 도 같은 방식으로 바꾼다. 두 함수는 이미 키워드 전용이다(`*`, 215·226 행).
- **새 경로:** `heading_ref_dyaw` 와 `ref_source ∈ {"truth","imu"}`. 결과는 `{rad_per_m, odo_minus_ref_rad, ref_source, distance_m, distance_source, note}` 이고 `odo_minus_imu_rad` 는 없다(`ref_source=="imu"` 여도 없다).
- **별칭 경로:** `imu_dyaw` 만 주면 `ref_source="imu"` 로 두고 옛 키 `odo_minus_imu_rad` 를 남긴다. 결과는 옛 dict 에 `ref_source:"imu"` 만 더한 것이다.
- **배타:** 둘 다 주거나, 둘 다 없거나, `imu_dyaw` 와 `ref_source` 를 같이 주거나, `ref_source` 값이 틀리면 ValueError 다.
- `m2_gains` 결과에도 `ref_source` 를 담고, "imu" 이면 note 에 "gyro scale needed" 를 붙인다. 판정 가능 여부는 입구가 정한다(`no_gyro_scale`).
- **확인된 소비자:** `tests/unit/plant/test_m_analysis.py:76-91,144-149`. 78 행은 dict 전체를 대조하므로 `ref_source` 가 들어간 새 dict 로 고친다.
  - 이 검토는 Read 만 써서 grep 을 돌리지 못했다. 구현 첫 단계에서 `grep -rn 'm1_drift\|m2_gains\|odo_minus_imu_rad' src tools tests` 로 다시 센다.

**(나) 자세·순수 함수**
- `odometry_pose(states, cfg, *, max_gap_s)` 를 새로 두고 `{"mcu_time_us","x","y","yaw","broken","reasons"}` 를 돌려준다(`RealOdometry.update(...).pose`; 179 행은 [2] 만 쓴다). `reasons` 에 `time_not_increasing` 도 남긴다. `odometry_heading` 은 이 함수의 껍데기로 바꾸고 반환 키는 그대로 둔다.
- `relative_pose(p0, p1) -> (dx_fwd, dy_lat, dyaw_wrapped)` 와 `relative_truth_sigma(cov0, cov1, dp) -> (sigma_dyaw, sigma_lat)` 를 더한다.
- `allan_dev_tau(t_ns, rate, tau_s)`: 실제 시각으로 사다리꼴 적분한 각을 τ/10 격자에 선형 보간한 뒤 겹침 Allan 편차를 낸다. 균일 dt 를 가정하지 않는다(실측 400.8 Hz).
- `quiet_window`: 걸러 낸 표본이 원래 인덱스에서 이웃이 아니거나 표본 간격이 문턱을 넘으면 연속 구간을 끊는다.

## 5. 출력 JSON(`m_analyze/1`, 표준 출력 한 줄) — PR ① 이 정하는 계약

```json
{"synthetic_inputs": true, "not_for_judgement": true, "schema": "m_analyze/1", "result": "ok", "measurement": "m1",
 "registration": {"doc": "docs/plans/2026-11-15-m1-m3-registration.md", "revision": "초안 개정 2", "status": "draft",
                  "constants": {"quiet_max_s": 60, "...": "..."}},
 "inputs": {"record": {"path", "sha256", "host_boot_id", "footer"}, "imu": {"path", "sha256", "serial", "profile_ok", "footer"},
            "truth": {"path", "sha256", "schema", "method"} | null, "field_log": {...} | null,
            "profile": {"path", "name", "sha256"}, "vehicle": {"path", "sha256", "calibration_id"},
            "up": {"vector": [...], "source": "g3|argument"}},
 "clock": {"domain": "host_monotonic_ns", "mcu_fits": [summary...], "imu_fits": [summary...], "imu_dropped": {...}},
 "counts": {"trials": 6, "valid": 5, "void": {"void:deadman": 1}},
 "trials": [{"trial": 1, "status": "valid", "result": "done", "window_ns": [t0, t1], "move_start_ns": ...,
             "distance": {"source": "truth", "s_true_m": 3.001, "s_enc_path_m": 2.996, "disp_enc_m": 2.996,
                          "s_m_profile": 3.000, "radius_scale": 1.0017},
             "truth_budget": {"sigma_dyaw": ..., "sigma_lat": ..., "ok": true},
             "values": {"d_rad_per_m": {"value": -0.0004, "judgeable": true},
                        "end_lateral_m": {"value": 0.0012, "judgeable": true, "report_only": true}},
             "imu": {"why": [], "dyaw_rad": ..., "windows": {...}, "bias_change_yaw_rad_s": ...,
                     "bias_slope": {"value", "block", "n_windows", "recheck"}, "n_body": {"before", "after", "pooled_info"},
                     "sigma_lower": {...}, "report_only": true}}]}
```

- 합성 표지(`OUTPUT_MARKS`, `synthetic.py:22`)를 맨 앞에 둔다.
- void 시행은 `status:"void", why` 만 적는다. 판정 불가 항목은 `{"value": x|null, "judgeable": false, "why": [...]}` 로 적는다.
- M2·M3 는 같은 `values` 아래에 자기 키를 채운다. 사유 어휘는 3.4 표가 전부다.
- exit code:
  - 0: 출력을 썼다(void·판정 불가가 섞여 있어도).
  - 2: 거부(2 절의 신원 검사, 합성인데 플래그 없음, 모르는 스키마, `--truth`·`--no-truth` 둘 다 없음). 거부일 때는 `{"result":"refused","why"}` 만 낸다.
- 라이브러리 입구는 `m_entry.analyze(rec_rows, imu_rows, truth_rows|None, field_rows|None, profile, cfg, up, *, allow_synthetic) -> dict` 다. CLI 는 파일을 읽고 sha256 을 계산하고 exit code 를 정하는 일만 한다.

## 6. 시험(먼저 쓴다; 실행은 ws1 pytest)

**`m_synth`(PR ①)**
- 구현 상태(2026-11-15, 3c8dd760): PR ① 에는 M1 에 필요한 조절값만 넣었다. 반경 배율, 드리프트, 기울어진 up, 편향과 편향 계단, 적대적 옵션, void 가 그것이다.
- 나머지 조절값은 쓰는 PR 에서 더한다(Codex 구현 검토 P2).
  - `gain`·`q_offset`(조향 이득·추종 오프셋)은 PR ②(M2)에서 더한다.
  - `delay_s`(yaw 응답 지연)는 PR ③(M3 q 계단)에서 더한다.
  - `bias_slope`(편향 변화율)는 여러 시행이 한 IMU 파일에 이어지는 블록 생성기와 함께 더한다. 직전 6 구간 맞춤은 시행 하나로는 만들 수 없다. **편향 변화율 검사(3.3 ⑦)와 함께 PR ③ 으로 옮겼다**(2026-11-16). 이 검사는 IMU 항목을 판정에 쓸 수 있는지만 정한다. 그런데 IMU 를 판정에 쓰는 측정은 M3 주행 중 점뿐이다(등록 6.1·6.3). M1·M2 의 IMU 는 보고값이다. 그래서 PR ① 의 IMU 블록은 `report_only` 이고, 기울기 검사는 그 블록에 결과를 내지 않는다.
- `rehearsal_imu_truth.py:56-79` 의 IMU 잡음·편향 생성을 옮긴다. 그 도구는 기존 기록에서 IMU·참값만 만들 뿐 STATE·`profile` 행은 만들지 않는다. 그래서 STATE·`profile` 생성은 새로 쓴다.
- `profile` 행은 `ProfileRunner.step` 에 합성 관측을 넣어 실행기 자신이 쓰게 한다(D3 은 손으로 만든 행이 놓친 결함이었다). STATE 행은 `state_row`(`record_rows.py:13`)로 쓴다.
- 조절할 수 있는 참 plant 값: `true_radius_scale`, `true_yaw_per_m`, `gain`, `q_offset`, `delay_s`, `up`, `bias`, `bias_step`, `bias_slope`, `arw`, `still_s`, `void`, `seed`.
- 적대적 사례: dev_us 감김·늦은·중복 표본, MCU–IMU 사이 알 수 없는 지연 오프셋, deadman 대기, 참값 yaw ±π 감김, STATE 간격, 조향 슬루 중 정지.
- **머리 줄 `synthetic` 은 기본 참이다.** 거부 시험만 명시적으로 끈다.

| 이름 | 준비 | 단언 |
|---|---|---|
| `test_m1_recovers_drift_and_radius` | 드리프트 0.003 rad/m, 반경 1.02 | d = −0.003(±1e-6), `radius_scale` = s_true/s_enc_path = 1/1.02(엔코더가 1.02 배 과대 계상 — 정의대로), 횡 ≈ d·s²/2 |
| `test_s_enc_path_vs_chord` | 드리프트가 큰 시행 | `s_enc_path_m` > `disp_enc_m`, 배율은 경로로 |
| `test_m1_imu_with_tilted_up` | 숙임 0.10 rad, 편향 3e-4 | IMU Δθ 가 하한의 3 배 안. 숙임을 무시하면 cos 배 |
| `test_relative_budget_camera_numbers` | 등록 6.2 ①③: σ_θ 0.68/0.68, σ_y 0.57/0.57 mrad·mm | 1.32 m 통과(1.21 mm), 3 m `truth_over_budget`(2.19 mm), 정지–주행 0.68/0.74 → 1.005 > 1.0 mrad. ② 3 m(0.34, 0.39)는 1.16 mm 로 통과 |
| `test_truth_yaw_wrap` | θ₀ = 3.13, θ₁ = −3.13 | Δθ = 0.023 |
| `test_void_trials_excluded` | `void:deadman`, 엔코더 간격, `ENCODER_JUMP_L`·`STUCK_R`·`POLARITY_L`·`STEER_SENSOR_INVALID`, `time_not_increasing`, MCU boot 변경 | 모두 값 없음, 사유별로 세어짐 |
| `test_truth_missing_vs_no_truth` | 참값 파일에 시행 1 만 / `--no-truth` | 시행 2 `void:truth_missing` / 모든 시행 유효, 값 `no_truth`, encoder 표시 |
| `test_truth_rows_dup_extra_refused` | 같은 시행 행 둘, 없는 시행 행 | exit 2 |
| `test_field_log_identity_and_order` | `field_log` 의 `record` 가 다름 / 행이 시각 역순 / 재확인이 뒤 시행 뒤 | 셋 다 exit 2 |
| `test_truth_moving_point_needs_time_cov` | `run_042` 에 `host_ns` 나 `cov` 가 없음 | exit 2 |
| `test_field_void` / `..._not_closed_refused` | `t:"void"` 행 / `closed_before_results` 없음 | void(`field:...`) / exit 2 |
| `test_header_required_per_file` | 머리 줄 없는 참값 | exit 2 |
| `test_synthetic_truth_alone_refused` | 기록·IMU 는 표지 끔, 참값만 합성 | 플래그 없으면 exit 2. 있으면 표지 전파 |
| `test_pty_header_unmarked_refused` / `test_synthetic_marked_with_flag` | pty 장치, 표지 없음 / `--allow-synthetic` | exit 2 / 맨 앞에 표지 |
| `test_identity_refusals` | boot 다름, `start_mono_ns` 다름, `body_point` 다름, 일정표 이름 다름(label 같음: `m2_q008_left_fwd` 대 `_rev`), `req_mps` 부호 다름, 텔레옵 꼬리 줄 없음 | 각각 exit 2 |
| `test_imu_record_unclean` | `dropped` 1, motion correction 1, `rate_mismatch`, `stop:"stalled"` | IMU 항목 `imu_record_unclean`, 시행 유효 |
| `test_imu_late_dup_samples_dropped` | 늦은·중복 dev_us | 예외 없이 빼고 수를 냄 |
| `test_imu_clock_break_and_coverage` | 시행 중간에 fit 경계, IMU 가 창보다 짧음 | `imu_clock_break`, `imu_coverage` |
| `test_quiet_window_gap_and_steer` | 정지 사이 5 s STATE 결측, presteer 슬루 | 창이 결측·슬루를 넘지 않음(`quiet_window` 자체 시험 포함) |
| `test_quiet_window_absent` / `test_bias_jump` | `still_s` 5 / `bias_step` 2e-4 | `no_bias_window` / `bias_jump`. 둘 다 시행 유효 |
| `test_bias_slope_field_log` | 기울기 초과; 뒤 시행; 블록 통과 기록 있음·없음 | `bias_slope`; 기록 없으면 `bias_slope_unconfirmed`, 있으면 판정 가능 |
| `test_allan_irregular_t` | 400.8 Hz, 지터, 백색 N | σ(1 s) ≈ N(±5 %) |
| `test_allan_false_rejection` | 50 s 창 둘, 참 N = 책상 N, 시드 여럿 | k = 1.14·1.48 에서 창별 판정의 오거부율을 낸다(정보 — 판정 규칙은 등록이 정한다, 9 절) |
| `test_profile_label_mismatch_refused`, `test_registered_constants`, `test_s_m_per_trial` | 앞 판과 같다 | 앞 판과 같다 |
| `test_rename_paths` | 새 경로(truth·imu), 별칭, 배타 | 두 경로의 반환 dict 전체를 대조, ValueError |
| `test_odometry_pose_*` | 직진·원호, 껍데기 회귀, 간격, `time_not_increasing` | 앞 판 단언 + reasons |
| `test_m2_gains_fwd_rev`(②) | gain 0.9, `q_offset` 0.005 | 두 이득, 후진 부호, `distance_source` "truth" |
| `test_m2_under_reached_arc_only`(②) | max, arc 의 q 가 0.03 모자람, presteer 는 도달, meas_q 일부 None | `under_reached` 참, None 비율 |
| `test_m2_small_rotation`(②) | Δθ_true = 0 | `rotation_too_small`, 예외 없음 |
| `test_m3_merge_judging_point`(③) | A 와 B 각각 | A: `end` + 대체 규칙 표지. B: `run_042` 는 `judgeable:false, why:"run_042_definition_pending"`(정의가 정해질 때까지, 9 절), `end` 보고, `odo_dt_ns` |
| `test_m3_l21_merge_end_not_judgeable`(③) | 2.1 m, 책상 N | `lower_bound_over_budget`(계산으로 나옴) |
| `test_m3_qstep_delay`(③) | 엔코더 지연 0.083 s, STATE 10 ms | 0.083 ± 0.005 s. ns 입력. 사후 창이 맞지 않으면 사유 붙은 지연 void |
| `test_m3_imu_delay_offset`(③) | MCU–IMU 오프셋 15 ms | IMU 지연에 오프셋이 그대로 들어가고 `unknown_clock_offset` |

## 7. 확인 필요

1. **등록이 초안이다.** 출력에 `status:"draft"` 를 낸다. 0.025·0.15 는 10-23 실기록으로 아직 확인하지 않았다(2.2).
2. **`truth_missing` 을 void 로 둘지:** 제안은 void 다(등록 1.1 72 행). `--no-truth` 는 유효 + 판정 불가다. 총괄 결정이 필요하다.
3. **서 있음 문턱**(바퀴 속도: `stop_speed_mps` 후보)과 **조향 |Δq| 문턱**.
4. **간격 문턱(정함, 총괄 확인):** IMU run 10 ms, 서 있음·오도메트리 `max_gap_s` 0.05 s(9 절). D5 의 150 ms 는 호스트 정지 상한이라 쓰지 않는다.
5. **Δb 의 정의:** û 투영을 제안한다(3 축 노름은 보조값).
6. **`q_threshold`:** 0.02 rad(`PRESTEER_TOL_RAD`) 또는 계단의 10 %.
7. **현장 void 와 재확인을 어디에 적을지:** 참값 파일의 void 행으로 둘지 `field_log` 로 모을지. "결과 전에 닫음" 을 어떻게 증명할지(시각, 해시).
8. **다리 머리 줄**에 일정표 sha256 과 `vehicle.yaml` 의 `calibration_id`·해시를 넣는 작은 다리 PR. 그 전에는 `--vehicle` 이 기록 때의 설정이라는 보장이 없다.
9. **G3 출력 스키마**와 날짜·카메라 serial 대조.
10. **A 의 바닥 표시 → 뒤축 중심 변환**(G9). 추정 뒤축 위치와 실제 기구 중심의 차이는 M2 횡에 δx·sin Δθ 로 들어간다.
11. **M3 A 대체 판정점**(5.1, 총괄)과 **B `run_042` 의 0.42 m 를 무엇으로 잴지**(G9).
12. **움직임 시작의 정의**(첫 카운트 변화 STATE)를 등록에 넣을지.
13. **STATE 주기 `state_period_ms`**(10 ms 가정), 엔코더 y 의 평활(2–3 표본), MCU `wheel_rate` 필터.
14. **N_body 판정은 등록대로 창별**(9 절). 오거부율이 크면 등록 개정을 총괄에 올린다.
15. **IMU 파일 단위:** 블록 하나(데움부터 이어짐)를 제안한다. 시행마다 파일을 끊으면 6 구간 이동 맞춤이 성립하지 않는다(등록 81 행의 디렉터리 구조).
16. **M2 θ_min**(제안 0.01 rad).
17. **텔레옵 꼬리 줄 결함을 거부로 할지**(제안), 시행 창 단위로 완화할지. MCU·IMU fit 품질 지표의 문턱(지금은 보고만 한다).
18. 등록 문서 쪽 표기(8 절 agy 모순 8)를 그쪽 세션에 전달하는 일.
19. **교차검증:** Codex 적대 검토 5 회차에서 "새 P1/P2 없음"으로 수렴했다(q 계단 산식은 반박이 받아들여짐). agy 전제 검토 12 항목은 8 절에서 처리했다. 원자료 재계산은 해당 없다(실차 기록이 아직 없다). 등록 상수의 재대조는 Codex 1 회차가 했다.

## 8. 검토 반영

**Codex 적대 검토 1 회차**

| 지적 | 처리 |
|---|---|
| P1 합성 참값이 판정을 우회 | 받음. 파일마다 머리 줄을 필수로 하고 `refuse_or_mark` 를 파일마다 부른다. 이 함수가 header 행만 보는 것을 확인했다(`synthetic.py:79`). 참값만 합성인 경우의 시험을 넣었다(2 절, 6 절) |
| P1 `truth/1` 가 A/B 에 부족 | 받음. id·종류 고정, moving_point 의 `host_ns`·`cov` 필수, A 대체를 별도 규칙으로, B 판정점은 `run_042`(2 절, 3.7) |
| P1 void 가 좁고 no-truth 를 섞음 | 받음. 측정별 표(3.4)를 두고 `--no-truth` 와 `truth_missing` 을 갈랐다. 앞 판도 7.2 에서 둘을 갈랐지만 측정별 정의가 없었다 |
| P2 편향 변화율의 조치 기록 | 받음. 기울기를 블록·시각과 함께 내고, `field_log/1` 의 재확인으로만 판정 가능하게 했다. 없으면 `bias_slope_unconfirmed`(3.3 ⑦) |
| P2 `s_enc` 현 | 받음. `s_enc_path_m`(경로)와 `disp_enc_m`(변위)를 나눴다(3.5) |
| P2 이름 변경 호환 | 받음. 별칭 경로에서만 옛 키를 남기고, 두 경로의 반환 dict 전체를 시험한다. 소비자 목록과 grep 재확인을 적었다. 위치 인자 걱정은 해당 없다: 두 함수는 이미 키워드 전용이다(`m_analysis.py:215,226`) |
| P2 q 계단의 시계 | 받음. 호스트 ns 하나로 통일했다. `ClockFit` 에 장치 → 호스트만 있으므로(`clock_map.py:117-123`) STATE 를 호스트로 옮긴다. 지연은 측정 q 출발에서 잰다(3.2, 3.8) |
| P2 신원·시계 무결성 | 받음. 신원 검사 표(2 절): fit valid·품질, MCU boot, IMU 도메인·늦은 표본·덮음 |
| P2 입구 경로 실패 시험 | 받음. 6 절의 신원·void·경계·합성 전파 시험 |
| 답 1: 상수 표기 | 받음. k 한계를 점마다 풀어 썼다. 525-532 행은 등록 원문과 맞다. "직전 6 창" 은 "직전 6 개 편향 구간"(180 행)으로 고쳤다 |
| 답 6: PR 의존성 | 받음. ① 이 공유 모델·검증·결과 계약을 먼저 고정한다 |

**agy 전제 검토**

| 전제·모순 | 처리 |
|---|---|
| 1 레버암 빠진 예산 | 받음. 상대량으로 검사하고 카메라 수치로 시험한다(3.3 ⑩) |
| 2 label 집합으로는 파일을 못 가림 | 받음. 확인했다(m1 4 개, m2 16 개가 각각 같은 집합). `profile_start` 이름(`teleop_bridge.py:178`)과 `req_mps` 를 대조한다 |
| 3 IMU 역행·중복, `dom` | 대부분 받음. `imu_stream` 은 걸러 내지 않는다(92-122 행). 그래서 늦은·중복 표본을 빼고 fit 의 valid·품질을 본다. 다만 dev_us 는 프레임 메타데이터라(`tools/imu_record.py:132`) `dom` 이 섞여도 그 자체로 dev_us 가 틀리지는 않는다. 도메인이 섞이면 보수적으로 판정 불가로 둔다 |
| 4 `quiet_window` 가 빈틈을 건넘, presteer | 받음. 확인했다(83-87·122 행). 함수를 고치고 입구에서도 검사하며, 조향 |Δq| 를 뺀다 |
| 5 `calibration_id` 가 머리 줄에 없음 | 받음. 확인했다(402-404 행). 다리 PR 로 확인 필요에 올렸다 |
| 6 STATE 50 Hz, µs, 역사상, 필터 | 부분 받음. "50 Hz" 는 틀렸다: STATE 행은 받은 STATE 마다 쓰이고(`_on_state` 223-230), `CYCLE_S` 20 ms 는 다리 주기다. 그러나 10 ms 여도 25 ms 창에는 2–3 표본뿐이라 확인 필요로 올렸다. ns 단위(261 행)와 역사상 문제는 받았다(3.2, 3.8). 명령 시각과 MCU 수신 시각의 편차는 측정 q 출발을 쓰므로 영향이 없다 |
| 7 yaw 감김, 뒤축 불일치, B 시각 | 받음. atan2 정규화, host_ns 보간과 `odo_dt_ns`. 뒤축 위치는 G9 변환 문제라 확인 필요 10 에 올렸다 |
| 8 `under_reached` 범위, Δθ = 0 | 받음. arc 의 움직인 뒤 행만 보고 None 비율을 낸다. sinc 가드와 `rotation_too_small`(3.6) |
| 9 boot 만으로 부족 | 받음. 덮음, IMU `profile`·꼬리 줄, 텔레옵 꼬리 줄. G3 대조는 확인 필요 |
| 10 Allan max·비균일 | 받음. 실제 시각으로 계산하고 오거부 시험을 둔다. 판정은 **9 절에서 창별(등록대로)로 바꿨다** — 이 표의 1 회차 처리(합침 판정)는 대체됨 |
| 11 결함 비트·움직임 시작 | 받음. 마스크를 넓히고 `time_not_increasing` 을 넣었다(`real_vehicle.py:328-329`). 움직임 시작을 정의했다(확인 필요) |
| 12 `m_synth` 기본값·이상적 모형 | 받음. 기본 참, 적대적 사례, 실행기가 쓰는 `profile` 행 |
| 모순 1–3 | 위 1·6·11 과 같이 처리 |
| 모순 4 "같은 파일 직전 6 창" | 받음. 파일에 구간이 모자라면 `field_log` 가 없을 때 판정 불가로 둔다. 파일 단위는 확인 필요 15 |
| 모순 5 적분 구간 표현 | 받음. M1·M2 는 창 사이, M3 는 움직임 시작부터. T 는 지속 시간이다 |
| 모순 6 B 의 `end` 꼬리표 | 받음. B 에서는 `run_042` 가 판정점이고 `end` 는 보고값이다(등록 5.5, 399·403 행) |
| 모순 7 `m_synth` 출처·기본값 | 받음(6 절) |
| 모순 8 등록 대 코드 | 이 도구와 무관하다. 등록 쪽에 전달한다: G6 경계 수정은 이미 코드에 있다(`drive_profile.py:298-314`). 등록 6.3 의 T 13.45 s 는 가속 0.125 s 를 두 번 넣은 값이다(13.325 s). 이 도구는 T 를 기록에서 재므로 영향이 없다 |

## 9. Codex 2 회차 반영 (2026-11-15, 담당 세션)

| 지적 | 판단 | 처리 |
|---|---|---|
| P1 q 계단 산식이 함수와 안 맞음 | **반박.** `step_response_delay`(`m_analysis.py:249-290`)의 출발은 첫 인자 q 의 출발이고, 반환값은 "그 출발부터 y 가 63 % 에 닿을 때까지"다. 두 번째 호출 `(q_t, q, q_t, q)` 에서는 y 가 q 자신이다. q 는 0.1312 rad 계단이라 `step ≥ 1e-9` 이므로 `no_response` 가 나지 않는다. 결과는 q 출발부터 q 의 63 % 까지다. 두 호출의 출발이 같으므로 차는 등록 5.2 의 t₆₃(y) − t₆₃(q) 다(등록 문서 해당 절 13 행). 시각은 ns 로 해석된다(`int(pre_s * 1e9)`, 261 행) | 3.8 그대로 둔다. 시험 `test_m3_qstep_delay` 에 "두 번째 호출이 `delay_s` 를 낸다"는 단언과 램프 0.4 s 만 있을 때(y 지연 0)의 값 0 ± 5 ms 를 더한다 |
| P1 B 의 `run_042` 판정점 뜻이 미정 | 받음 | 0.42 m 를 무엇으로 잴지(엔코더냐 참이냐)를 G9·총괄이 정하기 전에는, B 의 `run_042` 항목을 **보고값**(`judgeable:false, why:"run_042_definition_pending"`)으로만 낸다. 정지 자세 σ 는 머리 줄의 축별 σ 로, 축 사이 상관 없이 대각 공분산을 쓴다(계약에 적는다). 상관이 있으면 B 처럼 점마다 `cov` 를 준다 |
| P2 레버암 식이 상관을 무시 | 받음 | 상대 자세 `Δ = R(−θ₀)(p₁ − p₀)`, `Δθ = θ₁ − θ₀` 의 야코비안으로 두 자세의 3×3 공분산(자세끼리는 독립)을 전파해 Δ 의 공분산을 낸다. 예산 검사는 그 대각 성분으로 한다. 등록 6.1–6.3 의 식은 대각·독립인 경우의 특수형이다. 시험 `test_relative_budget_jacobian`: 대각이면 등록 식과 1e-12 안에서 같고, 상관이 있으면 다르다 |
| P2 IMU 정리 뒤 배열 쌍 | 받음 | 늦은·중복 표본은 하나의 불리언 마스크로 정해 `dev_us`·`host_ns`·`xyz` 에 같이 적용한다. 그 뒤에 G7 맞춤을 다시 낸다. 시험 `test_imu_cleaning_keeps_pairs`: 늦은 표본 하나를 넣으면 그 표본만 빠지고, 나머지 쌍은 원본과 같다 |
| P2 Allan 합침 | 받음 — **등록대로 창마다** | 등록 6.3 은 "앞뒤 정지 창에서 다시 재" 한계와 비교한다. 그래서 두 창 각각의 N 이 k 한계 안일 때만 자이로 항목을 값으로 낸다(최대값으로 판정, 합침은 판정에 쓰지 않는다). 오거부율 시뮬레이션은 정보로만 낸다. 오거부가 크면 등록 개정이 필요한 일이지 도구가 바꿀 일이 아니라고 적는다 |
| P2 `quiet_window` 간격 문턱 | 받음 — 값을 정함 | IMU 연속 run 은 이웃 자이로 표본 간격이 10 ms(400 Hz 의 4 주기)를 넘으면 끊는다. 서 있는 구간은 STATE 간격이 `max_gap_s = 0.05 s`(STATE 10 ms 의 5 주기, 호스트 신선도 문턱과 같은 값)를 넘으면 끊는다. 오도메트리 `max_gap_s` 도 0.05 s 다. D5 의 150 ms 는 호스트 정지 상한이지 데이터 간격이 아니므로 쓰지 않는다. 두 값은 총괄 확인을 받는다(7 절 4 갱신) |
| 부분 해결: `field_log` 신원·순서 | 받음 | `field_log/1` 머리 줄에 `host_boot_id`·기록 `start_mono_ns` 를 넣고 텔레옵 기록과 대조한다. 행은 시각 순이어야 하고, 각 재확인 행은 해당 블록의 마지막 시행보다 앞서야 한다. 어기면 exit 2 |
| 새 문제: 주행 중 점 보간 | 받음 | STATE 의 `mcu_time_us` 를 기록의 MCU G7 맞춤으로 호스트 ns 로 옮긴 오도메트리 시계열에서, 참값 `host_ns` 를 둘러싼 두 표본으로 선형 보간한다. `odo_dt_ns` 는 가까운 쪽 표본과의 차다. 맞춤이 그 시각을 덮지 않으면(외삽) `odo_extrapolated` 로 판정 불가다 |

