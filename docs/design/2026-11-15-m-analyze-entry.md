# G8 입구 — `tools/m_analyze.py` 와 `forklift_core.plant.m_entry` (D4) — 설계 (2026-11-15)

- 출처: [도착일 리허설 기록](../validation/2026-11-15-arrival-day-rehearsal.md) D4(13 행), [M1–M3 등록](../plans/2026-11-15-m1-m3-registration.md)(초안 개정 2) 1–6 절, [프로필 구동·M 분석](2026-11-14-profile-drive-and-m-analysis.md) 3 절, [리허설 설계](2026-11-15-arrival-day-rehearsal.md) 2.3·2.5.
- 상태: 초안(크레딧 opus 초안을 담당 세션이 대조). 구현 전에 독립 검토(Codex 적대·agy 전제)를 거친다.
- **PR 나누기**(제안): ① `m_analysis` 변경(4 절)과 공통 입력·M1·CLI ② M2 ③ M3. 각 PR 은 시험부터 쓰고, 머지 전 ws1 전체 시험을 거친다. 합성 생성기 `m_synth` 는 ① 에 넣는다.

## 1. 목적·범위

- **D4 가 고치는 것:** `m_analysis.py` 의 계산 함수는 다 있지만, 기록을 읽어 그 함수들에 넣어 주는 입구가 없다. 리허설에서는 세션 스크립트가 아래 고리를 손으로 이었다.
  - 기록 행 → `McuState`: `sample_from_row(r).state`(`src/forklift_core/plant/record_rows.py:22-37`)
  - IMU 행 → `imu_stream`(`src/forklift_core/sensors/imu_record.py:92-122`)
  - 서 있는 구간, `quiet_window`, `trials()` 의 구간, 일정표 행의 `s_m`, 참값 파일
  
  이 일을 맡는 라이브러리 `src/forklift_core/plant/m_entry.py` 와 CLI `tools/m_analyze.py` 를 만든다. 시행마다 등록 3.3·4.3·5.4 의 값을 낸다.
- **세션 스크립트에서 드러난 빈틈**(도구가 같은 실수를 하지 않게 적는다):
  1. `s_m` 을 모든 시행의 `straight` 행에서 max 로 골랐다. 시행으로 거르지 않았다.
  2. 그 `s_m` 은 엔코더 거리인데 `distance_source="truth"` 로 넘겼다.
  3. `quiet_window` 를 5/3 s, 기본 `QUIET_GYRO_RAD_S` 0.01(`m_analysis.py:21`)로 불렀다. 등록 2.2(160·163 행)는 60/50 s, 0.025 다.
  4. `up=(0,-1,0)` 을 코드에 박았다. G3 의 û_ω 를 입력으로 받아야 한다.
  5. 오도메트리 `broken` 을 보고만 하고 시행을 void 로 만들지 않았다. 엔코더 결함 비트도 보지 않았다(등록 1.1, 70 행).
  6. `heading_sigma_lower` 에 책상 N 과 창 3.0 s 를 상수로 넣었다.
  7. 참값을 시행 번호로만 짝지었다. 기록과 IMU 의 `host_boot_id` 도 대조하지 않았다.
- **범위 밖:**
  - 판정 문턱 L·H, 중앙값·순서통계량 구간, +5 회. 이것은 등록 1.1 의 규칙이고, 여러 기록을 합치는 판정 도구가 맡는다. 이 도구는 시행별 값과 그 값을 판정할 수 있는지만 낸다(설계 3 절 82 행: "G8 은 문턱을 박지 않는다").
  - G9 참값 방법(A/B) 선택. 이 도구는 참값을 파일로 받기만 한다.
  - G4.
  - G3 배율 계산(`gyro_scale` 의 Δψ 변경, 등록 2.6). 지금은 배율이 없다고 보고, 그것이 필요한 값을 판정 불가로 낸다.

## 2. 입력

```
tools/m_analyze.py --record TELEOP.jsonl --imu IMU.jsonl --profile config/profiles/m1_m3/<name>.yaml
                   (--truth TRUTH.jsonl | --no-truth) (--mount G3.json | --up X,Y,Z)
                   [--vehicle config/real/vehicle.yaml] [--trial N] [--allow-synthetic]
```

- **텔레옵 기록**(`teleop_bridge.py --record --profile`)
  - 머리 줄의 `host_boot_id`·`synthetic`·`start_mono_ns`(`tools/teleop_bridge.py:402-404`), `state` 행, `profile` 행(`drive_profile.py:384-394`)을 읽는다.
  - 머리 줄에 일정표 이름이 없다(402 행). 그래서 `--profile` 을 필수로 받는다. 그 파일을 `parse_profile` 로 읽은 구간 label 집합이 기록의 `profile` 행 label 집합과 다르면 거부한다(exit 2).
  - 측정 종류는 일정표 `name` 의 접두사로 정한다: `m1_`, `m2_`, `m3_merge_`, `m3_qstep_`. 그 밖(`g3_` 등)은 거부한다.
- **IMU 기록**(`imu_record/1`)
  - `imu_stream(rows, stream="g"|"a")` 로 읽는다. 한 파일에 boot 나 시계가 둘 이상이면 거기서 이미 거부된다(`imu_record.py:116-119`).
  - 시각은 `imu_clock_fits`(125-133 행)의 G7 맞춤으로 dev_us 를 호스트 시각으로 옮긴 값을 쓴다. 콜백 `host_ns` 의 지터를 적분에 넣지 않기 위해서다. 시행 구간이 맞춤 구간 둘에 걸치면 그 시행의 IMU 항목은 판정 불가(`imu_clock_break`)다.
  - 가속도는 자이로 시각에 선형 보간한다(리허설과 같다).
- **장착 축**
  - `--mount` 는 G3 출력에서 두 방향 평균 û_ω 를 읽는다.
  - `--up` 은 손으로 주는 벡터다. 출력에 `up_source: "argument"` 로 남는다.
  - 기본값은 두지 않는다. D435i 의 y 가 아래라서 위쪽 축은 (0,−1,0) 근처지만(등록 1.1 표 27 행) 숙임은 G3 가 준다.
- **참값 `truth/1`**(새 스키마; G9 방법과 상관없다)
  - 머리 줄: `{"t":"header","schema":"truth/1","method":"A_floor_marks"|"B_camera"|"synthetic","synthetic":bool,"record":{"host_boot_id","start_mono_ns"},"body_point":"rear_axle_center","frame":"floor","sigma":{"pose_yaw_rad","pose_xy_m","source":"repeatability_<date>"|"camera_validation_<date>"}}`
  - 시행 행: `{"t":"truth","trial":N,"poses":{"start":P,"end":P,["merge_end":P],["run_042":P]}}`. `P = {"x_m","y_m","yaw_rad",["host_ns"],["sigma_yaw_rad","sigma_xy_m"]}` 이다. 자세별 σ 는 B 처럼 자세마다 다를 때만 넣고, 없으면 머리 줄 값을 쓴다. A 의 σ 는 그날의 반복성 검사로 정한다. 시행마다 고르지 않는다(등록 6 절 415 행).
  - 현장 void 행: `{"t":"void","trial":N,"reason":"operator|contact|mark_lost|...","closed_before_results":true}`. 등록 1.1(71 행)은 현장 void 를 "결과 계산 전에 닫는다" 고 정했다. 그래서 도구는 결과를 계산하기 전에 이 행만 읽는다.
  - `record` 가 텔레옵 머리 줄과 다르면 거부한다(exit 2). `body_point` 가 `rear_axle_center` 가 아니어도 거부한다. `RealOdometry` 의 자세와 같은 점이어야 하기 때문이다.
- **옛 `rehearsal_truth/1` 호환**
  - `tools/rehearsal_imu_truth.py:94-99` 가 쓰는 형식이다. 행에 `trial`·`dyaw_rad`·`start/end_host_ns` 만 있다.
  - `--allow-synthetic` 일 때만 읽고, heading 만 있는 참값으로 다룬다. 거리와 횡 위치는 없으므로 거리는 엔코더 거리로 표시한다.
  - 리허설 도구는 `odometry_pose`(4 절)가 생기면 `truth/1` 을 쓰도록 바꾼다.
- **짝 맞추기·합성 차단**
  - 맨 먼저 `refuse_or_mark(rec + imu + truth, allow=args.allow_synthetic)`(`src/forklift_core/live/synthetic.py:78-85`)를 부른다. 머리 줄에 표지가 없어도 장치가 pty 인 기록은 합성으로 본다(68-75 행).
  - 그다음 텔레옵 `host_boot_id` == IMU `host_boot_id` 를 확인한다. 리허설 IMU 도 이 값을 옮겨 쓴다(`rehearsal_imu_truth.py:62-63`).
  - 어느 쪽이든 실패하면 `bringup.py` 처럼 `{"result":"refused","why":...}` 를 내고 exit 2 다(`tools/bringup.py:941-947`).

## 3. 계산

**등록 상수.** `m_entry.REGISTERED` 에 한 곳으로 모으고 등록 판 이름을 붙인다. `m_analysis` 의 기본값(`QUIET_GYRO_RAD_S` 0.01, 21 행)은 바꾸지 않고, 등록값을 인자로 넘긴다.

| 상수 | 값 | 등록 |
|---|---|---|
| `quiet_window` max_s / min_s | 60 / 50 s | 2.2(160 행) |
| gyro_tol / accel_tol | 0.025 rad/s / 0.15 m/s² | 2.2(163·169 행) |
| 시행별 \|Δb\| 급변 | 1.5×10⁻⁴ rad/s | 2.3 (a) |
| 편향 변화율, 직전 6 창 맞춤 | 3.5×10⁻⁷ rad/s² | 2.3 (b) |
| 책상 N | 2.054×10⁻⁴ rad/√s | 1.1 표 28 행 |
| heading 1σ 예산 | 1 mrad(자세 0.7), 위치 1.7 mm(자세 1.2) | 6 절 413-414 행 |
| 차체 위 N 의 k 한계 | 합류 끝 1.48, 주행 중 0.42 m 점 1.13 | 6.3, 525-532 행 |
| 배율 없이 허용하는 회전 | 0.04 rad | 1.1(79 행) |
| 가속 램프 | 0.125 s | 4.3·5.1 |
| 평활 창 | 중심 이동평균 25 ms | 5.2(371 행) |
| `pre_s` / `frac` | 0.5 s / 0.63 | 5.2 |

**모든 시행에 공통인 순서**

1. `trials(rows)` 를 `valid_only=False` 로 부른다(`m_analysis.py:41-59`). void 도 사유와 함께 세기 위해서다.
2. `states = [sample_from_row(r).state for r in rows if r["t"]=="state"]` 를 만든다. 오도메트리는 기록 전체에 한 번 돌린다. 연속성이 시행 밖에서 끊긴 것은 상관없다.
3. 시행 창 [t₀, t₁] 은 그 시행의 첫·끝 `running` 행이다. 주행 구간은 일정표에서 `kind == "drive"` 인 label 들이다.
4. 시행 void(등록 1.1, 68-72 행). 해당하면 값을 내지 않고 사유를 센다.
   - G6 결과 ≠ `done`
   - 창 안의 `broken`
   - 창 안 STATE 의 `ENCODER_GAP|ENCODER_JUMP_L|_R`(`m_analysis.py:24`)
   - 참값 파일의 현장 void 행
   - 참값 파일은 있는데 그 시행 행이 없음(`truth_missing`)
5. 서 있는 구간: 두 바퀴 속도가 문턱 아래인 STATE 의 연속 `host_ns` 구간. 문턱은 7 절 확인 필요다.
6. 편향 창
   - 앞 창: `quiet_window(..., boundary_ns=첫 주행 구간 시작, side="before", max_s=60, min_s=50, standing=..., gyro_tol=0.025, accel_tol=0.15)`(64-122 행). 뒤 창은 마지막 주행 구간 끝에서 `side="after"` 로 찾는다.
   - 하나라도 None 이면 IMU 항목은 `no_bias_window` 로 판정 불가다. 시행은 유효로 남는다(1.1, 76-77 행).
   - 창의 편향은 û 성분으로 비교한다(Δb_yaw). `bias_between`(`imu_mount.py:72-75`)의 3 축 노름은 보조값으로만 낸다(7 절).
   - |Δb_yaw| > 1.5×10⁻⁴ 이면 `bias_jump` 다.
   - 같은 IMU 기록에서 직전 6 창의 편향을 시각에 대해 직선 맞춤해 그 기울기를 낸다. 3.5×10⁻⁷ 을 넘으면 `bias_slope` 다. 현장 조치(10 분 정지)를 대신하지 않는 사후 보고다.
7. 차체 위 N: 두 창에서 û 투영 rate 의 τ = 1 s Allan 편차를 각각 내고(등록 6.3, 531 행) 큰 쪽을 쓴다. 새 함수 `allan_dev_tau` 로 계산한다(4 절에 시험).
8. IMU heading: `imu_heading(t[창0 끝 : 창1 시작], g, up=û, bias=두 창 평균)`(125-137 행). 하한은 `heading_sigma_lower(arw=N_body, drive_s=적분 길이, bias_window_s=짧은 쪽 창 길이, bias_change_rad_s=Δb_yaw)`(140-152 행)다. 책상 N 으로 낸 값도 나란히 적는다. 출력에 `is_lower_bound` 와 `not_included` 를 그대로 옮긴다.
9. 상대 자세: 오도메트리(`odometry_pose`, 4 절)와 참값 모두 출발 자세의 좌표계로 바꾼다. Δ = R(−θ₀)(p₁ − p₀), Δθ = θ₁ − θ₀ 이다. 참값 자세의 σ 가 예산(자세 0.7 mrad·1.2 mm)을 넘으면 그 항목은 `truth_over_budget` 으로 판정 불가다.

**M1**(등록 3.3)
- Δθ_odo 와 Δθ_true 는 시행 창 양 끝, 곧 정지 자세에서 잰다.
- s_true = |Δp_true| 다. 직진이라 현의 길이를 쓴다.
- s_enc = |Δp_odo| 다. 일정표 `s_m` 이 아니라 오도메트리 현을 쓴다. `s_m` 은 구간 끝에서 멈추고(`drive_profile.py:293`) 감속 3.3 mm 를 빠뜨려, 바퀴 반경 배율에 0.1 % 가 섞이기 때문이다. `s_m` 은 그 시행 행으로만 걸러서 교차 확인값으로 낸다.
- d 는 `m1_drift(odo_dyaw, heading_ref_dyaw=Δθ_true, ref_source="truth", distance_m=s_true, distance_source="truth")` 로 낸다.
- 끝 횡 오차는 Δp_odo,y − Δp_true,y 다. `radius_scale = s_true/s_enc` 를 낸다.
- IMU Δθ 와 그 하한은 보고값이다(6.1).
- `--no-truth` 면 `distance_source="encoder"`, 기준을 `ref_source="imu"` 로 두고 모든 값을 `judgeable:false, why:"no_truth"` 로 낸다.

**M2**(등록 4.3)
- 원호 식은 Δθ = Δθ_true, c = |Δp_true|, s = c·(Δθ/2)/sin(Δθ/2) 다. direction 은 일정표 속도의 부호다.
- `kappa_measured_q` 는 arc 구간에서 움직임 시작 + 0.125 s 뒤 `meas_q` 의 평균을 `RealKinematics.held_curvature` 로 바꾼 값이다. `kappa_commanded_q` 는 같은 구간 `drive_q` 의 평균에서 낸다.
- 이득은 `m2_gains(heading_ref_dyaw=Δθ_true, ref_source="truth", distance_m=s, direction=±1, ...)` 로 낸다.
- 보고값:
  - `meas_q` 의 흔들린 폭, `drive_q − meas_q` 의 중앙값
  - presteer 가 다가온 방향(`presteer_zero` → 목표)
  - `under_reached`. 행에 없으므로 `max` 일정표에서 \|meas_q − q(κmax)\| ≤ 0.02 를 한 번도 만나지 않았는지로 다시 계산한다.
- IMU κ 는 G3 배율이 없으면 `no_gyro_scale` 로 판정 불가다(2.6, 회전 ≥ 0.068 > 0.04 rad).

**M3 합류**(등록 5.4)
- 오도메트리 자세는 `merge` 끝(그 span 의 끝 `host_ns`)과 `cont` 뒤 정지에서 낸다.
- A 의 판정점은 `cont` 끝 정지 자세, 곧 참값의 `end` 다. e_θ = Δθ_odo − Δθ_true, e_y 는 횡 성분, e_x 는 보고만 한다. 출력에 `point:"cont_end_stop", substitute_pending_lead:true` 를 붙인다(5.1 승인 대기).
- 참값에 `merge_end`·`run_042` 자세가 있으면(B) 같은 식으로 낸다.
- IMU heading 은 움직임 시작에서 각 점까지 적분한다.
  - T 는 주행 시간에 움직임 시작 시각을 더한 값으로, 엔코더로 잰 실제 시간이다.
  - N_body ≤ k × 책상 N 이고 그 N 의 하한 ≤ 1 mrad 일 때만 값이다. 아니면 `imu_noise_over_limit` 또는 `lower_bound_over_budget` 이다. 2.1 m 합류 끝이 판정 불가인 것은 이 계산에서 나와야 한다. 따로 박아 넣지 않는다.
- 배율은 필요 없다(5.4 395 행). 단 시행 순회전이 0.04 rad 를 넘으면 `no_gyro_scale` 이다.

**M3 q 계단**(등록 5.2)
- q 는 STATE `steering_rad`, y 는 `encoder_yaw_rate(wheel rates, radius_m, track_m)`(185-190 행)다. 둘 다 `mcu_time_us` 한 시계에 있다. y 는 25 ms 중심 평활을 거친다.
- 사후 창의 시작은 "계단 명령 시각 + 램프 + 1.0 s" 다.
  - 계단 명령 시각은 `step` 구간에서 `req_kappa` 가 목표로 바뀐 첫 `profile` 행이다.
  - 램프는 `Segment.q_step_ramp_s`(`drive_profile.py:175`), 1.0 s 는 `Q_STEP_SETTLE_S`(31 행)다.
  - 이 시각을 G7 맞춤(`clock_fits_from_rows`)으로 MCU 시각에 놓는다. 창의 끝은 구간 끝이다.
- 지연은 `step_response_delay(q_t, q, y_t, y, ...)` − `step_response_delay(q_t, q, q_t, q, ...)` 로 낸다(249-290 행). 그 결과가 t₆₃(y) − t₆₃(q) 다.
- 어느 쪽이든 None 이면 그 사유로 지연 void 다(5.6).
- IMU 지연은 같은 식으로 계산하고 `unknown_clock_offset:true` 를 붙여 보고만 한다.

## 4. G8 변경 요청 처리(`m_analysis.py`)

**(가) `imu_dyaw` 이름**(등록 3.3 261 행, 4.3 314 행)
- `m1_drift` 의 시그니처를 `(*, odo_dyaw, heading_ref_dyaw=None, ref_source=None, imu_dyaw=None, distance_m, distance_source)` 로 바꾼다. `m2_gains` 도 같은 방식으로 바꾼다.
- `heading_ref_dyaw` 와 `imu_dyaw` 는 정확히 하나만 받는다. `imu_dyaw` 는 하위 호환 별칭이다: `heading_ref_dyaw = imu_dyaw`, `ref_source = "imu"`.
- `ref_source` 는 `("truth", "imu")` 가운데 하나다. 결과에는 `odo_minus_ref_rad` 와 `ref_source` 를 담는다. 옛 키 `odo_minus_imu_rad` 는 `ref_source == "imu"` 일 때만 남긴다. 참값을 "imu" 로 적는 일이 없게 하기 위해서다.
- `m2_gains` 에서 `ref_source == "imu"` 이면 note 에 "gyro scale needed" 를 붙인다.

**(나) x·y 자세**(3.3 264 행, 5.4 391 행)
- `odometry_pose(states, cfg, *, max_gap_s)` 를 새로 둔다. `{"mcu_time_us","x","y","yaw","broken"}` 을 돌려준다. `RealOdometry.update(...).pose` 가 이미 (x, y, yaw) 를 준다(179 행은 [2] 만 쓴다).
- `odometry_heading` 은 이 함수의 껍데기로 바꾸고 반환 키는 그대로 둔다.
- `relative_pose(p0, p1) -> (dx_fwd, dy_lat, dyaw)` 를 더한다. `allan_dev_tau(t_ns, rate, tau_s)` 도 더한다. 둘 다 순수 함수다.

**시험**(`tests/unit/` 의 m_analysis 시험 파일에 더한다)

| 시험 | 단언 |
|---|---|
| `test_m1_drift_truth_ref_named` | 결과에 `ref_source=="truth"` 가 있고 `odo_minus_imu_rad` 키가 없다 |
| `test_m1_drift_imu_alias_unchanged` | 옛 호출의 수치가 바뀌지 않고 `ref_source=="imu"` 가 붙는다 |
| `test_ref_args_exclusive` | 둘 다 주거나 둘 다 없거나 `ref_source` 가 잘못되면 ValueError |
| `test_m2_gains_ref_source` | M2 에서 같은 세 가지(이름, 별칭, 배타) |
| `test_odometry_pose_straight_arc` | 같은 바퀴 직진 3 m → x 3, y 0. q 고정 원호 → 현·Δθ 가 운동학과 1e-9 안에서 맞는다 |
| `test_odometry_heading_regression` | 껍데기의 `yaw`·`broken` 이 이전 구현과 같다 |
| `test_odometry_pose_break` | 간격 > `max_gap_s` 가 `broken` 에 남는다 |
| `test_relative_pose_rotated_start` | 출발 heading 이 0 이 아니어도 횡 성분이 맞다 |
| `test_allan_white` | 백색 rate 잡음 N → σ(1 s) ≈ N(±5 %) |

## 5. 출력 JSON(`m_analyze/1`, 표준 출력 한 줄)

```json
{"synthetic_inputs": true, "not_for_judgement": true,
 "schema": "m_analyze/1", "result": "ok", "measurement": "m1",
 "registration": {"doc": "docs/plans/2026-11-15-m1-m3-registration.md", "revision": "초안 개정 2", "status": "draft",
                  "constants": {"quiet_max_s": 60, "quiet_min_s": 50, "quiet_gyro_rad_s": 0.025, "...": "..."}},
 "inputs": {"record": {"path", "sha256", "host_boot_id"}, "imu": {"path", "sha256", "fits": [...]},
            "truth": {"path", "sha256", "schema", "method"} | null, "profile": {"path", "name", "sha256"},
            "vehicle": {"path", "sha256"}, "up": {"vector": [...], "source": "g3|argument"}},
 "counts": {"trials": 6, "valid": 5, "void": {"void:deadman": 1}},
 "trials": [{"trial": 1, "status": "valid", "result": "done", "window_ns": [t0, t1],
             "condition": {"direction": 1, "speed_mps": 0.1},
             "distance": {"source": "truth", "s_true_m": 3.001, "s_enc_m": 2.996, "s_m_profile": 3.000,
                          "radius_scale": 1.0017},
             "values": {"d_rad_per_m": {"value": -0.0004, "judgeable": true},
                        "end_lateral_m": {"value": 0.0012, "judgeable": true, "report_only": true}},
             "imu": {"status": "value|not_judgeable", "why": ["no_bias_window"], "dyaw_rad": ...,
                     "windows": {"before": {"start_ns", "end_ns", "len_s", "bias_yaw"}, "after": {...}},
                     "bias_change_yaw_rad_s": ..., "bias_slope": ..., "n_body": ..., "sigma_lower": {...},
                     "report_only": true}}]}
```

- 합성 표시(`OUTPUT_MARKS`, `synthetic.py:22`)는 맨 앞에 둔다(bringup 과 같음).
- void 시행은 `status:"void", why` 만 적고 값을 내지 않는다.
- 판정 불가 항목은 `{"value": x 또는 null, "judgeable": false, "why": ...}` 로 적는다. 값이 계산되어도 판정에 쓰지 않는다는 표시다.
- exit code:
  - 0: 출력을 썼다(void·판정 불가가 섞여 있어도).
  - 2: 거부. 합성인데 `--allow-synthetic` 이 없음, boot 나 기록 신원 불일치, 일정표 label 불일치, 모르는 스키마, `--truth`·`--no-truth` 둘 다 없음. 거부일 때는 `{"result":"refused","why"}` 만 출력한다.
- 라이브러리 입구는 `m_entry.analyze(rec_rows, imu_rows, truth_rows|None, profile, cfg, up, *, allow_synthetic) -> dict` 다. CLI 는 파일을 읽고 sha256 을 계산하고 exit code 를 정하는 일만 한다.

## 6. 시험 목록(먼저 쓴다; 실행은 ws1 pytest)

**합성 생성기.** `rehearsal_imu_truth.py:56-79` 에 인라인으로 있는 생성 코드를 `forklift_core.plant.m_synth` 로 옮긴다. 리허설 설계 2.5(55 행)가 이 모듈을 이미 가리키지만 파일이 없다. 옮긴 뒤 리허설 도구는 그 모듈을 부른다.

생성기 `synth_trial(profile, cfg, *, ...)` 는 다음을 만든다.
- 일정표를 따르는 STATE·`profile` 행: `state_row`(`record_rows.py:13`)와 `_row` 형식을 쓴다.
- IMU 행, `truth/1` 행.

조절할 수 있는 참 plant 값:

| 인자 | 뜻 |
|---|---|
| `true_radius_scale` | 바퀴 반경 오차 |
| `true_yaw_per_m` | 직진 드리프트 |
| `gain` | 곡률 이득 |
| `q_offset` | 명령 q − 측정 q |
| `delay_s` | yaw 응답 지연(1 차) |
| `up` | 장착 축(숙임 포함) |
| `bias`, `bias_step` | 자이로 편향, 편향 계단 |
| `arw` | 각도 랜덤워크 |
| `still_s` | 정지 길이 |
| `void` | void 시행 |
| `seed` | 난수 씨앗 |

머리 줄은 `synthetic` 을 켜고 끌 수 있다(기본 끔).

| 이름 | 준비 | 단언 |
|---|---|---|
| `test_m1_recovers_drift_and_radius` | m1_fwd_v010, 드리프트 0.003 rad/m, 반경 1.02, 참값 정확 | d = −0.003(±1e-6), `radius_scale` 1.02, 횡 오차 ≈ d·s²/2 |
| `test_m1_imu_with_tilted_up` | 숙임 0.10 rad 의 up, 편향 3e-4 | IMU Δθ 가 참과 하한의 3 배 안. 숙임을 무시한 up 을 주면 cos 배만큼 틀린다 |
| `test_m2_gains_fwd_rev` | gain 0.9, `q_offset` 0.005, 전진·후진 | `gain_measured_q` 0.9, `gain_commanded_q` = 0.9·κ(meas)/κ(cmd), 후진의 부호, `distance_source` "truth" |
| `test_m2_under_reached_recomputed` | max 일정표, q 가 목표의 0.03 아래에서 멈춤 | `under_reached` 참, 시행은 유효 |
| `test_m3_merge_errors_at_stop` | 합류 뒤 오도메트리 yaw 오차 0.004 rad, 횡 3 mm | e_θ −0.004, e_y −0.003, `substitute_pending_lead` |
| `test_m3_l21_merge_end_not_judgeable` | 2.1 m 합류, 책상 N | 합류 끝 IMU 하한 > 1 mrad → `lower_bound_over_budget`(계산으로 나옴) |
| `test_m3_qstep_delay` | 엔코더 yaw 지연 0.083 s | 지연 0.083 ± 0.005 s. 사후 창이 맞지 않으면 사유가 붙은 void |
| `test_void_trials_excluded` | 시행 2 `void:deadman`, 시행 3 창 안에 엔코더 간격, 시행 4 `ENCODER_JUMP_L` | 셋 다 값 없음. `counts` 에 사유별로 세어짐 |
| `test_field_void_from_truth` | `t:"void"` 행 | 그 시행 void(`field:...`) |
| `test_synthetic_refused_without_flag` | 머리 줄 synthetic | exit 2, `refused`, 다른 출력 없음 |
| `test_pty_header_unmarked_refused` | `device_id.major` 가 pty, 표지 없음 | exit 2 |
| `test_synthetic_marked_with_flag` | 위 둘 + `--allow-synthetic` | 맨 앞에 `synthetic_inputs`·`not_for_judgement` |
| `test_host_boot_mismatch_refused` | IMU boot ≠ 기록 boot | exit 2 |
| `test_truth_record_mismatch_refused` | `truth.record.start_mono_ns` 다름, 또는 `body_point` 다름 | exit 2 |
| `test_no_truth_encoder_label` | `--no-truth` | `distance_source:"encoder"`, 모든 값 `judgeable:false, why:"no_truth"` |
| `test_truth_missing_trial_void` | 참값 파일에 시행 1 행만 있음 | 시행 2 `void:truth_missing` |
| `test_rehearsal_truth_heading_only` | `rehearsal_truth/1` | 플래그 없으면 exit 2. 있으면 heading 은 참값, 거리는 encoder |
| `test_quiet_window_absent` | `still_s` = 5 | IMU 항목 `no_bias_window`, 시행 유효, M1 d 는 그대로 |
| `test_bias_jump` | `bias_step` 2e-4 rad/s | `bias_jump`, 시행 유효 |
| `test_profile_label_mismatch_refused` | `--profile` 이 다른 일정표 | exit 2 |
| `test_registered_constants` | 출력의 `constants` | 등록 2.2·2.3·1.1 값과 같다 |
| `test_s_m_per_trial` | 시행 둘의 `s_m` 이 다름 | 각 시행의 `s_m_profile` 이 제 시행 값이다(세션 스크립트 빈틈 1) |

## 7. 확인 필요

1. **등록이 초안이다**(공동 등록 전, 등록 6 행). 도구는 `revision`·`status:"draft"` 를 출력한다. 등록이 확정된 뒤 상수가 바뀌면 판을 올린다. 0.025·0.15 는 10-23 실기록으로 아직 확인하지 않았다(2.2, 168-169 행).
2. **참값이 시행 하나에만 없을 때**
   - 등록 1.1(72 행)은 그 시행을 void 로 본다. 설계 3 절(73 행)은 "엔코더 거리 기준으로 표시" 라 한다.
   - 제안: 참값 파일이 있는데 그 시행만 없으면 void. 파일 자체가 없을 때(`--no-truth`)는 엔코더 표시를 하고 모두 판정 불가로 낸다. 총괄 결정이 필요하다.
3. **서 있음 문턱**(바퀴 속도)은 등록에 없다. 후보: 다리의 시작 조건과 같은 `stop_speed_mps`. 리허설은 1e-3 rad/s 를 썼다. 조향이 움직이는 것을 서 있음에서 뺄지도 정해야 한다.
4. **오도메트리 `max_gap_s`**: 리허설은 0.2 s 를 썼고 등록에는 값이 없다. D5 PR 의 MCU 간격 판정(150 ms)과 맞출지 정해야 한다.
5. **Δb 의 정의**: 등록 2.3 의 σ 는 y 한 축의 N 으로 셈했다. 제안은 û 투영이다. `bias_between` 의 3 축 노름은 보조값으로만 남긴다.
6. **`step_response_delay` 의 `q_threshold`** 가 등록에 없다. 후보는 0.02 rad(`PRESTEER_TOL_RAD`)나 계단의 10 %(0.013) 다.
7. **현장 void 를 어디에 적을지**: 참값 파일의 `void` 행으로 할지 점검표 파일을 따로 둘지. "결과 전에 닫음" 을 어떻게 증명할지(파일 시각, 해시)도 정해야 한다.
8. **다리 기록 머리 줄**에 일정표 이름·sha256 을 넣는 작은 다리 변경이 필요하다(별도 PR). 그 전까지는 label 집합만으로 대조한다.
9. **G3 출력 스키마**: `--mount` 가 읽을 필드 이름, 그리고 G3 와 기록의 boot·날짜를 대조할지.
10. **A 의 바닥 표시 → 뒤축 중심 자세 변환**: 지시봉 위치에서 뒤축 중심을 구하는 일이다. G9 쪽 도구이고, `truth/1` 은 변환된 자세만 받는다.
11. **M3 판정점**: A 의 `cont` 끝 정지 자세로 대신하는 것은 총괄 승인 대기다(5.1). 승인되지 않으면 M3 합류 출력은 모두 보고값이 된다.
12. **교차검증 전**: Codex 적대적 검토와, 그 결과를 보지 않은 독립 검토(새 Codex 스레드나 agy)를 구현 전에 거쳐야 한다.
