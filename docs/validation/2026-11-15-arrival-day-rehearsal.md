# 도착일 리허설 기록 (2026-11-15, 1 차)

- 설계: [리허설 설계](../design/2026-11-15-arrival-day-rehearsal.md). 모형 MCU(`tools/rehearsal_bench.py`, mcu_model 이 pty master)에 절차서 도구를 그대로 붙였다. 모든 기록은 synthetic 표지가 있고 측정·판정에 쓰지 않는다.
- **범위 밖:** G4·7b(스캔 → 다리 바닥 허가) 연결부는 시험하지 못했다 — 다리는 모형 전용 허가 대역(synthetic_permit)으로 돌았다. S0·S1, 실 D435i·A2M12, G9 참값 취득, 보드 고유 동작도 범위 밖이다.

## 결함

| 번호 | 단계 | 결함 | 처리 |
|---|---|---|---|
| D1 | 12 G6 | 다리가 시행 끝에 `{"t": "profile_end", "trial", "result"}` 행을 쓰지 않는다(G6 설계 2 절은 쓴다고 함). `m_analysis.trials()` 는 `profile` 행의 `result` 로 시행을 나누므로 G8 은 깨지지 않는다 — 설계와 구현의 불일치(낮음) | PR 예정: 다리가 시행 끝에 한 번 쓰고 시험으로 단언(설계대로) |
| D3 | 12 G6→G8 | **도착일 기록을 G8 이 읽지 못했다.** 일정표 실행기가 구간이 끝나는 주기의 행에 다음 구간 번호(`seg`)와 끝난 구간의 label 을 함께 써서, `m_analysis.trials()` 가 "label 이 두 구간" 으로 모든 다구간 시행을 거부했다. 단위 시험은 합성 행만 써서 못 잡음 | PR `fix/profile-row-seg`(D1 과 함께): 행이 그 주기의 구간을 가리키게, 시험으로 trials() 통과 단언 |
| D2 | 6 S2 kth | 합성 기록을 `--allow-synthetic` 로 읽으면 출력 맨 위 `synthetic_inputs: true` 와 함께 kth 자신의 `"synthetic": false`(HIL_PLANT 비트로 정함)가 나와 서로 어긋나 보인다 | PR 예정: 머리 줄이 합성이면 kth 의 `synthetic` 도 참, 판정은 `synthetic` |
| D4 | 12 G8 | G8 을 기록에 돌리는 입구(CLI·적재기)가 없다. 기록 행 → `McuState`(`sample_from_row(r).state`), IMU 행 → `imu_stream`, 서 있는 구간·창 계산을 매번 손으로 이어야 한다. 리허설은 세션 스크립트로 이었다 | `tools/m_analyze.py`(등록 3–5 절 계산, 합성 기본 거부) — D5 다음(총괄) |
| D5 | 12 G6 | **구조적 민감성 — 실제 다리에서도 넘음(아래 측정).** 호스트는 새 STATE 마다 직전 수신 이후 간격을 먼저 판정하고(`mcu_host._on_state` → `_expire`) 수신 시각이 다리 주기 시각이라, 다리 루프가 약 30 ms(= 50 ms 문턱 − 주기 20 ms) 넘게 멈추면 링크가 멀쩡해도 `stale` → 무장 해제(모형 E2E 하네스에서 30 ms 멈춤으로 결정적 재현). 리허설 M1 시행을 무효로 만든 멈춤은 **리허설 도구 탓이었다** — `SyntheticPermit` 의 1 s 주기 전체 /proc 열거가 제어 루프 안에서 30–50 ms 걸렸다(감시 스레드 스택 19 번 중 16 번). 열거를 시작 때 한 번으로 옮기자 85 s 정지 시행이 `done`, 25 ms 넘는 주기는 1 번 | 리허설 도구 고침(전체 열거는 시작 때만, 루프는 `model_alive_refusal`). 실제 다리 측정에서 무부하에도 50 ms 를 넘어 총괄이 (가) MCU 시각 간격 판정으로 결정(호스트 상한 150 ms = MCU cap, USB 버퍼 지연 상한, 50 ms 문턱 불변), (나) 지터 줄이기는 후보로 설계 메모에 — D5 PR |
| D6 | 7a 스캔 | 합성 scan/1 머리의 `synthetic` 이 `scan_check` 의 요약·`--record` 에 실리지 않는다 | 낮음: scan_check 가 머리의 synthetic 을 요약·기록에 옮김 |

## 단계별 기록

### 6–9 bring-up(단계마다 새 모형)

| 단계 | 명령(장치는 bench 의 pty, `--bench` 붙임) | 결과 |
|---|---|---|
| 6.2 S2 기록 | `bringup.py record --seconds 12 --markers`(Enter 2 번) | 머리 줄 synthetic, 표지 2 개, STATE 기록 |
| 6.2 kth | `bringup.py kth --configured-m/h g704_bytes --allow-synthetic` | `ok`, 출력 `not_for_judgement`. 같은 명령에 `--allow-synthetic` 없으면 거부(exit 2). D2 |
| G7 clock | `bringup.py clock --allow-synthetic` | `ok`, 드리프트 0.67 ppm(모형), 지연 p50 1.08 ms |
| 6.3 cpr | `bringup.py cpr --turns-left 2 --turns-right 0`(모형 손 회전 2 바퀴) | `valid`, cpr 왼쪽 327,680(모형 설정값), `gear` 없음(`--motor-counts-confirmed` 안 붙임) |
| 6.4 재무장 보류 | `rearm_hold_check.py --stop-grace-ms 300 --limit-s 2.0`(모형 commission·출력 잠금·stop grace 300) | `ok`, 7 개 확인 모두 참. 처음엔 모형 stop grace(100)를 안 맞춰 도구가 거부 — 리허설 설정 실수 |
| 7 S3 commission L·M | `bringup.py commission --axis L --duty 0.05 --seconds 0.5`, `--axis M … --window -0.2 0.2` | 둘 다 `ok`(방향 질문에 운용자 대역이 `pos`). 앞 실행을 중단한 같은 모형에서 M 은 "not ready after ARM" — 재무장 보류 탓, 새 모형에서 `ok` |
| 8 S4 step | `bringup.py step --wheel left --target 0.8` | `ok`, 상승 0.04/0.06 s, 오버슈트 7.8 % (합성 plant) |
| 9.1 S5 | commission `--scenario limit_disarm`·`hold_disarm`(stop grace 300), `scenario --kind tx_cut|disarm|expiry`, 링크 침묵(`--silent-at-ms 6000`) + `analyze --kind iwdg` | 시나리오 다섯 모두 `ok`(소프트웨어 증거만). iwdg 는 모형이 리셋하지 않아 `new_boot` 없음 |

- 9.1 에서 못 한 것: 손에 든 주 전원 차단, 외부 리셋 버튼·IWDG 실제 리셋(모형은 침묵만 흉내), 실제 정지 시간·거리 외부 측정, 출력 반응 외부 측정(`unavailable` 로 넣음).


### 12 M1–M3 연결(모형 drive 단계)

- M1 일정표(`config/profiles/m1_m3/m1_fwd_v010.yaml`, 155 s) 를 `rehearsal_drive.py` 로 두 번: 1 번은 3.0 m 주행 뒤 정지 구간에서 `void:disarmed:mcu_stopping`, 2 번은 앞 정지 24 s 에서 `void:disarmed:mcu_armed` — 리허설 도구의 루프 안 열거가 만든 멈춤(D5). 고친 뒤 85 s 정지 시행은 `done`. 12 s 정지로 줄인 시행도 22.7 s 에 같은 무효. 5 s 정지·1 m 시행은 `done`.
- G7: `bringup.py clock --allow-synthetic` 이 다리 기록으로 `ok`(드리프트 −0.14 ppm, 모형).
- 대역: `rehearsal_imu_truth.py` 가 같은 기록에서 IMU(`imu_record/1`, 400/200 Hz)와 참값 파일을 씀(둘 다 synthetic).
- G8(등록 3.3 순서, 세션 스크립트): D3 고친 뒤 `trials(valid_only)` 1 시행, `odometry_heading` Δθ 0(모형 직진), `m1_drift` 계산됨, `heading_sigma_lower` 하한 1.07 mrad(3 s 창·10 s 주행). `quiet_window` 는 5 s 정지에서 창을 찾지 못함(등록의 60 s/50 s 규칙은 긴 정지가 필요한데 D5 가 막음). 오도메트리 연속성 `gap` 1 건(D5 의 멈춤).
- 7a: `rehearsal_scan.py`(빈 방, 1,600 빔, 10 Hz, synthetic 머리) → `scan_check.py` 100 개 보내 91 개 받음, 9 개는 시각 쌍 세우는 중 거부(설계대로), 주기 0.100 s, 빔·각 맞음. D6.

### 12 G6 — 짧은 직진(모형 drive 단계)

- 명령: `rehearsal_bench.py --stage drive --out-dir R` → `rehearsal_drive.py --bench R/bench.json --profile short.yaml --out R/drive1`
- 결과: 리스·TELEOP·무장 ack, 바닥 허가 `synthetic_permit`, Enter 로 시행 1 시작, 0.10 m/s 로 0.30 m 주행, 시행 `done`. 다리 주기 넘침 72 회(노트북).
- 기대와 다른 점: D1.

## D5 실제 다리 지터 측정 (2026-11-15)

노트북에서 1,080 s 동안 쟀다. 합성 허가 없이 실제 다리(`teleop_bridge.py`, `--bench` 기록만)를 리허설 모형 pty 에 붙였다. 콘솔 서버와 G1 기록을 켜고, 200 Hz IMU 기록 대역을 다른 프로세스로 돌렸다. 600–720 s 에는 18 코어를 모두 태우는 부하 창을 걸었다. 같은 시간에 Codex 와 다른 세션도 돌고 있었다. 차는 무장한 채 deadman 을 잡고 서 있었다. 세션 도구로 잰 것이고, 숫자는 다리 기록의 주기 시작 간격(`now_ns`)이다.

| 구간 | 주기 | p50 | p99 | p999 | 최대 | >30 ms | >40 ms | >50 ms |
|---|---|---|---|---|---|---|---|---|
| 전체 | 53,701 | 20.06 | 21.01 | 23.57 | 327.9 | 14 | 7 | 5 |
| 무부하 | 47,861 | 20.06 | 20.13 | 20.45 | 68.0 | 3 | 2 | 2 |
| 부하 창 | 5,840 | 20.08 | 23.3 | 37.45 | 327.9 | 11 | 5 | 3 |

- `stale` 은 5 번 났고, 간격이 50 ms 를 넘은 5 번과 같은 곳이다. 무부하 2 번은 839.3 s(57.6 ms)와 1065.2 s(68.0 ms)다. 무장 중이었다면 매번 무장이 풀린다.
- 이 실행의 무장 해제(643.2 s)는 다리 탓이 아니었다. 모형 자신의 STATE 가 MCU 시각으로 30.8 ms 비었고(같은 CPU 위에서 굶음) `no_feedback:rear_axle_speed` 가 났다. 실차의 MCU 는 따로 있으므로 리허설 고유 현상으로 본다. 그 뒤의 `stale` 은 모두 무장 해제 뒤다.
- 앞선 부분 실행(916 s, 중단되어 판정 근거 아님)도 같은 양상이다. 부하 창 644.7 s 에 166 ms 가 멈췄고, 무장 중이라 `stale` → `authorization_revoked` → 무장 해제까지 갔다. 무부하 838.7 s 에는 51.9 ms 가 나왔다.
- Jetson 측정은 아직 없다.

