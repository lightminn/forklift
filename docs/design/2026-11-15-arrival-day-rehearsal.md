# 도착일 리허설 — 실차 없이 절차서를 모형 MCU 로 끝까지 돌리기 (2026-11-15)

- 근거: 총괄 배정·결정(2026-11-15). 목표는 [도착일 절차서](../plans/2026-11-12-arrival-day-procedure.md)의 소프트웨어 단계를, 운용자가 치는 명령 그대로 처음부터 끝까지 한 번 돌려 단계 사이의 통합 결함(파일 경로·형식·인자·순서·게이트)을 찾는 것이다. 결과는 단계별 명령·출력·결함 기록 문서와, 결함마다 작은 PR 이다.
- 합성 plant 라 곡률·지연·미끄럼 값은 뜻이 없다. 경로·게이트·형식만 검증한다.

## 1. 범위

| 절차서 | 리허설 |
|---|---|
| 6 S2 링크·센서(record, 손 회전 → 모형 회전, cpr, G, kth, 재무장 보류) | 한다 |
| 7 S3 축별 개루프(step, `--allow-locked` 는 모형 전용) | 한다 |
| 8 S4 commission(`--markers`) | 한다 |
| 9.1 S5 지금 할 수 있는 안전 시나리오(hold_disarm 등) | 모형에서 되는 것만 한다 |
| 12 M1–M3: G1·G5·G7 텔레옵 기록·clock fit → G6 일정표 → G8 분석(G3 포함) | 한다(대표 일정표 M1·M2·M3 합류 각 1 개, G3 원호 1 개) |
| S0·S1(굽기·전원), 실 D435i·A2M12, G9 참값 측정, 보드 고유 동작 | 하지 않는다 |
| **G4·7b(스캔 → 다리 바닥 허가) 연결부** | **시험하지 못한다**(7b 는 A2M12 실측 뒤 설계) |

## 2. 구성 요소

### 2.1 모형 실행기 `tools/rehearsal_bench.py`

- `firmware/` 를 `-DFW_MCU_MODEL=ON` 으로 빌드(시험 fixture `mcu_model` 과 같은 명령, 이미 있으면 재사용)하고, `os.openpty()` 의 짝을 모형에 주어 띄운다. 모형 인자(commission 빌드·출력 잠금·`--estop-at-ms` 등)는 절차서 단계별 이름(`--stage s2|s3|s4|s5|drive`)으로 고른다.
- 운용자에게 장치 경로(`/dev/pts/N`)를 알려 주고, 확인용 기록 `bench.json`(`{"schema": "rehearsal_bench/1", "pts", "model_pid", "model_exe", "model_args", "started_mono_ns", "synthetic": true}`)을 쓴다. Ctrl-C·TERM 이면 모형을 끝내고 `bench.json` 을 지운다.

### 2.2 모형 상대 확인 `forklift_core.live.model_peer`

`model_peer_refusal(device, bench) -> str | None` — None 일 때만 합성 대역을 켠다. 세 조건이 모두 맞아야 한다(총괄 승인 (가)):
1. `device` 를 따라간(stat -L) 문자 장치의 주번호가 Unix98 pty slave(136–143)이다. USB CDC(`ttyACM*`, 주번호 166)·UART(`ttyS*`·`ttyUSB*`, 4·188)·`/dev/forklift-mcu` 류는 `not_a_pty` 로 거부한다.
2. `bench.json` 의 `model_pid` 가 살아 있고 `/proc/<pid>/exe` 가 `bench.json` 의 `model_exe` 와 같으며 그 파일 이름이 `mcu_model` 이다.
3. 그 프로세스의 `/proc/<pid>/fd/*` 가운데 하나가 `device` 와 같은 장치(st_rdev)를 가리킨다.
- `/proc` 와 stat 은 주입할 수 있게 해 시험에서 USB CDC·UART·죽은 pid·다른 exe·짝을 안 연 경우를 단언한다.

### 2.3 모형 전용 허가 대역(synthetic_permit) — `tools/teleop_bridge.py --synthetic-permit BENCH_JSON`

- 시작할 때 `model_peer_refusal(--device, bench)` 가 None 이 아니면 그 사유를 내고 exit 2(인자 자체를 거부). 실차 장치에서는 1 번 조건으로 늘 거부된다.
- 켜지면 바닥 허가 함수가 `no_scan` 대신 `(1.0, "synthetic_permit")` 을 돌려준다. 1 s 마다 2·3 번 조건을 다시 보고, 어긋나면 그 뒤로 `(0.0, "synthetic_permit_lost")` 로 닫는다(모형이 죽거나 장치가 바뀜).
- 다리의 다른 게이트(무장·리스·명령 나이·STATE 나이·S6 속도 감시·운용자 확인)는 그대로다.
- 기록: 머리 줄에 `"synthetic": {"permit": true, "bench": <bench.json 내용>}`, 시작 출력의 `floor_permit` 에 `synthetic_permit (rehearsal, model MCU only)`. 이 기록을 읽는 분석(G8)은 머리 줄의 `synthetic` 이 있으면 결과에 `"synthetic_inputs"` 를 붙인다(판정용 아님 표시).
- "운영 런처": 지금 텔레옵 다리를 띄우는 런처는 저장소에 없다(다리를 직접 띄움). 위 장치 확인이 실차 경로의 거부이고, 앞으로 생길 운영 런처는 이 인자를 넘기지 않는다는 규칙을 다리 도움말과 이 문서에 적는다.

### 2.4 운용자 대역 `tools/rehearsal_operator.py`

- 콘솔 웹 페이지 대신 WebSocket 으로 다리에 붙어 운용자가 하는 것을 순서대로 한다: 리스 요청, 무장(다리의 to_ready 순서), deadman 을 누른 텔레옵 메시지(속도 비 0, 일정표가 비를 다시 씀), 끝나면 deadman 을 놓음. `tools/live_console_probe.py` 의 연결·요청 부분을 쓴다.
- 다리의 Enter(일정표 시작)는 실행기가 다리를 pty 위에서 띄우고 그 pty 에 줄바꿈을 써서 넣는다(다리의 marker_reader 는 터미널만 받는다).

### 2.5 IMU·참값 대역(형식 확인 전용)

- IMU: `tools/imu_record.py` 와 같은 `imu_record` 형식(`{"t": "imu", "s", "dev_us", "ts_ms", "dom", "host_ns", "x", "y", "z"}`)으로, 다리 기록의 STATE 엔코더에서 낸 yaw rate 에 `plant/m_synth.py` 의 잡음·편향을 더해 쓴다. 머리 줄 `"synthetic": true`. G3 의 정지·원호 구간이 나오게 다리 기록의 시각을 따른다.
- 참값: 일정표 시행의 양 끝 정지 구간에서 다리 기록의 오도메트리 자세를 G9 참값 형식(등록 6.1 의 양 끝 자세)으로 쓴다. 머리 줄 `"synthetic": true`. G8 분석이 끝까지 흐르는지만 본다.

### 2.6 합성 scan/1 송신기(7a 이음새 전용)

- `tools/rehearsal_scan.py`: 빈 방(일정 거리 벽) scan/1 프레임을 ScanFeed 소켓으로 보낸다. 머리(head) JSON 에 `"synthetic": true`. `scan_check.py` 로 받아 속도·빔·각·시각 판정이 흐르는지 본다. **다리 허가와는 연결하지 않는다.**

## 3. 리허설 실행과 기록

- 실행 문서 `docs/validation/2026-11-1x-arrival-day-rehearsal.md`: 절차서의 절마다 친 명령(모형 경로로 바꾼 것 그대로), 출력 요약, 기대와 다른 점, 결함 번호. 결함마다 작은 PR(시험 먼저, Codex 검토, ws1 pytest).
- 리허설 기록 파일은 모두 `synthetic` 표지를 달고, 실측 기록 폴더(`artifacts/…`)가 아니라 리허설 출력 폴더에 둔다.

## 4. 시험(먼저 쓴다)

- `model_peer_refusal`: pty·살아 있는 mcu_model·짝을 엶 → None. USB CDC(주번호 166)·UART(4, 188) → `not_a_pty`. 죽은 pid·다른 exe·이름이 mcu_model 아님·짝을 안 엶 → 각 사유.
- 다리 `--synthetic-permit`: 거부 사유가 있으면 exit 2 와 사유, 기록 파일을 만들지 않음. 켜졌을 때 허가 `synthetic_permit`, 모형이 죽으면 `synthetic_permit_lost` 로 닫힘, 기록 머리 줄의 `synthetic`.
- G8: 입력 기록에 `synthetic` 이 있으면 결과에 `synthetic_inputs`.
- 운용자 대역·IMU·참값 대역: 모형 E2E 로 G6 시행 하나가 `done` 으로 끝나고, 그 기록으로 G8 이 결과를 내는지.

## 5. 기간·자원

- 이틀 안팎: 구성 요소와 시험 반나절–하루, 리허설 실행 반나절, 결함 PR 나머지. ws1 은 pytest 만 쓴다. 크레딧은 구성 요소 초안·시험에 opus 로 쓰고, 일할 잔액으로 막히면 직접 한다.

## 6. 다음 판 요구 사항 후보(기록만)

- mcu-link/4 의 모형 식별 비트: STATE flags 8 비트가 다 쓰여 모형과 실 보드를 STATE 로 가를 수 없다. 리허설은 장치 단 확인으로 대신한다(2.2). 펌웨어 세션의 다음 판 요구 사항(#469 계열) 후보로만 적고, 지금은 바꾸지 않는다(총괄 2026-11-15).
