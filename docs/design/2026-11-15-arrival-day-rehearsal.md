# 도착일 리허설 — 실차 없이 절차서를 모형 MCU 로 끝까지 돌리기 (2026-11-15)

- 근거: 총괄 배정·결정(2026-11-15). 목표는 [도착일 절차서](../plans/2026-11-12-arrival-day-procedure.md)의 소프트웨어 단계를, 운용자가 치는 명령 그대로 처음부터 끝까지 한 번 돌려 단계 사이의 통합 결함(파일 경로·형식·인자·순서·게이트)을 찾는 것이다. 결과는 단계별 명령·출력·결함 기록 문서와, 결함마다 작은 PR 이다.
- 합성 plant 라 곡률·지연·미끄럼 값은 뜻이 없다. 경로·게이트·형식만 검증한다.

## 1. 범위

| 절차서 | 리허설 |
|---|---|
| 6 S2 링크·센서(record, 손 회전 → 모형의 손 회전 궤적, cpr, G, kth, 재무장 보류) | 소프트웨어 경로·기록 형식만. VM 투입·전기 측정·PB4/SPI·실 센서 판정은 못 함 |
| 7 S3 축별 개루프(step, `--allow-locked` 는 모형 전용) | 명령·STATE 경로만. 실제 모터·드라이버 전류·회전 방향·기구는 못 함 |
| 8 S4 commission(`--markers`) | 도구 경로만(절차서의 정상 빌드 폐루프는 다음 PCB 판까지 보류라 대신하지 못함) |
| 9.1 S5 지금 할 수 있는 안전 시나리오 | 모형 인자로 되는 상태 전이만(hold_disarm, 링크 침묵 `--silent-at-ms`, e-stop `--estop-at-ms`, 결함 `--fault-at-ms`). 손 전원 차단·외부 리셋/IWDG 계측·실제 정지 거리는 못 함 — 빠진 시나리오를 결과 문서에 목록으로 |
| 12 M1–M3: G1·G5·G7 텔레옵 기록·clock fit → G6 일정표 → G8 분석(G3 포함) | 분석 파이프라인과 기록 형식만(대표 일정표 M1·M2·M3 합류 각 1 개, G3 원호 1 개). 합성 IMU·참값이라 독립 참값·장착·편향·물성·바닥은 확인하지 못하고, 결과는 측정·등록 판정 자료에서 배제(2.3 분석 차단) |
| S0·S1(굽기·전원), 실 D435i·A2M12, G9 참값 취득, 보드 고유 동작 | 하지 않는다 |
| **G4·7b(스캔 → 다리 바닥 허가) 연결부** | **시험하지 못한다**(7b 는 A2M12 실측 뒤 설계) |

## 2. 구성 요소

### 2.1 모형 실행기 `tools/rehearsal_bench.py`

- `firmware/` 를 `-DFW_MCU_MODEL=ON` 으로 빌드(시험 fixture `mcu_model` 과 같은 명령, 이미 있으면 재사용)한다.
- **pty 의 끝 배치(Codex 설계 검토 P1):** 다리(`ReconnectingLink` → `open_raw(device)`)는 경로로 여는 쪽, 곧 slave 를 연다. 그래서 모형이 **master** 를 쥐어야 한다. 실행기가 `os.openpty()` 로 쌍을 만들고 master fd 를 모형에 물려준다 — `mcu_model` 에 `--link-fd N`(이미 열린 fd 를 그대로 쓰는 인자, 기존 `--link PATH` 와 함께 둘 중 하나)을 더한다. 실행기는 slave 를 닫고 slave 경로(`/dev/pts/N`)만 운용자에게 알린다. 시험 fixture 는 모형이 slave, 호스트가 master 인 반대 배치라 그대로 둔다.
- 모형 인자는 절차서 단계 이름으로 고른다(`--stage`): `s2`(일반 빌드, 손 회전 궤적), `s3`(일반 빌드, `--allow-locked` 쪽 시험은 `--outputs-locked 1`), `s4`(`--commission-build 1`), `s5`(`--estop-at-ms` 등 시나리오별), `drive`(일반 빌드: 출력 열림·e-stop 배선 — 텔레옵이 무장되는 설정). 단계마다 모형을 새로 띄운다(설정 재사용 금지).
- 확인용 기록 `bench.json`: `{"schema": "rehearsal_bench/1", "pts", "pts_rdev", "model_pid", "model_starttime"(/proc/<pid>/stat 22 번째, pid 재사용 판별), "model_exe", "model_args", "stage", "synthetic": true}`. Ctrl-C·TERM 이면 모형을 끝내고 `bench.json` 을 지운다.

### 2.2 모형 상대 확인 `forklift_core.live.model_peer`

`model_peer_refusal(device, bench) -> str | None` — None 일 때만 합성 대역을 켠다. 모두 맞아야 한다:
1. `device` 를 따라간(stat -L) 문자 장치가 Unix98 pty slave(주번호 136–143)이고 그 st_rdev 가 `bench.json` 의 `pts_rdev` 와 같다. USB CDC(`ttyACM*`, 166)·UART(`ttyS*` 4, `ttyUSB*` 188)·`/dev/forklift-mcu` 류는 `not_a_pty` 로 거부한다.
2. `model_pid` 가 살아 있고 `/proc/<pid>/stat` 의 시작 시각이 `model_starttime` 과 같으며 `/proc/<pid>/exe` 가 `model_exe`(파일 이름 `mcu_model`)다.
3. 그 프로세스가 `/dev/ptmx` fd 를 쥐고 있고, 그 fd 의 `/proc/<pid>/fdinfo/<fd>` `tty-index` 가 `device` 의 pts 번호(소번호)와 같다 — 곧 **다리가 여는 slave 의 짝 master 를 모형 자신이 쥐고 있다.**
4. 모형 말고는 같은 `tty-index` 의 master 를 쥔 프로세스가 없다(socat 같은 중계가 끼면 거부). **완전 열거, 실패면 닫음(Codex 2 회차 P2):** `/proc` 의 모든 pid 를 돌고, 같은 uid 의 프로세스는 `fd`·`fdinfo` 를 끝까지 읽어야 한다 — 읽기 오류나 순회 중 사라진 프로세스가 하나라도 있으면 한 번 다시 돌고, 그래도 생기면 거부(`peer_scan_incomplete`)한다. 구현에서 확인한 두 예외(실제 데스크톱에서 엄격 규칙은 늘 거부했다): ① 같은 uid 라도 non-dumpable 이라 fd 를 읽을 수 없는 세션 구성 요소(sd-pam, systemd --user, kwin_wayland, polkit 에이전트, v4l2-relayd)는 **모형보다 먼저 시작했을 때만** 넘어가고(그 뒤 우리 프로세스가 fd 를 넘겨야만 이 master 를 쥘 수 있다 — 남는 한계), 모형 뒤에 시작한 읽을 수 없는 프로세스는 거부한다. ② 목록과 읽기 사이에 닫힌 fd 하나는 지금 아무것도 쥐지 않으므로 뺀다(이 프로세스가 목록을 읽는 디렉터리 fd 가 늘 그렇다). 프로세스 자체가 사라지면 다시 돈다. 다른 uid 의 프로세스는 읽을 수 없다; pty master 는 만든 프로세스가 fd 를 넘겨야만 얻을 수 있으므로 다른 uid 가 쥐는 경우는 우리 프로세스가 넘긴 경우뿐이다 — 이 가정을 남는 한계로 적고, 실행기는 root 로 띄우지 않는다(root 로 띄우면 거부). **총괄 결정(2026-11-15, Codex 구현 검토 P1 에 대해):** 모형보다 먼저 시작한 non-dumpable 프로세스가 fd 를 넘겨받아 실 MCU 로 중계하는 경로는 누군가 일부러 중계를 꾸며야만 생기므로 한계로 둔다. 실차 보장은 "pty 가 아니면 거부" 와 "운영 경로 인자 거부" 가 맡는다. 그 경우까지 거부하면 데스크톱에서 늘 거부돼 리허설이 불가능하다.
- 이 조건들은 실 MCU 가 pty 중계 뒤에 있는 경우를 막는다: 중계에서는 다리 쪽 master 를 중계 프로세스가 쥐므로 3 이 어긋난다.
- `/proc`·stat 은 주입할 수 있게 해 시험에서 USB CDC·UART·죽은 pid·재사용된 pid(시작 시각 다름)·다른 exe·master 를 안 쥠·다른 프로세스도 쥠·rdev 다름을 각각 단언한다.

### 2.3 모형 전용 허가 대역(synthetic_permit) — `tools/teleop_bridge.py --synthetic-permit BENCH_JSON`

- 시작할 때 `model_peer_refusal(--device, bench)` 가 None 이 아니면 그 사유를 내고 exit 2(인자 자체를 거부). 실차 장치에서는 1 번 조건으로 늘 거부된다.
- 켜지면 바닥 허가 함수가 `no_scan` 대신 `(1.0, "synthetic_permit")` 을 돌려준다. 1 s 마다, 그리고 `ReconnectingLink` 가 다시 열 때마다 1–4 번 조건 전부(경로가 가리키는 장치의 st_rdev 포함)를 다시 보고, 한 번이라도 어긋나면 그 세션 동안 `(0.0, "synthetic_permit_lost")` 로 닫는다(다시 열리지 않음).
- 다리의 다른 게이트(무장·리스·명령 나이·STATE 나이·S6 속도 감시·운용자 확인 네 개 `--empty-forks --forks-clear --lift-low --no-open-recovery`)는 그대로다. 우회하지 않는다 — 모형 `drive` 설정(출력 열림·e-stop 배선)에서 이 게이트들이 통과하는지 자체가 리허설의 확인 대상이고, 막히면 결함으로 기록한다.
- 기록: 머리 줄에 `"synthetic": {"permit": true, "bench": <bench.json 내용>}`, 시작 출력의 `floor_permit` 에 `synthetic_permit (rehearsal, model MCU only)`.
- **모든 기록 작성자의 표지(Codex 2 회차 P2):** 기록을 쓰는 도구(`bringup.py record`·`step`·`commission`·`scenario`, `teleop_bridge.py --record`)는 `--device` 가 pty 이면 `--bench BENCH_JSON` 을 요구하고 `model_peer_refusal` 이 None 일 때만 열며, 그때 머리 줄에 `"synthetic": {"bench": …}` 를 늘 넣는다. pty 인데 bench 가 없거나 확인이 실패하면 거부한다(실 MCU 는 pty 로 붙지 않는다). 실 장치(ttyACM 등)의 기록에는 표지가 없다.
- **분석 진입점의 차단(Codex P2):** 합성 표지를 읽는 공용 함수 `forklift_core.live.synthetic.refuse_or_mark(header, allow_synthetic)` 를 두고(머리 줄의 `device` 가 pty 경로인데 `synthetic` 이 없는 기록도 합성으로 본다 — 표지가 빠진 옛 리허설 기록 대비), 기록을 읽는 모든 측정 진입점 — `bringup.py analyze`·`clock`·`kth`, G8 의 기록 적재(텔레옵 기록·IMU·참값), 등록 판정 도구 — 이 머리 줄에 `synthetic` 이 있으면 기본으로 거부(exit 2)한다. 리허설에서는 `--allow-synthetic` 를 줘야 돌고, 그 출력의 맨 위에 `"synthetic_inputs": true` 와 `"not_for_judgement": true` 를 붙인다. 여러 기록을 합치는 분석은 하나라도 합성이면 같은 규칙이다.
- "운영 런처": 지금 텔레옵 다리를 띄우는 런처는 저장소에 없다(다리를 직접 띄움). 위 장치 확인이 실차 경로의 거부이고, 앞으로 생길 운영 런처는 이 인자를 넘기지 않는다는 규칙을 다리 도움말과 이 문서에 적는다.

### 2.4 운용자 대역 `tools/rehearsal_operator.py`

- 콘솔 웹 페이지(`console/web`)가 하는 순서를 `console/1` WebSocket 으로 그대로 한다(다리 E2E 시험 `to_ready` 와 같은 순서): `lease_request` → 서 있음 확인을 기다림 → `mode_request(to=TELEOP)` → `arm` → `phase == ready` 를 기다림. 그 뒤 deadman 을 누른 `teleop` 메시지(속도·곡률 비 0, 일정표가 비를 다시 씀)를 `TELEOP_MIN_INTERVAL_MS` 보다 짧지 않게 계속 보내고, 시행이 `zeroing` 으로 끝나면 deadman 을 놓는다. STOPPED 면 서서 `rearm` 한다.
- 응답마다 기록(보낸 시각·응답·사유)하고, 거부나 시간 초과는 리허설 결함 후보로 남긴다. live_console_probe 를 통째로 쓰지 않고, 그 연결·요청 부분(wire 모듈)만 쓴 작은 클라이언트로 만들고 모형 E2E 시험으로 위 순서와 deadman 끊김 → `void:deadman` 을 단언한다.
- 다리의 Enter(일정표 시작)는 실행기가 다리를 pty 위에서 띄우고 그 pty 에 줄바꿈을 써서 넣는다(다리의 marker_reader 는 터미널만 받는다). 운용자 대역은 `ready` 와 deadman 을 확인한 뒤 실행기에 Enter 를 요청한다.

### 2.5 IMU·참값 대역(형식 확인 전용)

- IMU: `tools/imu_record.py` 와 같은 `imu_record` 형식(`{"t": "imu", "s", "dev_us", "ts_ms", "dom", "host_ns", "x", "y", "z"}`)으로, 다리 기록의 STATE 엔코더에서 낸 yaw rate 에 `plant/m_synth.py` 의 잡음·편향을 더해 쓴다. 머리 줄 `"synthetic": true`. G3 의 정지·원호 구간이 나오게 다리 기록의 시각을 따른다.
- 참값: 일정표 시행의 양 끝 정지 구간에서 다리 기록의 오도메트리 자세를 G9 참값 형식(등록 6.1 의 양 끝 자세)으로 쓴다. 머리 줄 `"synthetic": true`. G8 분석이 끝까지 흐르는지만 본다.

### 2.6 합성 scan/1 송신기(7a 이음새 전용)

- `tools/rehearsal_scan.py`: 빈 방(일정 거리 벽) scan/1 프레임을 ScanFeed 소켓으로 보낸다. 머리(head) JSON 에 `"synthetic": true`. `scan_check.py` 로 받아 속도·빔·각·시각 판정이 흐르는지 본다. **다리 허가와는 연결하지 않는다.**

## 3. 리허설 실행과 기록

- 실행 문서 `docs/validation/2026-11-1x-arrival-day-rehearsal.md`: 절차서의 절마다 친 명령(모형 경로로 바꾼 것 그대로), 출력 요약, 기대와 다른 점, 결함 번호. 결함마다 작은 PR(시험 먼저, Codex 검토, ws1 pytest).
- 리허설 기록 파일은 모두 `synthetic` 표지를 달고, 실측 기록 폴더(`artifacts/…`)가 아니라 리허설 출력 폴더에 둔다.

## 4. 시험(먼저 쓴다)

- `model_peer_refusal`: 열거 중 같은 uid 프로세스의 읽기 오류·사라짐·fd 교체가 다시 돌아도 생기면 `peer_scan_incomplete`, root 로 실행하면 거부. pty·살아 있는 mcu_model·그 slave 의 master 를 모형만 쥠 → None. USB CDC(166)·UART(4, 188) → `not_a_pty`. rdev 다름, 죽은 pid, 시작 시각 다른 pid(재사용), 다른 exe, 이름이 mcu_model 아님, master 를 안 쥠, 다른 프로세스도 같은 master 를 쥠 → 각 사유. 실제 모형을 `--link-fd` 로 띄운 통합 시험 하나.
- 다리 `--synthetic-permit`: 거부 사유가 있으면 exit 2 와 사유, 기록 파일을 만들지 않음. 켜졌을 때 허가 `synthetic_permit`, 모형이 죽으면 `synthetic_permit_lost` 로 닫힘, 기록 머리 줄의 `synthetic`.
- 분석 진입점: 합성 머리 줄이면 `--allow-synthetic` 없이는 exit 2, 있으면 결과 맨 위 `synthetic_inputs`·`not_for_judgement`(bringup analyze·clock·kth, G8 적재, 여러 기록 합침).
- 다리 재연결: 경로가 다른 장치를 가리키게 바뀌면 허가가 `synthetic_permit_lost` 로 닫히고 다시 열리지 않음. 실행 중 재검사에서 열거가 불완전해도 닫힘.
- 기록 작성자: pty 장치에 bench 없음·확인 실패 → 거부, 통과 → 머리 줄 `synthetic`. 실행기가 master fd 를 `pass_fds` 로 모형에 물려주고 자신은 닫는지(모형만 쥠).
- 운용자 대역·IMU·참값 대역: 모형 E2E 로 G6 시행 하나가 `done` 으로 끝나고, 그 기록으로 G8 이 결과를 내는지.

## 5. 설계 검토

- Codex 적대 1 회차: P1 pty 끝 배치(다리와 모형이 같은 slave 를 열 뻔함) → 모형이 master 를 쥐는 `--link-fd`, 확인 3·4 를 master 소유·단독으로 바꿈. P2 중계·재연결·pid 재사용 → 조건 1 에 rdev 대조, 2 에 시작 시각, 4 에 단독 소유, 재연결마다 전부 재검사·한 번 어긋나면 세션 동안 닫음. P2 합성 표지가 분석을 막지 못함 → 측정 진입점 기본 거부와 `--allow-synthetic`. P2 게이트 → 단계별 모형 인자와 우회 금지를 적음. P2 운용자 대역 프로토콜 → 순서·간격·끊김을 적고 E2E 로 단언. P3 범위 → 1 절 표를 절마다 좁힘.
- Codex 2 회차: P2 열거 불완전을 없음으로 취급 → 같은 uid 완전 열거·실패면 닫음·root 거부·다른 uid 가정을 한계로. P2 모든 기록의 표지 → pty 장치면 모든 기록 작성자가 bench 확인 뒤 표지, 분석은 pty 기록의 표지 누락도 합성으로 봄.

## 6. 기간·자원

- 이틀 안팎: 구성 요소와 시험 반나절–하루, 리허설 실행 반나절, 결함 PR 나머지. ws1 은 pytest 만 쓴다. 크레딧은 구성 요소 초안·시험에 opus 로 쓰고, 일할 잔액으로 막히면 직접 한다.

## 7. 다음 판 요구 사항 후보(기록만)

- mcu-link/4 의 모형 식별 비트: STATE flags 8 비트가 다 쓰여 모형과 실 보드를 STATE 로 가를 수 없다. 리허설은 장치 단 확인으로 대신한다(2.2). 펌웨어 세션의 다음 판 요구 사항(#469 계열) 후보로만 적고, 지금은 바꾸지 않는다(총괄 2026-11-15).
