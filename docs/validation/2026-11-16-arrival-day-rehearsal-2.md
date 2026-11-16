# 도착일 리허설 2 차 — 2026-11-16 main 으로 처음부터 끝까지

- 목적: 1 차 리허설([기록](2026-11-15-arrival-day-rehearsal.md)) 뒤 들어간 D1–D6·D5·기록 머리 줄·D4(M1–M3 분석 입구) 가 모두 든 트리로 도착일 절차를 한 번 더 돌려, 새 결함을 찾는다(총괄 지시 2026-11-16).
- 트리: origin/main(2026-11-16 오전, #545 머지 뒤). 모형 MCU(호스트 전용, 실제 펌웨어 코어 + 합성 plant)만 썼다. 모든 기록은 합성 표시가 붙고, 측정 도구는 `--allow-synthetic` 없이 거부한다.
- 노트북 상태: 배터리 방전(65 %), `power-saver`, governor `powersave`. 부하 발생기는 쓰지 않았다. 노트북의 지터·정체 절대값은 도착일 판단 근거가 아니다(총괄 2026-11-16).
- 실행: 세션 스크립트(단계마다 새 모형, `rehearsal_bench.py --stage …`). 10:58–11:13.

## 결과 요약

| 단계 | 결과 |
|---|---|
| S2 기록·표지(6.2) | `bringup.py record --markers` 표지 2 개. **터미널이 아니면 표지가 조용히 0 개**(R2-1) |
| S2 kth | `ok`(합성: 두 변형을 가를 프레임 없음 → `undecided`), `--allow-synthetic` 없으면 거부 |
| G7 clock | `ok`(S2 기록, 다리 기록 셋 모두) |
| S2 cpr(6.3) | `valid`, 왼쪽 327,680(모형 설정값), `gear` 없음 |
| 6.4 재무장 보류 | `ok`(출력 잠금 시운전 모형, stop grace 300), 확인 7 개 모두 참 |
| S3 commission L·M(7) | 둘 다 `ok`, 방향 `pos` → `duty_sign_ok`·`sensor_sign_ok` 참 |
| S4 step(8) | `ok`, 상승 0.05/0.07 s, 오버슈트 7.7 % |
| S5(9.1) | `limit_disarm`·`hold_disarm`·`tx_cut`·`disarm`·`expiry` 모두 `ok`(외부 측정은 `unavailable`), 링크 침묵 기록 + `analyze --kind iwdg --marker 0` 결과 냄 |
| G6 M1·M3 합류·q 계단(12) | **세 시행 모두 void**: MCU `COMMAND_TIMEOUT` → `mcu_stopping`(R2-3) |
| IMU·참값 대역 → G8(`m_analyze.py`) | IMU 기록은 받음. **참값 대역 `rehearsal_truth/1` 은 거부**(R2-2). `--no-truth` 로 돌리면 세 기록 모두 `result:"ok"`, void 를 사유와 함께 셈 |

1 차 때의 D1–D6 은 이번에 다시 나오지 않았다. 처음 실행에서 몇 단계가 거부된 것은 리허설 스크립트의 인자 실수였다(kth `--image`, commission `--power-switch-in-hand`, M 축 `--seconds` 0.1–0.2, S5 commission 시나리오의 `--axis`·`--duty`, 재무장 보류의 출력 잠금 모형). 도구는 설계대로 거부했고, 인자를 고쳐 다시 돌린 결과가 위 표다.

## 새 결함

| 번호 | 결함 | 영향 | 제안 |
|---|---|---|---|
| R2-1 | `bringup.py record --markers`(와 같은 표지 읽기)는 stdin 이 터미널이 아니면 설계대로 꺼진다(`markers.marker_reader`). 그런데 그 사실을 알리지 않아, 표지가 0 개인 기록이 조용히 남는다 | ssh 나 nohup 처럼 tty 없이 띄우면 S5 ②·전원 차단의 시각 표시를 잃는다 | 꺼질 때 한 줄 경고와 기록 머리 줄에 `markers: inactive (<사유>)` |
| R2-2 | 리허설 참값 대역(`rehearsal_imu_truth.py`)은 `rehearsal_truth/1` 을 쓰는데, `m_analyze` 는 `truth/1` 만 받는다. 입구 설계 2 절은 "옛 `rehearsal_truth/1` 은 `--allow-synthetic` 일 때만 heading 참값으로 읽는다" 고 정했지만 구현에 없다 | 리허설에서 참값을 넣은 G8 경로를 시험할 수 없다(실차에는 영향 없음) | 대역이 `truth/1`(method `synthetic`)을 쓰게 하거나, 입구가 설계대로 옛 형식을 읽게 한다 |
| R2-3 | **G6 시행 셋이 모두 void.** 다리 주기는 정상(40 ms 넘는 주기 0), MCU 는 DRIVE 를 20 ms 마다 받아들였고(거절 없음) 그런데도 `COMMAND_TIMEOUT` 이 났다. 계측판 대역으로 다시 돌린 M1 에서 대역의 텔레옵 송신 간격은 p50 51 ms 인데 p99 223 ms·최대 245 ms 였고, 정지 직전 233 ms 비었다 | 아래 "R2-3 의 기전" | 도착일 전에 실제 콘솔 단말의 송신 간격을 잰다(점검표). 대역은 시험 도구일 뿐이다 |

### R2-3 의 기전

- 다리가 보내는 DRIVE 의 만료는 min(결정 + 100 ms, 텔레옵 명령 기한, lease 끝) 이고, MCU 는 그보다 지연(≈ 20 ms)만큼 일찍 만료한다(D5 의 회색 구역 처리, 절차서 12.3b).
- 텔레옵 명령 기한은 "콘솔이 되돌려 보낸 상태 틱의 송신 시각 + 0.30 s" 다(`arbiter.py` 565–568). 틱의 나이만큼 줄어드므로, 콘솔이 보낸 시각부터 보면 여유는 대략 0.2 s 다.
- 그래서 콘솔이 약 0.2 s 넘게 텔레옵을 보내지 않으면, 다리는 아직 명령이 살아 있다고 보는 동안 MCU 가 먼저 멈추고 시행이 void 된다. 이번 정지들은 모두 그 간격(200–245 ms)에서 났다.
- 노트북 대역(asyncio·websocket, power-saver)이 50 ms 주기를 자주 200 ms 넘게 놓쳤다. 실제 콘솔은 브라우저라서 사정이 다르지만, **탭이 백그라운드로 가면 타이머가 1 s 단위로 느려질 수 있어** 같은 일이 난다.
- 설계를 바꾸는 결함은 아니다(명령 기한은 안전 설계값). 도착일의 운용 조건이다: 콘솔 단말을 앞쪽 탭·화면 켬·유선 또는 AP 근거리로 두고, 송신 간격을 측정 전에 확인한다.

## 근거 파일

세션 임시 폴더에 둔 실행 출력(커밋하지 않음, 판정 근거 아님): 단계별 `*.out`, 다리 기록 `drive_*/teleop.jsonl`, 계측판 대역의 `drive_t/drive.json`(`teleop_send_ns`).
