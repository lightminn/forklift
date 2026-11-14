# 도착일 리허설 기록 (2026-11-15, 진행 중)

- 설계: [리허설 설계](../design/2026-11-15-arrival-day-rehearsal.md). 모형 MCU(`tools/rehearsal_bench.py`, mcu_model 이 pty master)에 절차서 도구를 그대로 붙였다. 모든 기록은 synthetic 표지가 있고 측정·판정에 쓰지 않는다.
- **범위 밖:** G4·7b(스캔 → 다리 바닥 허가) 연결부는 시험하지 못했다 — 다리는 모형 전용 허가 대역(synthetic_permit)으로 돌았다. S0·S1, 실 D435i·A2M12, G9 참값 취득, 보드 고유 동작도 범위 밖이다.

## 결함

| 번호 | 단계 | 결함 | 처리 |
|---|---|---|---|
| D1 | 12 G6 | 다리가 시행 끝에 `{"t": "profile_end", "trial", "result"}` 행을 쓰지 않는다(G6 설계 2 절은 쓴다고 함). `m_analysis.trials()` 는 `profile` 행의 `result` 로 시행을 나누므로 G8 은 깨지지 않는다 — 설계와 구현의 불일치(낮음) | PR 예정: 다리가 시행 끝에 한 번 쓰고 시험으로 단언(설계대로) |
| D2 | 6 S2 kth | 합성 기록을 `--allow-synthetic` 로 읽으면 출력 맨 위 `synthetic_inputs: true` 와 함께 kth 자신의 `"synthetic": false`(HIL_PLANT 비트로 정함)가 나와 서로 어긋나 보인다 | PR 예정: 머리 줄이 합성이면 kth 의 `synthetic` 도 참, 판정은 `synthetic` |

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


### 12 G6 — 짧은 직진(모형 drive 단계)

- 명령: `rehearsal_bench.py --stage drive --out-dir R` → `rehearsal_drive.py --bench R/bench.json --profile short.yaml --out R/drive1`
- 결과: 리스·TELEOP·무장 ack, 바닥 허가 `synthetic_permit`, Enter 로 시행 1 시작, 0.10 m/s 로 0.30 m 주행, 시행 `done`. 다리 주기 넘침 72 회(노트북).
- 기대와 다른 점: D1.
