# 도착일 리허설 기록 (2026-11-15, 진행 중)

- 설계: [리허설 설계](../design/2026-11-15-arrival-day-rehearsal.md). 모형 MCU(`tools/rehearsal_bench.py`, mcu_model 이 pty master)에 절차서 도구를 그대로 붙였다. 모든 기록은 synthetic 표지가 있고 측정·판정에 쓰지 않는다.
- **범위 밖:** G4·7b(스캔 → 다리 바닥 허가) 연결부는 시험하지 못했다 — 다리는 모형 전용 허가 대역(synthetic_permit)으로 돌았다. S0·S1, 실 D435i·A2M12, G9 참값 취득, 보드 고유 동작도 범위 밖이다.

## 결함

| 번호 | 단계 | 결함 | 처리 |
|---|---|---|---|
| D1 | 12 G6 | 다리가 시행 끝에 `{"t": "profile_end", "trial", "result"}` 행을 쓰지 않는다. G6 설계 2 절이 정하고 `m_analysis` 시험이 그 행으로 시행을 나누는데, 실제 기록에는 `profile` 행의 `result` 와 표지만 있다. 도착일 기록으로 G8 을 돌리면 시행 끝을 못 찾는다 | PR 예정(다리가 시행 끝에 한 번 쓰고, 시험으로 단언) |

## 단계별 기록

### 12 G6 — 짧은 직진(모형 drive 단계)

- 명령: `rehearsal_bench.py --stage drive --out-dir R` → `rehearsal_drive.py --bench R/bench.json --profile short.yaml --out R/drive1`
- 결과: 리스·TELEOP·무장 ack, 바닥 허가 `synthetic_permit`, Enter 로 시행 1 시작, 0.10 m/s 로 0.30 m 주행, 시행 `done`. 다리 주기 넘침 72 회(노트북).
- 기대와 다른 점: D1.
