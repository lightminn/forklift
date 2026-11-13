# 텔레옵 기록: STATE 원행·시행 표시·시각 변환 (G1·G5·G7) — 설계 (2026-11-14)

출처: [도착일 절차서](../plans/2026-11-12-arrival-day-procedure.md) 12.3 의 G1·G5·G7(총괄 2026-11-14: 실차 도착 첫날의 바탕, 먼저 한다). 담당 콘솔 세션.

## 1. 무엇이 빠졌나

- 텔레옵 주행 중에는 `teleop_bridge.py --record` 만 쓸 수 있다(`bringup.py record` 와 같은 tty 의 수신 스트림을 나눠 읽게 된다). 그 기록은 주기마다의 명령·상태 요약(`applied`·`phase`·탈출 사건 등)뿐이고, MCU 가 보낸 STATE 의 원값(바퀴 원 카운트, 조향 q, `mcu_time_us`, 결함 비트)과 그 수신 시각이 없다.
- 시행의 시작·끝을 기록에 찍을 방법이 없다.
- MCU 시각과 호스트 시각을 잇는 변환과 그 오차를 내는 함수가 없다(상위 계획의 시간 규칙: 둘을 섞지 않는다).

## 2. mcu-link 계약

바꾸지 않는다. STATE(mcu-link/3)에 필요한 필드가 모두 있고, 브리지는 이미 받은 STATE 를 `_on_state(state, received_ns)` 로 받는다. 펌웨어·고정 벡터 변경 없음.

## 3. G1 — 텔레옵 기록에 STATE 원행

- 브리지가 호스트가 받아들인 STATE 를 하나도 빠짐없이 기록에 남긴다: 행 `{"t": "state", "host_ns": <수신 단조 ns>, ...McuState 의 모든 필드}`. 형식은 `bringup.py record` 의 state 행과 같다 — 그 직렬화(`state_row`)와 역직렬화(`sample_from_row`)를 `tools/bringup.py` 에서 `forklift_core/plant/record_rows.py` 로 옮겨 두 도구가 같은 함수를 쓴다(bringup 의 출력은 바이트 단위로 같게 유지, 시험으로 확인).
- 쓰는 때: 받는 즉시가 아니라 주기의 끝(송신 뒤, 지금 cycle 행을 쓰는 자리)에서 그 주기에 받은 STATE 행들을 cycle 행 앞에 순서대로 쓴다. 설계 5 의 "받기만 → 결정 → 보내기만" 순서와 주기 시간을 건드리지 않는다. 기록 쓰기가 실패해도 주행은 계속하고 `record_errors` 만 센다(지금과 같다).
- 기존 cycle 행은 바꾸지 않는다(`"t"` 없음). 새 행은 `"t"` 로 구분한다. 기록 첫 줄에 `{"t": "header", "schema": "teleop_bridge_record/1", ...}`(도구 판본, 장치 경로, 시작 시각)을 쓴다.
- 기록을 켜지 않으면(지금처럼 `--record` 없음) 아무것도 하지 않는다.

## 4. G5 — 시행 표시

- `--markers` 를 주면 터미널의 Enter 가 표시를 남긴다: 행 `{"t": "marker", "host_ns", "index", "label"}`. Enter 앞에 친 글자가 `label`(예: `m1 fwd 0.13 #3 start`). `bringup.py` 의 `_marker_reader`(select + os.read, 주행 고리를 막지 않음, 줄 단위)를 공용 함수로 바꿔 쓴다.
- 표시는 시각만 남긴다 — 자세를 재지 않는다(G9 는 따로).
- 터미널이 tty 가 아니면 표시를 받지 않는다고 시작 줄에 적는다(지금 bringup 과 같다).

## 5. G7 — 시각 변환과 정렬 오차

`forklift_core/plant/clock_map.py`:

```python
fit_clock(device_us: Sequence[int], host_ns: Sequence[int]) -> ClockFit
clock_fits_from_rows(rows) -> list[ClockFit]   # state 행을 boot_id 별로 나눠
```

- 모형: `host = a + b·device + d`, `d ≥ 0`(전송·처리 지연은 음이 될 수 없다). `a`·`b` 는 모든 점 위로 지나지 않는 직선 중 잔차 합이 가장 작은 하한 포락선(점들의 아래 볼록 껍질의 한 변)으로 정한다 — 결정적이고 지연의 꼬리에 끌리지 않는다.
- 낸 값: `n`, 구간 길이, `drift_ppm`(`b` 를 1000 ns/µs 에 대비), 포락선 위 잔차(관측 지연 변동) p50·p95·최대, 그리고 `to_host_ns(device_us)`(포락선 위의 값 = 그 STATE 의 가장 이른 가능 수신 시각). 실제 사건 시각은 그보다 알 수 없는 최소 지연만큼 이르다 — 지연을 재는 분석(M3 의 q 계단 지연)은 이 한계를 함께 적는다.
- 나누기: `boot_id` 가 바뀌면 MCU 시각이 다시 시작하므로 따로 맞춘다. 시각이 뒤로 가는 행은 그 구간에서 빼고 수를 센다.
- 무효: 점 < 20 이거나 구간 < 5 s 이면 `valid=False` 와 이유.
- IMU(G2) 의 장치 시각에도 같은 함수를 쓴다(단위만 µs 로 맞춰 넘긴다).
- `tools/bringup.py analyze` 에 `--clock` 을 더해 기록 하나의 boot 별 결과를 낸다(텔레옵 기록과 bringup 기록 모두 state 행 형식이 같다).

## 6. 시험 (먼저 쓴다)

- record_rows: bringup 의 state 행이 옮기기 전과 같다(고정 표본), 왕복(`sample_from_row(state_row(s)) == s`).
- 브리지: 모형 MCU 로 돌린 기록에 받은 STATE 수만큼 state 행이 있고 `host_ns` 가 수신 시각이다, header 가 첫 줄이다, cycle 행은 지금과 같다, 기록 없이 돌 때 행을 만들지 않는다, 기록 쓰기 실패가 주행을 멈추지 않는다.
- 표시: tty 가짜로 Enter 두 번 → marker 행 두 개(색인·라벨·시각), tty 가 아니면 없음.
- clock_map: 알려진 오프셋·drift(±50 ppm)·지연 분포(지수, 가끔 큰 지연)로 만든 점에서 `drift_ppm` 이 1 ppm 안, 포락선이 최소 지연을 따른다, boot 바뀜에 두 구간, 뒤로 가는 시각을 뺌, 점·구간 부족은 무효.

## 7. 범위 밖

- G2(IMU 기록기)·G3·G4·G6·G8·G9 는 따로 한다. 이 설계는 기록의 바탕만 만든다.
- 라이브 Isaac 경로·runner 는 건드리지 않는다(텔레옵 다리와 bringup 도구, 새 순수 함수만).
