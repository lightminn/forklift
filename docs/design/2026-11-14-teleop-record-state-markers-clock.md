# 텔레옵 기록: STATE 원행·시행 표시·시각 변환 (G1·G5·G7) — 설계 (2026-11-14)

출처: [도착일 절차서](../plans/2026-11-12-arrival-day-procedure.md) 12.3 의 G1·G5·G7(총괄 2026-11-14: 실차 도착 첫날의 바탕, 먼저 한다). 담당 콘솔 세션.

## 1. 무엇이 빠졌나

- 텔레옵 주행 중에는 `teleop_bridge.py --record` 만 쓸 수 있다(`bringup.py record` 와 같은 tty 의 수신 스트림을 나눠 읽게 된다). 그 기록은 주기마다의 명령·상태 요약(`applied`·`phase`·탈출 사건 등)뿐이고, MCU 가 보낸 STATE 의 원값(바퀴 원 카운트, 조향 q, `mcu_time_us`, 결함 비트)과 그 수신 시각이 없다.
- 시행의 시작·끝을 기록에 찍을 방법이 없다.
- MCU 시각과 호스트 시각을 잇는 변환과 그 오차를 내는 함수가 없다(상위 계획의 시간 규칙: 둘을 섞지 않는다).

## 2. mcu-link 계약

바꾸지 않는다. STATE(mcu-link/3)에 필요한 필드가 모두 있고, 브리지는 이미 받은 STATE 를 `_on_state(state, received_ns)` 로 받는다. 펌웨어·고정 벡터 변경 없음.

## 3. G1 — 텔레옵 기록에 STATE 원행

- **원천.** 호스트가 받아들여 `_on_state(state, received_ns)` 로 넘긴 STATE 전부. `core.samples` 는 쓰지 않는다 — 오래된 표본을 거부하고 길이를 자르므로 "받아들인 모든 STATE" 의 원천이 될 수 없다.
- **행.** `{"t": "state", "host_ns": <received_ns>, "read_ns": <읽기 완료 단조 ns>, ...McuState 의 모든 필드}`. 형식은 `bringup.py record` 의 state 행과 같다 — 그 직렬화(`state_row`)와 역직렬화(`sample_from_row`)를 `tools/bringup.py` 에서 `forklift_core/plant/record_rows.py` 로 옮겨 두 도구가 같은 함수를 쓴다(bringup 의 출력은 그대로, 시험으로 확인). `read_ns` 는 텔레옵 행에만 더하는 선택 필드다.
- **시각의 뜻(Codex 설계 검토 P2-2).** `received_ns` 는 지금 주기 시작 시각이다 — `ReconnectingLink.service` 가 읽기를 마친 뒤에도 그 값을 `on_bytes` 에 넘기고, 한 번에 읽은 여러 STATE 가 같은 값을 받는다. 제어의 나이 계산은 이 값을 그대로 쓴다(바꾸지 않는다). 시각 변환용으로는 링크가 `read_available` 이 돌아온 직후의 단조 시각을 `last_read_ns` 로 두고, 그 읽기에서 나온 STATE 의 `read_ns` 로 기록한다. 한 읽기의 STATE 들은 같은 `read_ns` 를 가지며, 앞쪽 STATE 일수록 실제 수신은 그보다 최대 한 주기(20 ms) 이르다 — G7 의 포락선은 각 읽기 직전에 온 STATE 가 정한다.
- **쓰기(P2-3·P2-4).** 받는 즉시(`_on_state`, 그리고 cycle 행·marker 행도 같은 줄에) 유한 큐에 넣기만 하고, 직렬화와 파일 쓰기는 별도 writer 스레드가 한다. 큐 상한 20,000 행(약 3 분 분량)을 넘으면 새 행을 버리고 버린 수를 센다 — 주행 고리를 막지 않는 쪽을 고른다(무손실·유한 메모리·디스크 정지를 함께 보장할 수 없다). 버린 수·쓰기 오류 수는 끝 줄(`{"t": "footer", "dropped", "write_errors", ...}`)과 stdout 에 낸다. `close()` 는 DISARM 뒤 수신까지 큐에 넣은 다음 writer 를 비우고 닫는다(제어 `cycle()` 은 다시 부르지 않는다). Ctrl-C 에서도 같은 종료 경로를 탄다.
- **파일.** 기록 경로가 이미 있으면 시작을 거부한다(실행마다 새 산출물; 지금의 `"a"` 이어쓰기를 `"x"` 로). 첫 줄은 `{"t": "header", "schema": "teleop_bridge_record/1", "tool", "device", "host_boot_id", "start_wall", "start_mono_ns"}` — 호스트 단조 시계의 원점(`/proc/sys/kernel/random/boot_id`)을 적어 다른 호스트 부팅의 기록과 섞이지 않게 한다.
- 기존 cycle 행은 바꾸지 않는다(`"t"` 없음). 기록을 켜지 않으면 큐도 스레드도 만들지 않는다.

## 4. G5 — 시행 표시

- `--markers` 를 주면 터미널의 Enter 가 표시를 남긴다: 행 `{"t": "marker", "host_ns", "index", "label"}`. Enter 앞에 친 글자가 `label`(예: `m1 fwd 0.13 #3 start`), 빈 Enter 는 빈 라벨. `host_ns` 는 Enter 를 친 시각이 아니라 읽어 낸 시각이다(주기 하나 안).
- `bringup.py` 의 `_marker_reader`(select + os.read, 줄 단위)를 공용 함수로 바꿔 쓴다. 공용 API 는 `(poll, active, reason)` 을 돌려준다 — tty 가 아니면 `active=False` 와 이유를 시작 줄에 낸다. 한 poll 의 작업량을 묶는다: 읽기 4,096 바이트·표시 16 개까지, 미완성 라벨은 200 바이트에서 자른다. UTF-8 이 나뉘어 와도 줄이 끝날 때 디코드한다(지금처럼). 표시 행도 큐에 넣기만 한다.
- 표시는 시각만 남긴다 — 자세를 재지 않는다(G9 는 따로).

## 5. G7 — 시각 변환

`forklift_core/plant/clock_map.py`:

```python
fit_clock(device_us, host_ns) -> ClockFit
clock_fits_from_rows(rows, *, time_key="read_ns") -> list[ClockFit]   # state 행을 (host_boot, boot_id) 별로
```

- **모형과 방법.** `host = a + b·device + d`, `d ≥ 0`. `a`·`b` 는 모든 점이 위에 있는 직선 가운데 잔차 합이 가장 작은 것 — 점들의 아래 볼록 껍질에서 잔차 합을 최소로 하는 변(공선점이면 결정적으로 첫 변).
- **뜻과 한계(P2-1).** `to_host_ns(device_us)` 는 '지연 하단에 맞춘 수신 기준 시각' 의 추정값이다. 사건 시각과의 차이는 알 수 없는 최소 지연과, 지연 추세가 drift 로 흡수된 몫을 함께 가진다. 포락선 위 잔차(p50·p95·최대)는 **관측 지연의 변동**이지 정렬 오차가 아니다. 지연이 시간에 따라 변하면(예: 10 → 15 ms 선형) 그 추세가 drift 로 들어간다 — 그래서 품질 지표를 따로 낸다: 앞·뒤 절반 각각의 fit 기울기 차(`drift_split_ppm`)와, 전체 fit 과 반쪽 fit 의 구간 끝 시각 차(`split_end_ns`). 20 점·5 s 는 계산 가능 조건일 뿐 정확도 보장이 아니다.
- **정리 규칙(P3).** MCU 시각 역행은 마지막으로 채택한 값과 비교해 뺀다(`[10, 8, 9, 11]` → 8·9 를 뺌). 호스트 시각 역행도 뺀다. 같은 장치 시각이 거듭되면 가장 이른 호스트 시각 하나만 남긴다. 뺀 수를 종류별로 낸다. 정리 뒤 점 < 20 이거나 구간 < 5 s 면 `valid=False` 와 이유. 무효 fit 의 `to_host_ns` 는 예외. 구간 밖 변환은 값과 함께 `extrapolated=True` 를 돌려준다.
- **나누기.** 호스트 부팅(header 의 `host_boot_id`)과 MCU `boot_id` 가 바뀔 때마다 따로 맞추고, 각 fit 에 그 구간(장치 시각·호스트 시각의 처음·끝)을 붙인다.
- IMU(G2)의 장치 시각에도 같은 함수를 쓴다(µs 로 맞춰 넘긴다).
- **CLI(P3).** `bringup.py clock --record R`(새 하위 명령, `analyze` 의 `--kind`·marker 조건과 무관). 텔레옵 기록은 `read_ns`, bringup 기록은 `host_ns` 를 쓴다(bringup 의 `host_ns` 의 뜻도 같은 한계와 함께 적는다).

## 6. 시험 (먼저 쓴다)

- **record_rows:** bringup state 행이 옮기기 전과 같다(고정 표본), 왕복이 같다.
- **기록:** 모형 MCU 기록에 받은 STATE 수만큼 state 행(`host_ns`·`read_ns`), header 가 첫 줄·footer 가 끝 줄, cycle 행은 지금과 같다; 한 번에 64 개를 넘는 STATE; 재연결 뒤 core 가 거부하는 옛 STATE 도 기록됨; 같은 배치 안의 boot 변경; 종료 중(close) 받은 STATE 와 Ctrl-C; 중간 쓰기 실패와 close 실패에서 주행 계속·오류 수; 느린 writer 에서 큐 포화 → 버림 수, 주기 시작 간격 유지; 기존 파일이면 시작 거부; 기록 없이 돌 때 행·스레드 없음.
- **표시:** tty 가짜로 Enter·라벨 → marker 행, 빈 Enter, 부분 줄·Ctrl-D(기존 회귀 유지), UTF-8 분할, 입력 폭주(한 poll 의 작업량 상한), tty 종료, tty 아님 → 비활성과 이유.
- **clock_map:** 알려진 오프셋·drift(±50 ppm)·지수 지연에서 여러 고정 시드로 drift 오차와 `drift_split_ppm` 이 함께 작게(값은 시험이 정한 범위), 선형·계단형 지연 변화에서 `drift_split_ppm`·`split_end_ns` 가 그 변화를 드러냄, 중복·연속 역행·호스트 역행 정리, 정확히 20 점·5 s 경계, 공선점의 결정성, 구간 밖 `extrapolated`, 무효 fit 의 예외, boot·호스트 부팅 나누기, marker 없는 기록의 `clock`.

## 7. 범위 밖

- G2(IMU 기록기)·G3·G4·G6·G8·G9 는 따로 한다. 이 설계는 기록의 바탕만 만든다.
- 라이브 Isaac 경로·runner 는 건드리지 않는다(텔레옵 다리와 bringup 도구, 새 순수 함수만).

## 8. 검토

- Codex 설계 검토(새 스레드): P1 없음. P2 5 — 포락선 잔차를 정렬 오차로 읽음(→ 5 절 뜻·한계, 반쪽 fit 품질 지표), `received_ns` 는 주기 시작 시각(→ `read_ns` 따로), 송신 뒤 동기 쓰기의 주기 영향(→ 유한 큐 + writer, 포화 때 버림), 종료 중 수신 누락(→ 받는 즉시 큐, close 가 비움, `core.samples` 와 독립), append 와 header(→ 기존 파일 거부, 호스트 부팅 id). P3 3 — 표시의 작업량 상한·활성 여부, 역행·중복 규칙과 구간 밖 변환, `clock` 독립 하위 명령. 모두 반영.
