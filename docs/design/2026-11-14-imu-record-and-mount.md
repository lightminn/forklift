# D435i IMU 기록기와 장착 회전 (G2·G3) — 설계 (2026-11-14)

출처: [도착일 절차서](../plans/2026-11-12-arrival-day-procedure.md) 12.3 의 G2·G3(총괄 순서 2번), [IMU 책상 검토](2026-10-22-d435i-imu-desk-review.md) 3 절, [텔레옵 기록 설계](2026-11-14-teleop-record-state-markers-clock.md)(G1·G7). 담당 콘솔 세션.

## 1. 무엇이 필요한가

- M1–M3 의 참값 heading 과 M3 의 응답 지연은 D435i 자이로를 차체 yaw rate 로 옮겨 쓴다(12.4). 그러려면 표본마다 **장치 시각과 호스트 수신 시각을 함께** 남겨야 한다 — 수신 시각만 쓰면 USB·librealsense 버퍼 지연이 M3 의 지연에 섞인다.
- 10-23 책상 기록(`record_imu.py`, 저장소 밖)은 장치 `frame_timestamp`(µs, 32 bit 라 4,295 s 마다 되감김), `get_timestamp()`(ms, 시간 영역 미기록), `time.time()`(벽시계)을 남겼다. 호스트 **단조** 시각과 시간 영역이 없다. 형식 시험의 입력으로는 쓰되(4 절), 실차 기록은 이 설계의 도구로 한다.
- 실차에서는 D435i 와 MCU 가 모두 Jetson 에 붙는다. 텔레옵 다리(G1)와 IMU 기록기는 **같은 호스트**에서 돌아야 한다 — 두 기록의 호스트 단조 시각을 같은 축으로 쓰기 위해서다(header 의 `host_boot_id` 가 같아야 한다).

## 2. G2 — `tools/imu_record.py`

```
imu_record.py --out imu.jsonl [--seconds S] [--gyro-hz 400] [--accel-hz 200] [--global-time off|on]
```

- pyrealsense2 로 자이로(기본 400 Hz)·가속도(기본 200 Hz) 원 표본을 받는다. motion correction 은 끈다(10-23 과 같게, 장치에 보정이 있으면 SDK 가 적용 — header 에 내장 보정값을 적는다). `global_time` 은 기본 **끔**: `get_timestamp()` 가 장치 하드웨어 시계 영역에 머물게 해서, 장치 시각 ↔ 호스트 시각 변환을 G7 로 직접 맞춘다(SDK 의 Theil-Sen 변환에 기대지 않는다). 켜면 그 영역을 그대로 기록한다.
- 행: `{"t": "imu", "s": "g"|"a", "dev_us": frame_timestamp(메타데이터, 32 bit 원값), "ts_ms": get_timestamp(), "dom": 시간 영역 이름, "host_ns": 콜백에서 읽은 time.monotonic_ns(), "x", "y", "z"}`. 단위: 자이로 rad/s, 가속도 m/s², 축은 SDK 가 내는 깊이 좌표계(x 오른쪽, y 아래, z 앞).
- `host_ns` 는 콜백이 불린 시각이다 — 실제 표본 시각과의 차이(전송·버퍼 지연)는 보장하지 않는다(G7 이 지연 하단 포락선으로 다룬다).
- 기록은 G1 의 `RecordWriter`(유한 큐 + writer 스레드)로 쓴다. 콜백은 큐에 넣기만 한다(librealsense 콜백 스레드를 막지 않는다). header(`schema: imu_record/1`, 장치 이름·serial·firmware·USB·IMU 내장 보정·요청/실제 빈도·`global_time`·`host_boot_id`·시작 시각), 60 s 마다 온도 행(`asic_temperature` 등, 읽히면), footer(표본 수 축별·버림·쓰기 오류·관측 빈도). 기존 파일은 거부.
- 시작 때 프로파일이 요청 빈도와 다르면 거부하지 않고 실제 값을 header 에 쓴다.
- pyrealsense2 는 `import` 를 함수 안에서 한다 — 장치 없는 ws1 시험에서도 모듈이 읽히게.

## 3. G7 연결 — `forklift_core/sensors/imu_record.py`

- `unwrap_u32_us(values) -> list[int]`: 32 bit 되감김 풀기(한 번에 2³¹ µs 넘게 뒤로 가면 한 바퀴로 본다).
- `imu_rows(rows, stream="g") -> (dev_us_unwrapped, host_ns, xyz)`: header 의 `host_boot_id` 와 함께. 
- `imu_clock_fit(rows, stream="g") -> ClockFit`: `clock_map.fit_clock(dev_us, host_ns, host_boot=...)`. 같은 함수로 MCU(G1)와 IMU 를 각각 호스트 시각에 맞추면 두 장치의 표본을 같은 축에 놓을 수 있다. 각 fit 의 한계(G7 설계 5 절)는 그대로다 — 두 장치의 최소 지연 차는 알 수 없는 상수로 남고, M3 의 지연 결과는 그 한계와 함께 적는다.

## 4. 10-23 정지 기록으로 하는 형식 시험

- 그 기록의 표본(`chunk_*.npz` 의 행 `(get_timestamp ms, frame_timestamp µs, time.time s, x, y, z)`)을 이 도구의 행 형식으로 바꾸는 변환 함수 `rows_from_static_chunk` 를 둔다(호스트 시각은 `time.time()` 을 ns 로 — 단조가 아님을 header 에 표시).
- ws1 의 분석 잡으로 7,922 s 전체에 `unwrap_u32_us` → `imu_clock_fit` 을 돌려 되감김 한 번 넘김, 표본률 400.8 Hz, drift·지연 분포를 확인한다. 결과는 PR 에 적는다(원자료는 ws1 에 있다; 시험에는 넣지 않는다).
- 단위 시험은 그 형식을 흉내 낸 작은 합성 표본(되감김 포함)으로 한다.

## 5. G3 — 장착 회전과 yaw rate

평면 운동을 가정한다: 바닥은 평평하고 차체는 바닥 위에서만 움직인다(바닥 기울기는 G9 의 수평계로 확인). 그러면 차체 yaw rate 는 `ω · û` 이다 — `û` 는 IMU 좌표로 본 차체 위쪽 단위 벡터. 전체 회전 행렬은 M1–M3 에 필요하지 않다.

- `û` 를 두 가지로 따로 잰다(`forklift_core/sensors/imu_mount.py`):
  1. **정지 중력:** 완전 정지 구간의 가속도 평균 방향(가속도계는 위쪽 반력 +g 를 낸다) → `û_g`.
  2. **제자리 회전:** 바닥에서 한쪽으로 제자리 회전하는 구간의 자이로 평균 방향 → `û_ω`. 부호는 운용자가 본 회전 방향(위에서 보아 반시계 = +yaw)으로 정한다.
  - 두 벡터 사이 각을 낸다. 바닥이 평평하고 IMU 가 단단히 붙어 있으면 작다 — 크면(문턱은 등록 때 정함) 평면 가정이나 장착을 의심하고 멈춘다. yaw rate 에는 `û_ω` 를 쓴다(회전축 그 자체).
- 장착 피치 오차 δ 의 몫(감도 cos δ, 롤·피치 속도의 새어 듦 sin δ)은 `û_ω` 를 쓰면 같은 바닥 위에서는 사라지고, 남는 것은 주행 중 차체 롤·피치(가감속·요철)의 새어 듦이다 — 기록의 `ω` 에서 `û` 에 수직인 성분의 크기를 함께 내어 그 크기를 보인다.
- **부호 확인:** 제자리 회전에서 `ω · û_ω > 0` 이 운용자가 본 반시계와 맞는지, 그리고 바퀴 오도메트리의 yaw 부호와 맞는지(같은 시각축, G7).
- **승강:** IMU 는 캐리지에 붙어 승강과 함께 움직인다(책상 검토 3 절). M1–M3 는 포크를 가장 낮게 둔 채 하고, `û` 도 그 상태에서 잰다. 승강하면 다시 잰다.
- **차체 위 G3 절차**(도착일 절차서 12 절에 넣을 내용): 데움(책상 실측의 데움 시정수 450–960 s) → 정지 60 s(중력·편향) → 반시계 제자리 회전 2 바퀴(S6 속도) → 정지 60 s → 시계 방향 2 바퀴 → 정지 60 s. 결과: `û_g`·`û_ω`(두 방향)·두 벡터 사이 각·부호·수직 성분 크기.

## 6. 시험 (먼저 쓴다)

- `imu_record.py`: 가짜 프레임 원천(pyrealsense2 없이)으로 행 형식·header·footer·온도 행·기존 파일 거부·콜백이 막지 않음(큐에만 넣음)·빈도 불일치 기록.
- `unwrap_u32_us`: 되감김 0·1·2 번, 작은 역행(되감김 아님) 처리.
- `rows_from_static_chunk`: 10-23 형식 합성 표본 → 행, header 의 '호스트 시각이 벽시계' 표시.
- `imu_clock_fit`: 되감김을 넘는 합성 기록에서 drift·포락선(G7 과 같은 기준).
- `imu_mount`: 알려진 장착 회전(피치 5.7° 숙임 등)으로 만든 정지·회전 표본에서 `û_g`·`û_ω` 복원, 두 벡터 사이 각, 회전 방향 부호, 수직 성분 크기; 운용자 방향과 반대면 부호 경고.

## 7. 범위 밖

- G6·G8·G9·G4 는 따로. IMU 를 실시간 오도메트리에 쓰는 일(자이로 heading 계획)은 이 설계 밖이다(사용자 결정 사항).
- 실기 확인: 노트북에 D435i 를 꽂아 `imu_record.py --seconds 120` 으로 형식·빈도·시간 영역을 확인하는 짧은 실기 시험을 사용자에게 부탁한다(8 절).

## 8. 사용자에게 부탁할 실기 확인

- 준비물: D435i, USB 3 케이블(10-23 과 같은 것), 노트북(conda base 에 pyrealsense2 2.58 이 있다).
- 명령 한 줄(저장소 루트): `/home/light/anaconda3/bin/python tools/imu_record.py --out /tmp/imu_check.jsonl --seconds 120`
- 시간: 약 3 분(책상 위에 그냥 두고, 마지막 30 초에 손으로 한두 번 천천히 돌려 본다).
- 돌려받을 것: 그 jsonl(수 MB). 확인할 것: 축별 표본 수·빈도, `dev_us` 와 `host_ns` 가 같이 찍히는지, 시간 영역, 회전 구간에서 `û` 추정이 돌아가는지.
