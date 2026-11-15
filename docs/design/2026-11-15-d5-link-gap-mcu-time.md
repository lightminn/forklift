# D5 — 링크 신선도를 MCU 시각 간격으로 판정 (설계 메모, 2026-11-15)

상태: 설계(구현 전 — 3 절 이하의 "한다·둔다·더한다"는 모두 구현할 규칙이며, 지금 코드에는 없다) · 결정: 총괄 (가) 채택, 조건 1–5 고정 · 독립 검토 9 절

## 1. 배경과 측정

`McuLinkHost` 는 STATE 신선도를 **호스트 수신 시각**으로 판정한다. 텔레옵 다리의 주기 시각(20 ms 루프)이 그 수신 시각이라서, 다리 루프가 약 30 ms(= 50 ms 문턱 − 20 ms 주기)를 넘게 멈추면 `stale` 이 난다. 그러면 권한이 철회되고 disarm 된다. 그동안 STATE 는 끊기지 않고 USB/pty 버퍼에 쌓여 있었어도 마찬가지다. 모형 E2E 에서 30 ms 정지로 결정적으로 재현된다.

노트북 측정(2026-11-15, 실제 다리, 합성 허가 없음, 리허설 MCU 모형 on pty, 1,080 s, 같은 프로세스에 콘솔 서버 + G1 기록, 다른 프로세스에 200 Hz IMU 기록기, 600–720 s 동안 18코어 부하, Codex·다른 세션 동시 실행, 데드맨을 누른 무장 정지 상태):

| 구간 | 주기 간격 수 | > 30 ms | > 40 ms | > 50 ms | p999 | 최대 |
|---|---|---|---|---|---|---|
| 전체 | 53,701 | — | — | 5 | 23.57 ms | 327.9 ms |
| 무부하 | 47,861 | 3 | 2 | 2 (839.3 s 57.6 ms, 1065.2 s 68.0 ms) | 20.45 ms | 68.0 ms |
| 부하 | 5,840 | 11 | 5 | 3 | 37.45 ms | 327.9 ms |

- `stale` 은 5번 났다. 50 ms 를 넘은 간격 5개와 정확히 같은 시점이다(무부하 2개 포함).
- 앞선 부분 실행(916 s, 판정용 아님): 부하 구간 166 ms 정지 → `stale` → `authorization_revoked` → disarm. 무부하 51.9 ms.
- 643.2 s 에 **모형 자신의** STATE 흐름에 MCU 시각 30.8 ms 간격이 생겼다(같은 CPU 에서 모형이 굶음). 결과는 `no_feedback:rear_axle_speed` disarm. 엔코더 속도의 `dt_max_s = 0.025`(`config/real/teleop_bridge.yaml:10`) 때문이며, 이 메모의 판정과는 별개다. 실제 MCU 는 별도 하드웨어라 리허설 산물로 보지만 기록해 둔다.
- Jetson 은 아직 재지 않았다(LAN 에 없음).

## 2. 현재 판정 경로

- 신선도: `src/forklift_core/plant/mcu_host.py:181-182` `_fresh(now) = now − _state_rx_ns ≤ state_max_age_s`. 모듈 설명 `:15-19`("Freshness on the host clock … never mixed with it"). 계약 `docs/interfaces/2026-10-22-mcu-link-contract.md:162`(4절: 호스트 단조 시각으로 판단, `mcu_time_us` 와 빼지 않음).
- 수신: `mcu_host.py:212-221` `on_bytes` → STATE 마다 `_on_state`(`:244-277`). `:252-255` 에서 **갱신 전에** `_expire(now_ns)` 를 부른다(Codex P0-7 2라운드 P1). `_expire`(`:318-325`)는 `_fresh` 가 거짓이면 `stale` + `_revoke("stale")`, 그다음 무장 마감을 본다. `tick`(`:327-330`)도 같은 `_expire` 를 쓴다.
- `now_ns` 의 출처: `tools/teleop_bridge.py:289-309` `run` 이 주기 시작 시각 → `Bridge.cycle(now)`(`:214-216`) → `ReconnectingLink.receive` → `service`(`tools/real_mcu_link.py:165-191`). 여기서 `on_bytes(data, now_ns)`(`:178`) 다음 `tick(now_ns)`(`:179`)를 부른다. 한 번에 읽은 STATE 들은 모두 주기 시작 시각을 받는다(기록 설계 `docs/design/2026-11-14-teleop-record-state-markers-clock.md:19`). 실제 읽은 시각은 `last_read_ns`(`real_mcu_link.py:177`)이고 기록에만 쓴다.
- 설정: `teleop_bridge.py:55` `HOST_CFG = HostConfig(state_max_age_s=0.05, cap_ms=150, …)`. 같은 값이 `tools/bringup.py:101`, `COMMISSION_HOST_CFG`(`:103-104`), `real_mcu_link.py:236-237`(모니터, 무장 안 함)에도 있다.
- DRIVE 유효시간: `mcu_host.py:438` `valid = min(cap_ms, valid_after_echo_ms(expires_ns, self._state_rx_ns))`. 계약 2절(`:34-35`): `deadline = echo + min(valid, command_timeout_ms)`.
- 다리의 DRIVE 만료: `src/forklift_core/live/real_teleop.py:398` `OUT_LIFE_NS = 100 ms`, `:501-509` `_deadline_ns`. 그래서 실제 `valid ≈ min(150, 100) = 100 ms` 다.
- 왕복 지연 상한은 이미 있다: `real_teleop.py:156-250` `DelayBound`. 엔코더 속도는 `age = (now − host_ns) + delay_s > max_age_s(0.05)` 이면 무효다(`:117-119`, 설정 `teleop_bridge.yaml:8`). 그러나 이 판정은 다리의 속도 경로에만 있고 `McuLinkHost` 의 신선도·권한에는 없다.

## 3. (가) 설계

### 3.1 무엇을 보는가 (요지)

호스트 정지 중에 쌓인 STATE 는 **순서대로, 하나도 빠짐없이** 도착한다. 그래서 그 사이의 모드 전이(STOPPING 등)도 모두 보인다. 정보가 실제로 빠졌는지는 **MCU 시각의 간격**에 나타난다(MCU 의 침묵, 송신 버퍼가 차서 버린 STATE: 계약 5절 `:166`). 남는 위험은 둘이다. 하나는 "호스트 자신이 너무 오래 멈춤"으로, 상한 150 ms 로 막는다. 다른 하나는 "버퍼에 묶여 늦게 온 STATE 가 신선해 보임"으로, 3.4 의 지연 상한으로 막는다.

### 3.2 상태 변수 (호스트에 더함)

- `_state_delay_s: float | None` — 마지막으로 받아들인 STATE 의 측정→수신 지연 상한(3.4). 상한이 없으면 None.
- 기존 `_state`, `_state_rx_ns`, `_stale_reported` 는 그대로 둔다.

### 3.3 판정 규칙 (`gap_clock = "mcu"` 일 때. `"host"` 는 지금과 한 글자도 다르지 않다)

정의: `age(now) = (now − _state_rx_ns) + (_state_delay_s or 0)`. 신선 ⇔ `age(now) ≤ state_max_age_s`. `_fresh`·`health`·`request_arm`·`request_clear_faults`·`_motion_check` 가 모두 이 정의를 쓴다.

`_on_state(s, now)` — 기존 `boot_id 0`·역순·중복 거부(`:245-251`)는 그대로. 그 뒤:

1. `d = delay_bound(s, now)` 를 STATE 를 받아들이기 **전에** 계산한다. 그 전 STATE 들이 만든 기준점만 쓰므로 상한은 같거나 더 크다(보수적).
2. **간격 검사**(`_expire` 의 신선도 부분을 대신함, 갱신 전):
   - `host_gap = now − _state_rx_ns > host_gap_max_s` (0.15 s) → `stale(why="host_gap")`. 늘 적용한다.
   - 그렇지 않고 같은 부팅이며 `d is not None` 이면: `mcu_gap = (s.mcu_time_us − prev.mcu_time_us)/1e6 > state_max_age_s` → `stale(why="mcu_gap")`.
   - 그 밖(`d is None`, 부팅이 바뀜): **지금 규칙** `host_gap > state_max_age_s` → `stale(why="host_gap")`. **상한을 모르면 (가)는 아무것도 느슨하게 하지 않는다.**
   - 무장 마감 검사(`:324-325`)는 그대로.
3. 갱신: `_state, _state_rx_ns, _state_delay_s = s, now, d`. 그다음 콜백·사건·권한은 지금 순서 그대로(`:256-277`).

`on_bytes` 끝(묶음 전체를 처리한 뒤)과 `tick`: `_expire(now)` 는 `age(now) > state_max_age_s` → `stale(why="age")`. **나이 검사는 묶음의 가장 새 STATE 에만 건다.** 정지 뒤 묶음의 앞쪽 STATE 들은 지연이 정지 길이만큼 크지만 곧 다음 STATE 에 덮인다. STATE 마다 검사하면 조건 1 이 깨진다.

P0-7 P1 과의 관계: "정지 구간 뒤에 온 STATE 가 그 구간을 가리면 안 된다"는 요구는 유지된다. 구간은 출처의 침묵(MCU 시각 간격 > 50 ms)이나 호스트 정지(> 150 ms)로 정의하고, 검사는 지금처럼 갱신 전에 한다. 바뀌는 것은 "호스트가 못 본 30–150 ms" 를 그 자체로 끊김으로 보지 않는다는 것뿐이다.

### 3.3a 묶음 처리 순서 (검토 반영, 3.3 을 대신하는 정본)

검토 ②(Codex)와 ③(agy)의 공통 지적은 이것이다. 정지 뒤 한 번에 읽힌 묶음에서 STATE 마다 상태 전이를 돌리면 문제가 생긴다. 옛 STATE 가 신선도를 가리거나, 옛 `mcu_time_us`·조향으로 zero DRIVE 를 만들 수 있다(후자는 MCU 의 `stale_echo` 로 무장 실패). 그래서 `"mcu"` 모드의 `on_bytes(data, rx)` 는 다음 순서를 정본으로 한다.

```
rx = 실제 읽은 시각(read 직후 monotonic, 3.3b)
accepted = []
for s in 해독된 STATE:
    boot_id 0·같은 부팅 역행·중복 거부(지금과 같음)
    d = delay_bound(s, rx)              # 앞서 받아들인 STATE(같은 묶음의 앞쪽 포함)가 만든 기준점만 씀
    if accepted == []:                  # 묶음의 첫 STATE: 직전 묶음과의 간격
        rx - _state_rx_ns > host_gap_max_s           -> stale(host_gap)
    same boot 이고 d 가 있음: (s.mcu - prev.mcu)·(1+ρ) > state_max_age_s -> stale(mcu_gap)   # ρ: 3.11
    d 가 없음: 지금 규칙(rx - _state_rx_ns > state_max_age_s -> stale(host_gap))
    _state, _state_rx_ns, _state_delay_s = s, rx, d
    on_state 콜백(DelayBound.add 등) -- 받아들인 STATE 마다, 순서대로
    철회만 하는 전이: boot_changed, mode≠ARMED(ZERO_PENDING/READY), arm_gen_changed, build_refused, 거부 이유
    accepted.append(s)
newest = accepted[-1]
나이 검사: (rx - _state_rx_ns) + _state_delay_s > state_max_age_s -> stale(age)   # newest 의 d
앞으로 가는 전이(ARM_PENDING→ZERO_PENDING, ZERO_PENDING→READY)는 newest 로만, 묶음 안에서 stale 이 났으면 하지 않음
```

- 철회는 `_auth` 를 `_NONE` 으로 만든다. `_NONE` 에서 `_advance_auth` 는 아무것도 하지 않으므로(`mcu_host.py:279-316`), 묶음 안에서 철회된 권한이 같은 묶음의 뒤쪽 STATE 로 되살아나는 경로는 지금도 없다. 다시 무장하려면 운전자의 새 `request_arm`(신선도 검사 포함)이 필요하다. 이 성질을 시험으로 고정한다(5.1 추가).
- 앞으로 가는 전이를 newest 로만 하면, zero DRIVE 의 echo·조향은 가장 새 STATE 의 값이 된다.
- `tick(now)` 의 나이 검사는 같은 식에 `now` 를 넣는다. 호스트 루프가 주기 c(20 ms)로 도는 동안에는 이 검사가 호스트 시계로만 판정한다. 그래서 MCU 시계와 상관없이, 실제 STATE 도착 간격이 `state_max_age_s − d + c` 를 넘으면 반드시 `stale` 이 난다. d ≥ 0 이면 최악 70 ms 다.
  - 검토 ② 의 반례(MCU 시계 0.5 배, STATE 가 호스트 기준 80 ms 마다, d = 10 ms)에서는 +40 ms tick 의 나이가 50(통과), +60 ms tick 의 나이가 70 > 50 이라 `stale` 이 난다. 시험으로 고정한다.
  - 여기서 c 는 설정 주기가 아니라 **호스트가 실제로 돈 간격** g 다. 놓칠 수 있는 실제 도착 간격의 상한은 `state_max_age_s − d + g` 이고, g 는 `host_gap_max_s`(150 ms) 넘으면 그 자체로 `stale` 이다. g = 20 ms 일 때의 70 ms 는 **지금 규칙의 입도와 같다**. 지금도 나이가 50 ms 를 넘은 것을 다음 tick 에서야 보므로, 놓치는 상한이 50 + 20 ms 다. 그러므로 (가)는 정상 루프에서 이 상한을 늘리지 않는다. 검토 ② 3 회차의 60 ms 반례(나이 최대 50, 통과)는 이 상한 안에 든다.
  - 호스트 루프 자체가 느려 사이에 tick 이 없는 경우는 두 가지가 막는다. 하나는 3.12 의 `loop_slow` 이고, 다른 하나는 `"mcu"` 규칙을 `rate_verified` 뒤에만 켠다는 3.11 의 전제다. 이 두 장치가 아직 듣지 않는 동안 한 번 긴 공백이 생기는 경우는, **검출 상한과 운동 상한을 따로** 본다(검토 ② 4 회차).
  - **검출 상한**(호스트가 알아차리는 때): `state_max_age_s − d + g` 이고, g ≤ 150 ms 이면 최대 200 ms 다. g 가 150 ms 를 넘으면 그 순간 `host_gap` 이다. 정확히 150 ms 는 경계라 통과한다.
  - **운동 상한**(MCU 가 옛 명령으로 움직이는 시간): 호스트의 검출과 무관하게 MCU 가 강제한다. MCU 의 마감은 `echo 의 방출 시각 + min(valid, command_timeout_ms)` 다(계약 2 절). 여기서 `valid = min(cap, expires − rx(echo))`, `expires` 는 결정 + 100 ms 다.
    - 호스트가 멈춘 동안에는 새 DRIVE 가 나가지 않는다. 그래서 MCU 는 마지막 결정에서 100 ms 안에 마감을 맞는다.
    - 호스트가 깨어나 DRIVE 를 다시 보내려면 `_motion_check` 가 신선도를 통과해야 한다. 그 DRIVE 는 묶음의 가장 새 STATE 를 echo 하므로, 마감은 방금 방출된 STATE 에서 센다.
    - STATE 가 안 오는 채로 호스트만 돌 때도 같다. 마감은 echo 된 STATE 의 방출 시각 + 150 ms 를 넘지 않고, 그 전에 `tick` 이 `stale` 을 낸다(정상 루프에서 ≤ 70 ms).
  - 정리하면 검출이 200 ms 까지 늦어질 수는 있지만, 그동안 차가 옛 명령으로 가는 시간은 지금과 같은 DRIVE 수명 안이다. 150 ms 경계의 실제 동작은 5.2 시험으로 확인한다.

### 3.3b 수신 시각은 실제 읽은 시각

지금은 한 주기에 읽은 STATE 가 모두 주기 시작 시각을 받는다(`real_mcu_link.py:165-191`, 실제 읽은 시각 `last_read_ns` 는 `:177` 에서 기록에만 쓰임). 검토 ③ 의 지적이 맞다. 주기 시작과 `read` 사이에서 GIL·GC 로 30 ms 가 멈추면 두 가지가 틀어진다. 첫째, 나이가 30 ms 작게 잡힌다. 둘째, `valid_after_echo_ms` 가 MCU 마감을 30 ms 늦게 잡아 계약 2 절의 보수성이 깨진다. `"mcu"` 모드에서는 시각을 시스템 호출 경계에서 **안전한 쪽**으로 찍는다(구현 예정, 지금 코드는 `real_mcu_link.py:136`·`:178` 둘 다 주기 시작 `now_ns`).

- **수신 시각**은 `os.read` 가 돌아온 **뒤**에 찍는다. 이 값은 바이트가 실제로 도착한 시각보다 늦다. 그래서 세 가지가 모두 보수적인 쪽으로 움직인다. ① `valid_after_echo_ms = expires − rx` 가 짧아진다. ② `D` 에 들어가는 `(rx − mcu)` 가 커진다. ③ 묶음 사이의 호스트 간격이 커진다. 나이 `now − rx + D` 에서는 `rx` 가 지워지므로(3.4) 이득도 손해도 없다. 3.5 의 1 ms 미만 마감 초과도 함께 없어진다.
- 한 번의 `read` 로 받은 여러 프레임의 개별 도착 시각은 알 수 없다. 모두 같은 읽은 시각을 받는다. 앞쪽 프레임에는 늦은 쪽(보수적) 값이다. 처리 중의 멈춤은 다음 `read` 의 시각에 들어간다. 시각 오차의 실측 분포는 기록 열(5.1a)로 구현 뒤 측정에서 낸다.
- **DelayBound 기준점의 쓰기 시각**은 `os.write` 를 부르기 **전**에 찍는다. 실제 송신보다 이른 값이라 왕복 `a_b = rx − w` 가 커지므로, 상한이 작게 나오지 않는다. 주기 시작보다 늦은 값이므로, 주기 시작과 쓰기 사이의 멈춤이 기준점을 부풀리는 일(검토 ③ 5)도 줄어든다. 시각을 찍은 뒤 실제 호출까지의 멈춤은 상한을 키우는 쪽(안전)이다. 다만 거짓 `stale` 원인이 될 수 있으므로 같은 기록 열로 잰다.

### 3.4 조건 3 — 버퍼 지연의 상한

- **지연의 정의**: `D(s)` = STATE `s` 의 측정→호스트 수신(`received_ns`) 지연의 **상한**. 기존 `DelayBound.delay_s`(`real_teleop.py:240-250`)를 그대로 쓴다. 이 값은 이번 세션이 쓴 프레임 k 의 쓰기 시각부터, k 를 처음 알린 STATE **다음** STATE 의 수신까지의 왕복으로 기준점을 잡고(`:157-179`), 오프셋 차와 드리프트 여유(A3 ≤ 100 ppm, 기준점 ≤ 2 s)로 다른 STATE 로 옮긴다.
- **넘었을 때**: `age = (now − rx) + D > 50 ms` 이면 `stale(why="age")` → 권한 철회. 문턱은 조건 4 대로 50 ms 그대로다. 엔코더 속도의 무효 조건(`real_teleop.py:117-119`)과 **같은 식, 같은 수**라서 정상 운용에서 새로 생기는 끊김은 그 속도가 이미 무효였던 주기에 한정된다. 그 주기는 주행 중이면 이미 `speed_invalid` 로 STOP 이다.
- **`received_ns` 가 주기 시작 시각이어도 괜찮은 이유**: `age = now − rx + D` 를 풀면 `rx` 가 지워진다(`D = a_b + (rx − mcu) − off_a + …`). 그래서 늦게 읽은 STATE 가 이득을 보지 않는다.
- **`clock_map.py` 를 쓰지 않는 이유(확인 필요 — 지시문과 다름)**:
  1. `fit_clock` 은 오프라인 함수다(점 ≥ 20, 구간 ≥ 5 s: `clock_map.py:21-22`).
  2. `to_host_ns` 는 지연 **하단** 포락선이라(`:3-12`, `:117-123`) "수신 − 변환값" 은 지연이 아니라 최소 지연 위의 초과분이다. 지속된 지연 증가는 오프셋·기울기에 흡수된다.
  3. 같은 종류의 한쪽 방향 오프셋 기준(5 s 창의 최솟값, 드리프트 상한만큼 올리는 기준, 부팅 최솟값)이 이 저장소에서 Codex P1 로 세 번 버려졌다(`docs/design/2026-11-12-teleop-bridge.md:161`, `:175`, `:184`. 빠른 MCU 시계가 늘어나는 지연을 숨김).
  
  `clock_map` 은 전제 A3(드리프트 ≤ 100 ppm)를 실차 기록에서 오프라인으로 확인하는 데 쓴다(`bringup.py clock`). 초과 지연(G7 잔차)은 진단 열로 기록에 남겨도 된다.
- **상한이 없을 때**(시작, 새 부팅, 재연결 뒤 `restart_check`): `d = None` → 3.3 의 2단계가 지금 규칙으로 돌아가고, 나이에는 0 을 더한다. 다리에서 무장하려면 이미 유효한 엔코더 속도(곧 상한)가 있어야 하고(`_arm_blocker`, `real_teleop.py:690-704`), 무장 중에는 매 주기 DRIVE 가 기준점을 만든다.

### 3.5 DRIVE 유효시간

`valid_after_echo_ms(expires_ns, self._state_rx_ns)` 를 **그대로 둔다(호스트 수신 기준)**. MCU 마감을 호스트 시각으로 옮기면 `방출 + (expires − rx) = expires − (rx − 방출) ≤ expires` 라서, 지연과 무관하게 보수적이다. 변환값이나 `D` 로 바꾸면 추정 오차에 기대게 되니 바꾸지 않는다. 정지 뒤 묶음에서 echo 는 가장 새 STATE 이고 `rx = now` 다.

(기존 관찰, 이 메모 범위 밖: `rx` 가 주기 시작 시각이라 STATE 가 주기 시작과 읽기 사이에 방출됐다면 MCU 마감이 결정 마감보다 1 ms 미만 늦을 수 있다. 확인 필요: 별건으로 `last_read_ns` 를 쓸지.)

### 3.6 MCU 쪽 만료와의 관계 (80–150 ms 구간)

다리의 DRIVE 는 결정 + 100 ms 다(`real_teleop.py:398`). MCU 마감은 결정으로부터 대략 `100 − e`(e = echo 의 방출→주기 시작, 약 0–12 ms + 지연) 뒤다. 그래서 주기 간격 G 가 약 85 ms 를 넘으면 MCU 가 먼저 만료돼 STOPPING 으로 간다. 그 STATE 들이 묶음으로 오면 호스트는 `mode_stopping` 으로 철회하고, 다리는 `mcu_stopping` 경로로 간다(`real_teleop.py:619-629`). 결과:

- G ≤ 약 80 ms: 끊김 아님(조건 1).
- 약 85–150 ms: 호스트 `stale` 은 없지만 MCU 만료로 정지(정지 이유가 `mcu_link` 가 아니라 `mcu_stopping`).
- G > 150 ms: 호스트 `stale`(조건 2). 327.9 ms 는 둘 다.

조건 2 의 "MCU 도 150 ms 에 멈춘다"는 상한 `cap_ms` 기준이고, 실제 다리는 100 ms 수명이라 더 일찍 멈춘다. 확인 필요: 이 회색 구간을 받아들일지. `OUT_LIFE_NS` 는 이 메모에서 바꾸지 않는다.

### 3.7 시계·부팅·첫 STATE

- `mcu_time_us` 는 u64 라 사실상 넘어가지 않는다(계약 `:33`, `:108`). 같은 부팅 안의 역행·중복은 지금처럼 거부한다(`mcu_host.py:249-251`).
- µs 간격과 s 문턱의 비교: 드리프트 100 ppm 이면 50 ms 에서 5 µs 다. 무시한다.
- 첫 STATE(`prev is None`), `disconnected()` 뒤(`:332-340`, `_state = None`): 간격 검사 없음. 지금과 같다.
- 부팅이 바뀌면 MCU 간격을 정의할 수 없다 → 호스트 규칙. 어차피 `boot_changed` 로 철회한다(`:263-265`). `DelayBound` 도 부팅별이라 `d = None`.

### 3.8 `HostConfig`·생성자·건강 정보

- `gap_clock: str = "host"` — `"host"` | `"mcu"`. `teleop_bridge.HOST_CFG` 만 `"mcu"`.
- `host_gap_max_s: float | None = None` — None 이면 `cap_ms / 1000`(0.15). 검증: `state_max_age_s ≤ host_gap_max_s ≤ cap_ms/1000`. 다리 값은 0.15.
- 생성자 `delay_bound: Callable[[McuState, int], float | None] | None = None`. `gap_clock="mcu"` 인데 없으면 `ValueError`. 다리는 `lambda s, rx: self.core.delays.delay_s(Sample(rx, s))` 를 넘긴다. host 를 core 보다 먼저 만들므로(`teleop_bridge.py:108-115`) 늦은 바인딩으로 넘긴다.
- `LinkHealth` 에 `delay_s`, `link_age_s` 를 더한다. `stale` 사건 detail 에 `why`(`host_gap`·`mcu_gap`·`age`)와 그 값을 넣는다. 다리 기록의 `link_events` 는 지금 종류만 남기는데(`teleop_bridge.py:246-247`), `stale` 의 detail 도 남기도록 확장한다.
- 모듈 설명(`mcu_host.py:15-19`)과 계약 4절 `:162` 의 문구를 고친다. 와이어·필드 변경이 없으므로 계약 version 은 그대로다(7절). 확인 필요: 계약 문구 개정에 펌웨어-H743 합의가 필요한지.

### 3.9 적용 범위

| 사용처 | 적용 | 이유 |
|---|---|---|
| `tools/teleop_bridge.py` | `"mcu"` | D5 의 대상. `DelayBound` 가 있다 |
| `tools/bringup.py` (`HOST_CFG`, `COMMISSION_HOST_CFG`) | `"host"` 유지 | 왕복 상한이 없다. 상한 없이 (가)만 켜면 3.4 의 위험이 그대로 남는다. 확인 필요: bring-up 에도 원하면 `DelayBound` 를 먼저 옮겨야 함 |
| `tools/real_mcu_link.py` 모니터 | `"host"` 유지 | 무장하지 않는 관찰 도구 |
| 그 밖 | 없음 | `src`·`tools` 의 생성 지점은 위 셋뿐이다(`grep -rn "McuLinkHost(" src tools`, 2026-11-15) |

### 3.10 재연결과 지연 상한의 초기화 (검토 ③ 2)

`DelayBound.restart_check()` 는 `core.cycle()` 안에서만 불린다(`real_teleop.py:519-522`). 다리의 주기 순서는 `receive → cycle → transmit` 이다(`teleop_bridge.py:214-245`). 그래서 `transmit` 에서 링크가 끊긴 뒤 다시 열린 첫 주기의 `receive` 는, 단절 전 기준점으로 `d` 를 계산할 수 있다. 이를 막으려고 `McuLinkHost.disconnected()` 가 `_state_delay_s = None` 으로 두고, 생성자의 `on_disconnect` 콜백으로 다리의 `DelayBound` 를 즉시 다시 시작하게 한다. 재연결 뒤 첫 STATE 들은 `d = None` 이므로 지금 규칙을 따르고, 무장은 이미 철회되어 있다.

### 3.11 MCU 시계 속도 감시 (검토 ② P1, ③ 1)

`mcu_gap` 판정과 DelayBound 는 둘 다 MCU 시계가 A3(≤ 100 ppm) 안이라는 전제에 기댄다. 그런데 H743 은 HSE 가 `WAIT_CYCLES` 안에 준비되지 않으면 HSI 로 폴백하고(`firmware/boards/weact_h743/src/clock.c:91-99`), 그 시계원은 STATE 에 실리지 않는다. HSI 의 오차는 데이터시트상 % 단위다(확인 필요: H743 HSI 정확도 수치). 그래서 `"mcu"` 모드에 온라인 감시를 둔다.

- **속도 범위**: 기준점 하나는 (쓰기 시각 `w`, 그 seq 를 처음 알린 STATE 의 MCU 시각 `m`, 그 다음 STATE 의 수신 `rx`)다. 그 MCU 시각에 해당하는 호스트 시각은 `[w, rx]` 안에 있다. 두 기준점 i<j 사이의 속도 범위는 다음과 같다.
  `r ∈ [Δm / (rx_j − w_i), Δm / (w_j − rx_i)]`
  이 식은 경로 지연이 비대칭이든, 그 지연이 변하든 그대로 성립한다. 지연은 두 구간 안에만 들어가기 때문이다. 범위의 폭은 약 `(a_i + a_j) / Δhost` 다(a = 왕복). 2 s 기준점 창으로는 왕복 30 ms 짜리 둘에서 3% 라, 0.5% 를 가를 수 없다(검토 ② P2).
- 그래서 속도 감시는 DelayBound 의 2 s 창과 따로, **세션의 첫 기준점**(왕복이 가장 짧은 것을 유지)과 현재 기준점으로 기선을 길게 잡는다. 판정은 세 가지다.
  - 범위가 `[1 − ρ_max, 1 + ρ_max]` 안에 들어오면 `rate_verified`.
  - 범위 전체가 밖이면 `clock_rate` 사건을 내고, 세션 동안 `"host"` 로 둔다.
  - 아직 둘 다 아니면 미정.
  기선이 범위 폭을 ρ_max 아래로 줄일 만큼 길어지면 판정이 난다(값과 기선은 아래).
- **`"mcu"` 규칙은 `rate_verified` 뒤에만 켠다**(검토 ② P1 의 정책안). 그 전에는 지금 규칙이다. 부팅이 바뀌거나 재연결하면 미정으로 돌아간다.
- **다리의 무장도 `rate_verified` 뒤에만 허용한다**(검토 ② 5 회차). 지금의 DelayBound 와 엔코더 속도 판정도 A3 에 기댄다. HSI 로 부팅한 보드(% 단위 오차)에서는 2 s 기준점 창에서 지연 상한이 수십 ms 작게 나올 수 있다(검토 ③ 1). 그래서 속도가 확인되기 전의 무장은 D5 와 상관없이 근거가 없다. 다리의 `_arm_blocker`(`real_teleop.py:690-704`)에 `clock_rate_unverified` 를 더한다. 범위를 벗어난 보드는 `clock_rate` 로 그 부팅 동안 무장을 거부하고, 콘솔에 이유를 띄운다.
  - 무장 전 판정에 필요한 기준점은 무장 전 핑이 만든다. 지금도 무장 전에는 DISARM 핑으로 기준점을 잇는다(상한이 없으면 0.1 s, 있으면 0.5 s, 세션 확인 뒤: `real_teleop.py:660-687`). 약 12 s 는 왕복이 약 30 ms 이고 핑이 끊기지 않을 때의 **최소 추정**이다. 세션 확인 재시도, 응답 유실, 지연 변동이 있으면 더 걸린다. 도착일 절차에는 "콘솔이 `rate_verified` 를 보일 때까지 기다린 뒤 무장"으로 적고, 시간을 약속하지 않는다.
  - **판정이 미정으로 돌아가는 모든 경계에서 권한과 출력을 끊는다.** 경계는 세 가지다.
    - 부팅이 바뀜: `boot_changed` 로 철회한다. 지금과 같다.
    - 재연결: `disconnected` 로 철회한다. 지금과 같다.
    - 범위를 벗어남: `clock_rate` 다. 무장 중이면 `_revoke("clock_rate")` 와 DISARM 을 큐에 넣는다. 새 규칙이다.
    그 뒤로 판정이 다시 `rate_verified` 가 되기 전까지는 `_arm_blocker` 가 무장을 막는다. 그러므로 READY 는 판정이 확인된 상태에서만 존재한다.
  - 비용은 세션 시작의 대기 약 12 s 다. 이득은 HSI 보드에서 무장 자체를 막는 것이다. bring-up 도구(`bringup.py`)는 DelayBound 가 없고 저속 시운전용이라 이번 범위 밖이다. 확인 필요: bring-up 에도 같은 확인을 둘지.
- 값: **`ρ_max = 0.001`(1,000 ppm)**. 처음에는 0.005 로 썼으나, 총괄 결정 4 의 대조에서 0.005 로는 HSI 폴백을 놓친다.
  - HSI 의 공장 보정 정확도는 30 °C 에서 ±0.4 %(63.7–64.3 MHz)다. 출처는 STM32H5 교육 자료와 AN6202, 그리고 ST 커뮤니티에 인용된 H743 데이터시트 값이다. DS12110 원문 표는 확인하지 못했다 — **인용 값, 원문 미확인**(총괄 2026-11-15: 원문을 확인하면 쪽·표 번호를 여기에 적는다). 0.4 % 는 0.5 % 안에 들어가므로 0.005 로는 걸러지지 않는다.
  - 0.001 은 A3(100 ppm)의 10 배다. 그래서 정상 HSE 크리스털(수십 ppm)은 통과한다. ±0.4 % 의 HSI 는 오차가 0.1 % 를 넘으면 걸러진다.
  - 우연히 0.1 % 안에 든 HSI 도 있을 수 있다. 그래서 `rate_verified` 뒤 DelayBound 의 드리프트 여유는 A3 의 100 ppm 이 아니라 **확인된 상한 ρ_max** 를 쓴다. 2 s 기준점에서 늘어나는 여유는 2 ms 다. 이렇게 하면 "통과한 보드의 실제 속도 오차 ≤ 여유" 가 늘 성립한다.
  - 판정에 드는 기선은 범위 폭 `(a_i + a_j)/Δhost ≤ ρ_max` 에서 나온다. 왕복이 가장 짧은 기준점 둘(3.3b 의 시각이면 수 ms)을 쓰면 수십 초 안이다. 그래서 3.11 의 "약 12 s" 는 더 길어질 수 있다. 콘솔에 진행을 보인다.
  - 두 값(ρ_max, loop_slow)은 총괄이 임시값으로 승인했다. Jetson·실차 측정 뒤 다시 본다.
- `mcu_gap` 은 `(Δmcu)·(1 + ρ_max)` 로 보수적으로 환산해 판정한다(3.3a 의 ρ).
- **운행 중에 시계가 바뀌는 경우**: H743 펌웨어는 운행 중 HSE 를 잃으면 CSS → NMI → `safe_stop` 으로 멈추고, IWDG 로 다시 시작한다(`firmware/boards/weact_h743/src/vectors.c:29-33`, CSS 켜기 `clock.c:128`). 그러므로 시계원은 부팅 때만 정해지고, 바뀌면 `boot_id` 가 바뀐다. 호스트는 그때 권한을 철회하고 속도 판정을 미정으로 되돌린다. "검증 뒤 시계가 갑자기 바뀌는" 경우는 펌웨어가 리셋으로 바꿔 놓는다. 그래도 감시는 세션 동안 계속 판정을 갱신하고, 범위를 벗어나면 그 즉시 `"host"` 로 돌아간다.
- 펌웨어 쪽 근본 대책은 STATE 에 시계원 비트를 싣는 것이다. 이는 다음 판 계약 후보(mcu-link/4)로 넘기고, 펌웨어-H743 과 합의한다.

### 3.12 느린 루프가 계속될 때 (검토 ③ 4)

(가)를 적용하면 40–80 ms 간격으로 루프가 계속 늦게 돌아도 링크는 끊기지 않는다. 그런데 다리는 고정 주기 `cycle_s = 0.02` 로 조향 slew(`real_teleop.py:745`), 탈출 정지 거리(`:423`), 중재기 `tick_s` 를 계산한다. 그래서 느린 루프가 지속되면 이 식들이 실제보다 낙관적이 된다. 다음 두 가지를 둔다.

- 다리는 실제 주기 간격 `g` 를 쓴다. 정지 거리는 `t = age + max(g, cycle_s) + OUT_LIFE`, slew 는 `min(g, 0.1)` 로 계산한다.
- 한 번의 느린 주기(검토 ② P2)를 받아들이는 근거는 이것이다. MCU 가 옛 명령으로 움직이는 시간의 상한은 DRIVE 의 수명이다(결정 + 100 ms, `OUT_LIFE_NS`, MCU 가 강제). (가)는 이 수명을 바꾸지 않고 링크 끊김 판정만 바꾼다. 그러므로 한 번의 80 ms 간격 동안 차가 옛 명령으로 가는 시간은 지금과 같은 100 ms 안이다(텔레옵 0.2 m/s 에서 20 mm 이하). 이를 확인하는 시험을 둔다(5.1a).
- 1 s 창에서 `g > 40 ms` 인 주기가 3 번 이상이면 `loop_slow` STOP 을 낸다. 운전자가 다시 무장할 수 있게 링크 끊김과는 구분해 기록한다. 값은 측정 근거 없이 정한 첫 값이다. 1,080 s 기록에 대 보면 40 ms 넘은 주기 7 번 가운데 1 s 안에 3 번 모인 곳은 없다(부하 창 687.1–687.2 s 에 2 번). 그래서 이 기록에서는 STOP 이 나지 않는다. 총괄 확인 필요.

## 4. 위험과 반례

1. **호스트 정지 + 전송로 정체가 함께 옴**: 정지 뒤 읽은 가장 새 STATE 도 오래됐다. 간격 검사만으로는 숨는다(MCU 간격 10 ms, 호스트 간격 < 150 ms). `D` 가 막는다(나이 > 50 ms). **그래서 `"mcu"` 는 `delay_bound` 없이는 켤 수 없게 한다.**
2. **지속 지연(끊김 없이 모든 STATE 가 60 ms 늦음)**: 지금 호스트 규칙도 못 잡는다. `D` 로 새로 잡힌다(개선).
3. **`D` 의 과소평가**: 전제는 펌웨어 주기 순서(측정 → 프레임 처리 → STATE: `real_teleop.py:160-166`), A3 ≤ 100 ppm, 기준점 ≤ 2 s 다. 펌웨어 순서가 바뀌거나 MCU 시계가 크게 틀리면(PLL 설정 오류) 상한이 깨진다. A3 은 실차 기록의 `bringup.py clock` 드리프트로 확인한다(7절).
4. **MCU 시계가 느림**: 실제 100 ms 침묵이 MCU 시각으로 50 ms 로 보이면 MCU 간격 검사가 늦는다. 호스트 상한 150 ms 와 `tick` 의 나이 검사(호스트가 도는 동안은 호스트 시계)가 덮는다. 빠른 시계는 보수적인 쪽이다.
5. **MCU 리셋**: 새 `boot_id` → 철회(변경 없음). 2⁻³² 의 같은 `boot_id` 에서는 MCU 시각이 0 근처로 돌아가 역행으로 거부되고, 호스트가 도는 동안 `tick` 이 50 ms 에 `stale` 을 낸다.
6. **재생·중복 STATE**: 같은 부팅의 같은·낮은 시각은 거부한다. 재연결 뒤에는 호스트가 이전 STATE 를 잊어 옛 STATE 를 받을 수 있다(지금과 같음). 하지만 `DelayBound.restart_check` 로 `d = None` → 호스트 규칙이고, 권한은 이미 철회됐으며, 다리는 옛 표본을 버린다(`real_teleop.py:477-487`).
7. **pty 와 USB CDC 의 차이**: pty 에는 USB 의 패킷(64 B, STATE 92 B 는 둘로 나뉨), cdc_acm 의 tty 버퍼·조절, 장치 쪽 송신 FIFO 와 DTR 규칙(`real_mcu_link.py:37-40`)이 없다. 예상: 앱이 멈춰도 호스트 컨트롤러는 IN 을 계속 폴링하므로, 지연은 커널 버퍼에 쌓이고 가장 새 STATE 는 신선하다. 장치 쪽에서 버리는 일은 커널 버퍼가 찰 때(초 단위)뿐일 것이다. 확인 필요: 실차 H743 USB 에서 정지 주입 측정.
8. **한 묶음의 처리 시간**: 정지 뒤 여러 STATE 를 처리하는 동안 `now` 는 고정이다. 처리 시간만큼 나이가 과소평가되지만 ms 미만으로 예상한다.
9. **엔코더 속도의 `dt_max_s` 25 ms**: MCU 시각 간격 25–50 ms 는 (가)로는 끊김이 아니지만 속도가 무효라 주행 중이면 STOP 이다(643.2 s 사례). 이 메모에서 바꾸지 않는다.

10. **USB CDC 와 Jetson 의 커널 쪽 동작**(검토 ③ 6): 펌웨어는 DTR·suspend·resume·mount 콜백마다 송신 링 전체를 버린다(`link_usb.c:114`). Jetson 의 `usbcore.autosuspend` 나 ModemManager(ST VCP VID/PID 탐지)가 DTR 을 건드리면 STATE 가 사라진다. 그 결과는 MCU 시각 간격으로 보이므로 `stale` 쪽(안전)이다. 다만 이것이 되풀이되면 운용할 수 없다. 도착일 절차에 udev 규칙(ModemManager 무시)과 autosuspend 끄기를 확인 항목으로 넣는다. 확인 필요.
11. **비대칭 지연**(검토 ③ 8): 왕복 상한은 송신 쪽 지연까지 STATE 의 지연으로 센다. 그래서 거짓 `stale`(가용성) 쪽으로 기운다. 쓰기 시각을 실제 `write` 직후로 바꾸는 것(3.3b)이 이 영향을 줄인다.
12. **STATE 주기 10 ms 전제**(검토 ③ 12): 나이 여유(9 절 ①: 약 16 ms)는 STATE 주기 10 ms 와 다리 주기 20 ms 에서 나온 값이다. 계약 5 절이 허용하는 더 긴 주기에서는 성립하지 않는다. `"mcu"` 모드는 시작할 때 실측 STATE 주기가 `dt_max_s`(25 ms) 이하인지 확인하고, 아니면 켜지 않는다.
13. **`now` 역행**(검토 ③ 13): `rx < _state_rx_ns` 이면 `state_invalid("host_clock")` 로 거부한다. monotonic 이므로 원래는 나지 않는다.

## 5. 시험 목록

### 5.1 단위 — `tests/unit/plant/test_mcu_host.py`

새 픽스처 `McuRig(Rig)`: `HostConfig(state_max_age_s=0.05, cap_ms=150, …, gap_clock="mcu", host_gap_max_s=0.15)`. 시험이 STATE 마다 방출 호스트 시각을 정하고, 가짜 `delay_bound` 가 `rx − 방출 + 여유`(또는 None)를 돌려준다. `burst(stall_ms, period_ms=10, newest_delay_ms=…)` 는 정지 동안 만든 STATE 들을 `on_bytes` 한 번에 넣는다.

| 이름 | 준비 | 단언 |
|---|---|---|
| `test_mcu_clock_host_stall_with_states_queued_is_not_stale[30,50,70,80]` | `ready()` 뒤 G ms 동안 10 ms 간격 STATE 를 쌓아 한 묶음으로 넣는다. 가장 새 지연 5 ms | `stale` 없음, `authorized`·`ready` 유지, `health.fresh`, `drive()` 가 None |
| `test_host_stall_over_the_cap_is_stale[150.000001,166,327.9]` | 같은 방식, 호스트 간격 > 150 ms | `stale(why=host_gap)`·`authorization_revoked`, `drive` → `Blocked("not_authorized")` |
| `test_host_stall_at_exactly_the_cap_is_not_stale` | 호스트 간격 정확히 150 ms | `stale` 없음(`≤` 경계) |
| `test_mcu_silence_60ms_is_stale_while_the_host_runs` | MCU 가 60 ms 동안 STATE 를 안 냄. 20 ms 마다 `tick` | `tick` 에서 `stale(why=age)` 한 번, 철회 |
| `test_mcu_silence_60ms_is_stale_on_receipt_after_a_host_stall` | 호스트도 60 ms 정지. 다음 STATE 가 MCU 간격 60 ms 로 도착 | 받는 즉시 `stale(why=mcu_gap)`·`recovered`, 이후 `ready` 아님 |
| `test_mcu_gap_boundary[50 → ok, 50.001 → stale]` | MCU 간격 50 ms / 50.001 ms | 표대로 |
| `test_delayed_burst_with_old_mcu_times_is_stale` | 호스트 60 ms 정지 + 묶음의 가장 새 STATE 지연 60 ms(전송로 정체) | 묶음 끝에서 `stale(why=age)`, 철회 |
| `test_delayed_burst_under_the_bound_is_fresh` | 같은 조건, 가장 새 지연 40 ms | `stale` 없음, `link_age_s` ≈ 0.04 |
| `test_persistent_delay_is_stale` | 호스트는 정상 주기, 모든 STATE 지연 60 ms(간격 없음) | 첫 묶음 끝에서 `stale(why=age)`. 같은 입력의 `"host"` 모드는 `stale` 없음(개선 고정) |
| `test_older_states_in_a_batch_do_not_trip_the_age_check` | 70 ms 정지 묶음, 첫 STATE 지연 70 ms·가장 새 지연 5 ms | `stale` 없음 |
| `test_unknown_delay_falls_back_to_the_host_rule` | `delay_bound` 가 None. 60 ms 정지 + 쌓인 STATE | `stale(why=host_gap)` (지금과 같음) |
| `test_boot_change_after_a_stall_uses_the_host_rule` | 60 ms 정지 뒤 새 부팅(MCU 시각이 더 작음) | 음수 간격 없음, `stale`·`boot_changed`·철회 |
| `test_first_state_and_reconnect_have_no_gap` | 긴 호스트 시간 뒤 첫 STATE, `disconnected()` 뒤 첫 STATE | `stale` 없음, `first_state` |
| `test_drive_validity_still_counts_from_host_receipt_after_a_stall` | 70 ms 정지 묶음 뒤 `drive(expires=now+100ms)` | `echo` = 가장 새 `mcu_time_us`, `valid_after_echo_ms = min(150, expires − now)` |
| `test_out_of_order_and_duplicates_do_not_refresh_in_mcu_mode` | 같은·낮은 MCU 시각 | `state_out_of_order`, 신선도는 이전 STATE 기준(`test_out_of_order…`, `:283-289` 의 짝) |
| `test_mcu_gap_clock_config_validation` | `delay_bound` 없이 `"mcu"`, `host_gap_max_s` < 0.05 또는 > 0.15 | `ValueError` |
| 기존 시험 | `"host"` 기본값 | `test_stale_link_revokes…`(`:131-143`)·`test_a_state_arriving_after_a_stale_gap_still_revokes`(`:327-334`) 그대로 통과. 뒤의 것은 `McuRig` 짝 하나를 더한다(양쪽 시계 60 ms → `mcu_gap` 으로 여전히 `stale`) |

### 5.1a 검토 반영 추가 시험

| 이름 | 준비 | 단언 |
|---|---|---|
| `test_a_stale_gap_inside_a_batch_is_not_reauthorized_by_later_states` | READY 에서 300 ms 정지, 묶음 [s0(간격 300), s1, s2](모두 ARMED·ZERO_SEEN) | `stale(host_gap)`·철회, 묶음 끝 `authorized` 거짓, 새 `request_arm` 없이는 READY 로 돌아가지 않음 |
| `test_forward_transitions_use_the_newest_state_of_a_batch` | ARM_PENDING 중 80 ms 정지, 묶음 [ARMED@t1 … ARMED@t8] | zero DRIVE 한 개, echo = t8 의 `mcu_time_us`, 조향 = t8 의 값 |
| `test_receipt_time_is_the_read_time` | 링크의 시계 함수를 주입: 주기 시작 t0, `os.read` 반환 뒤 호출이 t0+30 ms 를 돌려줌 | `on_bytes` 에 t0+30 ms 가 넘어감, `_state_rx_ns` = t0+30 ms, `valid_after_echo_ms(expires) = expires − (t0+30 ms)` |
| `test_link_records_the_read_and_write_offsets` | 같은 주입 | 기록 행에 `read_after_cycle_ms`(=30)·`write_before_ms`(쓰기 호출 전 시각 − 주기 시작)가 실림 — 구현 뒤 측정에서 시각 오차의 실측 상한을 내는 근거 |
| `test_reconnect_resets_the_delay_bound_before_the_first_state` | 무장 중 `transmit` 에서 LinkLost → 0.5 s 뒤 재연결, 첫 `receive` 에 단절 전 STATE 바이트 | 첫 STATE 의 `d is None`(지금 규칙), `authorized` 거짓, `health().delay_s is None`, DelayBound 기준점 0 |
| `test_slow_mcu_clock_is_caught_by_the_host_tick[80,100]` | MCU 시계 0.5 배, STATE 가 호스트 기준 80·100 ms 마다, d = 10 ms, tick 20 ms | 80: +60 ms tick 에서 `stale(age)`, 100: 같음 |
| `test_arm_waits_for_rate_verification` | 연결 직후(기선 < 12 s), 무장 전 핑만 | `request_arm` 이 `_arm_blocker` 의 `clock_rate_unverified` 로 막힘. 기선이 차면 무장 가능. 시계 1.02 배 보드는 `clock_rate` 로 무장 거부가 그 부팅 동안 유지 |
| `test_rate_verdict_reset_drops_ready_and_output[boot_change,reconnect,out_of_range]` | READY·주행 중 세 경계 각각 | 그 주기에 `authorized` 거짓, 이후 DRIVE 0 개, out_of_range 는 DISARM 큐, 판정 회복 전 `request_arm` 거부 |
| `test_unverified_rate_with_sustained_delay_never_moves` | 미정 구간에 STATE 가 40 ms 간격·지속 지연 60 ms, `d` 는 None 또는 작게 나옴 | ARM 거부, DRIVE 한 개도 나가지 않음 |
| `test_mcu_rule_waits_for_rate_verification` | 세션 시작, 기준점 기선 < 12 s | 30–80 ms 정지 묶음이 `stale(host_gap)`(지금 규칙). 기선을 채워 `rate_verified` 뒤 같은 정지는 `stale` 없음 |
| `test_rate_range_holds_under_asymmetric_and_varying_delay` | 정상 시계(1.0, 1.0001), 송신 쪽만 0–40 ms 변동하는 지연 | `clock_rate` 사건 없음, 범위가 참값을 포함 |
| `test_write_time_is_taken_before_the_write` | 쓰기가 30 ms 걸림 | 기준점 `w` = 호출 전 시각, `a_b` 가 30 ms 를 포함 |
| `test_one_slow_cycle_keeps_motion_within_the_drive_life` | 주행 중 한 번 g = 80 ms(모형) | MCU 가 그 DRIVE 의 수명 뒤 멈추거나 다음 DRIVE 를 받음, 옛 명령으로 가는 시간 ≤ 100 ms |
| `test_clock_rate_out_of_range_falls_back_to_the_host_rule[0.98,1.01,1.02]` | 기준점 기선이 MCU 시계 0.98·1.01·1.02 배를 보임 | `clock_rate` 사건, 이후 30–80 ms 정지 묶음에서 `stale(host_gap)`(지금 규칙) |
| `test_mcu_gap_uses_the_rate_margin` | MCU 간격 49.8 ms | `(1+ρ_max)` 환산으로 50.05 ms > 50 → `stale(mcu_gap)` |
| `test_anchor_expiry_while_armed_falls_back` | 무장 중 기준점이 2 s 경계를 넘음(DRIVE 응답이 끊김) | 2 s 직전 `d` 있음·직후 `None` → 지금 규칙. 다리는 속도 무효(`no_delay_bound`)로 STOP |
| `test_sustained_slow_loop_stops` | 다리 주기 g = 60 ms 가 1 s 에 3 번 | `loop_slow` STOP, 링크 사건 없음. g = 35 ms 반복은 STOP 없음 |
| `test_stop_reach_and_slew_use_the_real_cycle_gap` | g = 80 ms 한 주기 | 정지 거리 식의 t 가 80 ms 를 씀, slew 가 `min(g, 0.1)` |
| `test_host_clock_regression_is_rejected` | `rx < _state_rx_ns` | `state_invalid("host_clock")`, 갱신 없음 |
| `test_mcu_mode_needs_a_short_state_period` | 실측 STATE 주기 30 ms | `"mcu"` 켜지지 않음(시작 거부 또는 `"host"` 로 남고 사건) |

### 5.2 종단 — `tests/integration/test_teleop_bridge.py` (모형 pty, `Run`·`PtyDevice`, `:32-160`)

정지는 `r.cycles` 사이에 `time.sleep` 으로 넣고, **측정한 주기 간격**(`recs[i]["now_ns"]` 차)으로 단언한다. sleep 이 넘치면 G 가 커지므로, 목표 구간을 벗어난 시행은 한 번 다시 한다.

| 이름 | 준비 | 단언 |
|---|---|---|
| `test_a_bridge_stall_with_states_queued_keeps_the_arm[30,50,70,80]` | `to_ready()` → 데드맨 정지 10주기 → 정지 → 10주기 | 어떤 기록에도 `stale` 없음, `phase == "ready"`, `stop`·`disarm` 없음. MCU `ARMED` 는 측정 G ≤ 80 ms 일 때만 단언(3.6) |
| `test_a_bridge_stall_between_mcu_expiry_and_the_cap_stops_by_the_mcu` | G ≈ 120 ms, 모형 `--stop-grace-ms 300`(실보드 값)과 100 둘 다 | 호스트 `stale` 없음, MCU 가 멈춤(`mcu_stopping`, grace 100 에서는 `mcu_disarmed` 도 허용 — 검토 ③), 그 뒤 DRIVE 가 정지를 취소하지 않음 |
| `test_a_bridge_stall_at_the_cap_is_held_by_the_mcu_deadline` | 측정 G 가 145–150 ms(모형) | 호스트 `stale` 없음(경계 ≤), MCU 가 마지막 결정 + 100 ms 안에 STOPPING(또는 grace 100 에서 DISARMED). 깨어난 뒤 첫 DRIVE 의 echo 는 가장 새 STATE |
| `test_a_bridge_stall_over_the_cap_disconnects[166,328]` | G = 166·327.9 ms 근처 | `stale(why=host_gap)`·`authorization_revoked`, 중재기 STOPPED, 뒤에 MCU `DISARMED` |
| `test_mcu_silence_is_stale_end_to_end` | 모형 프로세스를 SIGSTOP 60 ms 후 SIGCONT, 다리는 계속 돔 | `stale`, 중재기 STOP. 모형의 시각은 `CLOCK_MONOTONIC`(`firmware/tools/mcu_model.c:68`)이고 주기는 `TIMER_ABSTIME` 잠(`:341`)이라, 멈춘 동안 MCU 시각 간격이 그대로 생긴다 |
| `test_a_delayed_burst_end_to_end` | 모형 ↔ 다리 사이에 중계 pty(`DelayRelay`: 바이트를 T ms 붙잡았다 한꺼번에 흘림). ① 다리도 T = 60 ms 정지 ② 지속 지연 60 ms | 두 경우 모두 `stale(why=age)`, 철회 |
| `test_receive_cut_still_goes_stale` (기존 경로 확인) | 수신만 끊김 | `tick` 에서 50 ms `stale`(변경 없음) |
| `test_tools_keep_their_gap_clock` (단위로 둬도 됨) | 설정 읽기 | `teleop_bridge.HOST_CFG.gap_clock == "mcu"`, `bringup.HOST_CFG`·`COMMISSION_HOST_CFG`·모니터는 `"host"` |

### 5.3 구현 전 기록 재평가 (시험이 아니라 판정 근거)

1,080 s 기록의 state 행(`host_ns`, `mcu_time_us`)과 cycle 행 `delay_s` 로 새 규칙을 오프라인으로 돌려 본다. 기대: `stale` 은 호스트 간격 150 ms 초과 수(327.9 ms 최소 1번. 부하 구간의 나머지 두 50 ms 초과 값은 다시 셈). `why=age` 는 0번(속도 무효 주기와 대조). 0이 아니면 구현 전에 원인을 본다.

## 6. (나) 지터 자체를 줄이는 후보 (구현은 측정 근거 뒤 결정)

먼저 원인을 나눌 계측: 주기 행에 수신·판단·송신 각 단계의 소요 시간, `gc.callbacks` 로 GC 시작·끝과 세대, 프로세스 CPU 시간을 더한다. 50 ms 를 넘은 간격마다 원인을 붙일 수 있게 한다.

| 후보 | 저장소에서 확인한 사실 | Jetson(JetPack 7.2.1, Ubuntu 24.04) 사용 가능성 |
|---|---|---|
| ① 메모리·GC: `TeleopBridge.records` 를 묶음 | `real_teleop.py:464` 생성, `:579` 매 주기 append, 자르는 곳 없음 → 1,080 s 에 약 5.4만 dict. 2세대 GC 비용이 실행 시간에 따라 커진다는 가설은 무부하 50 ms 초과 두 번이 839 s·1,065 s 에 나온 것과 맞지만 **증명 안 됨** | 권한 불필요. `deque(maxlen)`·`gc.freeze()`(시작 뒤) 모두 표준 Python. 시험이 `records` 를 읽는지 먼저 확인 필요 |
| ② 기록 쓰기를 고리 밖으로 | **이미 고리 밖 스레드다**: `RecordWriter`(`record_writer.py:64-81`, 유한 큐 + 스레드, 설계 3절). 다만 **같은 프로세스**라 `json.dumps` 가 GIL 을 놓고 다툰다. 콘솔 서버(`LiveServer`, `teleop_bridge.py:374-376`)도 같은 프로세스다. `live/io_writer.py`·`spool.py` 는 Isaac 라이브 세션 것이고 다리는 쓰지 않는다 | 후보: 직렬화·쓰기를 별도 프로세스로, `sys.setswitchinterval` 을 낮춤(기본 5 ms). 권한 불필요. 효과는 측정해야 함 |
| ③ 스레드 우선순위: `os.setpriority`(음수 nice), `os.sched_setscheduler(0, SCHED_FIFO, …)` | 저장소에 근거 없음 | 음수 nice·SCHED_FIFO 는 CAP_SYS_NICE 나 `RLIMIT_RTPRIO > 0`(limits.d) 이 필요하다. Ubuntu 기본 사용자 rtprio 값은 확인 필요(`ulimit -r`). **GIL 은 우선순위 상속이 없어**, 주 스레드만 RT 로 올려도 GIL 을 쥔 보통 스레드(기록기·서버)에 막힌다. ②와 함께여야 의미가 있다. `sched_rt_runtime_us` 조절도 확인 필요 |
| ④ PREEMPT_RT 커널 | 저장소에 근거 없음 | 확인 필요: JetPack 7.2.1 의 Orin Nano 용 RT 커널 제공 여부와 설치 방법 |
| ⑤ CPU 고정 | 저장소에 근거 없음 | `os.sched_setaffinity` 는 권한 불필요. `isolcpus`·`nohz_full` 은 부트 명령줄(Jetson 은 extlinux.conf)이라 확인 필요. 코어 수(Orin Nano 6코어)는 카탈로그 값, 실기 미확인 |
| ⑥ CPU 주파수·전원 모드 | 저장소에 근거 없음 | 확인 필요: `nvpmodel` 모드(MAXN SUPER)와 `jetson_clocks` 고정이 지터에 주는 효과 |

공통: 팀은 Jetson Orin Nano Super 를 갖고 있으나(2026-11-11) 아직 LAN 에서 닿지 않는다. 위 표의 Jetson 열은 하나도 실기에서 확인하지 않았다.

## 7. 확인 필요 목록

1. 조건 3 을 `clock_map` 대신 `DelayBound`(왕복 상한)로 구현하는 것(3.4). `clock_map` 은 A3 확인용 오프라인 도구로만 쓴다.
2. 80–150 ms 회색 구간: 호스트 `stale` 대신 MCU 만료(`OUT_LIFE_NS` 100 ms)로 멈추는 것을 받아들일지(3.6).
3. 계약 4절 `:162` 문구 개정(와이어 변경 없음, version 유지)에 펌웨어-H743 합의가 필요한지.
4. bring-up 도구에도 적용할지. 하려면 `DelayBound` 를 먼저 옮겨야 한다(3.9).
5. 실차 H743 USB CDC 에서 정지 주입 측정(4.7)과 A3 드리프트(`bringup.py clock`).
6. `rx = 주기 시작` 으로 인한 1 ms 미만 마감 초과(3.5) — 별건으로 다룰지.
7. Jetson 권한·커널·전원 모드(6절 표의 Jetson 열 전부), Jetson 확보 상태.
8. 3.11 의 `ρ_max = 0.005` 와 H743 HSI 정확도, 시계원 비트(mcu-link/4 후보)의 펌웨어-H743 합의.
9. 3.12 의 느린 루프 STOP 값(40 ms · 1 s · 3 번).
10. 도착일 절차에 Jetson 의 ModemManager 무시 udev 규칙과 USB autosuspend 끄기를 넣을지(4 절 10).

## 9. 자체 검토

### ① 원자료 재계산 (담당 세션, 2026-11-15)

1,080 s 기록(state 행 `host_ns`·`mcu_time_us`, cycle 행 `delay_s`)에 5.3 의 재평가를 돌렸다. 이 기록은 세션 임시 원자료이고, 판정 근거로 삼지 않으며 리허설 기록 D5 절에 요약되어 있다.

| 규칙 | `stale` |
|---|---|
| 지금(호스트 수신 간격 > 50 ms) | 5 |
| (가) 호스트 간격 > 150 ms | 1(643.9 s, 327.9 ms) |
| (가) MCU 시각 간격 > 50 ms | 0(최대 30.8 ms, 643.2 s 모형 굶음) |
| (가) 나이 `now − rx + delay_s` > 50 ms | 0(최대 34.0 ms) |

`delay_s` 는 p50 20.0, p99 22.8, p999 26.3, 최대 34.0 ms 이고 None 이 6 주기다. 수신 시각이 주기 시작이라 왕복 상한에 한 주기 가까이가 들어간다. 정상 운용에서 나이 판정의 여유는 약 16 ms(50 − 34)다. 이 기록의 시각은 주기 시작 값이다. 따라서 위 숫자는 간격 규칙(호스트 150 ms·MCU 50 ms)의 근거일 뿐, 3.3b 의 실제 읽은 시각·쓰기 시각에서 나이가 얼마일지, `rate_verified` 전 구간이 얼마나 길지의 근거는 아니다. 구현 뒤 같은 측정 도구로 다시 재고(노트북·Jetson), 그때 None 구간·판정 전 구간·속도 무효 주기를 함께 센다.

### ② Codex 적대 검토 (새 스레드, 2026-11-15)

| 지적 | 판단 | 처리 |
|---|---|---|
| P1 묶음 안 뒤쪽 STATE 가 철회된 권한을 다시 세움 | 반박: 철회는 `_auth = _NONE`, `_NONE` 에서 `_advance_auth` 는 아무것도 안 함, 재무장은 운전자의 새 `request_arm` 필요 | 성질을 3.3a 에 적고 시험으로 고정(5.1a 첫 줄) |
| P1 재연결 뒤 첫 STATE 는 간격 검사가 없음 | 지금도 같음(철회됨, 재무장 필요). 다만 옛 기준점이 남는 경로는 실제로 있음(③ 2) | 3.10 지연 상한 초기화, 시험 |
| P1 MCU 시계가 느리면 150 ms 상한이 못 막음 | 일부 반박: 호스트 루프가 도는 동안은 `tick` 나이 검사가 호스트 시계로 잡음. 정지 구간 안에서만 MCU 시계에 기댐 | 3.11 온라인 시계 속도 감시 + `mcu_gap` 여유 환산, 시험 |
| P2 지연 상한이 None 이면 지속 지연을 못 잡음 | 맞음, 지금 규칙과 같은 수준. 다리는 상한이 없으면 속도 무효로 STOP, 무장도 막힘 | 4 절과 시험(`test_anchor_expiry_while_armed_falls_back`) |
| P2 묶음 안 DelayBound 갱신 순서 불명확 | 맞음 | 3.3a 의사 코드로 고정 |
| P2 재연결 재생 시험 없음 | 맞음 | 5.1a |
| 9 절 수치 재현 절차 없음, 16 ms 여유 불충분 가능 | 맞음 | 9 절 ① 에 계산 방법을 적음. 여유는 Jetson 측정으로 확인(7 절 5) |

같은 Codex 스레드의 후속 회차:

| 회차 | 지적 | 처리 |
|---|---|---|
| 2 | P1 느린 시계 + 계속 도착하는 STATE, P1 시각을 찍는 위치, P2 속도 범위가 2 s 창으로는 0.5 % 를 못 가름, P2 한 번의 느린 주기 | tick 의 검출 상한을 계산식으로 적음, 읽은 시각은 `read` 뒤·쓰기 시각은 `write` 앞, 구간 산술 속도 범위 + 세션 첫 기준점으로 긴 기선 + `rate_verified` 뒤에만 `"mcu"`, DRIVE 수명 논증 |
| 3 | P1 60 ms 반례, P2 시각 오차 상한, P2 주기 정의 | 상한 `state_max_age_s − d + g` 는 지금 규칙의 입도와 같음, 운행 중 시계 변경은 펌웨어가 리셋으로 만듦(`vectors.c:29-33`), 시각 주입 시험과 기록 열 |
| 4 | P1 g = 150 ms 에서 검출 상한 200 ms 가 DRIVE 수명보다 김 | 검출 상한과 운동 상한을 나눔(운동은 MCU 가 강제하는 마감), 145–150 ms 종단 시험 |
| 5 | P1 `rate_verified` 전에도 무장 가능 | 다리의 무장을 `rate_verified` 뒤로(`clock_rate_unverified`) |
| 6 | P1 구현 예정임이 안 적힘·판정이 미정으로 돌아가는 경계, P2 12 s 가 보장처럼 보임 | 상태 줄에 구현 전 설계라고 밝힘, 세 경계(부팅·재연결·범위 밖)에서 철회, 12 s 는 최소 추정 |
| 7 | 새 P1/P2 없음 | — |

### ③ agy(Gemini 4 Hydrogen High) 적히지 않은 전제 (①② 미공개, 2026-11-15)

| 지적 | 처리 |
|---|---|
| H743 HSE→HSI 폴백, 시계원 미보고 | 3.11(인용 `clock.c:91-99` 확인) |
| 재연결 첫 `receive` 가 `restart_check` 전에 옛 기준점 사용 | 3.10(`real_teleop.py:519-522`, 다리 주기 순서 확인) |
| 주기 시작 `now` 공용으로 나이·마감 과소평가 | 3.3b 실제 읽은 시각 |
| 느린 루프 지속 시 고정 `cycle_s` 식 | 3.12 |
| 부풀려진 기준점·`anchors_max` 32 로 거짓 `stale` | 3.3b 쓰기 시각 실측. 기준점 창(50 Hz 에서 0.64 s)은 구현 시험에서 거짓 `stale` 이 없는지 확인 |
| USB CDC DTR·autosuspend·ModemManager 로 송신 링 비움 | 4 절 10(`link_usb.c:114` 확인), 7 절 10 |
| 비대칭 지연 | 4 절 11 |
| 묶음의 옛 STATE 로 zero DRIVE | 3.3a 앞으로 가는 전이는 newest 로만 |
| pty 하네스와 실보드 차이(grace 100 vs 300) | 5.2 grey-zone 시험에 두 값 |
| 85–150 ms 가 `mcu_stopping` 으로 보여 진단 혼선 | 기록에 직전 주기 간격을 함께 남김(구현) |
| STATE 주기 10 ms 전제 | 4 절 12 |
| `now` 역행·`disconnected()` 초기화 | 4 절 13, 3.10 |
| 메모와 코드 모순 3건 | 위 3.10·4 절 10·5.2 로 고침 |

### ①(덧붙임) 계산 방법

state 행 `host_ns` 의 연속 차를 호스트 간격, `mcu_time_us` 의 연속 차를 MCU 간격으로 한다. 나이는 cycle 행마다 `now_ns −` (그 시각까지 받은 마지막 state 의 `host_ns`) `+ delay_s` 로 구한다. 느린 루프 규칙은 cycle 행 `now_ns` 차가 40 ms 넘는 주기를 1 s 창에서 센다.

## 10. 총괄 결정 (2026-11-15)

1. 조건 3 은 DelayBound(왕복 상한)로 구현한다. clock_map 은 A3 확인과 진단에만 쓴다.
2. 85–150 ms 회색 구간은 MCU 의 DRIVE 만료(100 ms)가 맡는다. `mcu_stopping` 은 사건으로 기록하고 콘솔에 보인다.
3. 다리의 무장은 시계 속도 확인 뒤에만 한다. 도착일 절차와 콘솔 화면에 `rate_verified` 상태를 보인다.
4. ρ_max 와 loop_slow(40 ms·1 s·3 번)는 임시값이다. ρ_max 는 HSI 폴백을 잡도록 0.001 로 바꿨다(3.11). 두 값 모두 Jetson·실차 측정 뒤 다시 본다.
5. bring-up 도구의 시계 속도 확인은 이번 범위가 아니다. 후속 과제로 남긴다(지금 규칙 유지).
6. 계약 4 절 문구 개정과 mcu-link/4 시계원 비트는 후보로 기록만 한다. 펌웨어-H743 세션이 재개될 때 합의한다. 지금 계약은 바꾸지 않는다.
7. Jetson 의 ModemManager 무시 udev 규칙과 USB autosuspend 끄기를, 되돌리는 방법과 함께 도착일 절차에 넣는다.
- (나): 단계별 계측(GC 시점 포함)을 먼저 넣는다. `TeleopBridge.records` 가 끝없이 커지는 것은 그 자체로 결함이므로 이번에 고친다. 그 리스트를 읽는 곳은 저장소에 없다(`grep -rn "\.records\b" src tools tests`).
- 구현은 시험부터 쓴다. 머지 전에 Codex 리뷰를 수렴시키고, 리허설 하네스로 E2E 를 확인한다. E2E 조건은 셋이다: 다리 30–80 ms 멈춤은 끊김이 아니다, 150 ms 를 넘으면 끊김이다, MCU 가 60 ms 침묵하면 끊김이다.

## 11. 구현 노트 (2026-11-15, 설계와 달라진 곳)

- **드리프트 여유 = 확인된 범위의 편차.** 3.11 은 "확인 뒤 여유 = ρ_max" 로 썼지만, 구현은 측정한 속도 범위 `[lo, hi]` 의 편차 `max(1 − lo, hi − 1)`(A3 의 100 ppm 아래로는 내리지 않음)를 쓴다. 참값이 그 범위 안에 있으므로 엄밀성은 같다. 기선이 길어질수록 여유가 줄어든다(확인 시점에 ρ_max 이하).
  - 이렇게 바꾼 이유: 시험에서 ρ 를 0.05 로 넓히자 여유 5 % 가 지연 상한을 부풀려, 정상 링크에서도 나이가 50 ms 를 넘었다. 운용 값 0.001 에서는 차이가 작지만, 엄밀하면서 더 좁은 쪽을 택했다.
- **범위는 쌍들의 교집합이다.** 쌍마다 구한 범위가 모두 참값을 담으므로 교집합을 유지한다. 그래서 뒤에 들어온 느슨한 기준점(멈춤 동안의 긴 왕복)이 범위를 넓히지 못한다. 교집합이 비면 한 부팅 안에서 시계가 한 속도를 지키지 않은 것이므로 `out_of_range` 다. 시험: `tests/unit/live/test_rate_guard.py`.
- **판정에 드는 기선은 `max(b_i, b_j)/ρ` 다**(3.11 의 `(a_i + a_j)/ρ` 는 범위 전체 폭 기준). 범위의 양쪽 끝이 각각 `b/기선` 만큼 벌어지기 때문이다. 콘솔의 "n/m s" 는 이 식에서 m 을 낸다.
- **시각.** 링크(`ReconnectingLink(clock=…)`)와 다리(`Bridge(clock=…)`)는 CLI 에서만 시계를 받는다. 시계가 없으면 지금처럼 주기 시작 시각을 쓴다(시험은 시간을 직접 몬다). 시계가 있으면 다음과 같이 바뀐다.
  - 결정 시각은 수신 뒤의 시각이다. 읽은 시각이 주기 시작보다 늦어도 결정 시각이 그보다 앞서지 않는다.
  - 기록에 `cycle_start_ns`, `stage_ms`(rx·dec·tx), `gc_ms` 가 실린다(6 절 (나)의 계측).
- **E2E**(`tests/integration/test_teleop_bridge.py`, 모형 pty, 시험용 ρ 0.05)는 총괄 10 절의 세 조건을 확인한다.
  - 30·50·70·80 ms 멈춤: 무장이 유지된다.
  - 166·328 ms 멈춤: `host_gap` 끊김.
  - 모형 SIGSTOP 60 ms: `age`/`mcu_gap` 끊김.
  - 시험 하네스는 절차와 같이 `rate_verified` 를 확인한 뒤 무장한다.
- **records** 는 길이 64 의 링이다.
- **udev 규칙**은 보드의 USB 장치에 `power/control=on` 을 둔다. 되돌리기는 `deploy/udev/README.md` 의 Undo 절이다.

