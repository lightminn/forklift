# 콘솔 인터페이스 계약 초안 (`console/1`)

작성: 2026-10-13 · 상태: **초안, 9절 항목은 메인 개발과 합의함(2026-10-13), 10절 관찰 전용 프로파일도 합의함(2026-10-15). 재개 절차 이름·사유는 M-a 설계에서 확정(2026-10-15, 6절)** · 설계: [개별 지게차 운용 콘솔 설계](../design/2026-10-13-operator-console.md)

이 문서는 웹 앱과 차량 쪽 게이트웨이 사이의 메시지(`console/1`)와, 그 뒤에서 차량 스택이 지켜야 할 의미를 적는다. ROS 2 msg/action 정의로 옮길 항목은 8절에 표시한다. 차량 쪽(중재기 M-b, 미션 실행부 M-a)이 생기기 전에는 웹 앱의 가짜 차량 백엔드와, 10절의 관찰 전용 게이트웨이(C2a)가 이 형식을 쓴다.

## 1. 공통 규칙

**연결 둘.** 같은 클라이언트가 WebSocket 연결 두 개를 연다. 둘 다 같은 클라이언트·세션에 묶인다. 1단계는 암호화하지 않은 `ws://`를 쓴다(설계 7.1, 사용자 결정).

| 연결 | 싣는 것 | 규칙 |
|---|---|---|
| 제어 (`/api/v1/control`) | 권한·모드·정지·작업 요청, 텔레옵 명령, 상태, 회신 토큰 | 작은 JSON 텍스트 프레임만. 서버 송신 버퍼 상한을 작게 둔다. 상한을 넘으면 연결을 끊는다. 끊김의 결과는 모드에 따라 다르다(2절): TELEOP은 만료 규칙으로 정지하고, AUTO는 작업을 계속한다 |
| 미디어 (`/api/v1/media`) | 카메라·스캔·격자 | 바이너리 프레임(짧은 JSON 머리글 + 본문). 서버는 송신 버퍼가 상한을 넘으면 새 프레임을 보내지 않고 버린다. 하드 상한을 넘으면 미디어 연결만 끊는다 |

연결을 나누는 이유는 TCP 하나 안에서는 이미 송신 큐에 들어간 영상 바이트를 애플리케이션이 버릴 수 없고, 그 뒤의 정지·회신이 그만큼 늦어지기 때문이다. 두 연결도 같은 무선 구간을 쓰므로 패킷 손실의 영향은 함께 받는다. 연결을 나눠 막을 수 있는 것은 영상 적체로 제어 메시지가 늦어지는 것뿐이다.

**단위·좌표.** CONTRIBUTING 5절을 따른다. SI 단위를 쓰고, 이름에 단위 접미사를 붙인다(`speed_mps`, `yaw_rad`, `lift_m`). 자세에는 프레임 이름(`map` 등)과 보정 버전을 같이 싣는다. 속도·곡률은 **뒤 차축 중심** 기준이다(`AckermannCommand`).

**시각.** 모든 시각은 clock domain을 함께 싣는다(`vehicle_mono`, `sim`, `sensor`). 서로 다른 domain의 값을 빼지 않는다. 신선도 판정은 차량 단조 시계(`vehicle_mono`)로만 한다.

**세대(번호의 수명).**

| 이름 | 바뀌는 때 | 쓰임 |
|---|---|---|
| `session_epoch` | 차량 쪽 세션(실행기·게이트웨이 프로세스, Isaac 잡)이 새로 시작할 때 | 모든 메시지. 다르면 거부 |
| `lease_id` | 조종 권한을 줄 때마다(재연결 유예로 돌려줄 때 포함). 예측할 수 없는 값 | 조종 권한이 필요한 모든 요청 |
| `control_gen` | 정지·권한 경계마다 1 증가: STOP·안전 선점, 명령 만료, lease 만료, 텔레옵 중 통신 끊김 정지, 일시정지, 재무장(`rearm`) 완료. AUTO 중 lease 만료에서는 이 값과 권한만 바뀌고 모드·정지 종류·phase는 그대로다 | 조종 권한이 필요한 **모든** 요청(모드·무장·재무장·작업·복구)과 텔레옵 명령. 다르면 `reject(stale_control)`. 정지 경계 전에 보냈지만 늦게 도착한 요청이 경계 뒤에 실행되지 않게 한다 |
| `arm_gen` | 조종 준비(무장)를 할 때마다 1 증가 | 텔레옵 명령. 이전 세대의 명령은 버린다 |
| `state_seq`, `frame_id` | 메시지·촬영마다 증가. 한 `session_epoch` 안에서 재사용하지 않는다 | 회신 판정 |

**메시지 종류별 전달 성질.**

| 성질 | 해당 메시지 | 규칙 |
|---|---|---|
| 신뢰 + 응답 | 조종 권한, 모드 요청, STOP, 재무장, 작업 지시, 복구 동작 | `request_id`로 멱등이다. 차량은 반드시 `ack`(수락) 또는 `reject`(사유)로 답한다. 응답이 없으면 클라이언트는 같은 `request_id`로 재전송한다. 재전송에도 `session_epoch`·`lease_id`·`control_gen`이 붙으므로, 재접속이나 정지 경계 뒤에는 **아직 처리되지 않은** 이전 세대의 요청이 거부된다(STOP 제외: 항상 다시 실행한다. 이미 처리한 다른 요청은 아래 처리 순서대로 원래 결과를 돌려받는다). 클라이언트는 거부를 받으면 현재 세대로 자동 재전송하지 않고 조종자에게 다시 묻는다 |
| 최신값만 | 텔레옵 명령, 입력 상태, 회신 | 큐에 쌓지 않고 마지막 값만 처리한다. 응답하지 않는다. 결과는 상태 메시지로 확인한다 |
| 스트림 | 상태, 회신 토큰, 미디어 | 구독할 때 정한 상한(Hz, 해상도)을 넘지 않는다. 밀리면 오래된 것부터 버린다 |

**요청 처리 순서.** 신뢰 요청을 받으면 차량은 다음 순서로 처리한다.

0. **STOP은 예외다.** STOP은 결과 캐시·세대 검사와 무관하게 정지 효과를 먼저 실행하고 `ack`한다. 결과를 저장하지 않으며, 같은 `request_id`가 다시 와도 다시 정지시킨다. 이미 정지 상태면 정지가 유지될 뿐이라 결과가 같다. 재무장 뒤 늦게 도착한 중복 STOP이 차를 다시 세우는 것은 안전한 쪽의 동작으로 받아들인다. 캐시가 가득 차도 STOP은 막히지 않는다.
1. **이미 처리한 요청인가.** 같은 `client_id`·`session_epoch`·조종 권한 소유자 범위에서 같은 `request_id`를 이미 처리했다면, 저장해 둔 원래 결과(`ack` 또는 `reject`)를 그대로 돌려준다. 다시 실행하지 않고 세대 검사도 하지 않는다. 응답만 유실된 경우(예: 일시정지는 성공했지만 `ack`가 사라져 재전송한 경우)에 원래 성공을 확인할 수 있다.
2. **처음 받은 요청이면** 세대(`session_epoch`, `lease_id`, `control_gen`)를 검사하고, 맞으면 실행한다. 그 결과를 `request_id`로 저장한다.

결과 저장 기간은 해당 `session_epoch`가 끝날 때까지다. 저장 상한(개수·메모리)에 닿으면 저장된 결과를 버리지 않고 새 신뢰 요청을 `reject(busy)`로 받지 않는다. 버린 `request_id`의 재전송을 새 요청과 구분할 수 없게 되는 것을 피하려는 것이다(STOP은 0번대로 영향을 받지 않는다). `request_id`는 클라이언트가 만드는 예측할 수 없는 값이며, 재사용하지 않는다.

**요청 원장(상시 세션, 2026-10-18 추가 — 위 1·2번과 저장 기간을 대신한다).** 계속 띄워 두는 세션에서는 위 규칙대로면 저장소가 결국 다 차서 모든 새 요청이 `busy` 가 된다. 그래서 실시간 실행기(Isaac live)는 다음 원장으로 판정한다(`forklift_core/safety/generations.py` 의 `RequestLedger`, 계획 `docs/plans/2026-10-18-persistent-live-console.md` D7).
- `welcome` 에 `request_token`(정수)을 싣는다. 연결마다 새로 주고, 다시 쓰지 않는다.
- STOP 이 아닌 신뢰 요청은 `request_token` 과 `request_seq`(그 토큰 안의 단조 정수, 텔레옵 `seq` 와 별개)를 싣는다. 재전송은 같은 메시지다. `request_id` 는 회신 짝 맞추기에 그대로 쓴다.
- 원장은 실행 전에 판정한다.
  - 저장된 결과가 있으면 그것을 돌려준다.
  - 이 연결의 토큰이 아니거나(다른 클라이언트, 지난 연결, 위조) `request_seq` 가 워터마크 이하이면 `reject(stale)` 이다. 실행하지 않고 저장하지 않는다.
  - 상한에 닿으면 `reject(busy)` 다.
  - 결과는 확정 뒤 60 s 가 지나면 순번 순으로 지우고 워터마크를 올린다. 결과가 아직 정해지지 않은 미션 요청에서 멈춘다.
- 다시 연결하면 그 클라이언트의 옛 토큰은 새 요청을 받지 않는다. 살아 있는 토큰은 64개까지다.
- 클라이언트는 지난 토큰의 `stale` 을 받으면 그 요청만 끝낸다. 지금 토큰이 `stale` 이면 다시 연결해 새 토큰을 받는다.
- 토큰이 없는 back end(가짜 차량)는 위 1·2번대로 동작하고, 클라이언트도 토큰 없이 보낸다.
- STOP 은 토큰이 없거나 낡아도 언제나 실행한다(0번).

**버전.** 첫 메시지에서 `api: "console/1"`을 맞춘다. 맞지 않으면 연결을 거부한다.

## 2. 연결과 권한

| 메시지 | 방향 | 주요 필드 |
|---|---|---|
| `hello` | C→V | `api`, `client_id`. 인증 필드는 1단계에 없다(설계 7.4). 나중에 이 메시지에 넣는다 |
| `welcome` | V→C | `api`, `vehicle_id`, `session_epoch`, `backend: real \| isaac \| mock`, `capabilities` (3절) |
| `lease_request` / `lease_release` | C→V | `request_id`, `session_epoch` |
| `lease_granted` | V→C | `lease_id`, `lease_timeout_s` |

- 조종 권한(lease)은 한 클라이언트만 가진다. 다른 클라이언트는 관찰만 한다. 다만 STOP은 관찰 클라이언트도 보낼 수 있다(5절).
- lease 보유자는 제어 연결로 회신(4절)을 계속 보낸다. 마지막으로 받은 유효 회신이 담은 상태의 송출 시각에서 `lease_timeout_s`가 지나면 lease가 만료된다. 구분할 것: **콘솔과의 연결이 끊긴 경우**는 아래 규칙을 따르고, **차량 쪽 제어 프로세스가 죽은 경우**는 구동 어댑터의 하위 명령 만료(MCU 자체 시계)로 정지한다(설계 3절). 만료는 텔레옵이면 안전 선점 정지를 일으킨다. AUTO면 작업을 계속하고(사용자 결정, 설계 2절) 조종 권한만 풀린다. 조종자가 유예 안에 다시 붙으면 권한을 돌려받고, 유예가 지나면 다른 클라이언트가 권한을 요청할 수 있다.
- **`client_id`는 자기 신고 값이다.** 1단계에는 인증이 없으므로(설계 7.4) 다른 기기가 같은 `client_id`를 대면, 유예 중인 조종 권한을 새 `lease_id`로 받아 갈 수 있다. 그 기기가 이미 알고 있는 `request_id`로 저장된 결과를 조회할 수도 있다. 유예와 결과 캐시는 기기 신원을 보장하지 않는다. 다만 캐시 반환은 재실행하지 않으므로, 결과 조회만으로 차량이 다시 움직이지는 않는다.
- **재연결 유예.** 끊긴 클라이언트가 `grace_s` 안에 같은 `client_id`로 다시 연결하면 조종 권한을 돌려받는다. 그동안 다른 클라이언트의 `lease_request`는 `reject(lease_reserved)`된다. 권한을 돌려줄 때 **`lease_id`는 새로 발급한다.** 그래서 끊기기 전에 보냈지만 아직 처리되지 않은 요청(`arm`, `mission_resume` 등)은 재접속 뒤에 재전송되어도 `stale_lease`로 거부된다. 유예는 권한만 지키고 움직임은 지키지 않는다. 무장은 이미 풀렸으므로 다시 조종 준비가 필요하다.
- 나중에 중앙 서버를 두더라도, 중앙 서버가 클라이언트 대신 회신을 보내거나 lease를 갱신하지 않는다. 회신 토큰은 클라이언트가 실제로 받은 상태에서만 나온다.

## 3. 지원 범위와 정적 정보

`welcome.capabilities`에는 차량이 지원하는 기능과, 화면이 그림을 그리는 데 필요한 정적 정보를 싣는다. 콘솔은 지원하지 않는 요청 버튼을 숨기거나 비활성화하고, 차량은 그런 요청을 `reject(unsupported)`로 거부한다.

```json
{
  "mission": true,
  "teleop_direct": true,
  "teleop_video": true,
  "lift": true,
  "recovery_actions": ["withdraw_straight"],
  "feedback": {"rear_axle_speed": true, "steering": true, "lift_height": true, "load_state": false},
  "media": {"camera_rgb": {"max_hz": 15, "max_width": 1280}, "scan": {"max_hz": 10}, "grid": {"max_hz": 2}},
  "camera_model": {"calibration_version": 3, "width": 1280, "height": 720,
                   "fx": 0, "fy": 0, "cx": 0, "cy": 0, "base_from_camera": {"xyz_m": [0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]}},
  "chassis": {"rear_from_base_m": 0.0, "footprint_m": [[0, 0]], "fork_tip_from_rear_m": 0.0},
  "provisional_limits": true
}
```

- `feedback`는 차량이 실제로 잴 수 있는 값이다. 잴 수 없는 항목에 의존하는 기능은 그 차량에서 쓸 수 없다. 예를 들어 바퀴 속도를 잴 수 없으면 재무장과 재개가 불가능하다(설계 4.3).
- `camera_model`과 `chassis`는 화면 오버레이(예상 궤적, 바닥 투영)에만 쓴다. 안전 판단에는 쓰지 않는다. 위 숫자는 형식 예시이고 실제 값이 아니다.
- `provisional_limits: true`는 속도 상한과 유효기한이 H3 실측 전의 잠정값이라는 뜻이다. 콘솔은 이를 화면에 표시한다.
- `control`(없으면 `true`)과 `stop_instruction`: `control: false` 는 관찰 전용 연결이다(10절).

## 4. 텔레옵 명령과 신선도

**회신 토큰.** 차량은 제어 연결로 짧은 `tick`을 주기적으로 보낸다. 각 `tick`에는 `state_seq`와 예측할 수 없는 `token`이 있다. 차량은 이 토큰을 보낸 시각(`vehicle_mono`)을 기억한다. 텔레옵 명령과 lease 회신은 마지막으로 받은 `token`을 되돌려 보낸다.

```json
{"t": "teleop", "session_epoch": 41, "lease_id": "k3…", "control_gen": 9, "arm_gen": 5, "seq": 1532,
 "echo": {"state_seq": 88412, "token": "9f…"}, "frame_echo": {"frame_id": 3051},
 "mode": "direct", "deadman": true, "speed_ratio": 0.42, "curvature_ratio": -0.30, "lift_dir": 0}
```

| 필드 | 뜻 |
|---|---|
| `session_epoch`, `lease_id`, `control_gen`, `arm_gen` | 1절의 세대. 하나라도 현재와 다르면 버린다 |
| `seq` | 같은 `arm_gen` 안의 단조 증가 번호. 같거나 작은 `seq`는 버린다 |
| `echo` | 마지막으로 받은 `tick`의 번호와 토큰. 토큰이 맞지 않으면 버린다 |
| `frame_echo` | 영상 조종일 때 마지막으로 **표시한** 카메라 프레임(4.2). 육안 조종이면 생략 |
| `mode` | `direct`(육안) 또는 `video`(영상). 모드별 속도 상한이 다르다 |
| `deadman` | 누르고 있기가 눌린 상태. `false`면 0 명령 |
| `speed_ratio`, `curvature_ratio` | −1…1 비율(합의). 차량이 현재 상한(모드·잠정값·적재·허가)을 곱해 물리량으로 바꾼다. 곡률 비율에 곱하는 상한은 차량 설정의 수동 조종 곡률 상한이다(실측 모델 추종기 0.36 m⁻¹ 이하). 클라이언트는 물리 상한을 계산하지 않는다 |
| `lift_dir` | −1, 0, 1. 승강 속도는 차량 설정. 2026-10-19(상시 운용 콘솔 계획 D14): 실제로 구동한다 — 중재기가 자율과 같은 꼴(목표 위치 + 속도 상한 램프 + [0, 차체 모형의 승강 한계] 클립)의 목표를 낸다. 텔레옵 속도 0.02 m/s(자율 `lift_rate_mps` 이하). 허용된 명령이 이어지는 동안만 움직이고, 그 구간이 끝나는 첫 주기(데드맨·`lift_dir` 0·명령 기한·lease·STOP·preempt·PAUSED·정책 차단·TELEOP 이탈)에 목표를 측정 승강으로 한 번 고정한다. 속도형 변환은 실차 어댑터가 맡는다 |

lease 보유자는 `tick`을 받을 때마다 `{"t": "echo", "session_epoch", "lease_id", "echo": {"state_seq", "token"}}`를 보낸다(최신값만). 텔레옵 명령의 `echo`도 같은 역할을 한다.

### 4.1 만료 시각

차량은 명령을 받으면 **절대 만료 시각**을 정한다.

`deadline = sent_mono(echo) + command_max_age_s`

받은 시각에서 다시 세지 않는다. 그래서 회신이 늦게 돌아온 만큼 남은 유효시간이 줄어든다. 예를 들어 상한이 300 ms이고 290 ms 뒤에 회신이 왔다면 남은 시간은 10 ms다. 받은 시점에 이미 `deadline`이 지났으면 그 명령은 버린다.

차량은 제어 주기마다 현재 명령의 `deadline`을 검사한다. 지났으면 안전 선점 정지를 하고 무장을 푼다. 새 패킷이 하나도 오지 않아도 정지는 이 주기 검사로 일어난다.

- **창.** 차량은 토큰 송출 기록을 개수가 아니라 시간으로 보관한다. 최소 `max(command_max_age_s, lease_timeout_s)` 동안은 지우지 않는다. 그래서 유효한 회신이 창 밖으로 밀려 거부되는 일이 없다.
- **주기 예산.** `tick` 주기는 `command_max_age_s`의 1/4 이하로 둔다. 회신 나이에는 상태 대기(최대 한 주기), 왕복 지연, 클라이언트 입력 주기가 모두 들어가므로, 상한을 정할 때 이 셋을 합한 예산으로 정한다(설계 5.4).
- **보장 범위.** 이 규칙이 보장하는 것은 명령이 **최근에 차량이 보낸 tick을 받은 뒤에 만들어졌다**는 것까지다. 입력 자체가 방금 읽은 값인지는 보장하지 않는다. 5초 전에 읽어 둔 입력에 새 토큰을 붙이면 통과한다. 입력 신선도는 지원 클라이언트가 입력을 같은 주기 안에서 읽어 명령을 만든다는 구현 규칙(설계 5.2)과 입력 상태 보고에 기댄다. 토큰을 예측할 수 없게 만든 것은 회신의 인과(받은 뒤에만 만들 수 있음)를 지키기 위해서다.
- **만료 뒤.** 만료나 신선도 실패로 정지하면 무장이 풀린다. 다시 움직이려면 `arm`으로 새 `arm_gen`을 받아야 한다. 늦게 도착한 이전 세대의 명령은 무장을 되살리지 않는다.

### 4.2 영상 신선도

카메라 프레임마다 차량은 고유한 `frame_id`(촬영 단위, 재전송해도 같은 값)와 `acquired_mono`를 기록한다. `acquired_mono`는 차량이 카메라 드라이버나 렌더러에서 그 촬영을 받은 시각이다. 영상 조종 명령에서 차량은 `frame_echo.frame_id`의 나이를 다음과 같이 계산한다.

`video_age = now_mono − acquired_mono(frame_id) + camera_latency_bound_s`

이 값이 `video_max_age_s`를 넘으면 명령을 버린다. 받아들인 영상 조종 명령의 만료 시각은 두 시각 중 이른 쪽이다.

`deadline_video = min(sent_mono(echo) + command_max_age_s, acquired_mono(frame_id) + video_max_age_s − camera_latency_bound_s)`

차량은 이 시각을 4.1과 같이 제어 주기마다 검사한다. 그래서 새 패킷이 오지 않아도 영상이 상한을 넘는 순간 멈춘다.

- `camera_latency_bound_s`는 촬영부터 드라이버 수신까지 걸리는 지연의 보수적 상한이다. 측정해서 정하며, 그 전에는 잠정값이다. sensor·sim 시각을 차량 단조 시계로 바꾸지 않고 이 상한으로 덮는다.
- 같은 촬영을 다시 보내도 `frame_id`와 `acquired_mono`가 바뀌지 않는다. 그래서 오래된 JPEG를 새 번호로 재전송해 나이를 초기화할 수 없다.
- 카메라나 렌더가 멈추면 새 `frame_id`가 생기지 않는다. 그러면 나이가 계속 커져 영상 조종이 멈춘다. sim 시각이 멈춘 Isaac도 마찬가지다.
- **Isaac 실행기의 추가 검사(M-c, 2026-10-17).** 실행기는 물리 스텝마다 시작 단조 시각을 알고 있어, 프레임의 픽셀이 보여 주는 스텝(렌더 메타데이터 시각 − 가정한 픽셀 지연)의 시작 시각 `pixel_mono` 도 중재기에 넘긴다. 중재기는 위 식과 함께 `now_mono − pixel_mono ≤ video_max_age_s` 를 보고, 만료 시각도 `pixel_mono + video_max_age_s` 와의 이른 쪽으로 둔다. 그 스텝을 찾지 못하면 그 프레임으로는 영상 조종을 하지 않는다. 위 식보다 엄격한 쪽으로만 더하는 검사이고, `acquired_mono` 는 그대로 실제 수신 시각이다. 근거와 측정은 [M-c 증거](../validation/2026-10-17-m-c-evidence.md) 1.4절.
- `frame_echo`를 갱신하는 시점은 프레임을 그린 다음 표시 콜백이 실제로 호출됐을 때다(설계 5.3). 탭이 가려지면 콜백이 멈추므로 echo도 멈춘다. 그래도 이 판정은 지원 클라이언트의 표시 진행 보고를 **신뢰한다는 전제** 위에 있다. 브라우저 합성 이후 실제 화면 출력까지는 증명하지 못한다.

### 4.3 입력 상태

클라이언트는 탭 가림·포커스 잃음·게임패드 분리를 `input_state` 메시지로 알린다. 차량은 이를 받으면 무장을 푼다. 이 메시지가 유실되어도 텔레옵 명령이 멈추므로 4.1의 만료가 정지를 보장한다. `input_state`는 빠른 해제를 위한 보조 수단이다.

## 5. 모드·정지·작업 요청

| 메시지 | 권한 | 주요 필드 | 차량 처리 |
|---|---|---|---|
| `stop` | **연결된 모든 클라이언트** | `request_id`, `source: console \| gamepad \| key` | 안전 선점 정지. 항상 실행하고 수락한다(1절 처리 순서 0). 단, `control: false` 인 관찰 전용 연결은 예외로 `reject(unsupported)` 다(10절). 권한을 좁히는 방향이라 관찰 기기에도 허용한다. 인증이 없으므로 AP 안의 아무 기기나 STOP을 반복해 작업을 멈출 수 있다. 이것은 안전한 쪽의 고장으로 적는다(더 큰 한계는 설계 7.4) |
| `rearm` | lease | `request_id`, `lease_id`, `control_gen` | 정지 완료(설계 4.3의 1단계) 확인 뒤 `STOPPED → PAUSED` |
| `mode_request` | lease | `request_id`, `lease_id`, `control_gen`, `to: TELEOP \| PAUSED` | `STOPPED`에서는 모두 `reject(stopped_locked)`(나가는 길은 `rearm`뿐). 그 밖에서는 `to: PAUSED`를 어느 작업 단계에서나 받고, `to: TELEOP`만 상태 조건(설계 4.5)을 검사해 아니면 `reject(state_forbids_teleop)` |
| `arm` / `disarm` | lease | `request_id`, `lease_id`, `control_gen` | TELEOP에서만. `arm`은 새 `arm_gen`을 돌려준다 |
| `targets_request` | 관찰 이상 | `request_id` | 현재 작업 대상 목록(5.1)을 보낸다 |
| `mission_submit` | lease | `request_id`, `lease_id`, `control_gen`, `pallet_id`, `destination_id`, `targets_revision` | 대상 목록의 revision이 다르면 `reject(stale_targets)`. 수락하면 `mission_id`를 발급한다. 같은 `request_id`의 재전송은 1절의 처리 순서에 따라 저장된 결과(같은 `mission_id`)를 돌려받는다 |
| `mission_pause` / `mission_resume` / `mission_cancel` | lease | `request_id`, `lease_id`, `control_gen`, `mission_id` | 재개는 재개 절차(설계 4.3)를 시작하고, 각 단계 결과를 상태로 보낸다 |
| `recovery` | lease | `request_id`, `lease_id`, `control_gen`, `action: withdraw_straight` | 상태별로 허용된 복구 동작만 받는다. 나머지는 거부한다 |
| `session_end` | lease | `request_id`, `lease_id`, `control_gen` | 2026-10-19(상시 운용 콘솔 계획 D7·D13). 중재기 순서대로 피드백(`reject(no_feedback:…)`) → 확인된 서 있음(`reject(not_stopped)`, 면제 없음) → 진행 미션 없음(`reject(mission_active)`). 수락하면 그 틱에 AUTO·TELEOP 를 PAUSED 로 두고 새 `control_gen` 을 낸 뒤 실행기가 끝난다(감독 루프 종료 코드 0). **다시 열리지 않는다** |
| `sim_reset` | lease | 같음 | 감독 실행(`capabilities.sim_reset: true`)에서만, 아니면 `reject(unsupported)`. 조건은 `session_end` 와 같다. 수락하면 실행기가 끝나고(종료 코드 3) 감독 루프가 장면을 처음으로 되돌려 **새 `session_epoch`** 로 다시 연다 |

### 5.1 작업 대상 목록

```json
{"t": "targets", "targets_revision": 17, "map_frame": "map", "correction_version": 12,
 "pallets": [{"pallet_id": "P-01", "kind": "epal6", "pose": {"x_m": 12.3, "y_m": 5.2, "yaw_rad": 0.0}, "state": "available",
              "basis": "recognised", "uncertainty_m": 0.21}],
 "destinations": [{"destination_id": "D-A", "pose": {"x_m": 2.4, "y_m": 1.6, "yaw_rad": 3.14}}]}
```

- ID는 한 지도·세션 안에서 재사용하지 않는다. 대상의 자세나 상태가 바뀌면 `targets_revision`이 올라간다. 그래서 이전 목록을 보고 고른 ID가 다른 팔레트를 가리키는 일이 없다.
- 보정 버전이 바뀌면 자세를 다시 표현하고 revision을 올린다.
- 포크에 실린 팔레트(`state: carried`)는 `pose`를 `null`로 싣는다. 차체를 따라 매 주기 움직이는 자세를 대상 목록에 실으면 revision이 쉼 없이 오르기 때문이다. 내려놓으면 그 자리의 자세와 함께 revision을 올린다.
- **근거와 불확실성**(상시 콘솔 계획 D10, 2026-10-19). 팔레트마다 `basis`(`prior`·`recognised`·`delivered`)와 `uncertainty_m`(m, `prior` 는 `null`)을 싣는다. 등록한 자리는 `prior`, 관찰이 자리를 정하면 `recognised`, 내려놓으면 `delivered` 다. 보정 재표현은 근거를 바꾸지 않는다. 화면은 표시만 한다. 수동 조종 허가는 `recognised`·`delivered` 팔레트를 외곽 + `uncertainty_m` + 번들 여유로 키운 출입 금지 구역으로 쓰고, `prior` 팔레트는 표시 자리 대신 픽업 구역 전체(+ 차량 자세 오차)를 쓴다(시뮬레이션의 등록 자리는 장면 참값이라). 실차 중재기 적용은 메인 합의 사항이다.

**구현 상태(M-a4, 2026-10-16).** `forklift_core/mission/targets.py` 의 `TargetsBook` 이 위 규칙을 구현한다: 등록·자세·상태·보정 버전이 바뀌면 revision 이 오르고(같은 값이면 오르지 않음), `carried` 는 자세를 싣지 않으며 그동안의 자세 갱신은 무시하고, 내려놓으면 같은 ID 로 그 자리에 `delivered` 로 다시 싣는다. 보정 버전이 바뀌면 자세를 다시 표현하는 함수를 받아 모두 옮기고 revision 을 올린다(배치 실행기에서는 보정 버전이 SLAM release 횟수다 — 추적 중에는 보정이 스캔마다 바뀌어도 지도 자세가 뛰지 않고, release 때 코어가 추정치를 새 프레임으로 옮긴다. 목적지와 내려놓은 팔레트는 코어가 지금 쓰는 목적지 추정(도킹 보정, release 재표현)을 매 틱 따른다). `mission_submit` 은 revision 이 다르면 `reject(stale_targets)`, 없는 ID 나 이미 실린 팔레트는 `reject(invalid)`. 배치 실행기는 시나리오의 팔레트·목적지를 `P-1`·`D-1` 로 등록해 스스로 제출한다.

**거부 사유 코드(초안).** `no_lease`, `lease_reserved`, `stale_epoch`, `stale_lease`, `stale_control`, `stale_arm`, `stale_seq`, `bad_echo`, `expired`, `stale_video`, `not_armed`, `stopped_locked`, `state_forbids_teleop`, `stale_targets`, `stale`(요청 원장: 잊었거나 남의 토큰), `mission_active`·`session_ending`(세션 종료·장면 초기화), `scene_damaged`, `recovery_required`, `permission_denied:<허가 reason>`, `no_feedback:<항목>`, `unsupported`, `busy`, `invalid`. 2026-10-19(상시 운용 콘솔 계획 D3·D5a) 미션 요청: `stopping`, `cargo_held`(포크에 실림 — 단 상태의 `load.pallet_id` 인 그 팔레트는, 포크 위 명목 자리에서 가로·세로 0.02 m·방향 0.02 rad 안이고 화물 증거가 유효하며 격자 계획·SLAM 실행이면 **적재 제출**로 받는다: 대상 목록의 `carried` 팔레트를 그대로 내면 지금 자리에서 운반부터 시작하고, 승강이 운반 높이에 닿고 첫 현장 구간과 도킹이 준비될 때까지 서 있는다. 준비 실패는 결과의 `prepare:loaded:<단계>:<상태>`·`prepare:loaded:cargo_changed`·`prepare:loaded:start_wait:<남은 조건>`·`prepare:loaded:no_final_straight`, 상시 운용 콘솔 계획 D5 6c), `localization_lost`(SLAM 추적 전 — 화면 '위치 추정 준비 중'), `already_there`(팔레트가 검증된 그 목적지에 있음: 시작 때 D-0, 배달 판정을 통과한 목적지), `destination_occupied`(바닥의 다른 팔레트 외곽 — 사전값 0.30 m, 그 밖은 불확실성, 없으면 0.21 m 로 키움 — 이 목적지에 놓은 제출 팔레트와 겹침; 크기를 모르면 허용 형상 가운데 가장 큰 것), `held`, `not_running`, `not_paused`, `recovering`, `not_held`, `unknown_mission`, `near_tracking_active`. 화면의 한국어 문구는 `console/web/js/profile.js` 의 `REJECT`.

### 5.2 미션 결과 기록부 (2026-10-19, 상시 운용 콘솔 계획 D1·D13)

미션이 끝날 때마다 차량은 `missions` 메시지를 모든 연결에 보내고, 대상 목록처럼 저장해 늦게 붙은 연결에도 welcome·대상 목록 뒤에 보낸다.

```json
{"t": "missions", "missions_seq": 7, "older": 2,
 "results": [{"seq": 7, "mission_id": "M-0007", "outcome": "failed",
              "reason": "prepare:all_candidates_failed:invalid_start;invalid_start",
              "started_s": 1201.4, "ended_s": 1203.0, "ended_unix": 1792400123.5,
              "pallet_id": "P-1", "destination_id": "D-0",
              "advice": {"backoff_m": 0.15, "backoff_max_m": 0.6, "from_rear": [1.2, 3.4, 0.05]}}]}
```

- `missions_seq` 와 `seq` 는 **세션(`session_epoch`) 안에서만** 단조 증가한다. 새 실행(장면 초기화 포함)은 1부터 센다. 화면은 epoch 가 바뀌면 표를 비운다.
- `results` 는 최근 100건이고, 넘친 건수는 `older` 다. 각 미션의 전체 기록은 실행기 출력의 `missions/<mission_id>/result.json`·`records.json` 에 있다. 이 파일들은 실행기의 기록 쓰기 스레드가 미션 종료 뒤에 쓴다(2026-10-20, U29: 제어 틱은 디스크를 기다리지 않는다). 그래서 결과가 발행된 순간에는 아직 없을 수 있고, `record_error` 가 없다는 것이 저장 완료를 뜻하지는 않는다.
- `outcome`: `done`, `failed`, `cancelled`, `recovery_required`. `reason` 은 사유 코드(설명은 화면 쪽 표). `started_s`·`ended_s` 는 시뮬 시각, `ended_unix` 는 결과를 만든 순간의 벽시계(Unix) 시각이다(한 번 찍고 재전송·재생에서 바꾸지 않는다). `record_error` 는 기록 파일을 못 쓴 경우에만 있다. 실패를 안 다음 틱에 같은 `missions_seq` 로 다시 발행되며, 형식은 `result.json: …`·`records.json: …`(쓰기 실패), `result.json, records.json: dropped (…)`(쓰기 스레드가 가득 참), `files unsettled after N s`(세션 끝 대기 한도에 못 씀)다. `record_errors_older`(있을 때만)는 이미 목록 밖으로 밀려난 결과의 기록 실패 건수다(내용은 실행 로그에 있다).
- `advice`(준비의 모든 후보가 출발 자세에서 막힌 실패에만): 실패를 낸 준비 작업이 **같은 입력**(제출 때 고정한 격자·팔레트 대상·설정)과 계획기 자신의 출발 검사로, 그때 자세(`from_rear`)에서 곧게 뒤로 0.05 m 씩 1.0 m 까지 본 결과다. `backoff_m`–`backoff_max_m` 은 출발 검사가 이어서 통과하는 거리(그 사이 자세는 몸체가 지나가는 자리로 여유 0 에서 비어 있음). 찾지 못하면 `{"backoff_m": null, "within_m": 1.0, …}`, 가는 길이 막히면 `{"backoff_m": null, "blocked_at_m": d, …}`, 준비 시간을 다 쓰면 `{"skipped": "deadline"}`. **안내일 뿐 허가가 아니다** — 조종은 텔레옵 허가를 그대로 받는다. 상태의 자세는 지도 좌표라 이후 SLAM 보정이 움직이지 않은 차를 옮겨 보일 수 있으므로, 안내는 "실패 때 자세 기준" 으로만 쓴다.

## 6. 상태 메시지

`tick`은 회신용으로 짧고 자주 보낸다. `state`는 화면용으로 조금 더 드물게 보낸다. 둘 다 `state_seq`를 공유한다.

```json
{"t": "state", "state_seq": 88413, "session_epoch": 41,
 "mode": "TELEOP", "stop_kind": "none", "control_gen": 9, "armed": true, "arm_gen": 5,
 "lease": {"holder": "laptop-1", "observers": 1, "reserved_until_s": null},
 "pose": {"frame": "map", "x_m": 6.12, "y_m": 5.40, "yaw_rad": 0.08, "correction_version": 12, "source": "slam"},
 "rear_axle_speed_mps": 0.31, "steering_rad": [0.05, 0.04], "lift_m": 0.0,
 "load": {"state": null},
 "command": {"speed_mps": 0.21, "curvature_inv_m": -0.108},
 "applied": {"speed_mps": 0.30, "curvature_inv_m": -0.11, "steering_rad": [0.05, 0.04], "stop_kind": "none"},
 "permit": {"limit_mps": 0.5, "reason": "ok"},
 "limits": {"forward_mps": 0.5, "reverse_mps": 0.25, "curvature_inv_m": 0.36, "provisional": true},
 "drive_allowed": true, "lift_allowed": true,
 "mission": {"mission_id": "M-0007", "status": "held", "phase": "approach", "reason": "teleop",
             "phase_elapsed_s": 41.2, "resume_steps": null},
 "freshness": {"grid_free_age_s": 0.4, "slam_correction_age_s": 0.9, "near_obs_age_s": null},
 "pacing": {"realtime_factor": 0.99, "ok": true},
 "teleop_allowed": true, "teleop_forbidden_reason": null}
```

- 표시용 계획 경로는 `plan: {"revision": n, "poses": [[x_m, y_m, direction], …]}`로, 바뀔 때만 싣는다. 안전 판단에 쓰지 않는다.
- `mission.status`: `accepted`, `running`, `paused`, `held`(수동 조종 등으로 보류), `resuming`, `done`, `failed`, `cancelled`. 실패·보류에는 `reason`을 붙인다.
- `mission.resume_steps`: 재개 절차 진행 중일 때 단계별 `pending / ok / failed(reason)`. 단계 이름과 사유(확정, 2026-10-15 — [M-a 경계 설계 2부](../plans/2026-10-15-m-a05-boundary.md) 8–9절): `stop_confirmed`(`no_speed_feedback`, `not_stopped_timeout`) → `state_verified`(`pose_unbounded`, `load_unknown`, `load_changed`, `insertion_state_unknown`, `fork_in_pocket`, `lift_mismatch`, `hold_episode_changed`, `hold_state_mismatch`, `near_stopped:<근접 정지 사유>`) → `frame_reexpressed`(`reprojection_failed`) → `path_rechecked`(필요하면 재계획 포함; `path_blocked:<허가 reason>`, `replan_failed:<계획기 status>`, `reobserve_required`, `rejoin_failed`). 근접 관측이 소실됐지만 실패로 고정되기 전이면 `state_verified` 는 `pending` 으로 재획득을 기다린다.
- `mission.resume_result`: 재개 절차를 마치지 않고 다른 동작으로 넘어갈 때의 최종 결과 — `pickup_retry`(근접 불일치·재계획 거절이 기존 집기 재시도로 감), `recovery_only`(축을 따라 빼기만 허용), `no_resume`(재개 정책 표에 없는 조합, hold 중 재정렬, 재결합·재시도 불가). 어느 상태에서 어떤 결과가 나는지는 경계 설계 2부 8절의 표(R1–R15)가 정본이다.
- `phase_elapsed_s`는 일시정지 동안 멈추는 작업 시간이다. `freshness`의 값들은 멈추지 않는다.
- `mission.remaining_m`(2026-10-19 추가, 상시 운용 콘솔 계획 D4b): 지금 단계의 경로 추종기에 남은 거리(m). 추종기가 없는 단계(들기·내리기·대기 등)는 `null`. 표시와 시험(단계 끝 취소)용이며 안전 판단에 쓰지 않는다.
- **구현 상태(M-a3, 2026-10-16).** 코어의 `safety_state()`(배치 실행기에서는 명령 일정이 있을 때 `mission_safety.jsonl` 로 매 제어 틱)가 이 절의 미션 쪽 값을 낸다: `mission`(`id`·`status`·`generation`·`phase`·작업 시간 `phase_elapsed_s`·재개의 마지막 검사 `resume`·`stop_seen_s`·`not_stopped`), `load`·`fork_in_pocket`(근거 `phase`), 승강 목표, 근접 정지, 보정 버전·hold, 적용 명령, 허가 상한과 사유. 재개 절차의 거절은 `reason` 에 `no_resume:<사유>`·`state_verified:<사유>` 로 싣고 미션은 `paused` 로 돌아간다(새 `mission_resume` 이 필요). 중재기의 정지 종류(`controlled`·`preempt`)는 미션을 일시정지로 넘기고, 해제 뒤에도 `mission_resume` 으로만 재개한다.
- `command`는 텔레옵 비율에 현재 상한을 곱한 물리 명령이다. `applied`는 slew, 허가 상한, 안전 선점, Ackermann 바퀴 속도 한계, 조향 속도 한계까지 거친 값이다(배치 runner의 `applied_command` 지점과 같은 위치). 화면은 둘을 나란히 보여 준다.
- `drive_allowed`·`lift_allowed`는 중재기가 **제어 주기마다** 현재 상태로 따로 판정한 결과다(설계 4.5). TELEOP 진입 때 한 번만 보는 것이 아니다.
- 2026-10-19(상시 운용 콘솔 계획 D14) 추가: `lift_dirs`(허용된 승강 방향 배열, `lift_allowed` 는 그 배열이 비어 있지 않은지), `drive_mode`(`normal` / `reverse_straight` — 수동 복구의 곧은 후진: 곡률 0, 후진만, 0.10 m/s 이하, 측정 조향 ≈ 0 일 때만 속도), `lift_limit_m`(승강 한계), `recovery`(포크가 포켓에 남은 채 끝난 미션의 수동 복구 단계: `stage` `in_pocket`/`manual_clear`, `grounded`, `evidence`, `aligned`, `pallet_id`; 없으면 `null`). `teleop_allowed` 는 주행 또는 승강 중 하나라도 허용이면 참이다(내리기만 되는 상태에도 TELEOP 에 들어갈 수 있다). 수동 복구는 시뮬레이션에서만 열린다(빠져나옴 판정이 참값을 쓴다).
- 2026-10-19(메인 세션 합의, D14 보강) 추가: `state.stand_waiver` — `"lower_only"` 이면 서 있음 확인(`stopped_confirmed`)을 기다리는 요청 가운데 **TELEOP 진입과 rearm** 을 확인 없이 받는다. 조건은 정책이 내리기만 허용(주행 거절, `lift_dirs` = `[-1]`)하고, 지금 측정 속도 ≤ 0.10 m/s, 최근 1 s 순변위 ≤ 0.02 m(100 ms 넘는 피드백 공백 없이 1 s 전부 측정)인 것이다 — 포크가 데크를 누르는 동안 차가 흔들려 정지 감지기가 확인을 못 하는 경우다. 그 밖에는 `null`. 마지막으로 서 있음이 확인된 뒤 순이동이 0.05 m 를 넘으면(느린 미끄러짐) 면제되지 않는다. 순이동은 차량이 주행기록 위치(피드백 `odom_xy`)를 주면 확인 때 위치와 지금 위치의 거리다 — 피드백 공백 동안의 이동도 앞뒤 위치 차이로 들어간다. 위치를 주지 않는 차량은 측정 속도를 적분하고, 피드백 나이 한계(100 ms)를 넘는 공백이 나면 이동을 알 수 없으므로 다음 서 있음 확인까지 면제하지 않는다. 어느 쪽이든 공백이 든 1 s 동안은 면제하지 않는다(떨림인지 미끄러짐인지 모른다). 한계: 주행기록 위치는 바퀴 회전에서 나오므로, 바퀴가 돌지 않고 차체가 밀리는 미끄러짐(받친 포크가 팔레트와 엉켜 밀리는 경우)은 보지 못한다 — 누적 이동 판정이 그만큼 낙관적이다. 실차에서는 SLAM 보정이 들어간 지도 자세의 변위도 함께 보고 둘 중 큰 쪽을 쓰는 것이 후속 과제다(시뮬레이션에서는 지금 형태로 충분, 메인 세션 2026-10-19). 면제 중에는 PAUSED·STOPPED 의 `not_stopped` 고장이 나지 않지만 시간은 계속 세어, 면제가 끝나면 바로 판정한다. 선점 래치도 면제 상태에서 풀린다(이벤트 `latch_released` 에 `stand_waiver`). 확인 없이 들어오거나(이벤트 `stand_unconfirmed`, `reason` `lower_only`; rearm 은 `rearm_lower_only`) TELEOP 에서 래치가 면제로 풀리면(`latch_lower_only`), 정책이 주행을 열어도 정지 감지기가 확인한 틱에서만 주행이 열린다(그 전엔 정책 `reason` = `stand_unconfirmed`, 속도 0). 이 제약은 처음 실제 바퀴 속도가 나갈 때 끝난다 — 배속 차단·무장 해제 중에 잠깐 확인된 서 있음은 소모되지 않는다. 미션 제출·재개는 여전히 확인된 서 있음을 기다린다.
- 2026-10-19(상시 운용 콘솔 계획 D1·D6·D13) 추가: `last_result`(세션의 마지막 미션 결과 — 5.2 의 항목 하나, 없으면 `null`), `scene`(`{"damaged": null | {"reason", "t", "unix"}}` — 시뮬레이션의 참값 검사가 장면 손상을 처음 본 순간 고정된다; 그동안 자동 작업은 `reject(scene_damaged)`, 장면 초기화로만 풀린다), `feedback_ok`(중재기가 판정한 피드백 유효 — 세션 요청이 `no_feedback` 인지 `not_stopped` 인지 화면이 가려 보이기 위해). `mission.status` 에 `preparing`(제출 뒤 첫 구간 계획 중), `recovering`(취소·실패 뒤 스스로 물러나는 중), `recovery_required`(스스로 빼지 못해 사람이 복구해야 함 — 이때 상태의 `recovery` 가 단계를 준다)가 더해진다.
- `pacing`은 Isaac에서만 의미가 있다. 실시간 배속을 유지하지 못하면(`ok: false`) 수동 조종을 막는다. 기준은 M-c가 정한다.
- 잴 수 없는 값은 `null`로 둔다. 0이나 추정값으로 채우지 않는다.

## 7. 미디어 (미디어 연결의 바이너리 프레임)

카메라 머리글 예시:

```json
{"t": "camera", "frame_id": 3052, "acquired_mono_ns": 0, "capture": {"domain": "sensor", "stamp_ns": 0},
 "calibration_version": 3, "format": "jpeg", "width": 640, "height": 400}
```

- **카메라**: JPEG로 보낸다. 같은 구독 설정(해상도·품질)이면 한 번만 인코딩해 모든 클라이언트에 같은 바이트를 보낸다. 클라이언트마다 다시 인코딩하지 않는다.
- **검출 결과**: 영상에 굽지 않고 `{"t": "detections", "frame_id": …, "pockets": […]}`로 따로 보낸다. 화면이 그 `frame_id`의 프레임 위에 그린다. 표시용이다.
- **스캔**: 각도 범위·해상도·거리 배열(float32, m)을 보낸다. 측정 누락은 NaN이다.
- **격자**: 해상도, 원점, 프레임, 보정 버전, 상태 배열(uint8: UNKNOWN / FREE / OCCUPIED)을 보낸다.
- **구독**: 클라이언트는 `media_subscribe`로 종류별 상한(Hz, 해상도)을 정한다. 차량은 능력(3절)을 넘는 요청을 상한으로 깎고 그 값을 알려 준다. 관찰 클라이언트에는 별도의 더 낮은 상한을 둘 수 있다.

## 8. ROS 2 쪽으로 옮길 항목 (합의 뒤)

게이트웨이 뒤의 ROS 2 인터페이스는 메인 개발이 정한다. 콘솔이 필요로 하는 의미는 다음과 같다. 이름과 타입은 합의 때 정한다.

| 의미 | 형태 후보 |
|---|---|
| 작업 지시와 진행·결과 | action (목표: 팔레트·하역 위치·revision / 피드백: phase·재개 절차 / 결과: 성공·실패 사유) |
| 일시정지·재개·취소 | 서비스 또는 action 취소 |
| 텔레옵 명령 (세대·seq·회신·프레임 필드 포함) | 토픽, 최신값 QoS |
| 회신 토큰의 송출 기록 | 중재기가 직접 발급·보관한다(9절 1) |
| 중재 상태 (모드·정지 종류·무장·lease·허가) | 토픽, 신뢰성 QoS |
| 만료가 붙은 구동 명령과 적용 피드백 | 중재기 ↔ 구동 어댑터 사이. 콘솔은 적용 피드백만 읽는다 |
| 작업 대상 목록 | 토픽(latched) 또는 서비스, revision 포함 |

## 9. 합의 결과 (2026-10-13, 메인 개발 답)

1. **회신 토큰의 발급·검증 위치 — 동의(조건 둘).** 중재기(`forklift_core/safety/`)가 토큰을 발급·보관하고, 제어 주기마다 만료 시각을 판정한다. 조건은 두 가지다.
   - 중재기는 순수 모듈이라 시계와 난수원을 주입받는다. 노드가 단조 시각과 `os.urandom` 기반 토큰 생성기를 넘기고, 시험에서는 바꿔 끼운다.
   - 기록하는 송출 시각은 중재기가 tick을 만든 시각이다. 그래서 중재기 → 게이트웨이 → 소켓 구간의 지연도 나이에 들어간다(보수적).
   마지막 방어선은 MCU 자체 시계 만료다.
2. **TELEOP의 허가 입력 — 수정 합의.** 설계 4.4에 반영했다.
   - 원호를 앞 경로로 쓰는 방식과 적용 순서는 동의했다.
   - 설치 방식: 매 제어 주기 `update()`로 원호를 넣은 뒤 `allowed_speed`를 읽는다.
   - 방향은 실제 이동 방향, 길이·검사 속도는 남은 속도와 직전 적용 명령을 덮는 값으로 잡는다.
   - 보장 범위의 한계를 문서에 밝힌다.
   - 적재 footprint를 쓰고, `applied`를 정의한다.
3. **관찰 기기의 STOP — 동의.** 반복 STOP으로 작업을 멈출 수 있는 점은 안전한 쪽의 고장으로 적는다(5절).
4. **명령 형식 — 동의(비율).** 상태에 물리 명령(`command`)과 적용값(`applied`)을 함께 싣는다(6절).
5. **재개 절차 단계·사유 — 확정(2026-10-15).** 6절에 적은 이름과 사유이며, M-a 경계 설계 2부(재개 정책 표 R1–R15)에서 실제 분기와 맞춰 확정했다. 잠정안의 이름은 바꾸지 않고 사유와 `resume_result` 만 더했다.
6. **수치 예산 — 방법 동의, 측정 근거 수정.** 설계 5.4에 반영했다.
7. **바퀴 속도 피드백이 없는 차량 — 재무장·재개 금지에 동의, 대안은 보류.** 지금 구현을 실차에 쓰려면 검증된 바퀴·조향 피드백이 필요하다. 수단은 H1/H3에서 확인한다. 스캔 정합이나 영상 정지 감지 같은 다른 정지 확인 수단은 그 결과를 보고 검증 대상으로만 둔다.

**차량 쪽 작업 분담(합의).**
- M-a: 외부 명령 입력, 일시정지 중 phase 한도만 정지, 재개 단계별 결과, 작업 대상 목록과 `targets_revision`. 착수 시점은 사용자와 정한다.
- M-b: 세대 4종, 처리 순서, 결과 저장 상한, STOP 예외, `STOPPED` 탈출은 `rearm`뿐, 영상 조종 만료는 둘 중 이른 쪽.
- M-c: 실시간 배속 보고와 미달 시 텔레옵 차단, 촬영 단위 `frame_id`와 렌더 수신 단조 시각, 잡마다 새 `session_epoch`, 게이트웨이 노드·포트 파일, 판정 배치와 같은 GPU 동시 사용 금지, `camera_latency_bound_s` 측정.
- H3: 바퀴 속도·조향·승강 피드백, 적용 명령 피드백, MCU 자체 시계 만료.

## 10. 관찰 전용 프로파일 (C2a, 2026-10-15 메인 개발 동의)

중재기(M-b)가 없는 관찰 전용 게이트웨이(C2a, [설계](../design/2026-10-15-console-gateway-monitor.md))를 위한 규칙이다. 중재기가 생긴 뒤 `control: true` 연결은 지금까지의 규칙(STOP 항상 수락 등)으로 돌아간다.

- **우선순위.** 1절 처리 순서 0("STOP 은 세대와 상관없이 실행")과 2절 관찰 기기의 STOP 허용은 `control: true` 연결에 적용한다. `control: false` 연결에서는 이 절이 앞선다.
- **`capabilities.control: false`.** 차량은 `request_id` 가 있는 모든 요청(조종 권한·STOP·모드·무장·작업·복구·대상 목록)을 `reject(unsupported)` 로 답하고, `request_id` 가 없는 최신값 메시지(`teleop`·`echo`·`input_state`)는 답 없이 버린다. **이 연결의 STOP 은 차량을 멈추지 않는다.** `mission`·`teleop_direct`·`teleop_video`·`lift` 는 모두 `false` 다. `chassis` 는 화면 그림용 외곽(뒤 차축 기준)으로 실을 수 있다.
- **`capabilities.stop_instruction`(관찰 전용에서 필수).** STOP 대신 운용자가 할 일을 적은 문장이다. 콘솔은 STOP 자리에 눌러도 되는 '정지 불가' 버튼을 두고, 누르면 이 문장을 크게 보인다. 요청은 보내지 않는다. 실재하는 수단만 적는다(시뮬레이션은 실행기 정지, 실차는 그 차에 실제로 있는 차단 방법). 2026-10-15 기준 무선·물리 비상정지는 시험도 실기기도 없다. 실차의 정지 방법은 사용자가 **주 전원 스위치 끄기**로 정했다(2026-10-15).
- **판정 범위.** 관찰 전용 판정은 지금 열린 제어 연결의 welcome 에만 따른다. 재연결 중(새 welcome 전)에 누른 STOP 은 1절대로 보관하고 재전송한다.
- **상태.** `mode`·`stop_kind`·`armed`·`lease`·`permit`·`limits`·`command`·`applied`·`mission`·`drive_allowed`·`lift_allowed`·`teleop_allowed` 가 `null` 일 수 있고, 이때 `null` 은 '없음'이 아니라 '모름'이다. `pose` 는 `null` 이거나 `frame` 이 `map`·`odom` 이다. `pose.age_s` 는 상태를 보낼 때 그 자세가 이미 몇 초 묵었는지다. map 자세는 최신 map → odom 보정과 최신 odom → 차체 움직임을 합친 것이고, 나이는 움직임 쪽 변환의 것(stamp 의 나이와 변환이 멈춘 뒤 지난 시간 중 큰 쪽)이다. 보정의 나이는 `freshness.slam_correction_age_s` 로 따로 싣는다(보정 stamp 기준 — slam_toolbox 는 마지막 키프레임 시각을 찍으므로 차량이 서 있으면 늘어난다); 화면의 나이는 여기에 받은 뒤 지난 시간을 더한다. `freshness.scan_age_s` 를 더한다. `notes`(문자열 배열)는 운용자가 알아야 할 차량 쪽 사정이다.
- **미디어.** 격자 머리글은 `origin_m`(왼쪽 아래 모서리, 회전 없음)과 `values` 를 반드시 싣고, 보낼 수 없는 지도로 바뀌면 `{"t": "grid", "withdrawn": true, "reason": …}`(본문 없음)를 보내 화면이 가진 지도를 지우게 한다. 스캔 머리글의 `pose` 는 레이저 2D 자세, `frame` 은 그 기준, `pose_at` 은 `scan_stamp`(레이저의 odom 기준 움직임은 스캔 시각, map 보정은 최신값 — 보정의 나이는 `freshness.slam_correction_age_s`)·`latest`(스캔 시각의 움직임이 없어 최신값을 씀), `pose_delay_s` 는 `latest` 일 때의 지연, `age_s` 는 차량이 받은 뒤 지난 시간이다. 카메라 `frame_id` 는 촬영 단위(같은 stamp 의 재전송은 같은 번호), `capture.domain` 은 stamp 의 시계(`sensor`·`sim`)다. `media_subscribe` 마다 차량은 현재 프레임을 다시 보낸다. 콘솔은 welcome 으로 epoch 를 안 뒤 구독하고, 새 epoch 면 다시 구독하며 이전 세션의 지도·스캔·상태·대상 목록을 지운다.
- **조향·포크 높이·적재.** 지금 ROS 쪽 토픽이 없어 `null` 이다. 채우려면 메인이 `joint_states` 류를 내보낸다(필요할 때 요청).
