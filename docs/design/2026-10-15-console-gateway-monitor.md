# 콘솔 게이트웨이 C2a — 관찰 전용 (설계 초안)

2026-10-15 · 상위 설계: [운용 콘솔 설계](2026-10-13-operator-console.md) · 계약: [console/1 초안](../interfaces/2026-10-13-operator-console-contract-draft.md)

## 1. 목적과 범위

C2(게이트웨이 노드) 가운데 메인 개발(M-a 미션 실행부, M-b 중재기, M-c Isaac 실시간 실행기)을 기다리지 않는 부분만 먼저 만든다. 사용자 지시(2026-10-15): "메인과 별개인 부분부터 시작".

**하는 것.** ROS 2 토픽을 읽어 console/1 의 `welcome`·`state`·미디어(격자·스캔·카메라)로 바꿔 웹 콘솔에 보낸다. 콘솔은 지금 화면 그대로 실제 ROS 스택(실차·Isaac·rosbag 재생)의 위치·지도·LiDAR·영상을 본다.

**하지 않는 것.** 조종 권한, STOP, 모드 전환, 수동 조종, 작업 지시. 이 요청은 모두 `reject(unsupported)` 로 답한다. 차량을 움직이거나 멈추는 경로가 이 게이트웨이에는 없다. 그 경로는 중재기(M-b)가 생긴 뒤 C2b 에서 붙인다.

관찰 전용이라도 운용자가 콘솔의 STOP 이 동작한다고 믿으면 위험하다. 그래서 이 설계의 핵심은 **"이 연결로는 멈출 수 없다"를 계약과 화면에 드러내는 것**이다(4절, 6절).

## 2. 결정

| # | 결정 | 근거 |
|---|---|---|
| D1 | 관찰 전용 프로파일을 계약에 추가한다. `welcome.capabilities.control: false` 이면 차량은 `request_id` 가 있는 요청(`lease_request`·`stop`·`mode_request`·`arm`·`mission_*`·`recovery`·`targets_request`)을 모두 `reject(unsupported)` 로 답하고, `request_id` 가 없는 최신값 메시지(`teleop`·`echo`·`input_state`)는 답 없이 버린다. 콘솔은 권한·조종·작업 지정을 끄고 그 이유를 보이며, STOP 은 D12 의 '정지 불가' 버튼이 된다 | 계약 5절은 "STOP 은 항상 수락"이다. 멈출 수단이 없는 게이트웨이가 STOP 을 수락하면 거짓 안심을 준다. 거부하고 화면이 미리 알리는 쪽이 정직하다 |
| D2 | 코드 위치는 `console/gateway/`(ROS 를 import 하지 않는 변환·서버)와 `console/gateway/ros_node.py`(rclpy 어댑터, 컨테이너에서만 import) | 변환·전송을 호스트에서 ROS 없이 시험할 수 있다. 가짜 차량과 같은 `console` 패키지 안이라 웹 정적 파일·전송 규칙을 공유한다. 상위 설계 3절의 `ros2/src/forklift_console_gateway` 는 msg/action 이 생기는 C2b 에서 다시 본다(빈 scaffold 금지, CONTRIBUTING 1절) |
| D3 | 전송 공통부(JSON 검사·인코딩, 미디어 프레임, 정적 파일, 연결별 송신 큐)를 `console/mock_vehicle/server.py` 에서 `console/wire.py` 로 옮겨 두 서버가 함께 쓴다. 가짜 차량의 동작은 바꾸지 않는다 | C1 최종 검토에서 고친 전송 규칙(깊은 JSON 거부, 1e400 거부, 느린 독자 격리, 미디어 종류별 1건 전송)을 복제하면 한쪽만 고쳐지는 일이 생긴다. 기존 Python 시험 81개가 이 이동을 덮는다 |
| D4 | 위치는 TF `map → base_link` 를 상태 주기마다 **기다리지 않는 조회**(가장 최근 변환, 시간 제한 0)로 얻고, 콘솔 규약인 **뒤 차축 기준**으로 바꿔 싣는다(`rear_axle_x_m`, 기본값은 `dls08_measured` 의 −0.34 m, 매개변수로 받는다). `map` 이 없으면 `odom → base_link` 로 `frame: "odom"`, 둘 다 없으면 `pose: null`. 쓴 변환의 나이(`pose.age_s`)를 함께 싣는다. 나이는 변환 stamp 의 노드 시계 기준 나이와, 그 stamp 가 마지막으로 바뀐 뒤 지난 벽시계 시간 중 큰 쪽이다 — TF 버퍼는 발행이 멈춰도 마지막 변환을 계속 돌려주므로, 조회 시각을 나이로 쓰면 멈춘 위치가 늘 새것처럼 보인다(Codex P1). 버퍼는 노드 시계를 쓴다(`Buffer(node=…)`): `use_sim_time` 에서 bag 이 뒤로 가면 버퍼를 비운다 | 콘솔·가짜 차량은 뒤 차축 기준이다(`world.py`). base_link 그대로 실으면 차체 그림이 0.34 m 어긋난다. 프레임을 섞지 않도록 `frame` 을 꼭 싣는다. 시간 제한 있는 조회는 TF 가 끊겼을 때 ROS 스레드를 붙잡는다 |
| D5 | 속도는 연속한 두 `/odom` 자세의 stamp 차와 차체축 방향 이동으로 구한다(0.5 s 넘게 벌어지거나 시간이 안 가면 없음). twist 는 쓰지 않는다. 비유한 값은 `null`. 조향·포크 높이·적재는 토픽이 없어서 `null` | Isaac 브리지(`isaac_slam_bridge.py`)와 재생(`slam_replay.py`)은 자세만 채운 `Odometry` 를 낸다 — twist 를 그대로 쓰면 주행 중에도 0 이 신선한 측정값으로 나간다(Codex P2). 강체에서 차체 중심선 위 점들의 차체축 방향 속도는 같다 |
| D6 | 격자는 `OccupancyGrid` 를 map_server 와 같은 문턱(값/100 이 0.65 초과면 점유, 0.25 미만이면 빈칸, 그 사이와 −1 은 미확인)으로 0 미확인 · 1 빈칸 · 2 점유로 바꾸고 `origin_m` 을 싣는다. 원점 회전의 yaw 절댓값이 1 mrad 를 넘으면 그 격자는 보내지 않고, 화면이 이전 지도를 지우도록 `withdrawn: true` 격자 머리글을 보낸 뒤 상태 `notes` 에 사유를 싣는다(그 이하는 0 으로 본다) | slam_toolbox 의 `slam_recorder` 가 같은 문턱(0.65 · 0.25)을 쓴다. 웹은 회전 없는 격자만 그린다 |
| D7 | 스캔은 수신한 `LaserScan` 의 각 거리를 float32 로, `range_min` 미만·`range_max` 초과·inf·NaN 은 NaN 으로 바꾼다. 머리글의 `pose` 는 **스캔 stamp 의** `map → laser`(없으면 `odom → laser`) 2D 자세다(기다리지 않는 조회). 그 시각의 변환이 없으면 가장 최근 변환을 쓰고 `pose_at: "latest"`·`pose_delay_s` 로 표시한다; 화면은 그 점을 흐리게 그린다. 1 s 넘은 스캔 점은 그리지 않는다 | 웹의 `decodeScan` 은 머리글 자세로 점을 지도에 놓는다. 수신 때 자세를 쓰면 0.5 m/s·0.18 rad/s 에서 0.5 s 늦은 5 m 앞 점이 약 0.5 m 어긋난다(Codex P2, 처음 안의 D7 을 뒤집음). NaN 은 "반사 없음"이지 "비어 있음"이 아니다(계약 7절) |
| D8 | 카메라는 `sensor_msgs/CompressedImage` 의 JPEG 만 그대로 전달한다(다시 인코딩하지 않음). JPEG 판별은 `format` 문자열이 아니라 본문의 시작 바이트(`FF D8`)로 한다(`image_transport` 는 `"bgr8; jpeg compressed bgr8"` 같은 문자열을 쓴다). `frame_id` 는 촬영(stamp) 단위다 — 같은 stamp 로 다시 온 영상은 새 프레임이 아니다(계약 4.2). `capture.domain` 은 `use_sim_time` 이면 `sim`, 아니면 `sensor`. `camera_model` 은 C2a 에서 `null` | 오버레이(예상 궤적)는 수동 조종용이고 C2a 에는 수동 조종이 없다. Orin Nano 에 하드웨어 인코더가 없어(상위 설계 7.3) 원본 압축 영상을 넘기는 것이 가장 싸다 |
| D9 | `state.mode` 는 `null`(판정 주체 없음), `stop_kind`·`armed`·`lease`·`permit`·`limits`·`command`·`applied`·`mission` 도 `null`. `freshness.slam_correction_age_s` 는 `map → odom` 을 마지막으로 센 뒤 지난 시간, `pose.correction_version` 은 게이트웨이가 센 `map → odom` 변경 횟수(마지막으로 센 값에서 1 mm 또는 1 mrad 넘게 움직이면 하나 센다 — 느린 누적 변화도 결국 센다; slam_toolbox 는 같은 변환을 짧은 주기로 다시 발행한다). 표시용 잠정 값이며 SLAM 이 정상이라는 증거가 아니다. M-b·위치 추정 모듈이 정식 버전을 정하면 교체 | 모드·정지·허가는 중재기가 판정한다(상위 설계 3절 계층표). 게이트웨이가 지어내지 않는다 |
| D11 | 화면은 데이터가 끊긴 것을 '정지'로 보이게 두지 않는다. 브라우저가 마지막 `state` 를 받은 뒤 1 s 가 지나거나 `pose.age_s` 가 1 s 를 넘으면, 지도 위 차체를 흐리게 하고 "위치 n s 전 · 지금 위치가 아닐 수 있음" 띠를 띄운다 | 관찰 화면에서 가장 위험한 오해는 끊긴 화면의 차체를 '멈춘 차량'으로 읽는 것이다 |
| D12 | 관찰 전용이면 STOP 자리에 **눌러도 되는** "정지 불가" 버튼을 둔다. 누르거나 Esc·게임패드 정지를 누르면 화면 전체에 "이 화면으로는 멈출 수 없습니다"와 차량이 welcome 의 `stop_instruction` 으로 알린 실제 정지 방법을 크게 띄우고 이벤트에 남긴다. 요청은 보내지 않는다. Isaac 은 "시뮬레이션은 Isaac 실행기(잡)를 멈추세요"가 기본이고, 실차는 기본값이 없어 `--stop-instruction` 없이 시작하지 않는다(무선·물리 비상정지는 아직 시험도 실기기도 없다 — 메인 개발 답, 2026-10-15) | 반응 없는 비활성 버튼은 급한 순간에 운용자를 붙잡아 둔다. 눌렀을 때 곧바로 다음 행동을 알려 주는 쪽이 낫다 |
| D13 | 화면은 세션과 프레임을 섞지 않는다. 새 `session_epoch` 의 welcome 을 받으면 지도·스캔·상태·대상 목록을 지운다. 미디어 구독은 welcome 으로 epoch 를 안 뒤에 보내고, 구독할 때마다 서버는 현재 프레임을 다시 보낸다(미디어가 welcome 보다 먼저 와 버려진 정적 지도를 다시 받는다). 자세·스캔은 격자와 같은 프레임일 때만 지도에 그리고, 다르면 그 사실을 적는다 | 두 연결은 독립이라 순서가 보장되지 않는다(Codex P2 셋) |
| D14 | 관찰 전용에서 `null` 은 '없음'이 아니라 '모름'이다. 조종 기기·작업·목적지는 "모름 · 관찰 전용"으로, 통신 화면의 명령 유효기한·끊김 시 동작은 "해당 없음"으로 적는다 | 다른 경로로 차량이 작업 중이어도 "작업 없음"이라고 단정하면 안 된다(Codex P2) |
| D10 | 의존성: 컨테이너 이미지는 기존 `forklift/ros2-dev:jazzy` 를 바꾸지 않고, 그 위에 `websockets` 만 더한 별도 이미지(`deploy/console/Dockerfile`)를 쓴다 | 메인 개발의 SLAM 재생 배치가 쓰는 이미지를 건드리지 않는다 |

## 3. 구조

```
console/
  wire.py                 # 공통: JSON 검사·인코딩, 미디어 프레임, 정적 파일, 연결별 송신 큐 (D3)
  mock_vehicle/server.py  # wire.py 를 쓰도록 바뀜, 동작 같음
  gateway/
    __init__.py
    convert.py            # ROS 없는 순수 변환: 자세(뒤 차축), 격자, 스캔, 상태 조립, JPEG 판별
    store.py              # 최신값 저장소(스레드 안전): ROS 콜백이 쓰고 asyncio 가 읽는다
    server.py             # console/1 관찰 전용 서버(welcome, state, media, reject(unsupported))
    ros_node.py           # rclpy: 구독·TF 조회 → store (컨테이너 전용)
    __main__.py           # python -m console.gateway --port … --web … [ROS 매개변수]
deploy/console/Dockerfile # FROM forklift/ros2-dev:jazzy + websockets
```

제어와 미디어는 지금처럼 WebSocket 연결을 따로 쓴다(계약 1절). 큰 격자·영상이 상태 메시지를 붙잡지 않는다.

실행 흐름. `ros_node` 가 별도 스레드에서 rclpy executor 를 돌리며 콜백마다 `store` 에 최신값을 넣는다(큐에 쌓지 않는다). asyncio 쪽 `server` 는 정해진 주기로 `store` 의 스냅숏을 읽어 상태(5 Hz)·미디어(구독 상한)를 보낸다. TF 조회는 상태 주기마다 ROS 스레드에서 해서 결과만 `store` 에 넣는다.

## 4. 계약 추가 (console/1 초안에 넣을 문안)

**1절 처리 순서·2절 관찰자 STOP.** 처리 순서 0("STOP 은 세대와 상관없이 실행")과 관찰 기기의 STOP 허용은 `control` 이 `true` 인 연결에 적용한다. `control: false` 연결에서는 3절의 관찰 전용 규칙이 이 둘보다 앞선다.

**3절 `capabilities`.**
- `control`(bool, 없으면 `true`): `false` 면 이 연결은 **관찰 전용**이다. 차량은 `request_id` 가 있는 모든 요청(조종 권한·STOP·모드·무장·작업·복구·대상 목록)을 `reject(unsupported)` 로 답하고, `request_id` 가 없는 최신값 메시지(`teleop`·`echo`·`input_state`)는 답 없이 버린다. **이 연결의 STOP 은 차량을 멈추지 않는다.** 콘솔은 STOP 자리에 눌러도 되는 '정지 불가' 버튼을 두고, 누르면 `stop_instruction`(차량이 알린 실제 정지 방법, 관찰 전용에서 필수)을 크게 보인다. 요청은 보내지 않는다.
- 관찰 전용 판정은 **지금 열린 제어 연결의 welcome** 에만 따른다. 재연결 중(새 welcome 전)에 누른 STOP 은 지금처럼 보관하고 재전송한다.
- 관찰 전용이면 `mission`·`teleop_direct`·`teleop_video`·`lift` 는 모두 `false` 다. `chassis` 는 화면 그림용 외곽(뒤 차축 기준)으로 실을 수 있다.

**5절 STOP 행.** "항상 실행하고 수락한다" 뒤에 "(`control: false` 인 관찰 전용 연결은 예외 — `reject(unsupported)`, 3절)"을 붙인다.

**6절 상태.**
- 관찰 전용이면 `mode`·`stop_kind`·`armed`·`lease`·`permit`·`limits`·`command`·`applied`·`mission`·`drive_allowed`·`lift_allowed`·`teleop_allowed` 가 `null` 일 수 있고, 이때 `null` 은 '없음'이 아니라 '모름'이다.
- `pose` 는 `null` 이거나 `frame` 이 `map`·`odom` 중 하나다. `pose.age_s` 는 그 자세가 상태를 보낼 때 이미 몇 초 묵었는지다(변환 stamp 의 나이와 변환이 멈춘 뒤 지난 시간 중 큰 쪽). 화면에서의 나이는 `age_s` 에 상태를 받은 뒤 지난 시간을 더한 값이다.
- `freshness.scan_age_s` 를 더한다(마지막 스캔을 받은 뒤 지난 시간).
- `notes`(문자열 배열): 운용자가 알아야 할 차량 쪽 사정(예: 지도 원점이 회전되어 보내지 않음, JPEG 가 아닌 영상). 화면은 그대로 보인다.

**7절 미디어.**
- 격자 머리글에 `origin_m`(격자 왼쪽 아래 모서리, 회전 없음)과 `values`(코드표)를 반드시 싣는다. 보낼 수 없는 지도로 바뀌면 `{"t": "grid", "withdrawn": true, "reason": …}`(본문 없음)을 보내고, 화면은 가진 지도를 지운다.
- 스캔 머리글의 `pose` 는 레이저의 2D 자세이며 `frame` 에 기준 프레임을 적는다. `pose_at` 은 그 자세가 스캔 stamp 의 것(`scan_stamp`)인지 가장 최근 것(`latest`)인지, `pose_delay_s` 는 `latest` 일 때 스캔보다 몇 초 뒤의 변환인지다.
- 카메라 `frame_id` 는 촬영 단위다. 같은 촬영(같은 stamp)을 다시 받아도 새 번호를 주지 않는다. `capture.domain` 은 stamp 의 시계(`sensor`·`sim`)다.
- `media_subscribe` 를 받을 때마다 차량은 종류별 현재 프레임을 다시 보낸다. 콘솔은 welcome 으로 `session_epoch` 를 안 뒤에 구독하고, 새 epoch 의 welcome 을 받으면 다시 구독한다.

이 추가는 메인 개발이 동의했다(2026-10-15). 계약 초안 10절로 옮겼다. 정지 안내 문구는 메인 답(실차에 시험된 비상정지가 없음)에 따라 `stop_instruction` 으로 바꿨다.

## 5. ROS → console/1 대응

| console/1 | ROS 원천 | 변환 |
|---|---|---|
| `welcome.backend` | 매개변수 `backend` (`real`·`isaac`) | rosbag 재생 시험도 `isaac` 또는 `real` 중 기록 출처를 쓴다. `vehicle_id` 도 매개변수 |
| `state.pose` | TF `map → base_link` (없으면 `odom → base_link`) | 쿼터니언 → yaw, 뒤 차축으로 `rear_axle_x_m` 만큼 차체축 이동, `source: "slam"`(map) · `"odom"` |
| `state.rear_axle_speed_mps` | 연속한 두 `/odom` 자세 | 차체축 방향 이동 ÷ stamp 차(D5). 마지막 값이 1 s 넘게 지나면 `null` |
| `state.freshness` | TF·`/map`·`/scan` 수신 시각 | `slam_correction_age_s`(map→odom 변경 뒤), `grid_free_age_s`, `scan_age_s`(게이트웨이 단조 시계 기준), `near_obs_age_s` 는 `null` |
| 미디어 `grid` | `/map` (`nav_msgs/OccupancyGrid`, transient local) | D6 |
| 미디어 `scan` | `/scan` (`sensor_msgs/LaserScan`, sensor data QoS) | D7 |
| 미디어 `camera` | 매개변수 `camera_topic` (`sensor_msgs/CompressedImage`) | JPEG 만, 매번 새 `frame_id`, `capture.stamp_ns` 는 메시지 머리글 시각 |

## 6. 웹 변경

- `capabilities.control === false` 면: STOP 두 개를 "정지 불가" 버튼으로 바꾼다(D12). 하위 띠에 붉은 알림 "관찰 전용 연결 · 이 화면으로 차량을 멈출 수 없습니다 · (stop_instruction)"을 늘 보인다. 조종 권한·작업 지정·수동 조종 탭을 끄고 이유를 보인다.
- 데이터 끊김 표시(D11). 조향·포크 높이를 모르면 차체 그림 옆에 "조향·포크 높이 미측정"을 적는다. 범례에 "LiDAR 반사 없음은 빈 공간이 아님"을 적는다.
- `state` 의 `null` 필드를 모두 견딘다(지금 `render` 는 `permit`·`limits`·`command` 가 있다고 가정한다).
- 격자 `origin_m` 과 `values` 를 따른다(오늘 커밋에 반영).
- 연결 대상 표시는 `welcome.backend` 를 따른다(실차·Isaac 버튼은 계속 선택 기능이 아니라 표시다).

## 7. 시험

| 대상 | 시험 | 어디서 |
|---|---|---|
| `convert.py` | 자세 변환(뒤 차축 이동·yaw), 격자 문턱·원점·회전 거부, 스캔 NaN 처리, 상태 조립의 null, JPEG 판별 | 호스트 pytest |
| `server.py` | hello → welcome(`control:false`), 상태 주기, 미디어 구독 상한, 모든 제어 요청 → `reject(unsupported)`(STOP 포함), 잘못된 JSON, 느린 독자 격리(wire 공통 시험) | 호스트 pytest(가짜 store) |
| `wire.py` 이동 | 기존 가짜 차량 시험 81개 그대로 통과 | 호스트 pytest |
| 웹 | 관찰 전용 화면 규칙(순수 함수), null 상태 렌더, 원점 있는 격자 | node --test + 헤드리스 브라우저(가짜 store 서버) |
| `ros_node.py` | 합성 퍼블리셔(`/map`·`/scan`·`/tf`·`/odom`)를 띄우고 웹소켓 클라이언트가 받은 자세가 TF 합성 + 뒤 차축 이동과 같은지, 격자·스캔이 오는지 | 콘솔 이미지 컨테이너(ws1 Slurm CPU 잡) |

실제 slam_toolbox 출력 위에서 보는 것은 시험이 아니라 시연이다. 판정 배치가 없을 때 ws1 의 SLAM 재생(rosbag + slam_toolbox, CPU)에 게이트웨이를 붙여 화면을 확인한다.

## 8. 메인 개발에 알릴 것

- 계약 추가(4절)에 대한 동의 요청.
- 콘솔이 쓰는 토픽 이름(`/map`·`/scan`·`/odom`·TF `map`·`odom`·`base_link`·`laser`)과 뒤 차축 위치 −0.34 m. 이름이 바뀌면 매개변수로 맞춘다.
- `forklift_core`·`sim/isaac` 는 건드리지 않는다. `deploy/ros2` 이미지도 건드리지 않는다.

## 9. 자체 검토

**1. 원자료 재확인(Claude).** 뒤 차축 x −0.34 m(`sim/models/dls08_measured/parameters.yaml` `rear_axle_x_m`, README ▽ 추정 고정값), 조향축 앞(`steering_axle: front`), 격자 문턱 0.65·0.25(`slam_recorder.py` 156행), 프레임 이름 `odom`·`base_link`·`laser`(`slam_replay.py`), 가짜 차량 격자 코드 0·1·2 와 `origin_m`(`console/mock_vehicle/server.py`), 웹이 격자 값 0 을 빈칸으로 칠하고 원점을 무시하던 결함(오늘 커밋 뒤 고침). 문턱 문구를 '이상·이하'에서 map_server 의 '초과·미만'으로 고쳤다.

**3. 다른 관점 독립 검토(agy, Gemini 3.8 Flash High, 1·2 결론 비공개; 첫 시도 Claude Opus 4.6 은 본문 없이 끝남).** 받아들인 것: TF 를 기다리는 조회는 ROS 스레드를 붙잡는다 → 시간 제한 0(D4); 스캔 시각 조회는 외삽 실패로 점이 깜빡인다 → 최근 변환(D7); 격자 원점 yaw 의 부동소수 잡음 → 1 mrad 허용(D6); `map → odom` 같은 값 재발행·잡음 → 1 mm·1 mrad 문턱(D9); `CompressedImage.format` 문자열 → 시작 바이트 판별(D8); 끊긴 화면이 정지로 보임 → D11; 비활성 STOP 이 급한 순간 운용자를 붙잡음 → D12; 조향·포크 미측정과 NaN 표시 → 6절 문구. 받아들이지 않은 것: (a) "한 WebSocket 에 상태와 미디어가 섞여 지연" — 이미 연결을 둘로 나눴다(3절에 명시). (b) "회전 중 base_link 의 `linear.x` 는 뒤 차축 속도와 다르다" — 평면 강체에서 차체축 위 두 점의 속도 차이는 ω × r 이고 r 이 차체축 방향이면 그 차이는 차체축에 수직이다. 차체축 방향 성분은 같다. 조향축도 앞이다.

**2. Codex 적대적 검토(구현 f1df17d 기준, 1차).** 미수렴 — P1 1, P2 10, P3 1. 처리: (P1) 멈춘 TF 가 나이 0 으로 보임 → D4 나이 정의. (P2) `/odom` twist 가 비어 있음 → D5 자세 미분; TF 버퍼가 노드 시계를 안 씀 → `Buffer(node=…)`; 수신 때 자세로 스캔을 놓음 → D7 을 뒤집어 스캔 stamp 조회 + `latest` 표시; 프레임 섞임 → D13; 거부된 지도·끊긴 스캔이 남음과 freshness 키 이름 불일치 → `withdrawn` 격자, `notes` 표시, 1 s 넘은 스캔 숨김, 계약 이름 `grid_free_age_s`; welcome 보다 먼저 온 지도 유실 → 구독 때 재전송·welcome 뒤 구독; 새 epoch 에 이전 화면이 남음 → D13(protocol 의 세션 교체 때 상태·대상 목록도 지움); 같은 촬영 재발행이 새 프레임 → stamp 단위 `frame_id`; 비유한 속도가 상태 전체를 막음 → `null`; 관찰 전용 화면의 단정 문구 → D14. (P3) 보정 문턱 설명 → D9 문구. 계약 추가 문안(4절)은 지적대로 1·2절 우선순위, 무응답 메시지, `age_s`·`notes`·`scan_age_s`·`withdrawn`·`pose_at`·촬영 단위 ID·재구독 규칙을 채웠다. 동의받은 것: STOP 을 거짓 수락하지 않는 원칙, `console/gateway` 배치, 변환 부호·yaw·격자 문턱·NaN 처리·저장소 잠금.

**Codex 2차(fc53b87).** P2 4: 송신이 막혀 버린 지도 철회를 다시 보내지 않음 → 전송이 시작된 구독자만 보낸 것으로 기록(`_send_media` 가 성공 여부를 돌려줌); 재구독 때 다시 보낸 스캔이 신선해 보임 → 스캔 머리글 `age_s`; 새 세션에서 같은 대상 revision 이면 이전 목록이 남음 → 세션이 바뀌면 revision 캐시·목록 초기화; ±π 를 넘을 때 속도 부호가 뒤집힘 → 짧은 쪽 중간 heading. 같은 회차에서 UI 3차 지적(늦은 무장 응답)의 수정도 확인받았다.

**Codex 3차(0303d12).** 새 P1/P2·P3 없음 — 수렴. 남은 검증 경계: 실제 slam_toolbox 출력·Isaac 브리지·실차 위에서는 아직 돌리지 않았다(ROS 노드 시험은 합성 퍼블리셔).
