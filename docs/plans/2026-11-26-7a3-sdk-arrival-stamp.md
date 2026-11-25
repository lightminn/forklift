# 7a-3 — SDK 표본 시각을 바이트 도착 시각에서 (S10 전 필수)

작성일: 2026-11-26 · 상태: **계획 6판 승인(총괄 2026-11-26), 상한 근거는 B 로 결정(9절); 7a-3a 구현 중** · 관련: [PR 7 설계](../design/2026-11-25-real-sources-pr7.md) 2절 표의 7a-3 행·4절(7a-2 구현), `deploy/ros2/sllidar_patches/0001-sdk-first-sample-stamp.patch`, `config/real/lidar.yaml`

총괄 승인(2026-11-26): 7a-3 은 벤치 없이 되는 패치·패치 이미지·단위 시험까지 한다. 오차 상한 측정과 `first_beam_err_s` 기입은 벤치 항목으로 절차서에 넣는다(기입 형식은 9절의 총괄 결정: `timing.basis: engineering`, `timing.measured` 키는 쓰지 않음). 이미지 태그를 한 곳에 모으는 안을 넣는다.

## 1. 지금 SDK 가 시각을 읽는 곳 (고정 커밋 34300099 에서 확인)

- `sdk/src/sl_async_transceiver.cpp`
  - `_proc_rxThread`: 채널에서 바이트를 읽어 `Buffer{size, data}` 로 `_rxQueue` 에 넣는다. 시각은 남기지 않는다.
  - `_proc_decoderThread`: 큐에서 꺼내 `_codec.onDecodeData(data, size)` 를 부른다.
- `sdk/src/dataunpacker/dataunpacker.cpp` `getCurrentTimestamp_uS()` 는 `getus()` 다. 처리기(`handler_capsules.cpp` 의 dense·ultra·express capsule, `handler_hqnode.cpp`, `handler_normalnode.cpp`)가 **해석하는 순간** 이것을 읽고, 모드별 표본 지연(`_getSampleDelayOffset…`)을 빼서 각 표본 시각으로 `publishHQNode` 한다.
- 그래서 표본 시각에는 수신 큐에서 기다린 시간과 디코더 스레드가 늦게 깬 시간이 들어간다(Codex 7a-2 2 회차 P2-2). 이 값은 위로만 치우치고 상한이 없다.

## 2. 패치 0002 (SDK; 노드는 형식 이름과 발행 조건만)

- **도착 시각**: `Buffer` 에 `rx_us`·`rx_seq` 를 더한다. 수신 스레드가 `read()` 가 돌아온 직후 `getus()` 로 찍고 읽기 순번을 붙인다. 디코더는 실제 배선 그대로 넘긴다: `_codec.onDecodeData` 앞에 도착 시각을 맡기는 호출(`IAsyncProtocolCodec::onDecodeDataAt(data, size, rx_us, rx_seq)`, 기본은 지금 동작)을 두고, 드라이버의 `RPLidarProtocolCodec` → 드라이버 콜백(`sl_lidar_driver.cpp` 1655 부근) → 해석기까지 그 값을 들고 간다. 해석기의 `getCurrentTimestamp_uS()` 는 맡긴 값을 낸다.
- **도착 시각이 없는 표본은 v2 가 아니다**: 맡긴 값 없이 `getus()` 로 돌아간 표본이 하나라도 든 SDK 버퍼는 표식(`arrival_ok = false`)을 달고, 노드는 그런 스캔을 `scan_sdk` 에 내지 않는다(세어서 로그로).
- **capsule 기준을 자기 capsule 로**(Codex 계획 1 회차 P1): Express·UltraBoost 는 이전 capsule 의 표본을 이전 capsule 의 캐시된 시각으로 찍는데(`handler_capsules.cpp` 256), Dense·UltraDense 는 **다음 capsule 을 완성한 읽기의 시각**으로 이전 capsule 의 표본을 찍는다(739, 1033). 0002 에서 Dense·UltraDense 도 이전 capsule 을 완성한 읽기의 도착 시각을 캐시해 그것으로 찍는다. 그래서 모든 모드에서 표본의 기준은 "그 표본이 든 capsule 의 마지막 바이트가 든 읽기의 도착 시각"이다. 모드별 표본 지연(`_getSampleDelayOffset…`)을 빼는 식은 그대로다.
- **Dense·UltraDense 캐시의 교체와 폐기**(Codex 2 회차 P2-4): capsule 데이터와 그 도착 시각·읽기 순번·유효 여부를 한 묶음으로 바꾼다. 각도 점프로 바로 반환하는 분기에서도 함께 바꾸고, `reset()` 에서는 묶음 전체와 준비 비트를 내린다. 그래서 새 데이터에 옛 capsule 의 시각이 붙지 않는다.
- **실제 모드 정보 발행**(P2-2, 3 회차 P2-2·3): 런처가 실행마다 **실행 식별자**(무작위)를 발급해 노드(파라미터 `forklift_run_id`)와 어댑터(인자)에 같은 값을 넘긴다(4 회차 P2-1: 같은 토픽의 다른 v2 노드를 걸러냄). 노드는 스캔을 시작할 때마다(처음과 `start_motor` 서비스의 `startScan` 재시작 모두) 새 **원천 식별자**(시작마다 바뀌는 무작위 값)를 정하고, SDK 가 실제로 고른 응답 타입·표본 주기·보드레이트를 실행·원천 식별자와 함께 `scan_sdk_info`(latched)로 낸다. 재시작하면 옛 원천 식별자는 무효다. 각 `scan_sdk` 메시지의 `frame_id` 에도 두 식별자를 붙인다(`<frame>@sdk_mono_v2#<run>.<start>`). 재시작으로 모드가 바뀌는 경우를 시험에 넣는다.
  - **3a 에서 정한 형식**(3b 의 노드가 맞춘다): `scan_sdk_info` 는 `std_msgs/String`(JSON) — `{"format": "sdk_mono_v2", "run", "start", "ans_type", "sample_period_us", "baudrate"}`, 토픽 `/sensors/lidar/scan_sdk_info`(재매핑), QoS reliable·transient_local·depth 1. 실행·원천 식별자는 소문자 16진 8–32 자. 노드 파라미터 `forklift_run_id` 는 런처가 넘긴다(v1 노드는 선언하지 않아 무시). 어댑터는 v2 프레임에 `source: {run, start, image_id, usb, host[, sdk_info]}` 를 싣고, `sdk_info` 는 그 시작의 정보를 받았을 때만 싣는다.
- **진단 기록**(벤치용, 기본 끔): 환경변수 `FORKLIFT_SDK_RX_LOG=<경로>` 가 있을 때만 줄 단위로 쓴다. 읽기마다 `(rx_seq, rx_us, bytes)`, capsule 완성마다 `(rx_seq, capsule 순번, 응답 타입)` 을 고리 버퍼로 모아 쓰고, 넘친 줄 수를 센다.
- 노드는 형식 이름만 `@sdk_mono_v2` 로 올리고 위 `arrival_ok` 를 지킨다. 0001 의 첫·끝 표본·`publish_delay_ms` 는 그대로다.
- **3b 구현에서 정한 것**(Codex 3b 리뷰 1–2 회차):
  - codec 은 고정 길이로 블록을 자르므로 바이트 하나가 빠지면 이후 capsule 이 블록 안에서 끝난다. 그래서 "블록을 완성한 읽기"가 아니라 메시지마다 바이트가 든 읽기 구간(`ArrivalSegment`)을 함께 넘기고, 해석기는 구간별로 그 읽기의 도착 시각을 맡긴다. 단위의 마지막 바이트가 든 읽기의 시각이 그 단위의 기준이 되어 재동기화 뒤에도 정확하다. 도착 시각이 없는 바이트가 든 메시지의 표본은 exact 가 아니고, 그런 표본이 든 스캔(버퍼 단위)은 `scan_sdk` 에 나가지 않는다.
  - codec 은 잠금 안에서 메시지를 복사해 넘기고, 해석기의 해석과 enable·disable·reset 은 잠금 하나를 쓴다(스캔 재시작 경합). 노드는 시작 뒤 150 ms 안의 표본이 든 스캔을 내지 않는다.
  - 업스트림 버그 둘을 고쳤다: UltraDense 표본 지연의 `(31 - sampleIdx)`(표본 64 개라 뒤 절반이 unsigned 로 감김 → `63 - sampleIdx`), transceiver 종료 때 큐의 `new Buffer` 를 `delete[]` 로 지움.
  - 수신 기록 줄: `R rx_seq rx_us bytes decode_us`, `C rx_seq unit_seq ans_type exact`, `D <넘친 줄>`, 마지막 `E <넘친 줄>`.
  - 기준 이미지: `deploy/ros2/scan_image_base.txt` 의 아키텍처별 참조와 이미지 ID 가 태그 입력에 들어가고, 빌드는 그 ID 로 만든 태그 위에서 한다.

## 3. 호스트 쪽 — 한 판정을 모든 소비자가

- 어댑터(`real_scan_adapter.py`)는 v1·v2 를 받고 `stamp_kind` 를 `sdk_first_sample`(v1)·`sdk_first_sample_rx`(v2)로 싣는다. `ScanFeed` 는 두 이름을 받는다.
- **프레임별 판정 하나**: `forklift_core.sensors.lidar_cfg.frame_bound` 가 "이 프레임에 상한을 쓸 수 있는가"를 정하고 층(`to_layer_scan`)과 SLAM(`slam_real_scan.from_feed`)이 **둘 다** 이것을 쓴다(Codex P2-1). 조건: `lidar.yaml` 의 `timing.basis` 가 `measured` 이거나, `engineering` 이면서 `decision_id`·`basis_doc` 가 있음(총괄 결정: 공학 추정을 measured 로 쓰지 않는다); 프레임 `stamp_kind` 가 v2(`sdk_first_sample_rx`); **원천 세대의 실측 사실**이 적용 조건(`timing.applies_to`: image_id, usb, host, ans_type, sample_period_us, baudrate)과 같음. 하나라도 아니거나 값이 없으면 그 프레임은 진단 전용이고 그 사유(`no_time_bound`·`not_arrival_stamped`·`no_source_facts`·`source_mismatch`)가 남는다. 옛 `timing.measured` 키는 읽지 않고 거부한다.
- **적용 조건은 실제로 돈 원천에 묶는다**(Codex 2 회차 P2-2): 런처가 실행한 이미지 ID(`docker inspect`), 노드의 `scan_sdk_info`(응답 타입·표본 주기·보드레이트), USB 어댑터 식별(udev 의 vendor·product·serial), 호스트 이름을 원천 사실로 모은다. 판정은 이 값들을 `applies_to` 와 비교한다(태그 이름이 아니라 이미지 ID).
- **프레임마다 원천을 싣는다**(3 회차 P2-3): 어댑터는 실행 식별자가 자기 것이 아닌 스캔을 버리고 센다. 같은 실행 안에서 스캔의 원천 식별자와 같은 식별자의 `scan_sdk_info` 를 받았을 때만 그 정보를 그 프레임의 `scan/1` 머리에 싣는다(정보가 없거나 식별자가 다르면 정보 없음 → 진단). 그래서 데이터그램 수신자가 늦게 붙거나 첫 프레임을 놓쳐도 프레임 하나로 판정한다.
- **폐기한 원천에는 새 세대가 없다**(4 회차 P2-2): 원천 식별자가 바뀌면 어댑터의 생산자 세대가 오르고 옛 식별자는 폐기 목록에 들어간다. 폐기한 식별자의 스캔은 늦게 와도 버리고 센다(새 세대를 내주지 않음). 시험: `A → B → 늦은 A`.
- **세대 변경은 자격 판정보다 먼저**(3 회차 P2-3): `ScanFeed` 가 새 세대를 받으면, 그 프레임이 자격 미달이어도 SLAM 원천에 세대 변경을 먼저 알려 대기 스캔과 전송 중 응답을 무효화한 뒤 자격을 판정한다.
- **운영 SLAM 입력은 자격 있는 프레임만**(Codex 2 회차 P2-3): `RealSlamSource` 는 생성 때 `diagnostic` 또는 `operating` 이다. `operating` 은 판정을 통과한 프레임만 브리지에 보내고 시작 검사 계수에도 그것만 센다. `diagnostic` 은 자격 있는 프레임을 받아도 증거 응답을 내지 않는다(기록만). 진단 세션에서 운영으로 바꾸려면 새 세션(새 인스턴스)으로 시작한다.
- **세대가 바뀌면 진행 중인 시작·재위치 검사도 무효**(4 회차 P2-3, 7c-1 부터 있던 틈): 지금 `RealSlamSource` 는 세대 변경 때 대기 스캔과 전송 중 응답은 버리지만, 옛 인스턴스의 별도 상태 메시지(`ready` 등)는 받아 `can_confirm()` 이 참이 될 수 있다(Codex 가 재현). 세대가 바뀌면 진행 중인 시작 검사와 seed 의 끝 상태 대기를 무효로 하고, 새 세대 스캔으로 시작한 검사의 끝 상태만 확인을 연다. (3a 구현: 최초 시작 검사에 옛 세대 스캔이 하나도 들어가지 않았으면 그 검사는 새 세대 스캔으로 이어 간다 — 무효로 할 옛 증거가 없다.) 시험: 시작 스캔 7 개 → 세대 변경 → 옛 `processed`·`ready` 수신 → `can_confirm()` 거짓.
  - **브리지 세션 정리**(5 회차 P2): 브리지 프로토콜은 `Cancel` 을 seed 에만 받고 같은 연결의 새 `Hello` 도 고장으로 본다(연결은 하나뿐). 그래서 **최초 시작 검사(`Hello` 로 연 것) 중에** 세대가 바뀌면 원천은 그 SLAM 세션을 끝낸다(원천 고장으로 끝내고 기록에 사유 `start_check_generation_changed`). 운영 실행기는 지금의 원천 고장 경로대로 멈추고, 운용자는 새 브리지 세션(런처가 컨테이너를 새로 띄움)에서 다시 시작한다. 관측 실행(`real_observe --slam`)은 지금처럼 SLAM 경로만 끝내고 보드 관측과 종료 `DISARM` 은 이어 간다. 진행 중인 **seed** 중이면 기존 `Cancel → void` 를 쓴다. 시작 검사가 끝난(`ready`) 뒤의 세대 변경은 지금처럼 대기 스캔·응답만 무효로 한다.
  - 회귀 시험: 옛 `ready` 가 확인을 열지 않음, 최초 시작 검사 중 세대 변경은 세션 끝, seed 중 세대 변경은 `Cancel → void` 뒤 새 세대 스캔으로 끝 상태가 나와야 확인이 다시 열림.
- **HIL 은 지금 경로를 명시적으로 고른다**(3 회차 P2-1): 상한이 기입되기 전에 `diagnostic` 인 것은 **실 LiDAR 입력** 세션뿐이다. HIL 조립(취득 시각이 정확한 합성 스캔을 `measured=True` 로 넘김)은 `operating` 을 명시해 지금처럼 시작 확인과 임무를 한다. 3a 의 회귀는 기존 HIL 시작 확인·승강 통합 시험이다.
- 관측 실행 헤더의 `diagnostic` 은 설정만 보지 않고, 적용 조건을 적은 뒤 실제 판정은 프레임별 계수(`frames_measured`/`frames_diagnostic`)로 남긴다(Codex P3).
- 텔레옵 다리는 지금처럼 SDK 형식 둘 다 거른다. v1·v2 프레임이 주기 이력에 들어가기 전에 거부되는 시험을 둔다.
- 한 세션은 한 이미지라 v1·v2 가 섞일 일은 없다. 그래도 섞이면 v1 은 판정 함수가 진단으로 돌린다.

## 4. 오차 상한 — 벤치 항목 (측정 방법은 제안, 수치 기입은 총괄 판단)

Codex 1 회차 P1: `latency_timer`·덩어리 크기·회전 간격 흔들림만으로는 양방향 절대 오차를 묶지 못한다(도착 − capsule 완료의 적체, SDK 지연 모델의 잔차는 부호가 양쪽, 일정한 편향은 회전 간격에서 사라짐). 그래서 두 가지를 따로 잰다. 손작업 합 약 15 분.

- **상한을 기입할 근거는 아직 없다**(Codex 2 회차 P1): 아래 (가)·(나)는 **진단 측정**이다. 벽·자이로 시간 맞춤의 이동량에는 IMU 자신의 지연(`imu_record` 의 호스트 시각은 콜백 시각이고 IMU 지연 상한도 비어 있음)과 벽을 본 빔의 스캔 내 위상이 섞이고, 직선 맞춤 잔차에는 순번에 따라 선형으로 늘어나는 적체가 숨는다. 그래서 이 둘로 첫·끝 빔의 절대 상한을 기입하지 않는다. 기입하려면 빔 취득 순서·시각을 보존한 원자료와, 지연이 따로 제한된 독립 기준이 필요하다. 그 방법을 정하는 것은 총괄 결정으로 올린다(9절).
- **(가) 흔들리는 몫**(진단): 진단 기록에서 capsule 완성 시각(기준 도착 시각)을 capsule 순번에 대해 직선으로 맞춘 잔차의 범위(p0.5–p99.5)와, 넘친 줄 수 0, `latency_timer`(sysfs) 값. 이 잔차는 USB·TTY 적체와 수신 스레드 지연의 흔들림이다.
- **(나) 일정한 편향**(진단, 상대 지연 추정): 제자리에서 손으로 차를 천천히 돌리며(벽 하나가 보이는 곳, 약 2 분) 스캔에서 뽑은 벽 방향과 IMU 자이로 적분 방향을 비교해, 둘을 가장 잘 맞추는 시간 이동을 구한다(시간 보정). IMU 시각은 같은 호스트 단조 시계다.
- 각속도를 바꿔 가며(정지·좌·우) 같은 평탄한 벽의 긴 구간을 계속 본다. 일정한 각속도만으로는 시간 이동이 yaw 오프셋과 구분되지 않는다.
- 예산 참고: 첫·끝 빔에 같은 상한이 붙고 층은 매 스캔의 추정 폭에 양 끝 오차를 더하므로, 폭 98 ms 스캔이면 한쪽 11 ms 가 `scan_period_s` 0.12 의 한계다.
- 명령은 지금 런처로 실제로 도는 것을 절차서에 적는다(Codex P2-4): `ros2_dev.sh scan-a2m12` 에 `FORKLIFT_SCAN_SOURCE=sdk`, 진단 기록 경로(쓰기 가능한 마운트로 넘김), `publish_delay_ms` 를 넘기는 인자를 이 PR 에서 더한다.
- 7a-2 의 벤치 시간표 1(지연 주입 300 ms 로 나이가 그만큼 느는지)은 v2 이미지로 한 번 더 한다. 이것은 취득 시각이 보존되는지를 보는 시험이지 상한 측정이 아니다.

## 5. 이미지 태그를 한 곳으로

지금 태그가 박힌 곳은 셋이고(`tools/ros2_dev.sh` `:jazzy`, `tools/hwday/jetson_prep.sh`·`jetson_a2m12.sh` `:jazzy-arm64`, `deploy/slurm/scan_image_build.sbatch` `jazzy-p0001-<해시>`), `lidar.yaml` 에는 없다. 예시로 `Dockerfile.scan` 머리말과 `docs/design/2026-11-12-real-scan-obstacle-layer.md` 78 행에도 `:jazzy` 가 있다.

- 태그 = `forklift/ros2-scan:jazzy-s<입력 해시 8>`(amd64)와 같은 꼬리의 `-arm64`. 입력 해시는 패치 파일들 + `Dockerfile.scan`(기준 이미지 다이제스트와 SDK 커밋 인자가 들어 있음)의 sha256 이다(Codex P2-2: 패치만 같고 기준·커밋이 다른 이미지에 같은 태그가 붙지 않게).
- `config/real/lidar.yaml` 의 `scan_image: {amd64, arm64}` 가 유일한 기입처다. 빌드 진입점(amd64 Slurm, arm64 노트북 qemu)은 입력에서 태그를 계산해 이것과 다르면 멈춘다. `Dockerfile.scan` 은 0001·0002 를 순서대로 적용하고, 빌드 검사는 v2 계약(0002 기호, `@sdk_mono_v2`)으로 바꾼다. 빌드 기록에 이미지 ID·아키텍처를 남긴다.
- `ros2_dev.sh`·hwday 스크립트는 태그를 `lidar.yaml` 에서 읽는다. 타르볼 이름은 태그에서 만들고, 옛 고정 타르볼로 돌아가는 경로는 없앤다. `docker load` 뒤 그 태그의 이미지가 있는지 확인하고 실행한 이미지 ID 를 출력한다. `FORKLIFT_ROS2_SCAN_IMAGE` 덮어쓰기는 남기되 덮었다는 줄과 이미지 ID 를 출력한다(벤치 기록에 남게).
- 예시 둘은 빌드 진입점을 가리키게 고친다.

## 6. 시험(벤치 없이)

- **SDK 시험**(이미지 빌드 안, `build/` 지우기 전에 CTest 로; 실제 codec → 드라이버 콜백 → 해석기 → 첫·끝 시각 경로, Codex P2-3): 가짜 `IChannel` 이 capsule 바이트를 알려진 순번·시각에 내고, 디코더를 읽기 완료 뒤 300 ms 붙잡는다. 확인: 표본 시각이 그 capsule 을 완성한 읽기의 도착 시각(− 모드 지연)이다. 경우: capsule 이 두 읽기에 걸침, 한 읽기에 capsule 여럿, Dense 의 이전 capsule, Dense·UltraDense 에서 각도 점프 다음 capsule 이 자기 읽기 순번을 씀, 리셋 직후 이전 capsule 이 발행되지 않음, 도착 시각 없는 경로는 `arrival_ok = false` 로 `scan_sdk` 에 안 나감. 대조로 0001 만 댄 빌드에서는 같은 시험이 300 ms 늦게 찍힘을 기록한다. 이 시험이 증명하는 것은 도착 시각의 전달과 capsule 기준 선택이고, 절대 오차는 아니다.
- **저장소 단위 시험**: 어댑터 v1·v2 와 `stamp_kind`, `ScanFeed` 두 이름, 판정 함수(v1·측정 전·적용 조건 불일치는 진단, v2+측정+조건 일치만 상한)를 층·SLAM 두 소비자로, 텔레옵 다리의 두 형식 거부, 관측 헤더의 프레임별 계수, 태그 한 곳(입력 해시 = `lidar.yaml` 태그, `ros2_dev.sh`·hwday 가 그 태그를 씀; `test_ros2_dev_scan.py` 갱신).

## 7. 나눔과 게이트

- 크기 때문에 둘로 나눈다. **호스트를 먼저** 머지해, 중간 main 에서도 `sdk`·`raw` 가 모두 돈다(Codex 2 회차 P2-1).
  - **7a-3a 호스트**: 어댑터 v1·v2 수용과 `stamp_kind`·실행 식별자 검사·폐기 목록, `ScanFeed` 두 이름과 세대 변경 우선, 프레임별 판정(층·SLAM)과 원천 사실 필드, `RealSlamSource` 의 `diagnostic`/`operating` 과 세대 변경 때 시작 검사 무효화, HIL 조립 호출부(`real_run.py` 491 부근)의 `operating` 명시, 관측 헤더 계수, 텔레옵 시험, `ros2_dev.sh` 벤치 인자(`sdk`·실행 식별자·진단 기록 마운트·`publish_delay_ms`·이미지 ID 출력). 이미지는 아직 v1 이라 실 LiDAR 동작은 지금과 같다.
  - **7a-3b 이미지**: 패치 0002(도착 시각·Dense 기준·캐시 폐기·`scan_sdk_info`·진단 기록)·`Dockerfile.scan`·빌드 검사·SDK 시험·태그 한 곳(`lidar.yaml scan_image`, 빌드 진입점, hwday, 예시 문서)과 기본 태그 전환, 절차서 벤치 항목. 머지 상태마다 `sdk`·`raw` 를 확인한다.
- Isaac 경로(러너 `run_transport`·미션 코어·Isaac 장애물층)는 바꾸지 않는다. 재생 게이트는 그 경로만 재생하므로 생략으로 본다(Codex 동의). 실차 루프 쪽 변경(`RealSlamSource`·HIL 조립)은 HIL 통합 시험과 전체 pytest 로 확인한다. PR 에는 생략 근거와 함께 이미지 시험, `scan/1` v1·v2 소비 경로 시험, 텔레옵 `raw` 회귀를 적는다.
- 상한 값은 기입하지 않는다(`first_beam_err_s: null`, `basis: null` 유지).

## 9. 상한을 기입할 근거 — 총괄 결정(2026-11-26): B

**결정.** B 로 하되 측정값으로 보이지 않게 한다.
- 기록 이름: `timing.measured: true` 로 쓰지 않는다. `timing.basis: engineering` 과 근거 문서 경로(`basis_doc`)·결정 id(`decision_id`)를 함께 적고, 판정 코드는 basis 가 measured 이거나 engineering+결정 id(+근거 문서)가 있을 때만 상한을 쓴다. 이 구분은 3a 의 프레임별 판정에 들어갔다(값은 null).
- 기입 조건: (가)·(나) 진단과 SDK 지연 모델·`latency_timer`·전송 시간으로 보수적 값을 내고, 가정을 하나씩 적고, 그 값이 한쪽 11 ms 예산 안일 때만 기입한다.
- 재측정 조건: 운영 부하(Jetson·SLAM·층 동시)에서 다시 잰다. 진단 값이 상한의 절반을 넘으면 바로 무효로 돌리고 총괄에 올린다.
- 예산을 넘으면 A(독립 기준)나 C(S10 범위 조정)를 그때 다시 본다. 지금은 A 를 준비하지 않는다.
- 기입은 벤치 뒤 총괄 예외 결정으로 하고 기록 문서에 남긴다. 사용자에게는 측정이 아니라 공학 추정이라는 점을 알린다(총괄).

결정 전에 올린 선택지:

7a-3 의 코드(3a·3b)는 상한 없이도 진행된다. 다만 S10 은 실 스캔이 층·SLAM 증거가 되어야 하므로 상한 기입이 필요하고, Codex 는 학교 수준 벤치의 (가)·(나)로는 절대 상한의 근거가 안 된다고 본다. 선택지:
- **A. 독립 기준을 만든다**: 예) 호스트가 시각을 아는 순간에 LiDAR 평면을 가로지르는 물체(MCU 가 STATE 에 시각을 싣는 포크 승강이나 모터로 도는 표지)를 두고, 그 물체가 스캔에 처음 나타난 빔의 시각과 비교. 빔 순서·시각을 보존한 원자료가 필요하다. 준비가 크다.
- **B. 공학적 상한을 명시적 가정과 함께 쓴다**: (가)·(나) 결과와 SDK 지연 모델·`latency_timer`·전송 시간으로 보수적 상한을 정하고, 가정을 기록한 총괄 예외 결정으로 기입한다. 운영 부하에서 다시 재는 조건을 단다.
- **C. S10 범위를 조정한다**: 실 스캔 없이(층은 진단, SLAM 은 진단) S10 을 할 수 있는지 다시 본다.

## 8. 자체 검토

- **원자료 대조**(Claude): SDK 고정 커밋 34300099 을 받아 0001 이 깨끗이 적용되는 것을 보고, 수신·디코더 스레드(`sl_async_transceiver.cpp` 299–405), 해석기 시각(`dataunpacker.cpp` 177), 모드별 처리기(`handler_capsules.cpp` 256·739·1033), 보드레이트 추정(`sl_lidar_driver.cpp` 1516)을 직접 읽었다. 이미지 태그가 박힌 곳 셋과 `lidar.yaml` 에 없다는 것도 저장소 검색으로 확인했다.
- **Codex 적대적 검토, 한 스레드 6 회**:
  | 회차 | 지적 | 처리 |
  |---|---|---|
  | 1 | P1 1(벤치 식이 양방향 상한을 못 묶음, Dense 다음 capsule 기준, 회전 흔들림은 절대 오차 아님), P2 4(SLAM 판정, 태그 검증, C++ 시험 경로, 벤치 명령), P3 2 | Dense 기준 교정, 판정 함수 공용, 태그 입력 해시, 실제 codec 경로 시험, 런처 인자 |
  | 2 | P1 1(벽·자이로·직선 잔차로도 절대 상한 근거 없음), P2 4(PR 순서, 적용 조건을 원천에, 운영 SLAM 입력, Dense 캐시 폐기) | 벤치는 진단으로 낮추고 상한 근거는 총괄 결정(9절), 호스트 먼저, 원천 사실, `diagnostic`/`operating`, 캐시 묶음 |
  | 3 | P2 3(HIL 기본값, 재시작 뒤 모드, 원천 식별자), P3 1 | HIL `operating` 명시, 시작마다 원천 식별자, 세대 변경 우선 |
  | 4 | P2 3(다른 v2 노드, 폐기 ID 재수용, 옛 `ready` 수용 — 7c-1 부터 있던 틈), P3 1 | 실행 식별자, 폐기 목록, 세대 변경 때 시작 검사 무효 |
  | 5 | P2 1(최초 시작 검사 무효 뒤 브리지 세션) | 세션 끝·새 브리지, seed 는 `Cancel → void` |
  | 6 | 새 P1/P2 없음, P3 1(종료 범위 문구) | 반영 |
- **남은 것**: 상한을 기입할 근거(9절)는 총괄 결정이다. 독립 검토(새 스레드나 `agy`)는 총괄 지시에 따라 이번 계획에는 따로 두지 않았다.
