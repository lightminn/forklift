# 7r — 재생 실행: 뼈대와 계약

작성일: 2026-11-26 · 상태: **계획 11판 승인(총괄 2026-11-26) — 7r-1 구현 중.** 승인 조건: 7r-1 은 게이트 생략(루프·러너·미션 코어·층 무변경), 7r-2 의 루프 변경(HIL 확인 회차·완료 직전 검사)은 합성 HIL 에도 적용하되 재생 게이트 29/29 와 `test_real_run_hil` 전체 4 CPU 통과(흔들리면 `why()` 출력 첨부), `REPLAY_MIN_S` 40 s·시작 구간은 잠정(벤치 1′ 첫 기록으로 재확인) · 관련: [PR 7 설계](../design/2026-11-25-real-sources-pr7.md) 1.2·2 표 7r 행·8 절, [7a-3 계획](2026-11-26-7a3-sdk-arrival-stamp.md)

총괄 지시(2026-11-26, #637 머지 뒤): 다음 벤치 기록이 없으니 **뼈대·계약까지만** 한다. 기록 형식은 지금까지 머지된 관측(`observe_record/1`)·HIL 행(`slam_*` 포함)을 입력으로 잡는다. 계획은 Codex 수렴 뒤 승인 요청.

## 0. 범위·도달 범위·지금 할 수 없는 것

- **하는 것**: 관측 기록을 HIL 실행의 스캔·IMU 원천으로 내는 재생 원천, 기록 시각을 HIL 시각으로 옮기는 Δ, HIL 확인·부팅 경계(SLAM 포함), 재생 스캔을 실시간 실 SLAM 에 넣는 배선, `real_autonomy.py --run --hil --replay` 조립과 거부 규칙, 실 실행 층에 `lidar.yaml` 의 장착·주기를 배선(설계 4 의 "배선은 7r"), 합성 관측 기록으로 도는 시험.
- **도달 범위**(agy 1, Codex P2-5): 관측 기록은 차가 서 있거나 손으로 민 기록이고, HIL 모형은 미션대로 움직인다. 그래서 재생은 **원천 계약과 배관**(시각·세대·형식, 시작 검사·재위치·F1, 부팅·HIL 확인 경계, 정지 경로)을 보인다. HIL 이 움직이기 시작하면 실 SLAM 은 기록의 정지 장면에 맞춰 보정이 튀고 층은 정지 벽을 움직이는 오도메트리에 놓는다 — 추종·삽입까지 가는 것은 기대하지 않는다(설계 1.2 의 한계 그대로). 8 절 "7r 시점" 의 기한 측정은 **실제로 도달한 분기**(시작 확인·ARM·정지·대기)의 틱 기한으로 읽고, 추종 경로의 측정은 운동이 기록과 같은 입력(S10 의 실주행 기록)이 생긴 뒤의 몫이다.
- **지금 할 수 없는 것**: 출하 설정에서는
  - `lidar.yaml timing.basis: null`·`mount.measured: false` → 실 프레임은 층에서 `no_time_bound`, SLAM 에서 진단(`operating` 원천이 보내지 않음).
  - `imu.yaml` 지연 미측정 → `ImuSource(operating=True)` 가 `imu_source:delay_unmeasured`.
  - 그래서 출하 설정의 재생 조립은 **거부**한다(사유를 적어). 판정 경로까지 도는 재생은 시험 전용 설정으로만 보이고, 그 결과는 출력 표식으로 판정에서 뺀다(4 절). 실 측정은 벤치 1′(7a-3)·7b 상한·장착 실측 뒤.
- **하지 않는 것**: 깊이 재생(7e, D5 는 `d5_pocket` 우회), 출력 잠금 실보드로 재생(6b), SLAM 응답 재생.

## 1. 입력 — 관측 기록과 자격

**자격**(Codex P2-7, agy 7·9) — 조립 전, HIL 을 띄우기 전에 본다(agy 10: 5 s 기동 예산 밖):
- `real_observe.record_complete(path)`: `.ok` 의 sha256 이 파일과 같고 마지막 행이 `complete: true` 인 `end`.
- `end.result == "ok"`(`no_scans`·`no_imu`·`slam_failed`·`no_slam_replies`·`imu_overflow`·`interrupted` 는 거부). 완전한 기록에는 IMU 넘침이 없다(`complete` 가 거짓이 됨).
- 필수 스트림: `delay_bound_s` 가 유한한 `state` ≥ 1, 받아들인 `scan` ≥ 1, IMU 정책이 `required` 면 `imu` ≥ 1, `slam_header` 1 개.
- 시계 정합: 취득 시각을 가진(받아들인 SDK) 스캔마다 `first_beam_mono_ns ≤ last_beam_mono_ns ≤ recv_mono_ns ≤ host_recv_ns`(어댑터와 관측 도구가 같은 단조 시계였는지). 어긋나면 거부. 취득 시각이 없는 프레임(raw·v1)은 진단으로 보존하고 이 검사에서 뺀다(Codex 2 회차 P2-7).
- **선택 설정으로 운영 가능한 입력이 있는지**(Codex 2 회차 P2-7): 기동 전 확인 패스(2 절)가 선택한 `--lidar-config` 로 프레임마다 `frame_bound`·`to_layer_scan`·`from_feed` 결과를 미리 낸다. 실 브리지 기동 최대(설계 9 의 실측 8–12.5 s → `BRIDGE_STARTUP_MAX_S` 12.5 s) 뒤부터 시작 검사 기한(20 s) 1 s 전까지의 구간 `[앵커 + 12.5 s, 앵커 + 19 s]` 에 **한 생산자·피드 세대 안에서** 층·SLAM 둘 다 받는 스캔이 `START_SCANS`(7) 장 이상 없으면 `replay_no_operating_scans` 로 거부한다(Codex 3 회차 P2-5: 초반에 몰린 7 장이나 세대 경계를 사이에 둔 7 장은 시작 검사에 쓸 수 없다). 이것은 사전 최소 조건이고, 실제 성립은 실행 중 시작 검사 결과(기존 20 s 기한)로 판정한다.
- 합성: `synthetic.is_synthetic(header)`(벤치의 `{"bench": …}` 표식·pty 장치 포함) 또는 프레임 `synthetic: true` 가 하나라도 있으면 `--allow-synthetic` 없이는 거부.
- 길이: 첫 유효 STATE 뒤 기록이 `REPLAY_MIN_S`(잠정 40 s: 실 브리지 기동 8–12.5 s + 시작 검사 + 여유) 이상.

**읽는 행**(모두 지금 `real_observe.py`·`observe_slam.py`·`imu_source.py` 가 쓰는 행):

| 행 | 재생에서 |
|---|---|
| `header` | schema `observe_record/1`, 합성 표식, `lidar`(설정의 basis·applies_to), `imu`(`interval_bound_s`) |
| `state` | Δ 앵커에만: `delay_bound_s` 가 **유한한 수**인 첫 행의 `host_ns` |
| `scan` | 원시 프레임·`host_recv_ns`·기록 `verdict` |
| `scan_event` | `receive_gap`·`socket_error`·`receive_resumed`(피드 세대 사건, 2·3 절) |
| `imu` | 원시값(`host_ns`·`dev_us`·`ts_ms`·`dom`·`x,y,z`), 기록 판정은 대조용 |
| `imu_gen` | `why == "lost:<사유>"` 인 경계(장치 손실)와 그 `host_ns` |
| `slam_header` | `bundle`·`map_id`·`start_pose`(base_link)·`rear_offset_m`·`base_laser` |
| `slam_event`·`slam_scan`·`slam_fatal` | 읽지 않음(SLAM 응답은 재생하지 않음) |

HIL 실행 기록은 **출력** 형식이다. `head` 에 `replayed_from: {path, sha256, schema, configs: {경로: sha256}}` 를 달고, Δ 가 서면 `replay_anchor: {record_ns, hil_ns, delta_ns}` 행을 쓴다(Codex P2-1: `head` 를 쓸 때는 아직 Δ 가 없다).

## 2. 시각 — Δ 와 전달

- **HIL 앵커**(Codex P2-1, agy 5): 루프 `_on_state` 가 `DelayBound.add()` 직후 처음으로 유한한 지연 상한을 얻은 STATE 의 실제 `rx_ns`. 루프에 그 순간 한 번 부르는 콜백(`on_first_bounded_state(rx_ns)`)을 더해 재생 원천에 준다. 기록 앵커는 같은 조건(1 절). Δ = HIL 앵커 − 기록 앵커.
- **Δ 전**: 재생 원천은 아무것도 내지 않는다. 기록에서 앵커보다 앞선 행(스캔 소켓은 보드보다 먼저 열린다)은 `ScanFeed` 재판정 상태(시계 쌍 수립 등)를 만들 때만 쓰고 전달하지 않는다. 앵커 뒤 행 중 옮긴 도착 시각이 Δ 가 선 틱보다 이른 것도 몰아서 내지 않고 `before_anchor` 로 센다.
- **옮기는 것**: 스캔의 `recv_mono_ns`·`stamp_mono_ns`·`first/last_beam_mono_ns`, 행의 `host_recv_ns`, IMU `host_ns`, `imu_gen lost` 의 `host_ns`, `scan_event` 시각. **옮기지 않는 것**: ROS stamp 원값·`sys_minus_mono_ns`·장치 카운터·MCU 시각.
- **확인 패스와 실행 패스를 나눈다**(Codex 2 회차 P2-4):
  - **확인 패스**(기동 전, 기록 축, 기록 전체): 새 `ScanFeed` 에 스캔을 기록 순서대로, `scan_event` 의 `new_generation` 을 같은 자리에 섞어 넣고(agy 6a), 새 `ImuSource` 에 IMU 원시 행과 손실 경계를 기록 시각으로 넣어, 기록 판정과 같은지 본다. IMU 판정은 상한 말고도 `receive_gap_s`·`restart_back_us` 에 달린다(Codex 3 회차 P2-4). 그래서 7r-1 에서 관측 기록 머리에 IMU 원천 설정 전체(`ImuSourceConfig` 의 값과 설정 파일 sha256)를 더하고, 그것이 있는 기록은 그 설정으로 엄격히 대조한다. 없는 기존 기록은 지금 설정으로 대조하되 차이를 `imu_config_unrecorded` 로 따로 세고(`replay_mismatch` 가 아님) 거부하지 않는다. 스캔 판정은 설정에 기대지 않으므로 늘 엄격하다. 시계 쌍은 프레임 안 값으로만 서므로 같은 바이트·같은 순서면 스캔 판정이 재현된다(Codex 확인). 다르면 `replay_mismatch` 로 거부. 운영 가능 스캔 가용성(1 절)도 여기서 낸다.
  - **실행 패스**(HIL 위): 스캔은 새 `ScanFeed` 에 기록 처음부터 넣되(접두부는 상태만 만들고 전달하지 않음) 판정은 확인 패스와 같다. IMU 도 운영 설정의 새 `ImuSource` 에 **기록 처음부터 모든 행**을 넣는다(Codex 7 회차 P1: 관측기는 첫 유효 STATE 전에 IMU 를 시작할 수 있고, 앵커 앞의 장치 시각 되감기 같은 영구 정지를 빼면 원천이 되살아난다). 앵커 콜백은 Δ 만 확정한다. 앵커 앞 행은 그 뒤 호출마다 **틱당 상한(64 행) 안에서** 옮긴 시각(모두 과거)으로 파일 순서대로 넣고 곧바로 소비해 판정을 진행시킨 뒤 출력은 버린다(Codex 8 회차 P2: 확인 쌍을 재시도하는 동안 쌓인 수백 표본을 한꺼번에 넣으면 정상 기록도 큐 넘침으로 영구 정지한다). 접두부 처리가 끝나기 전에는 앵커 뒤 출력도 루프에 내지 않는다(그동안 루프는 IMU 표본을 받지 않고 `replay_imu_warmup` 으로 센다; 200 Hz 로 몇 초 분량이면 1 s 안팎). **도래한 행(접두부든 앵커 뒤든)이 아직 다 들어가지 않은 동안 `ReplayImu.last_read_capped` 는 참**이다(Codex 9 회차 P2): 기존 루프는 그럴 때 `generation()` 을 부르지 않으므로, 아직 넣지 않은 표본 때문에 기록에 없던 수신 공백이 생기지 않는다. 앵커 뒤 행은 2·3 절의 전달 규칙대로. 루프로 가는 출력은 앵커 뒤·부팅 경계 뒤의 것뿐이다. 운영 상한이 기록 상한과 다르면 판정이 기록과 다를 수 있어, 실행 패스는 대조하지 않고 차이를 센다.
- **전달**: 표본은 옮긴 도착 시각 이후의 호출에서만 처음 보인다. 늦게 온 스캔은 늦게 보이고, 층은 그것을 늦은 스캔으로 다룬다(`layer_overdue`·`layer_latency_exceeded` — Codex P2-3: 98 ms 회전에 300 ms 도착이면 층 예산 100 ms·SLAM `LATE_S` 250 ms 를 넘는다. 그것이 실 원천 그대로의 반응이다).
- **EOF**(agy 5, Codex 2 회차 P2-6, 3 회차 P2-3, 4 회차 P2-2): 스트림마다 색인 커서가 **마지막 행을 처리했거나 명시적으로 버렸을 때만** 끝난 것으로 본다(시각만 지났다고 끝이 아님 — 아직 읽지 못한 마지막 `socket_error` 를 정상 종료가 가리지 않게). LiDAR 스트림은 늘 활성이라 루프의 `scans()` 호출이 커서를 옮긴다. IMU 스트림은 루프가 읽지 않는 동안(IMU 감시 없음·`imu` 우회)에도 **모든 행을 운영 `ImuSource` 에 그대로 넣고(표본 `push`, 손실 경계 `lost`), 원천의 판정까지 진행시킨 뒤 감시기로 갈 출력만 버린다**: 러너의 EOF 검사가 `ImuSource.samples()` 를 대신 불러 판정을 진행시키고 결과를 버리며 센다. 그래서 장치 재시작·장치 시각 출처 변경·시계 불일치처럼 **원시 표본의 판정에서 생기는 영구 정지**도 우회 중에 그대로 남는다(Codex 5 회차 P1, 6 회차 P1: 손실 행이나 그런 표본 하나를 버리면 우회를 끈 뒤 고장 난 원천이 되살아난다). HIL 확인 상실·부팅 경계로 버리는 경우도 같다 — 넣고 판정을 진행시킨 뒤 출력만 버린다(부팅 경계 이전 `lo` 의 출력은 루프로 가지 않는다). 활성 IMU 의 "처리 완료"는 커서가 끝이고 **`ImuSource` 내부 큐(표본·경계)까지 루프가 소비했을 때**다(Codex 5 회차 P2: 수신 공백 경계가 하나 더 붙으면 마지막 손실이 읽기 상한 뒤에 남는다) — `ImuSource` 에 대기 항목 수를 묻는 읽기 전용 질의를 더하고, 마지막 읽기가 `last_read_capped` 면 끝이 아니다. 두 스트림이 끝나고 스캔 원천에 대기 중인 스캔이 없으면 `eof`. 러너의 `stop_reason` 에 `replay_eof` 를 더해(재생 원천일 때만) 그 다음 틱에 기존 정상 종료 경로(정지·DISARM 확인)로 닫고 `end.reason = replay_eof`, 종료 코드는 다른 정상 종료와 같다. 그 전에 스캔이 기한을 넘기면 층이 지금처럼 반응한다. 통합 시험이 실제로 `replay_eof` 로 닫히는 것을 본다.

## 3. 재생 원천(`forklift_core.live.replay_sources`, 새 모듈)

- **HIL 확인**(6a 규칙, agy 2 정정): 매 호출 `hil_plant is True` 이고 `hil_boot` 가 **지금 확인된 부팅**과 같아야 한다. 확인을 잃은 동안은 빈 결과이고 도착 시각이 든 표본은 버린다. 부팅이 바뀌면 그 순간의 HIL 시각을 경계로 두고, `lo`(스캔 첫 빔 하한·IMU `lo`)가 경계보다 이른 표본은 내지 않으며, 새 부팅에서 이어 간다.
- **`ReplayScan`** — `scans(now_ns)`·`scan_source_events(now_ns)`·`sensor`, `RunSources.slam_feed = None`(원천이 SLAM 에 직접 준다):
  - 한 호출에서 도착 시각이 온 사건(피드 스캔·`scan_event`)을 기록 순서대로 처리한다. 먼저 이번 호출에 층으로 갈 스캔(층 스캔이 되는 것 중 마지막 `SCANS_PER_CALL` 개)을 정하고, 그 다음 **기록 순서대로** 각 사건에 대해: 세대가 바뀌었으면 `slam.generation(…)`, 그 스캔이 층으로 가는 것이면 `slam.offer(from_feed(…))`(Codex P1: 세대 통지를 앞에 몰아서 하면 뒤에 넘긴 이전 세대 스캔이 세대를 되돌린다). 층으로 가지 않는 스캔(진단·잘린 것)은 SLAM 에도 주지 않는다(층이 받은 스캔이 SLAM 이 받는 스캔, 6.1 의 1). 반환 수가 `SCANS_PER_CALL` 이하라 루프의 자르기는 아무것도 자르지 않는다.
  - `scan_event` → 루프(agy 4, Codex 2 회차 P2-1): 상위 설계 3.2 의 운영 래치 규칙을 그대로 재현한다 — `receive_gap` 은 `("lost", sensor)`(즉시 래치 `scan_source_lost`), `socket_error` 는 원천 치명(`fatal_source`, 러너가 그 사유로 끝냄), `receive_resumed` 는 `("connected", sensor)` — 층의 연결 상태만 되돌리고 루프의 운행 래치는 그대로다(Codex 3 회차 P2-1: 연결을 되돌리지 않으면 층이 영구히 `disconnected` 라 수동 재무장도 못 한다). 끊김을 운행 복구로 다룰지는 S10 때 정한다는 설계 그대로다.
  - 층 스캔: `lidar_cfg.to_layer_scan(scan, cfg, start_ns)` 에 `origin="replayed"`(Codex P3, agy 9). 계수는 변환·전달·설치를 나눠 센다(층의 거부 사유 포함, Codex P3).
- **`ReplayImu`** — `samples(upto_ns, limit)`·`generation()`·`last_read_capped`(Codex P2-6):
  - 운영용 `ImuSource`(선택한 설정)에 주입 시계를 준다. 원시 행을 `push(host_ns + Δ, …)` 할 때와 손실 경계를 넣을 때는 주입 시계를 **그 사건의 옮긴 시각**으로 맞추고(agy 6b), 루프가 `samples(upto_ns)` 를 부르면 `min(upto_ns, T_read(IMU) + Δ)` 까지의 행을 넣고 그 값을 **입력 반영 시각**으로 적어 두며, 이어지는 `generation()` 은 지금 시각이 아니라 **그 입력 반영 시각**을 쓴다(Codex 10 회차 P2: 두 호출 사이에 다음 행이 도래하면, 지금 시각으로 공백을 판정할 때 아직 넣지 않은 표본 때문에 가짜 공백이 생긴다). 4 절 읽기 규칙과 같은 값이다. `ImuSource.lost(why, at_ns=None)` 에 시각 인자를 더한다(없으면 지금처럼 현재 시각).
  - 기록 판정 대조는 확인 패스의 몫이다(2 절). 실행 패스의 IMU 판정은 대조하지 않고 기록과의 차이를 센다.
- **SLAM**(Codex P2-2, agy 2; 2 회차 P2-2·3): 래퍼가 아니라 `RealSlamSource` 자체에 선택 인자 `hil_gate`(() → (확인 여부, 부팅))를 더한다(래퍼면 `isinstance(RealSlamSource)` 판별이 깨져 항등 `processed` 가 들어갈 수 있음).
  - 문 검사는 **모든 진입점의 맨 앞**(`offer`·`generation`·`replies`·`seed`·`request_processed`·`can_confirm`·`can_seed`)에서 한다. 부팅이 바뀌었으면 먼저 `input_break("hil_boot_changed")`, 진행 중인 최초 시작 검사는 세대 변경 규칙으로 끝내고, seed 는 기존 `boot_changed` cancel 경로. 루프의 확인 대기는 루프가 `relocalize_superseded` 로 닫는 기존 경로다.
  - 확인을 잃은 동안에도 연결 쪽 처리(프레임 읽기, 응답으로 슬롯 비우기, cancel 쓰기, void 받기)는 계속하고, **루프로 나가는 것만** 막는다: 응답은 `_out` 에 넣지 않고 세며, `can_confirm()`·`can_seed()` 는 거짓이다. 그래서 슬롯이 남아 다음 스캔·seed 가 막히는 일이 없다.
  - **루프의 완료 직전 검사**(Codex 4 회차 P1): 루프는 확인 시도가 `processed` 를 받은 뒤 층 설치를 기다렸다가 완료하는데, 그 완료 판정은 `can_confirm()` 을 다시 읽지 않는다. 그래서 루프에 HIL 확인 회차(전송기의 확인 상실·부팅 변경마다 오르는 수)를 두고, 확인·선언 시도마다 시작 때의 회차를 적어, **완료 직전** 지금 HIL 확인이 서 있고 회차가 같을 때만 완료한다. 아니면 그 시도를 `hil_check_lost` 로 끝낸다(합성 HIL 에도 같은 규칙: HIL 확인을 잃은 동안의 완료는 어느 원천이든 증거가 아니다). 시험: `processed` 를 이미 받고 층 설치를 기다리던 확인이 그 사이 확인 상실로 `confirmed` 가 되지 않음.
  - **진행 중인 확인·선언도 무효**(Codex 3 회차 P1): 루프의 선언 완료 판정은 `seed_state()` 를 직접 읽으므로, 확인을 잃는 순간 진행 중인 seed 는 `hil_check_lost` 로 cancel 하고 `seed_state()` 는 그 회차에 `("void", "hil_check_lost")` 를 낸다(void 가 브리지에서 돌아오기 전부터). 이미 `ready` 인 인스턴스는 확인을 잃은 뒤로 확인 불가로 두고, 확인이 돌아와도 새 seed·재위치 전에는 열지 않는다. 시험: 확인 상실 중 `seed_state()` 가 `ready` 를 내지 않음, 루프의 `declared` 0 건, F1 완료 표식 0 건.
  - 시험: 부팅 변경 뒤 늦은 `processed`·늦은 `ready` 각각 0 건, 확인 상실 동안 받은 응답 뒤 슬롯이 비어 다음 스캔이 나감, 진행 중 seed 의 cancel/void 가 확인 상실 동안에도 끝남.
  - 브리지는 바꾸지 않는다: `/clock` 은 현재 호스트 단조 시각, 스캔·TF stamp 는 실행기가 보낸 `stamp_ns`(이미 HIL 축). 가짜 브리지 시험은 소켓에 간 stamp 의 정수 일치와 응답 수용까지 보이고, 실 `/pose`·`/clock` 흐름은 기존 ROS 스택 점검(`tools/real_slam_stack_check.py`) 항목이다(Codex P3).

## 4. 조립 — `real_autonomy.py --run --hil --replay <기록>`

- 함께 써야: `--run --hil --hil-slam-bridge`. 거부: `--device`, `--hil-inject scan`/`scan_lost`(재생 원천에 주입 훅이 없음, agy 10).
- **준비 단계**(Codex 2 회차 P2-3): `prepare_run()` 의 `hil:` 장면 요구와 비영 `start_rear` 거부는 재생 준비에서만 풀고(장면 대신 기록·번들), 일반 HIL 과 `--hil-slam-bridge` 만 켠 HIL 은 지금처럼 둔다. 두 경우의 층 장착·주기가 바뀌지 않았음을 회귀 시험으로 고정한다.
- **설정 정합**(Codex P2-4, agy 3·8):
  - `--lidar-config`(기본 `config/real/lidar.yaml`) 하나가 층과 SLAM 모두의 LiDAR 원천: 실 실행 층의 센서 장착·`scan_period_s` 를 `obstacle_layer_real.yaml` 대신 이 설정에서 읽는다(재생 조립에서만; 합성 HIL 은 그대로). `mount.measured` 가 거짓이면 거부(층만 받고 SLAM 은 모두 진단이 되는 반쪽 상태를 막음).
  - 지도: 기록 `slam_header.bundle`, 또는 그 경로가 이 머신에 없을 때 `--map-bundle`(그 번들의 `map_id` 가 기록과 같아야).
  - 시작: 기록 `start_pose`(base_link)를 기록 `rear_offset_m` 로 후륜축 자세로 바꾸고, 지금 차량 모델의 후륜 오프셋과 같아야 한다. `--s10a` 의 `map_id`·`start_rear` 가 그것과 다르면 거부한다(허용 오차는 7c-2a 의 시작 비교와 같은 0.10 m / 5°). 루프·`check_plan`·`hello` 가 모두 이 한 자세를 쓴다.
  - `base_laser`: `--lidar-config` 장착으로 계산한 값이 기록 `slam_header.base_laser` 와 같아야 한다. 브리지의 정적 TF 는 브리지 기동 인자라 실행기가 대조하지 못한다 — 브리지 기동 명령을 기록에 남기고, ROS 스택 점검 항목으로 둔다.
  - `--watch` 의 `level_ref`·IMU 감시가 기록 IMU 와 맞지 않으면 ARM 이 거부된다(agy 1) — 그 거부도 재생이 보이는 원천 계약의 일부로 기록한다.
- 출하 설정(상한·장착 미측정)이면 `replay_needs_bound`(어느 설정의 무엇인지) 로 거부.
- **판정 제외 표식**(Codex P2-7, agy 9): 출력 `head` 에 `replayed_from.configs` 를 남기고, 다음 중 하나라도 해당하면 `OUTPUT_MARKS`(`synthetic_inputs`·`not_for_judgement`)를 단다: `--allow-synthetic`, 기록·프레임의 합성 표식, 출하 경로(`config/real/`)가 아닌 설정. 시험 전용 설정은 `tests/fixtures/replay/` 에만 둔다.
- **기록 읽기**(agy 10, Codex 2 회차 P2-5, 3 회차 P2-2): 자격 검사·확인 패스·색인은 HIL 기동 전에 끝낸다. 색인은 **스트림 두 개**의 파일 오프셋 목록이다: LiDAR(`scan`·`scan_event`)와 IMU(`imu`·`imu_gen` 손실 경계). 각 스트림 안은 **파일 순서 그대로** 둔다(Codex 4 회차 P2-1: 관측기는 같은 호출의 `socket_error` 에 호출 시작 시각을 붙여 앞선 스캔보다 이른 시각이 될 수 있다 — 시각으로 정렬하면 판정 순서가 뒤집힌다). 시각은 각 행을 **언제 내보낼지**에만 쓴다: 한 행은 그 시각 + Δ 가 지났고 같은 스트림의 앞 행이 모두 처리된 뒤에 나간다. 두 스트림은 서로 떨어져 있어, 관측기가 한 사이클에서 스캔을 먼저 쓰고 더 일찍 받은 IMU 를 뒤에 써도 IMU 가 스캔에 막히지 않는다(30 분 기록에 수 MB). 실행 중에는 읽기 스레드가 스트림마다 따로 그 목록을 따라 행을 읽어 스트림별 크기 제한 큐에 넣고, 그 스트림의 **읽기 완료 시각** `T_read`(큐에 넣은 행들 중 가장 늦은 시각, 목록 끝이면 무한 — 파일 순서라 행 시각이 조금 거꾸로 갈 수 있으므로 최댓값)를 함께 낸다. 스트림이 서로를 기다리지 않으므로 한 큐가 찼다고 다른 스트림이나 자기 소비가 막히지 않는다. 원천은 스트림마다 `min(지금, T_read + Δ)` 까지만 전달한다. 읽기가 지금을 못 따라가면 `replay_read_late` 로 세고, 그 구간은 결과에서 원천 공백과 따로 표시한다. 틱 안에서는 디스크를 읽지 않고, 틱당 처리하는 원시 행 수는 루프의 반환 상한(스캔 4·IMU 64)을 따른다.
- 기록: `head.replayed_from`, `replay_anchor`, `end` 에 원천 계수(재판정 대조, `before_anchor`, 부팅·확인 경계로 버린 수, 변환·전달·설치, `replay_read_late`, IMU 대조 차이).

## 5. 시험(벤치 없이)

- **합성 관측 기록 작성기**(`tests/fixtures/replay/make_record.py`, 시험 전용): 지금 행 형식으로 `header`·세션 쌍과 유한한 `delay_bound_s` 의 `state`·v2 `scan`(7a-3a `SdkStream` 프레임, 원천 사실 포함)·`imu`·`imu_gen`·`slam_header`·`end` 와 `.ok` 를 쓴다. 관측 통합 시험이 실제로 만드는 기록과 행 종류·키 집합을 대조하는 시험을 둔다(작성기가 실제 형식에서 벗어나면 실패).
- **단위**: Δ 앵커(루프 콜백, 같은 읽기에 든 두 STATE), Δ 전 0 건과 앵커 이전 행의 재판정 사용, 옮긴 도착 시각 전 미전달(300 ms 늦은 스캔), 재판정이 `scan_event` 순서를 포함해 기록과 같음과 다르면 `replay_mismatch`, **같은 호출의 A(운영) → 진단 B 와 A → `receive_gap`** 에서 이전 세대 응답 0 건(Codex P1), `receive_gap` → lost 래치와 `receive_resumed` → connected(래치 유지), `socket_error` → `fatal_source` 와 러너 종료 사유, EOF(IMU 우회 중 포함), HIL 확인 상실·부팅 경계(스캔·IMU·SLAM 늦은 `processed`·늦은 `ready`), IMU 손실 시각 보존과 `last_read_capped` 전달, `imu` 우회 중 손실 → 우회 해제 → 이후 표본 0 건, `imu` 우회 중 장치 시각 되감기 → 우회 해제 → 이후 유효 표본 0 건, 앵커 전 출력 0 건과 앵커 전 영구 정지 뒤 유효 표본 0 건, 앵커 앞 400 표본이 넘침 없이 처리되고 그 뒤 표본이 정상 수용(실제 루프 호출 순서 `samples()`→`generation()` 로, 가짜 수신 공백 0 건·앵커 뒤 첫 수용 표본 시각이 기록과 같음), `samples()` 와 `generation()` 사이에 다음 행이 도래해도 가짜 공백 0 건, 마지막 손실이 읽기 상한 뒤에 남을 때 EOF 가 그 소비를 기다림, 자격 거부 전부(`.ok`, `end.result`, 필수 스트림, 시계 정합, 합성, 길이), 설정 정합 거부 전부.
- **통합**(가짜 브리지): 합성 기록 + 시험 설정으로 `--run --hil --replay`:
  - 0 이 아닌 x·y·yaw 시작에서 재위치 증거(`ready`) 뒤 **확인 요청 이후의 `processed`** 와 새 층 설치를 각각 관찰(Codex P2-5),
  - F1: 운용자 시도만 `STOP`(또는 lease 상실)으로 끝나고 브리지 seed 는 `ready` 까지 간 경우 → 후속 확인 → F1 차단을 따로 본다(시작 확인만으로는 F1 차단이 처음부터 없고, 부팅 변경으로 void 된 seed 는 확인할 인스턴스가 없어 F1 전에 `can_confirm()` 에서 막힌다 — Codex 2 회차 P3),
  - 다른 날의 기록(다른 `host_ns` 축)으로 seed 뒤 첫 유효 `/pose`(소켓에 간 stamp 가 HIL 축),
  - 출력 `head.replayed_from`·`replay_anchor`·`end` 계수·판정 제외 표식.
  - 가짜 브리지는 운동 불일치를 가린다(Codex P2-5) — 이 시험은 배관 증명이고, 실 slam_toolbox 재생은 ws1 ROS 스택 점검 항목으로 둔다.
- **게이트**: 실차 루프 조립(`real_run`·`real_loop` 콜백·`real_autonomy`)을 건드리므로 재생 게이트를 최종 트리에서 한 번.

## 6. 나눔

- **7r-1 재생 원천**: 자격 검사·색인·읽기 스레드(1·4 절의 기록 부분), Δ·전달(2 절), `ReplayScan`·`ReplayImu`·HIL 확인·부팅 경계, `ImuSource.lost(at_ns)`, 합성 기록 작성기와 단위 시험. 루프·조립은 건드리지 않는다(게이트 불필요).
- **7r-2 조립**: 루프 앵커 콜백·HIL 확인 회차와 완료 직전 검사, `--replay` 조립·거부·설정 정합(층의 `lidar.yaml` 배선), SLAM 재생 문, `replayed_from`·`replay_anchor`·표식, 통합 시험, 재생 게이트.
- **측정(8 절 7r 시점)**: 벤치 1′·7b 상한·장착 실측과 실 관측 기록 뒤. 이 계획 밖.

## 7. 자체 검토

- **원자료 대조**(Claude): 관측 기록 행(`real_observe.py` 의 `rec.put` 전부·`end` 의 `complete`/`result` 계산·`record_complete`), `state_row`, `ImuSource` 의 `imu`·`imu_gen` 행과 `push`·`lost`, `observe_slam` 의 `slam_*` 행, `RunSources` 계약과 `real_run` 의 스캔 공급(`SCANS_PER_CALL` 자르기 뒤 `slam_feed`, 사건의 lost/connected 처리), 층 조립(`obstacle_layer_real.yaml` 의 `scan_period_s`·장착), 합성 원천의 HIL 확인, 실차 모드 브리지 시계를 코드에서 읽었다.
- **Codex 적대적 검토**(한 스레드):
  | 회차 | 지적 | 처리 |
  |---|---|---|
  | 1 | P1 1(같은 틱 세대 일괄 통지가 이전 스캔을 되살림), P2 7(Δ 앵커, SLAM 부팅 경계, 늦은 스캔·층 주기, 장착·시작 좌표, 운동 불일치와 가짜 브리지, IMU 손실 시각·읽기 상한, 입력 자격), P3 2(브리지 증명 범위, 계수) | 기록 순서 처리·층으로 가는 스캔만 SLAM, 루프 앵커 콜백과 `replay_anchor`, SLAM 재생 문, 층 배선·설정 정합, 도달 범위 명시·시험 분리, `lost(at_ns)`·`last_read_capped`, `record_complete`·결과·스트림·표식 |
  | 2 | P1 없음(1 회 P1 해소), P2 7(`receive_gap` 래치 정책, SLAM 문의 슬롯·void 정리, `prepare_run` 제약과 래퍼 판별, IMU 접두부와 대조, 읽기 큐의 가짜 공백, EOF 종료 경로, 진단 프레임 자격·운영 스캔 가용성), P3 1(F1 음성 사례) | 설계 3.2 래치 규칙 재현(`receive_gap` → lost, `socket_error` → 치명), `RealSlamSource` 안의 `hil_gate`(맨 앞 검사, 연결 처리는 계속·루프로만 막음), 재생 준비에서만 제약 해제 + 회귀, 확인 패스(기록 전체·기록 축)와 실행 패스 분리, 스트림별 큐·`T_read`·`REORDER_S`, 러너 `replay_eof`, 시계 정합은 취득 시각 있는 프레임만·`replay_no_operating_scans`, F1 사례 명시 |
  | 3 | P1 1(확인 상실 중 `seed_state()` 로 선언 완료 → F1 기록), P2 5(`receive_resumed` 무시로 층 영구 단절, 읽기 큐 교착·IMU 시계 규칙 불일치, IMU 우회 중 EOF, IMU 설정 미기록으로 판정 재현 불가, 시작 검사 공급 가능성), P3 1(시험 목록) | 확인 상실 때 진행 seed cancel·`seed_state()` void·ready 인스턴스 확인 불가, `receive_resumed` → connected, 스트림별 정렬 색인·스트림별 `T_read`·IMU 시계 한 규칙, 시각 기준 EOF, 관측 머리에 IMU 설정 전체 추가·없으면 `imu_config_unrecorded`, 시작 구간 `[앵커+12.5, 앵커+19]` 한 세대 7 장, 시험 목록 정정 |
  | 4 | P1 1(`processed` 를 받고 층 설치를 기다리던 확인이 확인 상실 틱에 완료), P2 2(시각 정렬 색인이 LiDAR 행 순서를 뒤집음, 읽지 못한 마지막 오류를 시각 EOF 가 가림) | 루프의 HIL 확인 회차와 완료 직전 검사(합성 HIL 포함), 스트림 두 개를 파일 순서로·시각은 내보낼 때만·`T_read` 는 최댓값, 커서 기준 EOF(IMU 는 미사용 때 명시적 폐기) |
  | 5 | P1 1(우회 중 손실 경계를 버리면 고장 원천이 되살아남), P2 1(활성 IMU 의 EOF 가 내부 큐 소비 전에 닫힘) | 버릴 때도 손실 경계는 적용, 활성 IMU 완료 = 커서 끝 + 내부 큐 소비(대기 수 질의, `last_read_capped` 면 미완), 시험 둘 |
  | 6 | P1 1(버린 원시 표본이 장치 재시작을 드러낸 유일한 표본이면 원천이 되살아남), P2 없음 | 읽지 않는 동안에도 모든 행을 넣고 판정을 진행시킨 뒤 출력만 버림(HIL 확인 상실·부팅 경계도 같음), 시험 추가 |
  | 7 | P1 1(앵커 앞 IMU 행을 빼면 그 영구 정지가 사라짐), P2 없음 | 실행 패스 IMU 도 기록 처음부터 모든 행을 넣고 앵커 앞은 판정만 진행·출력 버림, 시험 둘 |
  | 8 | P1 없음, P2 1(앵커 앞 행을 한 틱에 넣으면 큐 넘침) | 앵커 콜백은 Δ 만, 접두부는 틱당 64 행으로 넣고 소비·출력 버림, 끝날 때까지 앵커 뒤 출력 보류, 시험 |
  | 9 | P1 없음, P2 1(접두부 처리 중 `generation()` 이 가짜 수신 공백) | 도래 행이 남은 동안 `last_read_capped` 참(루프가 `generation()` 보류), 실제 루프 호출 순서 시험 |
  | 10 | P1 없음, P2 1(`samples()`·`generation()` 의 시각이 달라 그 사이 도래 행이 가짜 공백) | `generation()` 은 직전 `samples()` 의 입력 반영 시각을 씀, 시험 |
  | 11 | 새 P1/P2 없음(수렴) | — |
- **독립 검토**(agy Gemini 4 Neon High, 적히지 않은 전제 — Codex 결론을 보지 않음): 10 항목. 운동 불일치(→ 0 절 도달 범위), SLAM 의 HIL·부팅 경계(→ 3 절), 층·SLAM 설정 불일치와 `mount.measured`(→ 4 절), `scan_event` 의 루프 사건 변환과 자르기 전 세대 통지(→ 3 절), Δ 앵커·Δ 이전 행·`hello` 기한·EOF·시계 정합(→ 1·2 절), 재판정에 `new_generation` 순서·IMU 손실 시각·상한 차이·완전 기록의 넘침 없음(→ 2·3 절), `.ok`·결과·필수 스트림(→ 1 절), 지도 경로·시작 좌표·`base_laser`(→ 4 절), 합성 표식·판정 제외·`origin`(→ 1·3·4 절), 기동 예산·틱 안 I/O·주입 훅(→ 4 절).
