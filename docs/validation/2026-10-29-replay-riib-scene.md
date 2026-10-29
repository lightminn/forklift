# 재생 도구의 riib 장면 기록 대응 (2026-10-29)

총괄 지시: 런처 기본 켬 배치에 앞서 재생 도구(`tools/boundary_replay.py`, `src/forklift_core/mission/replay.py`)가 riib 장면 기록을 재생하게 한다.
- riib 장면 기록이란 `riib_scene.sbatch`, slam 모드의 `--recognition far`, 영상 켬, 단계 사진 카메라로 찍은 기록이다.
- 러너의 미션 동작은 바꾸지 않고 기록·재생 쪽만 고친다.

코드: `fix/replay-riib-scene`, 최종 `10f93cec`(ws1 스냅숏 `rp_10f93cec`).
- 재생 게이트 runs29 29/29(`test_runs/replay_gate/rp_10f93cec`).
- ws1 전체 시험(아래 PR 본문).
- Codex 구현 리뷰 3 회차 수렴(새 P1/P2 없음).

## 재생한 riib 기록

| 기록 | 모드 | 끝 | 재생 |
|---|---|---|---|
| `xpc_b3b26aca/xpc_B_aisle_rack_s0_c3truth` | truth | 완주 | 33 095 틱, 종단 상태 일치 |
| `rpr3_6fe86a6d/rpr3P_aisle_rack_s0` | slam(far) | transport 추종 실패 | 24 232 틱, 같은 예외로 끝남, 종단 상태 일치 |
| `rpr3_6fe86a6d/rpr3P_random_1_s0` | slam(far) | 완주 | 26 090 틱, 종단 상태 일치 |

재생 결과는 ws1 `test_runs/replay_gate/rp_try/s5_{1,2,3}.json` 이다. slam 기록 두 건은 `6fe86a6d` 로 찍었다. 그래서 아래 4 의 주행 프레임 감시가 없지만, 두 실행 모두 마지막 stop 뒤에 주행 프레임이 없어 재생 결과가 같다. 다음 배치는 최종 코드로 찍는다.

`xpi_box15_aisle_rack_s0_c3truth` 는 재생되지 않는다. `scene_replans` 에 프로세스 주소가 섞인 종단 해시를 담은 옛 기록이기 때문이다(아래 5).

## 막힘과 수정

1. 단계 사진 카메라(`OverviewCamera` 설정 서비스)는 그림만 만든다. 재생에서는 아무 일도 하지 않는 대역으로 바꾼다.
2. 영상 켬 기록은 영상 프레임에서 나오는 출력을 물리 step 안에서 채운다. 대상 출력은 `frame_audit.json`, `video_frames.json`, `plan_history.json`, `obstacle_grid_frames.npz` 다. 재생은 물리 step 을 돌리지 않으므로 다음처럼 다룬다.
   - 이 출력들은 내용 대신 경로로 맞춘다.
   - 재생이 쓰지 않는 쓰기는 기록에서 성공한 경우에만 건너뛴다.
   - 기록에 없는 쓰기, 기록에서 실패한 쓰기의 생략, 재생에서만 난 쓰기 실패는 모두 불일치로 본다.
   - 종단 상태 비교에서는 `state.frames` 하나만 뺀다.
3. far 인식 런타임은 형식 2 기록에서 `FarRecognition` 포트로 기록한다. 프레임과 stop 요청은 해시로 남긴다. 설정 스냅숏에서는 opaque 값이 아니라 서비스로 잡힌다.
4. `FarRuntime.summary()` 는 자기 frames·attempts·leg_endings 리스트를 그대로 넘긴다. 그 뒤의 주행 프레임(물리 step 안)과 stop 이 이 리스트에 덧붙인다. 그래서 `state["recognition"]` 의 세 리스트를 append-only 로 감시한다. 감시 위치는 물리 step 과 far 포트의 `drive_frame`·`stop` 두 곳이다.
5. `scene_replans[*].tracker_serial` 은 `id(tracker) % 10**6`, 곧 프로세스 주소다. 종단 투영에서는 처음 나온 순서의 번호로 바꾼다. cusp 추적기 id 를 다루는 기존 방식과 같다.
6. 응답이 인자의 객체를 그대로 품는 경우가 있다. 예: `final_straight_prefix` 가 `dataclasses.replace` 로 운반 경로의 `exit_prefix` dict 를 넘긴다.
   - 기록기가 [응답 경로, 인자 경로] 쌍을 남긴다.
   - 재생은 값이 같은지 확인한 뒤 그 자리에 재생 쪽 인자 객체를 넣는다.
   - 쌍이 없는 옛 기록(`true`)은 예전처럼 거부한다.
   - 순환 응답은 컨테이너마다 한 번만 돈다. 튜플은 codec 이 자리마다 따로 쓰므로 자리마다 쌍을 남긴다.

1·2 는 재생 도구만 바꾼다. 3·4·5·6 은 기록과 종단 투영만 바꾸고, 실행은 같다. runs29 기록에는 far 인식, 영상, `scene_replans` 가 모두 없어 이 경로들이 게이트에서는 쓰이지 않는다.

## random_1 slam 기록의 멈춤

`rpr_275cc9e5/rpr_random_1_s0`(잡 12712)은 insert 단계 t≈60 s 에서 멈춰 시간 한도로 끝났다. 남은 스택은 없다.

같은 장면을 `rpr3_6fe86a6d` 에서 두 팔로 다시 찍었다. far 포트 래핑이 있는 팔(P)과 없는 팔(N, `70320bb4`) 모두 완주했다. 멈춤은 재현되지 않았고, 래핑이 원인이라는 근거도 없다.
