# S4 라이브 확인 — 경로 표시 overlay 의 라이브 영상 가시성·어긋남·비용 (2026-11-15, 초안, 결과 전)

- 근거: [경로 표시 누출 수정 계획](2026-11-14-path-display-perception-leak.md) 4 절 S4(총괄 승인). 구현 #512·#513·#515 를 포함한 origin/main `23b858b3`.
- 상태: 제출 전 등록. 7 절 확인 결과와 8 절 자체 검토(Codex) 반영.

## 1. 목적

콘솔 라이브 세션에서 아래 세 가지를 잰다. 그 값을 보고 총괄이 `--path-display-pixel-lag-s` 를 고른다. 이 문서는 통과·실패 문턱을 두지 않고 값만 보고한다.

- 범위 밖: 표시선이 perception 카메라 입력(원·근접 depth, 인식 RGB)에 섞이는지는 이 실행이 판정하지 않는다. 그것은 결정 모드에서 overlay 두 번과 off 의 센서 해시를 비교한 S1 의 몫이다([smoke 결과](../validation/2026-11-14-path-display-fix-smoke.md)). 이 실행이 기록하는 영상은 콘솔 서버의 media socket 스트림(캡처가 media_subscribe 로 받는 JPEG)뿐이다.
- 결과가 말하는 범위(독립 검토 반영):
  - 가시성은 **media socket 스트림에서의 가시성**이다. 브라우저 콘솔의 디코드·리사이즈·프레임 드롭·화면 갱신·단말 표시 지연은 들어 있지 않다.
  - 어긋남은 **모형 기반 오프라인 추정**이다. 카메라 자세를 참값 base 자세 + 마운트 + 리프트로 다시 만들고(overlay 가 읽는 카메라 prim 자세 이력이 아니다, live_frames.py:811-819), 참 지연 11/120 s 를 가정한다(프레임에서 읽은 값이 아니다). 재구성 자세와 실제 카메라 자세의 차이는 재지 않았다 — S3 도 그 비교를 하지 않고 영상의 선 일치(중앙값 1 px 안)만 보였다(smoke 결과 4 절). 그래서 delta 값에는 이 차이를 뺀 오차 범위를 붙일 수 없다고 적는다. 실측 end-to-end 지연도 없다.
  - 비용·pacing 은 **캡처와 탐침이 같은 CPU 4 개에서 함께 돈 조건**의 값이고, 팔마다 한 세션이다. 캡처 없는 운영 조건의 비용 추정으로 쓰지 않는다.

1. 계획 경로·목적지 고리 선이 콘솔 라이브 영상에 보이는가: `tools/live_frames.py capture` + `judge`.
2. 오프라인 어긋남 추정: 픽셀 지연 설정 0 과 11/120 s 에서의 값(`tools/live_frames.py delta`, `RUN/path_display.jsonl` 을 읽음).
3. 비용: `live_log.jsonl` status 줄의 timers(`step.*`, `overlay.compose`, `overlay.note_moving`, `isaac.step`)와 pacing.

## 2. 두 팔과 고정 조건

| 항목 | 값 | 근거 |
|---|---|---|
| 팔 A | `--path-display` 기본(overlay). `EXTRA_ARGS="--max-sim-seconds 1200"` | run_transport.py:963-972 기본 overlay |
| 팔 B | `EXTRA_ARGS="--path-display off --max-sim-seconds 1200"` | 같은 곳. 런처는 stage 만 거부한다(isaac_live.sbatch:87-98) |
| 픽셀 지연 | 두 팔 모두 `--path-display-pixel-lag-s` 를 주지 않는다(기본 0). 11/120 s 는 오프라인 delta 로만 본다 | run_transport.py:973-981, 1677-1678 |
| 스냅숏 | 이 등록의 커밋(origin/main 위에 이 문서와 래퍼만 더함) 하나를 `git archive` 로 `snapshots/s4ov_<커밋 8자>` 에 푼다. 두 팔이 같은 것을 쓴다 | 7 절 |
| 런처 | 스냅숏 안 `deploy/console/isaac_live.sbatch` 를 래퍼 `deploy/console/s4_overlay_run.sh` 가 부른다 | — |
| 장면 | `random_m`(registry 행 `random_m_session.json`, sha256 8b8fe554…, `start_relocalizable: true`) | config/slam_map_bundles.yaml:22-35. line_m(:4-17) 대신 random_m: P-1·D-1 이 있고 registry 묶음이 있다(7 절) |
| SLAM | `MAP_BUNDLE=registry`: registry 묶음으로 위치 추정만 한다. 행이 없으면 실행을 거부하고 mapping 으로 대체하지 않는다. `SLAM_BUNDLE_ROOT` 가 있어야 한다 | slam_choice.sh:24-41 |
| 시작 | `START_RELOC=off` → START_MODE "known (START_RELOC=off)" | isaac_live.sbatch:259-270 |
| seed / 배속 | `SEED=0`, `FACTOR=0.4`(런처 기본) | isaac_live.sbatch:339 |
| 미션 | 탐침 cycle 시나리오의 한 다리 `P-1:D-1`, 끝나면 `--end-session` | live_console_probe.py:1142-1444, 2559-2562. random_m 에 있음(7 절) |
| epoch | `MAX_EPOCHS=1`, `STANDBY_LINGER_S=0` | isaac_live.sbatch:213, 517-519 |
| 영상 | VIDEO 끔. 런처가 `--video` 를 넘기지 않는다 | isaac_live.sbatch:433-450, run_transport.py:948 |
| 원거리 인식 | 런처 기본(`--recognition` 없음 = off, learned 작업자 없음) | run_transport.py:999-1008 |
| 틱 추적 | `FORKLIFT_LIVE_TICK_TRACE=1`. 추적 버퍼 크기 때문에 `--max-sim-seconds 1200` 을 함께 준다(rv3 래퍼 H4 와 같은 이유) | run_transport.py:8903-8911. 두 팔 같은 값 |
| 따라잡기·위상 | `FORKLIFT_LIVE_CATCHUP_STEPS`·`FORKLIFT_LIVE_CAMERA_PHASE` 는 설정하지 않는다(0, 런타임 기본) | run_transport.py:8549-8559 |
| 런처 기본(그대로) | ENCODER_MODEL team_spec_quantized, D7_HOLD on, FAR_EXPECTED on, G3 1b·1c·1d 켬, RETURN_START boxed, NEAR_END_STAND on, NEAR_OBSERVE_ARRIVAL 0.05,0.05, LAYER_BUDGET 기본 파일 | isaac_live.sbatch:105-201, runner.sh 410 |
| 자원 | `--gres=shard:1 --cpus-per-task=4 --mem=7G --time=01:15:00 --signal=B:TERM@120` | wiki ws1-isaac.md(단독 live shard:1·CPU 4·7G, ct1_queue 1:15). 런처 기본은 CPU 6·8G·8 h(isaac_live.sbatch:4-7) |
| 실행 순서 | A 를 먼저 낸다. A 가 끝나고 읽은 뒤 점검(4 절)을 다시 하고 B 를 낸다. 노드에 단독으로 돈다. 잡 안에서는 런처 → 포트 running → 캡처 → 캡처의 첫 프레임 → 탐침(미션 제출) 순이다 | 래퍼 |

- CPU 4 의 뜻: 브리지에 물리 코어 하나, 세션에 물리 코어 하나가 간다(isaac_live.sbatch:305-333). 탐침과 캡처는 따로 묶지 않아 잡의 CPU 4 개를 함께 쓴다. 두 팔이 같은 조건이다.
- 시간: 미션 한도 2,400 s(ct1 값). 미션 하나짜리 세션의 실측 길이는 없다. 예약 1:15 는 기동(워밍업 포함 약 10–15 분)과 모의 최대 1,200 s 를 배속 0.4 로 돈 3,000 s 를 더한 값이다. 한 다리(P→D)는 CT 기록에서 모의 약 210–260 s 라 보통은 훨씬 먼저 끝난다. 실현 배속이 크게 낮아 한도에 걸리면 6 절대로 무효다.

## 3. 실행

```bash
cd <SNAP>
# 래퍼가 무엇을 돌릴지 출력만 한다(아무것도 실행하거나 쓰지 않음)
ARM=A VENV_PY=... BASE_SCENE=... FORKLIFT_URDF=... SCENE_FILE=.../random_m_session.json SLAM_BUNDLE_ROOT=... \
  DRY_RUN=1 bash deploy/console/s4_overlay_run.sh test_runs/s4ov/23b858b3/A
# 제출(4 절 점검 뒤)
ARM=A ... sbatch --chdir=<SNAP> --job-name=s4ov_A deploy/console/s4_overlay_run.sh test_runs/s4ov/23b858b3/A
```

- 출력 폴더 구조:
  - `OUT/epoch_1/run/`: 러너 출력(result.json, live_log.jsonl, path_display.json(l), tick_trace.npz)
  - `OUT/frames/`: 캡처
  - `OUT/probe.json`
  - `OUT/{launcher,probe,capture}.log`, `OUT/s4_wrapper.log`, `OUT/args.txt`
- 캡처를 러너 출력 안(`epoch_1/run/frames`)이 아니라 `OUT/frames` 에 둔다. 러너 출력 폴더를 바꾸지 않기 위해서다(run_transport.py:12135-12139).
- JPEG 용량 추정:
  - 카메라는 모의 시간 10 Hz(live_runtime.py:183, server.py:33)이고, 폭 640 이하(run_transport.py:8513-8514), JPEG 품질 80(server.py:60)이다.
  - 벽시계 10 Hz 라면 36,000 장/h × 25–60 KB(가정, 실측 아님) ≈ 0.9–2.2 GB/h 다.
  - 실현 배속 약 0.37 이면 벽시계 약 3.7 Hz 이므로 약 0.3–0.8 GB/h, 세션 하나에 0.1–0.4 GB 다.
  - 첫 실행의 `index.jsonl` `bytes` 로 다시 잰다.

## 4. 제출 전 점검(팔마다)

1. `squeue -h` 출력이 빈 것을 두 번 확인한다(60 s 간격). `scontrol show node ws1 | grep -E "CfgTRES|AllocTRES"`.
2. qemu 0: `pgrep -c qemu` → 0(총괄 지시 형태).
3. GPU persistence 2/2: `nvidia-smi --query-gpu=index,persistence_mode --format=csv,noheader` 가 두 줄 모두 Enabled.
4. vm_watch 켬: S3 때의 VM 상태 기록기(`vm_watch_s3.py sample <jsonl> --every 30`)를 두 팔이 끝날 때까지 돌린다.
5. 런처 자체 preflight(Isaac 없이). 스크래치 출력에 env.sh 등을 쓴다:
   `DRY_RUN=1 START_RELOC=off MAP_BUNDLE=registry MAX_EPOCHS=1 STANDBY_LINGER_S=0 EXTRA_ARGS="<팔 값>" ... bash deploy/console/isaac_live.sbatch <scratch>`
   (isaac_live.sbatch:464-490). 다음을 확인한다:
   - `SLAM_CHOICE localization (registry: random_m_team_spec_quantized_0facac3eea66)`
   - `ARGS_OK`
   - RUNNER_ARGS 에 B 만 `--path-display off` 가 있음
   - `SCAN_SWITCHES`·`RIIB_DEFAULTS` 줄이 두 팔에서 같음
6. 실행 뒤 `sacct -a -X -S <시작> -E <끝>` 으로 다른 잡이 겹치지 않았는지 확인한다.

## 5. 실행 뒤 읽는 것

`RUN=OUT/epoch_1/run`(실제 폴더는 `find OUT -maxdepth 1 -name 'epoch_*'` 로 확인하고, 하나가 아니면 6 절대로 무효).

- 캡처: `tail -n1 OUT/frames/index.jsonl` 의 `reason`(기대 `runner_ended`), `kept`, `reconnects`, `silent_reconnects`.
- 가시성(두 팔 함께):
  `"$VENV_PY" tools/live_frames.py judge A/frames --compare B/frames --per-frame > s4_judge.json`
  색별·팔별 화소 수와 성분 수를 본다(tol 40, min-component 50 기본, live_frames.py:1031-1038).
  - 읽는 법: B(off)는 같은 장면·같은 색 문턱의 대조군이다. 색 화소는 배경의 비슷한 색도 센다. 그래서 "보인다"는 A 의 값만으로 말하지 않고 A 와 B 를 나란히 적는다.
  - 색별로 A 에서 화소 수가 가장 많은 프레임 3 장과, 같은 미션 경과 시각의 B 프레임 3 장을 사람이 눈으로 보고, 선이 보이는지 보고서에 적는다(파일 이름을 남긴다). 색 화소 수는 이 확인의 보조 지표다.
    - A 의 3 장: `s4_judge.json` 의 `arm.per_frame[]` 에서 그 색의 `colours.<색>.pixels` 가 큰 순서(같으면 `file` 이름 순).
    - 미션 경과 시각: 각 팔에서 `index.jsonl` 의 frame 행을 `file` 로 이어 `stamp_sim_s` 를 얻고, 그 팔의 미션 기준 시각(`recv_mono_ns / 1e9` 가 `probe.json` 의 `missions_run[0].submitted_mono` 이상인 첫 frame 의 `stamp_sim_s`)을 뺀 값이다.
    - B 의 3 장: A 의 각 프레임과 미션 경과 시각이 가장 가까운 B 프레임(같으면 앞의 것). 두 팔의 미션 진행 속도가 달라 같은 경과 시각이 같은 장면이라는 보장은 없다 — 고른 B 프레임의 경과 시각 차이를 함께 적는다.
- 어긋남(A 만):
  `"$VENV_PY" tools/live_frames.py delta --run A/epoch_1/run --frames A/frames > A_delta.json`
  - 기본 설정 0 과 11/120, 가정 지연 11/120(live_frames.py:112, 1049-1052).
  - `settings.<lag>.{all,straight,turning}` 의 max·p95·중앙, `no_visible`, `counts.skipped` 를 본다.
  - 표본 충분성(미리 정함): 선이 보인 프레임 수를 straight·turning 별로 적는다(turning = \|yaw rate\| ≥ 0.05 rad/s, live_frames.py:113). turning 에서 선이 보인 프레임이 30 장 미만이면 turning 값은 "표본 부족" 으로 적고 지연 선택의 근거로 내지 않는다. 한 미션(P-1:D-1)이 회전·근거리 구간을 대표한다는 보장은 없다 — 더 많은 seed·접근 조건이 필요한지는 이 수를 보고 총괄이 정한다.
  - B 는 off 에서 머리 줄만 쓰이므로(#513) 표시 행이 없어 "no path" 로 exit 2 가 난다(live_frames.py:743-744). B 의 delta 는 계산하지 않는다.
- result.json:
  - `success`, `failure_reason`
  - `arguments.path_display`: A `overlay`, B `off`(run_transport.py:11918)
  - `arguments.path_display_pixel_lag_s`: 없음 = 0(11920)
  - `live_video_camera.intrinsics`, `perception_mount`, `scan_switches`, `tick_trace`
  - 같은 폴더의 `exit_code`
- `RUN/path_display.json`: compose_failures, note_failures, clock_mismatches, event_failures, last_error, pixel_lag_s, event_rows_pending(path_overlay.py:42, 143-145).
- `RUN/path_display.jsonl`:
  - 머리 줄의 `mode`·`pixel_lag_s`(path_overlay.py:77-78)
  - name 별 display 행 수
  - `time_s` 가 null 인 행 수
- 탐침 `OUT/probe.json`:
  - `missions_run[0].submit.t`(ack), `missions_run[0].result.outcome`
  - `session_end`, `close_code`, frames 의 `bytes`
- 타이머·페이스(status 줄, live_runtime.py:564-575; 값은 [wall s, calls, max s, cpu s], perf.py:202-208):

```bash
"$VENV_PY" - "$RUN" <<'EOF'
import sys, json, collections
from pathlib import Path
sys.path.insert(0, "src")
from forklift_core.live.spool import read_live_log
tot = collections.defaultdict(lambda: [0.0, 0, 0.0, 0.0]); rf = []; stalls = None
for e in read_live_log(Path(sys.argv[1])):
    if e.get("status"):
        p = e.get("pacing") or {}
        if p.get("realtime_factor") is not None: rf.append(p["realtime_factor"])
        stalls = p.get("stalls", stalls)
        for k, v in (e.get("timers") or {}).items():
            t = tot[k]; t[0] += v[0]; t[1] += v[1]; t[2] = max(t[2], v[2]); t[3] += v[3] or 0.0
keep = {k: v for k, v in sorted(tot.items()) if k.startswith(("step.", "overlay.", "isaac.step"))}
rf.sort()
print(json.dumps({"timers": keep, "status_lines_with_factor": len(rf),
                  "factor_median": rf[len(rf)//2] if rf else None, "stalls_last": stalls}, indent=1))
EOF
```

- `step.render`·`step.scan`·`step.plain` 은 step_world 만 잰다(run_transport.py:8770-8787).
- `overlay.compose` 는 live_camera() 안에서 잰다(run_transport.py:8510; LiveRuntime._camera 에서 불림, live_runtime.py:693-694, 717). 그래서 `step.*` 밖이다.
- `overlay.note_moving` 은 step_world 안(run_transport.py:8177, 물리 스텝 직후)에서 불리므로 `step.*` 안에 들어 있다.
- 비용 읽는 법: 두 팔을 한 번씩, A → B 순서로만 돈다. GPU·셰이더 캐시, 온도, 실행 시점이 팔과 섞이므로 A−B 차이는 탐색적 관찰값으로만 보고한다. 원인으로 단정하지 않고, 총괄이 필요하면 반복이나 B → A 순서를 추가로 정한다.
- tick_trace.npz 는 필요할 때만 읽는다.

## 6. 무효 조건(값과 상관없이 그 실행을 쓰지 않는다)

- 런처 preflight 가 거부했거나, supervisor 의 epoch 가 1 개가 아니거나, 마지막 exit ≠ 0 이다(`OUT/supervisor.jsonl`, `launcher.log`).
- `OUT/stall.jsonl` 이 있다(멈춤 감시가 끝냄).
- 미션 제출이 ack 가 아니다. 거부 detail 이 `pacing` 이거나, 재시도한 뒤에도 not_stopped 인 경우를 포함한다.
- 미션 결과가 오기 전에 세션이 끝났다(`missions_run[0].result` 가 없거나 `result_not_this_mission`).
- `arguments.path_display` 나 path_display.jsonl 머리 줄의 mode 가 정한 값이 아니다.
- SLAM_CHOICE 가 registry 위치 추정이 아니거나, START_MODE 가 known 이 아니다.
- 캡처가 runner_ended 로 끝나지 않았거나, kept 가 0 이다.
- 캡처의 첫 frame 행이 미션 제출보다 늦다: `index.jsonl` 첫 frame 의 `recv_mono_ns / 1e9` > `probe.json` 의 `missions_run[0].submitted_mono`(둘 다 같은 노드의 CLOCK_MONOTONIC).
- 탐침이 미션 제한시간으로 끝났거나, 잡이 Slurm TERM 을 받았다(`s4_wrapper.log`·`probe.log`).
- 실행 중에 다른 Slurm 잡이 겹쳤거나, persistence·VM 상태가 바뀌었다.
- 미션이 실패한 것 자체는 무효가 아니다. 값과 함께 보고한다.

## 7. 제출 전 확인 결과(2026-11-15, 콘솔 세션이 직접 확인)

- 스냅숏: 이전 배치와 같은 방식 — 이 브랜치의 커밋(origin/main `23b858b3` 이후에 이 문서와 래퍼만 더한 것)을 `git archive HEAD` 로 ws1 `snapshots/s4ov_<커밋 8자>` 에 풀고, 같은 이름이 있으면 거부(`test ! -e`). 러너·런처 파일은 main 과 바이트가 같다(이 브랜치는 그 둘을 바꾸지 않는다). 두 팔이 같은 스냅숏을 쓴다.
- ws1 경로(앞선 B-P2e 스모크 `test_runs/live/bp2e_smoke/smoke_registry.sh` 와 같은 값, 존재 확인): `BASE_SCENE=$A/base_scene/scene.usda`, `FORKLIFT_URDF=$A/dd_cfg/old/dls08_measured/forklift.urdf`(`A=/home/projects/forklift/artifacts/20261013_fork_spacing`), `VENV_PY=/home/projects/forklift/.venv/bin/python`, `SLAM_BUNDLE_ROOT=/home/projects/forklift/artifacts/slam_bundles`(`random_m_team_spec_quantized_0facac3eea66` 있음), `SCENE_FILE=/home/projects/forklift/test_runs/riibotics/scenes_ct1_a770031/random_m_session.json`(sha256 `8b8fe554…` 가 registry 와 같음).
- 장면 random_m 에 P-1~P-5, D-1~D-3 이 있다 → 미션 `P-1:D-1` 가능. random_m 은 registry 에서 `start_relocalizable: true` 지만 이 실행은 `START_RELOC=off`(known start)라 무관하다.
- 장면 파일 세션은 `--near-capture-redraw always` 가 있어야 러너가 받는다(B-P2e 스모크와 같음) — 래퍼의 두 팔 EXTRA_ARGS 에 넣었다.
- vm_watch: 콘솔 세션의 VM 상태 기록기(S3 때 쓴 `vm_watch_s3.py sample <jsonl> --every 30`, 30 s 마다 ws1 의 VM·qemu 상태)를 두 팔 동안 켠다.
- 라이브 카메라의 자세 이력: 비결정 실행에서 라이브 카메라가 움직이는 카메라로 등록되고 매 물리 스텝 뒤 기록된다(정적 검토, run_transport.py 의 등록·note_moving 호출). 이 실행은 픽셀 지연 설정을 0 으로 두므로 결과에 영향이 없다.
- off 모드에서도 `path_display.jsonl` 은 헤더만 쓴다(#513, 시뮬-물성 확인) → B 팔 delta 는 표시 행이 없어 계산하지 않는다(judge 비교용).
- 정한 것(콘솔): 두 팔 모두 `--max-sim-seconds 1200`·틱 추적, 순서 A → B 고정(순서 효과는 결과에 적음), 탐침·캡처가 잡의 CPU 4 개를 세션과 함께 쓰는 것을 두 팔이 똑같이 받음, 픽셀 지연 11/120 을 켠 세 번째 팔은 이번에 두지 않음(오프라인 delta 로 보고, 필요하면 총괄이 정함).
- 남는 것: 미션 하나짜리 세션의 실제 길이(예약 1:15 는 sim 최대 1,200 s 를 factor 0.4 로 3,000 s + 기동으로 잡음)와 JPEG 크기(10 Hz × 수십 KB × 길이)는 실행에서 잰다.

## 8. 자체 검토

- ① 원자료 재확인(콘솔): ws1 경로·장면 sha·registry 묶음·P-1/D-1·`--near-capture-redraw always` 요구를 직접 확인했다(7 절). 초안은 claude-credit(opus)이 썼고, 7 절과 이 절은 콘솔이 고쳤다.
- ② Codex 적대적 검토 1 회차(2026-11-15):
  - P1 "perception 카메라 누출을 이 실행으로 판정할 수 없다" → 맞다. 그것은 S1 의 몫이라 1 절에 범위 밖으로 적었다.
  - P2 캡처 시작과 미션 제출 순서가 보장되지 않음 → 래퍼가 캡처의 첫 frame 을 본 뒤에 탐침을 띄우게 바꾸고, 6 절에 첫 frame 이 제출보다 늦으면 무효를 더했다.
  - P2 색 화소만으로 표시선을 확정하지 못함 → B 대조와 사람의 눈 확인(색별 6 장)을 5 절에 정하고, 화소 수는 보조 지표로 낮췄다.
  - P2 A → B 한 번씩에는 순서·캐시가 섞임 → 비용 차이는 탐색적 관찰값으로만 보고한다(5 절).
  - P2 예약 시간이 미션 한도를 보장하지 않음 → 예약 근거를 2 절에 적고, 미션 한도·TERM 으로 끝난 실행은 무효로 했다(6 절).
- ② Codex 2 회차: P2 "눈 확인 프레임을 judge 기본 출력으로 고를 수 없음" → `--per-frame` 을 명령에 넣고, A 상위 3 장과 B 짝(미션 경과 시각 최근접)의 선택 규칙을 5 절에 정했다. 새 P1 없음.
  - 확인된 것: `--path-display off`·`--near-capture-redraw always`·`--max-sim-seconds` 철자, EXTRA_ARGS 전달 경로, delta 조인 키, B 의 머리 줄만 있는 기록 처리.
- ② Codex 3 회차: 새 P1 없음. P2 로 독립 검토가 빠졌다고 지적 → ③ 을 했다.
- ③ 독립 검토(앞의 검토를 보여주지 않은 새 Codex 스레드, "적히지 않은 전제"): P2 다섯·P3 하나.
  - 한 미션이 회전·근거리를 대표하지 못할 수 있음 → turning 표본 30 장 문턱과 "표본 부족" 처리를 5 절에 정했다. seed 추가는 총괄 결정으로 남겼다.
  - 세션 하나씩의 비용 비교, 캡처·탐침이 같은 CPU 를 씀, 캡처의 I/O 가 pacing 을 건드릴 수 있음 → 결과 범위를 "캡처를 켠 조건, 팔마다 한 세션, 탐색적" 으로 1 절에 적었다. 캡처 없는 팔·순서 교차는 더하지 않았다(총괄이 필요하면 정한다).
  - 캡처 스트림 ≠ 운영자 화면 → 가시성을 "media socket 스트림에서" 로 한정했다(1 절).
  - delta 의 카메라 자세는 재구성이고 참 지연은 가정값(P3 포함) → 모형 기반 추정이라고 적고, 재구성 자세 오차는 재지 않았다고 밝혔다(1 절).
