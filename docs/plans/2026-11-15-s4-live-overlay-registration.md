# S4 라이브 확인 — 경로 표시 overlay 의 라이브 영상 가시성·어긋남·비용 (2026-11-15, 초안, 결과 전)

- 근거: [경로 표시 누출 수정 계획](2026-11-14-path-display-perception-leak.md) 4 절 S4(총괄 승인). 구현 #512·#513·#515 를 포함한 origin/main `23b858b3`.
- 상태: 초안. 자체 검토(8 절) 전에는 제출하지 않는다. '확인 필요' 항목이 남아 있다.

## 1. 목적

콘솔 라이브 세션에서 아래 세 가지를 잰다. 그 값을 보고 총괄이 `--path-display-pixel-lag-s` 를 고른다. 이 문서는 통과·실패 문턱을 두지 않고 값만 보고한다.

1. 계획 경로·목적지 고리 선이 콘솔 라이브 영상에 보이는가: `tools/live_frames.py capture` + `judge`.
2. 오프라인 어긋남 추정: 픽셀 지연 설정 0 과 11/120 s 에서의 값(`tools/live_frames.py delta`, `RUN/path_display.jsonl` 을 읽음).
3. 비용: `live_log.jsonl` status 줄의 timers(`step.*`, `overlay.compose`, `overlay.note_moving`, `isaac.step`)와 pacing.

## 2. 두 팔과 고정 조건

| 항목 | 값 | 근거 |
|---|---|---|
| 팔 A | `--path-display` 기본(overlay). `EXTRA_ARGS="--max-sim-seconds 1200"` | run_transport.py:963-972 기본 overlay |
| 팔 B | `EXTRA_ARGS="--path-display off --max-sim-seconds 1200"` | 같은 곳. 런처는 stage 만 거부한다(isaac_live.sbatch:87-98) |
| 픽셀 지연 | 두 팔 모두 `--path-display-pixel-lag-s` 를 주지 않는다(기본 0). 11/120 s 는 오프라인 delta 로만 본다 | run_transport.py:973-981, 1677-1678 |
| 스냅숏 | origin/main `23b858b3` 의 스냅숏 하나(이름 예: `snapshots/s4ov_23b858b3`). 두 팔이 같은 것을 쓴다 | 만드는 방법·동일성 대조: 확인 필요(rv3 등록 1.1 방식을 따를지) |
| 런처 | 스냅숏 안 `deploy/console/isaac_live.sbatch` 를 래퍼 `deploy/console/s4_overlay_run.sh` 가 부른다 | — |
| 장면 | `random_m`(registry 행 `random_m_session.json`, sha256 8b8fe554…, `start_relocalizable: true`) | config/slam_map_bundles.yaml:22-35. 대안 line_m(:4-17). 선택은 확인 필요 |
| SLAM | `MAP_BUNDLE=registry`: registry 묶음으로 위치 추정만 한다. 행이 없으면 실행을 거부하고 mapping 으로 대체하지 않는다. `SLAM_BUNDLE_ROOT` 가 있어야 한다 | slam_choice.sh:24-41 |
| 시작 | `START_RELOC=off` → START_MODE "known (START_RELOC=off)" | isaac_live.sbatch:259-270 |
| seed / 배속 | `SEED=0`, `FACTOR=0.4`(런처 기본) | isaac_live.sbatch:339 |
| 미션 | 탐침 cycle 시나리오의 한 다리 `P-1:D-1`, 끝나면 `--end-session` | live_console_probe.py:1142-1444, 2559-2562. random_m 에 P-1·D-1 이 있는지 확인 필요 |
| epoch | `MAX_EPOCHS=1`, `STANDBY_LINGER_S=0` | isaac_live.sbatch:213, 517-519 |
| 영상 | VIDEO 끔. 런처가 `--video` 를 넘기지 않는다 | isaac_live.sbatch:433-450, run_transport.py:948 |
| 원거리 인식 | 런처 기본(`--recognition` 없음 = off, learned 작업자 없음) | run_transport.py:999-1008 |
| 틱 추적 | `FORKLIFT_LIVE_TICK_TRACE=1`. 추적 버퍼 크기 때문에 `--max-sim-seconds 1200` 을 함께 준다(rv3 래퍼 H4 와 같은 이유) | run_transport.py:8903-8911. 총괄이 이 추가를 받을지 확인 필요 |
| 따라잡기·위상 | `FORKLIFT_LIVE_CATCHUP_STEPS`·`FORKLIFT_LIVE_CAMERA_PHASE` 는 설정하지 않는다(0, 런타임 기본) | run_transport.py:8549-8559 |
| 런처 기본(그대로) | ENCODER_MODEL team_spec_quantized, D7_HOLD on, FAR_EXPECTED on, G3 1b·1c·1d 켬, RETURN_START boxed, NEAR_END_STAND on, NEAR_OBSERVE_ARRIVAL 0.05,0.05, LAYER_BUDGET 기본 파일 | isaac_live.sbatch:105-201, runner.sh 410 |
| 자원 | `--gres=shard:1 --cpus-per-task=4 --mem=7G --time=01:15:00 --signal=B:TERM@120` | wiki ws1-isaac.md(단독 live shard:1·CPU 4·7G, ct1_queue 1:15). 런처 기본은 CPU 6·8G·8 h(isaac_live.sbatch:4-7) |
| 실행 순서 | A 를 먼저 낸다. A 가 끝나고 읽은 뒤 점검(4 절)을 다시 하고 B 를 낸다. 노드에 단독으로 돈다 | — |

- CPU 4 의 뜻: 브리지에 물리 코어 하나, 세션에 물리 코어 하나가 간다(isaac_live.sbatch:305-333). 탐침과 캡처는 따로 묶지 않아 잡의 CPU 4 개를 함께 쓴다. 두 팔이 같은 조건이다. 총괄이 받을지 확인 필요.
- 시간: 미션 한도 2,400 s(ct1 값). 미션 하나짜리 세션의 실측 길이는 없다(확인 필요).

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
2. qemu 0: `pgrep -c qemu` → 0. 총괄이 쓰는 형태와 같은지 확인 필요.
3. GPU persistence 2/2: `nvidia-smi --query-gpu=index,persistence_mode --format=csv,noheader` 가 두 줄 모두 Enabled.
4. vm_watch 켬. 명령·위치가 저장소와 내가 읽은 wiki 에 없다. 확인 필요.
5. 런처 자체 preflight(Isaac 없이). 스크래치 출력에 env.sh 등을 쓴다:
   `DRY_RUN=1 START_RELOC=off MAP_BUNDLE=registry MAX_EPOCHS=1 STANDBY_LINGER_S=0 EXTRA_ARGS="<팔 값>" ... bash deploy/console/isaac_live.sbatch <scratch>`
   (isaac_live.sbatch:464-490). 다음을 확인한다:
   - `SLAM_CHOICE localization (registry: random_m_team_spec_quantized_0facac3eea66)`
   - `ARGS_OK`
   - RUNNER_ARGS 에 B 만 `--path-display off` 가 있음
   - `SCAN_SWITCHES`·`RIIB_DEFAULTS` 줄이 두 팔에서 같음
6. 실행 뒤 `sacct -a -X -S <시작> -E <끝>` 으로 다른 잡이 겹치지 않았는지 확인한다.

## 5. 실행 뒤 읽는 것

`RUN=OUT/epoch_1/run`(epoch 번호가 1 부터인지는 확인 필요, `find OUT -maxdepth 1 -name 'epoch_*'`).

- 캡처: `tail -n1 OUT/frames/index.jsonl` 의 `reason`(기대 `runner_ended`), `kept`, `reconnects`, `silent_reconnects`.
- 가시성(두 팔 함께):
  `"$VENV_PY" tools/live_frames.py judge A/frames --compare B/frames > s4_judge.json`
  색별·팔별 화소 수와 성분 수를 본다(tol 40, min-component 50 기본, live_frames.py:1031-1038).
- 어긋남(A 만):
  `"$VENV_PY" tools/live_frames.py delta --run A/epoch_1/run --frames A/frames > A_delta.json`
  - 기본 설정 0 과 11/120, 가정 지연 11/120(live_frames.py:112, 1049-1052).
  - `settings.<lag>.{all,straight,turning}` 의 max·p95·중앙, `no_visible`, `counts.skipped` 를 본다.
  - B 는 표시 행이 없어 "no path" 로 exit 2 가 날 것으로 예상한다(live_frames.py:743-744). off 에서 머리 줄이 쓰이는지는 확인 필요.
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
- `overlay.note_moving` 이 `step.*` 안에서 불리는지는 호출 위치를 찾지 못했다(확인 필요).
- tick_trace.npz 는 필요할 때만 읽는다.

## 6. 무효 조건(값과 상관없이 그 실행을 쓰지 않는다)

- 런처 preflight 가 거부했거나, supervisor 의 epoch 가 1 개가 아니거나, 마지막 exit ≠ 0 이다(`OUT/supervisor.jsonl`, `launcher.log`).
- `OUT/stall.jsonl` 이 있다(멈춤 감시가 끝냄).
- 미션 제출이 ack 가 아니다. 거부 detail 이 `pacing` 이거나, 재시도한 뒤에도 not_stopped 인 경우를 포함한다.
- 미션 결과가 오기 전에 세션이 끝났다(`missions_run[0].result` 가 없거나 `result_not_this_mission`).
- `arguments.path_display` 나 path_display.jsonl 머리 줄의 mode 가 정한 값이 아니다.
- SLAM_CHOICE 가 registry 위치 추정이 아니거나, START_MODE 가 known 이 아니다.
- 캡처가 runner_ended 로 끝나지 않았거나, kept 가 0 이다.
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

(자리. ① 원자료 재확인 ② Codex 적대적 검토 ③ 독립 검토 결과를 여기에 적는다. 2026-11-15 초안은 claude-credit 실행이라 ②③ 을 하지 않았다.)
