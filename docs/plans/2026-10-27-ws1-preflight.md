# ws1 Isaac 제출 전 사전 검사(preflight) 계획 (2026-10-27)

2026-10-27 에 인자·설정 실수로 GPU 를 쥐고 논 잡이 두 번 나왔다.
- **lvram:** #167 이전 스냅숏에 `MAP_BUNDLE=registry` 를 주었다. 그 스냅숏의 `isaac_live.sbatch` 는 등록부를 몰라 `MAP_START_POSE` 필수 검사에서 끝났다. probe 는 포트 파일을 기다렸고, 두 잡이 shard 8·CPU 16 을 11 분 넘게 쥐었다.
- **d7oor 11728:** 세션 장면 파일에 필수인 `--near-capture-redraw always` 가 빠졌다. runner 가 인자 검사에서 세 번 끝났고, shard 2 를 15 분 쥐었다.

PR #172 는 이런 잡을 실행 중에 끝낸다(runner 미기동 600 s, probe 의 `--launcher-pid`). 이 계획은 그 앞단에서 제출 자체를 막는다(총괄 지시).

## 1. 검사할 것(실행 목록의 각 줄)

1. **런처의 필수 변수와 SLAM 모드 해석.** 런처를 `DRY_RUN=1` 로 실제 셸에서 돌린다. `set -u`·`${VAR:?}` 와 `slam_choice.sh` 의 등록부 해석(행 없음·`SLAM_BUNDLE_ROOT` 없음·묶음 폴더 없음)이 실행 때와 똑같이 일어난다. DRY_RUN 은 bridge·supervisor·Isaac·docker 를 띄우지 않는다.
2. **runner 인자.** DRY_RUN 이 만든 실제 runner 인자로 `run_transport.py --check-args` 를 부른다. 이 모드는 Isaac 을 import 하지 않고, 실제 실행이 Isaac 전에 하는 검사를 같은 함수로 한 뒤 `ARGS_OK` 를 찍고 끝난다.
   - `arguments()` 의 argparse 와 그 안의 `parser.error` 계약(세션 장면 파일의 `--live`·`--near-capture-redraw always` 등)
   - settings·URDF·차대 곡률 검사
   - 공통 함수 `validate_before_isaac(args)` 로 뺀다. 실제 실행도 출력 폴더를 만들기 전에 같은 함수를 부르게 바꾼다. 검사와 실행이 갈리지 않게 하기 위해서다.
   - 출력 폴더와 stack 파일은 만들지 않는다. 대신 출력 폴더가 이미 있으면 실패한다(실제 실행의 `mkdir(exist_ok=False)` 와 같은 판정).
   - 이름이 틀린 플래그 문제: `parse_known_args()` 가 받은 나머지(`kit_arguments`) 중 `--` 로 시작하고 `--/` 가 아닌 것이 있으면 검사 모드에서 실패한다. Kit 설정은 `--/…` 꼴이므로 오타 난 우리 플래그(`--recognition-threadz`)를 잡는다. 실제 실행의 동작은 바꾸지 않는다.
3. **SLAM 묶음.** 등록부나 폴더로 정해진 묶음을 `forklift_core.localization.slam_map.load_bundle(folder, expected_map_id)` 로 읽어 map id 를 대조한다. supervisor 가 시작 때 하는 것과 같은 검사다. 결정된 묶음 폴더와 map id 는 확인 파일에 남긴다.
4. **정적 검사(DRY_RUN 이 없는 런처만).** 런처 파일의 `${NAME:?…}` 이름 중, 파일 안에서 대입하지 않고 실행 env 에도 없는 것을 문제로 남긴다. DRY_RUN 이 없는 런처는 이 검사만으로 통과시키지 않는다(지금처럼 "not run").

**범위 밖:** 다음은 Isaac 이 장면을 세운 뒤에야 알 수 있어 미리 잡지 못한다. 이것들은 PR #172 의 실행 중 감지가 맡는다.
- 근접 계약(`check_near_contract`): 장면을 세운 뒤의 카메라·마운트 상태로 판정한다.
- 장면 USD 내용, GPU, docker 이미지

## 2. 확인 파일과 강제

- **확인 파일 `<RUNS>.preflight.json`:** `check` 가 모든 줄을 통과시킨 뒤 쓴다.
  - `list_sha256`: 검사한 바로 그 바이트의 해시다. 목록은 한 번만 읽고, 그 바이트로 검사하고 해시한다.
  - `inputs`: 검사에 쓰인 파일들의 sha256 이다. 실행마다의 래퍼(argv 의 스크립트), 스냅숏의 `deploy/console/isaac_live.sbatch`·`deploy/console/slam_choice.sh`·`deploy/riibotics/riib_scene.sbatch`·`sim/isaac/run_transport.py`·등록부 yaml 을 포함한다.
  - `runs`: 실행별 `{name, slam_choice, map_bundle, map_id}`
  - `checked_at`
- **상속 env 제한:** 런처가 읽는 변수가 목록의 `env` 에 없고 상속 환경에만 있으면 `check` 는 거부한다.
  - 대상은 나열하지 않고 규칙으로 정한다. 그 실행의 래퍼와 스냅숏의 `isaac_live.sbatch`·`slam_choice.sh`·`riib_scene.sbatch` 안에서 `${NAME`·`$NAME` 으로 읽는 이름을 정적으로 모은다. 그래서 `LAG`·`MODE`·`NEAR` 처럼 래퍼마다 다른 변수도 빠지지 않는다(Codex 반례).
  - 시스템 변수(`PATH`·`HOME`·`USER`·`LANG`·`TMPDIR`·`SLURM_*`·`CUDA_VISIBLE_DEVICES`)와 runner 가 실행마다 넣는 것(`SESSION_ID`·`BRIDGE_CPU`·`PORT`·`BUNDLE_*`)은 허용 목록이다.
  - 그래서 확인 파일이 다루는 실행 조건은 목록과 파일 내용뿐이다. 묶음 runner 와 `verify-preflight` 도 시작할 때 같은 규칙으로 거부한다.
- **묶음 runner(`isaac_bundle.py` 본 실행):** 시작할 때 확인 파일을 요구한다.
  - 확인 파일의 `list_sha256` 이 받은 목록 바이트와 같아야 한다.
  - `inputs` 의 파일 해시가 지금과 같아야 한다. 래퍼나 스냅숏이 그사이 바뀌면 거부한다.
  - 어긋나면 exit 2 로 실행을 하나도 시작하지 않는다. 잡은 몇 초 만에 끝난다.
  - `--no-preflight` 로 끌 수 있다. 시험이나, 사람이 이유를 남긴 예외에 쓴다.
- **array(`isaac_array.sbatch`):** 원본 목록 기준으로 검사한다.
  - 원본 목록의 확인 파일을 `isaac_bundle.py verify-preflight LIST --index I` 로 검사한다. 위 조건에 더해, task 가 뽑은 줄이 원본 목록에서 빈 줄을 뺀 I 번째 줄과 바이트로 같은지 본다.
  - 통과하면 runner 를 `--no-preflight --preflight-verified-by <원본 목록 sha256>` 로 부른다. 그 값을 결과에 남기기 위해서다.
  - 예외: array 는 `PREFLIGHT=off`(이유 `PREFLIGHT_WHY` 필수)를 runner 보다 먼저 스크립트에서 처리한다. 그러면 `verify-preflight` 를 건너뛰고 runner 에 `--no-preflight` 를 넘기며, 이유를 task 로그와 결과에 남긴다. `BUNDLE_ARGS=--no-preflight` 는 runner 에만 가므로 array 의 앞 검증을 건너뛰지 못하기 때문이다(Codex).
- 이미 낸 잡은 자기 스냅숏의 runner 를 쓰므로 영향이 없다.

## 3. 어디에 무엇을 넣나

- **`sim/isaac/run_transport.py`:**
  - `validate_before_isaac(args)` 로 공통 검사를 뺀다. `main()` 이 출력 폴더를 만들기 전에 부른다.
  - `--check-args` 를 더한다. 공통 검사, 출력 폴더 존재 판정, `kit_arguments` 판정을 한 뒤 `ARGS_OK` 를 찍고 끝난다.
  - `check_args` 는 `recorded_arguments()` 에서 뺀다. 넣으면 근접 계약의 측정 실행 인자 비교가 깨진다(Codex 재현). 회귀 시험을 둔다.
- **`deploy/console/isaac_live.sbatch`:**
  - `DRY_RUN=1` 이면 `env.sh`·`runner.sh` 를 만든 뒤 bridge·supervisor 대신 묶음 검사(1-3)를 하고, `RUNNER_CHECK=1 runner.sh` 로 `RUNNER_ARGS` 줄과 인자를 찍고 `--check-args` 로 실행한다. 그 종료 코드로 끝난다. docker·포트 파일은 만들지 않는다.
  - DRY_RUN 이 아니어도 bridge 를 띄우기 전에 같은 검사를 한 번 한다. 그러면 런처를 직접 sbatch 한 단독 잡도 몇 초 안에 실패한다.
- **`deploy/riibotics/riib_scene.sbatch`:** 기존 DRY_RUN(인자만 출력, exit 0)에 `--check-args` 실행을 더해 `ARGS_OK` 를 낸다. 실행 시 자체 검사도 같이 넣는다.
- **ws1 콘솔 래퍼 `ct1_queue.sbatch`·`d7_oor.sbatch`(콘솔 세션 소유, 저장소 밖):**
  - `DRY_RUN=1` 이면 probe·sampler 를 띄우지 않고 `DRY_RUN=1 bash $ROOT/deploy/console/isaac_live.sbatch "$OUT/live"` 의 종료 코드로 끝난다.
  - 스냅숏의 isaac_live 에 DRY_RUN 이 없으면 exit 2 로 끝난다. grep 이 실패했을 때 정상 실행 경로로 내려가지 않게 하기 위해서다.
  - `_dry_run_launcher()` 가 알아보도록 파일 안에 `DRY_RUN`·`RUNNER_ARGS` 문자열(주석이면 된다)을 둔다.
  - 콘솔에 요청한다.
- **`tools/isaac_bundle.py`:**
  - `check` 가 DRY_RUN 의 종료 코드 0 과 `RUNNER_ARGS`·`ARGS_OK` 줄, 기존 플래그 검사, 상속 env 제한을 모두 요구한다. 통과하면 확인 파일을 쓴다.
  - 새 하위 명령 `verify-preflight` 을 둔다.
  - 본 실행 시작에 확인 파일 검사를 넣고 `--no-preflight` 옵션을 둔다.
- **`deploy/slurm/isaac_array.sbatch`:** task 시작에 `verify-preflight` 를 부른다.

**배포 순서.**
1. 저장소 PR 을 머지한다(runner·런처·isaac_bundle).
2. 콘솔이 래퍼에 DRY_RUN 분기를 넣는다.
3. 그 뒤 새 스냅숏으로 내는 목록부터 check 와 확인 파일이 필요하다.

그 사이에 콘솔 래퍼를 쓰는 목록은 check 가 "no DRY_RUN" 으로 막는다. 그럴 때 우회하는 방법은 둘이다. 묶음은 `BUNDLE_ARGS="--no-preflight"`, array 는 `PREFLIGHT=off PREFLIGHT_WHY=…` 를 쓰고 이유를 남긴다.

## 4. 시험

- **runner:**
  - `--check-args` 정상: `ARGS_OK`·exit 0, 출력 폴더 없음, `isaacsim` import 없음
  - 세션 장면 + `--near-capture-redraw` 누락: exit 2(d7oor 사례)
  - 오타 플래그: exit 2
  - 출력 폴더가 이미 있음: exit 2
  - `check_args` 가 `recorded_arguments` 에 없음. 근접 계약 비교가 `check_args` 때문에 달라지지 않음.
- **isaac_live DRY_RUN:**
  - `MAP_BUNDLE=registry` 에 행이 없는 장면: exit 2
  - 폴더 지정에 `MAP_START_POSE` 없음: exit 2(lvram 사례)
  - 정상: `RUNNER_ARGS`·`ARGS_OK`
  - 가짜 `VENV_PY`·`docker` 로 docker·supervisor 를 띄우지 않는지 확인
- **riib DRY_RUN:** 정상은 `ARGS_OK`, 인자 오류는 exit 2
- **check:**
  - 위 실패가 문제로 나오고 확인 파일이 생기지 않음
  - 상속 env 의 `MAP_BUNDLE`·`LAG`(래퍼만 읽음)·`NEAR`(riib 만 읽음) 거부, 허용 목록 변수는 통과
  - 검사 도중 목록이 바뀌어도 확인 파일은 검사한 바이트의 해시
  - 정상은 확인 파일 생성
- **runner 강제:** 확인 파일 없음·목록 바뀜·래퍼 해시 바뀜·상속 제어 변수는 exit 2·실행 0. `--no-preflight` 는 기존대로.
- **array:**
  - `verify-preflight` 가 맞는 줄·인덱스는 통과
  - 다른 줄(빈 줄 포함 셈)이나 목록 변경은 exit 2
  - `PREFLIGHT=off` 에 이유가 없으면 exit 2, 이유가 있으면 검증을 건너뛰고 이유를 기록

## 5. 한계와 남는 위험

- 근접 계약·장면 USD·GPU·docker 이미지 문제는 미리 잡지 못한다. 이것은 PR #172 가 맡는다.
- 로그인 노드 비용: 실행마다 `.venv` python 으로 run_transport import 와 `load_bundle`(파일 해싱)을 한다. 구현 뒤 실행 수별 시간을 재서 적는다.
- sbatch 는 기본으로 제출 환경을 상속한다(`--export=ALL`). 제어 변수 상속을 거부하는 이유가 이것이다. 그 밖의 변수(PATH 등)는 다르게 상속될 수 있다.

## 6. 자체 검토

**① 원자료 확인(Claude).** 코드 위치를 직접 읽어 확인했다.
- `main()` 의 출력 폴더 생성 → settings·URDF → `SimulationApp` 순서
- `check_near_contract` 가 장면 상태를 쓴다는 점
- `slam_choice.sh` 의 등록부 분기
- `_dry_run_launcher` 의 표시 규칙
- riib DRY_RUN 은 인자만 출력한다는 점

**② Codex 적대 검토.** 1 회차는 P1 0·P2 4·P3 3 이었다. 모두 반영했다.
- P2: array 의 task 목록에는 확인 파일이 없다. → `verify-preflight --index`
- P2: `check_args` 필드가 근접 계약을 깬다. → `recorded_arguments` 에서 빼고 회귀 시험
- P2: 목록 해시만으로는 상속 env·래퍼·코드 변경을 못 잡고, 검사 중 수정도 문제다. → `inputs` 해시, 상속 제어 변수 거부, 한 번 읽은 바이트로 검사
- P2: riib DRY_RUN 과 콘솔 래퍼의 표시 규칙이 빠졌다. → riib 변경과 래퍼 계약·배포 순서
- P3: 옛 자식은 명시적으로 거부, 검사와 실행은 같은 함수, `kit_arguments` 의 오타 플래그

2 회차는 P1 0·P2 2 였다. 둘 다 반영했다.
- P2: 상속 env 제한 대상을 손으로 나열하면 `LAG`·`MODE`·`NEAR` 처럼 빠지는 것이 생긴다(Codex 반례: `LAG=oops`, riib `NEAR=oops`). → 파일에서 읽는 이름을 모으는 규칙과 허용 목록으로 바꿨다.
- P2: array 앞 검증은 `BUNDLE_ARGS=--no-preflight` 로 건너뛸 수 없다. → `PREFLIGHT=off`·`PREFLIGHT_WHY` 를 스크립트 단계에 넣었다.

**③ agy 독립 검토(Gemini 3.8 Flash, 문제 상황만 보여 줌).** 방향은 같았다(런처 자체 방어와 제출 전 게이트).
- 반영한 것: 검사는 스냅숏 안 파일로 한다는 점, 상속 env 오염 위험(→ 제어 변수 거부), 옛 스냅숏에는 정적 검사로 물러나되 통과시키지 않는다는 점
- 반영하지 않은 것: 런처 실패 시 스스로 `scancel` 하는 안. 런처가 실패하면 잡이 끝나고, 래퍼 쪽은 #172 의 `--launcher-pid` 가 맡는다.

## 7. 후속: 끝난 실행의 결과 판정(미션·순회)

사전 검사는 내기 전의 일이다. 그런데 rc 0 으로 끝난 실행이 성공으로 묻히는 일이 있었다.
- lvram 시범의 random_m 세션은 막힌 P-1 을 받아 세 미션이 2 초 만에 모두 실패했는데, 1 분 안에 rc 0 으로 끝났다.
- d7oor 11832 의 순회는 27.1 m 에서 `pallet:P-1` 근접 허용 0 에 막혀 멈췄는데, rc 0 이었다.

그래서 묶음 실행기가 콘솔 probe 보고를 읽어 판정하게 한다(총괄 결정 2026-10-27).

- **보고 위치.** 목록 줄의 `"missions": 경로`(`{attempt}` 치환), 없으면 `retry_move` 폴더의 `probe.json`(`tools/live_console_probe.py --report`). 보고가 없는 배치 실행(riib·게이트)은 지금처럼 rc 로만 판정한다.
- **상태**(rc 0 으로 끝난 실행에만 적용하며, rc 가 0 이 아니면 그 판정을 유지한다)
  - `early_mission_failure`: 미션을 하나 이상 냈는데 done 이 0 이다. 거부된 제출도 done 이 아니다.
  - `tour_incomplete`: 순회의 end 가 `arrived` 가 아니다. 사유로 end, progress_m, 마지막 permit reason 을 남긴다. 실행이 목적을 이뤘는지는 그 실행을 낸 세션이 사유를 보고 판단한다.
  - `no_mission_report`: 목록이 지정한 보고가 없거나, 보고를 읽을 수 없다.
  - done 이 하나라도 있으면 `ended` 이다. 뒤 미션의 실패는 실험 결과이고, 수는 행의 `missions` 에 남는다.
- **집계.**
  - 위 세 상태는 `failed` 로 센다. 재시도하지 않는다(목록이나 장면 문제라 다시 돌려도 같다).
  - summary 에 `statuses`(상태별 개수), `status_line`(한 줄), `early_mission_failures`·`tours_incomplete`·`no_mission_reports`·`missions_submitted`·`missions_done` 를 둔다.
  - 실행기는 끝에 `statuses: ended 4, early_mission_failure 2, ...` 를 stderr 에 찍는다.
  - array 의 `collect` 는 태스크마다 `missions` 를 내고 같은 한 줄을 찍는다. 빠진 태스크는 `missing` 으로 센다.
- **시험.** `tests/unit/tools/test_isaac_bundle_missions.py` 13 건.
- **우회에는 이유를 반드시 남긴다**(총괄 2026-10-27). `--no-preflight` 를 `--preflight-note` 없이(또는 빈 이유로) 주면 실행기가 exit 2 로 거부한다. 이유는 stderr 에 `preflight: off -- <이유>` 로 찍히고, summary 의 `preflight` 에 `off: <이유>` 로 남는다. array 는 이미 `PREFLIGHT_WHY` 를 요구한다.
- **SLAM 소켓 경로 길이.** isaac_live 의 사전 검사(DRY_RUN·실행 전)와 riib 런처(MODE≠truth)가 `<OUT>/…/sock/slam.sock` 이 107 바이트를 넘으면 거부한다. lvram random_m 재실행 11840 이 긴 폴더 이름 때문에 112 바이트가 되어, connect 에서 `AF_UNIX path too long` 으로 세 세션이 15 분씩 쓰고 죽었다. isaac_live 는 epoch 세 자리(`epoch_999`)까지 셈한다.
