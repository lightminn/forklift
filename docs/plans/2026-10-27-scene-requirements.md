# 장면 요구사항 파일 계획 (2026-10-27, 2 판)

관련: `docs/plans/2026-10-27-ws1-preflight.md`(사전 검사, 결과 판정), 총괄 지시(2026-10-27), 콘솔 답변(용도 프로필), v-g2 자료(trailer·aisle_rack).

## 0. 왜

장면마다 꼭 줘야 하는 러너 인자가 런처 기본값에 흩어져 있다. 빠뜨리면 실행은 정상으로 끝나고 결과만 틀린다.
- **trailer**: 운반 출구 접두가 필요하다(`--exit-clearance-m 0.30`, PR #91). riib 런처는 이것을 기본으로 주지 않는다. 이것이 빠진 배치가 0/10 이 되어 회귀로 오인됐다(82ffffa8). 정정 배치는 trx 11833 이다.
- **aisle_rack**(실제 크기 기둥판): `--lidar-keepout-m 0.15` 가 필요하다. 이 값은 riib 런처만 장면 이름을 보고 넣는다.
- **CT 기준선 비교**: S3·CT-2·CT-1 배치와 비교하는 실행은 같은 FAR+NEAR 집합이어야 한다(콘솔). 이것은 장면이 아니라 용도이므로 고를 때만 대조한다.

장면 JSON 에는 넣지 않는다. 등록부(`config/slam_map_bundles.yaml`)가 장면 파일을 전체 내용 sha256 으로 맞추기 때문이다.

1 판에서 뺀 것: "near_capture_expected on 이고 추적 없음이면 redraw always" 규칙(Codex P2).
- 그 옵션은 후보 선택 제한일 뿐, 근접 재관측이 켜지는 조건이 아니다.
- session 장면은 이미 파서가 `redraw always` 를 강제한다(`arguments()`). d7oor 11728 은 그 규칙으로 새로 막히는 경우가 아니다.
- redraw always 는 ct_baseline 프로필 안에만 둔다.

## 1. 파일

`config/scene_requirements.yaml`(format `scene_requirements_v1`). 키는 argparse dest 다.

```yaml
format: scene_requirements_v1
scenes:
  trailer:
    match: {}                              # 장면 JSON name == trailer 면 적용
    any_of:                                # 출구 접두가 있다: 고정 접두, 또는 P1c 자동 접두
      - {exit_clearance_m: 0.30, lidar_keepout_m: 0.0}
      - {exit_clearance_m: null, lidar_keepout_m: {gt: 0.0}}
    why: >-
      PR #91 출구 접두; 없으면 riib trailer 0/10 (82ffffa8, 정정 trx 11833). 0.30 은 trailer-exit 계획 D1 의
      실험 후보값이다(보장값 아님). keepout 자동 접두(1.5 (G + clearance) + 0.01)는 러너가 지원하는 대안이며,
      trailer 에서 검증되지는 않았다.
  aisle_rack:
    match: {source.upright_keepout_m: 0.0} # 실제 크기 기둥판만(PR #106 넓힌 사본은 keepout > 0)
    any_of:
      - {lidar_keepout_m: 0.15}
    why: "P1c: 기둥은 실제 크기, 계획기 LiDAR 여유 0.15(riib 런처 기본과 같다)."
profiles:
  ct_baseline:
    args: {recognition: far, recognition_proposals: learned, recognition_tau: 0.3, recognition_conf: 0.05,
           recognition_pixel_lag_steps: 12, near_capture_redraw: always, near_capture_redraw_lateral_m: 0.10,
           near_capture_expected: "on", near_capture_lower: waive_far_confirmed}
    recommends: {live_factor: 0.4, encoder_model: team_spec_quantized, slam: localization}
    why: "S3·CT-2·CT-1 과 비교할 때의 live FAR+NEAR 집합(콘솔). riib 배치의 pixel lag 는 24 라 이 프로필이 아니다."
```

- **장면 매칭**: 장면 JSON 의 `name` 이 후보를 고르고(로더가 `record["name"]` 을 바로 읽으니 로딩되는 파일에는 늘 있다), `match` 의 점 경로 값이 모두 같아야 적용한다.
  - 같은 이름 aisle_rack 이 기둥 크기가 다른 두 판으로 존재했다(5125518c, Codex P2). 그래서 `source.upright_keepout_m` 으로 구분한다. 지금 p1c 판은 0.0 이다.
  - 그 키가 없는 옛 사본은 적용하지 않고 "not_matched" 로 기록한다.
- **any_of**: 항목 하나라도 모두 맞으면 통과한다.
  - 값 `null` 은 "주지 않음(None)"을 뜻한다. `{gt: x}` 는 x 초과다.
  - 숫자는 1e-9 허용으로 비교하고, None 과 숫자는 항상 다른 것으로 본다(agy: None 빼기 숫자 크래시 방지).
  - 문자열은 그대로 비교한다. YAML 의 on/off 는 따옴표로 쓰고, 로더가 bool 을 만나면 형식 오류로 거부한다.
- 경로 같은 실행별 값(onnx, providers, device id, threads)은 넣지 않는다.

## 2. 어디서·언제 검사하나

러너 `validate_before_isaac` 에서 검사한다. 실제 실행과 `--check-args` 가 같은 판정을 받는다.
- **새 인자**
  - `--scene-requirements PATH`: 기본은 러너 파일 기준 `<repo>/config/scene_requirements.yaml`, 즉 스냅숏 안의 파일이다.
  - `--requirements-profile NAME`
  - `--scene-requirements-override "이유"`
- **적용 범위**: `--scene-file` 이 있고 운반을 하는 실행이다. 장면이 없는 홀 조사·factory 콘솔에는 장면 요구가 없다(agy 2: 지금 그쪽 장면 필수 인자는 없다. 생기면 `layouts:` 키로 넓힌다).
  - 운반하지 않는 실행(콘솔 쪽 scan_capture: manifest 인자를 복원하고 미션 없이 끝난다)은 그 런처가 `--scene-requirements-override "scan capture: no transport"` 를 넘긴다(Codex P2). 러너가 목적을 추측하지 않는다.
- **판정**
  - 장면 요구가 어긋나면 `require()` 실패다(사전 검사 exit 2). override 이유가 있으면 통과하고 어긋난 내용과 이유를 기록한다.
  - 프로필은 고를 때만 대조한다. 없는 이름은 거부한다.
  - `recommends` 는 경고만 한다(stderr `scene-requirements: warning:` 와 기록). 런처 env 가 아니라 러너가 받은 최종 값(`live_factor`, `encoder_model`, `--slam-choice` 문자열의 localization/mapping)과 비교한다. 래퍼 기본값이 목록 env 에 없어도 거짓 경고가 나지 않는다(agy 5, Codex P3).
- **함께 옮기는 검사**: `--exit-clearance-m` 와 keepout 자동 접두의 병용 거부는 지금 Isaac 시작 뒤(장면 조립 후)에 난다. 필요 여유는 1.5 (G + clearance) + 0.01 이고, G 0.15·여유 0.10 이면 0.385 m 다.
  - 이 검사는 장면과 계획기 설정만 있으면 계산되므로 `validate_before_isaac` 에서 가능한지 구현 때 본다.
  - 장면 조립이 필요하면 옮기지 않고 그대로 둔다. trailer 에서는 병용 조합을 any_of 가 이미 막는다(첫 항목은 keepout 0, 둘째는 exit 없음을 요구하므로 exit 0.30 과 keepout 0.15 를 함께 주면 둘 다 어긋난다).
- **런처 쪽**(agy 3: 러너 검사가 riib 의 trailer 기본을 바로 막으므로 같은 PR 에서 함께 고친다)
  - riib: trailer 에서 다음 조건을 모두 만족하면 `EXIT_CLEARANCE=${EXIT_CLEARANCE:-0.30}` 를 넣고, 값이 있으면 `--exit-clearance-m` 을 넘긴다. aisle_rack 기본 0.15 는 그대로 둔다.
    - 최종 keepout 이 **숫자로** 0 이다(`awk` 로 판정한다. 0·0.0·00 이 모두 허용된 표기다).
    - `EXTRA_ARGS` 에 `--lidar-keepout-m` 이나 `--exit-clearance-m` 가 없다. 뒤에 오는 `EXTRA_ARGS` 가 최종값을 정하므로 그쪽을 우선한다(Codex 2 차 P2).
  - riib·isaac_live 에 `REQUIREMENTS_PROFILE`·`SCENE_REQUIREMENTS_OVERRIDE` env 를 받아 러너 인자로 넘긴다(agy 5).
- **기록**
  - `result.json` state 에 `scene_requirements` 블록을 남긴다: file 경로·sha256, scene name·scene file sha256, matched, profile, required, mismatches, override, warnings.
  - 실행 인자 기록(`recorded_arguments`)에는 세 인자를 **기본값이 아닐 때만** 남긴다(quarter_* 와 같은 방식). 그래야 override 로 돈 기록을 CLI 로 다시 만들 때 override 가 살아난다(Codex P2-4).
  - 근접 계약은 기록 키를 일반 비교한다. 세 키는 물리 조건이 아니라 사전 검사 메타데이터이므로 `RUN_SPECIFIC_ARGUMENTS` 에 넣어 비교에서 뺀다.
    - `POST_MEASUREMENT_ARGUMENTS` 는 등록 기본값과 같은 값만 허용한다. 그래서 override 나 사용자 지정 YAML(파일 해시 비교로 바뀐다)을 통과시키지 못한다(Codex 2 차 P2, CPU probe 로 확인).
    - 실제로 동작을 바꾸는 인자(exit·keepout·인식 인자)는 지금처럼 비교한다.
  - `--scene-requirements` 의 기본 경로는 스냅숏마다 다르므로 기록하지 않는다. 기본이 아닐 때만 남긴다.
- **사전 검사 증명**: `isaac_bundle.preflight_inputs` 에 `config/scene_requirements.yaml`, 새 헬퍼 모듈, 실행 env 가 가리키는 다른 요구 파일을 넣는다(Codex P2-5, agy 4). 검사 중·후 변경과 삭제는 기존 증명 검증으로 잡힌다.

## 3. 시험

- 파일 형식: 모든 키가 러너 파서의 dest 이고, 값이 그 type·choices 를 통과한다. bool 값은 거부한다. match 경로는 점 표기다.
- trailer
  - exit 없음·keepout 0 → `--check-args` exit 2. 사유는 stderr 의 구조화된 줄이다(파일 출력 없음, Codex P3).
  - exit 0.30 → ARGS_OK.
  - keepout 0.15·exit 없음 → ARGS_OK.
  - override 이유 → ARGS_OK, 그리고 stderr 의 override 줄.
- aisle_rack
  - upright_keepout_m 0.0 이고 keepout 0 → 거부, 0.15 → 통과.
  - upright_keepout_m 0.20 사본 → 적용 안 함.
  - 키 없는 사본 → not_matched.
- 프로필: 안 고르면 FAR 없는 실행이 통과한다. 고르고 pixel lag 24 면 거부한다. 없는 이름은 거부한다. recommends 차이는 경고만 낸다.
- 기록: Isaac 없는 단위 시험으로 `scene_requirements` 블록을 만드는 함수를 검사한다. recorded_arguments 는 기본값이면 키가 없고, override 가 있으면 키가 있다. 근접 계약 비교는 옛 기록과 같다.
- 등록부: 장면 JSON 을 읽기만 하므로 sha 가 바뀌지 않는다. 등록부·재생 게이트 시험은 그대로 통과한다. CPU 경계 재생은 `validate_before_isaac` 를 부르지 않는다(Codex).
- 런처
  - riib trailer DRY_RUN(실제 검사, DRY_RUN_CHECK=1)이 0.30 을 넣어 ARGS_OK 가 된다.
  - riib aisle_rack 이 ARGS_OK 다.
  - 시험용 장면 JSON 이 불완전한 기존 riib 시험(DRY_RUN_CHECK=0)은 호환성의 증거로 쓰지 않는다. 실제 p1c 장면 사본으로 검사하는 시험을 따로 둔다(Codex P3).
- 사전 검사: 요구 파일을 고치면 증명 검증이 거부한다.

## 4. 범위 밖

- 장면 JSON 수정, 등록부 형식 변경
- aisle_out 계열과 trailer 변종으로의 확대: 근거가 없다(Codex). 필요해지면 그 비교 결과와 함께 항목을 추가한다.
- recommends 의 강제
- factory·hall 레이아웃 요구

## 5. 결정이 필요한 것

결정 범위 정책상 여유·규칙·런처 기본값은 진행한 뒤 총괄에 보고한다.
- (D1) trailer 를 장면 필수로 두고, any_of 로 keepout 자동 접두도 허용하는 것.
- (D2) riib 런처의 trailer 기본 exit 0.30. keepout 을 줬으면 넣지 않는다.
- (D3) 운반하지 않는 실행은 런처가 override 이유로 밝히는 것. 러너는 목적을 추측하지 않는다.

## 6. 자체 검토

**① 재계산(Claude, 원자료)**
- trailer 0.30: trailer-exit 계획 59 행 "X = 0.30 m 는 이번 장면의 실험 후보값", 82ffffa8 문서와 trx 정정 배치.
- aisle_rack 0.15: riib_scene.sbatch 117 행 `[ "$NAME" = aisle_rack ] && KEEPOUT=${KEEPOUT:-0.15}`.
- 자동 접두 1.5 (G + c) + 0.01: `pallet_mission.keepout_exit_prefix`(G 0.15, c 0.10 → 0.385).
- 장면 `name`: p1c trailer·aisle_rack·line_2_m, ct1 line_m·random_m 모두 있음(ws1 확인).
- aisle_rack `source.upright_keepout_m` = 0.0(p1c_81a007b1).
- 반박 없음.

**② Codex 적대적 검토(1 차)**: P1 없음, P2 5, P3 3. 모두 2 판에 반영했다.
- P2-1 trailer 대안·운반 아닌 실행 → any_of 와 런처 override(D3).
- P2-2 redraw 규칙의 조건 오류 → 규칙을 뺐다.
- P2-3 같은 이름 다른 판 → match.
- P2-4 override 재현 → 기본값이 아닐 때 기록하고 근접 계약의 RUN_SPECIFIC 에 넣는다(2 차에서 POST_MEASUREMENT 안을 고침).
- P2-5 증명 입력 → preflight_inputs.
- P3: recommends 는 최종 값과 비교하고, 시험은 stderr 를 관측하며, 불완전한 장면 시험은 쓰지 않는다.

**③ agy 독립 검토(Gemini 3.8 Flash, 1·2 의 결론을 보지 않음)**: 7 항목.
- 반영한 것
  - 3 런처 배포 순서 → 같은 PR 에서 함께 고친다.
  - 4 증명 입력.
  - 5 프로필 전달 env 와 거짓 경고 → 최종 값 비교.
  - 6 None 비교와 YAML bool.
  - 7 기록 필터.
- 반영하지 않은 것
  - 1 의 이름 변종(aisle_out_rack, trailer_m) 확대: 근거 없음(Codex 와 같은 판단). 이름 복사본이 검사를 건너뛰는 것은 기록(not_matched)으로 드러난다.
  - 2 의 factory 레이아웃: 지금 필수 인자가 없다.

**② Codex 2 차**: P1 없음, P2 2. 모두 반영했다.
- P2: 근접 계약에서 POST_MEASUREMENT 등록으로는 비기본 override·사용자 YAML 이 다르다고 판정된다 → RUN_SPECIFIC 으로 뺀다.
- P2: riib 기본의 `"$KEEPOUT" = 0` 이 문자열 비교라 0.0 을 놓치고, EXTRA_ARGS 덮어쓰기를 보지 않는다 → 숫자 판정과 EXTRA_ARGS 우선. 시험에 0·0.0·00 표기와 EXTRA_ARGS 덮어쓰기를 넣는다.
- 확인된 것
  - 기둥판 구분 키 `source.upright_keepout_m` 은 생성기 15 판 모두에서 맞다. 최초 판은 키 없음·실제 크기, ec227121(PR #106)은 0.20 넓힘, 5125518c 이후는 0.0 이다.
  - START_BAND·ESCAPE_START 는 출구 접두와 충돌하지 않는다.

**② Codex 3 차**: 새 P1/P2 없음. 계획 수준에서 수렴했다(구현·Isaac 검증은 범위 밖).

## 7. 구현

- `sim/isaac/scene_requirements.py`: 파일 읽기·형식 검사(load), 값 비교(holds), 판정(check), 한 줄 설명(describe).
- `config/scene_requirements.yaml`: trailer, aisle_rack, ct_baseline.
- `sim/isaac/run_transport.py`
  - 인자 세 개를 더했다.
  - `scene_requirements_report()` 를 `validate_before_isaac` 끝에서 부른다. 실제 실행과 `--check-args` 가 이 경로를 함께 쓴다.
  - state 의 `scene_requirements` 블록에 판정을 남긴다.
  - `recorded_arguments` 는 세 인자를 준 경우에만 기록한다.
- `sim/isaac/near_tracking.py`: RUN_SPECIFIC_ARGUMENTS 에 세 키를 더했다.
- `deploy/riibotics/riib_scene.sbatch`
  - trailer 기본 `EXIT_CLEARANCE` 0.30 을 넣는다. 단, 최종 keepout 이 숫자로 0 이고 EXTRA_ARGS 에 두 인자가 없을 때만이다.
  - `REQUIREMENTS_PROFILE`·`SCENE_REQUIREMENTS_OVERRIDE` 를 러너에 넘기고 riib_defaults.txt 에 기록한다.
- `deploy/console/isaac_live.sbatch`·`slam_survey.sbatch`: 같은 runner.sh 가 `REQ` 배열로 프로필과 override 를 넘긴다. live 는 env.sh 에 두 값을 적는다.
- `tools/isaac_bundle.py`: SNAPSHOT_INPUTS 에 요구 파일과 모듈을 더했다.
- 시험
  - `tests/unit/test_scene_requirements.py`
  - riib 런처 시험: 기본값 0·0.0·00, keepout·EXTRA_ARGS 우선, 통과 인자, 생성한 trailer·aisle_rack 장면으로 러너 검사
- 남은 것
  - 운반하지 않는 콘솔 scan_capture 런처의 override 는 그 런처(콘솔 담당)에 요청한다.
  - 다른 요구 파일을 `--scene-requirements` 로 주는 실행은 묶음 `check` 가 거부한다. 증명은 스냅숏의 기본 파일만 해시하기 때문이다(Codex 구현 검토 P2). 필요해지면 증명 입력에 그 경로를 넣는 방식으로 연다.
- Codex 구현 검토 1 차: P1 없음, P2 3. 모두 반영했다.
  - riib 의 EXTRA_ARGS 판정을 런처가 러너에 넘기는 것과 같은 단어 분리로 바꿨다(탭·줄바꿈, 글롭 끔).
  - 다른 요구 파일은 check 에서 거부한다.
  - .nan·.inf 는 형식 오류로 거부한다. 그대로 두면 result.json 의 엄격한 JSON 이 깨진다.
  - P3(형식 시험이 argparse type·choices 를 직접 통과시키지는 않음)는 남긴다. 프로필 값이 러너 검사를 통과하는지는 런처 시험이 생성 장면으로 확인한다.
