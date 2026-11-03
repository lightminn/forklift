# C2 주입 배치 등록 (2026-11-03, 1 판, 결과 전)

계획: `docs/plans/2026-11-03-near-end-stand-c2-injection.md`(4 판, PR #268, Codex 4 차 수렴). 구현: 이 PR 이다.
- 러너 `--inject-near-end-detour`
- 추종기 `set_input_offset`
- `sim/isaac/near_end_detour.py`

이 등록은 계획 §3·§4 를 결과 전에 실행 목록과 집계 규칙으로 고정한다.

## 1. 무엇을 정하는가

- 자연 배치(`c2_92cdd84e`, #265)에서 판정 불가였던 C2 §5 조건 1(회복)과, 설치 실행에 대한 조건 2(안전)의 판정 입력이다.
- 통과·미달·판정 불가를 총괄에 보고한다. 런처 기본은 총괄이 정한다.

## 2. 팔과 실행 목록

- 켬은 `--near-end-stand on --inject-near-end-detour`, 그림자는 `--near-end-stand shadow --inject-near-end-detour` 다.
- 두 팔 모두 경계 기록(`--record-boundary --record-boundary-format 2`)을 켜고, riib 런처 기본을 쓴다.
  - `RETURN_START=boxed` 는 생성기에서 명시한다.
  - `VIDEO=off` 다.
  - 스냅숏은 이 PR 의 머지 커밋이다.
- 실행은 random_1 seed 0–19 × 두 팔 = 40 이다. seed 별로 켬·그림자를 번갈아 놓는다.
- 생성기는 `tss_diag/make_runs_cin.py` 다. 자연 배치 생성기 `make_runs_c2.py` 와 다른 점은 다음이다.
  - 이름과 주입 인자
  - 템플릿 행(`epc7_f7c08464/RUNS.jsonl` 첫 행) md5 bff3d6a87f17d1c4492c4223656f17b1 의 확인
  - 이미 있는 실행 목록을 다른 내용으로 덮어쓰지 않음
- 제출한 `RUNS.jsonl` 의 md5 와 스냅숏 revision 은 결과 문서에 적는다.
- 짝 재실행은 회차 `_r1`–`_r5`, 합 4 seed 까지다. 실행 수는 최대 48 이다.
- 노출 기대: 계획 §1.3 굴림에서 38/40 기하가 발동해 범위 밖에 섰다. 팔마다 약 19/20 이다. seed 0 두 팔은 마지막 다리가 전진 0.13 m 라 미발동이 예상된다.
- 제출:
  - 크기는 `isaac_bundle.py sizes riib --runs 40 --jobs 2` 로 정한다.
  - 디스크 세션에 이름·용량·폴더를 알리고 롤링을 맡긴다.
  - 성공 실행의 경계 기록은 보관 후 삭제한다. 실패 실행의 기록·`result.json` 은 판정이 끝날 때까지 남긴다.

## 3. 유효성 (판정 전)

자연 배치 등록(#265) §4 와 같다.
- 인프라 무효는 허용 목록뿐이다. 결과 유실, 묶음 `infra_failed`·`stalled`, `slam_link_failed`·`slam_prewarm_timeout`·`plan_worker_dead`, 센서 출력 없음, 준비 단계 실패가 그것이다.
- 짝은 같은 seed·회차의 두 팔이다. 깨진 짝은 회차 단위로 두 팔을 함께 다시 돈다.
- 최소 유효 짝은 16/20 이다.
- 묶음의 자동 재시도는 회차가 아니다. 앞 시도 폴더는 안전 증거로만 읽는다.
- 모든 실행의 `result.json` 인자에 `inject_near_end_detour: true` 가 있어야 한다. 없으면 집계기가 멈춘다.

## 4. 집계 규칙 (계획 §4 → `result.json`)

기록은 `near_end_injections` 다.
- `events`: `armed`·`not_armed`·`restored`·`judged`·`superseded`·`superseded_after_restore`·`failed`·`arrived`·`left_observe`·`arrived_under_offset`
- `windows.injection`·`windows.install`: 끝이 `null` 이면 기록이 끝날 때까지 열려 있던 창이다.
- `pallet_shift_max_m.injection`·`.install`, `max_path_distance_m`, `watch_last`

용어는 다음과 같다.
- **노출 실행:** 다음을 모두 만족한다.
  - `armed` 가 있다.
  - `judged` 가 있다. 첫 C2 판정이 감시를 닫았다는 뜻이고, 그 전에 추종기 교체·실패·도착이 없었다는 뜻이다.
  - 그 `restored_before` 가 참이다.
  - `arrived_under_offset` 가 없다.
  - 주입 인공물이 아니다.
  - 판정 결과는 켬이면 `installed`·`failed:*`·`no_budget`, 그림자면 `shadow`·`shadow_no_budget` 다.
- **주입된 정지의 사건:** `observe_end_stands` 에서 `kind: near` 이고 시각이 `windows.injection[0]` 이후인 첫 기록이다.
- **회복:** 켬 노출 실행에서 다음이 모두 성립한 것이다.
  - 판정이 `installed` 다.
  - 미션이 완주했고, `near_capture.status` 가 `done` 이다.
  - `near_capture_refused`·`near_capture_history` 가 비어 있다. 재정렬·픽업 재시도 구제가 없었다는 뜻이다.
  - 완주했지만 구제가 있었으면 `rescued_completion` 으로 따로 보고한다.

판정:
0. **주입 인공물:** 무장한 실행에서 다음 중 하나가 났으면 주입 인공물이다. 두 팔 합쳐 3 이상이면 판정 불가다.
   - 주입 창이 열린 채 끝났고, 그 끝이 금지 접촉·overlap·SCENE 실패다.
   - 주입 창 팔레트 변위 최대가 ≥ 0.018 m 다.
1. **노출:** 켬 노출 ≥ 10 이고, 그림자 노출 ≥ 10 이다.
2. **주입 유효성(그림자):**
   - 평가할 수 있는 사건 ≥ 10 이다. 정의는 자연 배치 §5 4 와 같다: 사건 기록이 `loose_arrival` 이거나, 그 다리의 `timeout` 이다(판정 뒤 재계획 없음, `near_capture` `pending`, `Tracking timeout in observe`).
   - 그중 `loose_arrival` 이 2 이하이고 20 % 이하다.
3. **회복(켬):** 다음을 모두 만족하면 통과다.
   - 회복 ≥ 켬 노출의 절반
   - 회복 ≥ 2
   - 그 seed 의 그림자 노출 사건이 `timeout` 인 짝에서 나온 회복 ≥ 2
   - 설치율과 설치 조건부 지표는 보고한다.
4. **안전(켬):** 유효한 켬 실행 중 `installed` 가 있는 모든 실행에서 다음이 모두 0 이다. 짝 채택 여부·회차와 무관하다.
   - 금지 접촉·overlap(`failure_reason`·`forbidden_pocket_contacts`·실행 로그)
   - SCENE 실패
   - 캡처 채택(`done`) 뒤 `approach`·`insert` 실패
   - 설치 창 팔레트 변위 최대 ≥ 0.018 m. 창이 열린 채 실행이 끝났으면, 아직 창에 넘기지 않은 스텝 누적값(`near_end_step_shift_max_m`)도 더한다.
     - 창 경계는 C2 판정 기록으로 복원한다. 감시의 다음 틱 없이 끝난 실행에서는 기록에 창이 닫히거나 열린 표시가 없기 때문이다.
     - 주입 창은 무장 뒤 첫 근접 판정에서 끝난다.
     - 설치 창은 첫 `installed` 에서 열린다.
     - 남은 누적값은 끝에 열려 있던 창에 넣는다. 판정 뒤라면 설치 창에만 넣는다.
- 최종 판정:
  - 4 가 미달이면 미달이다(계획 §4: 0·1·2 와 무관하게 계산해 보고한다).
  - 그다음 유효 짝 < 16 이면 판정 불가다.
  - 0·1·2 중 하나라도 안 서면 판정 불가다.
  - 그 밖에는 3 이 통과하면 통과, 아니면 미달이다.

## 5. 보고 (판정과 별개)

계획 §5 그대로다.
- 발동·미발동 사유, 노출에서 빠진 무장 실행의 사건 순서
- 판정 시각 정지 오차(켬·그림자)와 굴림 예측(옆 57.5 mm, 방위 0.144 rad)·자연 4 건의 대조
- 그림자 짝별 결말, 구제된 완주, 팔별 실패 사유 표, 완주 수, 벽시계, VRAM
- §1.4 범위 문장

## 6. 도구 (결과 전 고정)

- 생성기 `tss_diag/make_runs_cin.py`(md5 f6afae595af62388891de2c23bbe3018).
- 집계기 `tss_diag/cin_sum.py`(md5 a5bb3e91083baf0456dd74e9d8e2d4eb). 배치 스냅숏에서 `PYTHONPATH=src` 로 실행하고 `failures.classify` 를 쓴다.
- ws1 `test_runs/riibotics/tss_diag/` 에 둔다. 로컬 `tmp/rx/cin/` 과 같다.
- 합성 자료 생성기 `tmp/rx/cin/syn_make.py` 의 40 실행 네 경우로 분류와 판정을 확인했다.
  - `base` → PASS: 미발동 짝 1, 구제된 완주 1, 설치 실패 1, 주입 창 변위 인공물 1, 그림자 완화 도착 1.
  - `artifacts_safety`: 주입 창 안 overlap 2 개로 인공물 3 → 판정 불가. 설치 창 변위 0.018 m 로 안전 미달 → 최종 미달.
  - `unpaired`: 그림자가 빠진 seed 의 켬 SCENE 실패 → 안전 미달.
  - `leftover`: 설치 창이 열린 채 캡처 실패로 끝나고 남은 observe 누적 25 mm → 안전 미달.
  - `ended_after_install`: 첫 설치 직후 다음 틱 없이 끝나 설치 창 기록이 없고, 남은 누적 25 mm → 안전 미달(주입 인공물 아님).

## 7. 자체 검토

- ① 원자료 재계산: 노출 기대 38/40(계획 §1.3, `tools/near_end_detour_sim.py` 를 로컬 미러로 다시 돌려 재현), 실행 수 40·최대 48, 최소 짝 16, 변위 문턱 0.018 m(러너 `Pallet displaced before pickup`).
- ② Codex 검토(구현 스레드 01a19f1c, 등록 1 차): P2 둘, P3 하나.
  - P2-1(짝이 깨진 켬 설치 실행이 안전에서 빠짐) → 모든 유효 켬 실행.
  - P2-2(종료 시 넘기지 않은 스텝 누적값 무시) → 열린 창에 합산.
  - P3(템플릿·RUNS 미고정) → 템플릿 행 md5 확인, 덮어쓰기 거부, 제출 RUNS md5·스냅숏 기록.
  - 반박하지 않은 것: 최종 판정 순서, `restored_before`, 18 mm 경계, timeout 연결, 구제 기록, timeout 짝 요건, 도구 md5.
- Codex 등록 2 차: P2 하나(설치 직후 종료로 설치 창 기록이 없을 때 남은 누적값이 주입 창으로만 감) → 판정 기록으로 창 경계 복원.
- ③ 독립 검토: 계획(#268)이 agy 독립 검토를 두 번 거쳤다. 이 등록은 계획 §4 를 고정한 것이라 따로 받지 않는다.

## 8. 구현 검증 (이 PR, d9122892)

- 재생 게이트: runs29 `SETUP_INPUTS=recorded` 29/29 다.
  - 결과는 `test_runs/replay_gate/cin_rec_d9122892` 에 있다.
  - 25·26 번은 재부팅으로 끊겨, `cin_rec_d9122892_b` 에 다시 돌렸다.
- ws1 전체 pytest: 8,446 통과, 74 건너뜀, 10 실패다. 실패는 알려진 목록뿐이다(preview_docking 6, evaluate_cli 1, remote_model_jobs 1, diagnose_detection 2).
- 주입 smoke 짝: random_1 seed 1, 경계 기록이다. 위치는 `test_runs/riibotics/cin_smoke_d9122892` 다.
  - 켬은 `armed`(마지막 다리 2.03 m, s = +1) → `restored`(남은 0.101 m) → `judged` `installed` 로 이어졌다.
    - 정지는 0.0578 m · −0.143 rad 였다.
    - 캡처가 채택됐고(`done`) 완주했다.
  - 그림자는 `armed`(단일 다리 2.01 m) → `restored` → `judged` `shadow` 로 이어졌다.
    - 정지는 0.0577 m · −0.147 rad 였다.
    - 그 다리의 `Tracking timeout in observe` 로 끝났다.
  - 두 팔 모두 비킴 중 차–원래 경로 거리 최대는 0.043–0.044 m, 팔레트 변위는 0 이었다.
  - 굴림 예측(옆 57.5 mm, 방위 0.143–0.144 rad)과 같다.
- smoke 두 기록의 format-2 재생: 2/2 통과했다(`replay_gate/cin_smoke_rep_d9122892`, strict).
- Codex 구현 검토 4 차(스레드 01a19f1c)에서 수렴했다.
  - 1 차 P2: 캡처 스텝 변위, 판정 뒤 표본, baseline 복원 뒤 감시
  - 2 차 P2: 판정 뒤 물리 스텝, format-2 재생의 스텝 값, 미션 종료 뒤 표본
  - 3 차 P2: 캡처와 approach 전이가 같은 호출에 있을 때
