# C2 검증 배치 등록 — 근접 재관측 다리 끝 정지 안전망 (2026-11-03, 2 판, 결과 전)

계획: `docs/plans/2026-11-02-near-leg-end-stand-c2.md`(PR #262, Codex 수렴). 구현: PR #264(main cbec36ba), 러너 `--near-end-stand off|shadow|on`. 이 등록은 계획 §5 의 승격 조건을 결과 전에 집계 규칙으로 고정한다. 런처 기본을 켤지는 총괄이 정하고, 이 배치는 그 입력이다.

## 1. 무엇을 정하는가

- riib 런처 기본에 `--near-end-stand on` 을 넣을지 판단할 입력(계획 §5 의 네 조건). 통과·미달·판정 불가를 총괄에 보고한다. 넣는 PR 은 총괄 결정 뒤에 낸다.

## 2. 팔

- 켬: `--near-end-stand on`.
- 그림자: `--near-end-stand shadow`(판정만 기록, 행동은 끔과 같음). 끔 팔 대신이다. 같은 실행 안에서 "발동했을 것" 뒤의 결말을 본다(계획 §5 4).
- 두 팔 모두 riib 런처 기본이다. `RETURN_START=boxed` 는 생성기에서 명시로 고정한다(#263). `VIDEO=off`, 같은 스냅숏(main cbec36ba 이후)이다.
- 경계 기록(`--record-boundary --record-boundary-format 2`)은 두 팔 모두 같게 켠다. 사건 원인을 보려는 것이고, 기록 부하를 두 팔에 같게 두려는 것이다(Codex 등록 1 차).

## 3. 실행 목록

- random_1 seed 0–19 × 두 팔 = 40 실행. seed 별로 켬·그림자를 번갈아 놓는다.
- 생성기 `tss_diag/make_runs_c2.py`. 짝 재실행은 회차 `_r1`–`_r5`, 합 4 seed 까지다. 실행 수는 최대 48 이다.
- 노출 예상: S1(10 kg, #252 전)의 random_1 에서 근접 다리 끝 정지로 끝난 실행은 20 중 3(15 %)이었다. 한 팔 20 실행이면 기댓값 3 회다. 그것은 정지 비율이지 설치율이 아니다. 설치율도 15 % 라고 하면 20 회에서 설치 2 회 이상일 확률은 약 82 % 다(Codex). 질량·코드가 바뀌어 빈도는 S1c 의 random_1 결과로 다시 본다. 설치가 2 회 미만이면 조건 1 은 판정 불가이고, 그때는 주입 배치를 따로 등록한다(계획 §4).
- 제출: 총괄 GPU 순서(f2b → S1c → 시뮬-물성 B → C2)에 따라 `--dependency=after:14719:14720:14721:14722` 를 붙인다. 크기는 `isaac_bundle.py sizes riib --runs 40 --jobs 2` 다. 디스크 세션에 이름·용량·폴더를 알리고 롤링을 맡긴다(성공 실행의 경계 기록은 보관 후 삭제, 실패 실행의 기록·`result.json` 은 판정 끝까지 남김).

## 4. 유효성 (판정 전)

- 인프라 무효는 허용 목록뿐이다(결과 유실, 묶음 `infra_failed`·`stalled`, `slam_link_failed`·`slam_prewarm_timeout`·`plan_worker_dead`, 센서 출력 없음, 준비 단계 실패). 나머지 끝은 모두 그 팔의 결과다.
- 짝은 같은 seed·회차의 두 팔이다. 깨진 짝은 회차 단위로 두 팔을 함께 다시 돈다(seed 오름차순, 합 4 seed). 원래 유효한 짝은 원래 것을 쓴다.
- 최소 유효 짝은 20 중 16 이다. 모자라면 다른 조건과 상관없이 먼저 판정 불가다(조건별 결과는 보고만).
- 묶음의 자동 재시도는 회차가 아니다. 앞 시도 폴더는 안전 증거로만 읽는다.

## 5. 판정 (계획 §5, 집계 규칙)

집계 모집단은 채택한 회차의 유효 짝이다. 사건은 `observe_end_stands` 의 `kind: near` 기록이다.

1. **회복(켬 팔):** 설치(`outcome: installed`)가 있는 실행마다, 첫 설치 뒤 미션 결말을 분류한다.
   - `completed`: 완주
   - `capture_refused`: 근접 캡처 채택 전 거부. `near_capture_misaligned`·`near_capture_failed` 이고 `near_capture.status` 가 `done` 이 아님.
   - `restand_timeout`: 관측 단계 `Tracking timeout in observe`
   - 그 밖은 `other` 로 분류해 조건 2 로 넘긴다.
   - 통과: 설치 실행이 2 이상이고, 그중 `completed` 가 2 이상이며 절반 이상이다. 설치 실행이 2 미만이면 판정 불가다.
   - 범위: 생성되는 riib 실행에는 `--near-tracking` 이 없어, 근접 다리와 C2 설치가 실행마다 하나다. 그래서 실행 단위와 설치 사건 단위가 같다. 한 실행에 설치가 2 회 이상 나오면 첫 설치로 합치지 않고 조건 1 을 판정 불가로 둔다(Codex).
2. **안전(켬 팔의 설치 실행):** 다음이 모두 0 이다.
   - 금지 접촉·footprint overlap(`failure_reason`·`forbidden_pocket_contacts`·실행 로그)
   - 공통 실패 분류(`failures.classify`)의 장면 손상(SCENE) 전부: 낙하·미끄러짐·기울기·변위·overlap·접촉(Codex 등록 1 차)
   - 근접 캡처 채택(`done`) 뒤 `approach`·`insert` 단계 실패. 이것은 "그 캡처 자세 때문에"보다 강한 보수적 기준이다. 걸리더라도 C2 의 틀린 확정을 입증한 것으로 보고하지 않는다.
   모든 실행과 앞 시도의 면제 키·접촉·장면 손상도 보고한다.
3. **무회귀:** 켬 완주 ≥ 그림자 완주 − 1 이다. 켬에만 나오고 근접 다리 밖인 새 실패 유형이 0 이다.
   - 근접 다리 실패는 관측 단계에서 `near_capture` 에 `waypoint` 가 있고, 상태가 `pending` 이거나 `failed` 이면서 `near_capture_failed:*` 로 끝난 실패다(근접 인식 실패는 `pending → failed` 로 바뀐다, Codex 등록 2 차).
   - 원거리 관측 실패도 근접 다리 밖이라 센다(Codex 등록 1 차).
4. **불필요 발동(그림자 팔):** `outcome: shadow` 사건마다 결말을 본다.
   - 평가할 수 있는 사건은 같은 정지에서 끝난 것뿐이다. 사건 자기 기록이 `loose_arrival` 이거나, 열린 채 그 다리의 시간 초과로 끝난 `timeout` 이다.
     - `timeout` 의 조건은 셋이다. 판정 뒤 재계획이 없고(기록의 `observe_replans_n` = 실행의 최종 개수), `near_capture` 가 `pending` 이며, 관측 단계 `Tracking timeout in observe` 로 끝났어야 한다.
   - `command_resumed`·`tracker_changed`·`status:*` 는 정지가 먼저 끝난 것이라 미판정이다. 그 밖의 열린 사건도 미판정이다(계획 §5.1, Codex 등록 1 차).
   - 통과: `loose_arrival` 이 0 이다. 평가할 수 있는 사건이 0 이면 판정 불가다. `shadow_no_budget` 는 따로 센다.
- 승격 입력: 유효 짝이 모자라면 판정 불가다. 그다음 넷이 모두 통과하면 통과, 하나라도 미달이면 미달, 그 밖은 판정 불가다.

## 6. 보고 (판정과 별개)

- 팔별 실패 사유 표, 완주 수, 근접 다리 끝 정지 사건 수(켬: 시도·설치·실패·예산 없음 / 그림자: 판정·연결·결말별).
- 설치 사건의 정지 오차 분포, 재계획 경로 길이.
- S1c random_1 의 같은 정지 빈도(합치지 않고 나란히).

## 7. 도구 (결과 전 고정)

- 생성기 `tss_diag/make_runs_c2.py`, 집계기 `tss_diag/c2_sum.py`(배치 스냅숏에서 `PYTHONPATH=src` 로 실행, `failures.classify` 사용). md5 는 Codex 수렴 뒤 이 절에 적는다. 합성 자료(로컬 `tmp/rx/c2b/syn`)로 분류·판정을 확인했다.

## 8. 자체 검토

- ① 원자료 재계산: 노출 기댓값(S1 random_1 3/20), 실행 수 40·최대 48, 최소 짝 16.
- ② Codex 등록 1 차(`tmp/rx/codex_c2b1.out`, 714d4008): P1 없음, P2 네 건.
  - 장면 손상 누락은 SCENE 분류 전체로 바꿨다.
  - 중단된 그림자 사건은 미판정으로 바꿨다.
  - 무회귀 범위는 근접 다리 밖으로 되돌렸다.
  - 최소 짝 미달은 먼저 판정 불가로 했다.
  - 그 밖: 다중 설치 검사, `RETURN_START` 명시, 두 팔 기록, 앞 시도 면제 키 보고.
- Codex 등록 2 차(`tmp/rx/codex_c2b2.out`): 새 P2 한 건. 근접 인식 실패의 종료 상태 `failed` 를 근접 다리로 연결하게 고쳤고, 합성 자료에 실제 형태를 넣었다.
- ③ 독립 검토: 계획(#262)은 상위 계획(#260)에서 agy 독립 검토를 거쳤다. 이 등록은 계획 §5 를 고정한 것이라 따로 받지 않는다.
