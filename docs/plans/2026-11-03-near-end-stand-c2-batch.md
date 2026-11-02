# C2 검증 배치 등록 — 근접 재관측 다리 끝 정지 안전망 (2026-11-03, 1 판, 결과 전)

계획: `docs/plans/2026-11-02-near-leg-end-stand-c2.md`(PR #262, Codex 수렴). 구현: PR #264(main cbec36ba), 러너 `--near-end-stand off|shadow|on`. 이 등록은 계획 §5 의 승격 조건을 결과 전에 집계 규칙으로 고정한다. 런처 기본을 켤지는 총괄이 정하고, 이 배치는 그 입력이다.

## 1. 무엇을 정하는가

- riib 런처 기본에 `--near-end-stand on` 을 넣을지 판단할 입력(계획 §5 의 네 조건). 통과·미달·판정 불가를 총괄에 보고한다. 넣는 PR 은 총괄 결정 뒤에 낸다.

## 2. 팔

- 켬: `--near-end-stand on`, 경계 기록(`--record-boundary --record-boundary-format 2`, 사건 원인용).
- 그림자: `--near-end-stand shadow`(판정만 기록, 행동은 끔과 같음). 끔 팔 대신이다. 같은 실행 안에서 "발동했을 것" 뒤의 결말을 본다(계획 §5 4).
- 두 팔 모두 riib 런처 기본(#263 의 `RETURN_START=boxed` 포함), `VIDEO=off`, 같은 스냅숏(main cbec36ba 이후).

## 3. 실행 목록

- random_1 seed 0–19 × 두 팔 = 40 실행. seed 별로 켬·그림자를 번갈아 놓는다.
- 생성기 `tss_diag/make_runs_c2.py`. 짝 재실행은 회차 `_r1`–`_r5`, 합 4 seed 까지다. 실행 수는 최대 48 이다.
- 노출 예상: S1(10 kg, #252 전)의 random_1 에서 근접 다리 끝 정지로 끝난 실행은 20 중 3(15 %)이었다. 한 팔 20 실행이면 기댓값 3 회다. 질량·코드가 바뀌어 빈도는 S1c 의 random_1 결과로 다시 본다. 설치가 2 회 미만이면 조건 1 은 판정 불가이고, 그때는 주입 배치를 따로 등록한다(계획 §4).
- 제출: 총괄 GPU 순서(f2b → S1c → 시뮬-물성 B → C2)에 따라 `--dependency=after:14719:14720:14721:14722` 를 붙인다. 크기는 `isaac_bundle.py sizes riib --runs 40 --jobs 2` 다. 디스크 세션에 이름·용량·폴더를 알리고 롤링을 맡긴다(성공 실행의 경계 기록은 보관 후 삭제, 실패 실행의 기록·`result.json` 은 판정 끝까지 남김).

## 4. 유효성 (판정 전)

- 인프라 무효는 허용 목록뿐이다(결과 유실, 묶음 `infra_failed`·`stalled`, `slam_link_failed`·`slam_prewarm_timeout`·`plan_worker_dead`, 센서 출력 없음, 준비 단계 실패). 나머지 끝은 모두 그 팔의 결과다.
- 짝은 같은 seed·회차의 두 팔이다. 깨진 짝은 회차 단위로 두 팔을 함께 다시 돈다(seed 오름차순, 합 4 seed). 원래 유효한 짝은 원래 것을 쓴다.
- 최소 유효 짝은 20 중 16 이다. 모자라면 판정 불가다.
- 묶음의 자동 재시도는 회차가 아니다. 앞 시도 폴더는 안전 증거로만 읽는다.

## 5. 판정 (계획 §5, 집계 규칙)

집계 모집단은 채택한 회차의 유효 짝이다. 사건은 `observe_end_stands` 의 `kind: near` 기록이다.

1. **회복(켬 팔):** 설치(`outcome: installed`)가 있는 실행마다, 첫 설치 뒤 미션 결말을 분류한다.
   - `completed`: 완주
   - `capture_refused`: 근접 캡처 채택 전 거부. `near_capture_misaligned`·`near_capture_failed` 이고 `near_capture.status` 가 `done` 이 아님.
   - `restand_timeout`: 관측 단계 `Tracking timeout in observe`
   - 그 밖은 `other` 로 분류해 조건 2 로 넘긴다.
   - 통과: 설치 실행이 2 이상이고, 그중 `completed` 가 2 이상이며 절반 이상이다. 설치 실행이 2 미만이면 판정 불가다.
2. **안전(켬 팔의 설치 실행):** 다음이 모두 0 이다.
   - 금지 접촉·footprint overlap(`failure_reason`·`forbidden_pocket_contacts`·실행 로그)
   - 장면 손상(팔레트 변위)
   - 근접 캡처 채택(`done`) 뒤 `approach`·`insert` 단계 실패
   모든 실행과 앞 시도의 면제 키·접촉도 보고한다.
3. **무회귀:** 켬 완주 ≥ 그림자 완주 − 1 이다. 켬에만 나오고 관측 단계 밖인 새 실패 유형이 0 이다.
4. **불필요 발동(그림자 팔):** `outcome: shadow` 사건마다 결말을 본다.
   - 사건 자기 기록의 `end`(`loose_arrival`·`command_resumed`·`tracker_changed`·`status:*`)를 그대로 쓴다.
   - 열린 채 끝났으면 `timeout` 으로 본다. 조건은 셋이다. 판정 뒤 재계획이 없고(기록의 `observe_replans_n` = 실행의 최종 개수), `near_capture` 가 아직 `pending` 이며, 실행이 관측 단계 `Tracking timeout in observe` 로 끝났어야 한다.
   - 그 밖에 열린 채 끝난 것은 `unlinked`(미판정)다.
   - 통과: `loose_arrival` 이 0 이다. 연결된 사건(`unlinked` 가 아닌 것)이 0 이면 판정 불가다. `shadow_no_budget` 는 따로 센다.
- 승격 입력: 넷이 모두 통과하면 통과, 하나라도 미달이면 미달, 그 밖은 판정 불가다.

## 6. 보고 (판정과 별개)

- 팔별 실패 사유 표, 완주 수, 근접 다리 끝 정지 사건 수(켬: 시도·설치·실패·예산 없음 / 그림자: 판정·연결·결말별).
- 설치 사건의 정지 오차 분포, 재계획 경로 길이.
- S1c random_1 의 같은 정지 빈도(합치지 않고 나란히).

## 7. 도구 (결과 전 고정)

- 생성기 `tss_diag/make_runs_c2.py`, 집계기 `tss_diag/c2_sum.py`. md5 는 Codex 수렴 뒤 이 절에 적는다. 합성 자료(로컬 `tmp/rx/c2b/syn`)로 분류·판정을 확인했다.

## 8. 자체 검토

- ① 원자료 재계산: 노출 기댓값(S1 random_1 3/20), 실행 수 40·최대 48, 최소 짝 16.
- ② Codex 등록 검토: 아래에 기록한다.
- ③ 독립 검토: 계획(#262)은 상위 계획(#260)에서 agy 독립 검토를 거쳤다. 이 등록은 계획 §5 를 고정한 것이라 따로 받지 않는다.
