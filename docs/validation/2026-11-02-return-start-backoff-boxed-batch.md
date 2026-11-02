# 복귀 출발 후진(막힌 시작 포함) 비교 배치 결과 — F2′′ (2026-11-02)

등록: `docs/plans/2026-11-02-return-start-backoff-boxed-batch.md`(2 판, 결과 전 고정). 배치 `f2b_22eac21d`(스냅숏 = main 22eac21d, 묶음 잡 14210·14211, 98 실행). 집계기 `tss_diag/f2b_sum.py`(md5 5478781e, ws1 잡 14758). 원자료 로컬 사본 `tmp/rx/f2bres/`.

## 판정: 통과 → riib 런처 기본 `RETURN_START=boxed`(slam 모드)

### 유효성 (§4)

- 인프라 무효는 두 팔 모두 0 이다.
- 유효 짝은 aisle 24/24, trailer 5/5, line_2 10/10, random_1 10/10 이다. 깨진 짝이 없어 재실행하지 않았다.

### 노출 (§5)

- 시도·채택은 발동별 `invalid_start` 1/1, `boxed` 1/1 이다.
- 후진·복귀 완료는 2 실행으로 문턱을 넘었다.
  - aisle s2(`invalid_start`, 0.07 m)
  - aisle s12(`boxed`, 0.06 m)
- 두 발동 모두 Isaac 에서 완료가 확인됐다. 조언 중앙값 0.035 m, 최대 0.04 m 다.

### 통과 조건

| 조건 | 결과 |
|---|---|
| 1 켬 팔 `return_home:invalid_start`·`slam_release_replan_failed: invalid_start` | 0 |
| 2 켬 팔 막힌 시작의 `slam_release_replan_failed: no_path` | 0 |
| 3 발동 자리 실패의 `start_backoff` 미기록(결함 후보) | 0 |
| 4 완주 켬 ≥ 끔 | 39 ≥ 39 |
| 5 부호 검정 | b′ 6, c′ 6, p 0.613 |
| 6 안전 | 98 실행 모두 면제 4 키가 0, 접촉 실패 0, 로그 접촉 목록 비어 있음, `unpermitted_dir_entries` 합 0 / 0 |
| 7 켬에만 나온 실패 유형 3 건 | 모두 F2′′ 와 무관(아래) |

조건 7 의 세 건은 세 실행 모두 `return_start_backoffs` 가 없고, 최종 `paths.return_home.start_backoff` 가 없다.
- **aisle s14 · random_1 s3(관측 단계):**
  - `near_capture_failed: opening_width_mismatch` 와 `near_capture_misaligned` 다.
  - 복귀 사전계획(s14 1 회, random_1 s3 2 회)은 후진 없이 직접 성공했다.
  - 그 뒤 팔레트를 들기 전 관측 단계에서 끝나, 해제 재계획·첫 live 복귀에는 이르지 않았다.
  - 즉 "직접 사전계획 성공 뒤, 해제·live 복귀 이전의 관측 실패"다.
- **aisle s17(복귀):** `scene_replan_exhausted:cusp_pose`.
  - 첫 live 복귀는 호출되지 않았다(`grid_planning` 거짓, `live_plans` 빈 목록).
  - 해제 재계획은 219.34 s 에 `replanned` 참·`replan_status` success 였다. 후진 분기를 거친 반환은 성공·실패 모두 `return_start_backoffs` 에 기록되고, 그 기록이 성공 이벤트보다 먼저 남는다. 그래서 기록이 없다는 것과 이 성공 이벤트를 합치면 직접 계획을 반환했다고 볼 수 있다(`pallet_mission.py` 919 행: `invalid_start` 도 막힌 시작도 아니면 직접 계획). 기록 부재만으로 한 논증이 아니다.
  - 끝은 장면 재계획 장치의 예산 소진이다. 운반 188.65 s cusp 부터 쓰였고, 복귀 cusp 235.26·261.18 s, permit_stop 268.30 s 를 거쳐 292.0 s 에 소진됐다. 장면 재계획에는 F2′′ 인자가 넘어가지 않는다.
  - 범위: "켬 실행에서도 후진을 붙이지 않은 직접 계획이었다"는 것까지다. 두 팔의 해제 자세가 달라 "끔과 같은 경로"를 확인한 것은 아니다.

## 보고 (판정과 별개)

- 해제 재계획 실패: 끔 2(`invalid_start` 1 — aisle s2, 막힌 `no_path` 1 — aisle s15), 켬 0. 진단(aisle 10/48)과 견주어 끔 팔은 aisle 24 중 2 다.
- 완주(장면별): aisle 14 → 15, random_1 10 → 9(끔 → 켬). 나머지는 같다.
- 실행당 벽시계 중앙값 472.7 → 641.8 s. 켬 팔에만 경계 기록이 있어 F2′′ 자체의 비용으로 돌릴 수 없다. 세션 VRAM 중앙값 4,040 MiB 로 두 팔이 같다.
- 끔의 막힌 시작 실패는 s15, 켬의 막힌 시작 발동 완료는 s12 다. 그래서 짝별 구제나 회복률을 입증한 것은 아니다(Codex). 노출 문턱은 기능이 실제로 작동해 복귀를 끝낸 사례가 있는지 보는 최소 조건이다.
- 팔별 실패 사유 표는 `tmp/rx/f2bres/f2b_sum_out.txt` 에 있다.

## 처분

- 등록 §1 대로 riib 런처 기본에 `--return-start-backoff boxed` 를 넣는다(`deploy/riibotics/riib_scene.sbatch` `RETURN_START`).
- 배치가 slam 모드였으므로 `MODE=truth` 는 요청하지 않으면 off 로 둔다(`EXIT_EXEMPT` 와 같은 선례). 러너 기본은 off 그대로다.

## 검토

- Codex 판정 상의(`tmp/rx/codex_f2res.out`): 새 P1/P2 없음. 통과와 런처 기본 PR 에 동의했다.
  - 요약 98 행을 재계산했다(유효 49/49, 39 대 39, p 0.61279, 면제 4 키가 모든 행에 있고 0).
  - P3 를 반영했다: 관측 실패 두 건의 표현, s17 의 live 복귀 미호출·해제 성공 이벤트 근거와 범위 한정, 장면별 완주·벽시계·짝별 구제 미입증.
