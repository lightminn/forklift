# 리보틱스 장면의 삽입 직전 재관측 — Isaac 검증 (2026-10-22)

계획: `docs/plans/2026-10-22-near-recapture-scenes.md`(3판 + 작업 C). 브랜치 `feat/near-recapture`.

## 조건

- ws1 Isaac 5.1, `test_runs/riibotics/riib.sbatch` 설정(`--recognition far` 학습 제안, 밑판 검사 끔, 근접 추적 끔, 정답 대신 SLAM 추정 제어)에 이번 인자만 더했다.
  - `riib2.sbatch`(A·B·D): `--near-capture-redraw always --near-capture-redraw-lateral-m 0.10 --near-capture-expected on`. 스냅숏 `nrc_be901a0`, 태그 `n2_*`.
  - `riib3.sbatch`(A·B·C·D): 위에 `--near-capture-lower waive_far_confirmed`. 스냅숏 `nrc_e083206`, 태그 `n3_*`.
- 장면 파일은 `scenes_riib_b35fcc9` 를 그대로 복사했다(장면 생성 도구는 이 브랜치에서 바뀌지 않았다).
- 결과 폴더: ws1 `test_runs/riibotics/{n2,n3}_<장면>_s<seed>/`.

## 근접 캡처 결과

8주차 실패(`r4_*`, `r5_aisle_out`)와 같은 장면·seed:

| 장면 | 8주차 | n2 (A·B·D) | n3 (A·B·C·D) |
|---|---|---|---|
| line_1 s0 | `no_opening_pattern`(lower 4) | `no_opening_pattern` | **수락, 면제 적용** — 사이클 완주, 하역 5.6 cm |
| line_2 s0 | `no_opening_pattern`(lower 14) | 수락 — 완주, 하역 4.8 cm | 수락(면제 불필요) — 운반 끝 정체 |
| random_1 s0 | `near_capture_misaligned`(6.7 cm·0.066) | 근접 다리 cusp 정지(아래) | **수락, 측면 0.10 m 재작도** — 완주, 하역 6.6 cm |
| random_1 s1 | `near_capture_misaligned`(9.8 cm·0.105) | 수락 — 완주, 하역 4.0 cm | — |
| aisle_out s0 | 옆 칸(0.94 m) 선택 → `approach:invalid_goal` | **진짜 팔레트 선택**(범위 안 1·밖 6) — 들기 기울기 | 오도메트리 변경 뒤 재측정(아래) |

회귀·확장(n3): line_1 s1, random_2 s0, line_3 s0, random_3 s0 모두 근접 캡처 수락, 면제·재작도 불필요.

- n3 7 건 전부 근접 캡처가 수락됐다. 금지 접촉 0, 포크는 7 건 모두 중심을 11.8–24.4 mm 넘었다.
- 근접 추정의 truth 거리(최종 계획 목표 기준): 0.008–0.047 m(SLAM 오차 포함).

### C(면제) 적용 사례 — n3 line_1 s0

- 첫 검출(0.20 m, lower 문턱 유지)은 `no_pallet:no_opening_pattern`.
- 조건 1: 원거리 시도 1 의 검출기 관측 `valid`, 면제·예상 위치 제한 없음, 검출기 위치와 채택 원거리 추정의 거리 기록(`lower_waiver.far_link_m`).
- 조건 2: 반경 0.15 m·15° 의 두 번째 검출 `valid`.
- 같은 장면의 n2 저장 프레임을 두 번째 검출로 재생하면 후보 1 개가 예상 위치에서 0.046 m, truth 와 0.6 mm·0.4 mrad 다(Codex 도 같은 값을 재현).

### D(측면 상자) 적용 사례 — n3 random_1 s0

- 근접 정지 offset 측면 −8.6 cm·yaw 0.090 rad → 재작도 `accepted`, `max_lateral_m` 0.10 기록.
- 두 시운전 모두 `arrived`, 끝 앞 2.5 mm·측면 0.1 mm·yaw 0.3 mrad, 궤적 충돌 검사 통과(`swept_clear`).
- 근접 캡처부터 삽입 끝까지 SLAM hold(`observe hold` → `transport release`).
- 삽입 끝 제어 추종 오차(제어 자세와 계획 경로 끝, `insertion_error`) 6.8 mm·1.2 mrad. truth 기준으로는 포크 중심 넘김 11.8·12.7 mm, 삽입 yaw 2.43 mrad, 포켓 측면 최소 여유 8.5 cm, 금지 접촉 0.
- 접근 궤적의 장애물·팔레트 최소 여유는 따로 계산하지 않았다(포켓 측면 여유만). 기본 속도에서만 확인했다. 계획 4절의 재작도 판정 항목 중 이것과 경계 기록 재생(이 실행들은 경계 기록 없음)은 남아 있으므로, B·D 를 장면 기본 설정으로 채택하지 않고 연구용 선택 인자로 둔다.

## 근접 캡처 뒤에 남은 실패 (이번 범위 밖)

| 유형 | 실행 | 내용 |
|---|---|---|
| 근접 다리 cusp 정지 래치 반복 종료 | n2 random_1 s0 | cusp 에 0.070 m·0.062 rad 모자라 섰다. cusp 정지 복구가 래치를 19 번 걸었고 그중 18 번이 `closed: status:braking` 으로 닫혔다(`stop_confirmed` 17, `observe_rule` 1). 닫히고 다시 잡히기를 되풀이하다 관측 시간 초과. 사용자 결정으로 별도 브랜치에서 고친다. |
| 다리 끝 크리프 | n2 line_1 s1, n3 line_2 s0, n3 line_1 s1 | 운반 다리 끝에서 작은 비영 명령(예: n3 line_1 s1 에서 −0.0039 m/s, n2 line_1 s1 은 −0.0034)과 곡률 −0.36 이 계속 나가 정지 판정의 영명령 누적이 매번 초기화됐다. 기록의 `signed_speed_mps` −0.01212 는 정지 미확인 동안 정지 감지기가 내는 대체값(마지막 명령 방향 × 0.012 × 1.01)이고, 그동안 차는 실제로 움직였다(n3 line_1 s1 마지막 9.9 s 에 truth 27.5 mm). 정지가 확정되지 않아 정체 재계획도 시작되지 않고 운반 시간 초과. |
| cusp 위치 정체 종료 | n3 line_3 s0(복귀), n3 random_3 s0(운반) | cusp 에 0.051–0.064 m 어긋나 섰다. 정지는 확인됐고(`stop_confirmed`), 자세가 판정 범위 밖인 채 대기 시간이 지나 `cusp_position_stall` 로 종료. |
| 들기 기울기 | n2 aisle_out s0 | 포크가 중심에 −2.7·−8.6 mm 못 미쳤다. PR #79 가 진단한 오도메트리 과대계상 계열로 의심하지만 이 실행에서 따로 확인하지 않았다. 다른 세션이 관절각 기반 오도메트리로 바꾸는 중이며, 그 변경이 머지되면 aisle_out 을 다시 잰다(기준선이 바뀐다). |
| GPU 렌더 실패 | n2 random_2 s0 | `run.log` 에 `Failed to import external memory in CUDA`·`Failed to allocate 1280x720 LdrColor resource`, 뒤이어 Slurm 시간 한도로 취소(`riib2-8068.log`). 결과 기록 없음. 당시 GPU 에 다른 잡이 함께 있었다(시작 시 12.1 / 16.3 GiB 사용). n3 에서 다시 돌려 완주. |

세 주행 유형(래치 반복 종료, 다리 끝 크리프, cusp 위치 정체 종료)의 공통 원인은 확정하지 않았다. cusp 래치 수정 때 정지 판정과 다리 끝 명령을 함께 본다.

## 시험

- ws1 전체(`~/bin/forklift-ws1-pytest`): 결과는 PR 설명에 적는다. 실패는 알려진 ws1 전용 10 건(mujoco 없음 6, evaluate_cli·remote_model_jobs 2, diagnose 게이트 2)뿐이어야 한다.

## 검토

- 계획: 독립 검토 3회(직접 재계산 · Codex 적대적 1–2차 수렴 · agy 독립). 구현: Codex A·D 1–2차, C 1차 "새 P1/P2 없음". 지적과 처리는 계획서 7절.
