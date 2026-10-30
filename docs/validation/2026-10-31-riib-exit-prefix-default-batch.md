# riib 런처 기본 켬 배치 — 출구 접두 면제 결과 (2026-10-31)

등록: `docs/plans/2026-10-29-riib-exit-prefix-default-batch.md`(3 판, 제출 기록 6e6d8e1c, 결과 전에 커밋).
배치: ws1 `test_runs/riibotics/rx_6e6d8e1c`, 90 실행(경계 기록 49), 잡 12981·12982, 스냅숏 `rx_6e6d8e1c`.
집계: `tss_diag/rx_sum.py` → `tss_diag/rx_sum.json`. 재생: `test_runs/replay_gate/rx_judge/j_{1..5}.json`(잡 13001).

## 결론

- **등록식으로는 규칙 7(C4) 미달 1 건 때문에 불합격이다. 그래서 riib 런처 기본은 끈 채로 둔다.**
- 나머지 규칙 1–6·8 은 모두 통과다. 효과와 안전 지표는 지난 C5 와 같은 방향이다.
- 미달 1 건(`rxi_box15_aisle_rack_s1`)은 운반 전환 순간의 SLAM 해제 재계획이 `invalid_start` 로 실패한 실행이다. 운반 제어 틱이 0 이라 면제 latch 가 무장될 수 없었다. 그러나 등록은 무효를 "운반 단계에 들어가지 못한 실행"으로만 정했고, 이 실행은 운반으로 전환한 기록이 있다. 무효로 돌리는 것은 결과를 본 뒤의 판단이므로 총괄 결정으로 넘긴다(아래).

## 판정

| 규칙 | 결과 | 판정 |
|---|---|---|
| 1 출발 통과 장면별 B ≥ A, 합 +3 | aisle 8 ≥ 5, trailer 5 ≥ 0, 합 +8 | 통과 |
| 2 완주 합 B ≥ A, 장면별 B ≥ A − 1 | 합 9 ≥ 4, aisle 4 ≥ 4, trailer 5 ≥ 0 | 통과 |
| 3 안전 | 면제 키 4 개 모두 0(B 13 실행), seed 별 `unpermitted_dir_entries` A·B 모두 0, hull 키 모두 0, overlap 실패 0, 같은 스냅숏(같은 평가기) | 통과 |
| 4 회귀 | line_2 A 9/10·B 10/10, random_1 A 7/10·B 7/10. B 실패 3 건 모두 끔 경로 재생 `REPLAY_EXIT 0` → 무관함을 보이지 못한 B 실패 0 | 통과 |
| 5 감시 오경보 | B 대상 실행 13 건 모두 latch 닫힘 `driven`·`length`, 울림 0 | 통과 |
| 6 새 실패 유형 | B 에만 나온 유형 둘(아래)은 이전 배치 A 팔에도 있었고, latch 닫힘 뒤 110 s 넘어서 났다 | 통과(설명) |
| 7 C4 주입 | 24 실행 중 23 통과, `rxi_box15_aisle_rack_s1` 은 latch 가 열리지 않고 주입도 발화하지 않음 | **미달** |
| 8 C3 | `rx_B_aisle_rack_s0`(latch active 에 이른 가장 낮은 seed) 27 104 틱, `rxi_box15_aisle_rack_s0` 34 069 틱, 둘 다 on 경로 `REPLAY_EXIT 0` | 통과 |

### 규칙 4 — 끔 경로 재생

| B 실패 | 단계·사유 | 끔 재생 |
|---|---|---|
| random_1 s3 | 관측, `near_capture_misaligned`(측면 −0.101 m) | 5 970 틱, 같은 예외, 종단 일치, exit 0 |
| random_1 s7 | 관측, `near_capture_misaligned`(측면 −0.089 m) | 6 144 틱, 같은 예외, 종단 일치, exit 0 |
| random_1 s9 | 관측, 추종 시간 초과 | 8 045 틱, 같은 예외, 종단 일치, exit 0 |

A 의 실패(line_2 1, random_1 3)는 집계하지 않고 보고만 한다(등록대로 상계 없음).

### 규칙 6 — B 에만 나온 실패 유형

- return_home `Tracking failed in return_home`(B aisle s0·s5) 과 settle `slam_release_replan_failed: invalid_start`(B aisle s1).
- 두 유형 모두 면제 없는 이전 배치 A 팔에 있다: `epc6_A_aisle_rack_s0`(return_home 추종 실패), `epc6_A_aisle_rack_s3`(settle `invalid_start`).
- latch 는 81–83 s 에 닫혔고, 실패는 195–222 s 의 하역 뒤 단계에서 났다.
- 이번 A 팔에 이 유형이 없는 것은 A 가 운반 출발에서 8 번 멈춰(`Tracking timeout in transport`) 그 단계까지 간 실행이 적었기 때문이다.

### 규칙 7 — C4

- 상자 18 건 중 17 건이 통과했다.
  - 생성 기록 면 거리는 0.15001–0.15007 m 다.
  - 모두 울림(`watch`)으로 닫혔다.
  - 울림 뒤 첫 정지까지 0.0008–0.0073 m 다.
  - 면제 키 위반 0.
- 무응답 6 건 모두 통과했다.
  - 층의 무응답 기록(`silence_trace`)이 `sensor_silent:high` 정지를 적었다.
  - 무응답 뒤 첫 정지까지 0.025–0.040 m 다(한계 0.108 m).
  - latch 는 무응답 직후 첫 0 틱에서 `zero:evidence_stale` 로 닫혔다. 그 실행 전체에서 `evidence_stale` 감속은 1 틱이고, 정지 사유는 `sensor_silent:high` 다(124 틱). 등록 조건은 차가 `sensor_silent` 로 서는 것이고 latch 닫힘 사유는 조건이 아니다. 처음 집계기는 latch 닫힘 사유를 읽어 6 건을 미달로 잘못 냈고, 고친 뒤 위와 같다.
- **미달 1 건 `rxi_box15_aisle_rack_s1`**:
  - 꺼내기 → 운반 전환(81.92 s) 순간의 `MissionCore.slam_release` 재계획이 `invalid_start` 로 실패해 실행이 끝났다.
  - 운반 단계의 제어 틱 0, 운반 표본 0, latch 기록 없음, 주입 로그 없음.
  - latch 는 운반 전환 뒤 첫 운반 허가 틱에 무장되므로, 이 실행에서는 면제 코드가 돌지 않았다.
  - 이 실행에는 경계 기록이 없다(C4 는 box15 aisle s0 만 기록). 따라서 끔 경로 재생으로는 보일 수 없다.
  - 같은 유형(`slam_release_replan_failed: invalid_start`)은 면제 없는 배치에도 있다(위 규칙 6).
  - 등록 3 판 규칙 7 은 무효를 "운반 단계에 들어가지 못한 실행"으로만 두었다. 운반 전환 기록이 있으므로 등록식으로는 "latch 가 열리지 않았다" → 미달이다.

## 총괄 결정이 필요한 것

- (a) 등록대로 둔다: 기본은 끈 채로 두고, 다음 배치에서 C4 를 다시 등록한다.
- (b) 이 실행을 무효(운반 제어 틱 0 = 운반에 들어가지 못한 실행과 같음)로 보고 같은 장면·seed·주입으로 한 번 다시 돌린다. 그것이 통과하면 런처 기본 켬 PR 을 낸다. 이 경우 결과를 본 뒤의 무효 처리라는 점을 문서에 따로 남긴다.
- 권고는 (b) 다. 근거는 셋이다.
  - 면제 코드가 실행되기 전의 실패다.
  - 같은 유형이 면제 없는 배치에도 있다.
  - 나머지 C4 23 건이 모두 통과했다.
- 다만 등록은 이런 사후 재해석을 막으려고 만든 것이므로 결정은 총괄에 맡긴다.

## 참고

- 운반 출발 통과(진행 문턱 aisle 0.85 m, trailer 0.90 m): aisle A 5/8, B 8/8. trailer A 0/5, B 5/5.
- A 의 대상 장면 실패 9 건 중 8 건이 운반 출발 추종 시간 초과다(출구 접두 칸에서 멈춤). B 의 같은 유형은 1 건(aisle s6, latch `driven` 닫힘 뒤)이다.
- 디스크: 배치 폴더 21 GB(recognition 프레임은 롤링 보관 후 삭제), `/home` 여유 35 GB.
