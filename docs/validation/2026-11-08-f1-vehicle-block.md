# 차량 측 F1 차단 검증 (2026-11-08)

계획 [2026-11-08-f1-vehicle-block](../plans/2026-11-08-f1-vehicle-block.md) 5 절의 검증이다. 구현은 브랜치 `feat/f1-vehicle-block`, 스냅숏 `f1b_ecf5f3d5` 다.

## 결과

- **재생 게이트 29/29**(잡 16748, `SETUP_INPUTS=recorded`). 판정 불변 확인이며, 차단 자체의 증거는 아니다.
- **Isaac 스모크: 차단·허용 모두 통과.** 판정은 차량 기록 사후 판정(`tools/f1_block_judge.py`)을 기준으로 했다.

| 세션 | 설정 | 판정 | 내용 |
|---|---|---|---|
| `val4_002_block_r2`(잡 16779) | `F1_AUTONOMY=block` | 통과 | 미션 제출·AUTO·팔레트 추천이 모두 `operator_placed_start` 로 거부, 상태에 `start_block`, AUTO 틱 0, 미션 0, 텔레옵 수락·0.063 m 이동 |
| `val4_002_allow`(잡 16749) | `F1_AUTONOMY=allow` | 통과 | 미션 제출 수락(취소함), `start_block` 없음 |

두 세션 모두 기록의 출처가 `operator_placed` 이고, 적용한 인자와 결정(`blocked`·`allowed`)이 러너 기록에 남았다. 로그는 줄 유실 없이 완전했다. 판정 파일은 [assets/f1_block](assets/f1_block/)에 있다.

**출발.** F1 검증 seed 5 의 `val4_002` 다. 매처가 확정하지 못해 운전자 놓기로 가는 출발이고, F1 검증에서 첫 놓기로 구제됐다. 세션 설정은 F1 수집 목록(`test_runs/live/f1_val2/RUNS.jsonl`)의 항목에 `RELOC_PROBE=place_block` 과 `F1_AUTONOMY` 만 더했다.

**첫 차단 세션(잡 16749 안의 `val4_002_block`)은 미도달로 실패했다.**
- 세 거부는 모두 맞았지만, 텔레옵 진입이 `state_forbids_teleop`(`pacing`)으로 거부됐다. 목표 배속 0.3 에 실제 0.26–0.28 이었다.
- 그 세션은 재생 게이트·전체 pytest 와 같은 노드에서 동시에 돌아 CPU 18/20 이 쓰이던 때였다. 노드를 혼자 쓴 재실행(`_r2`)은 통과했다.
- 실패 기록은 지우지 않았다.

## 남은 것

- 실차 검증. 그 전까지 9.23 의 실차 전 필수 항목은 '구현·시뮬 확인, 실차 검증 전'이다.
- 실차 전 요구(출처 보존, 재위치 없는 재시작 금지, 실차 런처의 `allow` 금지)는 콘솔 계약에 등록했다.
- 차단 중 텔레옵 허가가 놓은 좌표계에서 도는 위험은 실차 전 검토 필수 잔여 위험이다(총괄 결정). 이번 검증 범위 밖이다.
