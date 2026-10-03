# 낮은 캐리지 장착 — Isaac 개발 실행 159 seed (B2-1, 실행 기록)

상위: [계획](../plans/2026-10-03-carriage-mount-adoption.md) B2, [렌더 확인](2026-10-03-carriage-mount-render.md).
기준: 같은 159 seed 의 기록 — 0–8·1000–1029·2000–2029·3000–3029·4000–4029 는 [운반 단계 개발 실행](2026-10-03-transport-stage-dev-run.md)(작업 718, 125/129),
5000–5029 는 [다섯 번째 동결 평가](2026-10-03-fifth-frozen-evaluation.md)(작업 725, 30/30). 기준 완주 155.

## 사전 고정 (결과를 보기 전, 2026-10-04)

- 코드: 커밋 `4ecdcfb`(브랜치 `feat/carriage-mount`), ws1 snapshot `snapshots/mountdev_4ecdcfb`, 파일 목록 해시 `9c9af7d9…`. `run_transport.py` `6977e71e…`,
  `perception_adapter.py` `9bf6cac5…`, `pallet_mission.py` `146206ef…`(기준 5000–5029 와 같음), `pocket_detector.py` `7bd2a28c…`(같음),
  `config/isaac_transport.yaml` `51f05655…`(같음).
- 인자: 기준과 같고 `--perception-mount carriage_low --depth-quantize-mm 1` 을 더한다. 출력 `artifacts/20261004_mount_dev/`.
- 같은 작업에서 **legacy 대조**(같은 코드, 인자 추가 없음): seed 3012·4007·4013·3008. 결과와 기록이 달라진 seed 는 결과를 본 뒤 별도 작업으로 legacy 대조를 더
  돌린다(그 seed 목록은 이 문서에 결과와 함께 적는다).
- 스모크(결과 아님, 작업 743): seed 2001 이 carriage_low·양자화 1 로 62.9 s 완주(카메라 부모 `fork_carriage`), 같은 코드 legacy 로 63.0 s 완주.
- 재실행: `result.json` 없음·Slurm OOM/NODE_FAIL/CANCELLED·`phase == "startup"` 이면 같은 snapshot 으로 1 회. 그 외 실패는 결과.
- 판정(계획 B2-1, 그대로 옮김 — 비열등성 판정이며 약 3 % 회귀를 허용하는 느슨한 기준):
  - 기준 완주 155 개 중 실패로 바뀐 것 ≤ 5. 실패 → 성공은 따로 센다.
  - 실패로 바뀐 seed 중 검출 관련(관측 후보 소진, 검출 기각 뒤 도달 불가, 검출 기각으로 관측이 늘어 시간 예산 초과) ≤ 3.
  - 실패로 바뀐 seed 마다 legacy 대조로 "장착/양자화 탓" 과 "실행 변동" 을 가른다.
  - 5000–5029 는 이 실행으로 개발 데이터가 된다.
- 모든 관측 시도를 G3 도구로 분류한다(같은 커밋의 git worktree, 장착별 설정 키).
