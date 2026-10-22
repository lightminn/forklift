# IEEE CASE 초안 — 교차 큐 · 전역 순서 최적화 (Q3 · Q3p)

작성일: 2026-10-22. 초안이며 제출본이 아니다.

## 빌드

```bash
cd paper
latexmk -pdf -outdir="$TMPDIR/paper_build" main.tex   # IEEEtran, TeX Live 기본 패키지만
```

`main.pdf` 는 2026-10-22 빌드본(6쪽, 참고문헌 포함)이다.

## 수치의 출처

본문의 수치는 아래 기록에서만 옮겼다. 옮긴 곳마다 `main.tex` 에 `% src:` 주석이 있고, 그 수치로 계산한 비율은 `% derived` 로 표시했다.

| 내용 | 출처 |
|---|---|
| 117 칸 평가(A · B · Q1–Q3p), 판정, 개발 중 고침 | `docs/plans/2026-10-16-fleet-crossing-queues.md` |
| L2 · L3 · L4 117 칸, L1 개발 결과, 문헌 대응 | `docs/plans/2026-10-17-fleet-literature-baselines.md` |
| 36 칸 대규모(r2 + r2x), bootstrap, 붕괴 칸 진단, 계산 시간, 지평 시험 | `docs/plans/2026-10-17-fleet-large-random-chained.md` |
| 기준 방식 A(시간 창), 교차 · 순서 조건 · 고리 검사 | `docs/plans/2026-10-11-fleet-time-windows.md` |
| 물리 도착 예정(Q3p) | `docs/plans/2026-10-12-fleet-physical-time-windows.md` |
| 공장 생성 · 조건 C0–C5 | `docs/plans/2026-10-09-fleet-random-worlds.md` |
| 차체 치수 | `sim/models/dls08_measured/README.md` |
| MILP 식 · 큐 동작 | 커밋 2387240 의 `fleet_queue_opt.py` · `fleet_queues.py` |
| 36 칸 트럭끼리 겹침 0 | r2 · r2x `result.json` 의 `contacts` 에 `truck` 항목 없음(216 실행, 2026-10-22 확인) |
| 참고문헌 | `~/.local/state/forklift-queues/scratch_20261018/lit_notes.md`. 그 밖의 항목은 `refs.bib` 에 「from memory -- verify」 |

## 남은 TODO (본문에 빨간 글씨)

- 저자 · 소속 · 메일
- TA/RA 설명의 인용 또는 본문 설명 보강
- L1(타일 예약) 평가 결과 — `cpu/eval700_L1` 이 끝났으면 표 II 에 추가
- 그림 2(칸별 짝 비교 · 대기 길이 분포) — 데이터만 있고 그림은 없음
- 결론의 향후 과제 문장
- `refs.bib` 의 기억 인용 6 개와 저자 이니셜 2 개 확인
