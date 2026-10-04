# 낮은 캐리지 장착 — Isaac 렌더 확인 (B1a, 실행 기록)

계획: [캐리지 장착 적용](../plans/2026-10-03-carriage-mount-adoption.md) B1a. 대상: provisional 차체(`artifacts/base_scene_warehouse_forklift/forklift.usd`),
카메라 `fork_carriage` 아래 base (0.559, 0, 0.27)·틸트 0.10 rad, 승강 0, 정렬 자세, EPAL 6, face_gap −0.36…1.20 m 1 cm 157 칸, seed 12.

## 실행

- 렌더: `sim/isaac/render_mount_check.py --camera 0.559,0,0.27 --camera-tilt 0.10 --gaps=-0.36:1.20:0.01`(장착 인자 추가 커밋, snapshot `mount_render_<커밋>`),
  ws1 Slurm 작업 740, 24 분 34 초. 원본 ws1 `artifacts/20261003_render_mount_low/run_740/`(칸별 깊이 npz 157 개, `result.json` sha256 `c5868ba6…`).
  157 칸 모두 유효, 관통 0, 미확인 0, 세 마스크 확인 157.
- 재생: `tools/replay_isaac_nearfield.py` 로 장착 연구의 근접 파이프라인(`tools/nearfield_mount_study.near`)을 각 칸의 Isaac 깊이(1 mm 양자화, min-range 0.175 m)와
  CPU 깊이로 각각 돌렸다. ws1 작업 742, 5 분 17 초. 출력 `replay_isaac.json`·`replay_cpu.json`(둘 다 sha256 `c1b24de5…` — 바이트까지 같다).

## 결과

1. **깊이:** 157 칸 48,230,400 화소(둘 다 유한) 중 Isaac − CPU(float64) 차가 10 µm 를 넘는 화소는 **1 개**(칸 136 의 한 가장자리 화소: 1.726 대 1.406 m). 영역
   내부 최대 차는 기타(바닥·벽) 2.7 µm, 팔레트 3.9 µm, 캐리지 0.19 µm.
2. **1 mm 양자화 깊이:** 앞면 검출 성공·유효 수가 157 칸 모두 Isaac 과 CPU 에서 같다(첫 비영·첫 전 seed 모두 face_gap 0.02 m). 근접 연구 파이프라인을 재생하면
   157 칸 × 12 seed 의 앞면·윗판·인계 판정과 seed 별 무관측이 **모두 같다** — 인계 칸 53, 최대 무관측 0.01 m, 종단 0, 인계 없는 seed 0, 진입 뒤 윗판 최소 12/12.
   장착 연구의 선택은 Isaac 깊이에서도 성립한다.
3. **양자화하지 않은 깊이:** 앞면 검출의 첫 전 seed 칸이 Isaac 원시 깊이에서 face_gap 1.11 m, CPU float32 에서 0.77 m, CPU float64 에서 0.52 m 로 갈린다(첫 비영
   0.35 / 0.35 / 0.02 m). µm 이하 깊이 값에 대한 검출기 민감성(10/01 기록 결과 4)이다. 실행기는 원시 깊이를 검출기에 넣으므로 장착만 바꾸면 이 이점을 잃는다 —
   계획 B1c(1 mm 양자화)의 근거다.
4. 포크 끝 가시: 좌우 모두 face_gap −0.09 m 부터 보인다(Isaac·CPU 같음).

## 이 기록이 말하지 않는 것

- 정렬 자세·승강 0·소품 없는 장면이다. 원거리 검출, 소품 가림, 측방·yaw 오정렬은 B2(임무 실행)에서 본다.
- 실물 D435i 의 잡음·스테레오 가림·최소 거리 밖 결측.
