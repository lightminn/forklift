# 합성 장면 데이터 세트 v1 생성 검증 — 2026-09-11 (M1-b 3단계)

**범위:** [3단계 계획](../plans/2026-09-11-scene-capture-and-remote.md) Task 7. 브랜치 `feat/scene-capture`의 캡처 노드·runner(`96a8e62`)와 원격 `scenes` mode·병합 도구(`9b3902b`)로 카탈로그 v1의 장면 100개를 원격 Gazebo에서 실제로 캡처하고, 회수·병합·로더 검증까지 확인했다. 이 세트는 **합성**이며 D435i 특성·실물 팔레트·장착 보정을 나타내지 않는다.

## 구현 검증 (호스트)

| 검사 | 결과 |
|---|---|
| 위임 A(캡처 계층) 후 회귀 | 488 passed, 1 deselected (2단계 422 + 66) |
| 위임 B(원격·병합) 후 회귀 | **571 passed, 1 deselected** (+83) |
| `ruff check .` / `ruff format --check .` | 통과 / 85 files |
| `bash -n deploy/slurm/model_check.sbatch` | 통과 |
| `submit --mode scenes … --dry-run` | JSON에 `catalogue`, `scene_range`, `time_limit 01:30:00` 포함, 카탈로그 파일이 snapshot(64개) 안에 있음 |
| 원격 wrapper 전제 | `/usr/bin/python3` 3.12.3에 PyYAML 6.0.1 있음(범위 ID 재검증용) |

두 위임 모두 Codex가 구현했고 계획 대비 편차는 wrapper의 PyYAML 필요 명시 한 건이다.

## 2장면 spike — `20260911T123918Z_scenes_spike_s001_s002`

**Slurm 985 `COMPLETED 0:0`, 실행 16초**, revision `9b3902b` clean, 이미지 `sha256:489f4aa6…`, 47개 파일 회수·해시 일치. manifest `failed_count 0`.

| 장면 | build_world | ready | first_clock | captured | stopped |
|---|---:|---:|---:|---:|---:|
| s001 (occluded, eval) | 0.3 s | 0.8 s | 3.8 s | 5.8 s | 6.2 s |
| s002 (occluded, eval) | 0.3 s | 0.5 s | 3.6 s | 5.1 s | 5.5 s |

장면당 약 6초(2초 simulation warmup 포함)라 25장면 batch는 3분 안팎이며, 계획의 `--time-limit 01:30:00`은 충분하다. batch 크기는 승인 설계대로 25로 유지했다(실패 시 batch 전체 재제출 비용이 작다).

로더 검증: `load_scene_sample`이 두 장면을 읽었고 RGB 640×480, depth 640×480(유효 픽셀 약 14.7만·14.2만, 0.97–9.13 m), 실제 Gazebo CameraInfo가 좁은 계약을 통과(fx=fy=465.741, cx=320, cy=240, `camera_optical_frame`), stamp 2 000 000 000 ns·`ros_sim`·`synthetic`. 정답 파일은 시각·clock 외 카탈로그 정답과 동일. 정답 포켓 중심을 영상에 투영해 읽은 depth는 개구부 뒤 배경(3.56 m)이나 가림 상자(2.02 m)를 가리켰다. 이는 개구부 중앙 depth 한 픽셀을 포켓 위치로 쓰면 안 된다는 M2 규칙의 실증이다. RGB와 depth preview를 직접 열어 팔레트의 두 개구부·가림 상자·distractor 두 개를 확인했다.

## 100장면 batch — `20260911T124147Z_scenes_*`

25장면씩 4개 batch를 순차 제출했다. 모두 **Slurm `COMPLETED 0:0`, 실행 2분 23초**, 실패 0, 같은 이미지 ID `sha256:489f4aa6…`와 같은 소스 snapshot `d8a392c35a43…`(revision `9b3902b` clean). 각 batch는 507개 파일(장면당 8개 데이터 파일 + world·로그)을 회수했고 해시가 원격과 일치했다.

| batch | Slurm | 장면 | 실패 | 장면당 시간(중앙값) |
|---|---|---:|---:|---:|
| s001–s025 | 986 | 25 | 0 | 5.5 s |
| s026–s050 | 987 | 25 | 0 | — |
| s051–s075 | 988 | 25 | 0 | — |
| s076–s100 | 989 | 25 | 0 | — |

재제출은 필요하지 않았다.

## 반복 캡처(결정론 관찰) — `20260911T124147Z_scenes_repeat_s001_s002`

같은 두 장면을 새 실행 ID로 다시 캡처했다(Slurm 990 `COMPLETED 0:0`, 16초). spike·batch 1·반복 run의 세 독립 캡처에서 s001·s002의 `depth_mm.png`와 `rgb.png` SHA-256이 각각 완전히 같았다(s001 depth `6a91a077…`, s002 depth `6c936548…`). 이 이미지·llvmpipe 렌더러 조건에서의 관찰이며 일반 증명은 아니다.

## 세트 병합과 로더

`tools/merge_scene_batches.py --catalogue sim/gazebo/scenes/catalogue_v1.yaml --batches <4개> --output data/synthetic_scenes/catalogue_v1`가 카탈로그 해시·카메라·이미지·소스 동일성, ID 완비(중복·누락·실패 0), 파일 해시 재계산, 정답이 카탈로그와 같음(시각·clock만 치환), 범주별 status, 로더 성공을 모두 통과했다. 세트 manifest SHA-256 `e915cd6f27bef69e…`, `catalogue_sha256 50fec6ec…`, category 60/20/10/10, split 70/30, 전체 4.2 MB(단색 합성 장면이라 PNG 압축률이 높다).

로더 검증: `load_scene_sample`로 100장면 전부 읽었다. status **valid 80 / no_pallet 20**, 범주와 status 정합(positive·occluded → valid, 두 음성 → no_pallet), scene.json의 category/split이 카탈로그와 일치, 검정 RGB 없음, 유효 depth 픽셀 13.7만–16.9만(중앙값 13.99만), 캡처 stamp는 100장면 모두 2 000 000 000 ns(warmup 직후 첫 세트). 양성(s003: 근접 팔레트의 두 개구부), 유사물(s015: 개구부 없는 상자), 무팔레트(s012) 장면의 RGB를 직접 열어 확인했다.

데이터는 로컬 `data/synthetic_scenes/catalogue_v1/`(Git 제외)과 원격 팀 작업 공간 `data/synthetic_scenes/catalogue_v1/`에 같은 내용으로 두었다(manifest 해시 대조).

## 경계

- 캡처는 CPU llvmpipe 렌더이며 노이즈가 없다. 같은 장면의 반복 캡처 동일성은 이 이미지·렌더러 버전에서의 관찰이지 일반 증명이 아니다.
- 가시성·가림 비율은 카탈로그의 핀홀 계산값이고, 실제 렌더에서의 가림은 영상으로만 확인했다.
- 데이터는 Git 밖(로컬 `data/synthetic_scenes/catalogue_v1/`, 원격 artifacts 실행 디렉터리)에 있으며 세트 manifest 해시로 식별한다.
- 원격 루트 디스크가 94 % 사용(62 G 여유)이라 batch 결과는 회수 후 원격 정리 대상이다.
