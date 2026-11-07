# 확인 전 위치 배치 보관 대응표 (개정 계획 9.23 의 5, 2026-11-08)

ws1 경로는 `/home/projects/forklift` 기준이다. 해시는 2026-11-08 ws1 에서 `sha256sum` 으로 잰 값이다. 보관·삭제 실행은 총괄이 디스크 세션에 전달한다. 원본 삭제 조건은 둘이고 모두 통과해야 한다: (가) 이 표의 외부 의존성이 모두 보관 위치를 가짐, (나) 보관본을 복원해 032 2×2 를 다시 계산하고 `diag_cross_032f.json` 과 일치(`restore_check_032.sh`). md5 일치는 업로드 무결성 확인으로만 쓴다.

## 1. 보관 대상 (전부 gdrive)

| ws1 경로 | 크기·파일 수 | 내용 | 기존 보관본 |
|---|---|---|---|
| `test_runs/live/uc_rev_s4` | 495 MB, 8,680 | 72 세션(D 꼬리·폐회로·주입), `eslam_inputs.json` 의 tail 28 세션 | 일부만: `ws1-uc-rev-s4-rolling/chunk_001`(2026-11-07) |
| `test_runs/live/uc_ab_s4` | 298 MB, 3,535 | A·B·B2·B3·M1, `eslam_inputs.json` 의 B 4 세션(`b3/s`) | 없음 |
| `test_runs/live/uc_b2_s4` | 178 MB, 1,791 | 두 번째 B(잡 16605), `bounds_eslam.json` | 없음 |
| `test_runs/live/uc_diag_s4` | 46 MB, 499 | 진단 실행(잡 16726), 032 full 의 `pose_diag_premove.npz`·`prepare_input_1.*` | 없음 |
| `snapshots/ucx_6bdca7e6` | 48 MB | B3 실행 코드(#350 머지 트리) | 없음 |
| `snapshots/ucm_299d3650` | 48 MB | M1 계측 실행 코드 | 없음 |
| `snapshots/ucb2_51c85a8e` | 48 MB | 두 번째 B 실행 코드 | 없음 |
| `snapshots/ucd4_08909f49` | 48 MB | 진단 실행 코드(#353 머지 트리), `prepare_input_*.pkl` 을 여는 클래스의 정본 | 없음 |
| `test_runs/live/uc_dev_s4` | 1.1 GB, 11,250 | D 배치 30 세션, `eslam_inputs.json` 의 D 30 세션 | `ws1-cleanup-20261107-a/a_ucdev.tar.zst`(스냅숏 `ucd_94e912f8` 포함, `PAIRS.txt`) — ws1 원본은 콘솔 판정까지 유지로 남아 있음 |

`eslam_inputs.json`(62 세션, sha256 은 각 세션 `result.json`)의 출처: D `uc_dev_s4/s`, tail `uc_rev_s4/s`, B `uc_ab_s4/b3/s`. 보관 뒤에도 이 sha256 으로 세션 기록을 대조한다.

## 2. 외부 의존성

| 항목 | ws1 경로·식별자 | sha256 | 보관 위치 |
|---|---|---|---|
| 지도 묶음(seed 4) | `test_runs/live/bp1b_survey/hall_s4_G0/bundle/` (map_id `9a5cefd5…a941b`) | `grid.npz` 44346331…3260, `manifest.json` 97404be5…d931, `map.data` c9201467…e57f, `map.posegraph` f01f1f12…34db | BUNDLE_ARCHIVE |
| BASE_SCENE | `artifacts/20261013_fork_spacing/base_scene/scene.usda` (+ `base_scene_manifest.json`·`builder_hashes.txt`·`configuration/`·`forklift.usd`) | `scene.usda` 6471c8ef…f49d | `ws1-cleanup-20261101-b/b_fork_spacing.tar.zst`(같은 크기 7,466 B·시각; 보관본 내용의 sha256 은 복원 때 대조). ws1 원본은 `_keep` 으로 남아 있음 |
| URDF | `artifacts/20261013_fork_spacing/dd_cfg/old/dls08_measured/forklift.urdf` | 517e8260…b760 | 같은 보관본(74,927 B·시각 일치), ws1 원본 유지 |
| bounds | 실행 폴더의 `bounds_eslam.json`(두 번째 B·진단), 스냅숏의 `bounds_provisional.json`(B3·M1) | `bounds_eslam.json` 56828fc6…1f5d, 스냅숏 판 c396bfe0…717b | 각 실행 폴더·스냅숏 보관본 안 |
| 브리지 이미지 | `forklift/ros2-dev@sha256:e7c29c2a…2659`(ID e7c29c2a0c34, 3.01 GB) | 이미지 digest 가 식별자 | **없음** — 삭제 전 `docker save … | zstd` 로 `gdrive:` 에 보관 필요 |
| 실행 환경 | `.venv`(Python 3.11.15, `--system-site-packages` from `/opt/isaacsim-env`), Isaac Sim 5.1.0.0, numpy 1.26.0, scipy 1.15.3, numba 0.59.1, GPU RTX 5070 Ti, 드라이버 580.178.04 | `pip freeze`(346 줄) 4c64a527…bb9b | 보관하지 않음(공유 환경). 버전과 freeze 해시로만 식별하며, 바뀌면 재실행 결과의 비교 가능성이 깨진다 |

## 3. 복원 확인 (나)

`restore_check_032.sh`: 보관본에서 `uc_diag_s4/s/d_val3_032_full/epoch_1/run/{pose_diag_premove.npz,prepare_input_1.pkl,prepare_input_1.npz}` 와 URDF 를 빈 폴더에 풀고, main 의 `tools/start_check_cross.py`(#354)로 2×2 를 다시 계산해 `diag_cross_032f.json` 과 비교한다. `MATCH` 일 때만 종료 코드 0 이다. 같은 형식의 가짜 보관본으로 `MATCH` 를 확인했다(2026-11-08). 실제 보관본 확인은 업로드 뒤 콘솔이 돌린다.

복원된 파일의 sha256(2026-11-08 ws1 원본): `pose_diag_premove.npz` 00acbf92…ed6d, `prepare_input_1.pkl` 787865e0…7d47, `prepare_input_1.npz` 4123fc0f…58bf.
