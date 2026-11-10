# 라이브 riib ④⑤ — rv4 결과 (2026-11-11)

- 등록: [docs/plans/2026-11-10-live-riib-rv4-registration.md](../plans/2026-11-10-live-riib-rv4-registration.md)(#415, 고정). 바꾸지 않은 조건·판정은 [rv3 등록](../plans/2026-11-10-live-riib-v2-rerun-new-snapshot.md)(#398) 그대로다.
- 도구: 결과 전에 머지한 `tools/riib_v2_verdict.py --rv4`(#418)를 rv4 스냅숏 코드로, 배치가 끝난 뒤 한 번 실행했다. 결과는 [verdict.json](2026-11-11-live-riib-rv4-results/verdict.json)(sha256 f55c7f10…)이다.
- 실행 기록:
  - [manifest_groups.jsonl](2026-11-11-live-riib-rv4-results/manifest_groups.jsonl)(다른 사용자 잡 이름은 `(job)` 으로 지운 사본)
  - [sacct_night.txt](2026-11-11-live-riib-rv4-results/sacct_night.txt)(그날 밤 `sacct -a -X` 전체, 다른 사용자 잡 이름은 지움)
  - [dry_check.json](2026-11-11-live-riib-rv4-results/dry_check.json)(제출 전 27 세션 DRY_RUN 점검)
  - 동일성 재대조: [identity_after_pa4.json](2026-11-11-live-riib-rv4-results/identity_after_pa4.json), [identity_after_end.json](2026-11-11-live-riib-rv4-results/identity_after_end.json)

## 1. 결론

**판정: 통과(pass).** 고정 도구의 출력은 `outcome: pass`, `judged: true`, `held_by` 와 `to_lead_by` 모두 비어 있음이다.

**총괄 결정(2026-11-11): 아래 두 미확인을 '결과 뒤 예외'로 받아들이고, ④⑤ 승격으로 결정했다**(근거는 6 절). 아래는 결정 전의 권고 문구다.

**승격 권고는 조건부다.** 등록된 실행 조건 가운데 두 가지를 확인하지 않은 채 제출했다(6 절의 '미확인 실행 조건'). 하나는 제출 전 스레드 설정 대조이고, 다른 하나는 묶음군 종료 직후의 노드 기록이다. 도구의 `to_lead_by: []` 는 이 실행 조건까지 확인했다는 뜻이 아니다. 사후 조사로는 차이의 근거가 없다. 총괄이 이 두 미확인을 예외로 받아들이면, 등록(rv3 등록 2 절의 결과 처리)대로 **④⑤ 승격을 권고한다.** 결정은 총괄이 한다.

| 기준 | 결과 |
|---|---|
| 페이스(통합 A/B, 짝 8 개 모두 적격, 빠진 짝 0) | 통과 |
| 유효성 가드(짝 공통 구간 F ≥ 0.27) | 통과 |
| C2 재계획 스텝 2.0 s 상한 | 통과(C2 호출 1 건, 0.028 s) |
| 안전·기록 완결 | 통과(최소 이웃 간격 0.3495 m, 금지 접촉 0) |
| ④ 최소 유효 발화 | 충족(후진 표적 6 임무 중 5) |
| ⑤ 최소 유효 발화 | 충족(c2_line_m_on m1) |
| 생명주기 1·2·3 | 통과·통과·기록대로 끝남 |
| 생명주기 4(확인 판정) | 통과(판정 B 두 세션, 대조 A 두 세션 모두 통과) |
| 실패 목록(B 쪽에만·A 쪽에만·양쪽) | 0·0·0 |
| plant 관문 | 통과 |
| 실행 확인(지도·브리지 이미지·덤프 인자) | 위반 0 |
| #346 계열 분류 칸 | 해당 없음(적재 운반 cross_track 종결 0 건) |

## 2. 페이스

값은 짝 공통 구간에서 잰 F / R / 최장 연속 ok=false / stall 이다.

| 짝 | 장면·seed | 먼저 | A | B |
|---|---|---|---|---|
| 1 | line_m s0 | B | 0.3000 / 0.0056 / 1.82 s / 0 | 0.3000 / 0.0028 / 1.14 s / 0 |
| 2 | random_m s0 | B | 0.2999 / 0.0121 / 3.44 s / 0 | 0.2999 / 0.0125 / 2.40 s / 1 |
| 3 | line_m s1 | A | 0.3000 / 0.0031 / 1.36 s / 0 | 0.3000 / 0.0039 / 1.54 s / 0 |
| 4 | random_m s1 | B | 0.2999 / 0.0066 / 2.31 s / 0 | 0.2999 / 0.0084 / 1.78 s / 0 |
| 5 | line_m s2 | B | 0.3000 / 0.0058 / 1.48 s / 0 | 0.3000 / 0.0035 / 1.13 s / 0 |
| 6 | random_m s2 | A | 0.3000 / 0.0210 / 1.71 s / 0 | 0.3000 / 0.0103 / 1.57 s / 0 |
| 7 | line_m s3 | B | 0.3000 / 0.0044 / 1.55 s / 0 | 0.3000 / 0.0036 / 1.19 s / 0 |
| 8 | random_m s3 | A | 0.2998 / 0.0070 / 1.76 s / 2 | 0.3000 / 0.0063 / 1.15 s / 0 |

| 지표(팔별 합산, 8 짝) | A | B | 예산 | 결과 |
|---|---|---|---|---|
| 창 배율 F | 0.29995 | 0.29996 | B ≥ 0.97 × A | 통과 |
| ok=false 비율 R | 0.00824 | 0.00644 | B ≤ A + 0.01 | 통과 |
| 최장 연속 ok=false | 3.44 s | 2.40 s | B ≤ 1.2 × A + 2 s | 통과 |
| stall(> 0.25 s) | 2 | 1 | B ≤ A + 2 | 통과 |

- 고정 조건(따라잡기 6, 짝 슬롯 2)에서 본 새 코드의 A/B 상대 회귀 확인이다. 기준선 유지나 최적 동시 수를 주장하지 않는다(rv3 등록 1.2).
- 가설 분류(판정에 안 씀): A 는 캡처 처리 2, B 는 캡처 처리 1 이다.

## 3. 기능·생명주기·안전

- ④: `boxed_line_m` m1–m3, `boxed_random_m` m2·m3 가 유효 발화다(설치된 경로 접두, 참값 후진, 복귀 완료). `boxed_random_m` m1 은 설치된 경로 접두가 없어 세지 않았다. `trigger: boxed` 는 0 이다.
- ⑤: `c2_line_m_on` m1 이 `near` 설치 → 근접 캡처 `done` → 임무 완료다. 그 재계획 스텝 간격은 0.028 s 다. `c2_line_m_shadow` m1 은 shadow 발화 뒤 `Tracking timeout in observe` 로 끝났다(등록의 예상된 끝).
- 생명주기 1: 연쇄 11 단계 순서대로, 임무 완료, epoch 종료 0.
- **생명주기 2: 통과(H3 직접 기록 기준, rv4 등록 1.1).**
  - 탐침 송신 664318.374452 ≤ 직접 적용 기록 664318.387417 ≤ 탐침 응답 664318.387869 이다.
  - clock 확인 통과(탐침과 런타임의 `boot_id`·time namespace 가 같음). 런타임 시작 epoch = 탐침 epoch(46222113884997), 취소 대상 m1 = 적용 기록의 임무 m1 이다.
  - 보고 필드: 수신 순서도 채움. 스텝 대용(664318.387548)도 응답 앞이었고, 직접 기록보다 0.13 ms 늦다.
  - 운반 중 취소 `cancelled`(operator), 계획된 재제출 두 번의 `cargo_held` 거부, `session_end` ack, epoch 종료 0.
- 생명주기 3: `reloc_needs_operator` 로 기록대로 끝났다(임무 전, epoch 종료 코드 1). 제출 근거가 없어 안전 검사는 '제출 전 종료'로 적용하지 않았다. plant 관문은 이 세션에도 `result.json` 과 plant 키가 있어 다른 세션과 함께 검사했고 통과했다.
- 생명주기 4:
  - 네 세션 모두 연쇄·STOP 유효·대상 대조·안전·C2 상한을 채웠다. 모두 회복을 거쳤고, STOP 대기 중 실제 정지 유지 위반은 없다.
  - STOP 송신 → 응답: B0 0.018 s, B1 0.010 s, A0 0.004 s, A1 0.005 s.
  - CPU 계획 관문(17620)은 통과했다. 11 점 모두 검사했고 관문 7 점이 `passed` 다. 보고만 하는 접근 끝 2 점은 `observe_failed` 로 rv3 와 같은 모양이다.
- `--occupied-dumps`(생명주기 4 네 세션만, 승격할 구성과 이 옵션 하나가 다름. 보고만):
  - 세션마다 덤프 3 개(임무 시작 2, 운반 입구 1), 버림 0, 쓰기 실패 0.
  - 총 `write_s` 는 B0 0.0062, A0 0.0071, B1 0.0068, A1 0.0071 s 다.
  - 쓰기 틱과 STOP·pacing false 구간의 겹침(rv3 등록 1.3 의 보고): 네 세션의 첫 덤프(모의 0.825 s)는 모두 `pacing_ok=false` 틱이다. A0 의 둘째 덤프(39.108 s)도 `pacing_ok=false` 틱이다. 나머지는 `true` 다. 모든 덤프는 `AUTO` 모드에서 쓰였고, STOP 대기 구간에는 없다. 동시 발생일 뿐 덤프가 원인이라는 뜻은 아니다.
- 안전: 금지 접촉 0, 이웃 팔레트 겹침 0(최소 간격 0.3495 m, `ab_line_m_s3_B`), 정지 팔레트 이동 경계 초과 0, `unpermitted_dir_entries` 0, spool·`mission_results` 완결.

## 4. 재시도 목록(rv4 등록 1.2)

| 세션 | session_epoch | 거부 | 수락 | 간격 |
|---|---|---|---|---|
| life1_line_m_main | 162764741083237 | r2 `not_stopped` | r3 ack(같은 대상) | 0.53 s |
| l4c_s1_A | 127136464874341 | r2 `not_stopped` | r3 ack(같은 대상) | 2.29 s |

- 둘 다 그 세션 epoch 의 첫 `mission_submit` 시도이고, 바로 다음 시도가 같은 대상의 ack 다. 세션당 상한 1 안이다.
- 제출한 26 세션의 시도열이 모두 완전하다. 생명주기 3 은 제출 전 예상 종료라 시도가 없다. 시도 단위 거부 실패는 0 건이다. 생명주기 2 의 `cargo_held` 두 번은 계획된 대상 순서로 면제됐다.

## 5. plant 판(rv4 등록 3)

- 현재 결정은 plant 변경 없음(#414 뒤 총괄)이다. rv4 는 현재 강체 접촉·120 Hz plant 로 두 팔을 맞췄다.
- 관문 통과:
  - 세션 전체 같음: `settings_synthetic`(`physics_hz` 120), `physics_scene`, `plant_tyre_contact`, `plant_backend`, URDF·장면 해시, `pallet_mass.kg`.
  - 장면별 같음: `plant`, `pallet_mass`.
  - 매니페스트(차량 URDF 517e8260…, base scene USD 6 파일)와 같음.
- 이 매니페스트 값은 rv3 의 실행 기록 값과도 같다. 그래도 rv3 자료와 섞어 집계하지 않았다.

## 6. 실행과 실행 예외(판정식과 별개)

- 스냅숏 `snapshots/rv4_54bd84a9`(main 54bd84a9, #417·#418 포함), 실행 폴더 `test_runs/live/rv4_54bd84a9/`.
  - `lists.sha256` 의 sha256 은 79ef1dde…다. 제출 직전 `submit_group.sh` 에 afterok 의존 허용을 더해 bccedcd7… 에서 다시 썼다. 매니페스트 `pre_submit_tool_change` 에 기록했고, 목록·계획 파일은 그대로다.
- 동일성:
  - 제출 전: 스냅숏 트리 = 커밋, 바이트코드 검사 통과. 래퍼 v3(1804a415…, rv3 와 같음)·장면·ONNX·URDF·base scene·VRAM 표본기·브리지 이미지(e7c29c2a…)가 rv3 와 같다.
  - 인터프리터 `.venv`·`venvs/learned` 의 파일 내용 해시와 pip freeze 도 rv3 와 같다.
  - 런처 해시는 rv3 와 다르다(3e3cb622…, #405 의 G3 기본 켬).
  - 재대조: CPU 관문(rest 뒤)과 배치 끝 뒤 두 번 했다. 스냅숏·해시·인터프리터(끝 뒤에는 plant 매니페스트도)가 제출 때와 같다. 중간에 바뀌었다 되돌아간 경우는 못 잡는다.
- 스위치(매니페스트 기록):
  - 1e(`--layer-worker`)는 꺼짐이다(러너 기본, 27 세션의 해석 인자에 없음).
  - G3 1b·1c·1d 는 런처 운용 기본(켬)이다. 27 개 DRY_RUN 모두 `SCAN_SWITCHES grid_clear_kernel=1 shadow_kernel=1 lidar_cast=fast`.
  - 두 팔이 같다. 총괄 결정(2026-11-10): 운용 기본이 바로 승격 대상의 기준이고, 두 팔이 같으며, F 가드는 아래쪽 한계이고 G3 는 배율을 올리는 쪽이다.
- DRY_RUN: 27 세션 모두 ARGS_OK. A/B 짝 8 개의 해석 러너 인자는 `--return-start-backoff boxed --near-end-stand on` 차이뿐이다.
- 자원: 모든 Isaac 묶음이 노드 단독 대용(`--cpus-per-task=20 --mem=56G --gres=shard:4`)이었고, `taskset` 으로 8 CPU(0–3·12–15)에 묶었다.
  - 묶음 요약: 세션 CPU 0–3·12–15, 브리지 3·15.
  - affinity 표본기(261 회): 러너와 인식 작업자 0–2·12–14, 브리지 컨테이너 3·15, `CUDA_VISIBLE_DEVICES=0`. 모든 세션·두 팔이 같다.
  - 메모리 cgroup 은 56G 다.
- 실행 순서(등록 순서 그대로, 총괄 신호에 따른 순서 변경 없음):
  - pa_1 17611 23:59:56 → pa_2 17612 → … → pa_8 17618 03:42:07
  - → rest 17619 04:20:27 → CPU 관문 17620 04:23:28 → 생명주기 4 17621(afterok) 05:10:36–05:37:03
  - 17621 은 관문이 끝난 뒤 바로 시작하지 못했다. 그 사이 시작한 CPU 잡 17665(04:20:27–05:10:36)가 끝나기를 기다렸다. 노드 단독 대용이라 다른 잡이 있으면 시작하지 않는다.
- **17619(rest)의 Slurm `FAILED` 는 묶음 안의 예상된 끝 때문이다.** `summary.json` 은 ended 4, early_mission_failure 2, ended_rc_nonzero 1 이다. 세 실행은 다음과 같고, 모두 등록의 예상된 끝이다. 판정 도구의 실패 목록에도 없다.
  - `c2_line_m_shadow`: shadow 발화 뒤 `Tracking timeout in observe`
  - `life2_random_m_held`: 운반 중 operator 취소
  - `life3_random_m_reloc`: `reloc_needs_operator`, 종료 코드 3
- 겹침: 판정에 쓴 묶음 10 개(17611–17619, 17621)의 구간 안에 다른 잡은 없다(`sacct -a -X` 전체).
  - 초 해상도 기록상 같은 초 경계: 17665 시작 = rest 끝 04:20:27, 17665 끝 = 생명주기 4 시작 05:10:36, 17686 시작 = 생명주기 4 끝 05:37:03.
- **CPU 관문 17620(04:20:27–04:23:28)은 CPU 잡 17665(lightminn, 다른 세션)·17685(다른 사용자, 04:20:28–04:20:40)와 겹쳤다.** 등록은 관문이 Isaac 묶음과 겹치지 않을 것만 요구하므로 위반이 아니다. 관문 결과(인터프리터 해시 대조, CPU 계획 판정)는 실행 속도와 무관한 값이다.
- 재제출·제출 오류·취소는 없다. 실제 실행 27 세션으로, 34 상한 안이다.
- **미확인 실행 조건(등록 위반, 총괄 처분 필요)**:
  1. **제출 전 스레드 설정 대조를 하지 않았다.** rv4 등록 2 절은 CPU 수로 스레드를 정하는 설정(`OMP_NUM_THREADS`, numba 스레드 등)이 20 CPU 요청을 보고 달라지지 않는지 rv3 ② 의 8 CPU 실행과 대조하고, 다르면 제출하지 않도록 정했다. 제출 전에 이 대조를 빠뜨렸다. 사후 조사(코드·기록, 새 실행 없음):
     - `OMP_NUM_THREADS=2`: 런처의 상수(`isaac_live.sbatch` 의 export 줄)이고, rv3 런처에도 같은 값이다. 표본기가 러너·인식 작업자 모두에서 2 를 기록했다.
     - onnxruntime 스레드: 러너 인자 `--recognition-threads 4` 로 고정된다. rv3 `args.txt` 와 rv4 의 27 개 DRY_RUN 모두 4 다.
     - `OPENBLAS_NUM_THREADS=24`: 인식 작업자 환경에 있다. 저장소 코드는 설정하지 않으며, 러너의 시작 환경에도 없다(실행 중 부모 프로세스가 정한 값을 물려받은 것으로 보인다). 24 는 ws1 의 논리 CPU 수(`nproc --all`·`os.cpu_count()` = 24)와 같다. 작업자의 affinity(6 CPU)나 Slurm 요청(20)과는 다르다. 그래서 affinity·cgroup·Slurm CPU 수에서 정해진 값이 아니고, rv3 ② 의 8 CPU cgroup 에서도 같은 값이었을 것이다(추론이고, rv3 ② 의 값은 기록이 없어 직접 대조하지 못했다).
     - numba: `NUMBA_NUM_THREADS` 는 설정되지 않았다. numba 0.59.1 의 기본값은 `os.sched_getaffinity(0)` 의 CPU 수다(Codex 지적, numba `config.py`). 그래서 affinity 의 영향을 받는다. 다만 묶음 실행기가 세션·브리지 CPU 를 나누는 방식이 rv3 ② 와 같다(묶음 요약 모두 CPU 0–3·12–15, 브리지 3·15). G3 커널은 `parallel=True` 없는 `njit` 라 병렬 스레드를 쓰지 않는다. 그래서 차이가 났다고 볼 근거는 없다.
     - 결론: 차이가 있었다는 근거는 없다. 다만 등록이 요구한 '제출 전 대조'는 하지 않았다.
     - **총괄 결정(결과 뒤 예외)**: OMP·`--recognition-threads`·numba 근거로 받아들였다. 조건은 `OPENBLAS_NUM_THREADS` 를 rv3·rv4 스냅숏의 런처·sbatch·래퍼 어디에서도 설정하지 않는다는 grep 확인이다(새 실행 없이).
     - **grep 결과(조건 충족)**:
       - `git grep -i OPENBLAS` 를 941da05a(rv3)와 54bd84a9(rv4)의 `deploy`·`sim`·`src`·탐침·묶음 실행기·판정 도구에 돌렸다. 나온 곳은 `deploy/gazebo/Dockerfile`, `deploy/hpc/run_sweep.sbatch`, `deploy/slurm/recognition_stage2{a,b}_evaluate.sbatch` 뿐이다. 모두 라이브 경로(`isaac_live.sbatch`, `isaac_bundle.sbatch`)가 쓰지 않는다.
       - 래퍼 v3, `rv3_wide.sbatch`·`rv4_wide.sbatch`, 두 배치의 세션별 `runner.sh`·`env.sh` 에도 없다.
       - 그래서 같은 노드, 같은 동결 인터프리터의 기본값이고, rv3 도 24 다.
  2. **묶음군 종료 직후 노드 기록을 남기지 않았다.** 총괄 결정(결과 뒤 예외)으로 받아들였다. 근거는 셋이다. 겹침은 전체 `sacct -a` 로 판정했고, affinity 표본이 261 회 있으며, 결과 쪽 영향은 pace guard(F 0.2998–0.3000 ≥ 0.27)가 직접 막는다. rv4 등록 2 절은 묶음군이 끝난 직후 `sacct -a`, `/proc/loadavg`, `docker ps`(조건부 `nvidia-smi`)를 남기도록 정했다. 이번에는 제출 때의 기록(매니페스트), 그날 밤 전체 `sacct -a -X`, affinity 표본기(261 회)만 있다. 종료 직후 기록과 그 생략 사유는 없다. 겹침 판정은 전체 `sacct` 로 했다.

## 7. 한계

- 8 짝은 회귀 확인이지 비열등성 입증이 아니다.
- **v2·rv3 기준선은 G3 끔에서 잰 값이고, rv4 는 G3 켬이다.** 고정한 기준선(따라잡기 6, 슬롯 2·3, F < 0.27 가드)을 다시 재지 않고 썼다. G3 켬의 실현 배율 효과는 1e 판정 실행에서 AUTO 배율 0.885–0.890 → 0.933–0.943 으로 관측됐다(목표 배율이 다른 별도 설정). rv4 의 창 배율 F 는 목표 0.3 을 따라 0.2998–0.3000 이었다. rv4 는 rv3 와 섞지 않았다.
- 원래 확인 실행 실패(d72fd9b3)의 스냅숏 특이성과 간헐 실패의 발생률은 이 배치로 답하지 않는다(rv3 등록 0 절).
- rv3 의 B 운반 cross_track(#346 계열)은 이 배치에서 다시 나지 않았다. 표본이 8 짝이라 발생률이 줄었다고는 말하지 않는다.
- 생명주기 4 네 세션은 `--occupied-dumps` 하나가 승격할 구성과 다르다.
- 스레드 수: 표본기가 기록한 러너 스레드 수는 실행 중 대부분 159–165 개이고, 기동·종료를 포함하면 1–165 개다. rv3 는 스레드 수를 기록하지 않아 직접 대조하지 못했다(6 절의 미확인 실행 조건 1).
- plant 키는 러너가 기록하는 범위만 묶는다(rv4 등록 3 의 한계).
- docker 등 Slurm 밖의 부하는 확인하지 못했다. 겹침 판정은 Slurm 잡만 보며, 묶음군 종료 직후의 `docker ps`·`loadavg` 기록이 없다(6 절의 미확인 2).

## 8. 다음

- 총괄 결정: 승격(2026-11-11). 승격 PR 관문(rv3 등록 2 절):
  - PR 의 기준(그때의 main)과 고정 커밋 54bd84a9 사이에 운용 경로(`src/`·`sim/`·`deploy/`·`config/`, 런처·래퍼·탐침)가 바뀌지 않아야 한다.
  - PR 은 라이브 런처의 ④⑤ 기본값만 바꿔야 한다.
  - 그 사이 운용 경로가 바뀌면 이 배치의 근거를 옮기지 않고 총괄에 올린다.
- 원자료: 승격(#424 머지) 뒤 ws1 원본을 보관하고 지웠다(2026-11-11, md5 일치 확인).
  - `gdrive:ws1-rv4-54bd84a9-final/`
    - `rv4_run.tar.zst`: `test_runs/live/rv4_54bd84a9/` 전체(판정 입력, 세션 기록, 인식 프레임·영상 포함, 롤링 보관 없음), 27,022 항목, md5 97b20c1f…
    - `rv4_snap.tar.zst`: `snapshots/rv4_54bd84a9/`, 2,326 항목, md5 c25b67ee…
    - `archive_map.txt`·`PAIRS.txt`: 두 단위의 짝과 복원 순서(rv4_snap → rv4_run, 둘 다 `-C /home/projects/forklift`).
