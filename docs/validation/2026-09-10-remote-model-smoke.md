# 원격 지게차 모델 테스트런

검증일: 2026-09-10. **원격 MuJoCo 환경 설치, CPU 시험 80개, 물리 적분과 소프트웨어 렌더 영상 생성·회수·대표 프레임 확인 완료. NVIDIA EGL 및 Gazebo 통합은 미검증.**

실행 ID: `20260910T125341Z_remote_model_smoke_01`. 원본 기록은 Git 제외 `artifacts/20260910T125341Z_remote_model_smoke_01/`에 있으며 원격 팀 작업 공간에도 같은 ID로 보관했다. 개인 SSH 별칭·계정·환경 절대 경로는 개인 환경 기록과 실행 당시 `context.json`에 둔다.

## 실행 환경과 소스

- 기존 연구 환경과 분리한 새 Python 3.11.16 환경에 MuJoCo 3.10.0, NumPy 2.4.6, pytest 9.1.0, PyYAML 6.0.3, Pillow 12.2.0을 설치했다.
- `pyproject.toml`의 `test`·`model` 의존성을 해결하고 Linux x86_64/Python 3.11 대상 wheel 16개를 내려받아 파일별 SHA-256으로 고정했다. 원격 설치는 `--no-index --find-links ... --require-hashes`로 수행했다. 플랫폼 전체 패키지를 고정한 범용 lock은 아니다.
- 프로젝트 wheel 설치 후 `pip check`와 source 밖 `forklift_core` import가 성공했다. `site-packages` 경로를 확인했다.
- 기준 revision: `fc3c1d91e93d09e15d9e3f01339073e3490ae26d`. 시작 시 문서 3개가 미커밋이었고 실행 소스에 포함하지 않았다.
- 코어·모델·시험·도구·`pyproject.toml` 21개 파일을 새 원격 스냅샷으로 전달하고 읽기 전용으로 고정했다. 각 job이 시작할 때 파일 해시를 확인했다.
- 소스 manifest SHA-256: `db9410ab39103bdcfbcb3fbfc19fae679805eb2bc94829c0af666ac8f75ebda2`.
- dependency lock SHA-256: `71c8b9347eb9be92143130487624401af57cc1cc8c752092588ecca52c434834`.
- 프로젝트 wheel SHA-256: `fd8a67b3e8daf92ec713529b86fe083ff8096fc322c579be867c53216b78d545`.

## Slurm 실행 결과

CPU 단계는 2 CPU·4GiB를 요청했다. GPU 단계는 같은 CPU·메모리에 GPU 1개를 추가했다. 기존 연구 작업의 자원 점유를 고려한 이번 실행의 요청값이며 성능 최적값은 아니다.

| 단계 | Job | Slurm 결과 | 실제 확인 |
|---|---|---|---|
| 환경 준비 | 962 | COMPLETED, exit 0:0, 52초 | 전용 환경·hash 고정 설치·wheel import·pip check 성공 |
| CPU 검사 | 963 | COMPLETED, exit 0:0, 2초 | **80 passed, 1 deselected**, 실패·오류·skip 0개; 물리 적분과 demo 성공 |
| CPU 렌더링 | 964 | COMPLETED, exit 0:0, 16초 | OSMesa/llvmpipe 영상 96프레임 및 PNG 생성 |
| GPU 검사 | 965 | CANCELLED, 실행 시간 0초 | 자원 대기 중 이번 작업만 취소; 전체 81개 및 NVIDIA EGL 미실행 |

GPU 작업의 예상 시작은 9월 11일 20:18이었고 이번 검사의 완료 기한은 9월 10일 22:33이었다. 9월 10일 22:10에 대기 상태를 확인하고 취소했다. 기존 실행 작업과 대기열은 보존했다. Accounting이 비활성화되어 완료 직후 `scontrol show job`을 기록했고, 각 단계의 `result.json`·명령 종료 코드·실제 산출물도 대조했다. 취소된 작업의 `ExitCode=0:0`은 검사 통과를 뜻하지 않는다.

제출 후 일시적으로 `InvalidAccount`가 표시됐으나 CPU 렌더 작업은 설정 변경 없이 실행·완료됐고, GPU도 취소 직전에는 `Reason=None`이었다. 원인을 확정하지 않았으며 Slurm 설정을 수정하지 않았다.

## 물리 계산과 영상

검사 입력은 **합성**이다. 잠정 모델을 바닥보다 5cm 높게 시작해 `mujoco.mj_step`으로 2ms씩 적분했다. 난수는 사용하지 않았다.

- **비렌더링 물리 검사:** 10,000 step, 시뮬레이션 시간 20초. 최종 상태 유한값, MuJoCo 경고 0개, 최종 접촉 8개, 차체 속도 약 9.8e-16. 계산 시간 약 0.143초는 단순 무부하 장면만의 결과다.
- **영상 물리 검사:** 2,000 step, 시뮬레이션 시간 4초. 1–3초 구간에 기존 추정 승강 actuator 목표를 0.15m로 주고 나머지 구간은 0m로 지정했다. 기록된 포크 위치 범위는 약 −0.0030–0.1445m다. 최종 상태 유한값, 경고 0개, 접촉 8개를 확인했다.
- **렌더러:** `llvmpipe (LLVM 20.1.2, 256 bits)` / OSMesa. GPU 가속 결과가 아니다. 4초 영상의 물리 계산·렌더·인코딩에 약 14.69초가 걸렸다.
- **영상 파일:** `cpu_render/physics/physics_smoke.mp4`, H.264, 960×640, 24fps, 96프레임, 4.000초. `ffprobe`로 메타데이터를 확인하고 노트북에서 `ffmpeg -v error -i <video> -f null -` 전체 디코딩을 통과했다.
- **직접 본 대표 프레임:** `frame_000.png`, `frame_060.png`, `frame_095.png`. 모델이 프레임 안에 보이고 바닥 안정화, 포크 상승·하강과 합성 입력 안내 문구를 확인했다. 이는 영상 전체를 실시간 플레이어로 시청한 검사는 아니다.

기존 `tools/preview_forklift_model.py`는 관절 자세를 지정하는 운동학 미리보기다. 이번 영상은 실행 기록의 별도 `physics_probe.py`에서 실제 적분한 결과다. 추정 질량·접촉·승강 구동 파라미터를 사용하므로 실물의 하중 능력·모터 성능·주행 안정성을 입증하지 않는다.

## 재현 명령과 남은 작업

실제 절대 경로와 argv는 각 단계 `result.json`에 있다. 아래 변수는 실행 환경의 명시적 Python, 고정 소스, 이번 실행 기록, 새 출력 디렉터리를 가리킨다. 원격 계산은 Slurm으로 할당받은 자원 안에서 실행한다.

```bash
"$FORKLIFT_PYTHON" -m pytest "$FORKLIFT_SNAPSHOT/tests" \
  -m 'not rendering' -q -p no:cacheprovider -W error
"$FORKLIFT_PYTHON" "$FORKLIFT_RUN/physics_probe.py" \
  --model "$FORKLIFT_SNAPSHOT/sim/models/dls08_provisional/scene.xml" \
  --output "$FORKLIFT_NEW_OUTPUT" --backend osmesa
```

`requirements-model-py311.txt`, wheelhouse, `job.sbatch`, `run_stage.py`, `physics_probe.py`는 이번 실행 기록에 보존했다. `tools/submit_model_check.py`, `requirements/model_py311.txt`, `deploy/slurm/`의 표준화된 제출·재현 도구는 아직 구현하지 않았다. GPU 할당 후 NVIDIA EGL과 전체 시험 81개를 다시 검증해야 한다.

D435i/RPLIDAR 센서 출력, ROS–Gazebo 통합, 팔레트 인식·삽입·적재, A–D 자율주행 시나리오, GUI 원격 조작과 실물 검증은 이번 테스트런에 포함하지 않았다.
