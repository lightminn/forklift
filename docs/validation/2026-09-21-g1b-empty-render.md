# G1b 교정판 빈 프레임: clipping 진단과 조건부 수정

## 확인 범위

기준 HEAD `6f9fb82` + 기존 G1 미커밋 변경 위에서 작업했다. Isaac/Kit은 실행하지 않았다.
기존 원격 산출물 JSON과 설치된 Camera 소스는 SSH로 읽었고, 새 렌더 검증은 실행자가 한다.
**이 변경의 CPU 시험 통과는 교정판 렌더 복구나 G1 통과를 뜻하지 않는다.**

기존 실행: `artifacts/20260921T034500Z_g1_calibration/experiment/`.
`board_r0.8_p0_repeat0_capture.json`의 timeline은 전후 모두
`0.5083333598449826 s`, render product의 camera target도 올바른 경로였다.
이는 시간 이동·잘못된 camera 연결을 뒷받침하지 않는다. 당시 product 유효성,
playing/stopped 상태, clipping range는 저장되지 않아 사후 확정할 수 없다.

## 발견한 구체적 결함과 남은 확인

- 마커 측정 후 `hide_existing_geometry()`가 기존 Gprim을 모두 숨긴다.
  따라서 배경이 사라진 상태에서 교정판까지 clip되면 RGB 전체 검정·depth 전체 비유한이 된다.
  이 관측만으로 렌더 파이프라인 정지를 확정할 수 없다.
- 첫 판의 정점을 기존 `checkerboard_layout()`과 nominal mount로 재계산하면
  모든 정점의 optical Z는 약 **0.72335893 m**다. 보드 중심의 0.8 m는 optical Z가 아닌
  카메라 기준 수평 거리다.
- 확인한 설치본 `isaacsim.sensors.camera/camera.py`의 생성자는 새 USD Camera prim을
  만들며 clipping을 별도 설정하지 않는다. `get_clipping_range()`는
  prim의 `clippingRange`를 직접 읽고 setter도 해당 속성을 쓴다.
  현재 G1 코드도 clipping을 설정하지 않았다.
- [USD Camera 정의](https://openusd.org/dev/api/class_usd_geom_camera.html)의 기본 near는
  **1 stage unit**이다. 로컬 USD 0.26.5에서 metre stage의 새 Camera를 직접 만들었을 때도
  unauthored `clippingRange = (1, 1000000)`을 확인했다. 이는 USD CPU 확인이며 Isaac 렌더가 아니다.
  1 m near라면 첫 판 전체가 잘린다는 계산은 확정이다.

사용자가 제시한 마커 depth 최솟값 1.00 m와 일치하지만, 이전 실행의 실제 clipping readback은
없다. 따라서 **다음 실행의 readback과 near 변경 전후 캡처로 렌더 원인을 확인한다.**
타임라인 재생·정지, render product 재생성, 임의 대기 증가를 복구책으로 넣지 않았다.

## 구현 동작

1. 모든 준비 스텝에서 RGB mean/std/max·비영 값 수, depth 유한 비율·최솟값·최댓값을 저장한다.
   RGB의 alpha는 무시한다. RGB 0 + depth 전체 비유한이면 `empty_render_frame`을 우선 보고한다.
   segmentation의 원시 상태는 별도 보존하며 RGB만으로 라벨을 복원하지 않는다.
2. 스텝 전후 timeline time/playing/stopped, orchestrator 상태, render product 경로/유효성/
   active/resolution/camera targets, 원래 annotator 연결 경로와의 일치 여부, clipping을 기록한다.
   읽지 못한 API는 `readback_errors`에 남긴다. USD product가 valid여도 Hydra 정상의 증명은 아니다.
3. 마커 완료 → 기존 geometry 숨김 → 마커 삭제 → 판 Mesh → 각 GeomSubset → Border 사이의
   상태를 `render_pipeline_trace.json`에 기록한다. 이 관찰은 추가 render step을 만들지 않는다.
4. composed 정점 전체가 실제 near보다 앞이고 카메라 전방에 있는 판에 한해
   `_before_near_clip` 이름으로 기존 조건의 캡처를 먼저 저장한다.
   이후 **near = 최소 optical Z / 2**, far는 유지하며 setter readback을 확인한다.
   첫 판에서 near가 1 m였다면 약 0.36167947 m가 된다. 위치·K·장착·측정 허용치는 그대로다.
   변경 뒤의 정상 이름 캡처만 기존 G1 측정으로 전달한다.
5. 수정 후에도 빈 프레임이면 새 annotator를 붙여 기존·새 스트림을 **같은 스텝**에서 8회 비교한다.
   새 바인딩도 내부 renderer 노드를 공유할 수 있으므로 `both_nonempty`는 재연결 필요성의 증명이 아니다.
   진단용 프레임은 G1 성공 측정으로 쓰지 않으며 원래 실패를 유지한다.

## 다음 실행의 판독

기존 실행 명령에서 `--output`만 새 디렉터리로 바꾼다. 별도 옵션은 필요 없다.

| 산출물 | 확인할 내용 |
|---|---|
| `marker_2_capture.json` | 기존 정상 RGB-D 통계와 스텝별 pipeline 상태 |
| `render_pipeline_trace.json` | 숨김·삭제·Mesh·Subset 생성 경계의 상태 차이 |
| `board_r0.8_p0_repeat0_before_near_clip_*` | 조건부 수정 전 JSON·RGB·원본 depth/segmentation |
| `board_r0.8_p0_repeat0_*` | 수정 후 같은 판의 JSON·RGB·원본 depth·target mask |
| `result.json`의 `near_clip_experiments` | 실제 clipping/정점 Z, 변경 전후 통계, 복구 관측 |
| `*_attachment_probe.json` 및 PNG/NPZ | 계속 비었을 때 기존/새 바인딩의 동시 비교 |

`capture_diagnostics.attempts[*].render`와 `empty_render_steps`로 몇 번째 스텝까지 비었는지 본다.
`target_depth_finite_fraction`은 target mask 안의 depth 유한 비율이다.
배경을 숨긴 판 프레임의 **전 화면 depth 유한 비율은 마커의 98.5 %와 같을 필요가 없다**.
판의 라벨·RGB·depth와 기존 corner/depth/fit 게이트를 함께 확인해야 한다.

`near_clip_experiments.observation = target_recovered_after_near_only_change`는
near-only 변경 후 라벨과 비영 RGB, target 내 유한 depth가 관측됐다는 뜻이다.
G1 전체 통과를 뜻하지 않으며 이후 기존 측정 게이트는 그대로 적용한다.
near가 충분히 작았다면 수정 전 진단 캡처·clipping 변경 자체를 생략한다.

## 로컬 검증

- 변경 전 전체 기준선: **1,106 passed**.
- RED 확인: 빈 프레임/스텝 기록 8 failed, clipping/pipeline 8 failed,
  전후 캡처 3 failed, 라벨만으로 RGB-D 복구를 주장하는 반례 1 failed.
- 관련 시험: **45 passed**. 가짜 annotator와 CPU 정점 계산 사용.
- 전체 시험 명령:

  ```sh
  python -m pytest tests --ignore=tests/simulation -q -p no:cacheprovider -W error
  ```

전체 결과: **1,126 passed in 109.81s**, 기준선 대비 20개 추가, 실패·skip 없음.
Ruff lint/format check와 `git diff --check`도 통과했다.
기존 문서 세 개, `omni`, `run_transport.py`의 작업 전후 SHA256이 같다.
G1 protocol/허용치, 기존 marker centroid/semantic 준비 조건, 원래 orchestrator step,
mount와 최종 gate 함수도 보존했다. 커밋하지 않았다.
