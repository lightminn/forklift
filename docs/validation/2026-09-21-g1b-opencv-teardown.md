# G1b OpenCL 비활성화와 annotator 종료 처리

- 기준: HEAD `2671053`에 기존 primitive·반 픽셀 미커밋 변경이 있는 작업 트리.
- 사용자 제공 장애 기록: `20260921T060000Z_g1_halfpixel`에서 SB 코너 검출의
  OpenCL 할당 실패와 C++ 소멸자 예외로 종료. 이전 실행에서는 측정 완료 후
  `Annotator rgb is not attached to any render products` 발생.
- 이번 검증은 로컬 CPU 시험이다. 해당 원격 산출물을 직접 읽거나 Isaac을
  실행하지 않았으며, render product 재생성이 분리 장애의 원인인지는 미확정이다.

## 변경

- G1 `run()` 진입 시 같은 측정 스레드에서 `cv2.ocl.setUseOpenCL(False)` 실행.
  `result.json`에 `opencv_version`과 `opencv_opencl`의
  `requested_use_opencl`, `have_opencl`, `use_opencl`을 측정 전에 저장한다.
  실제 `useOpenCL()`이 여전히 참이면 측정을 시작하지 않는다.
- 정상 완료와 Python 예외 모두에서 결과를 먼저 저장하고 annotator를 정리한다.
  카메라는 정리가 끝날 때까지 유지한다. 일부 채널만 연결된 경우도 정리 대상이다.
- [Replicator 공개 API](https://docs.omniverse.nvidia.com/kit/docs/omni_replicator/1.12.16/source/extensions/omni.replicator.core/docs/API.html)의
  `is_attached`가 거짓이면 분리를 건너뛴다. 연결된 객체는 인자 없는 `detach()`로
  자신이 보유한 연결을 해제한다. 카메라의 현재 product 경로를 재조회하지 않는다.
  처리한 객체는 소유 목록에서 제거하여 재호출 시 중복 분리하지 않는다.
- `annotator_cleanup`에는 채널별 `detached`, `already_detached`, `error`를 기록한다.
  예기치 않은 정리 예외도 남기며 다른 채널의 정리는 계속한다. 진단용 임시
  annotator에도 같은 처리를 적용한다. 수치 게이트 판정은 바꾸지 않는다.

## 로컬 검증

- 새 회귀 시험 10건: 실패 확인 후 구현하여 통과. OpenCL 유무·비활성화 읽기 확인,
  상태의 사전 저장, 정상/예외 정리, 중복 분리, 정리 오류, 부분 연결과 카메라 수명 포함.
- 실제 로컬 OpenCV `4.13.0` CPU smoke: 합성 640×480 교정판의 63개 코너 검출.
  저장된 상태는 `have_opencl=false`, `use_opencl=false`. 원격 `4.11.0` 실행 증거는 아니다.
- 변경 Python 파일 3개의 Ruff lint와 format check 통과.
- 전체 시험 명령: `python -m pytest tests --ignore=tests/simulation -q -p no:cacheprovider -W error`.
  최종 결과는 **1,150 passed in 169.21s**. 기준선 1,140건과 신규 10건이 모두 통과했다.
- 기존 수치 측정 함수, 판정 함수와 측정 호출 인자는 AST 비교로 동일함을 확인했다.
  별도 기존 미커밋/미추적 파일 11개의 SHA-256이 그대로다. `omni`는 읽거나 수정하지 않았다.

Isaac에서의 크래시 해소와 실제 종료 상태는 사용자 원격 재실행으로 확인해야 한다.
교정 로직·허용치·반 픽셀 변환은 변경하지 않았고 커밋하지 않았다.
