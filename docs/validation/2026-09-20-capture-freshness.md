# 캡처 신선도 계약: CPU 검증

이 변경은 `SensorCapture`를 관측 지점들 사이에 유지하여 마지막 승인 프레임과 새 호출의
첫 시도를 비교한다. 전역 가변 상태는 없다. 원시 깊이 통계 `FrameDiagnostics`는 보존하고,
별도 `CaptureDiagnostics`에 센서 ID, 획득 시각, 물리 시각 전후, world 위치(m)와
wxyz 자세 전후, 시도 횟수와 거절 사유별 횟수를 기록한다. 실패 예외에도 진단을 붙인다.

- transport는 `Camera.get_current_frame()`의 `rendering_frame`을 식별자로,
  `rendering_time`을 획득 시각으로 사용한다. 물리 시각은 별도 기록이다.
- RGB/depth는 기존 직접 getter를 유지하고, 두 getter 전후 센서 메타데이터가 같아야 한다.
  센서 메타데이터가 없거나 지원하지 않는 형식이면 `CaptureFailure`로 종료한다.
- 자세·물리 시각은 렌더 스텝 직전부터 픽셀 읽기 직후까지 감싼다. 획득 시각이 이 구간 밖이면
  거부한다. 이동 1 mm 또는 회전 0.001 rad를 초과하면 거부한다. 허용된 구간의 사후 자세를
  검출 결과의 world 변환에 사용하며, 캡처 루프 뒤에서 자세를 새로 읽지 않는다.
- 이 계약은 단일 스레드에서 스텝을 진행하는 정지 관측의 **끝점 오차 제한**이다.
  구간 내 왕복 운동이나 정확한 획득 순간 자세를 복원하는 계약은 아니다.
- 일반 변환 함수 `capture_scene_input`은 `CaptureState`를 전달하면 호출 간 검사를 한다.
  상태를 생략한 호출은 일회성 변환이다. 센서 ID가 없는 일반 경로는 RGB와 depth 모두
  바뀌어야 승인하며, 호출 간에도 재사용 허용보다 정적 장면의 거짓 기각을 택한다.
  transport의 엄격한 경로는 메타데이터 누락 시 배열 비교로 자동 강등하지 않는다.

## API 근거와 검증 한계

[공식 Isaac Sim v5.1.0 Camera 소스](https://github.com/isaac-sim/IsaacSim/blob/v5.1.0/source/extensions/isaacsim.sensors.camera/isaacsim/sensors/camera/camera.py)의
`_data_acquisition_callback`은 fabric-time 딕셔너리를 `rendering_frame`에 저장하고,
`get_sim_time_at_time` 결과를 `rendering_time`에 저장한다. 기존 정수 ID 형식도 지원한다.
직접 getter는 동일 카메라의 annotator를 읽는다. 메타데이터 전후 비교는 SDK 내부에서
서로 다른 annotator가 잘못된 데이터를 내놓는 것까지 증명하지는 않는다.

요청된 `/opt/isaacsim-env` 설치본은 이 실행 환경에 없어 직접 확인하지 못했다.
실제 설치본 필드·시계 정합·annotator 대응은 미검증이며, Isaac 실행은 수행하지 않았다.
실제 버전에서 필드가 다르거나 렌더가 구간 밖 시각을 보고하면 의도적으로 실패한다.
그 경우 설치본에 맞는 센서 메타데이터 매핑 또는 획득 콜백에서 기록한 자세 이력이 필요하다.
물리 시각으로 센서 ID를 대체해서는 안 된다.

## 시험

기존 코드에서 고정 프레임의 두 번째 호출이 실패하지 않는 반례를 ID/배열 두 경로로 확인했다.
기존 시험 56개는 통과했고 추가 시험 8개는 실패했다(그중 6개는 신규 API 부재).
구현 후 기존 기대값 변경 없이 호출 간 재사용, 새 ID, 가변 ID 복사, 메타데이터 누락,
획득 시각 이탈, 읽기 중 ID 변경, 이동·회전·잘못된 자세를 가짜 카메라로 검사한다.
이는 CPU 계약 검증이며 렌더 신선도의 실측 증거가 아니다.

최종 결과: 어댑터 68개 통과, 전체 명령
`python -m pytest tests --ignore=tests/simulation -q -p no:cacheprovider -W error`
**916 passed (113.59 s)**. 수정한 Python 세 파일의 Ruff lint·format check와
`git diff --check` 통과. 기존 시험 함수·클래스는 HEAD와 AST 동일함을 별도로 확인했다.
conda base Python의 패키지를 공유하는 임시 가상환경에 editable 설치하여 실행했다.
