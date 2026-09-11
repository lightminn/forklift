# Gazebo 센서 영상 — 2026-09-11

- 원본: `20260910T145835Z_gazebo_sensors_final`의 ROS bag. 원본 SHA-256을 이전 수집 기록과 대조한 뒤 읽기 전용으로 추출했다.
- 영상: [MP4 영상](../../artifacts/20260910T151053Z_sensor_video_01/forklift_sensor_replay.mp4), H.264 / 1920×1080 / 5fps / 162프레임 / 32.4초.
- 동기화: RGB·depth·LiDAR의 같은 header timestamp 162세트를 사용했다. 시뮬레이션 시각 2.0–34.2초, 표본 시간 범위 32.2초이며 마지막 프레임 표시 0.2초를 포함한 MP4 길이는 32.4초다.
- 초기 렌더 warmup [0, 2)초만 제외했다. 노이즈 없는 정지 장면이며 RGB 원본 162개가 모두 동일하다. 이동이나 인식을 추가하지 않았고 영상에서도 합성·정지 상태를 표시한다.
- ffprobe 프레임 수·길이·포맷 확인, ffmpeg 전체 디코딩 오류 0, 인코딩된 처음·중간·끝 프레임(0/81/161) 직접 시각 확인을 마쳤다.
- `extract_bag.py`는 기존 Jazzy 컨테이너에서 ROS 메시지를 역직렬화한다. `render_video.py`는 추출 NPZ를 읽어 센서 영상과 시간 표시를 합성하며 실행 시 같은 이름의 MP4가 있으면 덮어쓰지 않는다. 두 스크립트는 `/input`·`/output`·폰트 경로가 고정된 1회용이며 Git 추적 대상이 아니다. 로컬 `artifacts/20260910T151053Z_sensor_video_01/`과 원격 팀 작업 공간의 같은 실행 ID 디렉터리에 보존한다(2026-09-11 확인). bag→영상 도구가 다시 필요하면 로드맵 M1-b에서 인자를 받는 도구로 설계한다.
- 재현·검증: `extraction.json`, `ffmpeg_command.json`, `ffmpeg_encode.log`, `video_verification.json`. 원본 bag과 프로젝트 실행 코드는 변경하지 않았다.

산출물 디렉터리: `artifacts/20260910T151053Z_sensor_video_01/`.
