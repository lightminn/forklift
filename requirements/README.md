# 재현용 Python lock

`model_py311.txt`는 CPython 3.11 / Linux x86_64 원격 모델 검사 전용 lock이다.
프로젝트 전체 머신 환경을 `pip freeze`한 파일이 아니다. `pyproject.toml`의 기본
의존성과 `test`, `model` extra를 2026-09-10에 해결한 뒤 실제 내려받은 wheel 하나씩의
SHA-256을 기록했다.

최초 해결 입력과 원본 wheel은 실행 기록
`artifacts/20260910T125341Z_remote_model_smoke_01/`에 보관했다. 입력 제약은 NumPy
2.4.6, pytest 9.1.0, MuJoCo 3.10.0, PyYAML 6.0.3, Pillow 12.2.0이었고,
`pip-compile --extra=model --extra=test` 결과에서 Linux x86_64 / CPython 3.11용
wheel을 선택해 해시를 고정했다. lock을 갱신할 때도 같은 대상에서 새 wheel을 별도
디렉터리에 다운로드하고 각 파일 해시를 확인한 뒤, 이 설명의 생성일과 입력 제약을
함께 갱신한다.

이미 준비된 Python 3.11 환경에는 다음처럼 설치한다.

```bash
python -m pip install --require-hashes -r requirements/model_py311.txt
python -m pip check
```

프로젝트 자체 wheel 또는 source snapshot은 이 lock과 별도로 설치·검증한다. 이
lock을 ARM64 Jetson, 다른 Python minor, ROS/Gazebo 컨테이너에 재사용하지 않는다.
