# DLS08 잠정 지게차 모델

상품 사진과 카탈로그에서 외형이 대응하는 후보를 모델링했다. 전체 크기
1.46 × 0.63 × 1.01m와 순중량 24kg은 후보 카탈로그 값이며, 실제 구매 차체의
SKU·부품 치수·조향·구동·승강 성능은 수령 후 확인해야 한다.

- `parameters.yaml`: 후보 사양과 추정 부품 치수·동역학 가정의 정본
- `forklift.xml`, `scene.xml`: MuJoCo 모델과 바닥 장면
- `forklift.urdf`: 형상·관절 교환용 URDF
- `model_manifest.json`: 생성 입력 해시, 출처 분류, 충돌·구동 가정

원본 자료의 URL·해시는 [출처 기록](../../../docs/references/dls08/README.md),
시험과 대표 이미지는 [모델 검증 기록](../../../docs/validation/2026-09-10-product-forklift-model.md)에 있다.

저장소 루트에서 설치하고, 새 결과 디렉터리에 생성한다.

```bash
python -m pip install -e '.[dev,model]'
MODEL_OUTPUT="artifacts/$(date -u +%Y%m%dT%H%M%SZ)_dls08_model_01"
python tools/build_forklift_model.py \
  --parameters sim/models/dls08_provisional/parameters.yaml \
  --output "$MODEL_OUTPUT"
python tools/preview_forklift_model.py \
  --model "$MODEL_OUTPUT/scene.xml" \
  --output "${MODEL_OUTPUT}_preview" --backend egl --frames 96
```

미리보기에는 EGL 그래픽 문맥과 ffmpeg가 필요하다. 영상은 관절 자세를 지정한
운동학 미리보기다. MJCF는 자기 충돌을 제외하며, URDF의 접촉·관절 구동은
가져오는 엔진에서 별도 설정해야 한다. 두 형식의 물리 동등성, 센서 시뮬레이션,
팔레트 적재 또는 자율 주행 성공을 검증한 모델은 아니다.
