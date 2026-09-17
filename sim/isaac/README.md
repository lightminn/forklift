# `sim/isaac/`

Isaac Sim work for this project. The engine decision that put it here is still
open — see the decision list in
[검토 수렴 기록](../../docs/validation/2026-09-17-dual-pallet-review-convergence.md).
Nothing in this directory is an approved milestone baseline.

## What is here

| 파일 | 하는 일 |
|---|---|
| `determinism_probe.py` | 같은 장면의 깊이 영상이 프로세스를 새로 띄워도 비트 단위로 같은지 잰다 |

## 실행

```bash
/opt/isaacsim/python.sh sim/isaac/determinism_probe.py --out /tmp/det --tag runA
/opt/isaacsim/python.sh sim/isaac/determinism_probe.py --out /tmp/det --tag runB
# 비교: 두 JSON 의 captures[].sha256 이 같은가
```

⚠️ **인터프리터 경로가 실험의 일부다.** 원격에 Isaac 설치가 두 벌 있고 `VERSION`
문자열이 `5.1.0-rc.19+release.26219.9c81211b.gl` 로 **똑같은데 한쪽만 돈다** —
`/opt/isaacsim` 은 실행되고 `~/isaacsim` 은 `isaacsim.core.api` import 에서
`typing_extensions` 없음으로 죽는다. 결과를 적을 때 경로를 같이 적는다.

## 2026-09-17 측정

원격 `kang-MS-7D77`, RTX 5070 12227 MiB, driver 580.126.09, `/opt/isaacsim` 5.1.0-rc.19.

| | waited | sha256 (앞 24) | finite | sum |
|---|---|---|---|---|
| runA first | 2 | `36e768b2b5ea2dba2329c522` | 135360 | 780902.875 |
| runA second | 0 | 같음 | 135360 | 780902.875 |
| runB first | 2 | 같음 | 135360 | 780902.875 |
| runB second | 0 | 같음 | 135360 | 780902.875 |

**실행 안에서도, 프로세스를 새로 띄워도 비트 단위로 같았다.** 채워지기까지 걸린
스텝 수까지 두 실행이 일치했다.

### 이 결과가 덮지 않는 것

같은 기계·같은 GPU·같은 드라이버·같은 빌드에서, **정지한 장면**을 기본 렌더러로
찍은 결과다. 다음은 **다시 재야 한다.**

- **기계 간 재현성** — 개발 PC 는 RTX 5070 Ti, 원격은 5070 이다. 둘이 갈리면
  누가 돌렸는지에 따라 수치가 달라진다. **팀에 제일 중요한 미측정 항목이다.**
- 움직이는·물리 구동 장면. NVIDIA 문서가 물리 재개의 비결정성을 명시한다.
- `PathTracing` 렌더러.
- `SingleViewDepthSensor` 같은 **잡음 경로** — 자체 RNG 가 들어간다.

### 왜 쟀는가

`tools/scene_rig.py` 를 남길 근거 중 하나가 "순수 numpy 라야 비트 재현이 된다"
였는데 **이 측정이 그것을 반증했다.** Isaac 도 재현한다. `scene_rig` 를 남길
근거로 남은 것은 ① GPU·EULA 없이 도는 CPU CI ② 기존 ADR 0003 수치가 그 리그의
양자화·좌표 모델 위에 있다는 이행 비용, 둘뿐이다.
