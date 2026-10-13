# 콘솔 C1 — 웹 앱과 가짜 차량 백엔드 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `console/1` 계약 초안대로 말하는 웹 콘솔(`console/web/`)과, 같은 계약을 구현한 가짜 차량 서버(`console/mock_vehicle/`)를 만든다. 실차·Isaac 없이 노트북 하나에서 콘솔의 전체 흐름(권한·모드·정지·재개·텔레옵·영상 신선도·작업)을 돌려 볼 수 있게 한다.

**Architecture:** 가짜 차량 서버는 Python `asyncio` + `websockets`로 만든다. 순수 상태 기계(`core.py`, 시계 주입)와 입출력(`server.py`)을 나눠서, 계약 규칙을 단위시험으로 고정한다. 웹 앱은 빌드 단계 없는 ES 모듈이다. 프로토콜·입력·표시 추적 로직은 순수 모듈로 두고 `node --test`로 시험하며, 화면은 v1 목업의 렌더링을 옮긴다. 이 가짜 서버는 **중재기(M-b)의 대체물이 아니다.** 콘솔을 개발하려고 계약을 흉내 낸 것이며, 그 규칙 시험은 공통 정책의 증거가 아니다.

**Tech Stack:** (웹 시험에는 Node ≥ 18이 필요하다. ROS 개발 컨테이너에는 없을 수 있으므로 README에 적는다) Python ≥ 3.10, `websockets` ≥ 13(`websockets.asyncio` API가 13.0에서 들어왔다), Pillow ≥ 10, NumPy; 브라우저 ES2020 모듈, Canvas 2D; Node ≥ 18 내장 테스트 러너(`node --test`).

**Spec:** [설계](../design/2026-10-13-operator-console.md), [계약 초안](../interfaces/2026-10-13-operator-console-contract-draft.md)

## Global Constraints

- `forklift_core/`와 `sim/isaac/`은 건드리지 않는다. runner의 `source_sha256` 범위와 측정 도구의 전체 소스 비교를 지키기 위해서다(설계 6절).
- 웹 앱 폴더를 만드는 변경에 `CONTRIBUTING.md` 1절 갱신을 같이 넣는다(설계 6절).
- 1단계는 `http`/`ws`만 쓴다. 인증·Origin 검증·암호화는 없다(설계 7.4, 사용자 결정).
- 브라우저는 시계를 차량과 비교하지 않는다. 신선도는 서버(가짜 차량)의 단조 시계로만 판정한다(계약 1·4절).
- 단위·이름은 CONTRIBUTING 3·5절을 따른다. 속도·곡률은 뒤 차축 중심 기준(`speed_mps`, `curvature_inv_m`)이다.
- 숫자는 모두 **잠정값**이고 상태의 `limits.provisional: true`로 표시한다: `command_max_age_s = 0.30`, `tick` 20 Hz(50 ms ≤ 0.30/4), `lease_timeout_s = 1.0`, `grace_s = 5.0`, `video_max_age_s = 0.50`, `camera_latency_bound_s = 0.05`, 전진 0.50 · 후진 0.25 m/s(적재 시 0.30 · 0.15), 곡률 상한 0.36 m⁻¹(실측 모델 추종기 값, `config/isaac_transport_measured.yaml`).
- 화면의 최소 회전 반경은 "모델 계산값 2.73 m(뒤 차축)"으로 표시한다(설계 5.4).
- AUTO 중 콘솔 연결이 끊겨도 작업을 계속한다. 텔레옵은 만료로 정지한다(사용자 결정).
- 적재 중 수동 조종은 운반 구간에서만 허용한다. 들기·내리기 중과 포크가 포켓 안이면 거부하고, 포켓 안이면 `withdraw_straight`만 허용한다(설계 4.5).
- 텔레옵 출력 순서: 비율 → 모드 상한 → 곡률 제한 → slew → 최종 허가 상한 → 안전 선점 0(설계 4.4).
- 새 Python 의존성은 `pyproject.toml`의 선택 의존성 `console`에만 넣는다. `websockets`가 없는 환경에서는 서버 통합시험이 skip 사유와 함께 건너뛴다(CONTRIBUTING 7절).

## Review Focus

1. **탭을 가리거나 다른 창을 누른 채 누르고 있기를 계속 누른 상태**: 차량이 멈추고 무장이 풀려야 한다. Task 6의 `test_visibility_hidden_releases_and_requests_disarm`과 Task 2의 `test_teleop_stops_when_commands_stop`.
2. **정지(STOP) 직후 늦게 도착한 재개·무장 요청**: 실행되지 않아야 한다. Task 2의 `test_late_resume_after_stop_is_rejected`.
3. **응답이 유실되어 같은 요청을 재전송**: 원래 결과를 돌려받고 두 번 실행되지 않아야 한다. Task 2의 `test_duplicate_request_returns_cached_result_without_rerun`, Task 5의 `test_stale_reject_is_not_auto_resent`.
4. **영상이 멈췄는데 영상 조종을 계속**: 영상 만료 시각에 멈춰야 한다. Task 2의 `test_video_deadline_is_earlier_of_two`.
5. **두 번째 브라우저 탭이 같은 콘솔을 열었을 때**: 관찰자로만 붙고 STOP은 보낼 수 있어야 한다. Task 3의 `test_second_client_is_observer_but_can_stop`.

---

## 파일 구조

| 파일 | 책임 |
|---|---|
| `console/README.md` | 무엇인지(가짜 차량, 계약 초안), 실행·시험 명령 |
| `console/mock_vehicle/__init__.py` | 빈 패키지 표시 |
| `console/mock_vehicle/world.py` | 가짜 작업장(벽·장애물·팔레트·하역 위치·대기 위치), 차체 상수, 레이캐스트, 이동 허가 흉내 |
| `console/mock_vehicle/paths.py` | Hermite 경로 조각(v1 목업의 `leg`)과 재생 |
| `console/mock_vehicle/core.py` | `MockVehicle`: 세대·lease·요청 처리 순서·모드·만료·텔레옵 출력 순서·작업 진행. 입출력 없음, 시계 주입 |
| `console/mock_vehicle/camera.py` | Pillow로 전방 카메라 JPEG 렌더(핀홀 투영) |
| `console/mock_vehicle/server.py` | `websockets` 서버: `/api/v1/control`, `/api/v1/media`, 정적 파일, tick·state·미디어 주기 |
| `console/mock_vehicle/__main__.py` | `python -m console.mock_vehicle --port 8765` |
| `console/web/index.html`, `console/web/app.css` | 화면 뼈대와 스타일(v1 목업에서 옮김) |
| `console/web/js/protocol.js` | 제어 연결 상태: 세대, 회신, 신뢰 요청(재전송·자동 재전송 금지), 텔레옵 명령 생성 |
| `console/web/js/inputs.js` | 터치·키보드·게임패드 입력 합치기, 가시성·포커스·패드 분리 해제 |
| `console/web/js/frames.js` | 표시한 프레임 추적(그린 뒤 다음 rAF에서 확정) |
| `console/web/js/media.js` | 미디어 연결: 바이너리 프레임 머리글 해석 |
| `console/web/js/map.js`, `console/web/js/camera.js` | 지도·카메라 캔버스 그리기 |
| `console/web/js/ui.js`, `console/web/js/main.js` | 패널 바인딩, 진입점 |
| `console/web/test/*.test.mjs` | `node --test` 시험 |
| `console/web/package.json` | `{"private": true, "type": "module"}` — `.js`를 ES 모듈로 읽게 한다(Node 18 기본은 CommonJS) |
| `tests/unit/console/test_mock_vehicle_core.py` | 계약 규칙 단위시험 |
| `tests/unit/console/test_mock_world.py` | 레이캐스트·허가·경로 |
| `tests/integration/console/test_mock_server.py` | 실제 WebSocket 왕복 |
| `pyproject.toml` | 선택 의존성 `console` |
| `CONTRIBUTING.md` | 1절 트리에 `console/`, 7절에 웹 시험 명령 |
| `docs/interfaces/2026-10-13-operator-console-contract-draft.md` | 6절에 표시용 `plan` 필드, 4절에 `echo` 메시지 명시 |

시험 실행(저장소 루트에서): `python -m pytest tests/unit/console tests/integration/console -q -p no:cacheprovider -W error`, `node --test console/web/test/`.

---

### Task 1: 뼈대, 의존성, 가짜 작업장

**Files:**
- Create: `console/README.md`, `console/mock_vehicle/__init__.py`, `console/mock_vehicle/world.py`, `console/mock_vehicle/paths.py`
- Modify: `pyproject.toml` (선택 의존성), `CONTRIBUTING.md` (1·7절), `docs/interfaces/2026-10-13-operator-console-contract-draft.md` (4·6절)
- Test: `tests/unit/console/test_mock_world.py`

**Interfaces:**
- Produces:
  - `world.Pose(x_m, y_m, yaw_rad)`(뒤 차축 중심), `world.Chassis`, `world.CHASSIS`(값은 Step 3 코드), `world.Pallet`
  - `world.World` (dataclass): `size_m: tuple[float, float]`, `circles: list[tuple[x, y, r]]`, `boxes: list[tuple[cx, cy, w, h]]`, `pallets: list[Pallet]`, `destinations: dict[str, Pose]`, `home: Pose`
  - `world.default_world() -> World`
  - `world.raycast(world, x, y, dx, dy, max_m, skip=()) -> float`
  - `world.clearance_ahead(world, pose, direction, curvature_inv_m=0.0, skip=()) -> float` — 진행 방향 앞쪽 최소 여유(m)
  - `world.loaded_nose_m(pallet) -> float` — 뒤 차축에서 실은 화물 앞면까지(`nose_from_rear_m − insert_depth_m + depth_m`)
  - `world.permission(world, pose, speed_mps, curvature_inv_m, load: Pallet | None, skip=()) -> tuple[float, str]` — (속도 상한, reason `ok` / `slow:<m>` / `occupied:<m>`). `load`가 있으면 폭은 `max(차체 반폭, 화물 폭/2)`, 앞쪽 끝은 `loaded_nose_m`로 검사한다
  - `paths.Leg`, `paths.hermite_leg(p0: Pose, p1: Pose, direction: int, speed_mps: float) -> Leg`, `paths.advance(leg, ds_m) -> Pose`, `paths.curvature_at(leg) -> float`

- [ ] **Step 1: 시험 작성** — `tests/unit/console/test_mock_world.py`

```python
import math

from console.mock_vehicle import paths, world


def test_raycast_hits_east_wall():
    w = world.default_world()
    assert math.isclose(world.raycast(w, 1.0, 0.5, 1.0, 0.0, 30.0), 13.0, abs_tol=1e-9)


def test_raycast_hits_barrel_before_wall():
    w = world.World(size_m=(14.0, 9.0), circles=[(5.0, 2.0, 0.3)], boxes=[], pallets=[],
                    destinations={}, home=world.Pose(1.0, 2.0, 0.0))
    assert math.isclose(world.raycast(w, 1.0, 2.0, 1.0, 0.0, 30.0), 3.7, abs_tol=1e-9)


def test_permission_stops_close_obstacle_and_slows_near_one():
    w = world.World(size_m=(14.0, 9.0), circles=[(3.0, 2.0, 0.3)], boxes=[], pallets=[],
                    destinations={}, home=world.Pose(1.0, 2.0, 0.0))
    nose = world.CHASSIS.nose_from_rear_m
    # fork tip 0.10 m from the barrel surface: stop
    limit, reason = world.permission(w, world.Pose(3.0 - 0.3 - 0.10 - nose, 2.0, 0.0), 0.3, 0.0, None)
    assert limit == 0.0 and reason.startswith("occupied")
    # 0.50 m away: slow
    limit, reason = world.permission(w, world.Pose(3.0 - 0.3 - 0.50 - nose, 2.0, 0.0), 0.3, 0.0, None)
    assert 0.0 < limit < 0.5 and reason.startswith("slow")


def test_loaded_footprint_is_wider_and_longer():
    load = world.Pallet("P", "epal6", world.Pose(0.0, 0.0, 0.0), 0.6, 0.8, state="carried")
    assert math.isclose(world.loaded_nose_m(load), 1.06 - 0.33 + 0.6)
    pose = world.Pose(1.0, 4.0, 0.0)
    box_x = 1.0 + world.loaded_nose_m(load) + 0.15 + 0.05  # 0.15 m ahead of the load, 0.40 m to the left
    w = world.World(size_m=(14.0, 9.0), circles=[], boxes=[(box_x, 4.40, 0.1, 0.1)], pallets=[],
                    destinations={}, home=pose)
    empty = world.permission(w, pose, 0.3, 0.0, None)
    loaded = world.permission(w, pose, 0.3, 0.0, load)
    assert empty[0] > 0.0  # the bare truck is narrower and shorter: at most slowed
    assert loaded[0] == 0.0 and loaded[1].startswith("occupied")


def test_straight_hermite_leg_is_straight_and_reaches_end():
    leg = paths.hermite_leg(world.Pose(0.0, 0.0, 0.0), world.Pose(2.0, 0.0, 0.0), 1, 0.3)
    assert math.isclose(leg.length_m, 2.0, abs_tol=1e-6)
    end = paths.advance(leg, 5.0)
    assert math.isclose(end.x_m, 2.0, abs_tol=1e-9) and math.isclose(end.yaw_rad, 0.0, abs_tol=1e-9)


def test_reverse_leg_keeps_heading_opposite_to_motion():
    leg = paths.hermite_leg(world.Pose(2.0, 0.0, 0.0), world.Pose(0.0, 0.0, 0.0), -1, 0.2)
    mid = paths.advance(leg, 1.0)
    assert math.isclose(mid.yaw_rad, 0.0, abs_tol=1e-6)
```

- [ ] **Step 2: 실패 확인** — `python -m pytest tests/unit/console/test_mock_world.py -q` → `ModuleNotFoundError: console.mock_vehicle`

- [ ] **Step 3: 구현** — `console/mock_vehicle/world.py`

```python
"""Mock bay for the console's fake vehicle (not a robot model, not a safety function).

Coordinates follow the map frame: x east, y north, metres; poses are rear-axle centres.
The permission here imitates the shape of DrivePermission's answer (limit, reason) so
the console can be exercised; it is not DrivePermission and proves nothing about it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Pose:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class Chassis:
    wheelbase_m: float = 0.66
    track_m: float = 0.53
    half_width_m: float = 0.315
    rear_overhang_m: float = 0.40  # rear bumper behind the rear axle (mock value)
    nose_from_rear_m: float = 1.06  # fork tip ahead of the rear axle (mock value)
    fork_length_m: float = 0.36
    insert_depth_m: float = 0.33  # dls08_measured insertion rule (CLAUDE.md)
    max_curvature_inv_m: float = 0.3665  # model value, both wheels at 15 deg (dls08_measured README)
    teleop_curvature_inv_m: float = 0.36  # tracker share in isaac_transport_measured.yaml


CHASSIS = Chassis()


@dataclass
class Pallet:
    pallet_id: str
    kind: str
    pose: Pose  # centre; yaw is the direction the truck faces when inserting
    depth_m: float
    width_m: float
    state: str = "available"  # available | carried | delivered


@dataclass
class World:
    size_m: tuple[float, float]
    circles: list[tuple[float, float, float]]
    boxes: list[tuple[float, float, float, float]]
    pallets: list[Pallet]
    destinations: dict[str, Pose]
    home: Pose
    carried: str | None = field(default=None)


def default_world() -> World:
    return World(
        size_m=(14.0, 9.0),
        circles=[(5.4, 2.9, 0.3), (6.0, 3.15, 0.3), (8.5, 7.9, 0.3), (11.6, 7.3, 0.3)],
        boxes=[(7.2, 7.2, 1.2, 0.8), (3.6, 7.9, 1.0, 0.8), (9.6, 3.4, 0.8, 1.0)],
        pallets=[
            Pallet("P-01", "epal6", Pose(12.3, 5.2, 0.0), 0.6, 0.8),
            Pallet("P-02", "t11_x0_6", Pose(12.3, 1.6, 0.0), 0.66, 0.66),
        ],
        destinations={"D-A": Pose(2.4, 1.6, math.pi), "D-B": Pose(2.4, 4.4, math.pi)},
        home=Pose(3.8, 6.2, 0.0),
    )


def _ray_box(ox, oy, dx, dy, cx, cy, hw, hh, yaw, best):
    cs, sn = math.cos(-yaw), math.sin(-yaw)
    lx, ly = (ox - cx) * cs - (oy - cy) * sn, (ox - cx) * sn + (oy - cy) * cs
    ldx, ldy = dx * cs - dy * sn, dx * sn + dy * cs
    t0, t1 = -math.inf, math.inf
    for o, d, h in ((lx, ldx, hw), (ly, ldy, hh)):
        if abs(d) < 1e-12:
            if o < -h or o > h:
                return best
            continue
        a, b = (-h - o) / d, (h - o) / d
        t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
    if t0 <= t1 and 0.0 < t0 < best:
        return t0
    return best


def raycast(world: World, x: float, y: float, dx: float, dy: float, max_m: float,
            skip: tuple[str, ...] = ()) -> float:
    """Distance along the unit direction (dx, dy) to the first wall, obstacle or pallet."""
    w, h = world.size_m
    best = max_m
    if dx > 1e-12:
        best = min(best, (w - x) / dx)
    if dx < -1e-12:
        best = min(best, -x / dx)
    if dy > 1e-12:
        best = min(best, (h - y) / dy)
    if dy < -1e-12:
        best = min(best, -y / dy)
    for cx, cy, r in world.circles:
        fx, fy = x - cx, y - cy
        b = fx * dx + fy * dy
        disc = b * b - (fx * fx + fy * fy - r * r)
        if disc >= 0:
            t = -b - math.sqrt(disc)
            if 0.0 < t < best:
                best = t
    for cx, cy, bw, bh in world.boxes:
        best = _ray_box(x, y, dx, dy, cx, cy, bw / 2, bh / 2, 0.0, best)
    for p in world.pallets:
        if p.pallet_id in skip or p.state == "carried":
            continue
        best = _ray_box(x, y, dx, dy, p.pose.x_m, p.pose.y_m, p.depth_m / 2, p.width_m / 2,
                        p.pose.yaw_rad, best)
    return best


def loaded_nose_m(load: Pallet) -> float:
    return CHASSIS.nose_from_rear_m - CHASSIS.insert_depth_m + load.depth_m


def clearance_ahead(world: World, pose: Pose, direction: int, curvature_inv_m: float = 0.0,
                    skip: tuple[str, ...] = (), load: Pallet | None = None) -> float:
    """Smallest free distance ahead of the travel edge (fork tip, load front or rear bumper)."""
    c = CHASSIS
    heading = pose.yaw_rad + (math.pi if direction < 0 else 0.0)
    half = max(c.half_width_m, load.width_m / 2) if load else c.half_width_m
    nose = loaded_nose_m(load) if load else c.nose_from_rear_m
    edge = c.rear_overhang_m if direction < 0 else nose
    # bend the probe rays toward the commanded turn (mock of the stop arc)
    bend = math.atan(curvature_inv_m * 0.8) * (1 if direction > 0 else -1)
    best = math.inf
    for off in (-half, 0.0, half):
        ox = pose.x_m + math.cos(heading) * edge - math.sin(heading) * off
        oy = pose.y_m + math.sin(heading) * edge + math.cos(heading) * off
        for da in (-0.12, 0.0, 0.12):
            a = heading + bend + da
            best = min(best, raycast(world, ox, oy, math.cos(a), math.sin(a), 5.0, skip))
    return best


STOP_M = 0.25
SLOW_M = 0.80


def permission(world: World, pose: Pose, speed_mps: float, curvature_inv_m: float,
               load: Pallet | None, skip: tuple[str, ...] = ()) -> tuple[float, str]:
    direction = -1 if speed_mps < 0 else 1
    free = clearance_ahead(world, pose, direction, curvature_inv_m, skip, load)
    cap = (0.25 if direction < 0 else 0.5) * (0.6 if load else 1.0)
    if free < STOP_M:
        return 0.0, f"occupied:{free:.2f}m"
    if free < SLOW_M:
        return min(cap, 0.12), f"slow:{free:.2f}m"
    return cap, "ok"
```

`console/mock_vehicle/paths.py`

```python
"""Hermite path legs for the mock mission (display and playback only)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from console.mock_vehicle.world import Pose


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Leg:
    points: list[tuple[float, float, float, float, float]]  # x, y, yaw, s, curvature
    direction: int
    speed_mps: float
    s_m: float = 0.0
    index: int = field(default=0)

    @property
    def length_m(self) -> float:
        return self.points[-1][3]


def hermite_leg(p0: Pose, p1: Pose, direction: int, speed_mps: float) -> Leg:
    d = math.hypot(p1.x_m - p0.x_m, p1.y_m - p0.y_m)
    k = max(1.1 * d, 0.4)
    t0 = (math.cos(p0.yaw_rad) * direction * k, math.sin(p0.yaw_rad) * direction * k)
    t1 = (math.cos(p1.yaw_rad) * direction * k, math.sin(p1.yaw_rad) * direction * k)
    n = max(16, math.ceil(d / 0.04))
    xy = []
    for i in range(n + 1):
        t = i / n
        a, b, c, e = 2 * t**3 - 3 * t**2 + 1, t**3 - 2 * t**2 + t, -2 * t**3 + 3 * t**2, t**3 - t**2
        xy.append((a * p0.x_m + b * t0[0] + c * p1.x_m + e * t1[0],
                   a * p0.y_m + b * t0[1] + c * p1.y_m + e * t1[1]))
    pts = [(xy[0][0], xy[0][1], p0.yaw_rad, 0.0, 0.0)]
    s = 0.0
    for i in range(1, len(xy)):
        (x0, y0), (x1, y1) = xy[i - 1], xy[i]
        ds = math.hypot(x1 - x0, y1 - y0)
        s += ds
        yaw = _wrap(math.atan2(y1 - y0, x1 - x0) + (math.pi if direction < 0 else 0.0))
        if i == len(xy) - 1:
            yaw = p1.yaw_rad
        kappa = _wrap(yaw - pts[-1][2]) / max(ds, 1e-9) * direction
        pts.append((x1, y1, yaw, s, kappa))
    return Leg(points=pts, direction=direction, speed_mps=speed_mps)


def advance(leg: Leg, ds_m: float) -> Pose:
    leg.s_m = min(leg.length_m, leg.s_m + max(ds_m, 0.0))
    pts = leg.points
    while leg.index < len(pts) - 2 and pts[leg.index + 1][3] < leg.s_m:
        leg.index += 1
    a, b = pts[leg.index], pts[min(leg.index + 1, len(pts) - 1)]
    u = (leg.s_m - a[3]) / (b[3] - a[3]) if b[3] > a[3] else 1.0
    return Pose(a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + _wrap(b[2] - a[2]) * u)


def curvature_at(leg: Leg) -> float:
    return leg.points[min(leg.index + 1, len(leg.points) - 1)][4]
```

`console/mock_vehicle/__init__.py`: `"""Fake vehicle for the operator console (console/1 draft); not robot software."""`

`console/README.md`:

```markdown
# 운용 콘솔

[설계](../docs/design/2026-10-13-operator-console.md) · [계약 초안 console/1](../docs/interfaces/2026-10-13-operator-console-contract-draft.md)

- `web/`: 브라우저 콘솔(빌드 없음, ES 모듈)
- `mock_vehicle/`: 계약 초안을 흉내 낸 **가짜 차량**. 실차·Isaac·중재기(M-b)가 아니다. 여기서 통과한 규칙 시험은 콘솔 개발용이며 차량 쪽 안전의 증거가 아니다.

## 실행

    python -m pip install -e '.[console]'
    python -m console.mock_vehicle --port 8765
    # 브라우저: http://localhost:8765/

게임패드는 브라우저가 보안 컨텍스트로 보는 `localhost`에서만 확실히 동작한다. 웹 시험에는 Node 18 이상이 필요하다.

가짜 차량의 한계: 정지 확인을 센서가 아니라 자기가 낸 적용 명령으로 하고(실차는 잡음 있는 피드백을 걸러야 한다), 모든 요청을 한 프로세스 안에서 즉시 처리한다(실차는 게이트웨이 → 중재기 → 미션 실행부가 비동기로 나뉘고, 결과 캐시의 주인은 중재기다). 이 차이는 C2·M-b에서 다룬다. 다른 기기에서 접속할 때는 `ssh -L 8765:localhost:8765 <호스트>` 뒤 `http://localhost:8765/`를 연다.

## 시험

    python -m pytest tests/unit/console tests/integration/console -q -p no:cacheprovider -W error
    node --test console/web/test/
```

`pyproject.toml` `[project.optional-dependencies]`에 한 줄 추가: `console = ["websockets>=13", "Pillow>=10"]`

`CONTRIBUTING.md` 1절 트리에서 `firmware/` 줄 앞에 추가:

```text
├── console/                          # 운용 콘솔: 웹 앱과 개발용 가짜 차량
│   ├── web/                          # 브라우저 콘솔(빌드 없음, ES 모듈, node --test)
│   └── mock_vehicle/                 # console/1 계약을 흉내 낸 가짜 차량 서버
```

7절 기본 검사 흐름 문장 뒤에 추가: "운용 콘솔의 웹 모듈은 `node --test console/web/test/`로 시험한다. 브라우저 화면은 자동 시험 범위 밖이므로 변경 시 렌더 확인을 남긴다."

계약 초안 4절 "회신 토큰" 문단 뒤에 추가: "lease 보유자는 `tick`을 받을 때마다 `{"t": "echo", "session_epoch", "lease_id", "echo": {"state_seq", "token"}}`를 보낸다(최신값만). 텔레옵 명령의 `echo`도 같은 역할을 한다." 6절 예시 아래에 추가: "표시용 계획 경로는 `plan: {"revision": n, "poses": [[x_m, y_m, direction], …]}`로, 바뀔 때만 싣는다. 안전 판단에 쓰지 않는다."

- [ ] **Step 4: 통과 확인** — `python -m pytest tests/unit/console/test_mock_world.py -q` → 6 passed

- [ ] **Step 5: 커밋** — `git add console/README.md console/mock_vehicle/__init__.py console/mock_vehicle/world.py console/mock_vehicle/paths.py tests/unit/console/test_mock_world.py pyproject.toml CONTRIBUTING.md docs/interfaces/2026-10-13-operator-console-contract-draft.md` → `console: mock bay, rays, permission imitation and Hermite legs for the fake vehicle (...)`

---

### Task 2: 가짜 차량 상태 기계 (`core.py`)

**Files:**
- Create: `console/mock_vehicle/core.py`
- Test: `tests/unit/console/test_mock_vehicle_core.py`

**Interfaces:**
- Consumes: Task 1 `world.*`, `paths.*`
- Produces:
  - `core.Config` (dataclass, 위 Global Constraints의 잠정값)
  - `core.MockVehicle(clock: Callable[[], float], token: Callable[[], str], config: Config | None = None, world: World | None = None, session_epoch: int | None = None)`
  - `MockVehicle.connect(client_id: str, conn_id: str | None = None) -> dict` (`welcome`), `disconnect(client_id: str, conn_id: str | None = None) -> None`. 서버는 WebSocket 연결마다 고유한 `conn_id`를 만들어 넘긴다. 단위시험은 생략할 수 있다(`None`은 "그 클라이언트의 유일한 연결"로 본다)
  - `MockVehicle.handle(client_id, msg, conn_id=None)`: 서버는 항상 `conn_id`를 넘긴다
  - `MockVehicle.make_tick() -> dict`, `make_state() -> dict`, `make_targets() -> dict`
  - `MockVehicle.handle(client_id: str, msg: dict) -> dict | None` (신뢰 요청은 `ack`/`reject`, 최신값 메시지는 `None`)
  - `MockVehicle.step(dt_s: float) -> None` (제어 주기)
  - `MockVehicle.record_frame(frame_id: int) -> None` (카메라 촬영 시각 기록)
  - 상태 필드 이름은 계약 6절 그대로

- [ ] **Step 1: 시험 작성** — `tests/unit/console/test_mock_vehicle_core.py`

```python
import itertools

import pytest

from console.mock_vehicle.core import Config, MockVehicle


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


@pytest.fixture
def clock_only():
    return Clock()


@pytest.fixture
def env():
    clock = Clock()
    tokens = (f"tok{i}" for i in itertools.count())
    v = MockVehicle(clock, lambda: next(tokens), Config())
    return v, clock


def _rid():
    _rid.n += 1
    return f"r{_rid.n}"
_rid.n = 0


def lease(v, cid="a"):
    w = v.connect(cid)
    ack = v.handle(cid, {"t": "lease_request", "request_id": _rid(), "session_epoch": w["session_epoch"]})
    assert ack["t"] == "ack", ack
    return w["session_epoch"], ack["lease_id"]


def req(v, cid, t, epoch, lid, **kw):
    return v.handle(cid, {"t": t, "request_id": kw.pop("request_id", _rid()), "session_epoch": epoch,
                          "lease_id": lid, "control_gen": kw.pop("control_gen", v.control_gen), **kw})


def teleop(v, cid, epoch, lid, tick, **kw):
    msg = {"t": "teleop", "session_epoch": epoch, "lease_id": lid, "control_gen": v.control_gen,
           "arm_gen": v.arm_gen, "seq": kw.pop("seq"), "echo": {"state_seq": tick["state_seq"], "token": tick["token"]},
           "mode": kw.pop("mode", "direct"), "deadman": True, "speed_ratio": kw.pop("speed", 0.5),
           "curvature_ratio": 0.0, "lift_dir": 0, **kw}
    return v.handle(cid, msg)


def into_teleop(v, cid, epoch, lid):
    assert req(v, cid, "mode_request", epoch, lid, to="TELEOP")["t"] == "ack"
    assert req(v, cid, "arm", epoch, lid)["t"] == "ack"


def test_second_client_is_observer_without_lease(env):
    v, _ = env
    lease(v, "a")
    w = v.connect("b")
    r = v.handle("b", {"t": "lease_request", "request_id": "x", "session_epoch": w["session_epoch"]})
    assert r["t"] == "reject" and r["reason"] == "busy"


def test_teleop_moves_then_stops_at_absolute_deadline(env):
    v, clock = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    tick = v.make_tick()
    clock.t += 0.29  # echo arrives 290 ms after the tick was sent
    teleop(v, "a", epoch, lid, tick, seq=1)
    v.step(0.005)
    assert v.applied_speed_mps > 0.0
    clock.t += 0.02  # 310 ms after the tick: past the 300 ms deadline, no new packet
    v.step(0.005)
    assert v.applied_speed_mps == 0.0 and v.stop_kind == "safety_preempt" and not v.armed


def test_teleop_stops_when_commands_stop(env):
    v, clock = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    for i in range(5):
        tick = v.make_tick()
        teleop(v, "a", epoch, lid, tick, seq=i + 1)
        clock.t += 0.05
        v.step(0.05)
    assert v.applied_speed_mps > 0.0
    for _ in range(8):
        clock.t += 0.05
        v.step(0.05)
    assert v.applied_speed_mps == 0.0 and not v.armed


def test_echo_older_than_deadline_on_arrival_is_rejected(env):
    v, clock = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    tick = v.make_tick()
    clock.t += 0.31
    teleop(v, "a", epoch, lid, tick, seq=1)
    v.step(0.005)
    assert v.applied_speed_mps == 0.0 and v.last_teleop_reject == "expired"


def test_wrong_token_is_rejected(env):
    v, _ = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    tick = dict(v.make_tick(), token="guess")
    teleop(v, "a", epoch, lid, tick, seq=1)
    assert v.last_teleop_reject == "bad_echo"


def test_video_deadline_is_earlier_of_two(env):
    v, clock = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    v.record_frame(7)  # acquired at t=100.00
    clock.t += 0.40
    tick = v.make_tick()  # sent at 100.40; command deadline 100.70
    teleop(v, "a", epoch, lid, tick, seq=1, mode="video", frame_echo={"frame_id": 7})
    v.step(0.005)
    assert v.applied_speed_mps > 0.0
    clock.t += 0.06  # 100.46 > 100.00 + 0.50 - 0.05 = 100.45 (video deadline)
    v.step(0.005)
    assert v.applied_speed_mps == 0.0 and not v.armed


def test_stop_locks_until_rearm_and_paused_request_cannot_bypass(env):
    v, clock = env
    epoch, lid = lease(v)
    w = v.connect("obs")
    assert v.handle("obs", {"t": "stop", "request_id": "s1", "session_epoch": w["session_epoch"], "source": "console"})["t"] == "ack"
    assert v.mode == "STOPPED"
    r = req(v, "a", "mode_request", epoch, lid, to="PAUSED")
    assert r["t"] == "reject" and r["reason"] == "stopped_locked"
    assert req(v, "a", "rearm", epoch, lid)["reason"] == "not_stopped"  # stop not yet confirmed
    for _ in range(12):  # 0.6 s below the speed threshold
        clock.t += 0.05
        v.step(0.05)
    assert req(v, "a", "rearm", epoch, lid)["t"] == "ack"
    assert v.mode == "PAUSED"


def test_late_resume_after_stop_is_rejected(env):
    v, clock = env
    epoch, lid = lease(v)
    assert req(v, "a", "mission_submit", epoch, lid, pallet_id="P-01", destination_id="D-A",
               targets_revision=v.targets_revision)["t"] == "ack"
    mission_id = v.mission["mission_id"]
    assert req(v, "a", "mission_pause", epoch, lid, mission_id=mission_id)["t"] == "ack"
    stale_gen = v.control_gen  # valid until the STOP below
    w = v.connect("obs")
    v.handle("obs", {"t": "stop", "request_id": "s2", "session_epoch": w["session_epoch"], "source": "console"})
    for _ in range(12):
        clock.t += 0.05
        v.step(0.05)
    assert req(v, "a", "rearm", epoch, lid)["t"] == "ack"
    late = req(v, "a", "mission_resume", epoch, lid, mission_id=mission_id, control_gen=stale_gen)
    assert late["t"] == "reject" and late["reason"] == "stale_control"


def test_duplicate_request_returns_cached_result_without_rerun(env):
    v, _ = env
    epoch, lid = lease(v)
    r1 = req(v, "a", "mission_submit", epoch, lid, request_id="dup", pallet_id="P-01",
             destination_id="D-A", targets_revision=v.targets_revision)
    gen_after = v.control_gen
    r2 = req(v, "a", "mission_submit", epoch, lid, request_id="dup", pallet_id="P-01",
             destination_id="D-A", targets_revision=v.targets_revision, control_gen=gen_after - 1)
    assert r1 == r2 and v.missions_created == 1


def test_stale_targets_revision_is_rejected(env):
    v, _ = env
    epoch, lid = lease(v)
    r = req(v, "a", "mission_submit", epoch, lid, pallet_id="P-01", destination_id="D-A",
            targets_revision=v.targets_revision - 1)
    assert r["t"] == "reject" and r["reason"] == "stale_targets"


def test_auto_continues_when_lease_expires(env):
    v, clock = env
    epoch, lid = lease(v)
    req(v, "a", "mission_submit", epoch, lid, pallet_id="P-01", destination_id="D-A",
        targets_revision=v.targets_revision)
    gen = v.control_gen
    for _ in range(30):  # 1.5 s without echoes
        clock.t += 0.05
        v.step(0.05)
    assert v.mode == "AUTO" and v.lease is None and v.control_gen > gen


def test_closing_a_duplicate_connection_keeps_the_holders_lease(env):
    v, _ = env
    w = v.connect("a", conn_id="c1")
    lid = v.handle("a", {"t": "lease_request", "request_id": "l1", "session_epoch": w["session_epoch"]},
                   conn_id="c1")["lease_id"]
    v.connect("a", conn_id="c2")           # second window with the copied client_id, observer only
    v.disconnect("a", conn_id="c2")
    assert v.lease is not None and v.lease["lease_id"] == lid


def test_late_close_of_an_old_connection_keeps_the_new_lease(env):
    v, clock = env
    w = v.connect("a", conn_id="old")
    v.handle("a", {"t": "lease_request", "request_id": "l1", "session_epoch": w["session_epoch"]}, conn_id="old")
    clock.t += 1.5
    v.step(0.05)                            # the old connection went silent: lease expired, reserved for "a"
    v.connect("a", conn_id="new")
    new = v.handle("a", {"t": "lease_request", "request_id": "l2", "session_epoch": w["session_epoch"]},
                   conn_id="new")
    assert new["t"] == "ack"
    v.disconnect("a", conn_id="old")        # the old socket finally closes
    assert v.lease is not None and v.lease["lease_id"] == new["lease_id"]


def test_reconnect_within_grace_gets_new_lease_id(env):
    v, clock = env
    epoch, lid = lease(v)
    v.disconnect("a")
    clock.t += 2.0
    v.step(0.05)
    w2 = v.connect("b")
    r = v.handle("b", {"t": "lease_request", "request_id": "q", "session_epoch": w2["session_epoch"]})
    assert r["reason"] == "lease_reserved"
    w = v.connect("a")
    r = v.handle("a", {"t": "lease_request", "request_id": "z", "session_epoch": w["session_epoch"]})
    assert r["t"] == "ack" and r["lease_id"] != lid
    assert req(v, "a", "arm", epoch, lid)["reason"] == "stale_lease"


def test_teleop_refused_with_forks_in_pocket_but_withdraw_allowed(env):
    v, _ = env
    epoch, lid = lease(v)
    v.debug_set_phase("insert", forks_in_pocket=True)
    r = req(v, "a", "mode_request", epoch, lid, to="TELEOP")
    assert r["t"] == "reject" and r["reason"] == "state_forbids_teleop"
    assert req(v, "a", "recovery", epoch, lid, action="withdraw_straight")["t"] == "ack"


def test_teleop_allowed_while_carrying_in_transport_with_loaded_caps(env):
    v, clock = env
    epoch, lid = lease(v)
    v.debug_set_phase("transport", forks_in_pocket=False, carried="P-01")
    into_teleop(v, "a", epoch, lid)
    for i in range(40):
        tick = v.make_tick()
        teleop(v, "a", epoch, lid, tick, seq=i + 1, speed=1.0)
        clock.t += 0.05
        v.step(0.05)
    assert 0.0 < v.applied_speed_mps <= 0.30 + 1e-9


def _stop_and_settle(v, clock, rid="s9"):
    w = v.connect("obs")
    v.handle("obs", {"t": "stop", "request_id": rid, "session_epoch": w["session_epoch"], "source": "console"})
    for _ in range(12):
        clock.t += 0.05
        v.step(0.05)
    return w


def test_stopped_lock_also_refuses_mission_and_recovery(env):
    v, clock = env
    epoch, lid = lease(v)
    _stop_and_settle(v, clock)
    r = req(v, "a", "mission_submit", epoch, lid, pallet_id="P-01", destination_id="D-A",
            targets_revision=v.targets_revision)
    assert r["reason"] == "stopped_locked"
    assert req(v, "a", "recovery", epoch, lid, action="withdraw_straight")["reason"] == "stopped_locked"
    assert v.mode == "STOPPED"


def test_resent_stop_after_rearm_stops_again(env):
    # contract 1, rule 0 (agreed with main): STOP is never cached; a late duplicate stops again (safe side)
    v, clock = env
    epoch, lid = lease(v)
    w = _stop_and_settle(v, clock, rid="once")
    assert req(v, "a", "rearm", epoch, lid)["t"] == "ack"
    gen = v.control_gen
    again = v.handle("obs", {"t": "stop", "request_id": "once", "session_epoch": w["session_epoch"], "source": "console"})
    assert again["t"] == "ack" and v.mode == "STOPPED" and v.control_gen == gen + 1


def test_full_result_cache_refuses_new_requests_but_not_stop(clock_only):
    v = MockVehicle(clock_only, lambda: "tok", Config(result_cache_max=2))
    w = v.connect("a")
    e = w["session_epoch"]
    assert v.handle("a", {"t": "targets_request", "request_id": "q1", "session_epoch": e})["t"] == "ack"
    assert v.handle("a", {"t": "targets_request", "request_id": "q2", "session_epoch": e})["t"] == "ack"
    r = v.handle("a", {"t": "lease_request", "request_id": "q3", "session_epoch": e})
    assert r["t"] == "reject" and r["reason"] == "busy"
    assert v.handle("a", {"t": "targets_request", "request_id": "q1", "session_epoch": e})["t"] == "ack"  # cached
    assert v.handle("a", {"t": "stop", "request_id": "s", "session_epoch": e, "source": "console"})["t"] == "ack"
    assert v.mode == "STOPPED"


def _idle(v, clock, epoch, lid, n):
    for _ in range(n):
        tick = v.make_tick()
        v.handle("a", {"t": "echo", "session_epoch": epoch, "lease_id": lid,
                       "echo": {"state_seq": tick["state_seq"], "token": tick["token"]}})
        clock.t += 0.05
        v.step(0.05)


def _drive(v, clock, epoch, lid, n, speed=1.0, lift=0, start_seq=1):
    for i in range(n):
        tick = v.make_tick()
        teleop(v, "a", epoch, lid, tick, seq=start_seq + i, speed=speed, lift_dir=lift)
        clock.t += 0.05
        v.step(0.05)


def test_drive_output_is_cut_the_tick_the_phase_forbids_it(env):
    v, clock = env
    epoch, lid = lease(v)
    v.debug_set_phase("transport", forks_in_pocket=False, carried="P-01")
    into_teleop(v, "a", epoch, lid)
    _drive(v, clock, epoch, lid, 20)
    assert v.applied_speed_mps > 0.2  # moving at the loaded cap
    v.debug_set_phase("lower", forks_in_pocket=True, carried="P-01")  # state changes after entry
    tick = v.make_tick()
    teleop(v, "a", epoch, lid, tick, seq=100, speed=1.0)
    v.step(0.005)
    assert v.make_state()["drive_allowed"] is False and v.applied_speed_mps == 0.0


def test_lift_and_drive_are_exclusive(env):
    v, clock = env
    epoch, lid = lease(v)
    v.debug_set_phase("transport", forks_in_pocket=False, carried="P-01")
    into_teleop(v, "a", epoch, lid)
    _drive(v, clock, epoch, lid, 10)               # moving
    lift0 = v.lift_m
    _drive(v, clock, epoch, lid, 3, speed=1.0, lift=1, start_seq=50)   # both requested while moving
    assert v.applied_speed_mps == 0.0 and v.lift_m == lift0   # drive cut, lift waits for stop
    _drive(v, clock, epoch, lid, 15, speed=0.0, lift=1, start_seq=60)  # stop confirmed, lifting
    assert v.lift_m > lift0
    _drive(v, clock, epoch, lid, 1, speed=1.0, lift=0, start_seq=80)   # first tick after release: lift moved last tick
    assert v.applied_speed_mps == 0.0
    _drive(v, clock, epoch, lid, 3, speed=1.0, lift=0, start_seq=81)   # then driving resumes normally
    assert v.applied_speed_mps > 0.0


def test_second_stop_while_stopped_invalidates_a_late_rearm(env):
    v, clock = env
    epoch, lid = lease(v)
    w = _stop_and_settle(v, clock, rid="s1")
    old_gen = v.control_gen                       # a rearm is prepared with this generation
    v.handle("obs", {"t": "stop", "request_id": "s2", "session_epoch": w["session_epoch"], "source": "console"})
    assert v.mode == "STOPPED" and v.control_gen == old_gen + 1
    late = req(v, "a", "rearm", epoch, lid, control_gen=old_gen)
    assert late["t"] == "reject" and late["reason"] == "stale_control"


def test_second_pause_invalidates_a_late_resume(env):
    v, clock = env
    epoch, lid = lease(v)
    req(v, "a", "mission_submit", epoch, lid, pallet_id="P-01", destination_id="D-A",
        targets_revision=v.targets_revision)
    mid = v.mission["mission_id"]
    assert req(v, "a", "mission_pause", epoch, lid, mission_id=mid)["t"] == "ack"
    old_gen = v.control_gen                       # a resume is prepared with this generation
    assert req(v, "a", "mission_pause", epoch, lid, mission_id=mid)["t"] == "ack"   # new request id
    assert v.mode == "PAUSED" and v.control_gen == old_gen + 1
    late = req(v, "a", "mission_resume", epoch, lid, mission_id=mid, control_gen=old_gen)
    assert late["t"] == "reject" and late["reason"] == "stale_control"


def test_pause_decelerates_and_auto_drives(env):
    v, clock = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    _drive(v, clock, epoch, lid, 10, speed=1.0)
    moving, gen = v.applied_speed_mps, v.control_gen
    assert req(v, "a", "mode_request", epoch, lid, to="PAUSED")["t"] == "ack"
    assert v.armed is False and v.control_gen == gen + 1 and v.stop_kind == "controlled_decel"
    v.step(0.005)
    assert 0.0 < v.applied_speed_mps < moving  # controlled deceleration, not a cut
    _idle(v, clock, epoch, lid, 40)  # echoes keep the lease while waiting
    assert req(v, "a", "mission_submit", epoch, lid, pallet_id="P-01", destination_id="D-A",
               targets_revision=v.targets_revision)["t"] == "ack"
    assert v.mode == "AUTO" and v.stop_kind == "none"
    _idle(v, clock, epoch, lid, 60)  # observe phase 1.5 s, then approach
    assert v.applied_speed_mps > 0.0


def test_permission_direction_holds_until_stop_is_confirmed(env):
    v, clock = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    v.applied_speed_mps = 0.019   # forward residue below the 0.02 threshold, stop not confirmed
    v.stopped_confirmed = False
    v._last_nonzero_sign = 1
    tick = v.make_tick()
    teleop(v, "a", epoch, lid, tick, seq=1, speed=-1.0)
    v.step(0.005)
    assert v.last_permission_direction == 1


def test_old_teleop_command_does_not_resume_after_new_lease_and_arm(env):
    v, clock = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    tick = v.make_tick()
    teleop(v, "a", epoch, lid, tick, seq=1, speed=1.0)
    v.disconnect("a")  # lease lost; the accepted command is still before its deadline
    w = v.connect("a")
    ack = v.handle("a", {"t": "lease_request", "request_id": "re", "session_epoch": w["session_epoch"]})
    assert req(v, "a", "arm", epoch, ack["lease_id"])["t"] == "ack"
    v.applied_speed_mps = 0.0
    v.step(0.005)
    assert v.applied_speed_mps == 0.0


@pytest.mark.parametrize("bad", [
    {"seq": "1"}, {"speed_ratio": float("nan")}, {"speed_ratio": 1.5}, {"lift_dir": 2}, {"mode": "auto"},
])
def test_malformed_teleop_is_dropped(env, bad):
    v, _ = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    tick = v.make_tick()
    msg = {"t": "teleop", "session_epoch": epoch, "lease_id": lid, "control_gen": v.control_gen,
           "arm_gen": v.arm_gen, "seq": 1, "echo": {"state_seq": tick["state_seq"], "token": tick["token"]},
           "mode": "direct", "deadman": True, "speed_ratio": 0.5, "curvature_ratio": 0.0, "lift_dir": 0}
    msg.update(bad)
    assert v.handle("a", msg) is None
    assert v.last_teleop_reject == "invalid"


def test_reliable_request_without_request_id_is_invalid(env):
    v, _ = env
    w = v.connect("a")
    r = v.handle("a", {"t": "lease_request", "session_epoch": w["session_epoch"]})
    assert r["t"] == "reject" and r["reason"] == "invalid"


def test_instances_created_back_to_back_have_distinct_session_epochs():
    epochs = {MockVehicle(Clock(), lambda: "t").session_epoch for _ in range(200)}
    assert len(epochs) == 200


def test_leaving_teleop_disarms_and_reentry_needs_a_new_arm(env):
    v, _ = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    old_arm = v.arm_gen
    assert req(v, "a", "mode_request", epoch, lid, to="PAUSED")["t"] == "ack"
    assert req(v, "a", "mode_request", epoch, lid, to="TELEOP")["t"] == "ack"
    assert v.armed is False
    tick = v.make_tick()
    msg = {"t": "teleop", "session_epoch": epoch, "lease_id": lid, "control_gen": v.control_gen,
           "arm_gen": old_arm, "seq": 99, "echo": {"state_seq": tick["state_seq"], "token": tick["token"]},
           "mode": "direct", "deadman": True, "speed_ratio": 0.5, "curvature_ratio": 0.0, "lift_dir": 0}
    v.handle("a", msg)
    assert v.last_teleop_reject == "not_armed"


def test_cancel_while_stopped_keeps_the_lock(env):
    v, clock = env
    epoch, lid = lease(v)
    req(v, "a", "mission_submit", epoch, lid, pallet_id="P-01", destination_id="D-A",
        targets_revision=v.targets_revision)
    _stop_and_settle(v, clock)
    assert req(v, "a", "mission_cancel", epoch, lid, mission_id=v.mission["mission_id"])["t"] == "ack"
    assert v.mode == "STOPPED"
    assert req(v, "a", "mode_request", epoch, lid, to="TELEOP")["reason"] == "stopped_locked"


def test_lease_can_be_taken_while_stopped_so_someone_can_rearm(env):
    v, clock = env
    _stop_and_settle(v, clock)  # an observer stops before anyone holds the lease
    epoch, lid = lease(v, "a")
    assert req(v, "a", "rearm", epoch, lid)["t"] == "ack"


def test_slew_then_permission_cap_never_exceeds_cap(env):
    v, _ = env
    v.applied_speed_mps = 0.5  # debug_output(target, curvature, explicit cap, dt) bypasses the world check
    out = v.debug_output(target_mps=0.5, curvature_inv_m=0.0, permission_cap_mps=0.1, dt_s=1 / 120)
    assert out <= 0.1 + 1e-12
```

- [ ] **Step 2: 실패 확인** — `python -m pytest tests/unit/console/test_mock_vehicle_core.py -q` → `ModuleNotFoundError ... core`

- [ ] **Step 3: 구현** — `console/mock_vehicle/core.py`. 아래 규칙을 이 순서대로 코드로 옮긴다(계약 절 번호를 주석으로 단다).

```python
"""State machine of the fake vehicle that speaks the console/1 draft.

Mock only: it imitates the arbiter rules the console relies on so the web app can be
developed; it is not forklift_core/safety (M-b) and its tests are not evidence for it.
Times come from the injected monotonic clock; the browser's clock is never used.
"""

from __future__ import annotations

import math
import secrets
from collections import OrderedDict
from dataclasses import dataclass
from collections.abc import Callable

from console.mock_vehicle import paths
from console.mock_vehicle.world import CHASSIS, Pose, World, default_world, permission

PHASES = ("observe", "approach", "insert", "lift", "extract", "transport", "lower",
          "withdraw", "return")
TELEOP_FORBIDDEN = {"insert", "lift", "lower", "withdraw"}
LEASE_REQUIRED = {"rearm", "mode_request", "arm", "disarm", "mission_submit", "mission_pause",
                  "mission_resume", "mission_cancel", "recovery", "lease_release"}


@dataclass(frozen=True)
class Config:
    command_max_age_s: float = 0.30
    tick_hz: float = 20.0
    lease_timeout_s: float = 1.0
    grace_s: float = 5.0
    video_max_age_s: float = 0.50
    camera_latency_bound_s: float = 0.05
    forward_mps: float = 0.50
    reverse_mps: float = 0.25
    loaded_scale: float = 0.6
    accel_mps2: float = 0.8
    decel_mps2: float = 1.5
    stop_threshold_mps: float = 0.02
    stop_hold_s: float = 0.5
    result_cache_max: int = 4096


class MockVehicle:
    def __init__(self, clock: Callable[[], float], token: Callable[[], str],
                 config: Config | None = None, world: World | None = None,
                 session_epoch: int | None = None):
        self.clock, self.token, self.cfg = clock, token, config or Config()
        self.world = world or default_world()
        # random 48-bit (JSON-safe) per instance: a restarted fake vehicle never reuses an epoch (contract 1)
        self.session_epoch = session_epoch if session_epoch is not None else secrets.randbits(48) | 1
        self.lease = None          # {"client_id", "lease_id", "last_sent"}
        self.reserved = None       # {"client_id", "until"}
        self.control_gen = 1
        self.arm_gen = 0
        self.armed = False
        self.mode = "PAUSED"
        self.stop_kind = "none"
        self.state_seq = 0
        self._sent = OrderedDict()  # state_seq -> (token, sent_mono)
        self._frames = {}           # frame_id -> acquired_mono
        self._results = {}          # (client_id, epoch, request_id) -> response
        self._teleop = None         # accepted command + deadline
        self._last_seq = 0
        self.last_teleop_reject = None
        self.pose = self.world.home
        self.applied_speed_mps = 0.0
        self.applied_curvature_inv_m = 0.0
        self.lift_m = 0.0
        self.mission = None
        self.missions_created = 0
        self.targets_revision = 1
        self.forks_in_pocket = False
        self.permit = (0.5, "ok")
        self._still_since = None
        self.clients = set()
        self.events = []
    # ... methods below
```

메서드 규칙(각 항목이 위 시험 하나 이상에 대응한다):

1. `connect(client_id)`: `clients`에 추가, `welcome` 반환(`api`, `vehicle_id="mock-1"`, `session_epoch`, `backend="mock"`, `capabilities` — 계약 3절 형식. `feedback`은 모두 `True`, `camera_model`은 Task 4의 값, `chassis`는 `CHASSIS`에서).
2. **연결 식별(같은 `client_id`의 연결이 여럿일 수 있다. opener가 있는 새 창은 `sessionStorage`를 복사한다).** lease는 `{client_id, conn_id, lease_id, last_sent}`로 그것을 얻은 **연결**에 묶는다. `disconnect(client_id, conn_id)`는 `lease.conn_id == conn_id`일 때만 `reserved = {client_id, until: now + grace_s}`, `lease = None`, `_lease_lost()`를 한다. 다른 연결이 끊긴 것은 lease에 영향을 주지 않는다. lease가 필요한 요청과 echo·teleop도 `lease.conn_id`와 보낸 연결이 같아야 받는다(다르면 `no_lease`). 유예 회수(`reserved.client_id` 일치)는 새 연결에 새 `lease_id`로 준다.
1-1. lease 부여: `lease = {client_id, conn_id, lease_id: self.token(), last_sent: now}`(첫 회신 전까지 부여 시각을 기준으로 만료를 잰다). 이미 보유자가 있으면 `busy`, `reserved`가 다른 클라이언트면 `lease_reserved`. 같은 클라이언트의 유예 회수도 새 `lease_id`.
3. `_lease_lost()`: `control_gen += 1`, `_teleop = None`. 모드가 TELEOP이면 `_safety_stop("lease_lost")`(모드는 TELEOP 유지, 무장 해제). AUTO면 아무것도 바꾸지 않는다(사용자 결정).
4. `make_tick()`: `state_seq += 1`, `tok = self.token()`, `_sent[state_seq] = (tok, now)`, 창 정리(`now - sent > max(command_max_age_s, lease_timeout_s) + 0.5`인 앞쪽 항목 삭제, 시간 기준), `{"t": "tick", "session_epoch", "state_seq", "token": tok}` 반환.
5. `_echo_sent(echo) -> float | None`: `_sent.get(state_seq)`의 토큰이 같으면 송출 시각, 아니면 `None`.
0. **입력 검증** `_valid(msg) -> str | None`: 형식이 틀리면 사유를 돌려준다. 신뢰 요청은 `reject(invalid)`, 최신값 메시지는 `last_teleop_reject = "invalid"`로 버린다. 규칙: `t`는 문자열, 신뢰 요청은 비어 있지 않은 문자열 `request_id`, `session_epoch`·`control_gen`·`arm_gen`·`seq`·`state_seq`·`frame_id`는 `bool`이 아닌 `int`, `speed_ratio`·`curvature_ratio`는 유한한 `float|int`이고 `[-1, 1]` 안, `lift_dir`는 `-1, 0, 1`, `mode`는 `direct|video`, `deadman`은 `bool`. 서버는 `json.loads(..., parse_constant=_reject_constant)`로 `NaN`·`Infinity`를 받지 않는다.
6. `handle(client_id, msg)`:
   - 먼저 0번 검증.
   - `t == "stop"`(계약 1절 처리 순서 0, 메인 개발과 합의): `client_id in clients`면 캐시·세대 검사 없이 **항상** `_stop("console")` 후 `ack`. 결과를 저장하지 않는다(같은 `request_id`가 다시 오면 다시 정지).
   - 그 밖의 신뢰 요청: `key = (client_id, msg["session_epoch"], msg["request_id"])`가 `_results`에 있으면 그것을 그대로 반환한다(처리 순서 1). 없고 `len(_results) >= result_cache_max`이면 저장된 결과를 버리지 않고 `reject(busy)`를 돌려준다(이 거절은 저장하지 않는다).
   - `t in ("echo", "teleop", "input_state")`: 최신값 처리 후 `None`.
     - echo/teleop: 보유자이고 `session_epoch`·`lease_id`가 맞고 `_echo_sent`가 있으면 `lease["last_sent"] = max(...)`.
     - teleop: 아래 7번.
     - input_state(`visible: false` 또는 `focused: false` 또는 `gamepad_lost: true`): 보유자면 `_disarm("input_state")`.
   - 세대 검사(처리 순서 2): `session_epoch` 불일치 → `stale_epoch`. `t == "lease_request"`는 lease 검사를 하지 않는다. `LEASE_REQUIRED`면 `lease is None or lease.client_id != client_id` → `no_lease`, `lease_id` 불일치 → `stale_lease`, `control_gen` 불일치 → `stale_control`. **`mode == "STOPPED"`이면 `STOPPED_ALLOWED = {stop, rearm, disarm, lease_request, lease_release, mission_cancel, mission_pause, targets_request}` 말고는 모두 `stopped_locked`**(작업 시작·재개·복구·모드·무장이 정지 잠금을 우회하지 못하게 한다). 이 검사는 lease 검사보다 앞이 아니라 뒤에 두되, `lease_request`는 잠금 대상이 아니다(누구도 lease를 못 얻어 재무장할 수 없게 되는 것을 막는다). 허용된 요청도 **모드를 바꾸지 않는다** — `STOPPED`에서 나가는 길은 `rearm`뿐이다.
   - 실행 후 결과를 `_results[key]`에 저장하고 반환. 결과 형식: `{"t": "ack", "request_id", ...추가}` 또는 `{"t": "reject", "request_id", "reason"}`.
7. teleop 처리: 세대(`session_epoch`, `lease_id`, `control_gen`, `arm_gen`) 중 하나라도 다르면 해당 사유(`stale_*`)로 `last_teleop_reject` 기록 후 무시. `seq <= _last_seq` → `stale_seq`. `not armed` → `not_armed`. `_echo_sent`가 `None` → `bad_echo`. `deadline = sent + command_max_age_s`. `mode == "video"`면 `frame_echo.frame_id`의 `acquired`가 없으면 `stale_video`, 있으면 `deadline = min(deadline, acquired + video_max_age_s - camera_latency_bound_s)`. `now >= deadline` → `expired`. 통과하면 `_teleop = {msg, deadline, gens: (session_epoch, lease_id, control_gen, arm_gen)}`, `_last_seq = seq`, lease `last_sent` 갱신.
8. `step(dt)`: 
   - lease 만료: `lease and now - lease["last_sent"] > lease_timeout_s` → `reserved = {client_id, until: now + grace_s}`, `lease = None`, `_lease_lost()`. `reserved` 만료 시 `None`.
   - 저장된 텔레옵 명령의 세대 재검사: `_teleop["gens"]`가 현재 `(session_epoch, lease["lease_id"], control_gen, arm_gen)`과 다르거나 lease가 없거나 무장이 풀렸으면 `_teleop = None`(이전 lease·무장 세대의 명령이 새 무장 뒤에 다시 출력되지 않게 한다).
   - 텔레옵 만료: `_teleop and now >= _teleop["deadline"]` → `_safety_stop("expired")`, `_teleop = None`.
   - 매 주기 허가(설계 4.5, 합의). TELEOP에서 주행과 승강은 **서로 배타적**이다.
     - `lift_active = (요청 lift_dir ≠ 0) or lift_moving`. `lift_moving`은 지난 주기에 승강이 실제로 움직였는지다.
     - `drive_allowed = mode == "TELEOP" and not (phase in TELEOP_FORBIDDEN or forks_in_pocket) and not lift_active`(TELEOP에서만 뜻이 있다. AUTO·PAUSED에서는 상태에 `null`로 싣는다)
     - `lift_allowed = mode == "TELEOP" and phase not in {"insert", "withdraw"} and stopped_confirmed`. 승강은 차량이 정지 확인된 뒤에만 시작한다. 정지 확인 전의 승강 요청은 승강 0으로 두되, 그동안에도 주행은 막는다(`lift_active`).
     - **TELEOP에서** `drive_allowed`가 거짓이면 텔레옵 주행 목표를 0으로 두고, `_output`의 최종 출력도 0으로 자른다(안전 선점과 같은 자리, slew 뒤). 그래서 금지되는 순간 출력이 남지 않는다. PAUSED는 제어 감속(`stop_kind = "controlled_decel"`, 목표 0에 감속 한계로 접근), AUTO는 작업 재생 출력이며 이 차단을 받지 않는다.
     - `lift_moving`은 지난 한 주기만 본다. 그래서 승강을 놓은 첫 주기에는 주행이 막히고, 그다음 주기부터 풀린다(시험이 이를 고정한다).
     - `lift_allowed`가 거짓이면 승강을 0으로 둔다.
     - 진입 뒤 상태가 바뀌어도 주기마다 다시 판정한다.
   - 목표 속도: STOPPED·PAUSED → 0(제어 감속; STOPPED는 안전 선점이라 즉시 0). TELEOP: `_teleop`가 있고 `deadman`이면 `ratio × 모드 상한(적재면 × loaded_scale)`, 곡률은 `curvature_ratio × CHASSIS.teleop_curvature_inv_m`. AUTO: 작업 재생(9번).
   - `_output(target, curvature, dt)`: 곡률을 `±teleop_curvature_inv_m`로 자름 → slew(가속 `accel_mps2`, 감속 `decel_mps2`; 부호가 바뀌는 요청이면 먼저 0까지 감속) → `permission(...)` 상한을 넘으면 그 값으로 자름. 허가의 **방향은 실제 이동 방향**(**정지 확인 전에는** 마지막으로 0이 아니었던 `applied`의 부호를 유지한다. 순간적으로 문턱 아래로 내려가도 바꾸지 않는다. `stopped_confirmed`가 된 뒤에만 목표의 부호로 바꾼다. 검사에 쓴 방향은 `last_permission_direction`에 남긴다)으로 잡고, 검사 속도는 `max(|applied|, |직전 적용값|)`으로 잡는다(합의, 설계 4.4) → `mode == "TELEOP" and not drive_allowed`이거나 `stop_kind == "safety_preempt"`면 0. 결과를 `applied_speed_mps`에 저장. `debug_output`은 이 함수를 바로 호출하는 시험용 진입점이다.
   - 운동: `x += v cos yaw dt`, `y += v sin yaw dt`, `yaw += v κ dt`(뒤 차축 자전거). AUTO는 경로 재생이 자세를 정한다.
   - 정지 판정: `stopped_confirmed`는 처음 `False`. `|applied| < stop_threshold_mps`인 동안 `_still_since`를 유지하고, `stop_hold_s`가 지나면 `True`. 문턱을 넘는 순간 `_still_since = None`, `stopped_confirmed = False`.
9. 작업(AUTO): v1 목업의 단계 흐름을 그대로 옮긴다. 경로 생성은 `paths.hermite_leg`, 경유점은 `world`의 값(P-01: `(7.0,5.5,0)`, 회전점 `(9.2,6.6,−π/2)`; P-02: `(7.0,4.6,−0.6)`, `(8.8,1.9,−0.15)`, 회전점 `(8.6,1.15,π/2)`). `insert`·`lift`·`lower`·`withdraw` 중에는 `forks_in_pocket`를 맞게 둔다. 일시정지 중에는 `phase_elapsed_s`를 늘리지 않는다.
10. 모드 요청: `STOPPED`면 모두 `stopped_locked`. **모든 모드 전이는 `_set_mode(new)` 하나로만 한다.** 부수효과는 다음과 같다.
    - 공통: `_boundary()` = `armed = False`, `_teleop = None`, `control_gen += 1`. `_set_mode`는 모드가 실제로 바뀔 때 `_boundary()`를 한 번 부른다. 모드 전이는 설계 5.1의 무장 해제 사유이고, 정지·권한 경계(계약 1절)이기도 하다.
    - **정지 요청은 모드가 같아도 경계다.** STOP, 새로 처리하는 `mission_pause`, `mode_request(to=PAUSED)`는 모드가 이미 같으면 `_set_mode` 대신 `_boundary()`를 직접 한 번 부른다. 그래서 정지 전에 만들어진 `rearm`·`mission_resume` 같은 요청이 늦게 와도 `stale_control`로 거부된다. 어느 경로든 한 요청에 정확히 한 번만 올린다.
    - `stop_kind`: PAUSED → `"controlled_decel"`, AUTO·TELEOP → `"none"`, STOPPED → `"safety_preempt"`. TELEOP은 `"none"`이어도 새 `arm` 전에는 움직이지 않는다.
    - 다시 TELEOP에 들어와도 새 `arm`이 필요하다.
    - 작업 시작·재개(AUTO), 일시정지·취소(PAUSED), STOP(STOPPED), 재무장(PAUSED)도 모두 `_set_mode`를 거친다. 그래서 위 `rearm`·`_stop`·작업 요청 항목의 `control_gen += 1`·`stop_kind` 설정은 `_set_mode`가 대신한다(두 번 올리지 않는다). `to == "PAUSED"`: AUTO면 작업 `paused`, `control_gen += 1`. `to == "TELEOP"`: `phase in TELEOP_FORBIDDEN or forks_in_pocket` → `state_forbids_teleop`. 작업 중이면 `held`.
11. `arm`: TELEOP이 아니면 `invalid`. `arm_gen += 1`, `armed = True`, `stop_kind = "none"`, `_last_seq = 0`, `_teleop = None`, `ack`에 `arm_gen`. `disarm`: `_disarm("request")`.
12. `rearm`: `mode != "STOPPED"` → `invalid`. `stopped_confirmed`가 아니면 `reject("not_stopped")`. 통과하면 `mode = "PAUSED"`, `stop_kind = "none"`, `control_gen += 1`, `_teleop = None`.
13. `_stop(source)`: `mode = "STOPPED"`, `stop_kind = "safety_preempt"`, `armed = False`, 작업 `held`, `_teleop = None`, `control_gen += 1`. `_safety_stop(reason)`: 모드 유지, `stop_kind = "safety_preempt"`, 무장 해제, `_teleop = None`, `control_gen += 1`(만료 경계). `arm`이 `stop_kind`를 `"none"`으로 되돌린다.
14. 작업 요청: `mission_submit`는 `targets_revision` 불일치 → `stale_targets`, 팔레트가 `available`이 아니면 `invalid`, 아니면 `missions_created += 1`, 작업 생성(`mission_id = f"M-{n:04d}"`), 모드 AUTO, `ack`에 `mission_id`. `mission_pause` → `paused`, 모드 PAUSED, `control_gen += 1`. `mission_resume` → 재개 절차 상태 `resume_steps = [stop_confirmed, state_verified, frame_reexpressed, path_rechecked]`(계약 6절 이름)를 차례로 평가(가짜 차량: 1단계는 `stopped_confirmed`, 실패면 `not_stopped_timeout`; 2단계는 `forks_in_pocket`면 `fork_in_pocket`으로 실패해 복구 동작만 허용; 3단계는 즉시 ok; 4단계는 현재 자세에서 남은 경로를 재생성하고 막히면 `path_blocked:<reason>`), 모두 ok면 AUTO. `mission_cancel` → `cancelled`. 모드는 `STOPPED`면 그대로, 아니면 PAUSED.
15. `recovery(withdraw_straight)`: `forks_in_pocket`가 아니면 `invalid`. 맞으면 현재 방향의 반대로 `fork_length_m + 0.1`만큼 직선 후진 경로를 재생하는 특수 단계 `recover` 시작.
16. `make_state()`: 계약 6절 필드를 모두 채운다(`command` = 비율 × 현재 상한의 물리 명령, `applied` = `_output` 결과와 그 곡률; 가짜 차량에는 Ackermann 바퀴 한계가 없으므로 `applied`가 실차보다 단순하다는 것을 README에 적는다; `drive_allowed`·`lift_allowed`). `load.state`는 `"loaded" | "empty"`, `teleop_allowed`/`teleop_forbidden_reason`, `limits`(`provisional: True`, 적재면 적재 상한), `pacing: None`(가짜 차량), `plan`은 남은 경로 점을 10개마다 하나씩.
17. `debug_set_phase(phase, forks_in_pocket, carried=None)`: 시험용. 작업이 없으면 가짜 작업(`status="running"`, 모드 AUTO)을 만들고 그 단계로 둔다. `carried`가 주어지면 그 팔레트의 `state = "carried"`, `world.carried = carried`, 이후 허가는 그 팔레트를 `load`로 검사한다. `carried`가 없으면 모든 `carried`를 `available`로 되돌린다.

- [ ] **Step 4: 통과 확인** — `python -m pytest tests/unit/console -q -p no:cacheprovider -W error` → 전부 통과

- [ ] **Step 5: 커밋** — `console: fake vehicle state machine for console/1 (generations, lease and grace, request order, STOPPED lock, absolute and video deadlines, slew before the permission cap, teleop states, mock mission)`

---

### Task 3: 가짜 차량 서버와 카메라 렌더

**Files:**
- Create: `console/mock_vehicle/camera.py`, `console/mock_vehicle/server.py`, `console/mock_vehicle/__main__.py`
- Test: `tests/integration/console/test_mock_server.py`

**Interfaces:**
- Consumes: `core.MockVehicle`, `world.*`
- Produces:
  - `camera.CAMERA_MODEL: dict` (계약 3절 `camera_model` 형식: 640×400, 수평 화각 69°, 높이 1.05 m, 아래로 14°, 뒤 차축 앞 0.70 m)
  - `camera.render_jpeg(world, pose, lift_m, frame_id, width=640, height=400) -> bytes`
  - `async server.start(host: str, port: int, web_root: Path, *, test_frame_pad_bytes: int = 0) -> Handle` — `Handle.media_drops`·`Handle.media_closed`는 송신 감시가 건너뛴 프레임 수와 닫은 미디어 연결 수(시험이 정체 도달을 확인하는 데 쓴다). `test_frame_pad_bytes`는 시험 전용으로 카메라 본문 뒤에 **무작위 바이트**(`os.urandom`)를 붙인다. 서버는 모든 연결을 `compression=None`으로 연다(JPEG은 더 줄지 않고, 압축하면 정체 시험이 무의미해진다). `close_timeout=1.0`으로 열고, `Handle.close()`는 서버를 닫은 뒤 1초 안에 닫히지 않은 연결을 `transport.abort()`로 끊는다. `Handle.close()`는 서버와 주기 작업(제어·tick·state·미디어)을 모두 취소하고, `await Handle.wait_closed()`는 그 종료를 기다린다. `async server.main(host, port, web_root)`은 `start` 후 끝없이 기다린다. 경로 `/` → `web_root/index.html`, `/js/*`, `/app.css`; WebSocket `/api/v1/control`, `/api/v1/media`
  - 미디어 바이너리 프레임 형식: `uint32 big-endian 머리글 길이` + 머리글 JSON(UTF-8) + 본문

- [ ] **Step 1: 시험 작성**

```python
import asyncio
import json
import struct

import pytest

websockets = pytest.importorskip("websockets")
pytest.importorskip("PIL")

from console.mock_vehicle import server  # noqa: E402


async def _open(port, path):
    return await websockets.connect(f"ws://127.0.0.1:{port}{path}")


async def _until(ws, t, pred=lambda m: True, echo=None):
    """Read until a message of type t that satisfies pred; answer ticks with an echo if given."""
    while True:
        msg = json.loads(await ws.recv())
        if echo and msg.get("t") == "tick":
            await ws.send(json.dumps(dict(echo, echo={"state_seq": msg["state_seq"], "token": msg["token"]})))
        if msg.get("t") == t and pred(msg):
            return msg


async def _keepalive(ws, keep, sink):
    async for raw in ws:
        msg = json.loads(raw)
        sink.append(msg)
        if msg.get("t") == "tick":
            await ws.send(json.dumps(dict(keep, echo={"state_seq": msg["state_seq"], "token": msg["token"]})))


def run(coro):
    return asyncio.run(asyncio.wait_for(coro, 20))


def test_second_client_is_observer_but_can_stop(tmp_path, unused_tcp_port):
    async def body():
        srv = await server.start("127.0.0.1", unused_tcp_port, tmp_path)
        try:
            a = await _open(unused_tcp_port, "/api/v1/control")
            await a.send(json.dumps({"t": "hello", "api": "console/1", "client_id": "a"}))
            wa = await _until(a, "welcome")
            await a.send(json.dumps({"t": "lease_request", "request_id": "1", "session_epoch": wa["session_epoch"]}))
            lease_ack = await _until(a, "ack")
            assert lease_ack["request_id"] == "1"
            keep = {"t": "echo", "session_epoch": wa["session_epoch"], "lease_id": lease_ack["lease_id"]}
            seen_a = []
            pump_a = asyncio.create_task(_keepalive(a, keep, seen_a))  # answers every tick for the whole test
            b = await _open(unused_tcp_port, "/api/v1/control")
            await b.send(json.dumps({"t": "hello", "api": "console/1", "client_id": "b"}))
            wb = await _until(b, "welcome")
            await b.send(json.dumps({"t": "lease_request", "request_id": "2", "session_epoch": wb["session_epoch"]}))
            assert (await _until(b, "reject"))["reason"] == "busy"
            await b.send(json.dumps({"t": "stop", "request_id": "3", "session_epoch": wb["session_epoch"], "source": "console"}))
            assert (await _until(b, "ack"))["request_id"] == "3"
            for _ in range(50):
                if any(m.get("t") == "state" and m["mode"] == "STOPPED" for m in seen_a):
                    break
                await asyncio.sleep(0.05)
            stopped = next(m for m in seen_a if m.get("t") == "state" and m["mode"] == "STOPPED")
            assert stopped["stop_kind"] == "safety_preempt"
            pump_a.cancel()
        finally:
            srv.close()
            await srv.wait_closed()
    run(body())


def test_wrong_api_version_is_refused(tmp_path, unused_tcp_port):
    async def body():
        srv = await server.start("127.0.0.1", unused_tcp_port, tmp_path)
        try:
            a = await _open(unused_tcp_port, "/api/v1/control")
            await a.send(json.dumps({"t": "hello", "api": "console/0", "client_id": "a"}))
            with pytest.raises(websockets.ConnectionClosed):
                await _until(a, "welcome")
        finally:
            srv.close()
            await srv.wait_closed()
    run(body())


def test_control_ticks_keep_flowing_while_a_media_client_never_reads(tmp_path, unused_tcp_port):
    async def body():
        # 1 MB padding per camera frame fills the socket buffers within about a second
        srv = await server.start("127.0.0.1", unused_tcp_port, tmp_path, test_frame_pad_bytes=1_000_000)
        try:
            m = await websockets.connect(f"ws://127.0.0.1:{unused_tcp_port}/api/v1/media",
                                         max_queue=1, max_size=None, compression=None)
            await m.send(json.dumps({"t": "media_subscribe", "client_id": "slow", "camera_rgb": {"hz": 15}}))
            a = await _open(unused_tcp_port, "/api/v1/control")
            await a.send(json.dumps({"t": "hello", "api": "console/1", "client_id": "a"}))
            await _until(a, "welcome")
            loop = asyncio.get_running_loop()
            start, ticks = loop.time(), 0
            while loop.time() - start < 2.0:
                if json.loads(await a.recv()).get("t") == "tick":
                    ticks += 1
            assert ticks >= 30  # 20 Hz nominal; the unread media socket must not stall control
            assert srv.media_drops > 0 or srv.media_closed > 0  # congestion was really reached
        finally:
            srv.close()
            await srv.wait_closed()
    run(body())


def test_static_files_are_served_with_module_mime_types(tmp_path, unused_tcp_port):
    import urllib.request

    (tmp_path / "js").mkdir()
    (tmp_path / "index.html").write_text("<!doctype html>", encoding="utf-8")
    (tmp_path / "js" / "main.js").write_text("export {};", encoding="utf-8")

    async def body():
        srv = await server.start("127.0.0.1", unused_tcp_port, tmp_path)
        try:
            def fetch(path):
                with urllib.request.urlopen(f"http://127.0.0.1:{unused_tcp_port}{path}") as r:
                    return r.headers["Content-Type"]
            assert (await asyncio.to_thread(fetch, "/js/main.js")).startswith("text/javascript")
            assert (await asyncio.to_thread(fetch, "/")).startswith("text/html")
        finally:
            srv.close()
            await srv.wait_closed()
    run(body())


def test_media_frames_carry_frame_id_and_jpeg(tmp_path, unused_tcp_port):
    async def body():
        srv = await server.start("127.0.0.1", unused_tcp_port, tmp_path)
        try:
            m = await _open(unused_tcp_port, "/api/v1/media")
            await m.send(json.dumps({"t": "media_subscribe", "client_id": "a", "camera_rgb": {"hz": 10}}))
            for _ in range(20):
                data = await m.recv()
                if isinstance(data, bytes):
                    (n,) = struct.unpack(">I", data[:4])
                    head = json.loads(data[4:4 + n])
                    if head["t"] == "camera":
                        assert head["frame_id"] >= 1 and isinstance(head["session_epoch"], int)
                        assert data[4 + n:4 + n + 2] == b"\xff\xd8"
                        return
            pytest.fail("no camera frame")
        finally:
            srv.close()
            await srv.wait_closed()
    run(body())
```

`unused_tcp_port` 픽스처는 `tests/integration/console/conftest.py`에 직접 정의한다(외부 플러그인 의존 없이):

```python
import socket

import pytest


@pytest.fixture
def unused_tcp_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]
```

- [ ] **Step 2: 실패 확인** — `python -m pytest tests/integration/console -q` → `server` 없음으로 실패 (websockets 없는 환경이면 skip으로 보고되므로, conda base에서 확인)

- [ ] **Step 3: 구현**
  - `camera.py`: v1 목업 `drawCam`의 핀홀 투영(`toCam`, 근거리 자르기, 볼록 껍질, 벽 사각형, 바닥 격자, 팔레트 포켓)을 Pillow `ImageDraw`로 옮긴다. 프레임 번호와 `MOCK` 글자를 왼쪽 위에 찍는다. `quality=70`.
  - `server.py`:
    - `start(host, port, web_root)`: `await websockets.asyncio.server.serve(handler, host, port, process_request=static, compression=None, close_timeout=1.0)`로 서버를 열고, 제어 주기(120 Hz `vehicle.step`), tick(20 Hz, 모든 제어 연결에 송신), state(10 Hz), 대상 목록(바뀔 때), 미디어(카메라 15 Hz, 스캔 10 Hz, 격자 1 Hz) 작업을 띄운다.
    - `static`: `/`, `/index.html`, `/app.css`, `/js/<name>.js`만 `web_root`에서 읽어 응답한다. `Content-Type`을 확장자별로 붙인다(`.html` → `text/html; charset=utf-8`, `.js` → `text/javascript; charset=utf-8`, `.css` → `text/css; charset=utf-8`). 브라우저는 MIME이 맞지 않는 ES 모듈 실행을 거부한다. `..`이 든 경로나 그 밖의 경로는 404. WebSocket 경로는 `None`을 돌려 업그레이드를 진행한다.
    - 제어 연결: 첫 메시지가 `hello`이고 `api == "console/1"`이 아니면 닫는다(코드 1008). 이후 각 메시지를 `vehicle.handle(client_id, msg, conn_id=conn_id)`에 넘기고(`conn_id`는 연결마다 `uuid4().hex`) 응답이 있으면 그 연결로 보낸다. 연결이 끊기면 `vehicle.disconnect(client_id, conn_id=conn_id)`.
    - 송신 감시: 연결마다 송신을 `asyncio.wait_for(ws.send(...), timeout)`으로 감싼다(제어 0.2 s, 미디어 1.0 s). 제어 연결은 시간 초과면 닫는다(그 클라이언트의 텔레옵은 만료로 정지한다). 미디어 연결은 `ws.transport.get_write_buffer_size()`가 512 KiB를 넘으면 그 프레임을 건너뛰고, 4 MiB를 넘거나 시간 초과면 미디어 연결만 닫는다. `write_limit`만으로는 자동 종료가 보장되지 않으므로 이 감시를 둔다.
    - 카메라 렌더·JPEG 인코딩은 `loop.run_in_executor`(스레드 하나)로 돌려 제어 주기와 tick 송신이 렌더 동안 멈추지 않게 한다. 프레임마다 `frame_id += 1`, `vehicle.record_frame(frame_id)` 후 한 번만 인코딩해 구독자 모두에게 같은 바이트를 보낸다(계약 7절). 머리글에 `session_epoch`를 싣는다.
    - 수신 JSON은 `json.loads(text, parse_constant=_reject)`로 읽고, 실패하면 그 메시지를 버린다(신뢰 요청이면 `request_id`를 읽을 수 있을 때만 `reject(invalid)`).
  - `__main__.py`: `argparse`(`--host 127.0.0.1`, `--port 8765`, `--web-root` 기본값은 이 파일 기준 `../web`)로 `asyncio.run(server.main(...))`. 다른 기기에서 붙으려면 `--host 0.0.0.0`을 명시한다.

- [ ] **Step 4: 통과 확인** — `/home/light/anaconda3/bin/python -m pytest tests/integration/console -q -p no:cacheprovider -W error` → 5 passed

- [ ] **Step 5: 커밋** — `console: fake vehicle WebSocket server (control and media connections, static web files, tick/state/media loops, send-buffer limits, once-encoded camera frames)`

---

### Task 4: 웹 프로토콜 모듈 (`protocol.js`)

**Files:**
- Create: `console/web/js/protocol.js`, `console/web/package.json` (`{"private": true, "type": "module"}`)
- Test: `console/web/test/protocol.test.mjs`

**Interfaces:**
- Produces (ES 모듈, DOM 없음):
  - `export function createSession({clientId, send, now, newId})` → 객체:
    - `onMessage(msg)`: `welcome`·`tick`·`state`·`ack`·`reject`·`targets` 처리
    - `request(type, fields) -> Promise<ack|reject>`: 현재 `session_epoch`·`lease_id`·`control_gen`을 붙여 보내고, `retryMs`(기본 500)마다 같은 `request_id`로 재전송, `ack`/`reject`를 받으면 끝. **`reject`의 `stale_*`는 자동 재전송하지 않고 그대로 돌려준다**
    - `stop(source)`: 권한 없이 보냄
    - `teleopCommand({mode, deadman, speedRatio, curvatureRatio, liftDir, frameId}) -> object|null`: 무장 상태가 아니거나 tick을 받은 적이 없으면 `null`. `seq`는 `arm_gen`이 바뀌면 1부터 다시 센다
    - `echoMessage() -> object|null`
    - getter `state`, `generations`, `hasLease`, `armed`

- [ ] **Step 1: 시험 작성**

```js
import { test } from "node:test";
import assert from "node:assert/strict";
import { createSession } from "../js/protocol.js";

function setup() {
  const sent = [];
  let t = 0, n = 0;
  const s = createSession({ clientId: "a", send: (m) => sent.push(m), now: () => t, newId: () => `id${++n}` });
  s.onMessage({ t: "welcome", api: "console/1", session_epoch: 3, capabilities: {} });
  s.onMessage({ t: "state", state_seq: 1, session_epoch: 3, control_gen: 7, arm_gen: 0, armed: false, mode: "PAUSED" });
  return { s, sent, tick: (dt) => { t += dt; } };
}

test("requests carry current generations", async () => {
  const { s, sent } = setup();
  const p = s.request("lease_request", {});
  assert.equal(sent.at(-1).session_epoch, 3);
  s.onMessage({ t: "ack", request_id: sent.at(-1).request_id, lease_id: "L1" });
  await p;
  s.request("arm", {});
  assert.equal(sent.at(-1).lease_id, "L1");
  assert.equal(sent.at(-1).control_gen, 7);
});

test("stale reject is not auto-resent", async () => {
  const { s, sent } = setup();
  const p = s.request("mission_resume", { mission_id: "M-1" });
  const id = sent.at(-1).request_id;
  s.onMessage({ t: "reject", request_id: id, reason: "stale_control" });
  const r = await p;
  assert.equal(r.reason, "stale_control");
  const count = sent.filter((m) => m.request_id === id).length;
  assert.equal(count, 1);
});

test("unanswered request is resent with the same id", async () => {
  const { s, sent, tick } = setup();
  s.request("mission_pause", { mission_id: "M-1" });
  const id = sent.at(-1).request_id;
  tick(600);
  s.pump();
  assert.equal(sent.filter((m) => m.request_id === id).length, 2);
});

test("teleop command needs arm and a received tick, echoes the latest token", () => {
  const { s } = setup();
  assert.equal(s.teleopCommand({ mode: "direct", deadman: true, speedRatio: 0.5, curvatureRatio: 0, liftDir: 0 }), null);
  s.onMessage({ t: "state", state_seq: 2, session_epoch: 3, control_gen: 7, arm_gen: 4, armed: true, mode: "TELEOP" });
  assert.equal(s.teleopCommand({ mode: "direct", deadman: true, speedRatio: 0.5, curvatureRatio: 0, liftDir: 0 }), null);
  s.onMessage({ t: "tick", session_epoch: 3, state_seq: 9, token: "k9" });
  const c = s.teleopCommand({ mode: "video", deadman: true, speedRatio: 0.5, curvatureRatio: 0, liftDir: 0, frameId: 12 });
  assert.deepEqual(c.echo, { state_seq: 9, token: "k9" });
  assert.equal(c.arm_gen, 4);
  assert.equal(c.seq, 1);
  assert.deepEqual(c.frame_echo, { frame_id: 12 });
});

test("seq restarts when arm_gen changes", () => {
  const { s } = setup();
  s.onMessage({ t: "state", state_seq: 2, session_epoch: 3, control_gen: 7, arm_gen: 4, armed: true, mode: "TELEOP" });
  s.onMessage({ t: "tick", session_epoch: 3, state_seq: 9, token: "k9" });
  const args = { mode: "direct", deadman: true, speedRatio: 0.1, curvatureRatio: 0, liftDir: 0 };
  s.teleopCommand(args); s.teleopCommand(args);
  s.onMessage({ t: "state", state_seq: 3, session_epoch: 3, control_gen: 8, arm_gen: 5, armed: true, mode: "TELEOP" });
  assert.equal(s.teleopCommand(args).seq, 1);
});

test("a new session_epoch drops lease and tick", () => {
  const { s } = setup();
  s.onMessage({ t: "tick", session_epoch: 3, state_seq: 9, token: "k9" });
  s.onMessage({ t: "welcome", api: "console/1", session_epoch: 4, capabilities: {} });
  assert.equal(s.hasLease, false);
  assert.equal(s.echoMessage(), null);
});
```

- [ ] **Step 2: 실패 확인** — `node --test console/web/test/` → `Cannot find module ../js/protocol.js`

- [ ] **Step 3: 구현** — 시험이 요구하는 동작만. 대기 중인 요청은 `Map<request_id, {msg, resolve, lastSent}>`에 두고 `pump()`가 `now() - lastSent >= retryMs`인 것을 다시 보낸다(`main.js`가 100 ms마다 호출). `ack`·`reject`에 `lease_id`가 있으면 그것으로 바꾼다. `state`에서 `control_gen`·`arm_gen`·`armed`·`mode`를 갱신한다. 새 `welcome`의 `session_epoch`가 다르면 lease·tick·대기 요청을 모두 버리고, 대기 중인 Promise는 `{t: "reject", reason: "stale_epoch"}`로 끝낸다.

- [ ] **Step 4: 통과 확인** — `node --test console/web/test/` → 6 passed

- [ ] **Step 5: 커밋** — `console(web): protocol session (generations on every request, same-id resend, no auto-resend after a stale reject, teleop command with token echo and per-arm seq)`

---

### Task 5: 입력과 표시 추적 (`inputs.js`, `frames.js`)

**Files:**
- Create: `console/web/js/inputs.js`, `console/web/js/frames.js`
- Test: `console/web/test/inputs.test.mjs`, `console/web/test/frames.test.mjs`

**Interfaces:**
- Produces:
  - `export function createInputs({onRelease})` → `{ setTouch(partial), setKeys(partial), setPad(snapshot|null), visibility(hidden), focus(focused), read() -> {deadman, speedRatio, curvatureRatio, liftDir, source} }`. `onRelease(reason)`는 가림·포커스 잃음·패드 분리 때 한 번 호출된다(무장 해제 요청과 `input_state` 송신은 `main.js`가 한다). 해제 뒤에는 모든 입력이 0이 되고, 새 눌림이 올 때까지 `deadman=false`.
  - `export function createFrameTracker({raf})` → `{ shouldDraw(sessionEpoch, frameId), drawn(sessionEpoch, frameId), reset(sessionEpoch), displayedFrameId }`. `shouldDraw`는 **디코딩이 끝난 뒤 그리기 직전에** 호출한다: 현재 epoch이고 이미 그린 프레임보다 새 `frame_id`일 때만 `true`(이전 세션이나 순서가 뒤바뀐 디코딩이 새 화면을 덮지 않게). `drawn`은 `raf` 콜백 안에서만 `displayedFrameId`를 갱신한다. `reset`은 표시 프레임을 비우고, 그 전에 예약된 콜백과 다른 epoch의 `drawn`을 무시한다(재시작한 차량의 같은 `frame_id`를 이전 영상으로 오인하지 않게).
  - `export function padSnapshot(gamepad) -> {id, axes, buttons: {deadman, liftUp, liftDown, stop}} | null` — 원시 Gamepad API 객체에서 부울을 만든다. `gamepad.mapping !== "standard"`이면 `null`을 돌려 쓰지 않는다(축 번호가 다른 패드로 급발진하지 않게). 화면은 "표준 매핑이 아닌 게임패드" 안내를 띄운다. 브라우저는 패드 버튼을 한 번 누르기 전까지 패드를 보여 주지 않으므로 "버튼을 한 번 누르세요" 안내도 둔다. 버튼은 `.pressed`(또는 `.value > 0.5`)로 읽는다. 버튼 객체 자체의 참·거짓을 쓰지 않는다

- [ ] **Step 1: 시험 작성**

```js
import { test } from "node:test";
import assert from "node:assert/strict";
import { createInputs } from "../js/inputs.js";
import { createFrameTracker } from "../js/frames.js";
import { padSnapshot } from "../js/inputs.js";

test("visibility hidden releases and requests disarm", () => {
  const released = [];
  const i = createInputs({ onRelease: (r) => released.push(r) });
  i.setTouch({ deadman: true, x: 0, y: 0.8 });
  assert.equal(i.read().deadman, true);
  i.visibility(true);
  assert.deepEqual(released, ["hidden"]);
  const r = i.read();
  assert.equal(r.deadman, false);
  assert.equal(r.speedRatio, 0);
});

test("held touch stays released until pressed again", () => {
  const i = createInputs({ onRelease: () => {} });
  i.setTouch({ deadman: true, x: 0, y: 0.8 });
  i.focus(false);
  i.focus(true);
  i.setTouch({ deadman: true, x: 0, y: 0.8 });   // same press reported again
  assert.equal(i.read().deadman, false);
  i.setTouch({ deadman: false, x: 0, y: 0 });
  i.setTouch({ deadman: true, x: 0, y: 0.8 });   // a new press
  assert.equal(i.read().deadman, true);
});

test("gamepad disconnect releases", () => {
  const released = [];
  const i = createInputs({ onRelease: (r) => released.push(r) });
  i.setPad({ id: "pad", axes: [0, -1, 0.5, 0], buttons: { deadman: true, liftUp: false, liftDown: false, stop: false } });
  assert.equal(i.read().source, "gamepad");
  i.setPad(null);
  assert.deepEqual(released, ["gamepad_lost"]);
});

test("keyboard overrides idle touch; gamepad axes override keyboard", () => {
  const i = createInputs({ onRelease: () => {} });
  i.setKeys({ deadman: true, w: true, d: true });
  let r = i.read();
  assert.equal(r.speedRatio, 1); assert.equal(r.curvatureRatio, -1); assert.equal(r.source, "keyboard");
  i.setPad({ id: "pad", axes: [0, 0.5, 0, 0], buttons: { deadman: true, liftUp: false, liftDown: false, stop: false } });
  r = i.read();
  assert.equal(r.speedRatio, -0.5); assert.equal(r.source, "gamepad");
});

test("frame is displayed only after the next animation frame", () => {
  const queue = [];
  const f = createFrameTracker({ raf: (cb) => queue.push(cb) });
  f.reset(1);
  f.drawn(1, 5);
  assert.equal(f.displayedFrameId, null);
  queue.shift()();
  assert.equal(f.displayedFrameId, 5);
});

test("a new session drops the displayed frame and pending callbacks", () => {
  const queue = [];
  const f = createFrameTracker({ raf: (cb) => queue.push(cb) });
  f.reset(1);
  f.drawn(1, 5);
  f.reset(2);
  queue.shift()();
  assert.equal(f.displayedFrameId, null);
  f.drawn(1, 6);  // late frame from the old session
  queue.shift()?.();
  assert.equal(f.displayedFrameId, null);
});

test("a decode that finishes after a newer session is not drawn", () => {
  const queue = [];
  const f = createFrameTracker({ raf: (cb) => queue.push(cb) });
  f.reset(1);
  // decode of (1, 8) starts; meanwhile the vehicle restarts and (2, 1) is drawn
  f.reset(2);
  assert.equal(f.shouldDraw(2, 1), true);
  f.drawn(2, 1); queue.shift()();
  assert.equal(f.shouldDraw(1, 8), false);   // old session's decode completes late
  assert.equal(f.shouldDraw(2, 1), false);   // not newer than what is shown
  assert.equal(f.shouldDraw(2, 2), true);
});

test("gamepad buttons are read from .pressed, not object truthiness", () => {
  const off = { pressed: false, value: 0 }, on = { pressed: true, value: 1 };
  const raw = { id: "pad", connected: true, mapping: "standard", axes: [0, 0, 0, 0], buttons: Array.from({ length: 17 }, () => off) };
  assert.equal(padSnapshot(raw).buttons.deadman, false);
  raw.buttons = raw.buttons.map((b, i) => (i === 5 ? on : b));
  assert.equal(padSnapshot(raw).buttons.deadman, true);
  assert.equal(padSnapshot(null), null);
  assert.equal(padSnapshot({ ...raw, mapping: "" }), null);  // non-standard layout is ignored
});
```

- [ ] **Step 2: 실패 확인** — `node --test console/web/test/` → 모듈 없음
- [ ] **Step 3: 구현** — 우선순위는 게임패드 축(0.08 데드존 밖) > 키보드 > 터치. 곡률 비율은 오른쪽(+x) 입력이 음수(우회전)다. 게임패드 매핑은 `padSnapshot`이 한다: `buttons[5]` 또는 `buttons[7]`의 `.pressed`(또는 `.value > 0.5`) → deadman, `12/13` → 승강, `1` → stop, 축은 `-axes[1]` 속도, `axes[2]`(없으면 `axes[0]`) 조향. 해제 뒤 같은 눌림이 계속 보고되면 무시하고, 한 번 떼었다가 다시 누를 때 재개한다.
- [ ] **Step 4: 통과 확인** — `node --test console/web/test/` → 14 passed
- [ ] **Step 5: 커밋** — `console(web): input merging with release on hide, blur and gamepad loss; displayed-frame tracking after the next animation frame`

---

### Task 6: 화면 (v2) — 지도·카메라·패널

**Files:**
- Create: `console/web/index.html`, `console/web/app.css`, `console/web/js/media.js`, `console/web/js/map.js`, `console/web/js/camera.js`, `console/web/js/ui.js`, `console/web/js/main.js`
- Test: 렌더 확인(아래 Step 4), `node --test`로 `media.js`의 머리글 해석만

**Interfaces:**
- Consumes: `createSession`, `createInputs`, `createFrameTracker`
- Produces: `media.parseFrame(arrayBuffer) -> {head, body}` (Task 3 형식)

- [ ] **Step 1: 시험 작성** — `console/web/test/media.test.mjs`

```js
import { test } from "node:test";
import assert from "node:assert/strict";
import { parseFrame } from "../js/media.js";

test("parses length-prefixed header and body", () => {
  const head = new TextEncoder().encode(JSON.stringify({ t: "camera", frame_id: 3 }));
  const buf = new Uint8Array(4 + head.length + 2);
  new DataView(buf.buffer).setUint32(0, head.length);
  buf.set(head, 4); buf.set([0xff, 0xd8], 4 + head.length);
  const f = parseFrame(buf.buffer);
  assert.equal(f.head.frame_id, 3);
  assert.deepEqual([...new Uint8Array(f.body)], [0xff, 0xd8]);
});
```

- [ ] **Step 2: 실패 확인** — `node --test console/web/test/` → 모듈 없음
- [ ] **Step 3: 구현** — v1 목업(스크래치패드 `mockup_v1.html`, 게시본 claude.ai/artifact/8BcZFguhdDZDTUC8cHKysH)의 마크업·CSS·그리기 코드를 옮기고, 데이터 출처를 가짜 시뮬레이션에서 서버 메시지로 바꾼다. v2에서 바꾸는 것:
  1. 상단: 연결 대상 선택은 1단계에 `가짜 차량`만 활성, 나머지는 "준비 중" 비활성. 모드 옆에 정지 종류 칩(`없음 / 제어 감속 / 안전 선점`). 세대(`epoch · control · arm`)를 작은 고정폭 글자로 표시.
  2. 작업 지시: 대상 목록은 서버 `targets`에서, 제출 시 `targets_revision`을 싣는다. 재개를 누르면 `resume_steps` 체크리스트 4단계를 보여 준다. 수동 조종이 금지된 상태면 `teleop_forbidden_reason`과 허용된 복구 동작(`축 따라 빼기`) 버튼을 보여 준다.
  3. 차량 상태: 조향 한계 문구를 "모델 계산값 · 최소 회전 반경 2.73 m(뒤 차축)"으로 바꾼다. `phase_elapsed_s`(작업 시간)와 `freshness`(신선도)를 구분해 보여 준다. `null` 값은 "측정 안 됨"으로 표시한다.
  4. 지도: 서버 격자(바이너리) + 스캔 + 자세 + `plan` 경로(후진 구간 점선) + 수동 조종 예상 궤적(차체 전체 footprint 외곽 두 줄).
  5. 카메라: 프레임 머리글의 `session_epoch`가 현재와 다르면 버린다. JPEG을 `createImageBitmap`으로 디코딩하고, **디코딩이 끝난 뒤 `frames.shouldDraw(epoch, frame_id)`가 참일 때만** 그린 다음 `frames.drawn(epoch, frame_id)`. 검출 오버레이와 예상 궤적 바닥 투영은 `capabilities.camera_model`로 그린다. 영상 나이는 서버가 `state`에 싣지 않으므로 화면에는 "마지막 프레임 이후 경과(브라우저 기준, 참고용)"로만 표시하고, 판정은 서버가 한다는 문구를 둔다.
  6. 수동 조종: 육안/영상 모드 토글. 영상 신선도로 무장이 풀린 직후 육안으로 바꾸면 "차량이 직접 보인다" 확인 버튼을 거친다. 텔레옵 명령은 `requestAnimationFrame` 한 번 안에서 차례로 만들되, **전송은 직전 전송 뒤 33 ms(약 30 Hz)가 지났을 때만** 한다(고주사율 화면에서 같은 tick 회신을 초당 백 번 넘게 보내지 않게). rAF가 느려지거나 멈추면 명령도 느려지거나 멈추고, 그 결과는 만료 정지다(의도한 동작). 순서: `navigator.getGamepads()`를 새로 읽어 `inputs.setPad(padSnapshot(...))` → `inputs.read()` → `session.teleopCommand(...)` → 전송. 터치는 `pointerdown`에서 `setPointerCapture`, `pointerup`·`pointercancel`·`lostpointercapture`에서 해제하고 그때마다 `setTouch`. 키보드는 `keydown`/`keyup`. rAF가 멈추면(탭 가림) 명령도 멈춘다(설계 5.2). 세션의 `welcome`이 새 epoch이면 `frames.reset(epoch)`. `client_id`는 탭마다 한 번 만든 예측할 수 없는 값을 `sessionStorage`에 두고(읽기·쓰기는 try/catch), 새로고침하면 같은 값으로 다시 붙어 재연결 유예 안에 조종 권한을 회수한다. 탭이 가려지면 `onRelease`에서 `input_state` 전송과 `disarm` 요청.
  7. 통신: 끊김 시 동작 문구를 "자동 주행은 계속 · 수동 조종은 만료로 정지"로 바꾼다. `끊김 시험` 버튼은 가짜 차량 전용으로, 제어 연결을 3초 닫았다가 다시 연다.
  8. 게임패드: `localhost`가 아니면 "게임패드는 localhost(ssh -L) 접속에서만" 안내를 보인다.
- [ ] **Step 4: 확인**
  - `node --test console/web/test/` → 15 passed (Task 5의 패드 시험에 비표준 매핑 단언이 추가됨, 개수는 같다)
  - 가짜 서버를 띄우고 헤드리스 브라우저로 데스크톱 폭(1440)과 폰 폭(400) 스크린숏을 한 장씩 찍어, 지도·카메라·패널이 겹치지 않는지 확인한다. 브라우저 자동화 프로세스는 확인 뒤 정리한다.
  - 손으로 흐름 확인: 권한 받기 → 작업 시작 → 수동 조종으로 보류 → 무장 → 누르고 있기로 이동 → 탭 전환 시 무장 해제 → STOP → 재무장 → 재개 체크리스트 → 완료.
- [ ] **Step 5: 커밋** — `console(web): v2 screens on the fake vehicle (targets with revision, stop kinds and generations, resume checklist, teleop-forbidden reasons and withdraw, model turning radius 2.73 m, direct/video modes with the direct-view confirmation, rAF teleop loop)`

---

### Task 7: 문서 마무리와 PR

**Files:**
- Modify: `docs/design/2026-10-13-operator-console.md` (6절 C1 상태), `console/README.md` (확인한 브라우저·장치 목록)

- [ ] **Step 1**: 전체 시험 — `python -m pytest tests/unit/console tests/integration/console -q -p no:cacheprovider -W error`, `node --test console/web/test/`, `ruff check console tests/unit/console tests/integration/console`, `ruff format --check console tests/unit/console tests/integration/console`. 전체 CPU 스위트는 이 변경이 `forklift_core`를 건드리지 않으므로 생략하고, 그 사실을 PR에 적는다.
- [ ] **Step 2**: 설계 6절 C1 행에 "가짜 차량으로 동작(커밋)"을 적고, README에 확인한 조합(Chrome/Firefox, 데스크톱 폭·폰 폭)을 적는다.
- [ ] **Step 3**: Codex 구현 리뷰를 지적이 없을 때까지 반복한다(`adversarial-review`, 초점: 계약 규칙과 가짜 차량의 일치, 텔레옵 해제 경로).
- [ ] **Step 4**: 커밋·푸시, `docs/operator-console` PR #21에 이어 올린다.

---

## 자체 검토

- **스펙 대응**:
  - 설계 4.1–4.5 → Task 2 (모드, 정지 종류, 재개 절차, 허가 흉내, 상태별 허용)
  - 설계 5.1–5.5 → Task 2·4·5·6
  - 설계 7.3 → Task 3 (연결 분리, 버퍼 상한, 한 번 인코딩)
  - 설계 8 → Task 6
  - 계약 1–7 → Task 2·3·4
  - 계약 8절(ROS 2)과 실차·Isaac 연결은 C1 범위 밖이다
- **자리표시**: 없음. Task 2 Step 3과 Task 6 Step 3은 코드 전체 대신 규칙 목록과 출처 파일(v1 목업)을 준다. 규칙마다 Step 1의 시험이 대응한다.
- **이름 일관성**: `control_gen`, `arm_gen`, `lease_id`, `session_epoch`, `state_seq`, `frame_id`, `teleop_forbidden_reason`, `targets_revision`은 계약 초안과 같다. 가짜 차량 내부 이름(`applied_speed_mps`, `stopped_confirmed`, `last_teleop_reject`, `debug_output`, `debug_set_phase`)은 Task 2 시험과 같다.
- **Codex 계획 검토 1차 반영**: STOPPED 잠금을 작업·복구까지 확대, 저장된 텔레옵 명령의 세대 재검사와 무효화, 인스턴스마다 다른 `session_epoch`와 표시 프레임 epoch 구분, STOP도 결과 캐시 먼저, 입력 검증과 NaN 거부, 적재 footprint 허가와 시험, `websockets>=13`·`package.json`, 렌더를 실행기로 분리하고 송신 감시·느린 구독자 시험, 서버 시험의 상태 대기 조건과 lease 유지, Gamepad `.pressed`·같은 rAF 안 polling, lease 최초 기준·정지 확인 초기값·`debug_set_phase` 동기화, ruff `UP035`·`B008`. 계획의 코드 블록은 구현 때 `ruff format`을 적용한다.
- **Codex 계획 검토 2차 반영**: STOPPED 중 `mission_cancel`은 잠금을 유지하고 `lease_request`는 허용, 모든 모드 전이에서 무장 해제, epoch는 인스턴스마다 무작위 48비트(연속 생성 200개 시험), 디코딩 후 그리기 직전 `shouldDraw` 검사(경합 시험), 관찰자 시험은 시험 내내 echo를 보내는 작업으로 lease 유지, 느린 구독자 시험은 프레임 부풀리기로 실제 정체를 만들고 건너뛴·닫은 수로 도달을 확인.
- **Codex 계획 검토 3차 반영**: 서버·시험 연결 `compression=None`, 패딩은 무작위 바이트, `close_timeout=1.0`과 `Handle.close()`의 강제 종료, 시험 제한 20초.
- **Codex 계획 검토 최종: 새 P1/P2 없음(수렴).** P3 표기(lease 예시·서버 호출의 `conn_id`)도 반영했다.
- **agy 반영분 재검토 반영**: lease를 연결(`conn_id`)에 묶고, 그 연결이 끊길 때만 해제(같은 `client_id` 중복 연결·늦은 종료 시험 2개). 33 ms는 최소 송신 간격일 뿐 왕복 지연 예산을 증명하지 않는다는 점을 기록.
- **독립 전제 검토(agy) 반영**: 정적 파일 MIME(시험), 텔레옵 전송 30 Hz 상한, `client_id`를 탭별 `sessionStorage`로 유지, 게임패드 `mapping === "standard"`만 사용(시험), Node 18 요구, 가짜 차량의 한계(자기 명령으로 정지 확인·동기 처리)를 README에 명시. 기각: 카메라 지연 모의 파라미터 추가(이미 `Config`로 조정 가능), 루프백 버퍼 `SO_SNDBUF` 조정(1 MB 무작위 패딩·15 Hz와 클라이언트 `max_queue=1`로 정체가 충분히 빨리 생긴다. 실패하면 그때 조정).
- **합의 반영 재검토 3차 반영**: 정지 요청(STOP, 새 `mission_pause`, `mode_request(PAUSED)`)은 모드가 같아도 `_boundary()`로 세대를 한 번 올림(시험 2개).
- **합의 반영 재검토 2차 반영**: `_set_mode` 하나로 모든 모드 전이의 부수효과(무장 해제·명령 폐기·`control_gen`·`stop_kind`)를 정의, 새 시험은 대기 중 echo로 lease 유지(`_idle`), PAUSED 직후·AUTO 진입 직후 상태 검사.
- **합의 반영 재검토 반영**: 최종 차단은 TELEOP에서만(PAUSED 제어 감속·AUTO 출력 시험 추가), 승강 해제 첫 주기만 주행 차단으로 시험 기대값 수정.
- **합의 반영 검토 반영**: 주행·승강 배타(`lift_active`, 승강은 정지 확인 뒤), 주행 금지를 slew 뒤 최종 출력에도 적용, 허가 방향은 정지 확인 전까지 마지막 이동 부호 유지(시험 3개).
- **메인 개발 합의(2026-10-13) 반영**: STOP은 캐시 없이 항상 실행(시험을 '재무장 뒤 중복 STOP은 다시 정지'로 바꿈), 결과 캐시 상한 `busy`(시험), 재개 단계 이름, 매 주기 주행·승강 허가(시험), 허가 방향은 실제 이동 방향·검사 속도는 최댓값, `command`·`applied` 상태 필드, `serve` 옵션 명시.
- **시험 흐름 재확인**: 재무장 시험은 정지 확인(0.02 m/s 미만 0.5 s 유지) 뒤에만 통과하도록 step을 넣었다. 후진 경로의 yaw는 감싸서(±π) 저장한다.
- **수치 재계산**:
  - tick 20 Hz는 50 ms이고 0.30/4 = 75 ms 이하다.
  - 영상 마감 시험: acquired 100.00 + 0.50 − 0.05 = 100.45 < 명령 마감 100.40 + 0.30 = 100.70이므로 영상이 먼저다.
  - 적재 상한: 0.5 × 0.6 = 0.30 m/s.
  - slew 반례: 0.5 − 1.5/120 = 0.4875 > 0.1이므로 상한 절단이 필요하다.
