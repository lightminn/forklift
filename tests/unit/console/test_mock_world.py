import math

from console.mock_vehicle import paths, world


def test_raycast_hits_east_wall():
    w = world.default_world()
    assert math.isclose(world.raycast(w, 1.0, 0.5, 1.0, 0.0, 30.0), 13.0, abs_tol=1e-9)


def test_raycast_hits_barrel_before_wall():
    w = world.World(
        size_m=(14.0, 9.0),
        circles=[(5.0, 2.0, 0.3)],
        boxes=[],
        pallets=[],
        destinations={},
        home=world.Pose(1.0, 2.0, 0.0),
    )
    assert math.isclose(world.raycast(w, 1.0, 2.0, 1.0, 0.0, 30.0), 3.7, abs_tol=1e-9)


def test_permission_stops_close_obstacle_and_slows_near_one():
    w = world.World(
        size_m=(14.0, 9.0),
        circles=[(3.0, 2.0, 0.3)],
        boxes=[],
        pallets=[],
        destinations={},
        home=world.Pose(1.0, 2.0, 0.0),
    )
    nose = world.CHASSIS.nose_from_rear_m
    # fork tip 0.10 m from the barrel surface: stop
    limit, reason = world.permission(
        w, world.Pose(3.0 - 0.3 - 0.10 - nose, 2.0, 0.0), 0.3, 0.0, None
    )
    assert limit == 0.0 and reason.startswith("occupied")
    # 0.50 m away: slow
    limit, reason = world.permission(
        w, world.Pose(3.0 - 0.3 - 0.50 - nose, 2.0, 0.0), 0.3, 0.0, None
    )
    assert 0.0 < limit < 0.5 and reason.startswith("slow")


def test_loaded_footprint_is_wider_and_longer():
    load = world.Pallet(
        "P", "epal6", world.Pose(0.0, 0.0, 0.0), 0.6, 0.8, state="carried"
    )
    assert math.isclose(world.loaded_nose_m(load), 1.06 - 0.33 + 0.6)
    pose = world.Pose(1.0, 4.0, 0.0)
    box_x = (
        1.0 + world.loaded_nose_m(load) + 0.15 + 0.05
    )  # 0.15 m ahead of the load, 0.40 m to the left
    w = world.World(
        size_m=(14.0, 9.0),
        circles=[],
        boxes=[(box_x, 4.40, 0.1, 0.1)],
        pallets=[],
        destinations={},
        home=pose,
    )
    empty = world.permission(w, pose, 0.3, 0.0, None)
    loaded = world.permission(w, pose, 0.3, 0.0, load)
    assert empty[0] > 0.0  # the bare truck is narrower and shorter: at most slowed
    assert loaded[0] == 0.0 and loaded[1].startswith("occupied")


def test_straight_hermite_leg_is_straight_and_reaches_end():
    leg = paths.hermite_leg(
        world.Pose(0.0, 0.0, 0.0), world.Pose(2.0, 0.0, 0.0), 1, 0.3
    )
    assert math.isclose(leg.length_m, 2.0, abs_tol=1e-6)
    end = paths.advance(leg, 5.0)
    assert math.isclose(end.x_m, 2.0, abs_tol=1e-9) and math.isclose(
        end.yaw_rad, 0.0, abs_tol=1e-9
    )


def test_reverse_leg_keeps_heading_opposite_to_motion():
    leg = paths.hermite_leg(
        world.Pose(2.0, 0.0, 0.0), world.Pose(0.0, 0.0, 0.0), -1, 0.2
    )
    mid = paths.advance(leg, 1.0)
    assert math.isclose(mid.yaw_rad, 0.0, abs_tol=1e-6)
