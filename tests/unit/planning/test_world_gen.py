from dataclasses import replace

import numpy as np
import pytest

from forklift_core.planning.pallet_mission import (
    SyntheticMissionGeometry,
    make_transport_planner_config,
    site_poses,
)
from forklift_core.planning.world_gen import (
    PALLET_KEEP_M,
    STRUCTURES,
    TRUCK_GAP_M,
    holding_box,
    holding_exit,
    make_world,
    pallet_box,
    run_up_box,
)
from forklift_core.planning.world_mission import Dispatcher, keep_outs, plan_job

GEOMETRY = SyntheticMissionGeometry()
SEEDS = (100, 101, 102)


@pytest.fixture(scope="module")
def worlds():
    return {
        (s, seed): make_world(s, seed, geometry=GEOMETRY, trucks=10)
        for s in STRUCTURES
        for seed in SEEDS
    }


def test_every_structure_builds_for_the_dev_seeds(worlds):
    for (structure, seed), world in worlds.items():
        assert world.structure == structure and world.seed == seed
        assert len(world.sources()) >= 2 and len(world.sinks()) >= 2
        assert len(world.holding) >= 10
        assert len({s.name for s in world.slots}) == len(world.slots)


def test_a_seed_gives_the_same_floor_and_another_seed_another(worlds):
    again = make_world("rack", 100, geometry=GEOMETRY, trucks=10)
    assert again == worlds[("rack", 100)]
    assert worlds[("rack", 100)].size_m != worlds[("rack", 101)].size_m


def test_places_and_run_ups_are_on_clear_floor(worlds):
    for world in worlds.values():
        walls = [b for _, b in world.obstacles]
        inner = world.bounds
        keeps = []
        for slot in world.slots:
            pallet, run = pallet_box(slot.site, GEOMETRY), run_up_box(slot.site, GEOMETRY)
            for box in (pallet, run):
                assert inner.x_min_m <= box.x0 and box.x1 <= inner.x_max_m
                assert inner.y_min_m <= box.y0 and box.y1 <= inner.y_max_m
                assert not any(box.overlaps(w) for w in walls), (world.structure, slot)
            keeps.append(pallet.grow(PALLET_KEEP_M))
        for i, a in enumerate(keeps):
            assert not any(a.overlaps(b) for b in keeps[i + 1 :])
        for pose in world.holding:
            body = holding_box(pose, GEOMETRY, 0.0)
            out = holding_box(holding_exit(pose), GEOMETRY, 0.0)
            assert not any(body.overlaps(w) or out.overlaps(w) for w in walls)


def test_the_multi_hall_mixes_both_kinds(worlds):
    for seed in SEEDS:
        names = " ".join(name for name, _ in worlds[("multi", seed)].zones)
        assert "랙" in names and "공정" in names


def test_keep_outs_leave_out_only_the_jobs_own_places(worlds):
    world = worlds[("shop", 100)]
    source, sink = world.sources()[0], world.sinks()[0]
    base = len(world.obstacles)
    rects = keep_outs(world, GEOMETRY, own_slots=(source.name, sink.name), own_holding=(0, 1))
    assert len(rects) == base + (len(world.slots) - 2) + (len(world.holding) - 2)
    held = holding_box(world.holding[2], GEOMETRY, TRUCK_GAP_M).rect()
    assert held in rects


def test_dispatcher_hands_out_pallets_and_places_once(worlds):
    world = worlds[("rack", 101)]
    dispatcher = Dispatcher(world, np.random.default_rng(0), fill=1.0)
    places = dispatcher.spawn(4)
    assert len(set(places)) == 4
    jobs = [dispatcher.next_job(k, places[k]) for k in range(4)]
    assert all(j is not None for j in jobs)
    assert len({j.source.name for j in jobs}) == 4
    assert len({j.sink.name for j in jobs}) == 4
    assert len({j.home for j in jobs}) == 4
    assert [j.number for j in jobs] == [1, 2, 3, 4]
    # Ranks: one per route given, each behind all before; job numbers unaffected.
    assert [dispatcher.next_rank() for _ in range(3)] == [1, 2, 3]
    assert dispatcher.next_job(0, places[0]).number == 5
    job = jobs[0]
    dispatcher.lifted(job, 10.0)
    assert not dispatcher.occupied[job.source.name]
    dispatcher.lowered(job, 20.0)
    assert dispatcher.occupied[job.sink.name]
    dispatcher.tick(10.0 + dispatcher.restock_s)
    assert dispatcher.occupied[job.source.name]
    dispatcher.tick(20.0 + dispatcher.ship_s)
    assert not dispatcher.occupied[job.sink.name]


def test_a_pallet_does_not_come_back_under_a_truck(worlds):
    world = worlds[("rack", 101)]
    dispatcher = Dispatcher(world, np.random.default_rng(0), fill=1.0)
    places = dispatcher.spawn(1)
    job = dispatcher.next_job(0, places[0])
    dispatcher.lifted(job, 10.0)
    dispatcher.lowered(job, 20.0)
    standing = {job.source.name, job.sink.name}
    later = 20.0 + dispatcher.ship_s
    dispatcher.tick(later, lambda name: name in standing)
    # The restock waits for the truck to clear; shipping out does not.
    assert not dispatcher.occupied[job.source.name] and not dispatcher.occupied[job.sink.name]
    assert dispatcher.held_back == {(10.0 + dispatcher.restock_s, job.source.name)}
    standing.clear()
    dispatcher.tick(later + 0.1, lambda name: name in standing)
    assert dispatcher.occupied[job.source.name] and dispatcher.timers == []


def test_a_larger_scale_stretches_the_halls_and_fills_them():
    small = make_world("rack", 100, geometry=GEOMETRY, trucks=20)
    assert make_world("rack", 100, geometry=GEOMETRY, trucks=20, scale=1.0) == small
    big = make_world("rack", 100, geometry=GEOMETRY, trucks=20, scale=1.5)
    assert big.size_m == pytest.approx(tuple(1.5 * v for v in small.size_m), abs=0.02)
    assert len(big.slots) > 2 * len(small.slots)
    assert len(big.holding) == len(small.holding)


def test_random_jobs_take_any_pallet_to_any_free_slot(worlds):
    world = worlds[("shop", 101)]
    dispatcher = Dispatcher(world, np.random.default_rng(0), fill=0.5, od="random")
    assert any(dispatcher.occupied[s.name] for s in world.sinks())
    places = dispatcher.spawn(8)
    jobs = [dispatcher.next_job(k, places[k]) for k in range(8)]
    assert {(j.source.role, j.sink.role) for j in jobs} > {("source", "sink")}
    dispatcher.lifted(jobs[0], 10.0)
    dispatcher.lowered(jobs[0], 20.0)
    assert dispatcher.timers == []  # nothing restocks or ships: the pallets only move


def test_a_chained_job_leaves_from_the_drop_but_not_with_its_pallet(worlds):
    world = worlds[("shop", 101)]
    dispatcher = Dispatcher(world, np.random.default_rng(0), fill=0.5, od="random")
    places = dispatcher.spawn(1)
    job = dispatcher.next_job(0, places[0])
    dispatcher.left(job)
    dispatcher.lifted(job, 10.0)
    dispatcher.lowered(job, 20.0)
    for name in dispatcher.occupied:  # the pallet just put down is the only one
        dispatcher.occupied[name] = name == job.sink.name
    dispatcher.free_home(job)
    assert dispatcher.next_job(0, -1, after=job.sink) is None
    dispatcher.take_home(job)
    assert dispatcher.holding_taken == {job.home: 0}
    dispatcher.free_home(job)
    again = dispatcher.next_job(0, -1)
    assert again.start == -1 and again.source == job.sink
    assert dispatcher.holding_taken == {again.home: 0}


def test_a_chained_job_plans_from_in_front_of_the_pallet_it_put_down(worlds):
    world = worlds[("shop", 101)]
    dispatcher = Dispatcher(world, np.random.default_rng(0), fill=0.5, od="random")
    places = dispatcher.spawn(1)
    job = dispatcher.next_job(0, places[0])
    chained = dispatcher.next_job(0, -1, after=job.sink)
    config = make_transport_planner_config(max_expansions=30000)
    travel = replace(config, obstacle_heuristic_resolution_m=0.25)
    start = site_poses(job.sink.site, GEOMETRY)["withdrawn"]
    result = plan_job(chained, world, config, travel, GEOMETRY, first_stage="to_pickup", start=start)
    assert result.success, result.status
    assert list(result.stages)[0] == "to_pickup"
    # The pallet's keep-out covers where the truck stands after withdrawing.
    plain = plan_job(replace(chained, after=None), world, config, travel, GEOMETRY, first_stage="to_pickup", start=start)
    assert plain.status == "to_pickup:invalid_start"
