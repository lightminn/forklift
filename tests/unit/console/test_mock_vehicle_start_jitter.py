"""The fake vehicle's start-up settling (queue plan D5's one exception, user decision 2026-10-25): an injection refuses
the next mission_submit with not_stopped and drops the stop confirmation, which the stand detector gives back after
its hold -- the case the console's one automatic resubmit is for."""

import itertools

import pytest

from console.mock_vehicle.core import Config, MockVehicle

from test_mock_vehicle_core import Clock, lease, req


@pytest.fixture
def env():
    clock = Clock()
    tokens = (f"tok{i}" for i in itertools.count())
    return MockVehicle(clock, lambda: next(tokens), Config()), clock


def submit(v, epoch, lid):
    return req(v, "a", "mission_submit", epoch, lid, pallet_id="P-01", destination_id="D-A",
               targets_revision=v.targets_revision)


def test_the_next_submit_is_refused_not_stopped_and_the_stand_comes_back_after_its_hold(env):
    v, clock = env
    epoch, lid = lease(v)
    v.step(0.1)  # standing from the start: the stand confirmed after its hold
    clock.t += 0.6
    v.step(0.6)
    assert v.stopped_confirmed
    v.inject("start_not_stopped")
    r = submit(v, epoch, lid)
    assert r["t"] == "reject" and r["reason"] == "not_stopped"
    assert v.stopped_confirmed is False and v.mission is None
    clock.t += 0.4
    v.step(0.4)
    assert v.stopped_confirmed is False  # 0.4 s: inside the 0.5 s hold
    clock.t += 0.2
    v.step(0.2)
    assert v.stopped_confirmed is True
    epoch, lid = lease(v)  # (this test sends no heartbeats; the lease taken again)
    assert submit(v, epoch, lid)["t"] == "ack", "once only"
