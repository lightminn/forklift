"""B-P2c (start relocalization plan 1.2 step 4; Codex B-P2c design rounds 1-3): one console for the whole session --
the parts a relocalizing runner opens before its mission core exists (arbiter, channels, link, media store, welcome,
server), adopted by the live runtime instead of new ones, so a STOP, the request ledger and every number live on; the
port file keeps what the runner adds (localization ready) through every later write."""

import json

import pytest

pytest.importorskip("websockets")

from forklift_core.safety.arbiter import ArbiterConfig  # noqa: E402
from sim.isaac import live_runtime as L  # noqa: E402
from test_live_runtime import DT, _MADE, Clock, runtime  # noqa: E402, F401


def _parts(tmp_path, config=None, **kw):
    return L.make_console(config=config or ArbiterConfig(tick_s=DT), session_epoch=9, port=0, **kw)


def test_the_welcome_is_one_function_for_both_paths():
    a = L.make_welcome(9, camera_hz=10.0, supervised=True)
    assert a["session_epoch"] == 9 and a["capabilities"]["sim_reset"] is True
    assert a["capabilities"]["mission"] is True and "relocalize" not in a["capabilities"]


def test_the_early_console_says_it_relocalizes_and_has_no_mission_yet(tmp_path):
    parts = _parts(tmp_path)
    assert parts.welcome["capabilities"]["relocalize"] is True and parts.arbiter.mission is None
    assert parts.arbiter.config.request_tokens is True and parts.session_epoch == 9


def test_the_runtime_adopts_the_parts_instead_of_building_its_own(tmp_path):
    parts = _parts(tmp_path)
    parts.arbiter.stop(0.0, "stop:early")  # a STOP before the mission core existed
    rt, core, _, _ = runtime(tmp_path, parts=parts, config=ArbiterConfig(tick_s=DT))
    assert rt.arbiter is parts.arbiter and rt.channels is parts.channels and rt.link is parts.link
    assert rt.store is parts.store and rt.server is parts.server and rt.session_epoch == 9
    assert rt.arbiter.mission is core.commands and rt.arbiter.mode == "STOPPED"  # the STOP lives on
    assert rt.welcome is parts.welcome and rt.welcome["capabilities"]["mission"] is True
    assert rt.welcome["capabilities"]["relocalize"] is True  # still said: a client's capability check stays true


def test_an_arbiter_config_that_differs_from_the_runtimes_is_refused(tmp_path):
    parts = _parts(tmp_path, ArbiterConfig(tick_s=DT, lease_timeout_s=3.0))
    with pytest.raises(ValueError, match="config"):
        runtime(tmp_path, parts=parts, config=ArbiterConfig(tick_s=DT))


def test_without_parts_the_runtime_is_built_as_before(tmp_path):
    rt, core, _, _ = runtime(tmp_path)
    assert rt.parts is None and rt.arbiter.mission is core.commands and "relocalize" not in rt.welcome["capabilities"]


def test_what_the_runner_adds_to_the_port_file_survives_start_and_close(tmp_path):
    rt, _, _, _ = runtime(tmp_path)
    rt.port_extra["localized_wall"] = 123.5
    rt.start()
    assert json.loads((tmp_path / "port.json").read_text())["localized_wall"] == 123.5
    assert json.loads((tmp_path / "port.json").read_text())["state"] == "running"
    rt.close()
    info = json.loads((tmp_path / "port.json").read_text())
    assert info["state"] == "ended" and info["localized_wall"] == 123.5  # Codex B-P2c design P2


def test_an_adopted_server_is_not_started_again_and_its_clients_reconnect(tmp_path):
    parts = _parts(tmp_path)
    port = parts.server.start()
    asked = []
    parts.server.reconnect_clients = lambda: asked.append(True)
    rt, _, _, _ = runtime(tmp_path, parts=parts, config=ArbiterConfig(tick_s=DT))
    rt.arbiter.reloc = object()
    assert rt.start() == port and asked == [True]  # the final welcome is read on the clients' next hello
    assert rt.arbiter.reloc is None  # localized: the mission opens
    assert json.loads((tmp_path / "port.json").read_text())["state"] == "running"
