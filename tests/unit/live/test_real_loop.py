"""docs/design/2026-11-19-real-loop.md (4c): the real loop on a fake truck (the real host and port, a fake MCU)."""

from forklift_core.live import real_autonomy as ra
from forklift_core.mcu_link.messages import Arm, Drive

from real_loop_rig import Rig


def test_a_submit_is_prepared_adopted_armed_once_and_driven_only_after_ready():
    r = Rig()
    r.start()
    r.submit()
    sent_before = len(r.transport.sent)
    for _ in range(80):
        r.tick()
        if r.permit.state == ra.DRIVING:
            break
    assert r.core.commands.status == "running" and r.core.ms.phase == "approach", r.loop.records[-1]
    sent = r.transport.sent[sent_before:]
    arms = [i for i, (_, k, _, m) in enumerate(sent) if isinstance(m, Arm)]
    assert len(arms) == 1
    first_nonzero = next((i for i, (_, k, _, m) in enumerate(sent)
                          if isinstance(m, Drive) and (m.left_wheel_rad_s or m.right_wheel_rad_s)), None)
    assert first_nonzero is not None and first_nonzero > arms[0]
    assert r.permit.state == ra.DRIVING


def test_the_executors_host_is_the_real_client_with_a_random_first_seq(monkeypatch):
    from forklift_core.live import real_loop
    from forklift_core.mcu_link.build_policy import Client
    from forklift_core.plant.mcu_host import HostConfig

    cfg = HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100, outbox_max=16)
    monkeypatch.setattr(real_loop.secrets, "randbelow", lambda n: 0xBEEF if n == 0x10000 else 0)
    h = real_loop.real_host(cfg)
    assert h.client == Client.REAL and h._seq == 0xBEEF  # noqa: SLF001
    assert real_loop.real_host(cfg, initial_seq=7)._seq == 7  # noqa: SLF001
