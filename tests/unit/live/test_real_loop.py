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
