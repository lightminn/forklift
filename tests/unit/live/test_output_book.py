"""docs/design/2026-11-19-real-loop.md 2.6: the confirmed target, what may still act per direction, and the core's
view written from them only."""

from types import SimpleNamespace

from forklift_core.live.output_book import Frame, OutputBook
from forklift_core.mcu_link.messages import Mode, StateFlag

MS = 1_000_000


def frame(seq, speed, *, expires=100, movable=True, kind="drive"):
    return Frame(seq, 5, 2, expires * MS, speed, 0.0, (0.01,), 0.0, movable, kind, queued_ns=0)


def state(seq, *, fresh=True, boot=5, gen=2, mode=Mode.ARMED):
    return SimpleNamespace(last_rx_seq=seq, flags=StateFlag.COMMAND_FRESH if fresh else StateFlag(0), boot_id=boot,
                           arm_gen=gen, mode=mode)


def test_a_frame_is_confirmed_only_when_written_before_the_state_of_its_boot_and_generation_names_it():
    b = OutputBook()
    b.queued(frame(10, 0.1))
    b.on_state(state(10), 5 * MS)
    assert b.confirmed is None  # not written yet
    b.written(10, 6 * MS)
    b.on_state(state(10, gen=3), 7 * MS)
    assert b.confirmed is None  # another arm generation
    b.on_state(state(10, fresh=False), 7 * MS)
    assert b.confirmed is None  # its command not fresh
    b.on_state(state(10), 8 * MS)
    assert b.confirmed.seq == 10 and b.frames == []


def test_unconfirmed_movable_frames_count_per_direction_until_they_expire_or_a_later_one_is_confirmed():
    b = OutputBook()
    b.queued(frame(1, 0.15))  # forward, in the queue
    b.queued(frame(2, -0.05))  # a reverse request after it
    assert b.may_act(10 * MS) == {1: 0.15, -1: 0.05}
    b.written(1, 11 * MS)
    b.written(2, 12 * MS)
    b.on_state(state(2), 13 * MS)  # the reverse confirmed: the forward one is replaced
    assert b.may_act(14 * MS) == {-1: 0.05}
    b.settle(200 * MS)  # past every expiry
    assert b.may_act(200 * MS) == {} and b.confirmed is None


def test_a_frame_never_written_and_discarded_cannot_act():
    b = OutputBook()
    b.queued(frame(3, 0.2))
    b.discarded(3)
    assert b.may_act(1 * MS) == {} and b.dropped[0].seq == 3


def test_publish_writes_the_confirmed_target_and_the_residuals_never_the_candidate():
    b = OutputBook()
    st, slam = {}, {}
    b.publish(st, slam, 1 * MS, (0.02,))
    assert st["applied_command"] == (0.0, [0.02]) and st["command_speed"] == 0.0 and st["outstanding_motion"] == {}
    b.queued(frame(4, 0.1))
    b.publish(st, slam, 1 * MS, (0.02,))
    assert st["command_speed"] == 0.0 and st["outstanding_motion"] == {1: 0.1} and slam["applied_speed"] == 0.0
    b.written(4, 2 * MS)
    b.on_state(state(4), 3 * MS)
    b.publish(st, slam, 3 * MS, (0.02,))
    assert st["applied_command"] == (0.1, [0.01]) and st["outstanding_motion"] == {1: 0.1}


def test_a_state_not_armed_leaves_nothing_confirmed_or_able_to_act():
    """Codex impl P2-2: once the MCU reports DISARMED the old target is not published as applied."""
    b = OutputBook()
    b.queued(frame(1, 0.1))
    b.written(1, 1 * MS)
    b.on_state(state(1), 2 * MS)
    b.queued(frame(2, 0.1))
    assert b.confirmed is not None and b.may_act(3 * MS)
    b.on_state(state(1, fresh=False, mode=Mode.DISARMED), 4 * MS)
    assert b.confirmed is None and b.may_act(5 * MS) == {}
    st = {}
    b.publish(st, None, 5 * MS, (0.02,))
    assert st["applied_command"] == (0.0, [0.02]) and st["outstanding_motion"] == {}
