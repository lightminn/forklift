import pytest

from console.gateway.__main__ import parse_args
from console.gateway.convert import STOP_INSTRUCTIONS


def test_isaac_defaults_to_stopping_the_simulation():
    args, ros = parse_args(
        ["--backend", "isaac", "--ros-args", "-p", "use_sim_time:=true"]
    )
    assert args.stop_instruction == STOP_INSTRUCTIONS["isaac"]
    assert ros == ["--ros-args", "-p", "use_sim_time:=true"]


def test_the_real_truck_defaults_to_its_main_power_switch():
    # user, 2026-10-15: the real truck is stopped by switching off its main power
    args, _ = parse_args(["--backend", "real"])
    assert args.stop_instruction == "주 전원 스위치를 끄세요"
    args, _ = parse_args(
        ["--backend", "real", "--stop-instruction", "배터리 커넥터를 뽑으세요"]
    )
    assert args.stop_instruction == "배터리 커넥터를 뽑으세요"


def test_a_blank_stop_instruction_is_refused(capsys):
    with pytest.raises(SystemExit):
        parse_args(["--backend", "real", "--stop-instruction", "  "])
    assert "--stop-instruction" in capsys.readouterr().err
